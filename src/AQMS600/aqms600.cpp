#include "aqms600.h"
#include "../Main/BoardDef.h"
#include "Arduino.h"

/**
 * @brief Initializes the sensor and attempts to read data.
 *
 * @param stream UART stream
 * @return true Sucecss
 * @return false Failure
 */

#define AQMS600_NOx_concentration_params 0x3f
#define AQMS600_baudrate 57600
#define SEND_READ   0x55
#define SEND_WRITE  0x66
#define RECV_READ   0xAA
#define RECV_WRITE  0x99

float AQMS600_NOx_Analyzer::get_no(void) { return nox_params.no_concentration; }
float AQMS600_NOx_Analyzer::get_no2(void) { return nox_params.no2_concentration; }
float AQMS600_NOx_Analyzer::get_nox(void) { return nox_params.nox_concentration; }
float AQMS600_NOx_Analyzer::get_gas_span(void) { return nox_params.span_gas_flow; }
uint8_t AQMS600_NOx_Analyzer::get_nox_unit(void) { return nox_params.unit; }

AQMS600_NOx_Analyzer::AQMS600_NOx_Analyzer(/* args */)
{
    nox_params.no2_concentration = 0.0;
    nox_params.nox_span_range = 0.0;
    nox_params.unit = 0;
}

/**
 * Initializes the sensor.
 */
bool AQMS600_NOx_Analyzer::begin(HardwareSerial &serial) {
    this->_serial = &serial;
    this->_serial->begin(AQMS600_baudrate);
    delay(100);    
    Serial.println("AQMS600 sensor initialized.");
    return true;
}

void AQMS600_NOx_Analyzer::read_NOx_concentration(void) {
    if (this->_serial)
    { 
        fpi_protocol_packet data;

        send_read(this->_serial, AQMS600_NOx_concentration_params);
        data = receivePackage(AQMS600_NOx_concentration_params);
        
        // Serial.println("---------");
        // Serial.printf("Received %d bytes\n", data.data_payload_len);

        if(updateParams(data.data_payload, data.data_payload_len))
        {
            // Serial.printf("NO Concentration: %.2f\n", nox_params.no_concentration);
            // Serial.printf("NO2 Concentration: %.2f\n", nox_params.no2_concentration);
            // Serial.printf("NOx Concentration: %.2f\n", nox_params.nox_concentration);
            // Serial.printf("Span Gas Flow: %.2f\n", nox_params.span_gas_flow);
            // Serial.printf("Unit: %d\n", nox_params.unit);
            // Serial.println("NOx concentration parameters updated successfully.");
            Serial.printf("Sucessfully Read!!");
        } else {
            Serial.println("Failed to update NOx concentration parameters: Invalid payload size.");
        }

        // Serial.println("---------");
    }
}

void AQMS600_NOx_Analyzer::send_read(Stream *serial, uint8_t cmd) {
    uint8_t header[] = {0x7D, 0x7B, 0x01, 0x10, 0x01, 0xF2};
    uint8_t cmd_code[] = {cmd, SEND_READ};
    uint8_t datalen[] = {0x00, 0x00};
    uint8_t crc16[2];
    uint8_t end[] = {0x7D, 0x7D};

    uint8_t payload[16];
    size_t idx = 0;
    memcpy(payload + idx, header, sizeof(header)); idx += sizeof(header);
    memcpy(payload + idx, cmd_code, sizeof(cmd_code)); idx += sizeof(cmd_code);
    memcpy(payload + idx, datalen, sizeof(datalen)); idx += sizeof(datalen);

    uint16_t cal_crc = crc16_modbus(payload + 2, idx - 2);
    payload[idx++] = (uint8_t)(cal_crc & 0xFF);
    payload[idx++] = (uint8_t)((cal_crc >> 8) & 0xFF);
    memcpy(payload + idx, end, sizeof(end)); idx += sizeof(end);

    if (this->_serial) {
        _serial->write(payload, idx);     // Send the payload to the serial port
        // Serial.printf("Sent %d bytes\n", idx);
    } else {
        Serial.println("Error: Serial port not initialized.");
    }
}

auto AQMS600_NOx_Analyzer::receivePackage(uint8_t cmd_type) -> fpi_protocol_packet {
    uint8_t packageIndex = 0;   // Index to track the current byte being verified
    uint8_t recv_state = 0;     // State variable to track the current position in the packet
    uint16_t datain_size = 0;
    uint32_t startTime = millis();
    const uint32_t TIMEOUT_MS = 1000; // Serial safety timeout

    fpi_protocol_packet incoming_packet;
    incoming_packet.data_payload_len = 0;

    // Use a more robust loop for Serial timing
    while (millis() - startTime < TIMEOUT_MS) {
        if (!_serial->available()) {
            delayMicroseconds(100); // Small breath for UART buffer to fill
            continue;
        }

        uint8_t byteReceived = _serial->read();
        Serial.printf("Stage %d - Byte[%d] received: %02x\n", recv_state, packageIndex, byteReceived);
        startTime = millis(); // Reset timeout on every byte received

        switch (recv_state) {
            case 0: // Header (0x7D, 0x7B, 0x01, 0xF2, 0x01, 0x10)
                if (packageIndex >= 0) {
                    // We only care about matching your ref_read_packet logic
                    const uint8_t ref_header[] = {0x7D, 0x7B, 0x01, 0xF2, 0x01, 0x10};
                    if (byteReceived == ref_header[packageIndex]) {
                        incoming_packet.header[packageIndex++] = byteReceived;
                        if (packageIndex == 6) { recv_state = 1; packageIndex = 0; }
                    } else { packageIndex = 0; }
                }
                break;

            case 1: // Cmd Code
                incoming_packet.cmd_code[packageIndex++] = byteReceived;
                if (packageIndex == 2) {
                    const uint8_t ref_cmd[] = {cmd_type, RECV_READ};
                    if (incoming_packet.cmd_code[0] == ref_cmd[0]
                    && incoming_packet.cmd_code[1] == ref_cmd[1]) {
                        recv_state = 2; // Move to the next state
                        packageIndex = 0; // Reset index for next section

                    } else {
                        recv_state = 0; // Reset state if mismatch
                        packageIndex = 0;
                    }
                }
                break;

            case 2: // Length
                incoming_packet.data_length[packageIndex++] = byteReceived;
                if (packageIndex == 2) {
                    datain_size = (incoming_packet.data_length[0] << 8) | incoming_packet.data_length[1];
                    // Sanity check for length to prevent memory exhaustion
                    if (datain_size > 256) { recv_state = 0; packageIndex = 0; }
                    else { recv_state = 3; packageIndex = 0; }
                }
                break;

            case 3: // Payload
                if (incoming_packet.data_payload_len < AQMS600_MAX_PAYLOAD_SIZE) {
                    incoming_packet.data_payload[incoming_packet.data_payload_len++] = byteReceived;
                }
                if (incoming_packet.data_payload_len >= datain_size) {
                    recv_state = 4;
                    packageIndex = 0;
                }
                break;

            case 4: // CRC16 - Direct Calculation
                incoming_packet.crc16[packageIndex++] = byteReceived;
                if (packageIndex == 2) {
                    // Optimization 3: Do NOT create a checkBuffer vector.
                    // Instead, calculate CRC in a way that respects the protocol's range.
                    if (verifyCRC(incoming_packet)) {
                        recv_state = 5;
                        packageIndex = 0;
                    } else {
                        memset(&incoming_packet, 0, sizeof(incoming_packet));
                        return incoming_packet;
                    }
                }
                break;

            case 5: // End Bytes (0x7D, 0x7D)
                incoming_packet.end[packageIndex++] = byteReceived;
                if (packageIndex == 2) return incoming_packet; // Success!
                break;

            default:
                recv_state = 0;
                packageIndex = 0;
                break;
        }
    }
    return incoming_packet;
}

uint16_t AQMS600_NOx_Analyzer::crc16_modbus(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF; // Initial value

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i]; // XOR byte into least significant byte of crc

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001; // Polynomial 0xA001
            } else {
                crc >>= 1;
            }
        }
    }

    // // Swap bytes: low byte first, then high byte
    // crc = (crc >> 8) | (crc << 8);

    return crc;
}

bool AQMS600_NOx_Analyzer::updateParams(const uint8_t *payload, size_t size) {
    
    memset((void*)&nox_params, 0, sizeof(nox_params));
    if (size < sizeof(nox_params)) { return false; }

    // 1. Bulk copy
    memcpy((void*)&nox_params, payload, sizeof(nox_params));

    // 2. Fast Byte-Swapping using ESP32-C3 (RISC-V) Intrinsics
    // We create a tiny lambda helper for readability
    auto swap = [](float* f) {
        uint32_t* p = reinterpret_cast<uint32_t*>(f);
        *p = __builtin_bswap32(*p);
    };

    // // Swap all floats (Big-Endian sensor -> Little-Endian ESP32)
    swap((float*)&nox_params.no_concentration);
    swap((float*)&nox_params.no2_concentration);
    swap((float*)&nox_params.nox_concentration);
    
    // // Note: nox_params.unit (at index 12) is 1 byte, so NO SWAP needed.
    
    swap((float*)&nox_params.span_gas_flow);
    return true;
}

bool AQMS600_NOx_Analyzer::verifyCRC(const fpi_protocol_packet& pkg) {
    uint8_t buf[256]; 
    uint16_t len = 0;

    // Based on your logic (index 2-5 of header, cmd, length, payload)
    memcpy(&buf[len], &pkg.header[2], 4); len += 4;
    memcpy(&buf[len], pkg.cmd_code, 2);    len += 2;
    memcpy(&buf[len], pkg.data_length, 2); len += 2;
    memcpy(&buf[len], pkg.data_payload, pkg.data_payload_len); len += pkg.data_payload_len;

    uint16_t calculated = crc16_modbus(buf, len);
    uint16_t received = (pkg.crc16[1] << 8) | pkg.crc16[0]; // Modbus usually Low Byte first
    // Serial.printf("CRC Expected: %04x, Got: %04x\n", calculated, received);
    return (calculated == received);
}