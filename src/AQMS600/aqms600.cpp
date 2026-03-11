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
        vector<float> nox_values;

        send_read(this->_serial, AQMS600_NOx_concentration_params);
        data = receivePackage(AQMS600_NOx_concentration_params);
        
        Serial.println("---------");
        Serial.printf("Received %d bytes\n", data.data_payload.size());

        updateParams(data.data_payload);

        // for (size_t i = 0; i < data.data_payload.size(); i++)
        // {
        //     Serial.printf("Data Payload[%d]: %2x \n", i, data.data_payload[i]);
        // }
        // memcpy(&nox_params.no_concentration, &data.data_payload[0], sizeof(float));
        // memcpy(&nox_params.no2_concentration, (data.data_payload.data() + 4), sizeof(float));
        // nox_params.no2_concentration = ((uint32_t)data.data_payload[0]<<24) | 
        //                                ((uint32_t)data.data_payload[1]<<16) | 
        //                                ((uint32_t)data.data_payload[2]<<8) | 
        //                                (uint32_t)data.data_payload[3];

        // Serial.printf("NO Concentration: %.2f\n", nox_params.no_concentration);
        // Serial.printf("NO2 Concentration: %.2f\n", nox_params.no2_concentration);
        // Serial.printf("NOx Concentration: %.2f\n", nox_params.nox_concentration);
        // Serial.printf("Span Gas Flow: %.2f\n", nox_params.span_gas_flow);
        // Serial.printf("Unit: %d\n", nox_params.unit);
        Serial.println("---------");
    }
}

void AQMS600_NOx_Analyzer::send_read(Stream *serial, uint8_t cmd) {
    uint8_t header[] = {0x7D, 0x7B, 0x01, 0x10, 0x01, 0xF2};
    uint8_t cmd_code[] = {cmd, SEND_READ};
    uint8_t datalen[] = {0x00, 0x00};
    uint8_t crc16[] = {0x00, 0x00}; // Placeholder for CRC16 bytes
    uint8_t end[] = {0x7D, 0x7D};

    // Create payload by concatenating header, cmd_type, and datalen
    vector<uint8_t> payload;
    payload.insert(payload.end(), std::begin(header), std::end(header));
    payload.insert(payload.end(), std::begin(cmd_code), std::end(cmd_code));
    payload.insert(payload.end(), std::begin(datalen), std::end(datalen));

    // Calculate CRC16 over the relevant part of the payload
    uint16_t cal_crc = crc16_modbus(&payload[2], payload.size()-2); 
    // CRC byte: low byte first, then high byte
    crc16[0] = (uint8_t)(cal_crc & 0xFF);        // Low byte
    crc16[1] = (uint8_t)((cal_crc >> 8) & 0xFF); // High byte

    // Concatenate payload with CRC16 and end bytes
    payload.insert(payload.end(), std::begin(crc16), std::end(crc16));
    payload.insert(payload.end(), std::begin(end), std::end(end));

    if (this->_serial) {
        _serial->write(payload.data(), payload.size());     // Send the payload to the serial port
        // Serial.printf("Sent %d bytes\n", payload.size());
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

    // Optimization 1: Pre-reserve vector memory to prevent multiple heap reallocations
    // Based on your AQMS-600 data, 64 bytes is a safe initial guess.
    incoming_packet.data_payload.reserve(64); 

    // Optimization 2: Use a more robust loop for Serial timing
    while (millis() - startTime < TIMEOUT_MS) {
        if (!_serial->available()) {
            delayMicroseconds(100); // Small breath for UART buffer to fill
            continue;
        }

        uint8_t byteReceived = _serial->read();
        // Serial.printf("Stage %d - Byte[%d] received: %02x\n", recv_state, packageIndex, byteReceived);
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
                incoming_packet.data_payload.push_back(byteReceived);
                if (incoming_packet.data_payload.size() >= datain_size) {
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

void AQMS600_NOx_Analyzer::updateParams(const vector<uint8_t>& payload) {
    
    memset((void*)&nox_params, 0, sizeof(nox_params));
    nox_params.span_gas_flow = payload.size();
    if (payload.size() < sizeof(nox_params)){ return; }

    // 1. Bulk copy
    memcpy((void*)&nox_params, payload.data(), sizeof(nox_params));

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
}

bool AQMS600_NOx_Analyzer::verifyCRC(const fpi_protocol_packet& pkg) {
    uint8_t buf[256]; 
    uint16_t len = 0;

    // Based on your logic (index 2-5 of header, cmd, length, payload)
    memcpy(&buf[len], &pkg.header[2], 4); len += 4;
    memcpy(&buf[len], pkg.cmd_code, 2);    len += 2;
    memcpy(&buf[len], pkg.data_length, 2); len += 2;
    memcpy(&buf[len], pkg.data_payload.data(), pkg.data_payload.size()); len += pkg.data_payload.size();

    uint16_t calculated = crc16_modbus(buf, len);
    uint16_t received = (pkg.crc16[1] << 8) | pkg.crc16[0]; // Modbus usually Low Byte first
    // Serial.printf("CRC Expected: %04x, Got: %04x\n", calculated, received);
    return (calculated == received);
}