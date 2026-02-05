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

float vectorToFloat(const std::vector<uint8_t>& data, size_t startIndex) ;


float AQMS600_NOx_Analyzer::get_no2(void) { return nox_params.no2_concentration; }
float AQMS600_NOx_Analyzer::get_nox_span(void) { return nox_params.nox_span_range; }
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
        
        Serial.printf("Received %d bytes\n", data.data_payload.size());

        nox_params.no_concentration = vectorToFloat(data.data_payload, 0);
        nox_params.no2_concentration = vectorToFloat(data.data_payload, 4);
        nox_params.nox_concentration = vectorToFloat(data.data_payload, 8);
        // nox_params.unit = data.data_payload[12];

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

        Serial.printf("NO2 Concentration: %.2f\n", nox_params.no2_concentration);
        Serial.printf("NOx Span Range: %.2f\n", nox_params.nox_span_range);
        Serial.printf("Unit: %d\n", nox_params.unit);
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
    uint8_t packageIndex = 0; // Index to track the current byte being verified
    uint8_t recv_state = 0; // State variable to track the current position in the packet
    uint8_t datain_size = 0;

    fpi_protocol_packet ref_read_packet = {
        .header = {0x7D, 0x7B, 0x01, 0xF2, 0x01, 0x10},
        .cmd_code = {cmd_type, RECV_READ},
        .end = {0x7D, 0x7D}
    };

    fpi_protocol_packet incoming_packet;

    while (_serial->available()) {
        uint8_t byteReceived = _serial->read(); // Read one byte from the serial port
        // Serial.printf("Stage %d - Byte[%d] received: %02x\n", recv_state, packageIndex, byteReceived);
        switch (recv_state) {
        case 0: // Verify header
            if (byteReceived == ref_read_packet.header[packageIndex]) {
                incoming_packet.header[packageIndex] = byteReceived;
                packageIndex++;
                if (packageIndex == sizeof(ref_read_packet.header)) {
                    recv_state = 1; // Move to the next state
                    packageIndex = 0; // Reset index for next section
                }
            } else {
                packageIndex = 0; // Reset if mismatch
            }
            break;

        case 1: // Verify command code
            if (byteReceived == ref_read_packet.cmd_code[packageIndex]) {
                incoming_packet.cmd_code[packageIndex] = byteReceived;
                packageIndex++;
                if (packageIndex == sizeof(ref_read_packet.cmd_code)) {
                    recv_state = 2; // Move to the next state
                    packageIndex = 0; // Reset index for next section
                }
            } else {
                recv_state = 0; // Reset state if mismatch
                packageIndex = 0;
            }
            break;

        case 2: // Verify data length
            incoming_packet.data_length[packageIndex] = byteReceived;
            packageIndex++;
            if (packageIndex == sizeof(ref_read_packet.data_length)) {
                recv_state = 3; // Move to the next state
                packageIndex = 0; // Reset index for next section
                // Serial.printf("Data payload received with %d bytes\n", incoming_packet.data_length[0] << 8 | incoming_packet.data_length[1]);
            }
            break;

        case 3: // Receive data payload
            incoming_packet.data_payload.push_back(byteReceived);
            packageIndex++;
            datain_size = incoming_packet.data_length[0] << 8 | incoming_packet.data_length[1];
            if (incoming_packet.data_payload.size() >= datain_size) {
                recv_state = 4; // Move to the next state
                packageIndex = 0; // Reset index for next section
            }
            break;

        case 4: // Verify CRC16
            incoming_packet.crc16[packageIndex] = byteReceived;
            packageIndex++;
            if (packageIndex == sizeof(ref_read_packet.crc16)) {
                recv_state = 5; // Move to the next state
                packageIndex = 0; // Reset index for next section
            }
            break;

        case 5: // Verify end bytes
            if (byteReceived == ref_read_packet.end[packageIndex]) {
                incoming_packet.end[packageIndex] = byteReceived;
                packageIndex++;
                if (packageIndex == sizeof(ref_read_packet.end)) {                    
                    break; // Packet received successfully
                }
            } else {
                recv_state = 0; // Reset state if mismatch
                packageIndex = 0;
            }
            break;

        default:
            recv_state = 0; // Reset state in case of unexpected behavior
            packageIndex = 0;
            break;
        }

        if(incoming_packet.end[1] == 0x7D) {
            Serial.println("Packet received successfully.");
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

float vectorToFloat(const vector<uint8_t>& data, size_t startIndex) {
    if (data.size() < startIndex + 4) return 0.0f;

    uint8_t temp[4] = {
        data[startIndex + 0], // Swap byte 3 to position 0
        data[startIndex + 1], // Swap byte 2 to position 1
        data[startIndex + 2],
        data[startIndex + 3]
    };

    float result;
    memcpy(&result, temp, 4);
    // Serial.printf("Converted float value: %.8f\n\n", result);
    return result;
}