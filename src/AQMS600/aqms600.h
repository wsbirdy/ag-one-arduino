#ifndef _FPI_AQMS600_H_
#define _FPI_AQMS600_H_

#include "../Main/BoardDef.h"
#include <vector> // Include the vector header for std::vector
#include <iterator> // Include iterator for std::begin and std::end

using namespace std; // Add this to avoid prefixing with std::

/** 
 * @brief The class define how to handle PMS5003 sensor bas on @ref PMS class
 */

class AQMS600_NOx_Analyzer
{
private:
    /* data */
    HardwareSerial* _serial;
    
    struct fpi_protocol_packet {
        uint8_t header[6];
        uint8_t cmd_code[2];
        uint8_t data_length[2];
        vector<uint8_t> data_payload;
        uint8_t crc16[2];
        uint8_t end[2];
    } packet;

    volatile struct __attribute__((packed)) nox_concentration_params
    {
        float no_concentration;
        float no2_concentration;
        float nox_concentration;
        uint8_t unit;
        float span_gas_flow;
        float sampling_pressure;
        float reaction_pressure;
        float temp;
        float no_conc_deviation;
        float no_span_range;
        float nox_span_range;

    }nox_params;

    bool active = true;
    uint16_t crc16_modbus(const uint8_t *data, size_t length);
    void updateParams(const vector<uint8_t>& payload);
    bool verifyCRC(const fpi_protocol_packet& pkg);

public:
    AQMS600_NOx_Analyzer(/* args */);
    ~AQMS600_NOx_Analyzer() {};
    bool begin(HardwareSerial &serial);
    void setActivation(bool state) { active = state; };
    bool hasActivated(void) { return active; };
    void read_NOx_concentration(void);
    void send_read(Stream *serial, uint8_t cmd);
    auto receivePackage(uint8_t cmd_type) -> fpi_protocol_packet;
    float get_no2(void);
    float get_no(void);
    float get_nox(void);
    float get_gas_span(void);
    uint8_t get_nox_unit(void);

};

#endif /** _FPI_AQMS600_H_ */