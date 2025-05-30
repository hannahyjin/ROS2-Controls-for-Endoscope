#ifndef MPRLS_HPP
#define MPRLS_HPP

#include <cstdint>
#include <string>
#include <limits>

/*
 * CONSTANTS
 */

constexpr uint8_t MPRLS_DEFAULT_ADDR = 0x18;
constexpr uint8_t MPRLS_STATUS_POWERED = 0x40;
constexpr uint8_t MPRLS_STATUS_BUSY = 0x20;
constexpr uint8_t MPRLS_STATUS_FAILED = 0x04;
constexpr uint8_t MPRLS_STATUS_MATHSAT = 0x01;
constexpr uint8_t MPRLS_STATUS_MASK = 0b01100101;
constexpr uint32_t COUNTS_224 = 16777216UL;
constexpr uint32_t MPRLS_READ_TIMEOUT_MS = 20;
constexpr float PSI_TO_HPA = 68.947572932f;

/*
 * CLASS DECLARATION
 */

class MPRLS {
    public:
        MPRLS(int bus = 1,
              uint8_t address = MPRLS_DEFAULT_ADDR,
              uint16_t psi_min = 0,
              uint16_t psi_max = 25,
              float output_min_pct = 10.0f,
              float output_max_pct = 90.0f,
              float conversion_factor = PSI_TO_HPA);
        
        ~MPRLS();

        bool begin();
        
        float readPressure();
        
        uint32_t readRaw();
        
        uint8_t readStatus();

        uint8_t last_status = 0;
        
    private:
        int fd_;
        int bus_;
        uint8_t addr_;

        uint16_t psi_min_, psi_max_;
        uint32_t output_min_, output_max_;
        float conv_factor_;

        bool writeCommand();
        bool waitReady();
};

#endif //MPRLS_HPP
