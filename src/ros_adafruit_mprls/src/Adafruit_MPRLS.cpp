#include "ros_adafruit_mprls/Adafruit_MPRLS.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <chrono>

MPRLS::MPRLS(int bus, uint8_t address, uint16_t psi_min, uint16_t psi_max,
             float output_min_pct, float output_max_pct, float conversion_factor)
    : fd_(-1), bus_(bus), addr_(address), 
      psi_min_(psi_min), psi_max_(psi_max),
      conv_factor_(conversion_factor)
{
    output_min_ = static_cast<uint32_t>(COUNTS_224 * (output_min_pct / 100.0f) + 0.5f);
    output_max_ = static_cast<uint32_t>(COUNTS_224 * (output_max_pct / 100.0f) + 0.5f);
}

MPRLS::~MPRLS() {
    if (fd_ >= 0) close(fd_);
}

bool MPRLS::begin() {
    char filename[20];
    snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus_);

    fd_ = open(filename, O_RDWR);

    if(fd_ < 0) {
        perror("Failed to open i2c bus.");
        return false;
    }

    if (ioctl(fd_, I2C_SLAVE, addr_) < 0) {
        perror("Failed to set i2c slave address");
        close(fd_);
        fd_ = -1;
        return false;
    }

    last_status = readStatus();
    return (last_status & MPRLS_STATUS_MASK) == MPRLS_STATUS_POWERED;
}

bool MPRLS::writeCommand() {
    uint8_t cmd[3] = {0xAA, 0x00, 0x00};
    ssize_t written = write(fd_, cmd, 3);
    if(written != 3) {
        perror("Failed to write trigger");
        return false;
    }

    return true;
}

bool MPRLS::waitReady() {
    using namespace std::chrono;

    auto start = steady_clock::now();

    while(true) {
        last_status = readStatus();
        if (!(last_status & MPRLS_STATUS_BUSY))
            return true;
        
        auto now = steady_clock::now();
        if (duration_cast<milliseconds>(now - start).count() > MPRLS_READ_TIMEOUT_MS) {
            fprintf(stderr, "Timeout waiting for sensor ready\n");
            return false;
        }

        usleep(1000);
    }
}

float MPRLS::readPressure() {
    uint32_t raw = readRaw();
    if (raw == 0xFFFFFFFF || output_min_ == output_max_)
        return std::numeric_limits<float>::quiet_NaN();

    float psi = (static_cast<float>(raw) - output_min_) * (psi_max_ - psi_min_);
    psi /= static_cast<float>(output_max_ - output_min_);
    psi += psi_min_;

    return psi * conv_factor_;
}

uint32_t MPRLS::readRaw() {
    if (fd_ < 0) {
        fprintf(stderr, "Device not initialized\n");
        return 0xFFFFFFFF;
    }

    if (!writeCommand())
        return 0xFFFFFFFF;
    
    if (!waitReady())
        return 0xFFFFFFFF;

    uint8_t buffer[4] = {0};
    ssize_t read_bytes = read(fd_, buffer, 4);
    if (read_bytes != 4) {
        perror("Failed to read data");
        return 0xFFFFFFFF;
    }

    uint8_t status = buffer[0];
    if ((status & MPRLS_STATUS_MATHSAT) || (status & MPRLS_STATUS_FAILED)) {
        fprintf(stderr, "Sensor error status: 0x%02X\n", status);
        return 0xFFFFFFFF;
    }

    return (static_cast<uint32_t>(buffer[1]) << 16) |
           (static_cast<uint32_t>(buffer[2]) << 8) |
           static_cast<uint32_t>(buffer[3]);
}

uint8_t MPRLS::readStatus() {
    if (fd_ < 0) {
        fprintf(stderr, "Device not initialized\n");
        return 0;
    }

    uint8_t status = 0;
    ssize_t read_bytes = read(fd_, &status, 1);

    if(read_bytes != 1) {
        perror("Failed to read status");
        return 0;
    }

    return status;
}
