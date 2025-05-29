#include "mux_manager/TCA9548A.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdio>
#include <string>
#include <iostream>

TCA9548A::TCA9548A() : fd(-1), bus(1), addr(0x70) {}

TCA9548A::~TCA9548A() {
    if (fd >= 0) close(fd);
}

int TCA9548A::init(int bus_id, uint8_t address){
    bus = bus_id;
    addr = address;

    std::string dev_path = "/dev/i2c-" + std::to_string(bus);

    fd = open(dev_path.c_str(), O_RDWR);

    if (fd < 0) {
        perror(("failed t open i2c bus " + dev_path).c_str());
        return -1;
    }

    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        perror("failed to set address");
        close(fd);
        fd = -1;
        return -1;
    }

    return 0;
}

void TCA9548A::set_channel(uint8_t channel) {
    if (fd < 0) {
        std::cerr << "i2c device not initialized" << "\n";
        return;
    }

    if (channel > 7) {
        std::cerr << "invalid channel selected: " <<  static_cast<int>(channel) << "\n";
        return;
    }

    uint8_t data = 1 << channel;

    if (write(fd, &data, 1) != 1) {
        perror("failed to write channel byte");
    }
}

void TCA9548A::no_channel() {
    if (fd < 0) {
        std::cerr << "i2c device not initialized" << "\n";
        return;
    }

    uint8_t data = 0x00;
    if(write(fd, &data, 1) != 1) {
        perror("failed to disable channels");
    }
}

int TCA9548A::get_fd() const {
    return fd;
}
