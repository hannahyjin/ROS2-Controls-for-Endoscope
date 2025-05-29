#pragma once

#ifndef TCA9548A_H
#define TCA9548A_H

#include <cstdint>

class TCA9548A {
    public:
        TCA9548A();
        ~TCA9548A();

        int init(int bus_id, uint8_t address=0x70);
        void set_channel(uint8_t channel);
        void no_channel();
        int get_fd() const;
    private:
        int fd;
        int bus;
        uint8_t addr;
};

#endif //TCA9548_H
