#pragma once

#include<mutex>
#include "TCA9548A.hpp"

class Mux_Manager {
    public:
        Mux_Manager(int bus = 1);

        void selectChannel(uint8_t channel);
        void disableAll();

    private:
        std::mutex mux_mutex_;
        TCA9548A mux_;
};
