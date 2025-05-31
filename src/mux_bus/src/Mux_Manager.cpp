#include "mux_bus/mux_manager/Mux_Manager.hpp"

Mux_Manager::Mux_Manager(int bus)
    : mux_() 
{
    mux_.init(bus);
}

void Mux_Manager::selectChannel(uint8_t channel) {
    std::lock_guard<std::mutex> lock(mux_mutex_);
    mux_.set_channel(channel);
}

void Mux_Manager::disableAll() {
    std::lock_guard<std::mutex> lock(mux_mutex_);
    mux_.no_channel();
}
