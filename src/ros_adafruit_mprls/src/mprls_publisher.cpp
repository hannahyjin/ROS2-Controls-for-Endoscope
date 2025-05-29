#include "rclcpp/rclcpp.hpp"

#include "ros_adafruit_mprls/msg/mprls_pressures.hpp"
#include "mux_manager/include/mux_manager/TCA9548A.hpp"
#include "ros_adafruit_mprls/Adafruit_MPRLS.hpp"
#include "mux_manager/include/mux_manager/Mux_Manager.hpp"

#include <chrono>

using namespace std::chrono_literals;

class MprlsNode : public rclcpp::Node
{
    public:
        MprlsNode()
            : Node("mprls_node"),
            mux_(),
            sensor1_(),
            sensor2_(),
            mux_manager_()
    {
        if(mux_.init(1) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize TCA9548A mux");
            throw std::runtime_error("Mux init failed");
        }

        sensor1_ = std::make_unique<MPRLS>(1);
        sensor2_ = std::make_unique<MPRLS>(1);

        mux_manager_.selectChannel(0);
        if (!sensor1_->begin()) {
            RCLCPP_ERROR(this->get_logger(), "Sensor1 MPRLS init failed");
            throw std::runtime_error("Sensor1 initialization failed");
        }

        mux_manager_.selectChannel(1);
        if(!sensor2_->begin()) {
            RCLCPP_ERROR(this->get_logger(), "Sensor2 MPRLS init failed");
            throw std::runtime_error("Sensor2 initialization failed");
        }

        pub_ = this->create_publisher<ros_adafruit_mprls::msg::MPRLSPressures>("mprls_pressures", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&MprlsNode::timer_callback, this));
    }

    private:
        void timer_callback() {
            mux_manager_.selectChannel(0);
            float pressure1 = sensor1_->readPressure();

            mux_manager_.selectChannel(1);
            float pressure2 = sensor2_->readPressure();

            auto message = ros_adafruit_mprls::msg::MPRLSPressures();

            message.pressure_sensor_1 = pressure1;
            message.pressure_sensor_2 = pressure2;

            pub_->publish(message);

            RCLCPP_INFO(this->get_logger(), "Published pressures: Sensor1=%.2f hPa, Sensor2=%.2f hPa", pressure1, pressure2);
        }

        TCA9548A mux_;
        std::unique_ptr<MPRLS> sensor1_;
        std::unique_ptr<MPRLS> sensor2_;
        Mux_Manager mux_manager_;

        rclcpp::Publisher<ros_adafruit_mprls::msg::MPRLSPressures>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MprlsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
