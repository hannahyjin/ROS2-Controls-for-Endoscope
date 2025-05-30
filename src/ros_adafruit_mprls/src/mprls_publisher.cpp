#include "rclcpp/rclcpp.hpp"

#include "ros_adafruit_mprls/msg/mprls_pressures.hpp"
#include "ros_adafruit_mprls/Adafruit_MPRLS.hpp"

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class MprlsNode : public rclcpp::Node
{
    public:
        MprlsNode()
            : Node("mprls_node"),
    {
        mux_client_ = this->create_client<example_interfaces::srv::SetInt32>("select_mux_channel");

        if (!mux_client_->wait_for_service(5s)) {
            RCLCPP_FATAL(this->get_logger(), "Mux service not available");
            throw std::runtime_error("Mux Service not found");
        }

        sensor1_ = std::make_unique<MPRLS>(1);
        sensor2_ = std::make_unique<MPRLS>(1);

        if (!select_channel(0) || !sensor1_->begin()) {
            RCLCPP_ERROR(this->get_logger(), "Sensor1 MPRLS init failed");
            throw std::runtime_error("Sensor1 initialization failed");
        }

        if(!select_channel(1) || !sensor2_->begin()) {
            RCLCPP_ERROR(this->get_logger(), "Sensor2 MPRLS init failed");
            throw std::runtime_error("Sensor2 initialization failed");
        }

        pub_ = this->create_publisher<ros_adafruit_mprls::msg::MPRLSPressures>("mprls_pressures", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&MprlsNode::timer_callback, this));
    }

    private:
        bool select_channel(int32_t channel) {
            auto req = std::make_shared<example_interfaces::srv::SetInt32::Request>();
            req->data = channel;

            auto result = mux_client_->async_send_request(req);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, 50ms) != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to call mux service for channel %d", channel);
                return false;
            }

            if (!result.get()->success) {
                RCLCPP_ERROR(this->get_logger(), "Mux service rejected channel %d: %s", channel, result.get()->message.c_str());
                return false;
            }

            return true;
        }
        void timer_callback() {
            if (!select_channel(0)) return;
            float pressure1 = sensor1_->readPressure();

            if (!select_channel(1)) return;
            float pressure2 = sensor2_->readPressure();

            auto message = ros_adafruit_mprls::msg::MPRLSPressures();

            message.pressure_sensor_1 = pressure1;
            message.pressure_sensor_2 = pressure2;

            pub_->publish(message);

            RCLCPP_INFO(this->get_logger(), "Published pressures: Sensor1=%.2f hPa, Sensor2=%.2f hPa", pressure1, pressure2);
        }

        std::unique_ptr<MPRLS> sensor1_;
        std::unique_ptr<MPRLS> sensor2_;

        rclcpp::Publisher<ros_adafruit_mprls::msg::MPRLSPressures>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::shared_ptr<rclcpp::Client<example_interfaces::srv::SetInt32>> mux_client_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MprlsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
