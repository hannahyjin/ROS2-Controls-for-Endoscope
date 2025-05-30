#include "rclcpp/rclcpp.hpp"

#include "mux_manager/TCA9548A.hpp"

#include <memory>
#include <mutex>

#include "mux_manager/srv/set_int32.hpp"

class MuxServiceNode : public rclcpp::Node {
public:
    MuxServiceNode()
        : Node("mux_service_node"), current_channel_(255)
    {
        this->declare_parameter<std::string>("device", "/dev/i2c-1");
        std::string i2c_dev;
        this->get_parameter("device", i2c_dev);

        tca_ = std::make_shared<TCA9548A>();

        if (tca_->init(1, 0x70) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to init TCA device");
        }

        srv_ = this->create_service<mux_manager::srv::SetInt32>(
            "select_mux_channel",
            std::bind(&MuxServiceNode::onRequest, this,
                        std::placeholders::_1, std::placeholders::_2));
    }

private:
    std::shared_ptr<TCA9548A> tca_;
    std::mutex mutex_;
    uint8_t current_channel_;

    rclcpp::Service<mux_manager::srv::SetInt32>::SharedPtr srv_;

    void onRequest(const std::shared_ptr<mux_manager::srv::SetInt32::Request> req,
            std::shared_ptr<mux_manager::srv::SetInt32::Response> res) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (req->data < 0 || req->data > 7) {
           res->success = false;
           res->message = "Invalid mux channel";
           return;
        }

        if (current_channel_ != static_cast<uint8_t>(req->data)) {
            bool success = tca_->set_channel(static_cast<uint8_t>(req->data));
            if (!success) {
                res->success = false;
                res->message = "Failed to select channel";
                return;
            }

            current_channel_ = static_cast<uint8_t>(req->data);
        }

        res->success = true;
        res->message = "Channel selected successfully";
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MuxServiceNode>());
    rclcpp::shutdown();
    return 0;
}
