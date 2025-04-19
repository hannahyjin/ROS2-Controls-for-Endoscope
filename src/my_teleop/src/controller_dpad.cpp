#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DpadTeleop : public rclcpp::Node
{
public:
    DpadTeleop() : Node("dpad_teleop")
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&DpadTeleop::joy_callback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        geometry_msgs::msg::Twist twist;
        
        bool enable = msg->buttons[0];

        // D-pad up/down
        const bool dpad_up = msg->buttons[13];
        const bool dpad_down = msg->buttons[14];

        // Triggers — many controllers report these as axes, where:
        // LT = axis[5] — usually 1.0 when released, -1.0 when fully pressed
        // RT = axis[4] — same logic
        const bool lb = msg->buttons[4];
        const bool rb = msg->buttons[5];

        // Motion logic
        if(enable)
        {
        if (dpad_up)
            twist.linear.x = 2.0;
        else if (dpad_down)
            twist.linear.x = -2.0;

        if (lb)
            twist.angular.z = 2.0;
        else if (rb)
            twist.angular.z = -2.0;
        }

        cmd_pub_->publish(twist);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DpadTeleop>());
    rclcpp::shutdown();
    return 0;
}
