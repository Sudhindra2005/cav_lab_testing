#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

#define RPI_IP   "192.168.0.64"
#define RPI_PORT 8888

class UdpJoySender : public rclcpp::Node {
public:
    UdpJoySender() : Node("udp_joy_sender"), packet_count_(0) {
        // Setup UDP socket
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to create UDP socket");
            rclcpp::shutdown();
            return;
        }

        memset(&rpi_addr_, 0, sizeof(rpi_addr_));
        rpi_addr_.sin_family = AF_INET;
        rpi_addr_.sin_port = htons(RPI_PORT);
        inet_pton(AF_INET, RPI_IP, &rpi_addr_.sin_addr);

        // Subscribe to joystick topic
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&UdpJoySender::joyCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "UDP joystick sender node started");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() < 2) {
            RCLCPP_WARN(get_logger(), "Not enough axes in Joy message");
            return;
        }

        // Extract joystick axes:
        float motor = msg->axes[1];  // vertical axis
        float servo = msg->axes[0];  // horizontal axis

        // Clamp
        motor = std::max(-1.0f, std::min(1.0f, motor));
        servo = std::max(-1.0f, std::min(1.0f, servo));

        // Packet float[2]
        float packet[2];
        packet[0] = motor;
        packet[1] = servo;

        // Send UDP packet
        int sent = sendto(
            sockfd_,
            packet, sizeof(packet),
            0,
            (sockaddr*)&rpi_addr_, sizeof(rpi_addr_)
        );

        // -----------------------------
        // PRINT STATIC STATUS ON SCREEN
        // -----------------------------
        packet_count_++;


    // STATIC TERMINAL UI
    printf("\033[2J");      // Clear screen
    printf("\033[H");       // Move cursor to top-left
    
    printf("=== Joystick → UDP Sender ===\n\n");
    printf("Motor axis (vertical): %.3f\n", motor);
    printf("Servo axis  (horiz) : %.3f\n", servo);

    printf("\nPacket sent:\n");
    printf("motor = %.3f\n", packet[0]);
    printf("servo = %.3f\n", packet[1]);

    fflush(stdout);         // Force update
    
        if (sent != sizeof(packet)) {
            RCLCPP_WARN(get_logger(), "UDP send failed");
        }
    }

    int sockfd_;
    sockaddr_in rpi_addr_;
    uint32_t packet_count_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpJoySender>());
    rclcpp::shutdown();
    return 0;
}

