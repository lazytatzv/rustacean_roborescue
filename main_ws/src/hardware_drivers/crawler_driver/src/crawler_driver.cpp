#include <termios.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <custom_interfaces/msg/crawler_velocity.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>
#include <vector>

// ──────────────────────────────────────────────
// Roboclaw Constants
// ──────────────────────────────────────────────
constexpr uint8_t ADDR = 0x80;
constexpr uint8_t M1_SPEED_CMD = 35; 
constexpr uint8_t M2_SPEED_CMD = 36; 

class SimpleRoboclaw {
public:
    SimpleRoboclaw(const std::string& port, int baud) : io_(), serial_(io_) {
        serial_.open(port);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    }

    void sendCommand(uint8_t cmd, int32_t qpps) {
        std::vector<uint8_t> pkt;
        pkt.push_back(ADDR);
        pkt.push_back(cmd);
        pkt.push_back(static_cast<uint8_t>((qpps >> 24) & 0xFF));
        pkt.push_back(static_cast<uint8_t>((qpps >> 16) & 0xFF));
        pkt.push_back(static_cast<uint8_t>((qpps >> 8) & 0xFF));
        pkt.push_back(static_cast<uint8_t>(qpps & 0xFF));
        
        uint16_t crc = 0;
        for (auto b : pkt) {
            crc ^= (static_cast<uint16_t>(b) << 8);
            for (int j = 0; j < 8; j++) {
                crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
            }
        }
        pkt.push_back(static_cast<uint8_t>(crc >> 8));
        pkt.push_back(static_cast<uint8_t>(crc & 0xFF));

        try {
            boost::asio::write(serial_, boost::asio::buffer(pkt));
            uint8_t ack = 0;
            boost::asio::read(serial_, boost::asio::buffer(&ack, 1)); 
        } catch (...) {}
    }

private:
    boost::asio::io_context io_;
    boost::asio::serial_port serial_;
};

class CrawlerDriver : public rclcpp::Node {
public:
    CrawlerDriver() : Node("crawler_driver") {
        std::string port = declare_parameter<std::string>("serial_port", "/dev/roboclaw");
        counts_per_meter_ = declare_parameter<double>("counts_per_meter", 86646.0);
        m1_qpps_limit_ = declare_parameter<int>("m1_qpps", 61000);
        m2_qpps_limit_ = declare_parameter<int>("m2_qpps", 61000);
        track_width_ = declare_parameter<double>("track_width", 0.4);
        m1_sign_ = declare_parameter<int>("m1_direction_sign", -1);
        m2_sign_ = declare_parameter<int>("m2_direction_sign", -1);

        try {
            roboclaw_ = std::make_unique<SimpleRoboclaw>(port, 38400);
            RCLCPP_INFO(this->get_logger(), "Connected to Roboclaw on %s", port.c_str());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port.c_str());
        }

        sub_raw_ = create_subscription<custom_interfaces::msg::CrawlerVelocity>(
            "/crawler_driver", 10, [this](const custom_interfaces::msg::CrawlerVelocity::SharedPtr msg) {
                this->process_velocity(msg->m1_vel, msg->m2_vel);
            });
        
        sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                const double half = this->track_width_ / 2.0;
                double m1_v = msg->linear.x - msg->angular.z * half;
                double m2_v = msg->linear.x + msg->angular.z * half;
                this->process_velocity(m1_v, m2_v);
            });

        running_ = true;
        worker_ = std::thread([this]() {
            while (this->running_) {
                int32_t m1, m2;
                bool valid = false;
                {
                    std::lock_guard<std::mutex> lk(this->mtx_);
                    if (this->has_new_cmd_) {
                        m1 = this->latest_m1_; m2 = this->latest_m2_;
                        this->has_new_cmd_ = false;
                        valid = true;
                    }
                }
                if (valid && this->roboclaw_) {
                    this->roboclaw_->sendCommand(M1_SPEED_CMD, m1);
                    this->roboclaw_->sendCommand(M2_SPEED_CMD, m2);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        });
    }

    ~CrawlerDriver() {
        running_ = false;
        if (worker_.joinable()) worker_.join();
    }

private:
    void process_velocity(double m1_vel, double m2_vel) {
        int32_t m1 = static_cast<int32_t>(m1_vel * counts_per_meter_) * m1_sign_;
        int32_t m2 = static_cast<int32_t>(m2_vel * counts_per_meter_) * m2_sign_;
        
        m1 = std::max<int32_t>(-m1_qpps_limit_, std::min<int32_t>(m1, m1_qpps_limit_));
        m2 = std::max<int32_t>(-m2_qpps_limit_, std::min<int32_t>(m2, m2_qpps_limit_));

        {
            std::lock_guard<std::mutex> lk(mtx_);
            latest_m1_ = m1; latest_m2_ = m2;
            has_new_cmd_ = true;
        }
    }

    std::unique_ptr<SimpleRoboclaw> roboclaw_;
    double counts_per_meter_, track_width_;
    int32_t m1_qpps_limit_, m2_qpps_limit_, m1_sign_, m2_sign_;

    std::thread worker_;
    std::atomic<bool> running_;
    std::mutex mtx_;
    int32_t latest_m1_ = 0, latest_m2_ = 0;
    bool has_new_cmd_ = false;

    rclcpp::Subscription<custom_interfaces::msg::CrawlerVelocity>::SharedPtr sub_raw_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CrawlerDriver>());
    rclcpp::shutdown();
    return 0;
}
