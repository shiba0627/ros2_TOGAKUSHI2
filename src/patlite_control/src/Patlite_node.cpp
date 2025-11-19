#include <memory>
#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
//#include <std_msgs/msg/int32.hpp>
//#include <std_msgs/msg/string.hpp>
#include "hidapi/hidapi.h"
#include <unistd.h>
#include <chrono>

using namespace std::chrono_literals;

enum class LED_COLORS {
    OFF, RED, GREEN, YELLOW, BLUE, PURPLE, CYAN, WHITE
};

enum class LED_PATTERNS {
    OFF, CONTINUOUS
};

class PatliteNode : public rclcpp::Node{
private:
    bool emergency_stop_; // 緊急停止フラグ
    std::chrono::steady_clock::time_point touch_start_time_; //タッチ開始時刻
    std::chrono::steady_clock::duration press_duration_;     // タッチ継続時間

public:
    PatliteNode() : Node("patlite_node"),
                    emergency_stop_(false),
                    touch_start_time_(std::chrono::steady_clock::time_point::min()),
                    press_duration_(0ms)
    {
        // 初期化
        patlite_init();
        patlite_device_open();

        // プログラム開始時に緑色で点灯
        patlite_lights(LED_COLORS::GREEN, LED_PATTERNS::CONTINUOUS);

        // サブスクライバの作成: 整数メッセージを購読して色を変更
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "led_color", 10, std::bind(&PatliteNode::color_callback, this, std::placeholders::_1));

        // タッチセンサの状態を監視するタイマ
        touch_timer_ = this -> create_wall_timer(
            100ms, std::bind(&PatliteNode::monitor_touch_sensor, this));
        
        // タッチセンサの状態を知らせるためのパブリッシャ
        touch_publisher_ = this -> create_publisher<std_msgs::msg::String>("socket_topic", 10);

    }

    ~PatliteNode() {
        patlite_lights(LED_COLORS::OFF, LED_PATTERNS::CONTINUOUS);
        patlite_device_close();
    }

private:
    void color_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // 受信したメッセージに基づいて色を変更
        switch (msg->data) {
            case 0: patlite_lights(LED_COLORS::OFF, LED_PATTERNS::CONTINUOUS); break;
            case 1: patlite_lights(LED_COLORS::RED, LED_PATTERNS::CONTINUOUS); break;
            case 2: patlite_lights(LED_COLORS::GREEN, LED_PATTERNS::CONTINUOUS); break;
            case 3: patlite_lights(LED_COLORS::YELLOW, LED_PATTERNS::CONTINUOUS); break;
            case 4: patlite_lights(LED_COLORS::BLUE, LED_PATTERNS::CONTINUOUS); break;
            case 5: patlite_lights(LED_COLORS::PURPLE, LED_PATTERNS::CONTINUOUS); break;
            case 6: patlite_lights(LED_COLORS::CYAN, LED_PATTERNS::CONTINUOUS); break;
            case 7: patlite_lights(LED_COLORS::WHITE, LED_PATTERNS::CONTINUOUS); break;
            case 8: 
                // 8を受信したらプログラムを終了
                patlite_lights(LED_COLORS::OFF, LED_PATTERNS::CONTINUOUS);
                rclcpp::shutdown();
                break;
            default:
                std::cerr << "Invalid color code received: " << msg->data << std::endl;
                break;
            
        }
    }

    void monitor_touch_sensor()
    {
        uint8_t buf2[3] = {};
        int result = patlite_get(buf2);

        if (result == -1){
            RCLCPP_ERROR(this->get_logger(), "Failed to read from Patlite device.");
            return;
        }

        std_msgs::msg::String touch_msg; //WHILL制御用メッセージ宣言

        // タッチセンサが入力されたかを監視
        bool is_touched = (buf2[1] == 0x01 || buf2[1] == 0x11);
        
        if (is_touched){ // タッチされた場合
            if (touch_start_time_ == std::chrono::steady_clock::time_point::min()){
                // タッチが開始された時刻を記録
                touch_start_time_ = std::chrono::steady_clock::now();
            } else {
                // タッチの継続時間を計測
                press_duration_ = std::chrono::steady_clock::now() - touch_start_time_;

                // 緊急停止状態で3秒以上押された場合、解除
                if (emergency_stop_ &&  press_duration_ >= 2s){
                    emergency_stop_ = false; // 緊急停止解除
                    RCLCPP_INFO(this -> get_logger(), "緊急停止解除");
                    
                    //　socket_topicに解除することを通知
                    touch_msg.data = "EG_stop_R";
                    touch_publisher_ -> publish(touch_msg);
                    
                    // 緑点灯
                    patlite_lights(LED_COLORS::GREEN, LED_PATTERNS::CONTINUOUS);
                }
            }
        } else {
            if (touch_start_time_ != std::chrono::steady_clock::time_point::min()) {
                // タッチセンサが離された場合、短押しで緊急停止を有効化
                if (!emergency_stop_ && press_duration_ < 2s) {
                    emergency_stop_ = true; 
                    RCLCPP_INFO(this -> get_logger(), "緊急停止");

                    //　socket_topicに緊急停止することを通知
                    touch_msg.data = "EG_stop";
                    touch_publisher_ -> publish(touch_msg);

                    // 紫点灯
                    patlite_lights(LED_COLORS::PURPLE, LED_PATTERNS::CONTINUOUS);
                }
                // タッチセンサ状態のリセット
                touch_start_time_ = std::chrono::steady_clock::time_point::min();
                press_duration_ = 0ms;
            }
        }
    }

    int patlite_get(u_int8_t *buf2){
        int result = 0;

        if (patlite_handle == nullptr){
            std::cerr << "Unable to open device. Please check that it is connected." << std::endl;
            return -1;
        } else {
            uint8_t buf[9] = {0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            result = hid_write(patlite_handle, buf, 9);

            if (result == -1){
                std::cerr << "Patlite set failed, return " << result << "." << std::endl;
                return result;
            }

            result = hid_read(patlite_handle, buf2, 3);
        }

        return result;
    }

    int patlite_lights(LED_COLORS color, LED_PATTERNS ptn) {
        uint8_t buf[9] = {0x00, 0x00, 0x00, 0x08, 0x0f, 0x00, 0x00, 0x00, 0x00};
        int led_pos = 5;

        switch (color) {
            case LED_COLORS::OFF: buf[led_pos] = 0x00; break;
            case LED_COLORS::RED: buf[led_pos] = 0x10; break;
            case LED_COLORS::GREEN: buf[led_pos] = 0x20; break;
            case LED_COLORS::YELLOW: buf[led_pos] = 0x30; break;
            case LED_COLORS::BLUE: buf[led_pos] = 0x40; break;
            case LED_COLORS::PURPLE: buf[led_pos] = 0x50; break;
            case LED_COLORS::CYAN: buf[led_pos] = 0x60; break;
            case LED_COLORS::WHITE: buf[led_pos] = 0x70; break;
            default: buf[led_pos] = 0xF0; break;
        }

        switch (ptn) {
            case LED_PATTERNS::OFF: buf[led_pos] |= 0x00; break;
            case LED_PATTERNS::CONTINUOUS: buf[led_pos] |= 0x01; break;
            default: buf[led_pos] |= 0x0F; break;
        }

        return patlite_set(buf);
    }

    int patlite_set(uint8_t *buf) {
        int result = 0;

        if (patlite_handle == nullptr) {
            std::cerr << "Unable to open device. Please check that it is connected." << std::endl;
            result = -1;
        } else {
            result = hid_write(patlite_handle, buf, 9);

            if (result == -1) {
                std::cerr << "Patlite set failed, return " << result << "." << std::endl;
            }
        }

        return result;
    }

    int patlite_device_open(void) {
        int result = 0;

        patlite_handle = hid_open(0x191A, 0x6001, nullptr); // Vendor ID and Product ID

        if (patlite_handle != nullptr) {
            std::cout << "Succeeded to open device." << std::endl;
        } else {
            std::cerr << "Unable to open device. Please check that it is connected." << std::endl;
            result = -1;
        }

        return result;
    }

    int patlite_device_close(void) {
        if (patlite_handle) {
            hid_close(patlite_handle);
            std::cout << "Succeeded to close device." << std::endl;
        }
        return 0;
    }

    int patlite_init() {
        return hid_init();
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr touch_publisher_; // 停止命令送信用のパブリッシャ
    rclcpp::TimerBase::SharedPtr touch_timer_; // 定期的にタッチセンサの状態を監視するためのタイマ

    static hid_device *patlite_handle;
};

hid_device *PatliteNode::patlite_handle = nullptr;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PatliteNode>());
    rclcpp::shutdown();
    return 0;
}
