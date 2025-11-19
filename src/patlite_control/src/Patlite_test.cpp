#include <memory>
#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "hidapi/hidapi.h"

using namespace std::chrono_literals;

enum class LED_COLORS {
    OFF, RED, GREEN, YELLOW, BLUE, PURPLE, CYAN, WHITE
};

enum class LED_PATTERNS {
    OFF, CONTINUOUS, NO1, NO2, NO3, NO4, NO5, NO6
};

enum class BUZZER_PATTERNS {
    OFF, CONTINUOUS, NO1, NO2, NO3, NO4, NO5, NO6
};

class PatliteNode : public rclcpp::Node
{
public:
    PatliteNode() : Node("patlite_node")
    {
        this->declare_parameter("interval", 3); // Interval in seconds
        timer_ = this->create_wall_timer(
            3s, std::bind(&PatliteNode::timer_callback, this));
        patlite_init();
        patlite_device_open();
    }

    ~PatliteNode() {
        patlite_device_close();
    }

private:
    void timer_callback()
    {
        int result;
        for (int i = 0; i < 7; ++i) {
            switch (i) {
                case 0: result = patlite_lights(LED_COLORS::RED, LED_PATTERNS::CONTINUOUS); break;
                case 1: result = patlite_lights(LED_COLORS::GREEN, LED_PATTERNS::CONTINUOUS); break;
                case 2: result = patlite_lights(LED_COLORS::YELLOW, LED_PATTERNS::CONTINUOUS); break;
                case 3: result = patlite_lights(LED_COLORS::BLUE, LED_PATTERNS::CONTINUOUS); break;
                case 4: result = patlite_lights(LED_COLORS::PURPLE, LED_PATTERNS::CONTINUOUS); break;
                case 5: result = patlite_lights(LED_COLORS::CYAN, LED_PATTERNS::CONTINUOUS); break;
                case 6: result = patlite_lights(LED_COLORS::WHITE, LED_PATTERNS::CONTINUOUS); break;
                default: result = patlite_lights(LED_COLORS::OFF, LED_PATTERNS::CONTINUOUS); break;
            }
            if (result < 0) break;
            rclcpp::sleep_for(3s);
        }

        for (int i = 0; i < 8; ++i) {
            switch (i) {
                case 0: result = patlite_buzzer(BUZZER_PATTERNS::CONTINUOUS); break;
                case 1: result = patlite_buzzer(BUZZER_PATTERNS::NO1); break;
                case 2: result = patlite_buzzer(BUZZER_PATTERNS::NO2); break;
                case 3: result = patlite_buzzer(BUZZER_PATTERNS::NO3); break;
                case 4: result = patlite_buzzer(BUZZER_PATTERNS::NO4); break;
                case 5: result = patlite_buzzer(BUZZER_PATTERNS::NO5); break;
                case 6: result = patlite_buzzer(BUZZER_PATTERNS::NO6); break;
                default: result = patlite_buzzer(BUZZER_PATTERNS::OFF); break;
            }
            if (result < 0) break;
            rclcpp::sleep_for(3s);
        }
    }

    int patlite_lights(LED_COLORS color, LED_PATTERNS ptn);
    int patlite_buzzer(BUZZER_PATTERNS ptn);
    int patlite_set(uint8_t *buf);
    int patlite_device_open(void);
    int patlite_device_close(void);
    int patlite_init(void);

    rclcpp::TimerBase::SharedPtr timer_;
    static hid_device *patlite_handle;
};

hid_device *PatliteNode::patlite_handle = nullptr;

#define PATLITE_VID (0x191A)
#define PATLITE_PID (0x6001)
#define COMMAND_SIZE 9

int PatliteNode::patlite_lights(LED_COLORS color, LED_PATTERNS ptn) {
    uint8_t buf[COMMAND_SIZE] = {0x00, 0x00, 0x00, 0x08, 0x0f, 0x00, 0x00, 0x00, 0x00};
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
        case LED_PATTERNS::NO1: buf[led_pos] |= 0x02; break;
        case LED_PATTERNS::NO2: buf[led_pos] |= 0x03; break;
        case LED_PATTERNS::NO3: buf[led_pos] |= 0x04; break;
        case LED_PATTERNS::NO4: buf[led_pos] |= 0x05; break;
        case LED_PATTERNS::NO5: buf[led_pos] |= 0x06; break;
        case LED_PATTERNS::NO6: buf[led_pos] |= 0x07; break;
        default: buf[led_pos] |= 0x0F; break;
    }

    return patlite_set(buf);
}

int PatliteNode::patlite_buzzer(BUZZER_PATTERNS ptn) {
    uint8_t buf[COMMAND_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x0A, 0xff, 0x00, 0x00, 0x00};
    int buz_pos = 3;

    switch (ptn) {
        case BUZZER_PATTERNS::OFF: buf[buz_pos] |= 0x00; break;
        case BUZZER_PATTERNS::CONTINUOUS: buf[buz_pos] |= 0x01; break;
        case BUZZER_PATTERNS::NO1: buf[buz_pos] |= 0x02; break;
        case BUZZER_PATTERNS::NO2: buf[buz_pos] |= 0x03; break;
        case BUZZER_PATTERNS::NO3: buf[buz_pos] |= 0x04; break;
        case BUZZER_PATTERNS::NO4: buf[buz_pos] |= 0x05; break;
        case BUZZER_PATTERNS::NO5: buf[buz_pos] |= 0x06; break;
        case BUZZER_PATTERNS::NO6: buf[buz_pos] |= 0x07; break;
        default: buf[buz_pos] |= 0x0f; break;
    }

    return patlite_set(buf);
}

int PatliteNode::patlite_set(uint8_t *buf) {
    int result = 0;

    if (patlite_handle == nullptr) {
        std::cerr << "Unable to open device with product ID " << PATLITE_PID << ". Please check that it is connected." << std::endl;
        std::cerr << "Error: patlite_handle is null" << std::endl;
        std::cerr << "Error: " << hid_error(patlite_handle) << std::endl;
        hid_exit();
        result = -1;
    } else {
        result = hid_write(patlite_handle, buf, COMMAND_SIZE);

        if (result == -1) {
            std::cerr << "Patlite set failed, return " << result << ", the handle will be destroyed." << std::endl;
        }
    }

    return result;
}

int PatliteNode::patlite_device_open(void) {
    int result = 0;

    patlite_handle = hid_open(PATLITE_VID, PATLITE_PID, nullptr);

    if (patlite_handle != nullptr) {
        std::cout << "Succeeded to open device with product ID " << PATLITE_PID << "." << std::endl;
    } else {
        std::cerr << "Unable to open device with product ID " << PATLITE_PID << ". Please check that it is connected." << std::endl;
        hid_exit();
        result = -1;
    }

    return result;
}

int PatliteNode::patlite_device_close(void) {
    hid_close(patlite_handle);
    std::cout << "Succeeded to close device with product ID " << PATLITE_PID << "." << std::endl;
    return 0;
}

int PatliteNode::patlite_init() {
    hid_init();
    return 0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PatliteNode>());
    rclcpp::shutdown();
    return 0;
}
