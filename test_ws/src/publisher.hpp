// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

#include <unistd.h>
#include <cstdio>

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rtlola_testnode/msg/adc.hpp"
#include "rtlola_testnode/msg/gps_pos.hpp"

#define READ_FAILED -1
using std::placeholders::_1;

class Publisher
{
public:
    Publisher(rclcpp::Node *node) : m_node(node)
    {
        m_node->declare_parameter("Adc", "testnode/Adc");
        m_node->declare_parameter("GpsPos", "testnode/GpsPos");
        rclcpp::QoS qos(10);
        // qos.best_effort();
        m_count = 0;
        // Publisher
        m_publisher1 = m_node->create_publisher<rtlola_testnode::msg::Adc>(m_node->get_parameter("Adc").get_parameter_value().get<std::string>(), qos);
        m_publisher2 = m_node->create_publisher<rtlola_testnode::msg::GpsPos>(m_node->get_parameter("GpsPos").get_parameter_value().get<std::string>(), qos);
        m_timer_ = m_node->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Publisher::task, this));
    }

    void task()
    {
        m_count++;
        auto msg = rtlola_testnode::msg::Adc();
        get_voltage(m_count, msg.a0, msg.a0_read_failed);
        get_voltage(m_count, msg.a1, msg.a1_read_failed);
        get_voltage(m_count, msg.a2, msg.a2_read_failed);
        get_voltage(m_count, msg.a3, msg.a3_read_failed);
        get_voltage(m_count, msg.a4, msg.a4_read_failed);
        get_voltage(m_count, msg.a5, msg.a5_read_failed);
        m_publisher1->publish(msg);

        m_count++;
        auto msg2 = rtlola_testnode::msg::GpsPos();
        msg2.time_of_week = m_count;
        msg2.lon = m_count;
        msg2.lat = m_count;
        msg2.height_above_ellipsoid = 0.3;
        msg2.height_above_mean_sea_level = 0.4;
        msg2.horizontal_accuracy_estimate = 0.5;
        msg2.vertical_accuracy_estimate = 0.6;
        msg2.data_3 = {1.0, 2.0, 3.0};
        msg2.data_n = {1.0, 2.0};
        m_publisher2->publish(msg2);
    }

private:
    rclcpp::Node *m_node;
    int m_count;
    rclcpp::Publisher<rtlola_testnode::msg::Adc>::SharedPtr m_publisher1;
    rclcpp::Publisher<rtlola_testnode::msg::GpsPos>::SharedPtr m_publisher2;
    rclcpp::TimerBase::SharedPtr m_timer_;

    void get_voltage(int channel, float &voltage, int &failure)
    {
        voltage = channel;
        failure = channel;
    }
};
