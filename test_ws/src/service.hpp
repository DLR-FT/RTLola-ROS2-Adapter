// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

#include "rclcpp/rclcpp.hpp"
#include "rtlola_testnode/srv/rt_lola_service.hpp"

#include <chrono>
using namespace std::chrono_literals;

#define READ_FAILED -1
using std::placeholders::_1;

class ServiceRequest
{
public:
    ServiceRequest(rclcpp::Node *node) : m_node(node)
    {
        rclcpp::QoS qos(10);
        // qos.best_effort();
        m_request_client = m_node->create_client<rtlola_testnode::srv::RTLolaService>("RTLolaService");
        m_timer_ = m_node->create_wall_timer(1s, std::bind(&ServiceRequest::task, this));
    }

    void task()
    {
        auto request = std::make_shared<rtlola_testnode::srv::RTLolaService::Request>();
        request->s0 = 0;
        request->s1 = 1;
        request->s2 = 2;
        request->s3 = 3;
        request->s4 = 4;
         m_request_client->async_send_request(request);
    }

private:
    rclcpp::Node *m_node;
    rclcpp::Client<rtlola_testnode::srv::RTLolaService>::SharedPtr m_request_client;
    rclcpp::TimerBase::SharedPtr m_timer_;
};
