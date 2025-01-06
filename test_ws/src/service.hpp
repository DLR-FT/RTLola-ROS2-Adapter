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
        m_request_client->async_send_request(request,
            std::bind(&ServiceRequest::handle_response, this, std::placeholders::_1));
    }

    void handle_response(std::shared_future<rtlola_testnode::srv::RTLolaService::Response::SharedPtr> future)
    {
        // Wait for the response (when the future is ready)
        auto response = future.get();
        // Process the response
        RCLCPP_INFO(m_node->get_logger(), "Service call succeeded. Response: i_16 = %ld, u_16 = %ld, i_32 = %ld, u_32 = %ld, i_64 = %ld, u_64 = %ld",
        response->i_16, response->u_16, response->i_32, response->u_32, response->i_64, response->u_64);
    }

private:
    rclcpp::Node* m_node;
    rclcpp::Client<rtlola_testnode::srv::RTLolaService>::SharedPtr m_request_client;
    rclcpp::TimerBase::SharedPtr m_timer_;
};
