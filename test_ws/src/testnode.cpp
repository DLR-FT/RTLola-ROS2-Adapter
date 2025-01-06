// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "publisher.hpp"
#include "service.hpp"


class TestNode : public rclcpp::Node
{
public:
  TestNode() : Node("rtlola_testnode")
  {
    RCLCPP_INFO(this->get_logger(), "Initialize test node\n");
    m_publisher_ = std::unique_ptr<Publisher>(new Publisher(this));
    m_service_ = std::unique_ptr<ServiceRequest>(new ServiceRequest(this));
  }

private:
  std::unique_ptr<Publisher> m_publisher_;
  std::unique_ptr<ServiceRequest> m_service_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>());
  rclcpp::shutdown();
  return 0;
}
