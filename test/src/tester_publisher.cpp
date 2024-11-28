// SPDX-FileCopyrightText: 2023 German Aerospace Center (DLR)
// SPDX-License-Identifier: Apache-2.0

#include <cstdio>
//#include <algorithm>
#include <memory>
//#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "adc_publisher.hpp"


class TesterPublisher : public rclcpp::Node
{
public:
  TesterPublisher() : Node("tester")
  {
    RCLCPP_INFO(this->get_logger(), "Initialize tester publisher\n");
    m_adc_publisher_ = std::unique_ptr<AdcPublisher>(new AdcPublisher(this));
  }

private:
  std::unique_ptr<AdcPublisher> m_adc_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TesterPublisher>());
  rclcpp::shutdown();
  return 0;
}
