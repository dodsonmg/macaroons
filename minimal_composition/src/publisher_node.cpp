// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>

#include "minimal_composition/publisher_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/* macaroons */
// #include "macaroons.h"
#include "minimal_composition/macaroon.hpp"
#include "minimal_composition/macaroon_verifier.hpp"

using namespace std::chrono_literals;

PublisherNode::PublisherNode(rclcpp::NodeOptions options)
: Node("publisher_node", options), count_(0)
{
  publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = create_wall_timer(
    2000ms, std::bind(&PublisherNode::on_timer, this));
}

void PublisherNode::on_timer()
{
  std::string location = "https://www.example.com/" + std::to_string(count_);
  std::string key = "this is the key";
  std::string identifier = "keyid";
  std::string predicate = "account = 3735928559";

  // create macaroon and serialise it
  std::cout << "\nCreate macaroon" << std::endl;
  Macaroon M(location, key, identifier);
  M.print_macaroon();

  std::string M_serialised = M.serialise();
  std::cout << "\nSerialise macaroon:" << M_serialised << std::endl;

  // add caveat and serialise it
  std::cout << "\nAdd caveat:" << std::endl;
  M.add_first_party_caveat(predicate);
  M.print_macaroon();

  M_serialised = M.serialise();
  std::cout << "\nSerialise macaroon:" << M_serialised << std::endl;

  // desearialise as a second macaroon
  Macaroon N(M_serialised);
  std::cout << "\nDeserialise macaroon" << std::endl;

  // create a verifier and verify
  MacaroonVerifier V;
  V.satisfy_exact(predicate);
  
  std::cout << "\n(GOOD) Verify macaroon" << std::endl;
  V.verify(N, key);

  std::string key_bad = "this is not the key";
  std::cout << "\n(BAD) Verify macaroon" << std::endl;
  V.verify(N, key_bad);
  std::cout << std::endl;

  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publisher: '%s'", message.data.c_str());
  publisher_->publish(message);
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(PublisherNode)
