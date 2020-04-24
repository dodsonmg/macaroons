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

using namespace std::chrono_literals;

PublisherNode::PublisherNode(rclcpp::NodeOptions options)
: Node("publisher_node", options), count_(0)
{
  publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = create_wall_timer(
    1000ms, std::bind(&PublisherNode::on_timer, this));
}

void PublisherNode::on_timer()
{
  std::string location = "https://www.example.com/" + std::to_string(count_);
  std::string key = "this is the key";
  std::string identifier = "keyid";
  Macaroon M(location, key, identifier);
  // struct macaroon* M = get_macaroon(location, key, identifier);
  struct macaroon* N = NULL;
  std::string M_serialised = M.serialise();

  if(M.initialised()) {
    std::cout << M_serialised << std::endl; 
    
    N = M.deserialise_macaroon(M_serialised);
  }

  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publisher: '%s'", message.data.c_str());
  publisher_->publish(message);
}

// std::string
// PublisherNode::serialise_macaroon(struct macaroon* M)
// {
//   enum macaroon_returncode err;
//   size_t buf_sz = 0;
//   unsigned char* buf = NULL;

//   /* DEBUG */
//   std::cout << "serialising:" << std::endl;
//   size_t data_sz = 0;
//   char* data = NULL;
//   data_sz = macaroon_inspect_size_hint(M);
//   data = (char*)malloc(data_sz);
//   macaroon_inspect(M, data, data_sz, &err);
//   std::cout << data << std::endl;
//   /* END DEBUG */

//   // If M is NULL
//   if(!M)
//   {
//     std::cout << "The macaroon is NULL" << std::endl;
//     return "";
//   }

//   buf_sz = macaroon_serialize_size_hint(M, MACAROON_V1);
//   buf = (unsigned char*)malloc(buf_sz);

//   macaroon_serialize(M, MACAROON_V1, buf, buf_sz, &err);

//   std::string serialised( (const char *)buf );

//   // std::cout << serialised << std::endl;
  
//   return serialised;
// }

// struct macaroon*
// PublisherNode::deserialise_macaroon(std::string M_serialised)
// {
//   const unsigned char* data = (const unsigned char*)M_serialised.c_str();
//   size_t data_sz = M_serialised.size();
//   enum macaroon_returncode err;
//   struct macaroon* M = NULL;

//   M = macaroon_deserialize(data, data_sz, &err);

//   if(!M) { print_macaroon_error(err); }
//   else {
//     /* DEBUG */
//     std::cout << "deserialising:" << std::endl;
//     size_t data_sz = 0;
//     char* data = NULL;
//     data_sz = macaroon_inspect_size_hint(M);
//     data = (char*)malloc(data_sz);
//     macaroon_inspect(M, data, data_sz, &err);
//     std::cout << data << std::endl;
//     /* END DEBUG */
//   }

//   return M;
// }

// struct macaroon*
// PublisherNode::get_macaroon(const std::string location, const std::string key, const std::string identifier)
// {
//   struct macaroon* M = NULL;
//   enum macaroon_returncode err;

//   const unsigned char* plocation = (const unsigned char*)location.c_str();
//   const unsigned char* pkey = (const unsigned char*)key.c_str();
//   const unsigned char* pidentifier = (const unsigned char*)identifier.c_str();

//   size_t location_sz = location.size();
//   size_t key_sz = key.size();
//   size_t identifier_sz = identifier.size();

//   /* DEBUG */
//   // std::cout << plocation << "\t" << location_sz << std::endl;
//   // std::cout << pkey << "\t" << key_sz << std::endl;
//   // std::cout << pidentifier << "\t" << identifier_sz << std::endl;
//   /* END DEBUG */

//   M = macaroon_create(plocation, location_sz, 
//                       pkey, key_sz, 
//                       pidentifier, identifier_sz, &err);

//   /* DEBUG */
//   // std::cout << "creating:" << std::endl;
//   // size_t data_sz = 0;
//   // char* data = NULL;
//   // data_sz = macaroon_inspect_size_hint(M);
//   // data = (char*)malloc(data_sz);
//   // macaroon_inspect(M, data, data_sz, &err);
//   // std::cout << data << std::endl;
//   /* END DEBUG */

//   // if macaroon creation fails, M will be NULL and an error code (hopefully) returned
//   if(!M)
//   {
//     print_macaroon_error(err);
//     return NULL;
//   }

//   return M;
// }

// void
// PublisherNode::print_macaroon_error(enum macaroon_returncode err)
// {
//   std::cout << "Macaroon not created" << std::endl;
//   std::cout << "Error:" << err << std::endl;
//   std::cout << "MACAROON_SUCCESS:" << MACAROON_SUCCESS << std::endl;
//   std::cout << "MACAROON_OUT_OF_MEMORY:" << MACAROON_OUT_OF_MEMORY << std::endl;
//   std::cout << "MACAROON_HASH_FAILED:" << MACAROON_HASH_FAILED << std::endl;
//   std::cout << "MACAROON_INVALID:" << MACAROON_INVALID << std::endl;
//   std::cout << "MACAROON_TOO_MANY_CAVEATS:" << MACAROON_TOO_MANY_CAVEATS << std::endl;
//   std::cout << "MACAROON_CYCLE:" << MACAROON_CYCLE << std::endl;
//   std::cout << "MACAROON_BUF_TOO_SMALL:" << MACAROON_BUF_TOO_SMALL << std::endl;
//   std::cout << "MACAROON_NOT_AUTHORIZED:" << MACAROON_NOT_AUTHORIZED << std::endl;
//   std::cout << "MACAROON_NO_JSON_SUPPORT:" << MACAROON_NO_JSON_SUPPORT << std::endl;
//   std::cout << "MACAROON_UNSUPPORTED_FORMAT:" << MACAROON_UNSUPPORTED_FORMAT << std::endl;
// }

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(PublisherNode)
