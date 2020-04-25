#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/string.hpp"

/* macaroons */
#include "macaroon.hpp"
#include "macaroon_verifier.hpp"

using namespace std::chrono_literals;

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Talker : public rclcpp::Node
{
public:
  explicit Talker(const std::string & topic_name, const std::string & msg_base)
  : Node("talker")
  {
    // Create a function for when messages are to be sent.
    auto publish_message =
      [this]() -> void
      {
        msg_ = std::make_unique<std_msgs::msg::String>();
        msg_->data = msg_base_;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        pub_->publish(std::move(msg_));
      };

    // Create a publisher to the topic which can be matched with one or more compatible ROS
    // subscribers.
    // Note that not all publishers/subscribers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies (in this case, buffer size of 10?).
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);

    // Assign incoming message base
    msg_base_ = msg_base;
  }

private:
  size_t count_ = 1;
  std::string msg_base_;
  std::unique_ptr<std_msgs::msg::String> msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node
{
public:
  explicit Listener(const std::string & topic_name, Macaroon* M)
  : Node("listener")
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    auto callback =
      [this](const std_msgs::msg::String::SharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        
        // create a macaroon from the recevied serialised message
        (*M_).deserialise(msg->data);
      };

    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<std_msgs::msg::String>(topic_name, 10, callback);
    M_=M;
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  Macaroon* M_;
};

int main(int argc, char * argv[])
{
    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize any global resources needed by the middleware and the client library.
    // You must call this before using any other part of the ROS system.
    // This should be called once per process.
    rclcpp::init(argc, argv);

    // create single threaded executor to run both the publisher and subscriber
    // within the same process
    rclcpp::executors::SingleThreadedExecutor exec;

    // Establish the topics for delegating a macaroon and receiving one for verification.
    auto issuer_topic = std::string("issue_macaroon");
    auto user_topic = std::string("use_macaroon");

    // Create and serialise a macaroon
    std::string location = "https://www.example.com/";
    std::string key = "this is the key";
    std::string identifier = "keyid";
    std::string predicate1 = "account = 3735928559";
    std::string predicate2 = "access = read";
    Macaroon M(location, key, identifier);
    M.add_first_party_caveat(predicate1);
    std::string M_serialised = M.serialise();

    // Create and initialise the MacaroonVerifier
    MacaroonVerifier V(key);
    V.satisfy_exact(predicate1);

    // Create a Macaroon for the listener to use
    Macaroon M_received;

    // Create a node for issuing a macaroon and for receiving macaroons from users.
    auto issuer_node = std::make_shared<Talker>(issuer_topic, M_serialised);
    // auto verifier_node = std::make_shared<Listener>(issuer_topic, &M_received);
    auto verifier_node = std::make_shared<Listener>(user_topic, &M_received);

    for (int i = 0; i < 5; i++)
    {
      exec.spin_node_once(issuer_node);
      exec.spin_node_once(verifier_node);

      if(M_received.initialised())
      {
        // verify the macaroon
        int result = V.verify(M_received);
        if (result == 0)
        {
            std::cout << "Received Macaroon verified!" << std::endl;
        }
      }

      std::cout << "spun..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << std::endl << "ADDING PREDICATE FOR ACCESS LEVEL" << std::endl << std::endl;

    V.satisfy_exact(predicate2);

    for (int i = 0; i < 5; i++)
    {
      exec.spin_node_once(issuer_node);
      exec.spin_node_once(verifier_node);

      if(M_received.initialised())
      {
        // verify the macaroon
        int result = V.verify(M_received);
        if (result == 0)
        {
            std::cout << "Received Macaroon verified!" << std::endl;
        }
      }

      std::cout << "spun..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    // exec.add_node(delegate_node);
    // exec.add_node(verify_node);    
    // exec.spin();

    rclcpp::shutdown();
    return 0;
}