#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/string.hpp"

/* macaroons */
#include "minimal_composition/macaroon.hpp"
#include "minimal_composition/macaroon_verifier.hpp"

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
  explicit Listener(const std::string & topic_name, MacaroonVerifier* V)
  : Node("listener")
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    auto callback =
      [this](const std_msgs::msg::String::SharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->data.c_str());
        
        // create a macaroon from the recevied serialised message
        Macaroon M(msg->data);

        // verify the macaroon with the private member V_
        int result = (*V_).verify(M);
        if (result == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Received Macaroon verified!");
        }
      };

    // Create a subscription to the topic which can be matched with one or more compatible ROS
    // publishers.
    // Note that not all publishers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies.
    sub_ = create_subscription<std_msgs::msg::String>(topic_name, 10, callback);
    V_ = V;
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  MacaroonVerifier* V_;
};

void print_usage()
{
  printf("Usage for talker app:\n");
  printf("talker [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to publish. Defaults to chatter.\n");
}

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
    auto delegate_topic = std::string("delegate_macaroon");
    //   auto verify_topic = std::string("verify_macaroon");
    auto verify_topic = delegate_topic;


    // Create and serialise a macaroon
    std::string location = "https://www.example.com/";
    std::string key = "this is the key";
    std::string identifier = "keyid";
    std::string predicate = "account = 3735928559";
    Macaroon M(location, key, identifier);
    M.add_first_party_caveat(predicate);
    std::string M_serialised = M.serialise();

    // Create and initialise the MacaroonVerifier
    MacaroonVerifier V(key);
    V.satisfy_exact(predicate);

    // Create a message for the delegate_node to publish (the serialised macaroon)
    auto delegate_msg = M_serialised;

    // Create a node for publishing a delegated macaroon and for receiving macaroons for verification.
    auto delegate_node = std::make_shared<Talker>(delegate_topic, delegate_msg);
    auto verify_node = std::make_shared<Listener>(verify_topic, &V);

    // spin will block until work comes in, execute work as it becomes available, and keep blocking.
    // It will only be interrupted by Ctrl-C.
    //   rclcpp::spin(delegate_node);
    // rclcpp::spin(verify_node);

    exec.add_node(delegate_node);
    exec.add_node(verify_node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}