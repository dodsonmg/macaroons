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
  explicit Talker(const std::string & node_name, const std::string & topic_name, Macaroon* M)
  : Node(node_name)
  {
    // Create a function for when messages are to be sent.
    auto publish_message =
      [this]() -> void
      {
        msg_ = std::make_unique<std_msgs::msg::String>();
        if((*M_).initialised())
        {
          msg_->data = (*M_).serialise();
          RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());
          // Put the message into a queue to be processed by the middleware.
          // This call is non-blocking.
          pub_->publish(std::move(msg_));
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Publishing: NOTHING.  MACAROON UNINITIALISED.");
        }
      };

    // Create a publisher to the topic which can be matched with one or more compatible ROS
    // subscribers.
    // Note that not all publishers/subscribers on the same topic with the same type will be compatible:
    // they must have compatible Quality of Service policies (in this case, buffer size of 10?).
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);

    // Assign Macaroon
    M_ = M;
  }

private:
  size_t count_ = 1;
  std::string msg_base_;
  std::unique_ptr<std_msgs::msg::String> msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Macaroon* M_;
};

// Create a Listener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Listener : public rclcpp::Node
{
public:
  explicit Listener(const std::string & node_name, const std::string & topic_name, Macaroon* M)
  : Node(node_name)
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
    auto attenuator_topic = std::string("attenuate_macaroon");
    auto user_topic = std::string("use_macaroon");

    // predicates for attenuating any received macaroon
    std::string predicate = "access = read";

    // Create NULL Macaroons for the listeners to use
    Macaroon M_intermediate;
    Macaroon M_user;    

    // Create a node for receiving a macaroon and for transmitting to the final user.
    auto intermediate_listener = std::make_shared<Listener>("intermediate_listener", issuer_topic, &M_intermediate);
    auto intermediate_talker = std::make_shared<Talker>("intermediate_talker", attenuator_topic, &M_intermediate);

    // Create a node for receiving a macaroon and transmitting it back to the owner
    auto user_listener = std::make_shared<Listener>("user_listener", attenuator_topic, &M_user);
    auto user_talker = std::make_shared<Talker>("user_talker", user_topic, &M_user);

    for (int i = 0; i < 10; i++)
    {
      exec.spin_node_once(intermediate_listener);

      if(M_intermediate.initialised())
      {
        // attenuate the Macaroon
        // TODO: What happens if we don't get a new macaroon?  Does it add a repeat caveat?
        M_intermediate.add_first_party_caveat(predicate);

        // transmit the attenuated macaroon
        exec.spin_node_once(intermediate_talker);

        // receive the attenuated macaroon at the user and retransmit back to the owner
        exec.spin_node_once(user_listener);
        exec.spin_node_once(user_talker);
      }

      std::cout << "spun..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    exec.add_node(intermediate_listener);
    exec.add_node(intermediate_talker);
    exec.add_node(user_listener);
    exec.add_node(user_talker);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}