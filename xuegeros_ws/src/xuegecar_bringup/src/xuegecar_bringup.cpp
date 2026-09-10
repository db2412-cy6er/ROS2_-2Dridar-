#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <memory>
#include <string>

// 订阅小车主控固件(micro-ROS)发布的 /odom,并把它转发为 TF: odom -> base_footprint。
// 设计要点:
//   1. 固件 /odom 为 best-effort,故用 SensorDataQoS 订阅;
//   2. 收到第一条 /odom 后才广播 TF —— 避免用伪造的零位姿 TF 掩盖"小车未连接"故障;
//   3. TF 发布频率与 /odom 一致(~50Hz),去掉原来 1000Hz 空转刷 TF 的写法;
//   4. odom -> base_footprint 的 z 固定为 0(路面/里程计平面基准),
//      避免把 base_link 高度(URDF 0.076)带到 TF 树(对应 P3/P4 报告的 -0.076 垂直基准问题)。
class TopicSubscribe01 : public rclcpp::Node
{
public:
  explicit TopicSubscribe01(std::string name) : Node(name), odom_received_(false)
  {
    odom_subscribe_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&TopicSubscribe01::odom_callback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    warn_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&TopicSubscribe01::warn_no_odom, this));
    // 中继:把固件 /odom(原 child=base_link, z=0 的平地盘式)重发布为
    // /odom/base_footprint(child=base_footprint),供 cartographer 以
    // tracking_frame=base_footprint 消费,从而把 map 平面锚在路面(z=0)。
    odom_ground_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odom/base_footprint", rclcpp::SensorDataQoS());
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscribe_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr warn_timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_ground_pub_;
  nav_msgs::msg::Odometry odom_msg_;
  bool odom_received_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 沿用 /odom 自带时间戳;z 固定 0(地面/里程计平面)。
    odom_msg_.header.stamp = msg->header.stamp;
    odom_msg_.pose.pose.position.x = msg->pose.pose.position.x;
    odom_msg_.pose.pose.position.y = msg->pose.pose.position.y;
    odom_msg_.pose.pose.position.z = 0.0;
    odom_msg_.pose.pose.orientation = msg->pose.pose.orientation;
    odom_received_ = true;

    publish_tf();
    publish_odom_ground(msg);
  }

  void publish_tf()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odom_msg_.header.stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";

    transform.transform.translation.x = odom_msg_.pose.pose.position.x;
    transform.transform.translation.y = odom_msg_.pose.pose.position.y;
    transform.transform.translation.z = 0.0;  // 固定路面/里程计平面基准
    transform.transform.rotation = odom_msg_.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void publish_odom_ground(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry out;
    out.header = msg->header;            // frame_id=odom, stamp 不变
    out.child_frame_id = "base_footprint";
    out.pose.pose.position.x = msg->pose.pose.position.x;
    out.pose.pose.position.y = msg->pose.pose.position.y;
    out.pose.pose.position.z = 0.0;      // 地面/里程计平面
    out.pose.pose.orientation = msg->pose.pose.orientation;
    out.twist = msg->twist;
    odom_ground_pub_->publish(out);
  }

  void warn_no_odom()
  {
    if (!odom_received_) {
      RCLCPP_WARN(this->get_logger(),
                  "仍收不到 /odom:小车尚未连上 micro-ROS Agent(UDP 8888)?请检查 agent 与小车主控状态");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  /*产生一个的节点*/
  auto node = std::make_shared<TopicSubscribe01>("xuegecar_bringup");
  /* 运行节点，并检测退出信号;TF 在 /odom 回调里按 50Hz 发布 */
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
