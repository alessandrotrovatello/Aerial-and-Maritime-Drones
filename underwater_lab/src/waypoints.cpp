#include "waypoints.h"

#include <rclcpp/rclcpp.hpp>
#include <exception>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <cmath>

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;
using namespace std;

class Control : public rclcpp::Node
{
public:
  Control() : Node("waypoints")
  {
    set_parameter(rclcpp::Parameter("use_sim_time", true));

    // load waypoints and thresholds
    Waypoint::load(waypoints, position_thr, orientation_thr);

    pose_cmd.header.frame_id = "world";
    pub = create_publisher<PoseStamped>("/bluerov2/cmd_pose", 1);

    static auto timer{create_wall_timer(10ms, [&]()
      {
        if(!buffer.canTransform("world", "bluerov2/base_link", tf2::TimePointZero))
          return;
        const auto transform{buffer.lookupTransform("world", "bluerov2/base_link", tf2::TimePointZero)};
        trackWaypoint(transform.transform);
      })};
  }

private:

  std::vector<Waypoint> waypoints;
  double position_thr, orientation_thr;

  tf2_ros::Buffer buffer{get_clock()};
  tf2_ros::TransformListener tl{buffer};

  PoseStamped pose_cmd;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub;


  void trackWaypoint(const Transform &pose)
  {
    static auto cur_wp{0};

    // TODO update cur_wp to cycle through the waypoints when the current one is reached
    float x_auv = pose.translation.x;
    float y_auv = pose.translation.y;
    float z_auv = pose.translation.z;
    float z_yaw_auv = pose.rotation.z;
    float w_yaw_auv = pose.rotation.w;
    float yaw = atan2(z_yaw_auv, w_yaw_auv)*2;
    float x_way = waypoints[cur_wp].x;
    float y_way = waypoints[cur_wp].y;
    float z_way = waypoints[cur_wp].z;
    float theta_way = waypoints[cur_wp].theta;

    float dist = sqrt(pow((x_auv - x_way),2)+pow((y_auv - y_way),2)+pow((z_auv - z_way),2));
    float dist_yaw = sqrt(pow((yaw - theta_way),2));

    if (abs(dist) < abs(position_thr)) {
        if (abs(dist_yaw) < abs(orientation_thr)) {
            cur_wp++;
            cout<<"Waypoint " << cur_wp << endl;
        }
    }

    waypoints[cur_wp].write(pose_cmd);
    pose_cmd.header.stamp = get_clock()->now();
    pub->publish(pose_cmd);
  }
};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Control>());
  rclcpp::shutdown();
  return 0;
}
