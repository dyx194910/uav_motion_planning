#pragma region include
#pragma region include::project
#include "path_searching/astar.hh"
#pragma endregion include::project
#pragma region include::third
#include <visualization_msgs/Marker.h>
#include <iomanip>
#pragma endregion include::third
#pragma region include::standard

#pragma endregion include::standard
#pragma endregion include

std::shared_ptr<path_searching::AStar> astar_;

ros::Subscriber goal_sub;
ros::Subscriber odom_sub;

ros::Publisher path_pub;
ros::Publisher goal_pub;

nav_msgs::Odometry::ConstPtr odom_;
std::vector<Eigen::Vector3d> path;

std::string test_mode_;
int test_state_;
std::vector<Eigen::Vector3d> test_goals_;
int goal_index_;
int path_id_;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom) { odom_ = odom; }

void publishGoal(Eigen::Vector3d goal) {
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.pose.position.x = goal(0);
  msg.pose.position.y = goal(1);
  msg.pose.position.z = goal(2);
  msg.pose.orientation.w = 1.0;
  goal_pub.publish(msg);
}

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  Eigen::Vector3d end_pt(msg->pose.position.x, msg->pose.position.y,
                         msg->pose.position.z);
  Eigen::Vector3d start_pt(odom_->pose.pose.position.x,
                           odom_->pose.pose.position.y,
                           odom_->pose.pose.position.z);
  std::cout << "Start point: " << start_pt.transpose() << std::endl;
  std::cout << "End point: " << end_pt.transpose() << std::endl;
  int success = astar_->search(start_pt, end_pt, path);

  if (success == 1) {
    float r, g, b;
    if (path_id_ == 0) { r = 1.0; g = 0.0; b = 0.0; }
    else if (path_id_ == 1) { r = 0.0; g = 1.0; b = 0.0; }
    else if (path_id_ == 2) { r = 0.0; g = 0.0; b = 1.0; }
    else if (path_id_ == 3) { r = 1.0; g = 1.0; b = 0.0; }
    else { r = 1.0; g = 0.0; b = 1.0; }

    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "world";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "hill_paths";
    path_marker.id = path_id_;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.15;
    path_marker.color.a = 1.0;
    path_marker.color.r = r;
    path_marker.color.g = g;
    path_marker.color.b = b;

    for (int i = 0; i < path.size(); i++) {
      geometry_msgs::Point pt;
      pt.x = path[i][0];
      pt.y = path[i][1];
      pt.z = path[i][2];
      path_marker.points.push_back(pt);
    }

    path_pub.publish(path_marker);
    std::cout << "Path found! Path ID: " << path_id_ << ", Nodes: " << path.size() << std::endl;
    path_id_++;
  } else {
    std::cout << "Path not found!" << std::endl;
  }
  path.clear();
  astar_->reset();

  if (test_mode_ != "manual") {
    ros::Duration(1.0).sleep();
    if (goal_index_ < test_goals_.size()) {
      std::cout << "\n=== Next goal (" << goal_index_ + 1 << "/" << test_goals_.size() << ") ===" << std::endl;
      publishGoal(test_goals_[goal_index_++]);
    } else {
      std::cout << "\n=== All tests completed ===" << std::endl;
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_hill_terrain_searching");
  ros::NodeHandle nh("~");

  nh.param("test_mode", test_mode_, std::string("manual"));

  if (test_mode_ == "crossing") {
    std::cout << "=== Hill Crossing Test Mode ===" << std::endl;
    test_goals_.push_back(Eigen::Vector3d(10.0, 0.0, 3.5));
    test_goals_.push_back(Eigen::Vector3d(15.0, 0.0, 3.5));
    test_goals_.push_back(Eigen::Vector3d(18.0, 0.0, 3.5));
  } else if (test_mode_ == "bypass") {
    std::cout << "=== Hill Bypass Test Mode ===" << std::endl;
    test_goals_.push_back(Eigen::Vector3d(10.0, -8.0, 1.5));
    test_goals_.push_back(Eigen::Vector3d(15.0, -8.0, 1.5));
    test_goals_.push_back(Eigen::Vector3d(18.0, -8.0, 1.5));
  } else {
    std::cout << "=== Manual Test Mode ===" << std::endl;
  }

  ros::Subscriber goal_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("/goal", 10, &GoalCallback);
  ros::Subscriber odom_sub =
      nh.subscribe<nav_msgs::Odometry>("/visual_slam/odom", 10, &OdomCallback);

  path_pub = nh.advertise<visualization_msgs::Marker>("path", 10);
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 10);

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  astar_ = std::make_shared<path_searching::AStar>();

  astar_->setParam(nh);
  astar_->setGridMap(grid_map);
  astar_->init();

  path_id_ = 0;

  if (test_mode_ != "manual") {
    ros::Duration(2.0).sleep();
    goal_index_ = 0;
    std::cout << "\n=== Starting test with first goal ===" << std::endl;
    publishGoal(test_goals_[goal_index_++]);
  }

  ros::spin();

  return 0;
}
