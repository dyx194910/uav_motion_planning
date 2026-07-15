#pragma region include
#pragma region include::project
#include "path_searching/kino_astar.hh"
#pragma endregion include::project
#pragma region include::third
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sys/stat.h>
#pragma endregion include::third
#pragma region include::standard

#pragma endregion include::standard
#pragma endregion include

path_searching::KinoAStar::Ptr kino_astar_;

ros::Subscriber goal_sub;
ros::Subscriber odom_sub;

ros::Publisher path_pub;

nav_msgs::Odometry::ConstPtr odom_;
std::vector<Eigen::Vector3d> path;

std::string dataset_folder_ = "/home/dsldx/dataset";
std::ofstream csv_file_;
bool csv_initialized_ = false;

void initDataset() {
  std::time_t now = std::time(nullptr);
  char timestamp[64];
  std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&now));

  std::string filename = dataset_folder_ + "/kino_astar_results_" + timestamp + ".csv";

  mkdir(dataset_folder_.c_str(), 0755);

  csv_file_.open(filename);
  csv_file_ << "timestamp,test_name,start_x,start_y,start_z,end_x,end_y,end_z,"
            << "path_found,search_time,node_num,cost,path_length\n";

  csv_initialized_ = true;
  std::cout << "[Dataset] Recording to: " << filename << std::endl;
}

void recordResult(const std::string& test_name,
                  const Eigen::Vector3d& start,
                  const Eigen::Vector3d& end,
                  bool found, double search_time,
                  int node_num, double cost) {
  if (!csv_initialized_) {
    initDataset();
  }

  std::time_t now = std::time(nullptr);
  char time_str[64];
  std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", std::localtime(&now));

  csv_file_ << time_str << ","
            << test_name << ","
            << std::fixed << std::setprecision(4)
            << start(0) << "," << start(1) << "," << start(2) << ","
            << end(0) << "," << end(1) << "," << end(2) << ","
            << (found ? "1" : "0") << ","
            << search_time << ","
            << node_num << ","
            << cost << ","
            << path.size() << "\n";

  csv_file_.flush();
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom) { odom_ = odom; }

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  Eigen::Vector3d end_pt(msg->pose.position.x, msg->pose.position.y,
                         msg->pose.position.z);
  Eigen::Vector3d start_pt(odom_->pose.pose.position.x,
                           odom_->pose.pose.position.y,
                           odom_->pose.pose.position.z);
  Eigen::Vector3d start_vel(odom_->twist.twist.linear.x,
                            odom_->twist.twist.linear.y,
                            odom_->twist.twist.linear.z);
  Eigen::Vector3d end_vel(0, 0, 0);
  std::cout << "Start point: " << start_pt.transpose() << std::endl;
  std::cout << "End point: " << end_pt.transpose() << std::endl;

  ros::Time start_time = ros::Time::now();
  int success = kino_astar_->search(start_pt, start_vel, end_pt, end_vel, path);
  ros::Time end_time = ros::Time::now();
  double search_time = (end_time - start_time).toSec();

  if (success == 1) {
    double cost = 0.0;
    for (size_t i = 1; i < path.size(); i++) {
      cost += (path[i] - path[i-1]).norm();
    }

    recordResult("kino_astar_hill", start_pt, end_pt, true, search_time, 0, cost);

    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "world";
    path_marker.header.stamp = ros::Time::now();

    path_marker.ns = "kino_astar/path";
    path_marker.id = 0;

    path_marker.type = visualization_msgs::Marker::LINE_STRIP;

    path_marker.action = visualization_msgs::Marker::ADD;

    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.08;
    path_marker.scale.y = 0.08;
    path_marker.scale.z = 0.08;
    path_marker.color.a = 1.0;
    path_marker.color.r = 1.0;
    path_marker.color.g = 0.0;
    path_marker.color.b = 0.0;

    for (int i = 0; i < path.size(); i++) {
      geometry_msgs::Point pt;
      pt.x = path[i][0];
      pt.y = path[i][1];
      pt.z = path[i][2];
      path_marker.points.push_back(pt);
    }

    path_pub.publish(path_marker);
    std::cout << "Path found! Time: " << search_time << "s, Cost: " << cost
              << ", Path length: " << path.size() << std::endl;
  } else {
    recordResult("kino_astar_hill", start_pt, end_pt, false, search_time, 0, -1.0);
    std::cout << "Path not found!" << std::endl;
  }
  path.clear();
  kino_astar_->reset();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_kino_astar_searching");
  ros::NodeHandle nh("~");

  nh.param("dataset_folder", dataset_folder_, std::string("/home/dsldx/dataset"));

  ros::Subscriber goal_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("/goal", 10, &GoalCallback);
  ros::Subscriber odom_sub =
      nh.subscribe<nav_msgs::Odometry>("/visual_slam/odom", 10, &OdomCallback);

  path_pub = nh.advertise<visualization_msgs::Marker>("kino_astar/path", 10);

  initDataset();

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  kino_astar_ = std::make_shared<path_searching::KinoAStar>();

  kino_astar_->setParam(nh);
  kino_astar_->setGridMap(grid_map);
  kino_astar_->init();

  ros::spin();

  if (csv_file_.is_open()) {
    csv_file_.close();
  }

  return 0;
}
