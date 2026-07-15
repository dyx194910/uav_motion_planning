#pragma region include
#pragma region include::project
#include "path_searching/rrt_star.hh"
#include "traj_optimization/minimum_control.hh"
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

path_searching::RRTStar::Ptr rrt_star_;
traj_optimization::MinimumControl::Ptr optimizer_;

ros::Subscriber goal_sub;
ros::Subscriber odom_sub;

ros::Publisher trajectory_pub;
visualization_msgs::Marker trajectory_marker;

nav_msgs::Odometry::ConstPtr odom_;
std::vector<Eigen::Vector3d> path;
std::vector<Eigen::Vector3d> optimal_path;
Eigen::VectorXd coef_1d;
std::vector<double> x_vec;
std::vector<double> y_vec;
std::vector<double> z_vec;

std::string dataset_folder_ = "/home/dsldx/dataset";
std::ofstream csv_file_;
bool csv_initialized_ = false;

void initDataset() {
  std::time_t now = std::time(nullptr);
  char timestamp[64];
  std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&now));

  std::string filename = dataset_folder_ + "/minimum_jerk_results_" + timestamp + ".csv";

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
            << optimal_path.size() << "\n";

  csv_file_.flush();
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom) { odom_ = odom; }

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  Eigen::Vector3d end_pt(msg->pose.position.x, msg->pose.position.y,
                         msg->pose.position.z);
  Eigen::Vector3d end_vel(0.0, 0.0, 0.0);
  Eigen::Vector3d end_acc(0.0, 0.0, 0.0);
  Eigen::Vector3d start_pt(odom_->pose.pose.position.x,
                           odom_->pose.pose.position.y,
                           odom_->pose.pose.position.z);
  Eigen::Vector3d start_vel(odom_->twist.twist.linear.x,
                            odom_->twist.twist.linear.y,
                            odom_->twist.twist.linear.z);
  Eigen::Vector3d start_acc(0.0, 0.0, 0.0);
  std::cout << "Start point: " << start_pt.transpose() << std::endl;
  std::cout << "End point: " << end_pt.transpose() << std::endl;

  ros::Time start_time = ros::Time::now();
  int success = rrt_star_->search(start_pt, end_pt, path);
  optimal_path = rrt_star_->getOptimalPath();
  ros::Time end_time = ros::Time::now();
  double search_time = (end_time - start_time).toSec();

  if (success == 1) {
    double cost = 0.0;
    for (size_t i = 1; i < optimal_path.size(); i++) {
      cost += (optimal_path[i] - optimal_path[i-1]).norm();
    }

    recordResult("minimum_jerk_hill", start_pt, end_pt, true, search_time, 0, cost);

    std::cout << "Path found, Start optimization" << std::endl;
    Eigen::VectorXd pos_x;
    Eigen::VectorXd pos_y;
    Eigen::VectorXd pos_z;
    pos_x.resize(optimal_path.size());
    pos_y.resize(optimal_path.size());
    pos_z.resize(optimal_path.size());
    for (int i = 0; i < optimal_path.size(); i++) {
      pos_x(i) = optimal_path[i][0];
      pos_y(i) = optimal_path[i][1];
      pos_z(i) = optimal_path[i][2];
    }
    Eigen::Vector2d bound_vel_x(start_vel[0], end_vel[0]);
    Eigen::Vector2d bound_vel_y(start_vel[1], end_vel[1]);
    Eigen::Vector2d bound_vel_z(start_vel[2], end_vel[2]);

    Eigen::Vector2d bound_acc_x(start_acc[0], end_acc[0]);
    Eigen::Vector2d bound_acc_y(start_acc[1], end_acc[1]);
    Eigen::Vector2d bound_acc_z(start_acc[2], end_acc[2]);

    Eigen::VectorXd time_vec;
    time_vec.resize(optimal_path.size() - 1);
    for (int i = 0; i < optimal_path.size() - 1; i++) {
      time_vec(i) = 1.0;
    }

    bool success_x =
        optimizer_->solve(pos_x, bound_vel_x, bound_acc_x, time_vec);
    if (success_x) {
      coef_1d = optimizer_->getCoef1d();
      for (int i = 0; i < optimal_path.size() - 1; i++) {
        for (double t = 0; t < time_vec(i); t += 0.1) {
          Eigen::Matrix<double, 1, 6> coef_matrix =
              Eigen::Matrix<double, 1, 6>::Zero();
          coef_matrix << coef_1d(6 * i), coef_1d(6 * i + 1), coef_1d(6 * i + 2),
              coef_1d(6 * i + 3), coef_1d(6 * i + 4), coef_1d(6 * i + 5);
          Eigen::Matrix<double, 6, 1> t_vector =
              Eigen::Matrix<double, 6, 1>::Zero();
          for (int i = 0; i < 6; i++) {
            t_vector(i) = pow(t, i);
          }
          x_vec.push_back(coef_matrix * t_vector);
        }
      }
      x_vec.push_back(end_pt[0]);
    } else {
      std::cout << "optimize failure!" << std::endl;
    }

    bool success_y =
        optimizer_->solve(pos_y, bound_vel_y, bound_acc_y, time_vec);
    if (success_y) {
      coef_1d = optimizer_->getCoef1d();
      for (int i = 0; i < optimal_path.size() - 1; i++) {
        for (double t = 0; t < time_vec(i); t += 0.1) {
          Eigen::Matrix<double, 1, 6> coef_matrix;
          coef_matrix << coef_1d(6 * i), coef_1d(6 * i + 1), coef_1d(6 * i + 2),
              coef_1d(6 * i + 3), coef_1d(6 * i + 4), coef_1d(6 * i + 5);
          Eigen::Matrix<double, 6, 1> t_vector;
          for (int i = 0; i < 6; i++) {
            t_vector(i) = pow(t, i);
          }
          y_vec.push_back(coef_matrix * t_vector);
        }
      }
      y_vec.push_back(end_pt[1]);
    } else {
      std::cout << "optimize failure!" << std::endl;
    }

    bool success_z =
        optimizer_->solve(pos_z, bound_vel_z, bound_acc_z, time_vec);
    if (success_z) {
      coef_1d = optimizer_->getCoef1d();
      for (int i = 0; i < optimal_path.size() - 1; i++) {
        for (double t = 0; t < time_vec(i); t += 0.1) {
          Eigen::Matrix<double, 1, 6> coef_matrix;
          coef_matrix << coef_1d(6 * i), coef_1d(6 * i + 1), coef_1d(6 * i + 2),
              coef_1d(6 * i + 3), coef_1d(6 * i + 4), coef_1d(6 * i + 5);
          Eigen::Matrix<double, 6, 1> t_vector;
          for (int i = 0; i < 6; i++) {
            t_vector(i) = pow(t, i);
          }
          z_vec.push_back(coef_matrix * t_vector);
        }
      }
      z_vec.push_back(end_pt[2]);
    } else {
      std::cout << "optimize failure!" << std::endl;
    }

    for (int i = 0; i < x_vec.size(); i++) {
      geometry_msgs::Point pt;
      pt.x = x_vec[i];
      pt.y = y_vec[i];
      pt.z = z_vec[i];
      trajectory_marker.points.push_back(pt);
    }
    trajectory_pub.publish(trajectory_marker);
    std::cout << "Optimization complete! Path length: " << optimal_path.size() << std::endl;
  } else {
    recordResult("minimum_jerk_hill", start_pt, end_pt, false, search_time, 0, -1.0);
    std::cout << "Path not found!" << std::endl;
  }
  path.clear();
  optimal_path.clear();
  trajectory_marker.points.clear();
  x_vec.clear();
  y_vec.clear();
  z_vec.clear();
  rrt_star_->reset();
  optimizer_->reset();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_minimum_jerk");
  ros::NodeHandle nh("~");

  nh.param("dataset_folder", dataset_folder_, std::string("/home/dsldx/dataset"));

  goal_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("/goal", 10, &GoalCallback);
  odom_sub =
      nh.subscribe<nav_msgs::Odometry>("/visual_slam/odom", 10, &OdomCallback);

  trajectory_pub = nh.advertise<visualization_msgs::Marker>("/trajectory", 10);

  trajectory_marker.header.frame_id = "world";
  trajectory_marker.header.stamp = ros::Time::now();
  trajectory_marker.ns = "trajectory";
  trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_marker.action = visualization_msgs::Marker::ADD;
  trajectory_marker.pose.orientation.w = 1.0;
  trajectory_marker.scale.x = 0.1;
  trajectory_marker.scale.y = 0.1;
  trajectory_marker.scale.z = 0.1;
  trajectory_marker.color.a = 1.0;
  trajectory_marker.color.r = 1.0;
  trajectory_marker.color.g = 0.0;
  trajectory_marker.color.b = 1.0;

  initDataset();

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  rrt_star_ = std::make_shared<path_searching::RRTStar>();
  optimizer_ = std::make_shared<traj_optimization::MinimumControl>();

  rrt_star_->setParam(nh);
  rrt_star_->setGridMap(grid_map);
  rrt_star_->init();

  ros::spin();

  if (csv_file_.is_open()) {
    csv_file_.close();
  }

  return 0;
}
