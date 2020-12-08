//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================


#include <vector>
#include <unordered_map>

#include "eigen3/Eigen/Dense"
#include "vector_map/vector_map.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Node{
    int x_;
    int y_;
    bool visited_ = false;
    float heuristic_ = 0.0f;
    float res = 2.0f;
    float cost_ = static_cast<float>(INT_MAX);

    Node(Eigen::Vector2f loc, float r){
	res = (1/r);
        float tmp_x = round(loc.x()*res) / res;
        float tmp_y = round(loc.y()*res) / res;
        x_ = (int) round(tmp_x*10.0f);
        y_ = (int) round(tmp_y*10.0f);
    }
    Node(){}

    Eigen::Vector2f get_loc(){
        return Eigen::Vector2f(x_/10.0f, y_/10.0f);
    }

    bool operator==(const Node &n) const
	{
		return x_ == n.x_ && y_ == n.y_;
	}
};

struct Node_hash_fn{
    size_t operator() (const Node & n) const{
        size_t h1 = std::hash<int>()(n.x_);
        size_t h2 = std::hash<int>()(n.y_);

        return h1^h2;
    }

};


class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);
  // Update point cloud using latest laidar readings
  void UpdatePointCloud(const std::vector<Eigen::Vector2f>& point_cloud_);
  // run obstacle avoidance
  void RunObstacleAvoidance(const Eigen::Vector2f& local_goal);
  void Plan();
  bool ReachedGoal();
  bool PlanStillValid();
  Eigen::Vector2f GetNextCarrot();
  bool intersects_wall(Eigen::Vector2f & start, Eigen::Vector2f & end, vector_map::VectorMap & map_);
 private:

  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
};

}  // namespace navigation

#endif  // NAVIGATION_H
