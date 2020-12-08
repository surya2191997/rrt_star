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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <unordered_map>
#include <unordered_set>

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"

#include "vector_map/vector_map.h"

#include "visualization/visualization.h"
#include "simple_queue.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::unordered_map;
using std::unordered_set;
using geometry::FurthestFreePointCircle;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
VisualizationMsg local_path_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.

float kEpsilon = 1e-6;
// float res = 0.5f;
//bool stop = false;

unordered_map<int, float> rrt_cost;
unordered_map<int , Vector2f> pos_map;
unordered_map<int, vector<int> > rrt_graph; 
unordered_map<int, int> rrt_parent; 
int rrt_iterations = 20000;
Vector2f curr_pos;
float curr_angle;
float odom_vel;
float curr_speed;
vector_map::VectorMap  map_;
//float rrt_n = 0.2;

} //namespace

namespace navigation {

struct PathOption curr_best;

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  //local_path_msg_ = visualization::NewVisualizationMessage("base_link", "navigation_paths");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
  curr_best.free_path_length = 4.0f;
  curr_best.curvature = 0.0f;
  map_.Load(map_file);
}

float computeDistSqr(const Vector2f& A, const Vector2f& B) {
    return pow(A(0)-B(0), 2.0) + pow(A(1)-B(1), 2.0);
}

bool colinear(const Vector2f & p0, const Vector2f & p1, const Vector2f & p2){
    float area = p0.x() * (p1.y() - p2.y()) + p1.x() * (p2.y() - p0.y()) + p2.x() * (p0.y() - p1.y());
    area *= 0.5;
    if(area > -kEpsilon && area < kEpsilon){
        return true;
    } 
    return false;
}

bool intersects_wall(Vector2f & start, Vector2f & end, vector_map::VectorMap & map_){
    for(const auto & line : map_.lines){
        if(line.Intersects(start, end)){
            return true;
        }
    }
    return false;
}


bool too_close(Vector2f & p, vector_map::VectorMap & map_){
    for(auto const & line : map_.lines){
        Vector2f line_perp = line.UnitNormal();
        Vector2f line_dir = line.Dir();
        float thresh = 0.3;
        if(line.RayIntersects(p, line_perp)){
            float dist = (line.RayIntersection(p, line_perp) - p).norm();
            if(dist < thresh){
                return true;
            }
        }
        if(line.RayIntersects(p, line_dir)){
            float dist = (line.RayIntersection(p, line_dir) - p).norm();
            if(dist < thresh){
                return true;
            }
        }
        if((p-line.p0).norm() < thresh){
            return true;
        }
        if((p-line.p1).norm() < thresh){
            return true;
        }
    }
    return false;
}

int Nearest(Vector2f x_rand) {
    int minm = 10e6, nearest = 0;
    for(pair<int, vector<int>> p : rrt_graph) {
        if((pos_map[p.first]-x_rand).norm() < minm) {
            minm = (pos_map[p.first]-x_rand).norm();
            nearest = p.first;
        }
    }
    return nearest;
}




vector<Vector2f> A_star(Vector2f start, const Vector2f end, vector_map::VectorMap & map_){
    vector<Vector2f> path;
    //Get nearest start node from graph. - 
    int init = Nearest(start);
    cout << init << "Astart" << pos_map[init] << endl;

    //Get nearest end node from graph - 
    int dest = Nearest(end);
    cout << dest << "Astart" << pos_map[dest] << endl;
    //cout << dest.id_ << endl;

    //using those nodes, run A* :))

    SimpleQueue<int, float> pq;

    unordered_map<int, int> parent;
    unordered_map<int, float> cost;

    cost[init] = 0.0f;

    //Negative for priroity to account for highest priority.
    pq.Push(init, -1.0*cost[init]);
    unordered_set<int> closed;

    // cout << "count: " << NodePrm::objectCount << endl;
    bool path_found = false;

    while(!pq.Empty()){
        int curr = pq.Pop();
        Vector2f curr_loc = pos_map[curr];

        if(curr == dest){
            //cout << "HYPE" << endl;
            path_found = true;
            break;
        }

        vector<int> & neighbors = rrt_graph[curr];
        cout << neighbors.size() << endl;

        for(int & next : neighbors){
            Vector2f next_loc = pos_map[next];
            float next_cost = cost[curr] + (next_loc - curr_loc).norm();
            float heur = (end - next_loc).norm();
            float priority = next_cost + heur;
            if(pq.Exists(next) && cost[next] <= next_cost){
                continue;
            }
            else if(closed.find(next) != closed.end() && cost[next] <= next_cost){
                continue;
            }
            else{
                pq.Push(next, -1.0* priority);
                cost[next] = next_cost;
                parent[next] = curr;
            }
        }
	cout << curr << "Astart" << pos_map[curr] << endl;
        closed.insert(curr);
    }

    if(!path_found){
        path.push_back(start);
        return path;
    }

    int &n = dest;
    //cout << dest.id_ << endl;
    path.push_back(end);
    path.push_back(pos_map[dest]);
    int index = 0;
    while(parent.find(n) != parent.end()){
        n = parent[n];
        path.push_back(pos_map[n]);
        index++;
    }
    path.push_back(start);

    return path;
}


Vector2f SampleFree(int i) {
    float rand1 = ((float) rand() / (RAND_MAX)) ;
    float rand2 = ((float) rand() / (RAND_MAX)) ;
    // cout << rand1 << " " <<rand2 << endl;
    Vector2f new_point(10-20.0*rand1, 10-20.0*rand2);
    cout << new_point.x() << " " << new_point.y() << endl;
    return new_point;
}

int Steer(Vector2f s, Vector2f e) {
    Vector2f mid = (s+e)/2;
    float delta = 0.2;
    //cout << "start :" << s << endl;
    //cout << "end :  " << e << endl;
    while((s-mid).norm() > delta) {
        mid =  (s+mid)/2;
        //cout << "mid : " << mid << endl;
    }
    int l = rrt_graph.size();
    rrt_graph[l] = {};
    pos_map[l] = mid;
    return l;
}

vector<int> Near(int i, float coeff) {
    vector<int> ans;
    for(pair<int, vector<int>> p : rrt_graph) {
        if(i != p.first && (pos_map[p.first]-pos_map[i]).norm() < coeff) 
            ans.push_back(p.first);
    }
    return ans;
}

void rrt_star() { // create graph using rapidly exploring random trees
    // pos_map
    // rrt_graph
    visualization::ClearVisualizationMsg(local_viz_msg_);
    visualization::ClearVisualizationMsg(global_viz_msg_);

    cout << "rrt star called" << endl;
    rrt_graph.clear();
    rrt_cost.clear();
    rrt_parent.clear();
    pos_map.clear();

    rrt_graph[0] = {};
    pos_map[0] = curr_pos;
    rrt_cost[0] = 0;
    float coeff = 1.0;
    for(int i = 0; i < rrt_iterations; i++) {
        Vector2f x_rand = SampleFree(i);
        int x_nearest = Nearest(x_rand);
        if(!intersects_wall(pos_map[x_nearest], x_rand, map_)) {
            int x_new = Steer(pos_map[x_nearest], x_rand);
	    //cout << "xrand : " << x_rand << endl;
	    //cout <<  "x_nearest" << x_nearest << endl;
	    //cout << "xnew : " << x_new << " " << pos_map[x_new] << endl;
            vector<int> X_near = Near(x_new, coeff);
            int x_min = x_nearest;
	    float c_min =  rrt_cost[x_nearest] + (pos_map[x_nearest]-pos_map[x_new]).norm();
            for(int x : X_near) {
		cout << "in ball" << x<< " " << pos_map[x] << endl;
                if(!intersects_wall(pos_map[x], pos_map[x_new], map_) && 
                    rrt_cost[x] + (pos_map[x]-pos_map[x_new]).norm() < c_min) {
                        x_min = x;
                        c_min = rrt_cost[x] + (pos_map[x]-pos_map[x_new]).norm();
                    }
            }
	    cout << "minx" << x_min << endl;
	    cout << "xnew" << x_new << endl;
            rrt_graph[x_min].push_back(x_new);
            rrt_graph[x_new].push_back(x_min);
            rrt_parent[x_new] = x_min;
	    rrt_cost[x_new] = rrt_cost[x_min] + (pos_map[x_min]-pos_map[x_new]).norm();
	    
            for(int x : X_near) {
                if(!intersects_wall(pos_map[x], pos_map[x_new], map_) && 
                    rrt_cost[x_new] + (pos_map[x]-pos_map[x_new]).norm() < rrt_cost[x]) {
                        for(size_t i = 0; i < rrt_graph[rrt_parent[x]].size(); i++) 
                            if(rrt_graph[rrt_parent[x]][i] == x)
                                rrt_graph[rrt_parent[x]].erase(rrt_graph[rrt_parent[x]].begin()+i);
                    rrt_parent[x] = x_new;
                    rrt_graph[x_new].push_back(x); 
		    rrt_graph[x].push_back(x_new);
		    rrt_cost[x] = rrt_cost[x_new] + (pos_map[x]-pos_map[x_new]).norm();
                }
            } 
  	    	    
            //visualization::DrawPoint(x_rand, 0x000000, global_viz_msg_);
	    //viz_pub_.publish(global_viz_msg_);
	}
    }
    // done
}

void vis_graph() {
    visualization::ClearVisualizationMsg(local_viz_msg_); 
    visualization::ClearVisualizationMsg(global_viz_msg_); 
    
    for(pair<int, vector<int> >  p : rrt_graph) {
        //cout << "here" << endl;
        for(size_t i = 0; i < p.second.size(); i++) {
	cout <<"HI " << p.first <<  " "  << i << " " << pos_map[p.first] << " " << pos_map[p.second[i]] <<  endl;


     visualization::DrawLine(pos_map[p.first], pos_map[p.second[i]], 0x000000, global_viz_msg_);
    viz_pub_.publish(global_viz_msg_);
   }

    }    
}


void vis_path(vector<Vector2f> path){
    cout <<"path size : " << path.size() << endl;
    float cost = 0.0;
    for(size_t index = 1; index < path.size(); index++){
    cout << "cost! : " << rrt_iterations << " " << cost << endl;
        visualization::DrawLine(path[index], path[index-1], 0xFF0000, global_viz_msg_);
	cost += (path[index]-path[index-1]).norm();
	
    }
    cout << "graph: " << rrt_iterations << " " << cost;
    viz_pub_.publish(global_viz_msg_);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    //if(curr_pos_set){
    // rrt_iterations += 100;
    nav_goal_loc_ = loc;
    //goal_loc_set = true;
    rrt_star();
    cout << rrt_graph.size() << endl;
    vector<Vector2f> path = A_star(curr_pos, nav_goal_loc_, map_);
    vis_graph();
    vis_path(path);
    //cout << "Here!" << curr_pos << endl;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
    //cout << "Location: " << loc << endl;
    curr_pos = Vector2f(loc.x(), loc.y());
    curr_angle = angle;
    // rrt_star();
    //vis_graph();
    //A_star(curr_pos, goal_loc);
    //vis_path();
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
    odom_vel = vel.norm();
    curr_speed = odom_vel;
}





void Navigation::Run() {
    // rrt_graph();
    // A_star();
    
}

}  // namespace navigation


