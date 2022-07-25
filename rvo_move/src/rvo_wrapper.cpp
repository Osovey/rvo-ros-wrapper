#include "rvo_wrapper.hpp"
#include <fstream>
#include <tf/tf.h>

#include <angles/angles.h>

#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
//================================ Utilities ================================//

RVO::Vector2 pose_to_rvo(const geometry_msgs::Pose& p) {
  return RVO::Vector2(p.position.x, p.position.y);
}

bool quat_angle(const geometry_msgs::Quaternion& msg, double *angle) {
  tf::Quaternion q = tf::Quaternion(msg.x, msg.y, msg.z, msg.w);
  if (fabs(q.length2() - 1) > 1e-4) {
    ROS_DEBUG_THROTTLE(1.0, "Quaternion not properly normalized");
    return false;
  } else {
    *angle = tf::getYaw(q);
    return true;
  }
}

RVO::Vector2 odom_to_rvo(const nav_msgs::Odometry& odom,
                         const geometry_msgs::Pose& p) {
  double vx = odom.twist.twist.linear.x;
  double vw = odom.twist.twist.linear.z;

  double theta;
  if (!quat_angle(p.orientation, &theta)) {
    ROS_DEBUG_THROTTLE(5.0, "odom_to_rvo()");
    return RVO::Vector2(0.0, 0.0);
  }
  // Rotate to get velocity in world frame
  double xdot = vx * cos(theta) - vw * sin(theta);
  double ydot = vx * sin(theta) + vw * cos(theta);
  return RVO::Vector2(xdot, ydot);
}

geometry_msgs::Twist vel_to_twist(const RVO::Vector2 vel) {
  geometry_msgs::Twist t;
  t.linear.x = vel.x();
  t.angular.z = vel.y();
  return t;
}

RVO::Vector2 eig_to_rvo(const Eigen::Vector2f& vec) {
  return RVO::Vector2(vec(0), vec(1));
}

Eigen::Vector2f rvo_to_eig(const RVO::Vector2& vec) {
  return Eigen::Vector2f(vec.x(), vec.y());
}

//================================= Wrapper =================================//
namespace rvo {
  RVOWrapper::RVOWrapper(vector<BotClient *> bots, size_t id, OccupancyMap *map,
                         const vector<vector<RVO::Vector2> > &obstacles) :
    bots_(bots), map_(map), id_(id), axel_width_(0.23), goal_tol_(0.1),
    path_margin_(0.2), timestep_(0.1), los_margin_(0.1), max_speed_(0.3) {
    if (id >= bots_.size()) {
      stringstream ss;
      ss << "RVOWrapper::RVOWrapper(): Bot id=" << id << " >= number of bots=" << bots_.size();
      throw std::runtime_error(ss.str());
    }

    RVO::Vector2 velocity(0.0, 0.0);
    float neighborDist = 3.0, timeHorizon = 2.0, timeHorizonObst = 2.0;
    float radius = 0.25;
    size_t maxNeighbors = 10;
    namespace_ = nh_.getNamespace();
    sim_ = new RVO::RVOSimulator();
    sim_->setAgentDefaults(neighborDist, maxNeighbors, timeHorizon,
                           timeHorizonObst, radius, max_speed_, velocity);
    sim_->setTimeStep(timestep_);

    vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_array", 10, true);
    vis_pub_throt_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_array_throttle", 10, true);

    // TODO: RVO should be able to use occupancy grid information.

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.header.stamp = ros::Time();
    m.header.frame_id = "/map";
    m.id = 1;
    m.ns = namespace_ + "/rvo";
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.2;
    m.color.r = 1.0;
    m.color.a = 1.0;
    m.points.resize(0);
    for (size_t i = 0; i < obstacles.size(); ++i) {
      const vector<RVO::Vector2> &obs = obstacles[i];
      m.points.clear();
      m.points.resize(obs.size() + 1);
      for (size_t j = 0; j < obs.size(); ++j) {
        m.points[j].x = obs[j].x();
        m.points[j].y = obs[j].y();
      }
      m.points.back().x = obs[0].x();
      m.points.back().y = obs[0].y();

      sim_->addObstacle(obs);
      ma.markers.push_back(m);
      m.id++;
    }
    vis_pub_.publish(ma);
    vis_pub_throt_.publish(ma);
    sim_->processObstacles();

  }

  RVOWrapper::~RVOWrapper() {
    delete sim_;
  }

  RVOWrapper* RVOWrapper::ROSInit(const ros::NodeHandle& nh, OccupancyMap *map, vector<BotClient*> bots) {
    // Setup simulation
    int id;
    double neighborDist, timeHorizon, timeHorizonObst, radius, maxSpeed, los_margin;
    double axel_width, waypoint_spacing, path_margin, timestep, goal_tol;
    int maxNeighbors;
    nh.param("neighborDist", neighborDist, 3.0);
    nh.param("maxNeighbors", maxNeighbors, 10);
    nh.param("timeHorizon", timeHorizon, 2.0);
    nh.param("timeHorizonObst", timeHorizonObst, 2.0);
    nh.param("radius", radius, 0.05);
    nh.param("maxSpeed", maxSpeed, 3.0);
    nh.param("id", id, -1);
    nh.param("timestep", timestep, 0.1);
    nh.param("axel_width", axel_width, 0.23);
    nh.param("waypoint_spacing", waypoint_spacing, 0.75);
    nh.param("path_margin", path_margin, 0.1);
    nh.param("goal_tolerance", goal_tol, 0.1);
    nh.param("los_margin", los_margin, 0.1);

    string my_namespace = getenv("ROS_NAMESPACE");
    if (id == -1) {
      for (size_t i = 0; i < bots.size(); ++i) {
        if (my_namespace == bots[i]->getName()) {
          ROS_INFO("My id is %zu", i);
          id = i;
          break;
        }
      }
      if (id == -1) {
        ROS_ERROR("Coudln't find namespace among robots and id param not set");
        ROS_ERROR("my_namespace = %s", my_namespace.c_str());
        ROS_BREAK();
      }
    }
    vector<vector<RVO::Vector2> > obstacles;
    if (nh.hasParam("obstacle_file")) {
      std::string fname;
      nh.getParam("obstacle_file", fname);
      ifstream ifs(fname.c_str(), ios::in);
      std::string line;
      vector<RVO::Vector2> obstacle;

      while (getline(ifs, line)) {
        if (line == "===") {
          if (obstacle.size() > 0) {
            ROS_DEBUG("Adding obstacle with %zu vertices", obstacle.size());
            obstacles.push_back(obstacle);
            obstacle.clear();
          }
        } else {
          float x, y;
          sscanf(line.c_str(), "%f %f", &x, &y);
          obstacle.push_back(RVO::Vector2(x, y));
        }
      }
    }
    if (obstacles.size() == 0) {
      ROS_WARN("NO OBSTACLES LOADED");
    } else {
      ROS_INFO("Loaded %zu obstacles", obstacles.size());
    }
    RVOWrapper *wrapper = new RVOWrapper(bots, id, map, obstacles);
    wrapper->setAgentDefaults(neighborDist, maxNeighbors, timeHorizon,
                              timeHorizonObst, radius, maxSpeed);
    wrapper->setTimestep(timestep);
    wrapper->setAxelWidth(axel_width);
    wrapper->setWaypointSpacing(waypoint_spacing);
    wrapper->setPathMargin(path_margin);
    wrapper->setGoalTolerance(goal_tol);
    wrapper->setMaxSpeed(maxSpeed);
    wrapper->addAgents();
    return wrapper;
  }

  void RVOWrapper::setAgentDefaults(float neighborDist, size_t maxNeighbors,
                                    float timeHorizon, float timeHorizonObst,
                                    float radius, float maxSpeed) {
    sim_->setAgentDefaults(neighborDist, maxNeighbors, timeHorizon,
                           timeHorizonObst, radius, maxSpeed);
  }


  void RVOWrapper::addAgents() {
    for (size_t i = 0; i < bots_.size(); ++i) {
      sim_->addAgent(RVO::Vector2(-1000, -1000));
    }
  }

  bool RVOWrapper::step() {
    // TODO: synchornize ROS clock and sim clock.
    sim_->doStep();
    RVO::Vector2 vel = sim_->getAgentVelocity(id_);
    // ROS_INFO("%s's actual xy_vel:    % 6.2f % 6.2f",
    //          bots_[id_]->getName().c_str(), vel.x(), vel.y());

    // ROS_INFO("Sim vel: % 6.2f % 6.2f", vel.x(), vel.y());

    double theta;
    if (!quat_angle(bots_[id_]->getPose().orientation, &theta)) {
      ROS_WARN_THROTTLE(5.0, "Problem with quaternion in step()");
      return false;
    }
    double desired_theta = atan2(vel.y(), vel.x());
    bool at_dest = RVO::abs(sim_->getAgentPosition(id_) - goal_) < goal_tol_;
    double vx;
    double vw;

    double norm_theta_diff = angles::normalize_angle(desired_theta - theta);
    bool straight_forward = fabs(norm_theta_diff) < M_PI / 8.0;
    bool straight_backward = fabs(norm_theta_diff) > M_PI - M_PI/16;
    if (at_dest) {
      // We're here, stop
      vx = 0.0;
      vw = 0.0;
    } else if (!straight_forward && !straight_backward) {
      // We're not oriented well, rotate to correct that
      vx = 0.0;
      vw = copysign(0.6, norm_theta_diff);
    } else {
      // We're correctly oriented and not at the goal, let RVO do its thing
      //
      // Solve for control inputs for standard kinematic model with feedback
      // linearization; see (eqn 2) in "Smooth and Collision-Free Navigation
      // for Multiple Robots Under Differential-Drive Constraints"
      double ct = cos(theta);
      double st = sin(theta);
      double L = sim_->getAgentRadius(id_) / 2.0;
      Eigen::Matrix2d m;
      m(0, 0) = ct / 2.0 + axel_width_ * st / L;
      m(0, 1) = ct / 2.0 - axel_width_ * st / L;
      m(1, 0) = st / 2.0 - axel_width_ * ct / L;
      m(1, 1) = st / 2.0 + axel_width_ * ct / L;

      Eigen::Vector2d v(vel.x(), vel.y());
      Eigen::Vector2d u = m.inverse() * v;
      // ROS_DEBUG("L: % 6.2f", L);
      // ROS_DEBUG_STREAM("m: " << m);
      // ROS_DEBUG_STREAM("m_inv: " << m.inverse());
      // ROS_DEBUG_STREAM("vel:       " << setprecision(2) << v.transpose());
      // ROS_DEBUG_STREAM("wheel vel: " << setprecision(2) << u.transpose());

      vx = (u(0) + u(1)) / 2.0;
      vw = (u(1) - u(0)) / L;
      vx = min(max(vx, -max_speed_), max_speed_);
      vw = min(max(vw, -max_speed_), max_speed_);
    }

    bots_[id_]->pubVel(vx, vw);
    // ROS_DEBUG("Theta: % 6.2f", theta);
    // ROS_DEBUG("sim pose:  % 6.2f % 6.2f", sim_->getAgentPosition(id_).x(),
    //          sim_->getAgentPosition(id_).y());
    // ROS_DEBUG("command:   % 6.2f % 6.2f\n", vx, vw);

    return at_dest;
  }

  std::vector<geometry_msgs::Pose> RVOWrapper::setGoal(const geometry_msgs::Pose& p) {
    RVO::Vector2 start = pose_to_rvo(bots_[id_]->getPose());
    goal_ = pose_to_rvo(p);
    waypoints_.clear();

    PointVector path;
    double margin = path_margin_;
    while (true) {
      path = map_->astar(start.x(), start.y(), goal_.x(), goal_.y(), margin);
      if (path.size() != 0 || margin < 0.05) {
        break;
      } else {
        ROS_DEBUG("Shrinking path");
        margin *= 0.9;
      }
    }

    if (path.size() != 0) {
      waypoints_.push_back(eig_to_rvo(path[0]));
      for (size_t i = 0; i < path.size() - 1; ++i) {
        const RVO::Vector2& prev = waypoints_.back();
        const RVO::Vector2& curr = eig_to_rvo(path[i]);
        if (RVO::abs(prev - curr) > way_spacing_) {
          waypoints_.push_back(eig_to_rvo(path[i-1]));
        }
      }
      waypoints_.push_back(eig_to_rvo(path.back()));
      ROS_DEBUG("Waypoint path has %zu vertices", waypoints_.size());
    }

    std::vector<geometry_msgs::Pose> ros_path;
    ros_path.resize(waypoints_.size());
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      tf::poseTFToMsg(tf::Pose(tf::createIdentityQuaternion(),
                               tf::Vector3(waypoints_[i].x(), waypoints_[i].y(), 0)),
                      ros_path[i]);
    }
    return ros_path;
  }

  bool RVOWrapper::getLeadGoal(RVO::Vector2 *goal) {
    // Direct robot towards successor of point that it is closest to
    float min_dist = std::numeric_limits<float>::infinity();
    int min_ind = -1;
    // Get closest visible waypoint
    RVO::Vector2 pos =  sim_->getAgentPosition(id_);
    for (size_t wayind = 0; wayind < waypoints_.size(); ++wayind) {
      if (!sim_->queryVisibility(pos, waypoints_[wayind], los_margin_)) {
        continue;
      }
      float dist = RVO::absSq(pos - waypoints_[wayind]);
      if (dist < min_dist) {
        min_dist = dist;
        min_ind = wayind;
      }
    }

    if (min_ind == -1) {
      ROS_WARN("No waypoints are visible!");
      *goal = RVO::Vector2();
      return false;
    } else {
      int orig_min = min_ind;
      // Find waypoint furthest along the path that robot can see
      while (static_cast<unsigned>(min_ind + 1) < waypoints_.size() &&
             sim_->queryVisibility(pos, waypoints_[min_ind+1], los_margin_) &&
             sqrt(RVO::absSq(waypoints_[orig_min] - waypoints_[min_ind+1])) < 1.0) {
        ++min_ind;
      }
      if (!sim_->queryVisibility(pos, waypoints_[min_ind])) {
        ROS_WARN("RVOWrapper::getLeadGoal() %s's next waypoint is not visible",
                 bots_[id_]->getName().c_str());
      }

      *goal = RVO::normalize(waypoints_[min_ind] - pos);
      // if (RVO::absSq(*goal) > 1.0f) {
      //   *goal = RVO::normalize(*goal);
      // }

      visualization_msgs::MarkerArray ma;
      ma.markers.resize(1);
      visualization_msgs::Marker &m = ma.markers.at(0);
      m.header.stamp = ros::Time();
      m.header.frame_id = "/map";
      m.action = visualization_msgs::Marker::ADD;
      m.type = visualization_msgs::Marker::SPHERE;
      m.id = 100;
      m.ns = namespace_ + "/rvo";
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.1;
      m.color.a = 1.0;
      m.color.r = 1.0;
      m.pose.position.x = waypoints_[min_ind].x();
      m.pose.position.y = waypoints_[min_ind].y();
      vis_pub_.publish(ma);

      // ROS_DEBUG("Advancing towards goal");
      // ROS_DEBUG("occ dist: % 6.2f",
      //           map_get_cell(map_, pos.x(), pos.y(), 0.0)->occ_dist);
      // ROS_DEBUG("min_ind: %i", min_ind);
      // ROS_DEBUG("position:   % 6.2f % 6.2f", pos.x(), pos.y());
      // ROS_DEBUG("waypoint:   % 6.2f % 6.2f",
      //           waypoints_[min_ind].x(), waypoints_[min_ind].y());
      // ROS_DEBUG("%s's actual goalVec: % 6.2f % 6.2f",
      //           bots_[id_]->getName().c_str(), goal->x(), goal->y());
      // ROS_DEBUG("Num obs: %zu", sim_->getAgentNumObstacleNeighbors(id_));
      return true;
    }
  }

  bool RVOWrapper::syncState() {
    // Update RVO simulator with latest position & velocitys from ROS
    bool have_everything = true;
    ros::Duration d(60.0);

    for (size_t i = 0; i < sim_->getNumAgents(); ++i) {
      BotClient *bot = bots_[i];
      if (bot->havePose(d)) {
        RVO::Vector2 position = pose_to_rvo(bot->getPose());
        sim_->setAgentPosition(i, position);
      } else {
        ROS_WARN_THROTTLE(5.0, "No pose info for %s (id: %zu)",
                          bot->getName().c_str(), i);
        have_everything = false;
      }

      if (bot->haveOdom(d)) {
        RVO::Vector2 vel = odom_to_rvo(bot->getOdom(), bot->getPose());
        sim_->setAgentVelocity(i, vel);
      } else {
        ROS_WARN_THROTTLE(5.0, "No odom info for %s (id: %zu)",
                          bot->getName().c_str(), i);
        have_everything = false;
      }
    }

    return have_everything;
  }

  bool RVOWrapper::setVelocities() {
    // Set preferred velocities
    bool ok = true;
    for (size_t i = 0; i < sim_->getNumAgents(); ++i) {
      if (i == id_) {
        RVO::Vector2 goalVector;
        ok = getLeadGoal(&goalVector);
        if (!ok) {
          ROS_WARN("%s problem getting goal", bots_[id_]->getName().c_str());
        } else {
          // Perturb a little to avoid deadlocks due to perfect symmetry.
          float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
          float dist = std::rand() * 0.0001f / RAND_MAX;
          sim_->setAgentPrefVelocity(i, goalVector +
                                     dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
        }
      } else {
        BotClient *bot = bots_[i];
        RVO::Vector2 goal = odom_to_rvo(bot->getOdom(), bot->getPose());
        // ROS_DEBUG("%s thinks %s's xy_vel:  % 6.2f % 6.2f", bots_[id_]->getName().c_str(),
        //           bot->getName().c_str(), goal.x(), goal.y());
        sim_->setAgentPrefVelocity(i, goal);
      }
    }
    return ok;
  }

  //============================== MoveServer ===============================//

  MoveServer::MoveServer(const std::string &server_name) :
    pnh_("~"), as_(nh_, server_name, boost::bind(&MoveServer::executeCB, this, _1), false),
    action_name_(ros::names::resolve(server_name)) {

    ROS_INFO("Setting up action server '%s'", action_name_.c_str());
    // Get the map
    OccupancyMap *map = OccupancyMap::FromMapServer("/static_map");
    map->updateCSpace(1.0);
    
    // Get bots
    namespace_ = nh_.getNamespace();
    bots_ = BotClient::MakeBots(pnh_);
    ROS_INFO("Listening to %zu bots", bots_.size());
    for (size_t i = 0; i < bots_.size(); ++i) {
      ROS_INFO("  Bot %zu: %s", i, bots_[i]->getName().c_str());
    }

    wrapper_ = RVOWrapper::ROSInit(pnh_, map, bots_);
    pnh_.param("timestep", timestep_, 0.1);
    wrapper_->setTimestep(timestep_);

    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 5, true);
    pnh_.param("map_frame_id", tf_frame_, std::string("/map"));
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = tf_frame_;
    path.poses.resize(0);
    path_pub_.publish(path);
    ROS_INFO("Done setting up %s",  action_name_.c_str());
  }

  MoveServer::~MoveServer() {
    delete wrapper_;
    for (size_t i = 0; i < bots_.size(); ++i) {
      delete bots_[i];
    }
  }

  void MoveServer::executeCB(const rvo_move::MoveGoalConstPtr &goal) {
    std::string prefix = action_name_;
    const char *pref = prefix.c_str();
    ROS_INFO("%s got request: (%.2f, %.2f)", pref,
             goal->target_pose.pose.position.x,
             goal->target_pose.pose.position.y);

    std::vector<geometry_msgs::Pose> path = wrapper_->setGoal(goal->target_pose.pose);
    // Publish the path
    nav_msgs::Path nav_path;
    nav_path.header.stamp = ros::Time::now();
    nav_path.header.frame_id = tf_frame_;
    nav_path.poses.resize(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
      nav_path.poses[i].header.stamp = ros::Time::now();
      nav_path.poses[i].header.frame_id = tf_frame_;
      nav_path.poses[i].pose = path[i];
    }
    path_pub_.publish(nav_path);

    if (path.size() == 0) {
      ROS_WARN("No path found");
      bots_[wrapper_->getID()]->pubVel(0.0, 0.0);
      as_.setAborted();
      return;
    }
    for (ros::Rate rate(1.0 / timestep_); ; rate.sleep()) {
      if (!ros::ok()) {
        ROS_INFO("%s Ros shutdown", action_name_.c_str());
        as_.setPreempted();
        break;
      }

      if (as_.isPreemptRequested()) {
        ROS_INFO("%s Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        break;
      }

      if (!wrapper_->syncState()) {
        ROS_WARN_THROTTLE(5.0, "%s Problem synchronizing state", pref);
      }

      if (!wrapper_->setVelocities()) {
        ROS_WARN("%s Problem while setting agent velocities", pref);
        as_.setAborted();
        break;
      }

      if (wrapper_->step()) {
        ROS_INFO("%s Reached destination", pref);
        as_.setSucceeded();
        break;
      }
    }
    nav_path.poses.resize(0);
    path_pub_.publish(nav_path);
    if (!as_.isNewGoalAvailable()) {
      bots_[wrapper_->getID()]->pubVel(0.0, 0.0);
    }
  }

  void MoveServer::start() {
    as_.start();
  }
}
