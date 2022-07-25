#include <player_map/rosmap.hpp>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/scoped_ptr.hpp>

using namespace rvo;

int main(int argc, char **argv) {
  ros::init(argc, argv, "draw_paths");

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("/marker_array", 10, true);

  boost::scoped_ptr<OccupancyMap> map;
  map.reset(OccupancyMap::FromMapServer("/static_map"));
  map->updateCSpace(1.0);
  double start_x = 9.0;
  double start_y = 12.5;
  Eigen::Vector2f start(start_x, start_y);  
  double dest_x = 19.0;
  double dest_y = 23.0;

  visualization_msgs::MarkerArray ma;
  if (false) {
    ROS_INFO("Planning");
    PointVector path = map->astar(start_x, start_y, dest_x, dest_y,
                                  0.25);
    ROS_INFO("Done");

    ma.markers.resize(1);
    visualization_msgs::Marker &m = ma.markers.at(0);
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.header.stamp = ros::Time();
    m.header.frame_id = "/map";
    m.id = 1;
    m.ns = "/draw_paths";
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.05;
    m.color.r = 1.0;
    m.color.a = 1.0;
    m.points.resize(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
      m.points.at(i).x = path.at(i)(0);
      m.points.at(i).y = path.at(i)(1);
    }
  } else {
    ROS_INFO("Planning");
    const PointVector& endpoints = map->prepareShortestPaths(start_x, start_y,
                                                             6.0, 2.0, 0.3);
    ROS_INFO("Done; got %zu", endpoints.size());

    double sep = 2.0;
    double los_margin = 0.1;
    std::vector<int> inds;
    inds.push_back(0);
    for (size_t curr = 1; curr < endpoints.size(); ++curr) {
      // Check if within threshold to any point
      bool too_close = false;
      for (size_t prev = 0; prev < inds.size(); ++prev) {
        Eigen::Vector2f first, second;
        first = endpoints.at(curr);
        second = endpoints.at(inds.at(prev));
        double distance = (first - second).norm();
        if (distance < sep || (first - start).norm() < 4.0) {
          too_close = true;
          break;
        }
      }
      if (too_close) {
        continue;
      } else {
        inds.push_back(curr);
      }
    }
    ROS_INFO("inds.size() = %zu", inds.size());
    
    visualization_msgs::Marker base;
    base.action = visualization_msgs::Marker::ADD;
    base.type = visualization_msgs::Marker::LINE_STRIP;
    base.header.stamp = ros::Time();
    base.header.frame_id = "/map";
    base.id = 0;
    base.ns = "/draw_paths";
    base.pose.orientation.w = 1.0;
    base.scale.x = 0.05;
    base.color.r = 1.0;
    base.color.a = 1.0;

    ma.markers.resize(inds.size());    
    for (size_t i = 0; i < inds.size(); ++i) {
      visualization_msgs::Marker &m = ma.markers.at(i);
      m = base;
      m.id = i;
      PointVector path = map->buildShortestPath(inds.at(i));
      m.points.resize(path.size());
      for (size_t i = 0; i < path.size(); ++i) {
        m.points.at(i).x = path.at(i)(0);
        m.points.at(i).y = path.at(i)(1);
      }
    }
  }

  pub.publish(ma);
  ros::spin();
}
