#ifndef BOT_COMM_HPP
#define BOT_COMM_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <boost/thread/mutex.hpp>

#include "nav_msgs/Odometry.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

namespace rvo {
  class BotClient {
  public:
    BotClient(std::string prefix);
    
    void odomCallback(const nav_msgs::Odometry &msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
    void velCallback(const geometry_msgs::Twist &msg);

    bool haveOdom();
    bool haveOdom(const ros::Duration &d);
    bool havePose();
    bool havePose(const ros::Duration &d);
    
    std::string getName() const { return name_; }
    nav_msgs::Odometry getOdom();
    geometry_msgs::Pose getPose();

    void pubVel(double v, double w);
    
    static std::vector<BotClient *> MakeBots(const ros::NodeHandle& nh);
    static void FreeBots(std::vector<BotClient *> bots);
  protected:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_, odom_sub_, vel_sub_;
    ros::Publisher vel_pub_;
    bool got_odom_, got_pose_;
    std::string name_;
    tf::TransformListener tf_listener_;
    
    boost::mutex mutex_;

    geometry_msgs::PoseStamped pose_;
    nav_msgs::Odometry odom_;
    geometry_msgs::Twist vel_;
  };
}
#endif
