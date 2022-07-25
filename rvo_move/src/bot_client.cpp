#include <rvo_move/bot_client.hpp>

#include <player_map/rosmap.hpp>

#include <tf/tf.h>
#include <assert.h>

using namespace std;

namespace rvo {
  BotClient::BotClient(string prefix) :
    got_odom_(false), got_pose_(false) {
    name_ = prefix;
    odom_sub_ = nh_.subscribe("odom", 5, &BotClient::odomCallback, this);

    tf::StampedTransform transform;
    if (tf_listener_.waitForTransform("/map", name_ + "/odom", ros::Time(0), ros::Duration(0.5))) {
      tf_listener_.lookupTransform("/map", name_ + "/odom", ros::Time(0), transform);
      got_pose_ = true;
      pose_.pose.position.x = transform.getOrigin().x();
      pose_.pose.position.y = transform.getOrigin().y();
      pose_.pose.orientation.w = transform.getRotation().getW();
      pose_.pose.orientation.x = transform.getRotation().getX();
      pose_.pose.orientation.y = transform.getRotation().getY();
      pose_.pose.orientation.z = transform.getRotation().getZ();
      pose_.header.stamp = ros::Time::now();
    }
    assert(got_pose_);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  }

  void BotClient::odomCallback(const nav_msgs::Odometry &msg) {
    boost::mutex::scoped_lock lock(mutex_);
    got_odom_ = true;
    odom_ = msg;
    // ROS_WARN_STREAM_THROTTLE(2, "" << getName() << ": odom = " << msg.twist.twist.linear.x <<
    //                          ", " << msg.twist.twist.angular.z);
  }

  void BotClient::poseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    boost::mutex::scoped_lock lock(mutex_);
    got_pose_ = true;
    pose_.pose = msg.pose.pose;
    pose_.header = msg.header;
    // ROS_WARN_STREAM_THROTTLE(2, "" << getName() << ": pose = " <<
    //                          pose_.pose.position.x << ", " << pose_.pose.position.y);
  }

  nav_msgs::Odometry BotClient::getOdom() {
    boost::mutex::scoped_lock lock(mutex_);
    return odom_;
  }

  geometry_msgs::Pose BotClient::getPose() {
    boost::mutex::scoped_lock lock(mutex_);

    tf::StampedTransform transform;
    ros::Time now = ros::Time::now();
    
    bool succeed = false;
    if (tf_listener_.waitForTransform("/map", name_ + "/base_link", now, ros::Duration(0.5))) {
      succeed = true;
      tf_listener_.lookupTransform("/map", name_ + "/base_link", now, transform);
      pose_.pose.position.x = transform.getOrigin().x();
      pose_.pose.position.y = transform.getOrigin().y();
      pose_.pose.orientation.w = transform.getRotation().getW();
      pose_.pose.orientation.x = transform.getRotation().getX();
      pose_.pose.orientation.y = transform.getRotation().getY();
      pose_.pose.orientation.z = transform.getRotation().getZ();
      pose_.header.stamp = ros::Time::now();
    }
    
    assert(succeed);

    return pose_.pose;
  }

  bool BotClient::haveOdom() {
    boost::mutex::scoped_lock lock(mutex_);
    return got_odom_;
  }

  bool BotClient::haveOdom(const ros::Duration &d) {
    boost::mutex::scoped_lock lock(mutex_);
    return got_odom_ && (ros::Time::now() - odom_.header.stamp) < d;
  }

  bool BotClient::havePose() {
    boost::mutex::scoped_lock lock(mutex_);
    return got_pose_;
  }

  bool BotClient::havePose(const ros::Duration &d) {
    boost::mutex::scoped_lock lock(mutex_);
    return got_pose_ && (ros::Time::now() - pose_.header.stamp) < d;
  }

  vector<BotClient *> BotClient::MakeBots(const ros::NodeHandle& nh) {
    vector<BotClient *> bots;
    int nbots;
    nh.param("nbots", nbots, 0);
    // instantiate listeners
    for (int id = 0; id < nbots; id++) {
      stringstream ss;
      ss.str();
      ss << "/robot_" << id;
      bots.push_back(new BotClient(ss.str()));
    }
    return bots;
  }

  void BotClient::FreeBots(vector<BotClient *> bots) {
    for (size_t i = 0; i < bots.size(); ++i) {
      delete bots[i];
    }
  }

  void BotClient::pubVel(double v, double w) {
    geometry_msgs::Twist t;
    t.linear.x = v;
    t.angular.z = w;
    vel_pub_.publish(t);
  }
}
