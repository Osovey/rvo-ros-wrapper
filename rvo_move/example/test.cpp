#include <ros/ros.h>
#include <vector>
#include <cstring>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_node");  
    ros::NodeHandle nh;

    int n;
    nh.param<int>("robot_num", n, 16);
    
    std::vector<ros::Publisher> goal_publishers;

    std::string prefix = "robot_";
    std::string topic_name = "/goal";
    
    for (int i = 0; i < n; i++) {
        std::ostringstream topic;
        topic << prefix << i << topic_name;
        goal_publishers.push_back(
            nh.advertise<geometry_msgs::PoseStamped>(topic.str(), 10));
    }

    ros::Duration(1).sleep();
    
    int group_num = 4;
    int group_size = 4;

    std::vector<std::pair<float, float>> goals{
        {2.21, 0.15}, 
        {0.63, -2.7},
        {-1.72, -0.02},
        {0.25, 2.75}
        };

    for (int i = 0; i < group_num; i++) {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "/map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = goals[i].first;
        goal.pose.position.y = goals[i].second;
        for (int j = 0; j < group_size; j++) {
            goal_publishers[i * group_size + j].publish(goal);
        }
    }
    
    return 0;
}
