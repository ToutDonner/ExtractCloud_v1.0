#include <unistd.h>
#include <ros/ros.h>
#include <string.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class accumulatePointcloud
{
private:
    ros::NodeHandle nh;

    ros::Subscriber sub_command;

    bool if_receive_command = false;
    bool if_launch = false;

public:
    accumulatePointcloud()
    {
        sub_command = nh.subscribe<std_msgs::Bool>("/command", 100, &accumulatePointcloud::receiveCommand, this);
    }
    ~accumulatePointcloud() {}

    void receiveCommand(const std_msgs::Bool::ConstPtr &command)
    {
        if (command->data == true)
        {
            ROS_INFO("true");
            // execlp("roslaunch", "roslaunch", "fast_lio", "accu.launch");
            system("/home/crz/aaa.sh");
            // if_launch = true;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "accumulatePointcloud");
    accumulatePointcloud aP;

    ros::spin();
    return 0;
}
