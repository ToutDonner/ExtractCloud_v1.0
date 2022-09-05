#include <ros/ros.h>

#include <thread>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <std_msgs/Bool.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>

#include <livox_ros_driver/CustomMsg.h>
#include <unistd.h>

typedef pcl::PointXYZI PointType;
using namespace std;

Eigen::Quaterniond q_f_to_i;
Eigen::Vector3d t_f_to_i;

pcl::PointCloud<PointType>::Ptr tmp_scan;
pcl::PointCloud<PointType>::Ptr pointcloud;

vector<pcl::PointCloud<PointType>::Ptr> pointcloud_vec(10);
vector<nav_msgs::Odometry> pose_vec(10);

nav_msgs::Odometry tmp_odom;

double time_new_scan;
double time_new_odom;

int frame_num = 0;
int if_acc = 0;

/**
 * @param extract_num 该参数为积累点云帧数量，extract_num +1 即为累计点云的数量，每帧0.1s，该参数可根据需要自行修改
 */
int extract_num = 4;

bool new_scan = false;
bool new_odom = false;
bool if_acc_pc = false;
bool if_pub = false;
bool if_enough_pc = false;
bool if_clear = false;
/**
 * @brief 处理雷达的回调函数，将livox_ros_driver::CustoMsg转化为pointcloud2
 * @details 使用此消息类型需要在workspace下编译livox_ros_driver
 *
 * @param msgs
 */
void livox_handler(const livox_ros_driver::CustomMsg::ConstPtr &msgs)
{
    tmp_scan.reset(new pcl::PointCloud<PointType>());
    tmp_scan->clear();
    tmp_scan->resize(msgs->points.size());
    ROS_INFO("cloud in");
    if (new_odom)
    {
        int num = msgs->points.size();
        for (int i = 0; i < num; i++)
        {
            PointType pt;
            pt.x = msgs->points[i].x;
            pt.y = msgs->points[i].y;
            pt.z = msgs->points[i].z;
            pt.intensity = msgs->points[i].reflectivity;
            tmp_scan->push_back(pt);
        }
        // new_scan = true;
        // ROS_INFO("receive scan");

        while (!if_clear)
        {
            pointcloud_vec.clear();
            pose_vec.clear();
            if_clear = true;
        }

        if (if_acc_pc)
        {
            if (frame_num < (extract_num + 1))
            {

                pointcloud_vec.push_back(tmp_scan);
                pose_vec.push_back(tmp_odom);

                frame_num++;
                cout << "frame_num:" << frame_num << endl;
            }
            else
            {
                ROS_WARN("enough points");
                if_enough_pc = true;
                if_acc_pc = false;
                // if_clear = false;
            }
        }
        else
            ROS_WARN("waiting for command");
    }
}

void cloud_handler(const sensor_msgs::PointCloud2::ConstPtr &msgs)
{
    tmp_scan.reset(new pcl::PointCloud<PointType>());
    tmp_scan->clear();
    pcl::fromROSMsg(*msgs, *tmp_scan);
    ROS_INFO("cloud in");

    // new_scan = true;
    // ROS_INFO("receive scan");

    while (!if_clear)
    {
        pointcloud_vec.clear();
        pose_vec.clear();
        if_clear = true;
    }

    if (if_acc_pc)
    {
        if (frame_num < (extract_num + 1))
        {

            pointcloud_vec.push_back(tmp_scan);
            pose_vec.push_back(tmp_odom);

            frame_num++;
            cout << "frame_num:" << frame_num << endl;
        }
        else
        {
            ROS_WARN("enough points");
            if_enough_pc = true;
            if_acc_pc = false;
            // if_clear = false;
        }
    }
    else
        ROS_WARN("waiting for command");
}

void odom_handler(const nav_msgs::OdometryConstPtr &odom_in) // recive Odom from RTK or SLAM
{
    time_new_odom = odom_in->header.stamp.toSec();
    tmp_odom = *odom_in;
    new_odom = true;
    ROS_INFO("receive odom");
}

/**
 * @brief 处理点云，将累计的点云转换到最后时刻里程计/GPS坐标系下
 *
 */
void process_pointcloud()
{

    pointcloud.reset(new pcl::PointCloud<PointType>());
    pointcloud->clear();
    Eigen::Quaterniond q_final = Eigen::Quaterniond::Identity();
    Eigen::Vector3d t_final = Eigen::Vector3d::Identity();

    q_final.w() = pose_vec[extract_num].pose.pose.orientation.w;
    q_final.x() = pose_vec[extract_num].pose.pose.orientation.x;
    q_final.y() = pose_vec[extract_num].pose.pose.orientation.y;
    q_final.z() = pose_vec[extract_num].pose.pose.orientation.z;

    t_final.x() = pose_vec[extract_num].pose.pose.position.x;
    t_final.y() = pose_vec[extract_num].pose.pose.position.y;
    t_final.z() = pose_vec[extract_num].pose.pose.position.z;

    int vec_9_size = pointcloud_vec[extract_num]->points.size();
    for (int num = 0; num < vec_9_size; num++)
    {
        PointType OD;
        OD.x = pointcloud_vec[extract_num]->points[num].x;
        OD.y = pointcloud_vec[extract_num]->points[num].y;
        OD.z = pointcloud_vec[extract_num]->points[num].z;
        OD.intensity = pointcloud_vec[extract_num]->points[num].intensity;
        pointcloud->push_back(OD);
    }

    for (int i = 0; i < extract_num; i++)
    {
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        Eigen::Vector3d t = Eigen::Vector3d::Identity();

        q.w() = pose_vec[i].pose.pose.orientation.w;
        q.x() = pose_vec[i].pose.pose.orientation.x;
        q.y() = pose_vec[i].pose.pose.orientation.y;
        q.z() = pose_vec[i].pose.pose.orientation.z;

        t.x() = pose_vec[i].pose.pose.position.x;
        t.y() = pose_vec[i].pose.pose.position.y;
        t.z() = pose_vec[i].pose.pose.position.z;

        q_f_to_i = q_final * q.inverse();

        t_f_to_i = t_final - t;

        int ptSize = pointcloud_vec[i]->points.size();

        for (int ptnum = 0; ptnum < ptSize; ptnum++)
        {
            Eigen::Vector3d pt = Eigen::Vector3d::Identity();

            pt.x() = pointcloud_vec[i]->points[ptnum].x;
            pt.y() = pointcloud_vec[i]->points[ptnum].y;
            pt.z() = pointcloud_vec[i]->points[ptnum].z;

            pt = q_f_to_i.inverse() * pt - t_f_to_i;

            PointType tmp_pt;

            tmp_pt.x = pt.x();
            tmp_pt.y = pt.y();
            tmp_pt.z = pt.z();
            tmp_pt.intensity = pointcloud_vec[i]->points[ptnum].intensity;

            pointcloud->push_back(tmp_pt);
        }
    }

    cout << "the pointcloud num is %d" << pointcloud->points.size() << endl;
}
/**
 * @brief 接受累计点云指令
 *
 * @param msg2 类型-> std_msgs::Bool
 */
void receive_command(const std_msgs::Bool::ConstPtr &msg2)
{
    if (msg2->data)
    {
        if_acc_pc = true;
        // system("/home/crz/aaa.sh");
    }
    else
        ROS_WARN("no command");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extract_pc");
    ros::NodeHandle nh;

    ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("/Odometry", 10000, &odom_handler);

    // ros::Subscriber sub_livox = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10000, &livox_handler);
    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered_body", 10000, &cloud_handler);

    ros::Subscriber sub_command = nh.subscribe<std_msgs::Bool>("/command", 1, &receive_command);

    ros::Publisher pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/scan", 10000);

    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        if (if_enough_pc)
        {

            process_pointcloud();

            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*pointcloud, msg);

            msg.header.frame_id = "map";
            msg.header.stamp = ros::Time().fromSec(pose_vec[9].header.stamp.toSec());

            pub_pc.publish(msg);
            system("rosnode kill /laserMapping");

            // if_pub = false;
            if_clear = false;
            if_enough_pc = false;
            frame_num = 0;
        }
        else
            ROS_WARN("not enough pc");

        rate.sleep();
    }

    return 0;
}
