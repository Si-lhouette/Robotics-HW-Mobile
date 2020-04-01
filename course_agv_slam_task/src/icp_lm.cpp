#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

typedef struct{
    std::vector<float> distances;
    std::vector<int> src_indices;
    std::vector<int> tar_indices;
} NeighBor;

class icp_lm{

public:

    icp_lm(ros::NodeHandle &n);
    ~icp_lm();
    ros::NodeHandle& n;

    // robot init states
    double robot_x;
    double robot_y;
    double robot_theta;
    // sensor states = robot_x_y_theta
    Vector3d sensor_sta;

    // max iterations
    int max_iter;
    // distance threshold for filter the matching points
    double dis_th;
    // tolerance to stop icp
    double tolerance;
    // if is the first scan, set as the map/target
    bool isFirstScan;
    // src point cloud matrix
    MatrixXd src_pc;
    // target point cloud matrix
    MatrixXd tar_pc;
    // min match_cnt
    int min_match;

    // main process
    void process(visualization_msgs::MarkerArray input);
    // landMarks to Eigen::Matrix
    Eigen::MatrixXd landMarksToMatrix(visualization_msgs::MarkerArray input);
    // fint the nearest points & filter
    NeighBor findNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar);
    // get the transform from two point sets in one iteration
    Eigen::Matrix3d getTransform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
    // calc 2D Euclidean distance
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
    // transform vector states to matrix form
    Eigen::Matrix3d staToMatrix(const Vector3d sta);

    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber landMark_sub;
    void publishResult(Matrix3d T);
 	tf::TransformBroadcaster odom_broadcaster;
 	ros::Publisher odom_pub;
};

icp_lm::~icp_lm()
{}

icp_lm::icp_lm(ros::NodeHandle& n):
    n(n)
{
    // get the params
	n.getParam("/icp_lm/robot_x", robot_x);
	n.getParam("/icp_lm/robot_y", robot_y);
	n.getParam("/icp_lm/robot_theta", robot_theta);
	sensor_sta << robot_x, robot_y, robot_theta;

    n.getParam("/icp_lm/max_iter", max_iter);
	n.getParam("/icp_lm/tolerance", tolerance);
	n.getParam("/icp_lm/dis_th", dis_th);
    n.getParam("/icp_lm/min_match", min_match);

    isFirstScan = true;
    landMark_sub = n.subscribe("/landMarks", 1, &icp_lm::process, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("icp_odom", 1);
}

void icp_lm::process(visualization_msgs::MarkerArray input)
{   
    cout<<"------Time:  "<<input.markers[0].header.stamp<<endl;
    
    double time_0 = (double)ros::Time::now().toSec();

    if(isFirstScan)
    {
        tar_pc = this->landMarksToMatrix(input);
        isFirstScan =false;
        return;
    }

    // init some variables
    Eigen::Matrix3d Transform_acc = Eigen::MatrixXd::Identity(3,3);

    // main LOOP
    for(int i=0; i<max_iter; i++)
    {
        // TODO: please code by yourself
    }

    tar_pc = this->landMarksToMatrix(input);

    this->publishResult(Transform_acc);

    double time_1 = (double)ros::Time::now().toSec();
    cout<<"time_cost:  "<<time_1-time_0<<endl;
}

Eigen::MatrixXd icp_lm::landMarksToMatrix(visualization_msgs::MarkerArray input)
{
    int markerSize = input.markers.size();
    cout<<markerSize<<" markers received !"<<endl;

    Eigen::MatrixXd pc = Eigen::MatrixXd::Ones(3, markerSize);

    for(int i=0; i<markerSize; i++)
    {
        pc(0,i) = input.markers[i].pose.position.x;
        pc(1,i) = input.markers[i].pose.position.y;
    }
    return pc;
}

NeighBor icp_lm::findNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
{
    // TODO: please code by yourself
}

Eigen::Matrix3d icp_lm::getTransform(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
{
    // TODO: please code by yourself
}

float icp_lm::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{
    // TODO: please code by yourself
}

Eigen::Matrix3d icp_lm::staToMatrix(Eigen::Vector3d sta)
{
	Matrix3d RT;
    RT << cos(sta(2)), -sin(sta(2)), sta(0),
          sin(sta(2)), cos(sta(2)),sta(1),
          0, 0, 1;
    return RT;
}

void icp_lm::publishResult(Eigen::Matrix3d T)
{	
    float delta_yaw = atan2(T(1,0), T(0,0));
    cout<<"sensor-delta-xyt: "<<T(0,2)<<" "<<T(1,2)<<" "<<delta_yaw<<endl;

    sensor_sta(0) = sensor_sta(0) + cos(sensor_sta(2))*T(0,2) - sin(sensor_sta(2))*T(1,2);
    sensor_sta(1) = sensor_sta(1) + sin(sensor_sta(2))*T(0,2) + cos(sensor_sta(2))*T(1,2);
    sensor_sta(2) = sensor_sta(2) + delta_yaw;

    cout<<"sensor-global: "<<sensor_sta.transpose()<<endl;

    // tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(sensor_sta(2));

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "world_base";
    odom_trans.child_frame_id = "icp_odom";

    odom_trans.transform.translation.x = sensor_sta(0);
    odom_trans.transform.translation.y = sensor_sta(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    // odom
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world_base";

    odom.pose.pose.position.x = sensor_sta(0);
    odom.pose.pose.position.y = sensor_sta(1);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_landMark");
    ros::NodeHandle n;

    icp_lm icp_lm_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    return 0;
}