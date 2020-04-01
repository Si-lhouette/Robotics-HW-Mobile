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

#include <math.h>
#include <geometry_msgs/Pose.h>

using namespace std;
using namespace Eigen;

// structure of the nearest neighbor 
typedef struct{
    std::vector<float> distances;
    std::vector<int> src_indices;
    std::vector<int> tar_indices;
} NeighBor;

class icp{

public:

    icp(ros::NodeHandle &n);
    ~icp();
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
    // laser range
    double laser_max;
    // if is the first scan, set as the map/target
    bool isFirstScan;
    // src point cloud matrix
    MatrixXd src_pc;
    // target point cloud matrix
    MatrixXd tar_pc;

    // ICP process function
    void process(sensor_msgs::LaserScan input);
    // transform the ros msg to Eigen Matrix
    Eigen::MatrixXd rosmsgToEigen(const sensor_msgs::LaserScan input);
    // fint the nearest points & filter
    NeighBor findNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar);
    // get the transform from two point sets in one iteration
    Eigen::Matrix3d getTransform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
    // calc 2D Euclidean distance
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
    // transform vector states to matrix form
    Eigen::Matrix3d staToMatrix(const Vector3d sta);

    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber laser_sub;
    void publishResult(Matrix3d T);
 	tf::TransformBroadcaster odom_broadcaster;
 	ros::Publisher odom_pub;
};

icp::~icp()
{}

icp::icp(ros::NodeHandle& n):
    n(n)
{	

	// get the params
	n.getParam("/myicp/robot_x", robot_x);
	n.getParam("/myicp/robot_y", robot_y);
	n.getParam("/myicp/robot_theta", robot_theta);
	sensor_sta << robot_x, robot_y, robot_theta;

	n.getParam("/myicp/max_iter", max_iter);
	n.getParam("/myicp/tolerance", tolerance);
	n.getParam("/myicp/dis_th", dis_th);
    n.getParam("/myicp/laser_max", laser_max);

    isFirstScan = true;
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &icp::process, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("icp_odom", 1);
}

void icp::process(sensor_msgs::LaserScan input)
{
    cout<<"------myseq:  "<<input.header.seq<<endl;

    // set the inital
    if(isFirstScan)
    {
        tar_pc = this->rosmsgToEigen(input);
        isFirstScan =false;
        return;
    }

    src_pc = rosmsgToEigen(input);
    MatrixXd src_copy = src_pc;
    cout<<src_pc.cols()<<endl;
    if(src_pc.cols() >= 100){
        cout<<"in1111"<<endl;
        double time_0 = (double)ros::Time::now().toSec();

        // init some variables
        Eigen::Matrix3d Transform_acc = Eigen::MatrixXd::Identity(3,3);

        // main LOOP
        Matrix2d R_init = Transform_acc.block(0, 0, 2, 2);
        Vector2d t_init = Transform_acc.block(0, 2, 2, 1);
        for(int i=0; i<max_iter; i++)
        {
            // TODO: please code by yourself
            cout<<"loop_"<<i<<endl;
            NeighBor neighbor = findNearest(src_pc, tar_pc);
            int N = neighbor.src_indices.size();
            MatrixXd src(3, N), tar(3, N);

            for(int j = 0; j < N; j++){
                src.col(j) = src_pc.col(neighbor.src_indices[j]);
                tar.col(j) = tar_pc.col(neighbor.tar_indices[j]);
            }

            Matrix3d T = getTransform(src, tar);
            Matrix2d R = T.block(0, 0, 2, 2);
            Vector2d t = T.block(0, 2, 2, 1);
            t = R * t_init + t;
            R = R * R_init;
            R_init = R; t_init = t;

            Transform_acc << R, t,
                             0, 0, 1;
            src_pc = Transform_acc * src_copy;

            double error = accumulate(neighbor.distances.begin(), neighbor.distances.end(), 0.0) / N;
            cout<<"e:"<<error<<endl;
            if (abs(error) <= tolerance)
                break;
        }

        tar_pc = this->rosmsgToEigen(input);

        this->publishResult(Transform_acc);

        double time_1 = (double)ros::Time::now().toSec();
        cout<<"time_cost:  "<<time_1-time_0<<endl;
    }
}

Eigen::MatrixXd icp::rosmsgToEigen(const sensor_msgs::LaserScan input)
{
    int total_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;

    Eigen::MatrixXd pc = Eigen::MatrixXd::Ones(3,total_num);

    float angle;
    for(int i=0; i<total_num; i++)
    {
        angle = input.angle_min + i * input.angle_increment;

        pc(0,i) = input.ranges[i] * std::cos(angle);
        pc(1,i) = input.ranges[i] * std::sin(angle);
        if(fabs(pc(0,i)) > laser_max || fabs(pc(1,i)) > laser_max ){
            i--;
            total_num--;
        }
    }
    if(total_num > 0)
        pc = pc.leftCols(total_num);
    else
        pc = pc.leftCols(1);

    return pc;
}

NeighBor icp::findNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
{
    // TODO: please code by yourself
    vector<float> dist;
    vector<int> src_ind, tar_ind;
    NeighBor res;
    MatrixXd src_coord(2, src.cols()), tar_coord(2, tar.cols()), temp_coord(2, tar.cols());
    VectorXd distance(tar.cols());
    VectorXd::Index minInd;
    double minNum;

    src_coord = src.topRows(2);
    tar_coord = tar.topRows(2);

    for(int i = 0; i < src_coord.cols(); i++){
        temp_coord = tar_coord;
        temp_coord.colwise() -= src_coord.col(i);
        distance = temp_coord.array().square().colwise().sum();
        minNum = distance.minCoeff(&minInd);

        if(minNum >= dis_th)
            continue;
        dist.push_back(sqrt(minNum));
        src_ind.push_back(i);
        tar_ind.push_back(minInd);
    }

    res.distances = dist;
    res.src_indices = src_ind;
    res.tar_indices = tar_ind;

    return res;
}

Eigen::Matrix3d icp::getTransform(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
{
    // TODO: please code by yourself
    Matrix2d W, U, V, R;
    MatrixXd src_coord(2, src.cols()), tar_coord(2, tar.cols());
    Vector2d src_mean, tar_mean, t;
    Matrix3d T;

    src_coord = src.topRows(2);
    tar_coord = tar.topRows(2);

    src_mean = src_coord.rowwise().mean();
    tar_mean = tar_coord.rowwise().mean();
    src_coord.colwise() -= src_mean;
    tar_coord.colwise() -= tar_mean;

    W = src_coord * tar_coord.transpose();
    JacobiSVD<Matrix2d> svd(W, ComputeThinU | ComputeThinV);
    U = svd.matrixU();
    V = svd.matrixV();

    R = V * U.transpose();
    t = tar_mean - R*src_mean;
    T << R, t,
         0, 0, 1;

    return T;
}

float icp::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{
    // TODO: please code by yourself
    float distance;
    distance = sqrt((pta-ptb).array().square().sum());
    return distance;
}

Eigen::Matrix3d icp::staToMatrix(Eigen::Vector3d sta)
{
	Matrix3d RT;
    RT << cos(sta(2)), -sin(sta(2)), sta(0),
          sin(sta(2)), cos(sta(2)),sta(1),
          0, 0, 1;
    return RT;
}

void icp::publishResult(Eigen::Matrix3d T)
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
    ros::init(argc, argv, "myicp");
    ros::NodeHandle n;

    icp icp_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    return 0;
}
