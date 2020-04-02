#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>


#include "std_msgs/Float64.h"


#include <cmath>

using namespace std;
using namespace Eigen;

// structure of the nearest neighbor 
typedef struct{
    std::vector<float> sqared_dis;
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

    // var for Wheel_v odometry
    double leftv = 0.0;
    double rightv = 0.0;
    double timelast;
    double dx = 0.0;
    double dy = 0.0;
    double dtheta = 0.0;
    double theta = 0.0;
    bool setzero = false;

    double rx = 1.0/0.08;
    double rw = (0.1+0.08/6)/0.08;

    // max iterations
    int max_iter;
    // distance threshold for filter the matching points
    double dis_th;
    // tolerance to stop icp
    double tolerance;
    // reject laser range
    double laser_max;
    // if is the first scan, set as the map/target
    bool isFirstScan;
    // src point cloud matrix this frame
    MatrixXd src_pc;
    // target point cloud matrix last frame
    MatrixXd tar_pc;
    
    //Wheel_v odometry
    void calcinitRT(Eigen::Matrix2d& R_all, Eigen::Vector2d& T_all);
    void leftv_callback(const std_msgs::Float64::ConstPtr& msg);
    void rightv_callback(const std_msgs::Float64::ConstPtr& msg);



    //build kd-tree
    void buildKdtree(Eigen::MatrixXd tar);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLast;
    // ICP process function
    void process(sensor_msgs::LaserScan input);
    // transform the ros msg to Eigen Matrix
    Eigen::MatrixXd rosmsgToEigen(const sensor_msgs::LaserScan input);


    // fint the nearest points & filter
    NeighBor findNearest(pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeLast, const Eigen::MatrixXd &src);
    // get the transform from two point sets in one iteration
    Eigen::Matrix3d getTransform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
    // calc 2D Euclidean distance
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
    // transform vector states to matrix form
    Eigen::Matrix3d staToMatrix(const Vector3d sta);
  
    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber laser_sub;
    ros::Subscriber leftv_sub;
    ros::Subscriber rightv_sub;
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
    n.getParam("/icp/robot_x", robot_x);
    n.getParam("/icp/robot_y", robot_y);
    n.getParam("/icp/robot_theta", robot_theta);
    sensor_sta << robot_x, robot_y, robot_theta;

    n.getParam("/icp/max_iter", max_iter);
    n.getParam("/icp/tolerance", tolerance);
    n.getParam("/icp/dis_th", dis_th);

    n.getParam("/icp/laser_max", laser_max);

    isFirstScan = true;
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &icp::process, this);
    leftv_sub = n.subscribe("/course_agv/left_wheel_velocity_controller/command", 1, &icp::leftv_callback, this);
    rightv_sub = n.subscribe("/course_agv/right_wheel_velocity_controller/command", 1, &icp::rightv_callback, this);

    odom_pub = n.advertise<nav_msgs::Odometry>("icp_odom", 1);

}

void icp::leftv_callback(const std_msgs::Float64::ConstPtr& msg){
    //cout<<"in left"<<endl;
    this->leftv = msg->data;
}
void icp::rightv_callback(const std_msgs::Float64::ConstPtr& msg){
    //cout<<"in right"<<endl;
    this->rightv = msg->data;

    double tnow = (double)ros::Time::now().toSec();
    double dt = tnow - this->timelast;
    //cout<<"dt:"<<dt<<endl;
    if(this->setzero){
        this->dx = 0;
        this->dy = 0;
        this->dtheta = 0;
        this->setzero = false;
    }

    double vx = (this->leftv + this->rightv)/(2.0*this->rx);
    double vw = (this->rightv - this->leftv)/(2.0*this->rw);
    //cout<<"v="<<vx<<", "<<vw<<endl;
    this->dx += vx*dt*cos(sensor_sta(2)+this->dtheta);
    this->dy += vx*dt*sin(sensor_sta(2)+this->dtheta);

    // this->dx += vx*dt*cos(theta + this->dtheta);
    // this->dy += vx*dt*sin(theta + this->dtheta);
    this->dtheta += vw*dt;

    this->timelast = tnow;
}

/* Wheel_v Odometry: record the dx dy dtheta between 2 frames */
void icp::calcinitRT(Eigen::Matrix2d& R_all, Eigen::Vector2d& T_all){
    //cout<<"in cal"<<endl;
    Vector3d sta;
    sta << this->dx, this->dy, this->dtheta;
    //cout<<"dxdydt:"<<sta.transpose()<<endl;
    Matrix3d RT = staToMatrix(sta);
    R_all = RT.block(0,0,2,2);
    T_all = RT.block(0,2,2,1);
    theta += this->dtheta;
    this->setzero = true;
}

/* Match Current Frame(src) to Last Frame(tar) */
void icp::process(sensor_msgs::LaserScan input)
{
    cout<<endl<<"------myseq:  "<<input.header.seq<<endl;


    // 1. If the first scan
    if(isFirstScan)
    {
        tar_pc = this->rosmsgToEigen(input);
        buildKdtree(tar_pc);

        // init the var used in Odometry
        this->timelast = (double)ros::Time::now().toSec();
        this->setzero = true;

        isFirstScan =false;
        return;
    }

    src_pc = this->rosmsgToEigen(input); // get current frame points

    // 2. Check the num of valiable points in src
    if(src_pc.cols() < 100){
        cout<<"Too Little points!!!!!!!!!"<<endl;
        return;
    }

    // 3. Prepare
    // make a copy
    MatrixXd src_pc_copy;
    src_pc_copy = src_pc;
    cout<<src_pc.cols()<<endl;

    // tic
    double time_0 = (double)ros::Time::now().toSec();

    // Init var for loop
    Eigen::Matrix3d Transform_acc = Eigen::MatrixXd::Identity(3,3);
    Eigen::Matrix2d R_all = Eigen::MatrixXd::Identity(2,2);
    Eigen::Vector2d T_all;
    T_all << 0.0,0.0;//T_all << 0.0,0.0,0.0; To my surprise the bug didin't crash!! i suffered from it
    
    double elast = 1000; //error of last time 


    // Use the wheel_v odometry to set the init tranform Matrix
    calcinitRT(R_all, T_all); //!! YOU CAN COMMENT IT, NOT REALLY USEFUL
    Transform_acc.block(0,0,2,2) = R_all;
    Transform_acc.block(0,2,2,1) = T_all;
    src_pc = Transform_acc*src_pc_copy;
    cout<<src_pc.cols()<<endl;



    // 4. Main LOOP for Match
    for(int i=0; i<max_iter; i++)
    {   
        //cout<<"loop_"<<i<<endl;

        //4.1. Find neareast correspond
        NeighBor s_t_near;
        s_t_near = findNearest(kdtreeLast.makeShared(), src_pc);
		int s_num = s_t_near.src_indices.size();
        
        MatrixXd tar_pcn;
        MatrixXd src_pcn;
        src_pcn = Eigen::MatrixXd::Constant(3, s_num, 1);
        tar_pcn = Eigen::MatrixXd::Constant(3, s_num, 1);
        for(int j = 0; j < s_num; j++){
            src_pcn.col(j) = src_pc.col(s_t_near.src_indices[j]);
            tar_pcn.col(j) = tar_pc.col(s_t_near.tar_indices[j]);
        }
		

        //4.2. Solve for RotationMatrix & translation vector
        MatrixXd T = Eigen::MatrixXd::Identity(3,3);
        T = getTransform(src_pcn, tar_pcn);
        Matrix2d R_12 = T.block(0,0,2,2);
        Vector2d T_12 = T.block(0,2,2,1);


        //4.3. Updata R&T
        T_all = R_12 * T_all + T_12;
        R_all = R_12 * R_all;
        Transform_acc.block(0,0,2,2) = R_all;
        Transform_acc.block(0,2,2,1) = T_all;
        src_pc = Transform_acc*src_pc_copy;


        //4.4. Cheak error tolerance
        double e = accumulate(s_t_near.sqared_dis.begin(), s_t_near.sqared_dis.end(), 0.0) / s_num;
        //cout<<"e:"<<e<<endl;
        if(e < tolerance || fabs((elast - e)/elast) < 0.0001){
            //cout<<"loop_"<<i<<endl;
            cout<<"e:"<<e<<endl;
            //break; //!!P.S. Compulsory Loop max_iteration For better performace. Problem not sloved here.
        }
        elast = e;
    }

    // 5. Update Last frame & build kd-tree
    tar_pc = src_pc_copy;
    buildKdtree(tar_pc);


    // 6. Publish
    Transform_acc.block(0,0,2,2) = R_all;
    Transform_acc.block(0,2,2,1) = T_all;
    this->publishResult(Transform_acc);

    //toc
    double time_1 = (double)ros::Time::now().toSec();
    cout<<"time_cost:  "<<time_1-time_0<<endl;
}


/* BuildKdtree: use pcl::KdTreeFLANN Class */
void icp::buildKdtree(Eigen::MatrixXd tar){
    pcl::PointCloud<pcl::PointXYZ>::Ptr tar_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for(int i = 0; i < tar.cols(); i++){
        pcl::PointXYZ tpoint;
        tpoint.x = tar(0,i);
        tpoint.y = tar(1,i);
        tpoint.z = 0.0;
        tar_ptr->push_back(tpoint);
    }
    this->kdtreeLast.setInputCloud(tar_ptr);
    cout<<"build kd done_"<<tar.cols()<<endl;

}

/* Find neareast correspond of 2 frame */
NeighBor icp::findNearest(pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtreeLast, const Eigen::MatrixXd &src)
{

    NeighBor s_t_near;
    std::vector<int> pointFindIndex;
    std::vector<float> pointFindDistance;
    //cout<<"src_num:"<<src.cols()<<endl;
    for(int i = 0; i < src.cols(); i++){
        pcl::PointXYZ tpoint;
        tpoint.x = src(0,i);
        tpoint.y = src(1,i);
        tpoint.z = 0.0;
        int len = 1;
        kdtreeLast->nearestKSearch(tpoint, len, pointFindIndex, pointFindDistance);
        //cout<<"serch done"<<endl;
        if(pointFindDistance[0] < dis_th*dis_th){
            s_t_near.src_indices.push_back(i);
            s_t_near.tar_indices.push_back(pointFindIndex[0]);
            s_t_near.sqared_dis.push_back(pointFindDistance[0]);        
        }

    }
    return s_t_near;
}

/* Solve for RotationMatrix & translation vector */
Eigen::Matrix3d icp::getTransform(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
{
	//copy input
    MatrixXd src_pcn = src;
    MatrixXd tar_pcn = tar;
    int s_num = src.cols();


    //Decentralization
    double tx_mean = tar_pcn.row(0).mean();
    double ty_mean = tar_pcn.row(1).mean();
    tar_pcn.row(0) -=  Eigen::MatrixXd::Constant(1, s_num, tx_mean);
    tar_pcn.row(1) -=  Eigen::MatrixXd::Constant(1, s_num, ty_mean);

    double sx_mean = src_pcn.row(0).mean();
    double sy_mean = src_pcn.row(1).mean();
    src_pcn.row(0) -=  Eigen::MatrixXd::Constant(1, s_num, sx_mean);
    src_pcn.row(1) -=  Eigen::MatrixXd::Constant(1, s_num, sy_mean); 

    MatrixXd W = Eigen::MatrixXd::Zero(2,2);
    W = src_pcn.topRows(2) * tar_pcn.topRows(2).transpose();


    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    Eigen::Matrix2d R_12 = V* (U.transpose());
    Eigen::Vector2d T_12 = Eigen::Vector2d(tx_mean, ty_mean) - R_12 * Eigen::Vector2d(sx_mean, sy_mean);

    MatrixXd T = Eigen::MatrixXd::Identity(3,3);
    T.block(0,0,2,2) = R_12;
    T.block(0,2,2,1) = T_12;

    return T;
}

// float icp::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
// {
//     // TODO: please code by yourself
// }

Eigen::Matrix3d icp::staToMatrix(Eigen::Vector3d sta)
{
    Matrix3d RT;
    RT << cos(sta(2)), -sin(sta(2)), sta(0),
          sin(sta(2)), cos(sta(2)),sta(1),
          0, 0, 1;
    return RT;
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
        //cout<<pc(0,i)<<", "<<pc(1,i)<<endl;
        if(fabs(pc(0,i)) > laser_max || fabs(pc(1,i)) > laser_max ){
            i--;
            total_num--;
        }
    }
    if(total_num > 0){
        pc = pc.leftCols(total_num);       
    }
    else{
        pc = pc.leftCols(1);
    }

    return pc;
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
    ros::init(argc, argv, "icp");
    ros::NodeHandle n;

    icp icp_(n); 


    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    return 0;
}