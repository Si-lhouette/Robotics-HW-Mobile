#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

#include <cmath>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
#include <ros/transport_hints.h>

using namespace std;
using namespace Eigen;

/**
 * Structure: 记录两帧之间匹配的index和匹配特征的距离
 **/
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
    Vector3d sensor_sta;    // sensor states = robot_x_y_theta

    // 轮式里程计相关变量
    double leftv = 0.0;
    double rightv = 0.0;
    double timelast;
    double dx = 0.0;
    double dy = 0.0;
    double dtheta = 0.0;
    double theta = 0.0;
    bool setzero = false;   // 清零flag：每获取一遍轮式里程计差值，就将其清零(更好的做法是将轮式里程计单独写为节点，并做好同步)

    double rx = 1.0/0.08;
    double rw = (0.1+0.08/6)/0.08;

    int max_iter;           // 最大迭代次数
    double dis_th_normal;   // 匹配点的最大距离
    double tolerance;       // 迭代的tolerance
    bool isFirstScan;       // 是否为第一帧
    MatrixXd src_pc;        // 当前帧点云矩阵
    MatrixXd tar_pc;        // 上一帧点云矩阵
    int min_match;          // ICP里程计两帧之间的最少匹配数量

    // 误差计算
    int pub_cnt = 0;
    double dx_mean = 0;
    double dy_mean = 0;

    // 跳帧操作变量
    int skip_num = 0;       // 连续当前帧特征数<min_match的次数
    bool jump_flag = false; // 跳帧的flag
    int jump_switch;        // 跳帧操作的总开关

    /* 轮式里程计 */
    // 轮式里程计：获得2帧之间的2D旋转矩阵和位移向量
    void calcinitRT(Eigen::Matrix2d& R_all, Eigen::Vector2d& T_all);
    // 回调函数-接收左轮轮速
    void leftv_callback(const std_msgs::Float64::ConstPtr& msg);
    // 回调函数-接收右轮轮速，并计算运动差值 
    void rightv_callback(const std_msgs::Float64::ConstPtr& msg);

    // 回调函数-匹配当前帧(src) to 上一帧(tar){Main Function}
    void process(visualization_msgs::MarkerArray input);
    // 将接收到的Landmark Set转换为3×n矩阵形式
    Eigen::MatrixXd landMarksToMatrix(visualization_msgs::MarkerArray input);
    // Find neareast correspond of 2 frame
    NeighBor findNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar, double dis_th);
    // 用SVD法求解 RotationMatrix & translation vector
    Eigen::Matrix3d getTransform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
    // 将状态增量向量转换成2D齐次变化矩阵
    Eigen::Matrix3d staToMatrix(const Vector3d sta);


    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber landMark_sub;
    void publishResult(Eigen::Matrix3d T ,std_msgs::Header the_head);
 	tf::TransformBroadcaster odom_broadcaster;
    tf::TransformListener listener;
 	ros::Publisher odom_pub;
    ros::Publisher error_pub;
    ros::Subscriber leftv_sub;
    ros::Subscriber rightv_sub;
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
	n.getParam("/icp_lm/dis_th", dis_th_normal);
    n.getParam("/icp_lm/min_match", min_match);

    n.getParam("/icp_lm/jump_switch", jump_switch);

    isFirstScan = true;
    landMark_sub = n.subscribe("/landMarks", 1, &icp_lm::process, this,ros::TransportHints().tcpNoDelay());
    leftv_sub = n.subscribe("/course_agv/left_wheel_velocity_controller/command", 1, &icp_lm::leftv_callback, this,ros::TransportHints().tcpNoDelay());
    rightv_sub = n.subscribe("/course_agv/right_wheel_velocity_controller/command", 1, &icp_lm::rightv_callback, this,ros::TransportHints().tcpNoDelay());

    odom_pub = n.advertise<nav_msgs::Odometry>("icp_odom", 1);
    error_pub = n.advertise<std_msgs::Float64>("icp_error", 1);
}

/**
 * Function: 回调函数-接收左轮轮速
 * Input: input{左轮轮速}
 **/
void icp_lm::leftv_callback(const std_msgs::Float64::ConstPtr& msg){
    this->leftv = msg->data;
}

/**
 * Function: 回调函数-接收右轮轮速，并计算运动差值
 * Input: input{右轮轮速}
 **/
void icp_lm::rightv_callback(const std_msgs::Float64::ConstPtr& msg){
    this->rightv = msg->data;

    double tnow = (double)ros::Time::now().toSec();
    double dt = tnow - this->timelast;
    if(this->setzero){
        this->dx = 0;
        this->dy = 0;
        this->dtheta = 0;
        this->setzero = false;
    }

    double vx = (this->leftv + this->rightv)/(2.0*this->rx);
    double vw = (this->rightv - this->leftv)/(2.0*this->rw);
    this->dx += vx*dt*cos(sensor_sta(2)+this->dtheta);
    this->dy += vx*dt*sin(sensor_sta(2)+this->dtheta);
    this->dtheta += vw*dt;
    this->timelast = tnow;
}

/**
 * Function: 轮式里程计：获得2帧之间的2D旋转矩阵和位移向量
 * Input: R_all{2D旋转矩阵}，T_all{位移向量}
 **/
void icp_lm::calcinitRT(Eigen::Matrix2d& R_all, Eigen::Vector2d& T_all){
    Vector3d sta;
    sta << this->dx, this->dy, this->dtheta;
    Matrix3d RT = staToMatrix(sta);
    R_all = RT.block(0,0,2,2);
    T_all = RT.block(0,2,2,1);
    theta += this->dtheta;
    this->setzero = true;
}

/**
 * Function: 回调函数-匹配当前帧(src) to 上一帧(tar)
 * Input: input{landmark set}
 **/
void icp_lm::process(visualization_msgs::MarkerArray input)
{   
    std_msgs::Header the_head;
    the_head.stamp = input.markers[0].header.stamp;
    cout<<endl<<"------Time:  "<<input.markers[0].header.stamp<<endl;
    
    double time_0 = (double)ros::Time::now().toSec();

    /* 1. 如果是第一帧 */
    if(isFirstScan)
    {
        tar_pc = this->landMarksToMatrix(input);

        // init the var used in Odometry
        this->timelast = (double)ros::Time::now().toSec();
        this->setzero = true;


        isFirstScan =false;
        return;
    }

    src_pc = this->landMarksToMatrix(input);// get current frame points

    /* 2. Prepare */
    // make a copy
    MatrixXd src_pc_copy;
    src_pc_copy = src_pc;

    // init some variables
    Eigen::Matrix3d Transform_acc = Eigen::MatrixXd::Identity(3,3);
    Eigen::Matrix2d R_all = Eigen::MatrixXd::Identity(2,2);
    Eigen::Vector2d T_all;
    T_all << 0.0,0.0;
    double elast = 10000; //error of last time 

    // 用轮式里程计初始化R&T矩阵
    // calcinitRT(R_all, T_all); //!! YOU CAN COMMENT IT, NOT REALLY USEFUL
    // Transform_acc.block(0,0,2,2) = R_all;
    // Transform_acc.block(0,2,2,1) = T_all;

    /* 3. 检查当前帧src中的特征数量 */
    if(src_pc.cols() < min_match){
        //this->publishResult(Transform_acc); //pub wheel_odometry Transform
        skip_num++;
        if(skip_num>=3){
            jump_flag = true;
        }
        cout<<"< min"<<endl;
        return;
    }
    // 当src中特征数量连续3次<min_match时 -> 跳帧操作
    if(jump_flag && jump_switch)
    {
        tar_pc = this->landMarksToMatrix(input);

        // init the var used in Odometry
        this->timelast = (double)ros::Time::now().toSec();
        this->setzero = true;

        cout<<"JUMP!!!!!!!!!!!!!"<<endl; 
        jump_flag =false;
        return;
    }


    /* 4. 匹配的 Main LOOP */
    for(int i=0; i<max_iter; i++)
    {
        //4.1. Find neareast correspond
        NeighBor s_t_near;
        s_t_near = findNearest(src_pc, tar_pc, dis_th_normal);
        int s_num = s_t_near.src_indices.size();

        MatrixXd tar_pcn;
        MatrixXd src_pcn;
        src_pcn = Eigen::MatrixXd::Constant(3, s_num, 1);
        tar_pcn = Eigen::MatrixXd::Constant(3, s_num, 1);
        for(int j = 0; j < s_num; j++){
            src_pcn.col(j) = src_pc.col(s_t_near.src_indices[j]);
            tar_pcn.col(j) = tar_pc.col(s_t_near.tar_indices[j]);
        }

        //Check Matched num
        int match_num = src_pcn.cols();
        if(i==0){
            cout<<"Matched num:"<<match_num<<endl;
        }

        if(match_num < min_match){
            skip_num++;
            if(skip_num>=3){
                jump_flag = true;
            }
            return;
        }
        skip_num = 0;

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
        double e = accumulate(s_t_near.distances.begin(), s_t_near.distances.end(), 0.0) / s_num;
        //cout<<"e:"<<e<<endl;
        if(e < tolerance || fabs((e - elast)/elast) < 0.001 ){
            cout<<"loop_"<<i<<endl;
            cout<<"e:"<<e<<endl;
            break;
        }
        if(i == max_iter){
            cout<<"Max_loop"<<endl;
            cout<<"e:"<<e<<endl;
        }
    }

    // 5. Update Last frame
    tar_pc = src_pc_copy;

    // 6. Publish
    Transform_acc.block(0,0,2,2) = R_all;
    Transform_acc.block(0,2,2,1) = T_all;

    this->publishResult(Transform_acc, the_head);

    double time_1 = (double)ros::Time::now().toSec();
    cout<<"time_cost:  "<<time_1-time_0<<endl;
}

/**
 * Function: 将接收到的Landmark Set转换为3×n矩阵形式
 * Input: input{Landmark Set}
 * Output: 3×n齐次坐标矩阵
 **/
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

/**
 * Function: Find neareast correspond of 2 frame
 * Input: src{当前帧点云}，tar{上一帧点云}，dis_th{两帧之间匹配特征的距离阈值}
 * Output: NeighBor{src和tar对应特征的index}
 **/
NeighBor icp_lm::findNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar, double dis_th)
{
    vector<float> dist;
    vector<int> src_ind, tar_ind;
    NeighBor res;
    MatrixXd src_pos(2, src.cols());
    MatrixXd tar_pos(2, tar.cols());
    MatrixXd temp_pos(2, tar.cols());
    VectorXd distance(tar.cols());
    VectorXd::Index minInd;
    double minNum;

    src_pos = src.topRows(2);
    tar_pos = tar.topRows(2);

    for(int i = 0; i < src_pos.cols(); i++){
        temp_pos = tar_pos;
        temp_pos.colwise() -= src_pos.col(i);
        distance = temp_pos.array().square().colwise().sum();
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

/**
 * Function: 用SVD法求解 RotationMatrix & translation vector
 * Input: src{匹配后的当前帧点云}，tar{匹配后的上一帧点云}
 * Output: T{T*src=tar}
 **/
Eigen::Matrix3d icp_lm::getTransform(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
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

/**
 * Function: 将状态增量向量转换成2D齐次变化矩阵
 * Input: sta{状态增量向量}
 * Output: 2D齐次变化矩阵
 **/
Eigen::Matrix3d icp_lm::staToMatrix(Eigen::Vector3d sta)
{
	Matrix3d RT;
    RT << cos(sta(2)), -sin(sta(2)), sta(0),
          sin(sta(2)), cos(sta(2)),sta(1),
          0, 0, 1;
    return RT;
}

/**
 * Function: Publish
 * Input: T{T*src=tar}，the_head{接收到的landmark set的消息头}
 **/
void icp_lm::publishResult(Eigen::Matrix3d T ,std_msgs::Header the_head)
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
    odom_trans.header.stamp = the_head.stamp;
    odom_trans.header.frame_id = "map";//"world_base";
    odom_trans.child_frame_id = "icp_odom";

    odom_trans.transform.translation.x = sensor_sta(0);
    odom_trans.transform.translation.y = sensor_sta(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    // odom
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";//"world_base";

    odom.pose.pose.position.x = sensor_sta(0);
    odom.pose.pose.position.y = sensor_sta(1);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom_pub.publish(odom);

    // tf error calculate
    // tf::StampedTransform transform;

    // try{
    //     listener.lookupTransform("world_base", "robot_base", ros::Time(0), transform);
    // }
    // catch (tf::TransformException &ex) {
    //     ROS_ERROR("%s",ex.what());
    //     cout<<"Don't Get"<<endl;
    //     ros::Duration(1.0).sleep();
    // }
    // double rel_x = transform.getOrigin().x();
    // double rel_y = transform.getOrigin().y();

    // double dx = fabs(rel_x - sensor_sta(0));
    // double dy = fabs(rel_y - sensor_sta(1));
    // cout<<"delta: "<<dx<<", "<<dy<<endl;
    // dx_mean = (dx_mean * pub_cnt + dx)/(pub_cnt+1);
    // dy_mean = (dy_mean * pub_cnt + dy)/(pub_cnt+1);
    // cout<<"delta_mean: "<<dx_mean<<", "<<dy_mean<<endl;

    // std_msgs::Float64 err;
    // err.data = sqrt(dx*dx+dy*dy);
    // error_pub.publish(err);

    pub_cnt++;
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