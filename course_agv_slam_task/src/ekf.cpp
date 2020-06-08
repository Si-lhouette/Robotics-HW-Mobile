#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <geometry_msgs/Pose.h>

#include <ros/transport_hints.h>

using namespace std;
using namespace Eigen;

class ekf{

public:
    ekf(ros::NodeHandle &n);
	~ekf();
    ros::NodeHandle& n;

    // robot init states
    double robot_x;
    double robot_y;
    double robot_theta;
    // match threshold;
    float match_th;
    // bool
    bool isFirstScan;
    // status
    VectorXd status;
    double last_x = 0;
    double last_y = 0;
    double last_t = 0;
    // covariance
    MatrixXd covariance;
    // noise R
    MatrixXd noise_R;
    // noise Q
    MatrixXd noise_Q;
    // landmark num
    int landMark_num;
    // noises
    float noise_motion, noise_measure;
    // count the non-zero elements in status
    int nonZero_cnt;

    // init all 
    void initAll();
    // predict phase
    void predict(nav_msgs::Odometry odom);
    // update phase
    void update(visualization_msgs::MarkerArray input);

    int updateFeatureMap(Eigen::Vector2d lm_pos);
    // landMarks to XY matrix
    Eigen::MatrixXd landMarksToXY(visualization_msgs::MarkerArray input);
    // get motion Jacobian
    MatrixXd getMotionJacobian();
    // get observation Jacobian
    Eigen::MatrixXd getObservJacobian(double q,Vector2d& delta,int lm_id);
    Eigen::MatrixXd getObservJacobianXY(Vector3d sta, double x, double y);
    // angle normalization
    double angleNorm(double angle);
    // find nearest map points
    int findNearestMap(Vector2d lm_pos);

    Vector2d cartesianToPolar(double x, double y);

    Eigen::Vector2d tranToGlobal(Eigen::Vector2d localP);

    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber landMark_sub;
    ros::Subscriber icpOdom_sub;
    ros::Publisher odom_pub;
    ros::Publisher odom_pub_tf;
    ros::Publisher lm_pub;
    tf::TransformBroadcaster ekf_broadcaster;

	//创建TransformListener类监听对象
	tf::TransformListener listener;  //一旦创建了监听器，它就开始通过线路接收tf转换，并将其缓冲10秒
    ros::Publisher error_pub;
 
    void publishResult();

    // pose to transform matrix
    Matrix3d staToMatrix(Vector3d sta);
    // transform matrix to pose
    Vector3d getPose(Matrix3d T);

    int update_cnt = 0;
    double dx_mean = 0;
    double dy_mean = 0;

    int mode;

    double time_origin;

    // run times
    int pre_num, upd_num;
};

ekf::~ekf()
{}

ekf::ekf(ros::NodeHandle& n):
    n(n)
{
    // get the params
	n.getParam("/ekf/robot_x", robot_x);
	n.getParam("/ekf/robot_y", robot_y);
	n.getParam("/ekf/robot_theta", robot_theta);

    n.getParam("/ekf/match_th", match_th);
    n.getParam("/ekf/landMark_num", landMark_num);
    n.getParam("/ekf/noise_motion", noise_motion);
    n.getParam("/ekf/noise_measure", noise_measure);

    n.getParam("/ekf/mode", mode);
    this->initAll();

    isFirstScan = true;
    icpOdom_sub = n.subscribe("/icp_odom", 1, &ekf::predict, this,ros::TransportHints().tcpNoDelay());
    landMark_sub = n.subscribe("/landMarks", 1, &ekf::update, this,ros::TransportHints().tcpNoDelay());
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 1);
    error_pub = n.advertise<std_msgs::Float64>("ekf_error", 1);
    lm_pub = n.advertise<visualization_msgs::MarkerArray>("ALL_landMarks", 1);

    odom_pub_tf = n.advertise<geometry_msgs::Pose>("ekf_odom_tf", 1);

    time_origin = (double)ros::Time::now().toSec();
    cout<<"INIT Done"<<endl;
}

void ekf::predict(nav_msgs::Odometry odom)
{

    cout << "---------------------Prediction-------------------" << endl;
    cout<<"time:"<<odom.header.stamp <<endl;

    // Get icp_odom delta info
    double nowx = odom.pose.pose.position.x;
    double nowy = odom.pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    double nowt = yaw;

    double dx, dy, dt;
    dx = nowx - last_x;
    dy = nowy - last_y;
    dt = nowt - last_t;

    // dx = 0;
    // dy = 0;
    // dt = 0;

    // Calculate Pos
    status[0] = status[0] + dx;
    status[1] = status[1] + dy;
    status[2] = status[2] + dt;
    status[2] = angleNorm(status[2]);

    last_x = nowx;
    last_y = nowy;
    last_t = nowt;


    // Calculate Cov
    MatrixXd G;
    G = getMotionJacobian();
    covariance = G*covariance*G.transpose() + noise_R;

    cout << "predict: " << status(0) << " " << status(1) << " " << status(2) << endl;
    pre_num++;
}

Eigen::Vector2d ekf::tranToGlobal(Eigen::Vector2d localP){
    Vector2d globalP;

    Vector2d r_phi = cartesianToPolar(localP(0), localP(1));
    globalP(0) = status(0) + r_phi(0)*cos(status(2)+r_phi(1));
    globalP(1) = status(1) + r_phi(0)*sin(status(2)+r_phi(1));

    return globalP;
}

void ekf::update(visualization_msgs::MarkerArray input)
{

    cout << "---------------------Update-------------------" << endl;
    double time_0 = (double)ros::Time::now().toSec();
    cout<<"time:"<<input.markers[0].header.stamp<<endl;

    MatrixXd landMarkFeatures = this->landMarksToXY(input);

    for(int i = 0; i < landMarkFeatures.cols(); i++){
        Vector2d lm_pos(landMarkFeatures(0, i), landMarkFeatures(1, i));
        // Trans Local Pos To Global Pos
        lm_pos = tranToGlobal(lm_pos);
        // Find Nearest Pos in status
        int lm_id = findNearestMap(lm_pos);
        // IF New feature
        if(lm_id < 0){
            lm_id = updateFeatureMap(lm_pos);
        }


        double lm_x = status(3+2*lm_id), lm_y = status(3+2*lm_id+1);
            // cout<<"now_lm:"<<lm_x<<","<<lm_y<<endl;
        double r_x = status(0), r_y = status(1), theta = status(2);
            // cout<<"nowt:"<<r_x<<","<<r_y<<","<<theta<<endl;

        // Real Observation
        Vector2d z_real = cartesianToPolar(landMarkFeatures(0,i),landMarkFeatures(1,i));
            // cout<<"z_real:"<<z_real.transpose()<<endl;

        // Predict Observation
        Vector2d z_pre;
        Vector2d delta = Vector2d(lm_x-r_x, lm_y-r_y);
            // cout<<"delta:"<<delta.transpose()<<endl;
        double q = delta.transpose()*delta;
            // cout<<"q:"<<q<<endl;
        z_pre(0) = sqrt(q);
        z_pre(1) = angleNorm(atan2(delta(1),delta(0))-theta);
            // cout<<"z_pre:"<<z_pre.transpose()<<endl;

        // Difference between Real Observation & Predict Observation
        Vector2d z_diff;
        z_diff =  z_real - z_pre;
        z_diff(1) = angleNorm(z_diff(1));
     
        // EKF Update
        MatrixXd H = getObservJacobian(q,delta,lm_id);  
        MatrixXd S = H*covariance*H.transpose() + noise_Q;
        MatrixXd K = covariance * H.transpose() * S.inverse();

        status = status + K*(z_diff);
        status(2) = angleNorm(status(2));
        cout << "updatepos: " << status(0) << " " << status(1) << " " << status(2) << endl;
        covariance = covariance - K*H*covariance;

            // cout<<"z_diff:"<<z_diff.transpose()<<endl;
            // cout<<"k:"<<endl<<K<<endl;
            // cout<<"H:"<<endl<<H<<endl;
            // cout<<"covariance:"<<endl<<covariance.block(0,0,3*nonZero_cnt,3*nonZero_cnt)<<endl;
    }

    


    // Get error from truth value
  //   tf::StampedTransform transform;

  //   try{
		// listener.lookupTransform("map", "robot_base", ros::Time(0), transform);
  //   }
  //   catch (tf::TransformException &ex) {
		// ROS_ERROR("%s",ex.what());
		// cout<<"Don't Get"<<endl;
		// ros::Duration(1.0).sleep();
  //   }
  //   double rel_x = transform.getOrigin().x();
  //   double rel_y = transform.getOrigin().y();

  //   double dx = fabs(rel_x - status(0));
  //   double dy = fabs(rel_y - status(1));
  //   cout<<"delta: "<<dx<<", "<<dy<<endl;
  //   dx_mean = (dx_mean * update_cnt + dx)/(update_cnt+1);
  //   dy_mean = (dy_mean * update_cnt + dy)/(update_cnt+1);
  //   cout<<"delta_mean: "<<dx_mean<<", "<<dy_mean<<endl;

  //   std_msgs::Float64 err;
  //   err.data = sqrt(dx*dx+dy*dy);
  //   error_pub.publish(err);






    this->publishResult();

    double time_1 = (double)ros::Time::now().toSec();
    cout<<"time_cost:  "<<time_1-time_0<<endl;
    cout<<"lm_num: "<<nonZero_cnt<<endl;

    update_cnt++;
    upd_num++;
}

void ekf::initAll()
{
    //status
    status = VectorXd::Zero(3+2*landMark_num);
    status(0) = robot_x;
    status(1) = robot_y;
    status(2) = robot_theta;

    //covariance
    covariance = 999999 * MatrixXd::Identity((3+2*landMark_num), (3+2*landMark_num));
    covariance.block(0, 0, 3, 3) = MatrixXd::Zero(3, 3);

    //noise
    VectorXd dR = MatrixXd::Zero(3 + landMark_num*2, 1);
    dR(0) = 0.5;
    dR(1) = 0.5;
    dR(2) = 0.1;
    MatrixXd R = dR.asDiagonal();
    R = R.array().square();
    noise_R = R;

    noise_Q = noise_measure * MatrixXd::Identity(2, 2);
    noise_Q(2,2) = noise_Q(2,2)*0.1;

    nonZero_cnt = 0;  
    pre_num = 0; upd_num = 0;
}

Eigen::MatrixXd ekf::landMarksToXY(visualization_msgs::MarkerArray input)
{
    int markerSize = input.markers.size();

    Eigen::MatrixXd pc = Eigen::MatrixXd::Ones(3, markerSize);

    for(int i=0; i<markerSize; i++)
    {
        pc(0,i) = input.markers[i].pose.position.x;
        pc(1,i) = input.markers[i].pose.position.y;
    }
    return pc;
}

int ekf::updateFeatureMap(Eigen::Vector2d lm_pos)
{   
    status(3+2*nonZero_cnt) = lm_pos(0);
    status(3+2*nonZero_cnt+1) = lm_pos(1);
    int lm_id = nonZero_cnt;
    nonZero_cnt++;
    return lm_id;
}

int ekf::findNearestMap(Vector2d lm_pos)
{   

    if(nonZero_cnt == 0)
        return -1;
    int mincol;

    MatrixXd temp_LM(2, nonZero_cnt); //Features in old status
    for(int i = 0; i < nonZero_cnt; i++){
        temp_LM(0,i) = status(3+2*i);
        temp_LM(1,i) = status(3+2*i+1);
    }
    VectorXd distance(nonZero_cnt);
    VectorXd::Index minInd;
    double minNum;


    temp_LM.colwise() -= lm_pos;
    distance = temp_LM.array().square().colwise().sum();
    minNum = distance.minCoeff(&minInd);

    if(minNum >= match_th)
        return -1;


    mincol = minInd;
    return mincol;

}

Eigen::MatrixXd ekf::getMotionJacobian()
{
    MatrixXd G;
    G = MatrixXd::Identity(3 + landMark_num*2, 3 + landMark_num*2);
    return G;

}

Eigen::MatrixXd ekf::getObservJacobian(double q,Vector2d& delta,int lm_id)
{
    Eigen::MatrixXd H_low;
    H_low.setZero(2,5);
    H_low<< -sqrt(q)*delta(0),-sqrt(q)*delta(1),0, sqrt(q)*delta(0),sqrt(q)*delta(1),
        delta(1),-delta(0),-q,-delta(1),delta(0);

    H_low=H_low/q;

    MatrixXd F = MatrixXd::Zero(5, 3+2*(landMark_num));
    F.block(0, 0, 3, 3) = MatrixXd::Identity(3, 3);
    F.block(3, 3+2*lm_id, 2, 2) = MatrixXd::Identity(2, 2);


    return H_low*F;
}



Vector2d ekf::cartesianToPolar(double x, double y)
{
    float r = std::sqrt(x*x + y*y);
    float phi = angleNorm(std::atan2(y, x));
    Vector2d r_phi(r, phi);
    return r_phi;
}


double ekf::angleNorm(double angle)
{
    // -180 ~ 180
    while(angle > M_PI)
        angle = angle - 2*M_PI;
    while(angle < -M_PI)
        angle = angle + 2*M_PI;
    return angle;
}


// double ekf::angleNorm(double angle)
// {
//     // 0 ~ 360
//     while(angle > 2*M_PI)
//         angle = angle - 2*M_PI;
//     while(angle < 0)
//         angle = angle + 2*M_PI;
//     return angle;
// }

Matrix3d ekf::staToMatrix(Vector3d sta)
{
    Matrix3d RT;
    RT << cos(sta(2)), -sin(sta(2)), sta(0),
          sin(sta(2)), cos(sta(2)), sta(1),
          0, 0, 1;
    return RT;
}

Vector3d ekf::getPose(Matrix3d T)
{
    Vector3d pose;
    pose(0) = T(0, 2);
    pose(1) = T(1, 2);
    pose(2) = angleNorm(atan2(T(1, 0), T(0, 0)));
    return pose;
}

void ekf::publishResult()
{
    // tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(status(2));

    geometry_msgs::TransformStamped ekf_trans;
    ekf_trans.header.stamp = ros::Time::now();
    ekf_trans.header.frame_id = "map";//"world_base";
    ekf_trans.child_frame_id = "ekf_slam";

    ekf_trans.transform.translation.x = status(0);
    ekf_trans.transform.translation.y = status(1);
    ekf_trans.transform.translation.z = 0.0;
    ekf_trans.transform.rotation = odom_quat;

    ekf_broadcaster.sendTransform(ekf_trans);



    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";//"world_base";

    odom.pose.pose.position.x = status(0);
    odom.pose.pose.position.y = status(1);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom_pub.publish(odom);


    geometry_msgs::Pose odom_tf;
    odom_tf.position.x = status(0);
    odom_tf.position.y = status(1);
    odom_tf.position.z = 0.0;
    odom_tf.orientation = odom_quat;

    odom_pub_tf.publish(odom_tf);


    // landmarks
    visualization_msgs::MarkerArray landMark_array_msg;

    landMark_array_msg.markers.resize(nonZero_cnt);
    for(int i=0; i<nonZero_cnt; i++)
    {
        landMark_array_msg.markers[i].header.frame_id = "map";
        landMark_array_msg.markers[i].header.stamp = ros::Time(0);
        landMark_array_msg.markers[i].ns = "ekf_lm";
        landMark_array_msg.markers[i].id = i;
        landMark_array_msg.markers[i].type = visualization_msgs::Marker::CYLINDER;
        landMark_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
        landMark_array_msg.markers[i].pose.position.x = status(3+2*i);
        landMark_array_msg.markers[i].pose.position.y = status(3+2*i+1);
        landMark_array_msg.markers[i].pose.position.z = 0; // 2D
        landMark_array_msg.markers[i].pose.orientation.x = 0.0;
        landMark_array_msg.markers[i].pose.orientation.y = 0.0;
        landMark_array_msg.markers[i].pose.orientation.z = 0.0;
        landMark_array_msg.markers[i].pose.orientation.w = 1.0;
        landMark_array_msg.markers[i].scale.x = 0.4;
        landMark_array_msg.markers[i].scale.y = 0.4;
        landMark_array_msg.markers[i].scale.z = 0.4;
        landMark_array_msg.markers[i].color.a = 0.5; // Don't forget to set the alpha!
        landMark_array_msg.markers[i].color.r = 1.0;
        landMark_array_msg.markers[i].color.g = 0.0;
        landMark_array_msg.markers[i].color.b = 0.0;
    }

    lm_pub.publish(landMark_array_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n;

    ekf ekf_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}
