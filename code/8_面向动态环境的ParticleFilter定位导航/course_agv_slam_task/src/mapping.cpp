#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/Float64.h"
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace Eigen;

class mapping{
public:
    mapping(ros::NodeHandle &n);
	~mapping();
    ros::NodeHandle& n;

    // subers & pubers
    ros::Subscriber pf_odom_sub;
    ros::Subscriber laser_sub;
    ros::Publisher map_pub;
    ros::Publisher err_map_pub;
    ros::Publisher error_pub;
    ros::Subscriber map_sub;
    tf::TransformListener listener;

    // transform
    tf::StampedTransform transform;
    string world_frame, sensor_frame;

    // global grid map
    nav_msgs::OccupancyGrid grid_map;       // 用于输出的贝叶斯栅格地图，每格取值[0,100]
    nav_msgs::OccupancyGrid origin_map;     // MapServer发出的真值地图
    nav_msgs::OccupancyGrid err_map;        // 建图误差地图
    MatrixXd map_log;                       // 建立的贝叶斯占用栅格地图(对数概率)
    bool firstMap = true;                   
    bool firstOriginMap = true;
    
    // map variables
    int map_height, map_width;
    float map_res;
    double sensor_range;                    // 激光雷达束最远测量半径
    double prob_init;                       // 地图初始占用概率
    double prob_occu;                       // 地图占用增量概率(一次观测后)
    double prob_free;                       // 地图未占用增量概率(一次观测后)
    double lprob_init;                      // 地图初始占用对数概率
    double lprob_occu;                      // 地图占用增量对数概率(一次观测后)
    double lprob_free;                      // 地图未占用增量对数概率(一次观测后)
    double occu_dis_thres;                  // 判断激光点是否落在障碍物栅格时采用的距离半径

    double prob_thres;                      // 地图二值化阈值

    //laser save
    sensor_msgs::LaserScan laserScan;       // 当前帧激光雷达消息
    
    // 回调函数: 接收当前帧laser信息并存储于类变量中
    void laserCallback(sensor_msgs::LaserScan input);
    // 建图主要函数
    void process(nav_msgs::Odometry odom);
    // 角度归一化
    double angleNorm(double angle);
    // 传感器重建模型
    double inverseSensorModel(Vector2d rgrid, Vector2d roboPose, double range);
    // 回调函数: 接收MapServer 发出的真实地图，用于比较建图误差
    void subMapSever(nav_msgs::OccupancyGrid origin_map);
    // 计算二值化后的建图误差(非必要函数)
    double CalMapDiff();
    // 将地图二值化
    void TransToBinaryMap();
};

mapping::~mapping()
{}

mapping::mapping(ros::NodeHandle& n):
    n(n)
{
    /* 获取参数服务器中变量 */
    n.getParam("/mapping/world_frame", world_frame);
	n.getParam("/mapping/sensor_frame", sensor_frame);

	n.getParam("/mapping/map_height", map_height); 
	n.getParam("/mapping/map_width", map_width);
	n.getParam("/mapping/map_res", map_res);//地图分辨率

    n.getParam("/mapping/sensor_range", sensor_range);
    n.getParam("/mapping/prob_init", prob_init);
    n.getParam("/mapping/prob_occu", prob_occu);
    n.getParam("/mapping/prob_free", prob_free);
    n.getParam("/mapping/prob_thres", prob_thres);

    /* 建图相关参数初始化 */
    occu_dis_thres = map_res*sqrt(2);
    lprob_init = log(prob_init/(1-prob_init));
    lprob_occu = log(prob_occu/(1-prob_occu));
    lprob_free = log(prob_free/(1-prob_free));

    // grid_map 初始化
    grid_map.info.height = map_height;
    grid_map.info.width = map_width;
    grid_map.info.resolution = map_res;
    grid_map.header.frame_id = world_frame;
    // set origin of map
    grid_map.info.origin.position.x = - float(grid_map.info.width) / 2 * grid_map.info.resolution;
    grid_map.info.origin.position.y = - float(grid_map.info.height) / 2 * grid_map.info.resolution;
    grid_map.info.origin.orientation.w = 1;
    // fill with -1 / unknown in the map
    grid_map.data.assign(map_width * map_height, -1);

    // err_map 初始化
    err_map.info.height = map_height;
    err_map.info.width = map_width;
    err_map.info.resolution = map_res;
    err_map.header.frame_id = world_frame;
    // set origin of map
    err_map.info.origin.position.x = - float(err_map.info.width) / 2 * err_map.info.resolution;
    err_map.info.origin.position.y = - float(err_map.info.height) / 2 * err_map.info.resolution;
    err_map.info.origin.orientation.w = 1;
    // fill with 0 / unknown in the map
    err_map.data.assign(map_width * map_height, -1);

    // map_log 初始化
    map_log = MatrixXd::Constant(map_width, map_height, lprob_init);

    map_pub = n.advertise<nav_msgs::OccupancyGrid>("grid_map_mine", 1);
    err_map_pub = n.advertise<nav_msgs::OccupancyGrid>("err_map_mine", 1);
    error_pub = n.advertise<std_msgs::Float64>("map_error", 1);

    pf_odom_sub = n.subscribe("pf_odom", 1, &mapping::process, this);
    map_sub = n.subscribe("/map",1,&mapping::subMapSever, this);
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &mapping::laserCallback, this);
}

/**
 * Function: 回调函数-接收当前帧laser信息并存储于类变量中
 * Input: input{gazebo发出的laser消息}
 **/
void mapping::laserCallback(sensor_msgs::LaserScan input){
    this->laserScan = input;
}

/**
 * Function: 回调函数-接收MapServer 发出的真实地图，用于比较建图误差
 * Input: input{gazebo发出的真值地图消息}
 **/
void mapping::subMapSever(nav_msgs::OccupancyGrid map){
    if(firstOriginMap){
        cout<<"Connected to MapSever And Get the map..."<<endl;
        firstOriginMap = false;
        this->origin_map = map;
    }else{
        return;
    }
}

/**
 * Function: 计算二值化后的建图误差(非必要函数)
 * Output: 建图错误的栅格比例
 **/
double mapping::CalMapDiff(){
    int err_num = 0;
    int all_num = map_height*map_width;
    err_map.data.assign(map_width * map_height, -1);
    for(int i = 0; i < map_height; i++){
        for(int j = 0; j < map_width; j++){
            if(grid_map.data[j*grid_map.info.width+i] < 0){
                all_num--;
                continue;
            }

            if(grid_map.data[j*grid_map.info.width+i] - origin_map.data[j*grid_map.info.width+i] != 0){
                err_map.data[j*grid_map.info.width+i] = 100;
                err_num++;
            }
            else{
                err_map.data[j*grid_map.info.width+i] = 0;
            }
        }
    }
    return (err_num*1.0)/(all_num*1.0);
}

/**
 * Function: 建图主要函数
 * Input: odom{建图信号}
 **/
void mapping::process(nav_msgs::Odometry odom)
{
    cout<<endl<<"------seq:  "<<laserScan.header.seq<<endl;
    if(firstMap){
        firstMap = false;
        return;
    }
    //mapping signal: odom.pose.pose.position.z = 1.0
    if(odom.pose.pose.position.z < 1.0){
        return;
    }
    int laser_num = (laserScan.angle_max - laserScan.angle_min) / laserScan.angle_increment + 1;

    /* 1.监听机器人局部坐标系 */
    try{
        listener.lookupTransform(world_frame, sensor_frame,  
                                    ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    Vector2d roboPose(transform.getOrigin().getX(), transform.getOrigin().getY());
    cout<<roboPose<<" origin"<<endl;
    cout<<"map_res"<<map_res<<endl;
    cout<<"map_height"<<map_height<<endl;
    roboPose(0) += map_res * map_height/2;
    roboPose(1) += map_res * map_width/2;
    cout<<roboPose<<" after"<<endl;
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    yaw = angleNorm(yaw);

    Matrix2d R;
    R << cos(yaw),-sin(yaw),
         sin(yaw),cos(yaw);

    double range;
    /* 2.遍历每个栅格，进行二值贝叶斯滤波 */
    for(int i = 0; i < map_height; i++){
        for(int j = 0; j < map_width; j++){
            // 2.1 将当前栅格中心坐标变换到机器人局部极坐标系下，寻找离其最近的一束激光
            Vector2d tgrid;
            tgrid << i*map_res + map_res/2.0, j*map_res + map_res/2.0;

            double grid_dis = sqrt(pow(tgrid(0) - roboPose(0), 2) + pow(tgrid(1) - roboPose(1), 2)); //栅格离机器人距离
            if(grid_dis > sensor_range + occu_dis_thres){
                grid_map.data[j*grid_map.info.width+i] = 100.0 * (1 - 1.0 / (1 + exp(map_log(i, j))));
                continue; //不在激光范围内
            }
            if(grid_dis < 0.4){ //机器人本身范围内默认不存在障碍物
                grid_map.data[j*grid_map.info.width+i] = 0.0;
                continue;
            }

            Vector2d rgrid;
            rgrid = R.inverse()*(tgrid - roboPose); //将全局坐标系下的栅格位置转换到局部坐标系下
            double rangle = angleNorm(std::atan2(rgrid(1), rgrid(0)));
            int inx = rangle /laserScan.angle_increment + laser_num/2;

            if(laserScan.ranges[inx] > laserScan.ranges[inx+1]){
                inx++;
            }
            range = laserScan.ranges[inx]+0.1;

            // 2.2 计算栅格占用对数概率
            map_log(i, j) = map_log(i, j) + inverseSensorModel(tgrid, roboPose, range) - lprob_init;

            // 2.3 计算栅格占用概率，grid_map值 0-100
            grid_map.data[j*grid_map.info.width+i] = 100.0 * (1 - 1.0 / (1 + exp(map_log(i, j))));
        }
    }

    map_pub.publish(grid_map);

    /* 3.将grid_map二值化 */
    TransToBinaryMap();

    /* 4.计算误识率 */
    double err_ratio = CalMapDiff();
    error_pub.publish(err_ratio);
    cout<<"err_ratio:"<<err_ratio<<endl;

    // publish
    err_map_pub.publish(err_map);
}

/**
 * Function: 将地图二值化
 **/
void mapping::TransToBinaryMap(){
    for(int i = 0; i < map_height; i++){
        for(int j = 0; j < map_width; j++){
            if(grid_map.data[j*grid_map.info.width+i] >=  prob_thres*100){
                grid_map.data[j*grid_map.info.width+i] = 100;
            }
            else{
                grid_map.data[j*grid_map.info.width+i] = 0;
            }
        }
    }
}

/**
 * Function: 传感器重建模型
 * Input: tgrid{当前栅格在全局坐标系下坐标}，roboPose{当前机器人全局坐标}，range{laser测量半径}
 * Output: 用于更新的地图增量对数概率
 **/
double mapping::inverseSensorModel(Vector2d tgrid, Vector2d roboPose, double range)
{
	double dis = sqrt(pow(tgrid(0) - roboPose(0), 2) + pow(tgrid(1) - roboPose(1), 2));

    if(dis > min(sensor_range, range + occu_dis_thres)){
        return lprob_init;
    }
    if(range < sensor_range && abs(dis - range) < occu_dis_thres){
        return lprob_occu;
    }
    if(range < sensor_range){
        return lprob_free;
    }
    return lprob_init;
}

/**
 * Function: 角度归一化[-pi~pi]
 **/
double mapping::angleNorm(double angle)
{
	while (angle > M_PI)
		angle = angle - 2 * M_PI;
	while (angle < -M_PI)
		angle = angle + 2 * M_PI;
	return angle;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping"); 
    ros::NodeHandle n;

    mapping mapping_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}