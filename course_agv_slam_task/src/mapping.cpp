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
    // global grid map
    nav_msgs::OccupancyGrid grid_map;
    bool firstmap = true;
    nav_msgs::OccupancyGrid origin_map;
    nav_msgs::OccupancyGrid err_map;
    MatrixXd map_log;
    // some variables
    string world_frame, sensor_frame;
    int map_height, map_width;
    float map_res;
    double sensor_range;
    double prob_init;
    double prob_occu;
    double prob_free;
    double lprob_init;
    double lprob_occu;
    double lprob_free;
    double occu_dis_thres;

    double prob_thres;

    // grid points location
    MatrixXd grid_points;

    //laser save
    sensor_msgs::LaserScan laserScan;
    
    // main process
    void laserCallback(sensor_msgs::LaserScan input);
    void process(nav_msgs::Odometry odom);
    double angleNorm(double angle);
    double inverseSensorModel(Vector2d rgrid, Vector2d roboPose, double range);
    void subMapSever(nav_msgs::OccupancyGrid origin_map);
    double CalMapDiff();
    void TransToBinaryMap();
};

mapping::~mapping()
{}

mapping::mapping(ros::NodeHandle& n):
    n(n)
{
    // get the params
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

    occu_dis_thres = map_res*sqrt(2);

    lprob_init = log(prob_init/(1-prob_init));
    lprob_occu = log(prob_occu/(1-prob_occu));
    lprob_free = log(prob_free/(1-prob_free));


    
    // iniitialization
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


    // iniitialization
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

    
    map_log = MatrixXd::Constant(map_width, map_height, lprob_init);

    map_pub = n.advertise<nav_msgs::OccupancyGrid>("grid_map_mine", 1);
    err_map_pub = n.advertise<nav_msgs::OccupancyGrid>("err_map_mine", 1);
    error_pub = n.advertise<std_msgs::Float64>("map_error", 1);

    pf_odom_sub = n.subscribe("pf_odom", 1, &mapping::process, this);
    map_sub = n.subscribe("/map",1,&mapping::subMapSever, this);
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &mapping::laserCallback, this);
}

void mapping::laserCallback(sensor_msgs::LaserScan input){
    this->laserScan = input;
}

void mapping::subMapSever(nav_msgs::OccupancyGrid map){
    if(firstmap){
        cout<<"Connected to MapSever And Get the map..."<<endl;
        firstmap = false;
        this->origin_map = map;
    }else{
        return;
    }
}

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

void mapping::process(nav_msgs::Odometry odom)
{
    cout<<endl<<"------seq:  "<<laserScan.header.seq<<endl;
    //mapping signal: odom.pose.pose.position.z = 1.0
    if(odom.pose.pose.position.z < 1.0){
        return;
    }
    int laser_num = (laserScan.angle_max - laserScan.angle_min) / laserScan.angle_increment + 1;
    //cout<<"laser_num:"<<laser_num<<endl;
    //cout<<"anglemaxmin:"<<input.angle_max<<", "<<input.angle_min<<endl;

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
            // cout<<"tgrid:"<<tgrid.transpose()<<endl;

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
            // cout<<"rangle:"<<rangle<<endl;
            int inx = rangle /laserScan.angle_increment + laser_num/2;
            // cout<<"inx:"<<inx<<endl;

            
            if(laserScan.ranges[inx] > laserScan.ranges[inx+1]){
                inx++;
            }
            range = laserScan.ranges[inx]+0.1;
            // cout<<"range:"<<range<<endl;

            // 2.2 计算栅格占用概率
            map_log(i, j) = map_log(i, j) + inverseSensorModel(tgrid, roboPose, range) - lprob_init;
            // cout<<"maplog:"<<map_log(i,j)<<endl;

            // 2.3 计算grid_map值 0-100
            grid_map.data[j*grid_map.info.width+i] = 100.0 * (1 - 1.0 / (1 + exp(map_log(i, j))));
            // cout<<"grid_map:"<<grid_map.data[j*grid_map.info.width+i]<<endl;


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

double mapping::inverseSensorModel(Vector2d tgrid, Vector2d roboPose, double range)
{
	double dis = sqrt(pow(tgrid(0) - roboPose(0), 2) + pow(tgrid(1) - roboPose(1), 2));
    // cout<<"roboPose:"<<roboPose.transpose()<<endl;
    // cout<<"tgrid:"<<tgrid.transpose()<<endl;
    // cout<<"dis:"<<dis<<endl;

    if(dis > min(sensor_range, range + occu_dis_thres)){
        // cout<<"lprob_init"<<endl;
        return lprob_init;
    }
    if(range < sensor_range && abs(dis - range) < occu_dis_thres){
        // cout<<"lprob_occu"<<endl;
        return lprob_occu;
    }
    if(range < sensor_range){
        // cout<<"prob_free"<<endl;
        return lprob_free;
    }
    return lprob_init;
}


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