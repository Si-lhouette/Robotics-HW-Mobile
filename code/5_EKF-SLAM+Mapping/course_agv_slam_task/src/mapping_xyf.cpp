#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>
#include <cmath>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;
using namespace Eigen;

class mapping {

public:
	mapping(ros::NodeHandle &n);
	~mapping();
	ros::NodeHandle& n;

	// subers & pubers
	ros::Subscriber laser_sub;
	ros::Publisher map_pub;
	tf::TransformListener listener;
	// transform
	tf::StampedTransform transform;
	// global grid map
	nav_msgs::OccupancyGrid grid_map;
	// some variables
	string world_frame, sensor_frame;
	int map_height, map_width;
	float map_res;
	float  dis_threshold, pos_threshold;
	float pos_occupy, pos_free;
	// grid points location
	MatrixXd grid_points;

	// main process
	void process(sensor_msgs::LaserScan input);
	double angleNorm(double angle);
	float inverseSensorModel(Vector3d grid, Vector3d base, int range);
	int cal_pos(double l);
};

mapping::~mapping()
{}

mapping::mapping(ros::NodeHandle& n) :
	n(n)
{
	// get the params
	n.getParam("/mapping/world_frame", world_frame);
	n.getParam("/mapping/sensor_frame", sensor_frame);

	n.getParam("/mapping/map_height", map_height);
	n.getParam("/mapping/dis_threshold", dis_threshold);
	n.getParam("/mapping/pos_threshold", pos_threshold);
	n.getParam("/mapping/pos_occupy", pos_occupy);
	n.getParam("/mapping/pos_free", pos_free);
	n.getParam("/mapping/map_width", map_width);
	n.getParam("/mapping/map_res", map_res);

	// iniitialization
	grid_map.info.height = map_height;	//栅格地图长	20
	grid_map.info.width = map_width;	//栅格地图宽	20
	grid_map.info.resolution = map_res;  //栅格地图分辨率，即一小格的长宽	0.25
	grid_map.header.frame_id = world_frame;

	// set origin of map
	/*(0,0)->*/
	grid_map.info.origin.position.x = -float(grid_map.info.width) / 2 * grid_map.info.resolution;	//-2.5
	grid_map.info.origin.position.y = -float(grid_map.info.height) / 2 * grid_map.info.resolution;	//-2.5
	grid_map.info.origin.orientation.w = 1;

	// fill with -1 / unknown in the map
	grid_map.data.assign(map_width * map_height, 50);

	map_pub = n.advertise<nav_msgs::OccupancyGrid>("grid_map_mine", 1);
	laser_sub = n.subscribe("/course_agv/laser/scan", 1, &mapping::process, this);
}

float calc_dis(Vector3d x, Vector3d y)
{
	return sqrt(pow(x(0) - y(0), 2) + pow(x(1) - y(1), 2));
}

float mapping::inverseSensorModel(Vector3d grid, Vector3d base, int range)
{
	double r = calc_dis(grid, base);
	if (abs(r - range) < dis_threshold)
		return pos_occupy;
	else if (r < range)
		return pos_free;
	else
		return 0;

}

int mapping::cal_pos(double l)
{
	double raw = 1 - 1.0 / (1 + exp(l));
	if (raw > pos_threshold)
		return 100;
	else
		return 0;
}

double mapping::angleNorm(double angle)
{
	while (angle > M_PI)
		angle = angle - 2 * M_PI;
	while (angle < -M_PI)
		angle = angle + 2 * M_PI;
	return angle;
}


void mapping::process(sensor_msgs::LaserScan input)
{
	cout << "------seq:  " << input.header.seq << endl;
	int total_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;
	MatrixXd l = MatrixXd::Zero(map_width / map_res, map_height / map_res);		//对数概率矩阵
	Vector3d grid;	//矩阵坐标
	/*1.监听来自EKF的大T矩阵,得到pose_base*/
	/*To Do*/
	/*返回  pose_base 机器人全局坐标三维向量，转化矩阵trans*/
	Vector3d pose_base;
	double range;

	/*2.对于矩形框内所有的小矩形进行判断*/
	for (int x = 0; x < map_width / map_res; x++) {
		for (int y = 0; y < map_height / map_res; y++) {

			grid << x * map_res + map_res / 2, y * map_res + map_res / 2, 1;
			if (calc_dis(grid, pose_base) <= input.range_max) {		//在筛选范围内

				grid = trans * grid;		//转换到机器人坐标系
				double angle = angleNorm(std::atan2(grid(1), grid(0)));	//角度
				int index = angle / input.angle_increment + total_num / 2;
				if (index < total_num - 1){
					range = input.ranges[index] < input.ranges[index + 1] ? input.ranges[index] : input.ranges[index + 1];
				}
				else{
					range = input.ranges[index];
				}
				l(x, y) = l(x, y)+inverseSensorModel(grid, pose_base, range);	//更新概率
			}
			grid_map.data[x + y * (map_height / map_res)] = cal_pos(l(x, y));
		}
	}
	map_pub.publish(grid_map);
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