#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>

#include <ros/transport_hints.h>

using namespace std;
using namespace Eigen;


// LandMark集合的structure
typedef struct{
    std::vector<float> position_x;
    std::vector<float> position_y;
    std::vector<int> id;
} LandMarkSet;

class extraction{
public:
	extraction(ros::NodeHandle &n);
	~extraction();
    ros::NodeHandle& n;
 
    // landmark参数过滤阈值
    float label_dr_threshold;               // 判定为不同cluster的距离阈值
    float range_threshold;                  // 在距离机器人多少半径范围内提取landmark
    float radius_max_th;                    // 判定一个cluster为landmark的最大半径
    int landMark_min_pt;                    // 判定为landmark时，cluster中最少要有多少个点

    ros::Subscriber laser_sub;
    ros::Publisher landMark_pub;
    ros::Publisher laser_pub;

    // 回调函数-将laser分成不同的cluster，并判定是否为圆柱landmark(主函数)
    void process(sensor_msgs::LaserScan input);
    // 回调函数-判定每个label的laser cluster是否为圆柱landmark
    LandMarkSet extractLandMark(sensor_msgs::LaserScan input);
    // 最小二乘拟合圆，返回圆心点
    Eigen::Vector2d fitcircle(std::vector<Eigen::Vector2d> &points);
    // Pub Landmark
    void publishLandMark(LandMarkSet input);
    // 计算2点的欧式距离
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
};

extraction::~extraction()
{}

extraction::extraction(ros::NodeHandle& n):n(n)
{   
    // get the params
    n.getParam("/extraction/range_threshold", range_threshold);
    n.getParam("/extraction/radius_max_th", radius_max_th);
    n.getParam("/extraction/landMark_min_pt", landMark_min_pt);
    n.getParam("/extraction/label_dr_threshold", label_dr_threshold);
    
    landMark_pub = n.advertise<visualization_msgs::MarkerArray>("landMarks", 1);
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &extraction::process, this,ros::TransportHints().tcpNoDelay());
    laser_pub = n.advertise<sensor_msgs::LaserScan>("myscan",1);
}

/**
 * Function: 回调函数-将laser分成不同的cluster，并判定是否为圆柱landmark
 * Input: input{gazebo发出的laser消息}
 **/
void extraction::process(sensor_msgs::LaserScan input)
{   
    double time_0 = (double)ros::Time::now().toSec();
    cout<<endl<<"--------------------------------------------------------------------------"<<endl;

    int label = 0;
    sensor_msgs::LaserScan laser_extr = input;
    int total_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;

    // 初始化上一个激光点
    Vector2d last_xy(input.ranges[0] * std::cos(input.angle_min), input.ranges[0] * std::sin(input.angle_min));

    // 将laser分成不同的cluster，存于intensities属性中
    Vector2d curr_xy;
    float angle, delta_dis;
    for(int i=0; i<total_num; i++)
    {   
        angle = input.angle_min + i * input.angle_increment;
        curr_xy << input.ranges[i] * std::cos(angle), input.ranges[i] * std::sin(angle);
        
        delta_dis = this->calc_dist(curr_xy, last_xy); // 与上一个激光点的距离
        if(delta_dis > label_dr_threshold)
            label++;

        laser_extr.intensities[i] = label;
        last_xy = curr_xy;
    }

    // 判定是否为圆柱landmark
    LandMarkSet landmarks_ = this->extractLandMark(laser_extr);
    this->laser_pub.publish(laser_extr);
    this->publishLandMark(landmarks_);
    double time_1 = (double)ros::Time::now().toSec();
    // cout<<"time_cost:  "<<time_1-time_0<<endl;
}

/**
 * Function: 回调函数-判定每个label的laser cluster是否为圆柱landmark
 * Input: input{加入label[0,...]的laser消息}
 * Output: 提取出的Landmark集合
 **/
LandMarkSet extraction::extractLandMark(sensor_msgs::LaserScan input)
{   
    int total_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;
    LandMarkSet landMarks;
    float angle;
    double dis;

    int cnt = 0;
    int len = 1;                // 上1簇中的点的数量
    int exp_label = 0;          // 希望取到的下一个点的label值

    bool firstlast = true;
    bool check_right = true;    // 检查landmark是否通过的flag

    // 局部坐标系下的landmark坐标
    Vector2d last_xy;           // 上1簇的第一个点
    Vector2d form_xy;           // 上1簇的最后一个点
    Vector2d curr_xy;           // 这一簇的第一个点

    for(int i=0; i<total_num - landMark_min_pt ; i++)  //Attention!! total_num - landMark_min_pt 否则可能出现位于激光sweep开始和结束位置重复识别
    {
        /* 轮到希望取到的下一个点的label值 */
        if(input.intensities[i] == exp_label){
            angle = input.angle_min + i * input.angle_increment;
            curr_xy << input.ranges[i] * std::cos(angle), input.ranges[i] * std::sin(angle);

            if(firstlast){
                last_xy = curr_xy;
                firstlast = false;
                exp_label++;
                continue;
            }
            angle = input.angle_min + (i-1) * input.angle_increment;
            form_xy << input.ranges[i-1] * std::cos(angle), input.ranges[i-1] * std::sin(angle);
            dis = calc_dist(last_xy, form_xy); //2 point at begin & end of the cluster
            double dis_from_robot = calc_dist(Vector2d(0,0), Vector2d((last_xy(0)+form_xy(0))/2,(last_xy(1)+form_xy(1))/2));

            /* 判断上一簇是否符合landmark参数过滤阈值  */
            if(0.1 < dis && dis < radius_max_th && len >= landMark_min_pt && dis_from_robot < range_threshold){

                /* 进一步检查特征 */
                int feature_label = input.intensities[i-1]; // 获取上一簇label值(记为第n簇)
                // 检查第n-2，n-3簇的最后一个点是否距离第n簇第一个点距离过小
                for(int prev_2_label = feature_label-3;prev_2_label<=feature_label-2;prev_2_label++){
                    if(prev_2_label > 0){
                        int j = i;
                        while(input.intensities[j]!=prev_2_label){
                            j--;
                        }
                        Vector2d last_point;
                        angle = input.angle_min + j * input.angle_increment;
                        last_point << input.ranges[j] * std::cos(angle), input.ranges[j] * std::sin(angle);
                        double dis_2_pre = calc_dist(last_point, last_xy);
                        if(dis_2_pre < label_dr_threshold){
                            check_right = false;
                            break;
                        }
                    }                   
                }

                // 检查第n+2，n+3簇的第一个点是否距离第n簇最后一个点距离过小
                for(int after_2_label = feature_label+3;after_2_label>=feature_label+2;after_2_label--){
                    int k = i;
                    while(input.intensities[k]!=after_2_label){
                        k++;
                        if(k==total_num){
                            break;
                        }
                    }
                    if(k!=total_num){
                        Vector2d after_point;
                        angle = input.angle_min + k * input.angle_increment;
                        after_point << input.ranges[k] * std::cos(angle), input.ranges[k] * std::sin(angle);
                        double dis_2_after = calc_dist(after_point, form_xy);
                        if(dis_2_after < label_dr_threshold){
                            check_right = false;
                            break;
                        }                   
                    }
                }

                /* 如果检查通过，计算landmark圆心并存储 */
                if(check_right){
                    landMarks.id.push_back(cnt);
                    cnt++;

                    //最小二乘拟合簇圆心
                    std::vector<Eigen::Vector2d> points;//存储上一簇所有点
                    for(int j=0; j<len; j++){
                        Eigen::Vector2d temp;
                        angle = input.angle_min + (i-j-1) * input.angle_increment;
                        temp << input.ranges[i-j] * std::cos(angle), input.ranges[i-j] * std::sin(angle);
                        points.push_back(temp);
                    }
                    Eigen::Vector2d mean_pos = Eigen::Vector2d::Zero();
                    if(points.size() >= 3)  // 多于三个点可以做最小二乘拟合
                    {
                        mean_pos = fitcircle(points);
                        landMarks.position_x.push_back(mean_pos(0));
                        landMarks.position_y.push_back(mean_pos(1));
                    }else{
                        landMarks.position_x.push_back((last_xy(0)+form_xy(0))/2);
                        landMarks.position_y.push_back((last_xy(1)+form_xy(1))/2);
                    }

                    cout<<"id:"<<cnt<<" radius:"<<dis<<" len:"<<len<<" frombot:"<<dis_from_robot<<endl;                   
                }
                else{
                    check_right = true;
                }

            }

            last_xy = curr_xy;
            exp_label++;
            len = 1;
        }
        else{
            len++;
        }


    }
    return landMarks;
}

/**
 * Function: 最小二乘拟合圆，获取圆心，不计算半径
 * Input: points{一个簇的所有点}
 * Output: 圆心
 **/
Eigen::Vector2d extraction::fitcircle(std::vector<Eigen::Vector2d> &points)
{
    Eigen::Vector2d mean_pos;
    double X1=0, Y1=0;
    double X2=0, Y2=0;
    double X3=0, Y3=0;
    double X1Y1=0, X1Y2=0, X2Y1=0;

    for (int i=0; i<points.size(); i++)
    {
        X1 += points[i][0];
        Y1 += points[i][1];
        X2 += pow(points[i][0], 2);
        Y2 += pow(points[i][1], 2);
        X3 += pow(points[i][0], 3);
        Y3 += pow(points[i][1], 3);
        X1Y1 += points[i][0] * points[i][1];
        X1Y2 += points[i][0] * pow(points[i][1],2);
        X2Y1 += pow(points[i][0],2) * points[i][1];
    }

    double C,D,E,G,H,N;
    double a,b;
    N = points.size();
    C = N*X2 - X1*X1;
    D = N*X1Y1 - X1*Y1;
    E = N*X3 + N*X1Y2 - (X2+Y2)*X1;
    G = N*Y2 - Y1*Y1;
    H = N*X2Y1 + N*Y3 - (X2+Y2)*Y1;
    a = (H*D-E*G)/(C*G-D*D);
    b = (H*C-E*D)/(D*D-G*C);

    mean_pos(0) = a/(-2);
    mean_pos(1) = b/(-2);
}

/**
 * Function: Pub Landmark
 * Input: input{Landmark集合}
 **/
void extraction::publishLandMark(LandMarkSet input)
{
    if(input.id.size() <= 0)
        return;

    visualization_msgs::MarkerArray landMark_array_msg;
    landMark_array_msg.markers.resize(input.id.size());
    cout<<endl<<"---------------------------------All:"<<input.id.size()<<endl;

    for(int i=0; i<input.id.size(); i++)
    {
        landMark_array_msg.markers[i].header.frame_id = "course_agv__hokuyo__link";
        landMark_array_msg.markers[i].header.stamp = ros::Time(0);//ros::Time::now(); //ros::Time(0);
        landMark_array_msg.markers[i].ns = "lm";
        landMark_array_msg.markers[i].id = i;
        // landMark_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
        landMark_array_msg.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        landMark_array_msg.markers[i].action = visualization_msgs::Marker::MODIFY;
        landMark_array_msg.markers[i].pose.position.x = input.position_x.at(i);
        landMark_array_msg.markers[i].pose.position.y = input.position_y.at(i);
        landMark_array_msg.markers[i].pose.position.z = 0; // 2D
        landMark_array_msg.markers[i].pose.orientation.x = 0.0;
        landMark_array_msg.markers[i].pose.orientation.y = 0.0;
        landMark_array_msg.markers[i].pose.orientation.z = 0.0;
        landMark_array_msg.markers[i].pose.orientation.w = 1.0;
        landMark_array_msg.markers[i].scale.x = 0.2;
        landMark_array_msg.markers[i].scale.y = 0.2;
        // landMark_array_msg.markers[i].scale.z = 0.2;
        // landMark_array_msg.markers[i].color.a = 0.4; // Don't forget to set the alpha!
        landMark_array_msg.markers[i].scale.z = 0.8;
        landMark_array_msg.markers[i].color.a = 0.9;
        landMark_array_msg.markers[i].color.r = 0.0;
        landMark_array_msg.markers[i].color.g = 0.0;
        landMark_array_msg.markers[i].color.b = 1.0;

        ostringstream str;
        str<<i;
        landMark_array_msg.markers[i].text = str.str();

        cout<<"id: "<<i+1;
        cout<<"| x,y: "<<input.position_x.at(i)<<", "<<input.position_y.at(i)<<endl;
    }

    landMark_pub.publish(landMark_array_msg);
}

/**
 * Function: 计算2点的欧式距离
 **/
float extraction::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{
    return (pta - ptb).norm();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extraction");
    ros::NodeHandle n;

    extraction extraction_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    return 0;
}