#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>
#include <chrono>
#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/LinkStates.h>

#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Float64.h"

#include <iostream>
#include <iterator>
#include <random>
#include <ctime>

#include <geometry_msgs/Pose.h>
#include <ros/duration.h>
#include <numeric>

using namespace std;
using namespace Eigen;

#define particle_num 500

ostream& operator << (ostream &output, vector<double> &v) {
    for (auto i = v.begin(); i < v.end(); i++) {
        output << *i;
        if (i < v.end() - 1)
            output << " ";
    }
    return output;
}


typedef struct particle {
    int id;
    float x;
    float y;
    float theta;
    float weight;
} particle;

class particle_filter{

public:
    particle_filter(ros::NodeHandle &n);
	~particle_filter();
    ros::NodeHandle& n;

    // some params
    float init_x;
    float init_y;
    float init_theta;
    float init_rand_xy;
    float init_rand_theta;

    // subers & pubers
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber map_sub;
    ros::Subscriber real_pos_sub;
    ros::Publisher particles_pub;
    ros::Publisher odom_pub;
    ros::Publisher odom_pub_tf;
    ros::Publisher likeli_map_pub;
    ros::Publisher error_pub;
    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener listener;

    // particles 
    particle particles[particle_num];
    double new_ratio = 0.1;
    double N_thres;

    // global state
    Eigen::Vector3d state;
    double last_x = 0;
    double last_y = 0;
    double last_t = 0;
    // map
    nav_msgs::OccupancyGrid global_map;
    bool isMapSet = false;
    int map_height, map_width;
    float map_res;

    //likelihood map
    MatrixXd likeli_map;
    double std_map = 0.02;
    int propag_range = 5;
    double sensor_range = 10;

    //real pose
    double real_x = 0.0;
    double real_y = 0.0;
    double cnt = 0.0;
    double dx_mean = 0.0;
    double dy_mean = 0.0;

    //mutex
    int motion_cnt = 0;
    int observe_cnt = 0;

    double w_diff;

    // set map
    void setMap(nav_msgs::OccupancyGrid input);
    void PubLikeliMap();
    void genLikelihoodFeild(nav_msgs::OccupancyGrid input);
    double getPGaussian(double dx, double dy);
    //sub real pose
    void realPoseCallback(gazebo_msgs::LinkStates);
    // init the state and particles 
    void init();
    // do Motion from ICP odom
    void doMotion(nav_msgs::Odometry input);
    // do Observation to weight each particle
    void doObservation(sensor_msgs::LaserScan input);
    void Cal_weight(sensor_msgs::LaserScan input);
    // publish the particles and the estimated state
    void publishAll();
    // angle normalization
    double angleNorm(double angle);
    // weight normalization
    double weightNorm();
    //Calculate the information entropy of particles
    double calEntropy();
    // re-sample the particle according to the weights
    void resampling();
    // get the final pose 
    void getFinalPose();
    // gen new particle 
    particle genNewParticle();
};

particle_filter::~particle_filter()
{}

particle_filter::particle_filter(ros::NodeHandle& n):
    n(n)
{
    n.getParam("/particle_filter/init_x", init_x);
    n.getParam("/particle_filter/init_y", init_y);
    n.getParam("/particle_filter/init_theta", init_theta);

    n.getParam("/particle_filter/init_rand_xy", init_rand_xy);
    n.getParam("/particle_filter/init_rand_theta", init_rand_theta);

    n.getParam("/particle_filter/std_map", std_map);
    n.getParam("/particle_filter/propag_range", propag_range);

    n.getParam("/particle_filter/new_ratio", new_ratio);
    n.getParam("/particle_filter/N_thres", N_thres);

    this->init();

    particles_pub = n.advertise<visualization_msgs::MarkerArray>("particles", 0, true);
    likeli_map_pub = n.advertise<nav_msgs::OccupancyGrid>("likeli_map", 1);
    odom_pub = n.advertise<nav_msgs::Odometry>("pf_odom", 1);
    map_sub = n.subscribe("/map", 1, &particle_filter::setMap, this);
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &particle_filter::doObservation, this);
    odom_sub = n.subscribe("/icp_odom", 1, &particle_filter::doMotion, this);
    odom_pub_tf = n.advertise<geometry_msgs::Pose>("pf_odom_tf", 1);
    error_pub = n.advertise<std_msgs::Float64>("particle_error", 1);
    real_pos_sub = n.subscribe("/gazebo/link_states", 1, &particle_filter::realPoseCallback, this);
}

void particle_filter::realPoseCallback(gazebo_msgs::LinkStates link){
    string link_name = "course_agv::robot_base";
    if(link.name[1].compare(link_name) == 0){
        this->real_x = link.pose[1].position.x;
        this->real_y = link.pose[1].position.y;
    }
}

void particle_filter::setMap(nav_msgs::OccupancyGrid input)
{   
    // set once at first time
    if(!isMapSet)
    {   
        cout<<"init the global occupancy grid map"<<endl;
        this->global_map = input;
        isMapSet = true;

        map_height = input.info.height;
        map_width = input.info.width;
        map_res = input.info.resolution;
        
        likeli_map = MatrixXd::Constant(map_width, map_height, 0.0);
        genLikelihoodFeild(input);
    }
}

void particle_filter::genLikelihoodFeild(nav_msgs::OccupancyGrid input){
    for(int i = 0; i < map_height; i++){
        for(int j = 0; j < map_width; j++){
            if(input.data[j*input.info.width+i] != 0){
                int black_cnt = 0;
                for(int p = i-1; p <= i+1; p++){
                    for(int q = j-1; q <= j+1; q++){
                        if(p<0 || q<0 || p>=map_height || q>=map_width){
                            continue;
                        }
                        if(input.data[q*input.info.width+p] == 100){
                            black_cnt++;
                        }
                    }
                }
                if(black_cnt>=7){
                    continue;
                }
                for(int m = i-propag_range; m < i+propag_range; m++){
                    for(int n = j-propag_range; n < j+propag_range; n++){
                        if(m<0 || n<0 || m>=map_height || n>=map_width){
                            continue;
                        }
                        likeli_map(m,n) = max(getPGaussian((m-i)*map_res, (n-j)*map_res), likeli_map(m,n));
                        // if(abs(m-i)+abs(n-j) == 1){
                        //     likeli_map(m,n) = 1.0;
                        // }
                    }
                }
            }
        }
    }
    PubLikeliMap();
}

double particle_filter::getPGaussian(double dx, double dy){
    double dis = sqrt(double(dx*dx+dy*dy));
    return exp(-0.5*dis*dis/std_map);
}

void particle_filter::PubLikeliMap(){
    nav_msgs::OccupancyGrid grid_map;
    // iniitialization
    grid_map.info.height = map_height;
    grid_map.info.width = map_width;
    grid_map.info.resolution = map_res;
    grid_map.header.frame_id = "map";

    // set origin of map
    grid_map.info.origin.position.x = - float(grid_map.info.width) / 2 * grid_map.info.resolution;
    grid_map.info.origin.position.y = - float(grid_map.info.height) / 2 * grid_map.info.resolution;
    grid_map.info.origin.orientation.w = 1;

    // fill with -1 / unknown in the map
    grid_map.data.assign(map_width * map_height, -1);

    double max=0;
    for(int i = 0; i < map_height; i++){
        for(int j = 0; j < map_width; j++){
            if(max<likeli_map(i,j)){
                max = likeli_map(i,j);
            }
        }
    }
    for(int i = 0; i < map_height; i++){
        for(int j = 0; j < map_width; j++){
            grid_map.data[j*grid_map.info.width+i] = floor(likeli_map(i,j)*100.0/max);
        }
    }
    likeli_map_pub.publish(grid_map);
    cout<<"likeli_map_pub"<<endl;
}


void particle_filter::init()
{   
    // set state
    state << 0, 0, 0;
    srand(time(0));

    for(int i=0; i<particle_num; i++)
    {   
        particles[i].id = i;
        particles[i].x = init_x + (float(rand()) / float(RAND_MAX)) * 2 * init_rand_xy - init_rand_xy;
        // if(particles[i].x > 10.0 || particles[i].x < -10.0){
        //     particles[i].x = init_x;
        // }
        // if(particles[i].y > 10.0 || particles[i].y < -10.0){
        //     particles[i].y = init_y;
        // }
        particles[i].y = init_y + (float(rand()) / float(RAND_MAX)) * 2 * init_rand_xy - init_rand_xy;
        particles[i].theta = init_theta + (float(rand()) / float(RAND_MAX)) * 2 * init_rand_theta - init_rand_theta;
        particles[i].weight = float(1/float(particle_num)); // same weight
    }
    cout<<"PF INIT DONE"<<endl;
}



void particle_filter::doMotion(nav_msgs::Odometry odom)
{   
    // cout<<"-------------------------------------doing Motion"<<endl;
    // TODO: Motion Model
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
    //注意，此处dt = nowt - last_t会造成角度突变，例如dt=-0.01==+6.27
    if(dt>M_PI){
        dt -= 2*M_PI;
    }
    else if(dt<-M_PI){
        dt += 2*M_PI;
    }
    // cout<<"dt:"<<dt<<endl;

    last_x = nowx;
    last_y = nowy;
    last_t = nowt;

    // Define random generator with Gaussian distribution
    const double mean = 0.0;//均值
    const double stdx = dx*1;//标准差
    const double stdy = dy*1;//标准差
    const double stdt = dt*0.1;//标准差

    unsigned seed=std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> noisex(mean, stdx);
    std::normal_distribution<double> noisey(mean, stdy);
    std::normal_distribution<double> noiset(mean, stdt);

    for(int i=0; i<particle_num; i++)
    {   
        particles[i].id = i;
        particles[i].x += dx + noisex(generator);
        particles[i].y += dy + noisey(generator);
        particles[i].theta += dt + noiset(generator);
        //particles[i].theta = angleNorm(particles[i].theta);
    }   
    getFinalPose();
    publishAll();
    motion_cnt++;
}

void particle_filter::doObservation(sensor_msgs::LaserScan input)
{   
    if(observe_cnt >= motion_cnt)
        return;

    cout<<endl<<"-----------------------------doing observation"<<endl;
    // TODO: Measurement Model

    Cal_weight(input);
    weightNorm();
    double w_mean = 1.0 / particle_num;

    /* 重采样 */
    double Neff;
    double w_sqsum = 0.0;
    for(int i=0; i<particle_num; i++){
        w_sqsum += (particles[i].weight-w_mean)*(particles[i].weight-w_mean);
    }
    Neff = 1/w_sqsum;
    cout<<"Neff:"<<Neff<<endl;

    if(Neff < N_thres){
        cout<<"Resample........."<<endl;
        resampling();
        Cal_weight(input);
        weightNorm();
    }



    double H = calEntropy();
    cout<<"Entropy: "<<H<<endl;
    w_diff = H;
    
    getFinalPose();
    publishAll();

    observe_cnt++;
}

void particle_filter::Cal_weight(sensor_msgs::LaserScan input){
    int laser_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;

    /* 计算每个粒子权重 */
    for(int i=0; i<particle_num; i++){
        double p_all = 0.0;
        int laser_cnt = 0;
        // cout<<"-------------------------------------PARTICLE "<<i<<endl;
        for(int k = 0; k < laser_num; k++){
            if(input.ranges[k] >= sensor_range){
                continue;
            }
            Vector2d g_laser;
            double r_angle = input.angle_min + k*input.angle_increment;
            g_laser(0) = particles[i].x + input.ranges[k]*cos(particles[i].theta+r_angle) + 10.0;
            g_laser(1) = particles[i].y + input.ranges[k]*sin(particles[i].theta+r_angle) + 10.0;
            if(g_laser(0)<0||g_laser(1)<0||g_laser(0)>20||g_laser(1)>20)
                continue;

            p_all += likeli_map(int(g_laser(0)/map_res), int(g_laser(1)/map_res));
            // cout<<"li:"<<likeli_map(int(g_laser(0)/map_res), int(g_laser(1)/map_res))<<endl;
            // cout<<"lixy:" << int(g_laser(0)/map_res) << "," << int(g_laser(1)/map_res) <<endl;
            laser_cnt++;
        }
        // cout<<"pall:"<<p_all<<endl;
        double p_avg = p_all/(1.0*laser_cnt);
        if(laser_cnt == 0){
            p_avg = 0;
        }
        if(p_avg>10000){
            while(1){}
        }
        particles[i].weight = p_avg;
        // cout<<"p_avg:"<<p_avg<<"| weight:"<<particles[i].weight<<endl<<endl;
    }
}

void particle_filter::resampling()
{
    // TODO: Resampling Step
    vector<double> w_step;
    double w_sum = 0;
    for(int i=0; i<particle_num; i++){
        w_sum += particles[i].weight;
        w_step.push_back(w_sum);
    }

    particle new_particles[particle_num];
    int resample_num = particle_num-floor(new_ratio*particle_num);
    srand(time(0));
    for(int i=0; i<resample_num; i++){
        
        double randw = rand()%1000/1000.0;
        w_step.push_back(randw);
        sort(w_step.begin(), w_step.end());
        vector<double>::iterator it=find(w_step.begin(),w_step.end(),randw);///返回的是地址
        int index=std::distance(w_step.begin(), it);///放入迭代器中得到容器中的位置
        w_step.erase(it);


        new_particles[i].id = i;
        new_particles[i].x = particles[index].x;
        new_particles[i].y = particles[index].y;
        new_particles[i].theta = particles[index].theta;
        new_particles[i].weight = particles[index].weight;
    }

    for(int i=0; i<particle_num - resample_num; i++){
        new_particles[resample_num+i] = genNewParticle();
    }
    memcpy(particles,new_particles,sizeof(new_particles));
}

void particle_filter::getFinalPose()
{   
    // TODO: Final State Achieve
    state(0) = 0.0;
    state(1) = 0.0;
    state(2) = 0.0;

    for(int i=0; i<particle_num; i++){
        state(0) += particles[i].x*particles[i].weight;
        state(1) += particles[i].y*particles[i].weight;
        state(2) += particles[i].theta*particles[i].weight;
        // if(i==particle_num-1)
            // cout<<"thetai:"<<particles[i].theta<<endl;
    }
    // state(2) = angleNorm(state(2));
    // cout<<"finalstate:"<<state.transpose()<<endl;

    double dx = fabs(real_x - state(0));
    double dy = fabs(real_y - state(1));

    cnt++;
    dx_mean = (dx_mean * cnt + dx)/cnt;
    dy_mean = (dy_mean * cnt + dy)/cnt;

    std_msgs::Float64 err;
    err.data = sqrt(dx*dx+dy*dy);
    cout<<"current_error: "<<err.data<<endl;
    //cout<<"Mean_err: "<<sqrt(dx_mean*dx_mean+dy_mean*dy_mean)<<endl;
    error_pub.publish(err);
}

particle particle_filter::genNewParticle()
{
    // TODO: Generate New Particle
    // Define random generator with Gaussian distribution
    const double mean = 0.0;//均值
    // const double stdxy = 1;//标准差
    // const double stdt = 0.1;//标准差
    const double stdxy = 3;//标准差
    const double stdt = 0.3;//标准差

    unsigned seed=std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);

    std::normal_distribution<double> noisexy(mean, stdxy);
    std::normal_distribution<double> noiset(mean, stdt);

    particle newp;
    newp.x = state(0) + noisexy(generator);
    newp.y = state(1) + noisexy(generator);
    while(newp.x > 8.0 || newp.x < -8.0){
        newp.x = state(0) + noisexy(generator);
    }
    while(newp.y > 8.0 || newp.y < -8.0){
        newp.y = state(1) + noisexy(generator);
    }
    newp.theta = state(2) + noiset(generator);
    newp.weight=1.0/particle_num;
    //newp.theta = angleNorm(newp.theta);
    return newp;
}

double particle_filter::weightNorm(){
    double weight_sum = 0.0;
    for(int i=0; i<particle_num; i++){
        weight_sum += particles[i].weight;
    }
    for(int i=0; i<particle_num; i++){
        particles[i].weight = particles[i].weight/weight_sum;
    }
    return weight_sum;
}

double particle_filter::calEntropy(){
    double H = 0.0;
    // for(int i=0; i<particle_num; i++){
    //     double w = particles[i].weight;
    //     H += -w*log2(w);
    // }
    vector<double> w_step;
    for(int i=0; i<particle_num; i++){
        w_step.push_back(particles[i].weight);
    }
    sort(w_step.begin(), w_step.end());

    vector<double> w_max;
    vector<double> w_min;
    double w_num = 10;
    for(int i = 0; i < int(w_num); i++){
        w_min.push_back(w_step[i]);
        w_max.push_back(w_step[w_step.size()-i-1]);
    }
    double w_max_mean = accumulate(w_max.begin(), w_max.end(),0.0);
    double w_min_mean = accumulate(w_min.begin(), w_min.end(),0.0);

    H = w_max_mean - w_min_mean;
        
    return H*100.0;
}

double particle_filter::angleNorm(double angle)
{
    // -180 ~ 180
	while (angle > 2*M_PI)
		angle = angle - 2 * M_PI;
	while (angle < 0)
		angle = angle + 2 * M_PI;
	return angle;
}


void particle_filter::publishAll()
{
    visualization_msgs::MarkerArray particle_markers_msg;
    particle_markers_msg.markers.resize(particle_num);

    for(int i=0; i<particle_num; i++)
    {
        particle_markers_msg.markers[i].header.frame_id = "map";
        particle_markers_msg.markers[i].header.stamp = ros::Time::now();
        particle_markers_msg.markers[i].ns = "particle";
        particle_markers_msg.markers[i].id = i;
        particle_markers_msg.markers[i].type = visualization_msgs::Marker::ARROW;
        particle_markers_msg.markers[i].action = visualization_msgs::Marker::ADD;
        particle_markers_msg.markers[i].pose.position.x = particles[i].x;
        particle_markers_msg.markers[i].pose.position.y = particles[i].y;
        particle_markers_msg.markers[i].pose.position.z = 0; // add height for viz ?
        particle_markers_msg.markers[i].pose.orientation.x = 0.0;
        particle_markers_msg.markers[i].pose.orientation.y = 0.0;
        particle_markers_msg.markers[i].pose.orientation.z = sin(particles[i].theta/2);
        particle_markers_msg.markers[i].pose.orientation.w = cos(particles[i].theta/2);
        particle_markers_msg.markers[i].scale.x = 0.1;
        particle_markers_msg.markers[i].scale.y = 0.02;
        particle_markers_msg.markers[i].scale.z = 0.05;
        // particle_markers_msg.markers[i].color.a = particles[i].weight * particle_num / 2; // Don't forget to set the alpha!
       particle_markers_msg.markers[i].color.a = 0.5;
        particle_markers_msg.markers[i].color.r = 1.0;
        particle_markers_msg.markers[i].color.g = 0.0;
        particle_markers_msg.markers[i].color.b = 0.0;
    }
    particles_pub.publish(particle_markers_msg);

    // tf
    geometry_msgs::Quaternion quat_ = tf::createQuaternionMsgFromYaw(state(2));

    geometry_msgs::TransformStamped pf_trans;
    pf_trans.header.stamp = ros::Time::now();
    pf_trans.header.frame_id = "map";
    pf_trans.child_frame_id = "pf_loc";

    pf_trans.transform.translation.x = state(0);
    pf_trans.transform.translation.y = state(1);
    pf_trans.transform.translation.z = 0.0;
    pf_trans.transform.rotation = quat_;
    tf_broadcaster.sendTransform(pf_trans);

    // OR publish others you want
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";//"world_base";

    odom.pose.pose.position.x = state(0);
    odom.pose.pose.position.y = state(1);
    odom.pose.pose.position.z = 0.0;
    if(w_diff < 2.5){
        odom.pose.pose.position.z = 1.0;
    }
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state(2));
    odom.pose.pose.orientation = odom_quat;

    odom_pub.publish(odom);

    //For robo_tf node
    geometry_msgs::Pose odom_tf;
    odom_tf.position.x = state(0);
    odom_tf.position.y = state(1);
    odom_tf.position.z = 0.0;
    odom_tf.orientation = odom_quat;

    odom_pub_tf.publish(odom_tf);

}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;
    ros::Duration(2.0).sleep();

    particle_filter particle_filter_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}
