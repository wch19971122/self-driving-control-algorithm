#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Float64.h>
#include <LQR/controlmsg.h>
#include <LQR/state.h>

#ifndef POINT
#define POINT
struct  Point
{
    double x;
    double y;
};
#endif

constexpr double pi = 3.1415;

double normalize_angle(double angle)//角度归一化 [-pi,pi];
{
    if (angle > pi) {
        angle -= 2.0 * pi;
    }
    if (angle <= -pi) {
        angle += 2.0 * pi;
    }
    return angle;
}

class Carmodel{
public:
    Carmodel();
    void init();
    void update(double steer_angle, double acc);
    void getPolygon(geometry_msgs::PolygonStamped &polygon, geometry_msgs::Point32& point);
    void lqrctrlCallback(const LQR::controlmsg::ConstPtr& msg);

private:
    Point center;//质心坐标
    Point lf, lr, rf, rr; // 轮廓点坐标
    double length, width;//车的box参数
    double Caf,Car; //前后轴侧偏刚度
    double a,b; //质心到前后轴的距离
    double m; //质量
    double Iz;  //绕z轴转动惯量
    double velocity; //速度
    double theta; //航向角
    double steering_angle; //前轮转角
    double dertaT = 0.01;//仿真步长

    ros::NodeHandle n;
    ros::Rate rate = ros::Rate(100);
    ros::Subscriber update_sub;
    ros::Publisher state_pub;
    ros::Publisher polygon_pub;
};

Carmodel::Carmodel(){
    center.x = -51;
    center.y = -2;
    velocity = 0.01;
    theta = 0;
    steering_angle = 0;
    a = 1.265;
    b = 1.682;
    m = 2020;
    length = 5.2;
    width  = 1.8;
    Iz = 4095;
    Caf = -175016;
    Car = -130634;
    update_sub = n.subscribe<LQR::controlmsg>("lqr_ctrl", 1, &Carmodel::lqrctrlCallback,this);
    state_pub = n.advertise<LQR::state>("lqr_state",1);
    polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("lqr_polygon_pub",1);
}

void Carmodel::init(){
    this -> lf.x = center.x + length/2*cos(theta) - width/2*sin(theta);
    this ->lf.y = center.y + length/2*sin(theta) +width/2*cos(theta);
    this ->rf.x = center.x + length/2*cos(theta) + width/2*sin(theta);
    this ->rf.y = center.y + length/2*sin(theta) - width/2*cos(theta);
    this ->lr.x = center.x - length/2*cos(theta) - width/2*sin(theta);
    this ->lr.y = center.y - length/2*sin(theta) + width/2*cos(theta);
    this ->rr.x = center.x - length/2*cos(theta) + width/2*sin(theta);
    this ->rr.y = center.y - length/2*sin(theta) - width/2*cos(theta);
}

void Carmodel::update(double steer_angle, double acc){
    //theta = normalize_angle(theta);
    ROS_INFO("steer_angle = %f, acc = %f",steer_angle, acc);

    this -> center.x = center.x + velocity *cos(theta) * dertaT;
    this ->center.y = center.y + velocity * sin(theta) * dertaT;

    this ->steering_angle = steer_angle;
    this ->theta =normalize_angle(theta + velocity * tan(steer_angle) * dertaT/length);
    this ->velocity = velocity + acc * dertaT;
    ROS_INFO("center.X = %f, center.Y = %f",center.x,center.y);
}

void Carmodel::getPolygon(geometry_msgs::PolygonStamped &polygon, geometry_msgs::Point32& point){
    point.x = lf.x;
    point.y = lf.y;
    point.z = 0;
    //ROS_INFO("lf = [%f,%f]",point.x,point.y);
    polygon.polygon.points.push_back(point);
    point.x = rf.x;
    point.y = rf.y;
    point.z = 0;
    //ROS_INFO("rf = [%f,%f]",point.x,point.y);
    polygon.polygon.points.push_back(point);
    point.x = rr.x;
    point.y = rr.y;
    point.z = 0;
    //ROS_INFO("rr = [%f,%f]",point.x,point.y);
    polygon.polygon.points.push_back(point);
    point.x = lr.x;
    point.y = lr.y;
    point.z = 0;
    //ROS_INFO("lr = [%f,%f]",point.x,point.y);
    polygon.polygon.points.push_back(point);
}

void Carmodel::lqrctrlCallback(const LQR::controlmsg::ConstPtr& msg){
    double steer_angle = msg ->steering_angle;
    double acc = msg -> acc;
    update(steer_angle,acc);
    init();
    ROS_INFO("State has update !" );
    LQR::state state = LQR::state();
    state.centerX = center.x;
    state.centerY = center.y;
    state.theta = theta;
    state.steering_angle = steer_angle;
    state.velocity = velocity;
    state_pub.publish(state);
    ros::spinOnce();

    geometry_msgs::PolygonStamped myPolygon;
    geometry_msgs::Point32 point;
    getPolygon(myPolygon,point);
    myPolygon.header.frame_id = "map";
    polygon_pub.publish(myPolygon);
    ROS_INFO("update has finished!");
    rate.sleep();
    
}


int main(int argc, char** argv){
    ros::init(argc, argv,"lqr_carmodel");
    ros::NodeHandle n;
    Carmodel carmodel;
    ros::spin();
    return 0;
}