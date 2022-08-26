#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pure_persuit/state.h>
#include <pure_persuit/controlmsg.h>

using namespace std;

struct Point{
    double x;
    double y;    
};

class Car{
public:
    Car(){};
    Car(double _x, double _y,  double _theta, double _velocity,   double _width, double _length)
        :centerX(_x),  centerY(_y),    theta(_theta),  velocity(_velocity), width(_width),    length(_length){};
    void init();
    void getPolygon(geometry_msgs::PolygonStamped &polygon, geometry_msgs::Point32& point);
    void update(double steer_angle,double acc);

    double dertaT = 0.01;
    Point lf, rf, lr, rr;
    double centerX, centerY;
    double rareX,rareY;
    double theta; //弧度表示
    double velocity;
    double width,length;
};

void Car :: init(){
                lf.x = centerX + length/2*cos(theta) - width/2*sin(theta);
                lf.y = centerY + length/2*sin(theta) +width/2*cos(theta);
                rf.x = centerX + length/2*cos(theta) + width/2*sin(theta);
                rf.y = centerY + length/2*sin(theta) - width/2*cos(theta);

                lr.x = centerX - length/2*cos(theta) - width/2*sin(theta);
                lr.y = centerY - length/2*sin(theta) + width/2*cos(theta);
                rr.x = centerX - length/2*cos(theta) + width/2*sin(theta);
                rr.y = centerY - length/2*sin(theta) - width/2*cos(theta);
                rareX = centerX - width/2 * cos(theta);
                rareY = centerY - width/2 * sin(theta);
}

void Car::getPolygon(geometry_msgs::PolygonStamped &polygon, geometry_msgs::Point32& point){
    point.x = lf.x;
    point.y = lf.y;
    point.z = 0;
    polygon.polygon.points.push_back(point);
    point.x = rf.x;
    point.y = rf.y;
    point.z = 0;
    polygon.polygon.points.push_back(point);
    point.x = rr.x;
    point.y = rr.y;
    point.z = 0;
    polygon.polygon.points.push_back(point);
    point.x = lr.x;
    point.y = lr.y;
    point.z = 0;
    polygon.polygon.points.push_back(point);
}

void Car::update(double steer_angle,  double acc){
    ROS_INFO("velocity = %f",velocity);
    rareX = rareX + velocity * cos(theta) * dertaT;
    rareY = rareY + velocity * sin(theta) * dertaT;
    centerX = rareX + width/2*cos(theta);
    centerY = rareY + width/2*sin(theta);
    theta = theta + velocity * tan(steer_angle) * dertaT/length;
    velocity = velocity + acc * dertaT;
}

class CarModel{
public:
    CarModel(){
        car = Car(0,0,0,2,2,4);
        car.init();
        state_pub = n.advertise<pure_persuit::state>("state_change",1);
        ctrl_sub = n.subscribe<pure_persuit::controlmsg>("pp_ctrl",    1,  &CarModel::updateCallback,    this);
        polygon_pub  = n.advertise<geometry_msgs::PolygonStamped>("Polygonpublisher",1);
    };

    void updateCallback(const pure_persuit::controlmsg::ConstPtr& msg){
        double steer_angle = msg ->derta;
        double acc = msg -> acc;
        car.update(steer_angle,acc);
        car.init();
        ROS_INFO("State has update!");
        pure_persuit::state state = pure_persuit::state();
        state.rareX = car.rareX;
        state.rareY = car.rareY;
        state.theta = car.theta;
        state.velocity = car.velocity;
        state_pub.publish(state);

        geometry_msgs::PolygonStamped myPolygon;
        geometry_msgs::Point32 point;
        car.getPolygon(myPolygon,point);
        myPolygon.header.frame_id = "map";
        polygon_pub.publish(myPolygon);
        //ROS_INFO("Initial state has published!");
        rate.sleep();
    }
    Car get(){return car;}

    Car car;
    ros::NodeHandle n;
    ros::Publisher polygon_pub;
    ros::Publisher state_pub;
    ros::Subscriber ctrl_sub;
    ros::Rate rate = ros::Rate(100);
};

int main(int argc, char** argv){
    ros::init(argc,argv,"car_model");
    ros::NodeHandle n;
    CarModel carmodel;

    ros::spin();
    return 0;
}
