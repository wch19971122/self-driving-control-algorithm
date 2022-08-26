#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Float64.h>
#include <MPC/controlmsg.h>
#include <MPC/state.h>


constexpr double pi = 3.1415926;
struct Point{
    double x;
    double y;
};

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

double limit_kesi(double kesi) {
	if (kesi > pi / 2) {
		kesi = pi /2;
	}
	if (kesi < -pi / 2) {
		kesi = -pi / 2;
	}
	return kesi;
}

class CarModel{
public:
    CarModel(){
        center.x = -51; center.y = 0;
        velocity = 10;
        theta = 0;steering_angle = 0.01;
        a = 1.265;b = 1.682;m = 2020;
        length = 5.2;width  = 1.8;
        Iz = 4095;Caf = -175016;Car = -130634;
        ROS_INFO("init car!!!");
    };
    void init(){
    this -> lf.x = center.x + length/2*cos(theta) - width/2*sin(theta);
    this ->lf.y = center.y + length/2*sin(theta) +width/2*cos(theta);
    this ->rf.x = center.x + length/2*cos(theta) + width/2*sin(theta);
    this ->rf.y = center.y + length/2*sin(theta) - width/2*cos(theta);
    this ->lr.x = center.x - length/2*cos(theta) - width/2*sin(theta);
    this ->lr.y = center.y - length/2*sin(theta) + width/2*cos(theta);
    this ->rr.x = center.x - length/2*cos(theta) + width/2*sin(theta);
    this ->rr.y = center.y - length/2*sin(theta) - width/2*cos(theta);
    }
    void update(double steer_angle, double acc){
    //theta = normalize_angle(theta);
    ROS_INFO("steer_angle = %f, acc = %f",steer_angle, acc);

    this -> center.x = center.x + velocity *cos(theta) * dertaT;
    this ->center.y = center.y + velocity * sin(theta) * dertaT;

    this ->steering_angle = limit_kesi(steering_angle+steer_angle);
    this ->theta =normalize_angle(theta + velocity * tan(steer_angle) * dertaT/length);
    this ->velocity = velocity + acc * dertaT;
    ROS_INFO("center.X = %f, center.Y = %f",center.x,center.y);
    }
    void getPolygon(geometry_msgs::PolygonStamped &polygon, geometry_msgs::Point32& point){
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
    double dertaT = 0.01;
    friend class CarNode;
};

class CarNode{
public:
    CarNode(){
        carmodel.init();
        ctrl_sub = n.subscribe<MPC::controlmsg>("mpc_ctrl",1,   &CarNode::ctrlCallback,this);
        state_pub = n.advertise<MPC::state>("mpc_state",1);
        polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("mpc_polygon",1);
    };

    void ctrlCallback(const MPC::controlmsg::ConstPtr& msg){
        double steering_angle = msg ->steering_angle;
        double acc = msg -> acc;
        carmodel.update(steering_angle,acc);
        carmodel.init();
        ROS_INFO("state has update!");

        MPC::state  state = MPC::state();
        state.centerX = carmodel.center.x;
        state.centerY = carmodel.center.y;
        state.steering_angle = carmodel.steering_angle;
        state.theta = carmodel.theta;
        state.velocity = carmodel.velocity;
        state_pub.publish(state);

        geometry_msgs::PolygonStamped polygon;
        geometry_msgs::Point32 point;
        polygon.header.frame_id = "map";
        carmodel.getPolygon(polygon,point);
        polygon_pub.publish(polygon);

        rate.sleep();
    }



private:
    CarModel carmodel;
    ros::NodeHandle n;
    ros::Rate rate = ros::Rate(100);
    ros::Subscriber ctrl_sub;
    ros::Publisher state_pub;
    ros::Publisher polygon_pub;
};



int main(int argc, char** argv){
    ros::init(argc,argv,"mpc_carmodel");
    CarNode carNode;
    ros::spin();
    return 0;
}