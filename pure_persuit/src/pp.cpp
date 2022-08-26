#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Float64.h>
#include <pure_persuit/state.h>
#include <pure_persuit/referenceline.h>
#include <pure_persuit/referencelinePoint.h>
#include <pure_persuit/controlmsg.h>

class PP{
public:
    PP(){
        ref_sub = n.subscribe<pure_persuit::referenceline>("referenceline_pub",1,&PP::reflineCallback,  this);

        state_sub = n.subscribe<pure_persuit::state>("state_change", 1, &PP::calculateCtrlCallback,   this);

        ctrl_pub = n.advertise<pure_persuit::controlmsg>("pp_ctrl",1);
    }
    int findPrePointIndex(double Ld, int startIndex);
    int findMatchPointIndex(pure_persuit::state state);
    void calculateCtrlCallback(const pure_persuit::state::ConstPtr &msg);
    void reflineCallback(const pure_persuit::referenceline::ConstPtr &msg);

private:
    pure_persuit::referenceline referenceline;
    ros::NodeHandle n;
    ros::Rate rate = ros::Rate(100);
    ros::Publisher ctrl_pub;
    ros::Subscriber state_sub;
    ros::Subscriber ref_sub;
    double ref_velocity = 10;
    double dertaT = 0.01;
    int preIndex = 1;
};

void PP::reflineCallback(const pure_persuit::referenceline::ConstPtr &msg){
    for(auto point : msg -> referencelinePoints){
        referenceline.referencelinePoints.push_back(point);    
    }
}

int PP::findMatchPointIndex(pure_persuit::state state){
    double minindex = 0;
    double minLength = pow(state.rareX - referenceline.referencelinePoints[0].xr,2) + pow(state.rareY - referenceline.referencelinePoints[0].yr,2);
    int count = 0;
    for(int i = preIndex; i < referenceline.referencelinePoints.size(); i++){
        double curLength = pow(state.rareX - referenceline.referencelinePoints[i].xr,2) + pow(state.rareY - referenceline.referencelinePoints[i].yr,2);
        if(curLength < minLength){
            minLength = curLength;
            minindex = i;
            count = 0;
        } else{
            count++;
            if(count > 50){
                break;
            }
        }
    } 
    preIndex = minindex;
    return minindex;
}

void PP::calculateCtrlCallback(const pure_persuit::state::ConstPtr &msg){
    pure_persuit::state state = pure_persuit :: state();
    state.rareX = msg ->rareX;
    state.rareY = msg -> rareY;
    state.theta = msg -> theta;
    state.velocity = msg -> velocity;
    int minIndex = findMatchPointIndex(state);
    ROS_INFO("minIndex = %d",minIndex );
    double Ld = 2;

    int preIndex = findPrePointIndex(Ld, minIndex);
    ROS_INFO("preIndex = %d",preIndex );
    pure_persuit::referencelinePoint prePoint = referenceline.referencelinePoints[preIndex];

    double alpha = atan2(prePoint.yr - state.rareY,prePoint.xr - state.rareX) - state.theta;

    ROS_INFO("alpha = %f",alpha);
    double derta = atan2(2 * 4 * sin(alpha),Ld);
    ROS_INFO("derta = %f",derta);

    double kp = 0.03;
    double a;
    if(referenceline.referencelinePoints[minIndex].kr > 0.1) {
        a = kp * (3 -state.velocity) / dertaT;
    }else{
        a = kp * (ref_velocity -state.velocity) / dertaT;
    }
    //a = kp * (ref_velocity -state.velocity) / dertaT;
    
    

    pure_persuit::controlmsg reMsg;
    reMsg.derta = derta;
    reMsg.acc = a;
    ctrl_pub.publish(reMsg);
    rate.sleep();
}

//找到预瞄点
int PP::findPrePointIndex(double Ld, int startIndex){
    double s = 0;
    for(int i = startIndex+1; i < referenceline.referencelinePoints.size(); i++){
        pure_persuit::referencelinePoint point = referenceline.referencelinePoints[i];
        pure_persuit::referencelinePoint prePoint = referenceline.referencelinePoints[i-1];
        s += sqrt(pow(point.xr - prePoint.xr,2) + pow(point.yr - prePoint.yr,2));
        ROS_INFO("S = %f",s);
        if(s >= Ld){
            return i;
        }
    }
    return 0;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "pp_ctrl");

    PP pp;

    ros::spin();

    return 0;
}

