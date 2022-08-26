#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <LQR/referenceline.h>
#include <LQR/referencelinePoint.h>
#include <LQR/state.h>
#include <LQR/controlmsg.h>

constexpr double    pi = 3.14159;
constexpr double    Caf = -175016;
constexpr double    Car = -130634;
constexpr double    a = 1.265;
constexpr double    b = 1.682;
constexpr double    m = 2020;
constexpr double    Iz = 4095;


double limit_kesi(double kesi) {
	if (kesi > pi / 2) {
		kesi = pi /2;
	}
	if (kesi < -pi / 2) {
		kesi = -pi / 2;
	}
	return kesi;
}

class LQR_control{
public:
    LQR_control(){
        ROS_INFO("LQR initial!");
        ref_sub = n.subscribe<LQR::referenceline>("referenceline_pub",1,&LQR_control::reflineCallback,  this);
        update_pub = n.advertise<LQR::controlmsg>("lqr_ctrl",1);
        state_sub = n.subscribe<LQR::state>("lqr_state", 1, &LQR_control::lqrctrlCallback,this);     
    };
    void reflineCallback(const LQR::referenceline::ConstPtr& msg);
    void lqrctrlCallback(const LQR::state::ConstPtr& msg);
    int findMatchPoint(LQR::state state);
    Eigen::Matrix<double,4,1> calcErr(int index, LQR::state state);
    Eigen::Matrix<double,4,1> calcK(LQR::state state);
    Eigen::Matrix<double,4,1> dlqr(Eigen::Matrix4d A, Eigen::Matrix<double,4,1> B,Eigen::Matrix4d Q,Eigen::Matrix<double,1,1> R);
    double longitudinalControl(LQR::state state);
    double lateralControl(LQR::state state);
    double calc_forword_angle(Eigen::Matrix<double,4,1> k, Eigen::Matrix<double,4,1> err,double index,LQR::state state);
    
private:
    double dertaT = 0.01;
    double ref_velocity = 20;
    double predict_time = 0.01;
    double max_acc = 5;
    double max_dec = -3;
    LQR::referenceline referenceline;

    ros::NodeHandle n;
    ros::Rate rate = ros::Rate(100);
    ros::Publisher update_pub;
    ros::Subscriber state_sub;
    ros::Subscriber ref_sub;
};

void LQR_control :: lqrctrlCallback(const LQR::state::ConstPtr& msg){
    LQR::state state = LQR::state();
    //预瞄时间0.01s
    state.centerX = msg -> centerX + msg -> velocity *cos(msg -> theta)*predict_time;
    state.centerY = msg -> centerY + msg -> velocity *sin(msg -> theta)*predict_time;
    state.theta = msg -> theta;
    state.steering_angle = msg -> steering_angle;
    state.velocity = msg -> velocity;

    ROS_INFO("--------------------------    LQR_catc has start!    -------------------------------");
    ROS_INFO("cur_state X= %f, cur_state Y = %f",state.centerX,state.centerY);
    ROS_INFO("cur_state theta= %f, cur_state velocity = %f",state.theta,state.velocity);
    double steer_angle =limit_kesi(lateralControl(state));
    ROS_INFO("steer_angle has calced! steer_angle = %f",steer_angle);
    
    double acc = longitudinalControl(state);

    ROS_INFO("acc has calced! acc = %f",acc);

    LQR::controlmsg remsg = LQR::controlmsg();
    remsg.steering_angle = steer_angle;
    remsg.acc = acc;

    //ROS_INFO("calc has finished!");

    update_pub.publish(remsg);
    rate.sleep();
}

double LQR_control::lateralControl(LQR::state state){
    int index = findMatchPoint(state);
    ROS_INFO("matchPoint Index = %d",index);
    Eigen::Matrix<double,4,1>err = calcErr(index,state);
   
    Eigen::Matrix<double,4,1> K = calcK(state);

    double forword_angle = calc_forword_angle(K,err,index,state);
    ROS_INFO("forward_angle = %f",forword_angle);
    //return -K.transpose()*err + forword_angle;
    return -K.transpose()*err ;
}

double LQR_control::longitudinalControl(LQR::state state){
    double kp = 0.03;
    double a;
    if(abs(referenceline.referencelinePoints[ findMatchPoint(state)].kr) > 0.1) {
        a = kp * (5 -state.velocity) / dertaT;
    }else{
        a = kp * (ref_velocity -state.velocity) / dertaT;
    }
    if(a > max_acc) a = max_acc;
    else if(a < max_dec) a = max_dec;
    return a;
}

int LQR_control::findMatchPoint(LQR::state state){
    double minindex = 0;
    double minLength = pow(state.centerX - referenceline.referencelinePoints[0].xr,2) + pow(state.centerY - referenceline.referencelinePoints[0].yr,2);
    for(int i = 1; i < referenceline.referencelinePoints.size(); i++){
        double curLength = pow(state.centerX - referenceline.referencelinePoints[i].xr,2) + pow(state.centerY - referenceline.referencelinePoints[i].yr,2);
        if(curLength < minLength){
            minLength = curLength;
            minindex = i;
        }
    } 
    return minindex;
}

Eigen::Matrix<double,4,1> LQR_control::calcErr(int index, LQR::state state){
    LQR::referencelinePoint match_point = referenceline.referencelinePoints[index];
    LQR::referencelinePoint projPoint = LQR::referencelinePoint();
    Eigen::Matrix<double,2,1> tor,nor,d_err;
    Eigen::Matrix<double,4,1> err;
    tor << cos(match_point.thetar),
                sin(match_point.thetar);
    nor << -sin(match_point.thetar),
                cos(match_point.thetar);
    d_err << state.centerX - match_point.xr,
                state.centerY - match_point.yr;
    double es = tor.transpose() *   d_err;
    double ed = nor.transpose() *   d_err;
    projPoint.xr = match_point.xr + es  *   tor(0,0);
    projPoint.yr = match_point.yr + es  *   tor(1,0);
    projPoint.thetar = match_point.thetar + match_point.kr * es;
    projPoint.kr = match_point.kr;
    ROS_INFO("proj_xr  = %f, proj_yr = %f", projPoint.xr, projPoint.yr);
    ROS_INFO("proj_thetar  = %f, proj_kr = %f", projPoint.thetar, projPoint.kr);

    double ed_dot = state.velocity  *  sin(state.theta - projPoint.thetar);
    double e_phi = sin(state.theta - projPoint.thetar);
    double s_ddot = state.velocity  *   cos(state.theta - projPoint.thetar);
    double s_dot = s_ddot/(1-projPoint.kr*ed);
    double e_dphi =   state.velocity*projPoint.kr -  projPoint.kr*s_dot;
    err << ed,ed_dot,e_phi,e_dphi;
    ROS_INFO("err1 = %f, err2 = %f, err3 = %f, err4 = %f ",err[0],err[1],err[2],err[3]);
    return err;
}

Eigen::Matrix<double,4,1> LQR_control::calcK(LQR::state state){
    Eigen::Matrix4d A,Q;
    Eigen::Matrix<double,4,1> B;
    Eigen::Matrix<double,1,1> R;
    A << 0, 1, 0, 0,
            0,  (Caf+Car)/(m*state.velocity),   -(Caf+Car)/m,   (a*Caf-b*Car)/(m*state.velocity),
            0,  0,  0,  1,
            0,  (a*Caf - b*Car)/(Iz*state.velocity),    -(a*Caf - b*Car)/Iz,    (a*a*Caf+b*b*Car)/(Iz*state.velocity);
    B << 0, -Caf/m, 0,  -a*Caf/Iz;

    Q<< 25,0,0,0,
             0,3,0,0,
            0,0,10,0,
            0,0,0,4;
    R <<15;
    return dlqr(A,B,Q,R);
}

Eigen::Matrix<double,4,1> LQR_control::dlqr(Eigen::Matrix4d A, Eigen::Matrix<double,4,1> B,Eigen::Matrix4d Q,Eigen::Matrix<double,1,1> R){
    //ROS_INFO("start calc K!");
    //最大循环次数
    int loop = 200;
    double minValue = 0.001;
    Eigen::Matrix4d P_old = Q;
    double dt = 0.01;
    Eigen::Matrix4d eye;
    eye.setIdentity(4,4);

    Eigen::Matrix4d A_;
    A_ = (eye - A*0.5*dt).inverse()*(eye+A*0.5*dt);
    Eigen::Matrix<double,4,1> B_;
    B_ = B * dt;
    for(int i = 0; i < loop; i++){
        Eigen::Matrix4d P_new = A_.transpose()*P_old*A_ - A_.transpose()*P_old*B_*(R+B_.transpose()*P_old*B_).inverse()*B_.transpose()*P_old*A_+Q;
        if((P_new - P_old).lpNorm<Eigen::Infinity>() < minValue){
            P_old = P_new;
            break;
        }
        P_old = P_new;
    }

    Eigen::Matrix<double,4,1> k;
    k = (R+B_.transpose()*P_old*B_).inverse()*B_.transpose()*P_old*A_;
    ROS_INFO("K has calced !    k1 = %f, k2 = %f, k3 = %f, k4 = %f ",k[0],k[1],k[2],k[3]);
    return k;
}

double LQR_control::calc_forword_angle(Eigen::Matrix<double,4,1> k, Eigen::Matrix<double,4,1> err, double index, LQR::state state){
    double k3 = k(2,0);
    //不足转向系数
    double kv = b * m/(Caf *(a+b)) - a*m/(Car*(a+b));

    double point_cur = referenceline.referencelinePoints[index].kr;
    double forword_angle = (a+b) * point_cur + kv * state.velocity * state.velocity * point_cur-
                                                        k3 *(b  * point_cur - a*m * state.velocity * state.velocity* point_cur/(Car*(b+a)));
                                                        return forword_angle;
}

void LQR_control :: reflineCallback(const LQR::referenceline::ConstPtr& msg){
    for(auto point : msg->referencelinePoints){
        referenceline.referencelinePoints.push_back(point);
    }
}



int main(int argc, char** argv){
    ros::init(argc, argv, "lqr_control");
    LQR_control lqr_control;
    ros::spin();

    return 0;
}