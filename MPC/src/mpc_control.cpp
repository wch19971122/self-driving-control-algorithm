#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <MPC/referenceline.h>
#include <MPC/referencelinePoint.h>
#include <MPC/state.h>
#include <MPC/controlmsg.h>

constexpr double    pi = 3.14159;
constexpr double    Caf = -175016;
constexpr double    Car = -130634;
constexpr double    a = 1.265;
constexpr double    b = 1.682;
constexpr double    m = 2020;
constexpr double    Iz = 4095;
constexpr int Np = 20;
constexpr int Nc = 10;
constexpr int numOfx = 4;
constexpr int numOfu = 1;
constexpr double dertaT =0.01;
constexpr double Umax =  1.75;
constexpr double Umin = -1.75; 
constexpr double dertaUmin = -0.6;
constexpr double derteUmax = 0.6;
constexpr double predict_time = 1;


double limit_kesi(double kesi) {
	if (kesi > pi / 2) {
		kesi = pi /2;
	}
	if (kesi < -pi / 2) {
		kesi = -pi / 2;
	}
	return kesi;
}

class MPC_control{
public:
    MPC_control(){
        ROS_INFO("MPC initial!");
        ref_sub = n.subscribe<MPC::referenceline>("referenceline_pub",1,&MPC_control::reflineCallback,  this);
        update_pub = n.advertise<MPC::controlmsg>("mpc_ctrl",1);
        state_sub = n.subscribe<MPC::state>("mpc_state", 1, &MPC_control::mpcctrlCallback,this);     
    };
    void reflineCallback(const MPC::referenceline::ConstPtr& msg);
    void mpcctrlCallback(const MPC::state::ConstPtr& msg);
    int findMatchPoint(MPC::state state);
    Eigen::Matrix<double,5,1> calcErr(int index, MPC::state state);
    double longitudinalControl(MPC::state state);
    bool lateralControl(OsqpEigen:: Solver &solver, MPC::state state, Eigen::Matrix<double,5,1> kesi);
    
private:
    int preIndex = 0;
    double ref_velocity = 15;
    double max_acc = 3;
    double max_dec = -2;
    MPC::referenceline referenceline;

    ros::NodeHandle n;
    ros::Rate rate = ros::Rate(100);
    ros::Publisher update_pub;
    ros::Subscriber state_sub;
    ros::Subscriber ref_sub;

    Eigen::SparseMatrix<double> Hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double>  linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
};

void MPC_control::mpcctrlCallback(const MPC::state::ConstPtr& msg){
    MPC::state state = MPC::state();
    state.centerX = msg -> centerX + predict_time*state.velocity*cos(state.theta);
    state.centerY = msg -> centerY + predict_time*state.velocity*sin(state.theta);
    state.theta = msg -> theta;
    state.steering_angle = msg -> steering_angle;
    state.velocity = msg -> velocity;

    ROS_INFO("--------------------------    MPC_catc has start!    -------------------------------");
    ROS_INFO("cur_state X= %f, cur_state Y = %f",state.centerX,state.centerY);
    ROS_INFO("cur_state theta= %f, cur_state velocity = %f",state.theta,state.velocity);

    OsqpEigen:: Solver solver;

    solver.settings() -> setWarmStart(true);

    int index = findMatchPoint(state);
    ROS_INFO("matchPoint Index = %d",index);
    Eigen::Matrix<double,5,1> kesi = calcErr(index,state);  

    lateralControl(solver,state,kesi);
    
    if(!solver.initSolver()) ROS_INFO("initSolver crash!!!");
    ROS_INFO("Quadprog start!");
    solver.solve();

    Eigen::VectorXd QPSolution;
    QPSolution = solver.getSolution();

    double steer_angle = QPSolution[0];
  
    for(int i = 0; i < Nc; i++){
        ROS_INFO("steer_angle[%d] = %f",i,QPSolution[i]);
    }



    ROS_INFO("steer_angle has calced! steer_angle = %f",steer_angle);
    
    double acc = longitudinalControl(state);

    ROS_INFO("acc has calced! acc = %f",acc);

    MPC::controlmsg remsg = MPC::controlmsg();
    remsg.steering_angle = steer_angle;
    remsg.acc = acc;

    //ROS_INFO("calc has finished!");

    update_pub.publish(remsg);
    rate.sleep();
}

bool MPC_control::lateralControl(OsqpEigen:: Solver &solver, MPC::state state, Eigen::Matrix<double,5,1> kesi){


   //初始化a_init，b_init，c_init矩阵
    ROS_INFO("initializing a_init,b_init,c_init...");
    Eigen::Matrix4d a_init;
    Eigen::Matrix<double,4,1> b_init,c_init;
    a_init << 0,1,0,0,
              0,(Caf+Car)/(m*state.velocity), -(Car+Caf)/m, (a*Caf-b*Car)/(m*state.velocity),
              0,0,0,1,
              0,(a*Caf - b*Car)/(Iz*state.velocity),-(a*Caf-b*Car)/Iz,(a*a*Caf+b*b*Car)/(Iz*state.velocity);
    b_init << 0,-Caf/m, 0, -a*Caf/Iz;
    c_init << 0, (a*Caf - b*Car)/(m*state.velocity)-state.velocity,0,(a*a*Caf+b*b*Car)/(Iz*state.velocity);

    ROS_INFO("discreating...");
    //离散化矩阵
    Eigen::Matrix4d A;
    Eigen::Matrix<double,2,4> D;
    Eigen::Matrix<double,4,1>  B,C;
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    A = (I - a_init*dertaT/2).inverse()*(I + a_init*dertaT/2);
    B = b_init*dertaT;
    C = c_init*dertaT;
    D(0,0) = 1;
    D(1,2) = 1;   


    ROS_INFO("creating new status...");
    //构造新的状态量
    Eigen::Matrix<double,5,5> A_tidle;
    Eigen::Matrix<double,5,1> B_tidle,C_tidle;
    Eigen::Matrix<double,2,5> D_tidle;

    //A_tidle
    A_tidle.block<4,4>(0,0) = A;
    A_tidle.block<4,1>(0,4) = B;
    A_tidle(4,4) = 1;

    //B_tidle
    B_tidle.block<4,1>(0,0) = B;
    B_tidle(4,0) = 1;

    //C_tidle,D_tidle
    C_tidle.block<4,1>(0,0) = C;
    D_tidle.block<2,4>(0,0) = D;


    //预测模型PSI、THETA、Gamma
    ROS_INFO("predicting model...");
    Eigen::Matrix<double,2*Np,5> PSI;
    Eigen::Matrix<double,5*Np,Nc> THETA_init;
    Eigen::Matrix<double,2*Np,Nc> THETA;
    Eigen::Matrix<double,5*Np,1> Gamma_init;
    Eigen::Matrix<double,2*Np,1> Gamma;

    for(int i = 0; i < Np; i++){
        if(i == 0){
            PSI.block<2,5>(i,i) = D_tidle*A_tidle;
            THETA_init.block<5,1>(i,i) = B_tidle;
            Gamma_init.block<5,1>(i,i) = C_tidle;
        }else if( i < Nc){
            THETA_init.block<5,1>(5*i,i) = B_tidle;
            Gamma_init.block<5,1>(5*i,0) = A_tidle*Gamma_init.block<5,1>(5*(i-1),0) + C_tidle;
            PSI.block<2,5>(2*i,0) = PSI.block<2,5>(2*(i-1),0)*A_tidle;
            for(int j = i-1; j >= 0; j--){
                THETA_init.block<5,1>(5*i,j) = A_tidle*THETA_init.block<5,1>(5*(i-1),j);
            }
        }else{
            //i >= Nc
            PSI.block<2,5>(2*i,0) = PSI.block<2,5>(2*(i-1),0)*A_tidle;
            Gamma_init.block<5,1>(5*i,0) = A_tidle*Gamma_init.block<5,1>(5*(i-1),0) + C_tidle;
            for(int j = 0; j < Nc; j++){
                THETA_init.block<5,1>(5*i,j) = A_tidle*THETA_init.block<5,1>(5*(i-1),j);
            }
        }
    }
    for(int i = 0; i < Np; i++){
        Gamma.block<2,1>(2*i,0) =D_tidle * Gamma_init.block<5,1>(5*i,0);
        for(int j = 0; j <= i && j < Nc; j++){
            THETA.block<2,1>(2*i,j) = D_tidle * THETA_init.block<5,1>(5*i,j);
        }
    }


    ROS_INFO("cost Function...");
    //代价函数
    Eigen::Matrix<double,2*Np,2*Np> Q = 30*Eigen::Matrix<double,2*Np,2*Np>::Identity();
    for(int i = 0; i < Np; i++){
        Q(2*i,2*i) = 80;
        Q(2*i+1,2*i +1) = 100;
    }


    Eigen::Matrix<double,Nc,Nc> R = Eigen::Matrix<double,Nc,Nc>::Identity();
    Eigen::Matrix<double,Nc,Nc> H = THETA.transpose()*Q*THETA + R;
    Eigen::Matrix<double,1,Nc> g = (PSI*kesi - Gamma).transpose()*Q*THETA;

    ROS_INFO("constraints...");
    //约束条件
    Eigen::Matrix<double,Nc,Nc> AI;
    for(int i = 0; i < Nc; i++){
        for(int j = 0; j <= i; j++){
            AI(i,j) = 1;
        }
    }
    ROS_INFO("to SparseMatrix...");
    //将H和不等式约束转化为稀疏矩阵
    Hessian.resize(Nc,Nc);
    linearMatrix.resize(2*Nc,Nc);
    lowerBound.resize(Nc*2);
    upperBound.resize(Nc*2);
    for(int i = 0; i <Nc; i++){
        for(int j = 0; j < Nc; j++){
                Hessian.insert(i,j) = H(i,j);
                lowerBound(i) = Umin - state.steering_angle;
                upperBound(i) = Umax + state.steering_angle;
                linearMatrix.insert(i,j) = AI(i,j);
        }
    }

    for(int i = 0; i < Nc; i++){
        linearMatrix.insert(Nc+i,i) = 1;
        lowerBound(i+Nc) = dertaUmin;
        upperBound(i+Nc) = derteUmax;
    }
     
    ROS_INFO("QP initializing...");
    solver.data() -> setNumberOfVariables(Nc);
    solver.data() -> setNumberOfConstraints(2*Nc); 
    if(!solver.data() -> setHessianMatrix(Hessian)) return false;
    if(!solver.data() -> setGradient(g)) return false;
    if(!solver.data() -> setLinearConstraintsMatrix(linearMatrix)) return false;
    if (!solver.data()->setLowerBound(lowerBound)) return false;
    if (!solver.data()->setUpperBound(upperBound)) return false;

    ROS_INFO("Assert Matrix fininshed!!!");
    return true;
    
}

double MPC_control::longitudinalControl(MPC::state state){
    double kp = 0.03;
    double a;
    /*
    if(abs(referenceline.referencelinePoints[ findMatchPoint(state)].kr) > 0.1) {
        a = kp * (5 -state.velocity) / dertaT;
    }else{
        a = kp * (ref_velocity -state.velocity) / dertaT;
    }
    */
    a = kp * (ref_velocity -state.velocity) / dertaT;
    if(a > max_acc) a = max_acc;
    else if(a < max_dec) a = max_dec;
    return a;
}

int MPC_control::findMatchPoint(MPC::state state){
    double minindex = preIndex;
    double minLength = pow(state.centerX - referenceline.referencelinePoints[minindex].xr,2) + pow(state.centerY - referenceline.referencelinePoints[minindex].yr,2);
    for(int i = preIndex; i < referenceline.referencelinePoints.size(); i++){
        double curLength = pow(state.centerX - referenceline.referencelinePoints[i].xr,2) + pow(state.centerY - referenceline.referencelinePoints[i].yr,2);
        if(curLength < minLength){
            minLength = curLength;
            minindex = i;
        }
    } 
    return minindex;
}

Eigen::Matrix<double,5,1> MPC_control::calcErr(int index, MPC::state state){
    MPC::referencelinePoint match_point = referenceline.referencelinePoints[index];
    MPC::referencelinePoint projPoint = MPC::referencelinePoint();
    Eigen::Matrix<double,2,1> tor,nor,d_err;
    Eigen::Matrix<double,5,1> err;
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
    err << ed,ed_dot,e_phi,e_dphi, state.steering_angle;
    ROS_INFO("err1 = %f, err2 = %f, err3 = %f, err4 = %f ",err[0],err[1],err[2],err[3]);
    return err;
}

void MPC_control :: reflineCallback(const MPC::referenceline::ConstPtr& msg){
    for(auto point : msg->referencelinePoints){
        referenceline.referencelinePoints.push_back(point);
    }
}

int main(int argc, char** argv){
    
    ros::init(argc,argv,"mpc_control");

    MPC_control mpc_control;

    ros::spin();
    return 0;
    
}

