#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "stdio.h"
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using namespace std;
//transformation
Eigen::Vector4d qji_r, qji_d;
double qji_r_w, qji_r_x, qji_r_y, qji_r_z;
double qji_d_w, qji_d_x, qji_d_y, qji_d_z;

double qji_r_w_k, qji_r_x_k, qji_r_y_k, qji_r_z_k;
double qji_d_w_k, qji_d_x_k, qji_d_y_k, qji_d_z_k;
Eigen::VectorXd qji_k(8);
//Eigen::VectorXd qji(8);
vector<double*> qji;
Eigen::VectorXd transform_hkxk(8);
Eigen::MatrixXd h_g(8,8);

Eigen::MatrixXd P_k(8,8), P_k_inverse(8,8), R_k_inverse(8,8);
Eigen::MatrixXd R_k(8,8);
Eigen::VectorXd y_k(8);
void initial(){
  R_k << 0.1, 0, 0, 0, 0, 0, 0, 0,
         0, 0.1, 0, 0, 0, 0, 0, 0,
         0, 0, 0.1, 0, 0, 0, 0, 0,
         0, 0, 0, 0.1, 0, 0, 0, 0,
         0, 0, 0, 0, 0.1, 0, 0, 0,
         0, 0, 0, 0, 0, 0.1, 0, 0,
         0, 0, 0, 0, 0, 0, 0.1, 0,
         0, 0, 0, 0, 0, 0, 0, 0.1;
  y_k.setZero();
  qji_k << 1, 0, 0, 0, 0, 2, 0, 0;
  qji[0][0] = 1.0;
  qji[0][1] = 0.0;
  qji[0][2] = 0.0;
  qji[0][3] = 0.0;
  qji[0][4] = 0.0;
  qji[0][5] = 2.0;
  qji[0][6] = 0.0;
  qji[0][7] = 0.0;
}

void transformation_model(){
  double leader_vx, leader_vy, leader_vz;
  double follower_vx, follower_vy, follower_vz;
  double leader_wx, leader_wy, leader_wz;
  double follower_wx, follower_wy, follower_wz;
  Eigen::VectorXd y(8);
  double lamda = 1.004;

  h_g << 0, leader_wx - follower_wx, leader_wy - follower_wy, leader_wz - follower_wz, 0, 0, 0, 0,
         follower_wx - leader_wx, 0, -(leader_wz + follower_wz), (leader_wy + follower_wy), 0, 0, 0, 0,
         follower_wy - leader_wy, (leader_wz + follower_wz), 0, -(leader_wx + follower_wx), 0, 0, 0, 0,
         follower_wz - leader_wz, -(leader_wy + follower_wy), (leader_wx + follower_wx), 0, 0, 0, 0, 0,
         0, leader_vx - follower_vx, leader_vy - follower_vy, leader_vz - follower_vz, 0, leader_wx - follower_wx, leader_wy - follower_wy, leader_wz - follower_wz,
         follower_vx - leader_vx, 0, -(leader_vz + follower_vz), (leader_vy + follower_vy), follower_wx - leader_wx, 0, -(leader_wz + follower_wz), (leader_wy + follower_wy),
         follower_vy - leader_vy, (leader_vz + follower_vz), 0, -(leader_vx + follower_vx), follower_wy - leader_wy, (leader_wz + follower_wz), 0, -(leader_wx + follower_wx),
         follower_vz - leader_vz, -(leader_vy + follower_vy), (leader_vx + follower_vx), 0, follower_wz - leader_wz, -(leader_wy + follower_wy), (leader_wx + follower_wx);
  transform_hkxk = h_g * qji_k;
  //P_k.inverse() + h_g.transpose()*R_k.inverse*h_g
  P_k = lamda*(P_k.inverse() + h_g.transpose()*R_k.inverse()*h_g).inverse();
  P_k_inverse = P_k.inverse();

}

void alpha_model(){
  double leader_alpha_x, leader_alpha_y, leader_alpha_z;
  double leader_a_x, leader_a_y, leader_a_z;
  double follower_alpha_x, follower_alpha_y, follower_alpha_z;
  double follower_a_x, follower_a_y, follower_a_z;
  Eigen::Vector3d leader_a, follower_a;
  Eigen::Vector3d y;
  leader_a << leader_a_x, leader_a_y, leader_a_z;
  follower_a << follower_a_x, follower_a_y, follower_a_z;
  //y = follower_a - Rji*leader_a;
}

void model(){

}

void p_model(){
  double F_x, F_y, F_z;
  double alpha_x, alpha_y, alpha_z;
  double w_x, w_y, w_z;
  double v_x, v_y, v_z;
  double a_x, a_y, a_z;
  double p_x, p_y, p_z;
  Eigen::MatrixXd h_pc(2,3);
  Eigen::MatrixXd F_i_T(2,3);
  Eigen::Matrix3d alpha_i_x;
  Eigen::Vector3d w_i;
  Eigen::Matrix3d w_i_x2;
  Eigen::Vector3d v_i;
  Eigen::Vector3d a_i;
  Eigen::Vector2d y_pc;
  Eigen::Vector3d p_ic;
  F_i_T << 1, -1, (-1*F_x + 1*F_y)/F_z,
           -1, 1, (1*F_x - 1*F_y)/F_z;
  alpha_i_x << 0, -alpha_z, alpha_y,
               alpha_z, 0, -alpha_x,
               -alpha_y, alpha_x, 0;
  w_i_x2 << -w_y*w_y-w_z*w_z, w_x*w_y, w_x*w_z,
            w_x*w_y, -w_x*w_x-w_z*w_z, w_y*w_z,
            w_x*w_z, w_y*w_z, -w_x*w_x - w_y*w_y;
  v_i << v_x, v_y, v_z;
  a_i << a_x, a_y, a_z;
  w_i << w_x, w_y, w_z;
  h_pc = F_i_T*(alpha_i_x + w_i_x2);
  y_pc = -F_i_T*(w_i.cross(v_i) + a_i);
  y_pc = h_pc * p_ic;
}

void inertia_model(){
  double alpha_x, alpha_y, alpha_z;
  double w_x, w_y, w_z;
  double F_x, F_y, F_z;
  double p_x, p_y, p_z;
  double I_xx, I_yy, I_zz, I_xy, I_xz, I_yz;

  Eigen::VectorXd I(6);
  Eigen::MatrixXd H_I(3,6);
  Eigen::Vector3d F_i;
  Eigen::Vector3d p_ic;
  Eigen::Vector3d y_I;

  I << I_xx, I_yy, I_zz, I_xy, I_xz, I_yz;
  H_I << alpha_x, -w_y*w_z, w_y*w_z, alpha_y - w_x*w_z, alpha_z+w_x*w_y, w_y*w_y-w_z*w_z,
         w_x*w_z, alpha_y, -w_x*w_z, alpha_x + w_y*w_z, w_z*w_z-w_x*w_x, alpha_z - w_x*w_y,
        -w_x*w_y, w_x*w_y, alpha_z, w_x*w_x-w_y*w_y, alpha_x - w_y*w_z, alpha_y + w_x*w_z;
  F_i << F_x, F_y, F_z;
  p_ic << p_x, p_y, p_z;

  y_I = F_i.cross(p_ic);
  y_I = H_I * I;

}

void m_model(){
  double a_x, a_y, a_z;
  double alpha_x, alpha_y, alpha_z;
  double p_x, p_y, p_z;
  double w_x, w_y, w_z;
  double v_x, v_y, v_z;

  Eigen::Vector3d H_m;
  Eigen::Vector3d y_m;
  Eigen::Vector3d F_i;
  Eigen::Vector3d a_i;
  Eigen::Vector3d alpha_i;
  Eigen::Vector3d p_ic;
  Eigen::Vector3d w_i;
  Eigen::Vector3d v_i;

  a_i << a_x, a_y, a_z;
  alpha_i << alpha_x, alpha_y, alpha_z;
  p_ic << p_x, p_y, p_z;
  v_i << v_x, v_y, v_z;
  H_m = a_i + alpha_i.cross(p_ic) + w_i.cross(w_i.cross(p_ic) + v_i);

}

void p_I_m_model(){
  double F_x, F_y, F_z;
  double alpha_x, alpha_y, alpha_z;
  double w_x, w_y, w_z;
  double v_x, v_y, v_z;
  double a_x, a_y, a_z;
  Eigen::MatrixXd F_i_T(2,3);
  Eigen::Matrix3d alpha_i_x;
  Eigen::Matrix3d w_i_x2;
  Eigen::Matrix3d F_i_x;
  Eigen::MatrixXd H_I(3,6);
  Eigen::Vector3d F_i;
  Eigen::Vector3d w_i, v_i, a_i;
  Eigen::VectorXd y_D(8);
  Eigen::VectorXd y_D_U(2);
  Eigen::VectorXd y_D_M(3);
  Eigen::VectorXd y_D_D(3);

  Eigen::MatrixXd H_D(8,10);
  Eigen::MatrixXd H_D_LU(2,3);
  Eigen::Matrix3d H_D_LD;
  F_i_T << 1, -1, (-1*F_x + 1*F_y)/F_z,
           -1, 1, (1*F_x - 1*F_y)/F_z;
  alpha_i_x << 0, -alpha_z, alpha_y,
               alpha_z, 0, -alpha_x,
               -alpha_y, alpha_x, 0;
  w_i_x2 << -w_y*w_y-w_z*w_z, w_x*w_y, w_x*w_z,
            w_x*w_y, -w_x*w_x-w_z*w_z, w_y*w_z,
            w_x*w_z, w_y*w_z, -w_x*w_x - w_y*w_y;
  F_i_x << 0, -F_z, F_y,
           F_z, 0, -F_x,
          -F_y, F_x, 0;
  H_I << alpha_x, -w_y*w_z, w_y*w_z, alpha_y - w_x*w_z, alpha_z+w_x*w_y, w_y*w_y-w_z*w_z,
         w_x*w_z, alpha_y, -w_x*w_z, alpha_x + w_y*w_z, w_z*w_z-w_x*w_x, alpha_z - w_x*w_y,
        -w_x*w_y, w_x*w_y, alpha_z, w_x*w_x-w_y*w_y, alpha_x - w_y*w_z, alpha_y + w_x*w_z;
  F_i << F_x, F_y, F_z;
  w_i << w_x, w_y, w_z;
  v_i << v_x, v_y, v_z;
  a_i << a_x, a_y, a_z;

  H_D_LU = F_i_T*(alpha_i_x + w_i_x2);
  H_D_LD = -alpha_i_x - w_i_x2;
  H_D << H_D_LU(0,0), H_D_LU(0,1), H_D_LU(0,2), 0, 0, 0, 0, 0, 0, 0,
         H_D_LU(1,0), H_D_LU(1,1), H_D_LU(2,2), 0, 0, 0, 0, 0, 0, 0,
         -F_i_x(0,0), -F_i_x(0,1), -F_i_x(0,2), H_I(0,0), H_I(0,1), H_I(0,2), H_I(0,3), H_I(0,4), H_I(0,5), 0,
         -F_i_x(1,0), -F_i_x(1,1), -F_i_x(1,2), H_I(1,0), H_I(1,1), H_I(1,2), H_I(1,3), H_I(1,4), H_I(1,5), 0,
         -F_i_x(2,0), -F_i_x(2,1), -F_i_x(2,2), H_I(2,0), H_I(2,1), H_I(2,2), H_I(2,3), H_I(2,4), H_I(2,5), 0,
         H_D_LD(0,0), H_D_LD(0,1), H_D_LD(0,2), 0, 0, 0, 0, 0, 0, F_i(0),
         H_D_LD(1,0), H_D_LD(1,1), H_D_LD(1,2), 0, 0, 0, 0, 0, 0, F_i(1),
         H_D_LD(2,0), H_D_LD(2,1), H_D_LD(2,2), 0, 0, 0, 0, 0, 0, F_i(2);
  y_D_U = -F_i_x*(w_i.cross(v_i) + a_i);
  y_D_M << 0, 0, 0;
  y_D_D = w_i.cross(v_i) + a_i;
  y_D << y_D_U(0), y_D_U(1), y_D_U(2), y_D_M(0), y_D_M(1), y_D_M(2), y_D_D(0), y_D_D(1), y_D_D(2);

}

void filer(Eigen::VectorXd &y_k, Eigen::MatrixXd &H_k, Eigen::MatrixXd &P_k, Eigen::MatrixXd &R_k, Eigen::VectorXd *x_k_1){



}

struct CostFunctor {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = T(10.0) - x[0];
     return true;
   }
};

struct F1 {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = (sqrt(P_k_inverse(5,5))) * (x[0] - (qji_d_x_k));
     return true;
   }
};

struct F2 {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = T(R_k_inverse(5,5))*(T(0) - T(transform_hkxk(5)));
     return true;
   }
};


struct F1_v {
   template <typename T>
   bool operator()(const double* const x, double* residual) const {
     residual[0] = ((x[0]-P_k_inverse(0))*h_g(0,0) + (x[1]-qji_k(1))*P_k_inverse(1,0) + (x[2]-qji_k(2))*P_k_inverse(2,0) + (x[3]-qji_k(3))*P_k_inverse(3,0) + (x[4]-qji_k(4))*P_k_inverse(4,0) + (x[5]-qji_k(5))*P_k_inverse(5,0) + (x[6]-qji_k(6))*P_k_inverse(6,0) + (x[7]-qji_k(7))*P_k_inverse(7,0))*(x[0] - qji_k(0))
                  +((x[0]-P_k_inverse(0))*h_g(0,1) + (x[1]-qji_k(1))*P_k_inverse(1,1) + (x[2]-qji_k(2))*P_k_inverse(2,1) + (x[3]-qji_k(3))*P_k_inverse(3,1) + (x[4]-qji_k(4))*P_k_inverse(4,1) + (x[5]-qji_k(5))*P_k_inverse(5,1) + (x[6]-qji_k(6))*P_k_inverse(6,1) + (x[7]-qji_k(7))*P_k_inverse(7,1))*(x[1] - qji_k(1))
                  +((x[0]-P_k_inverse(0))*h_g(0,2) + (x[1]-qji_k(1))*P_k_inverse(1,2) + (x[2]-qji_k(2))*P_k_inverse(2,2) + (x[3]-qji_k(3))*P_k_inverse(3,2) + (x[4]-qji_k(4))*P_k_inverse(4,2) + (x[5]-qji_k(5))*P_k_inverse(5,2) + (x[6]-qji_k(6))*P_k_inverse(6,2) + (x[7]-qji_k(7))*P_k_inverse(7,2))*(x[2] - qji_k(2))
                  +((x[0]-P_k_inverse(0))*h_g(0,3) + (x[1]-qji_k(1))*P_k_inverse(1,3) + (x[2]-qji_k(2))*P_k_inverse(2,3) + (x[3]-qji_k(3))*P_k_inverse(3,3) + (x[4]-qji_k(4))*P_k_inverse(4,3) + (x[5]-qji_k(5))*P_k_inverse(5,3) + (x[6]-qji_k(6))*P_k_inverse(6,3) + (x[7]-qji_k(7))*P_k_inverse(7,3))*(x[3] - qji_k(3))
                  +((x[0]-P_k_inverse(0))*h_g(0,4) + (x[1]-qji_k(1))*P_k_inverse(1,4) + (x[2]-qji_k(2))*P_k_inverse(2,4) + (x[3]-qji_k(3))*P_k_inverse(3,4) + (x[4]-qji_k(4))*P_k_inverse(4,4) + (x[5]-qji_k(5))*P_k_inverse(5,4) + (x[6]-qji_k(6))*P_k_inverse(6,4) + (x[7]-qji_k(7))*P_k_inverse(7,4))*(x[4] - qji_k(4))
                  +((x[0]-P_k_inverse(0))*h_g(0,5) + (x[1]-qji_k(1))*P_k_inverse(1,5) + (x[2]-qji_k(2))*P_k_inverse(2,5) + (x[3]-qji_k(3))*P_k_inverse(3,5) + (x[4]-qji_k(4))*P_k_inverse(4,5) + (x[5]-qji_k(5))*P_k_inverse(5,5) + (x[6]-qji_k(6))*P_k_inverse(6,5) + (x[7]-qji_k(7))*P_k_inverse(7,5))*(x[5] - qji_k(5))
                  +((x[0]-P_k_inverse(0))*h_g(0,6) + (x[1]-qji_k(1))*P_k_inverse(1,6) + (x[2]-qji_k(2))*P_k_inverse(2,6) + (x[3]-qji_k(3))*P_k_inverse(3,6) + (x[4]-qji_k(4))*P_k_inverse(4,6) + (x[5]-qji_k(5))*P_k_inverse(5,6) + (x[6]-qji_k(6))*P_k_inverse(6,6) + (x[7]-qji_k(7))*P_k_inverse(7,6))*(x[6] - qji_k(6))
                  +((x[0]-P_k_inverse(0))*h_g(0,7) + (x[1]-qji_k(1))*P_k_inverse(1,7) + (x[2]-qji_k(2))*P_k_inverse(2,7) + (x[3]-qji_k(3))*P_k_inverse(3,7) + (x[4]-qji_k(4))*P_k_inverse(4,7) + (x[5]-qji_k(5))*P_k_inverse(5,7) + (x[6]-qji_k(6))*P_k_inverse(6,7) + (x[7]-qji_k(7))*P_k_inverse(7,7))*(x[7] - qji_k(7));
     return true;
   }
};

struct F2_v {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = (R_k_inverse.llt().matrixL())*(y_k - (transform_hkxk));
     return true;
   }
};
DEFINE_string(minimizer, "trust_region",
              "Minimizer type to use, choices are: line_search & trust_region");
int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimate_payload");
  ros::NodeHandle nh;
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  google::InitGoogleLogging(argv[0]);

  initial();
  // The variable to solve for with its initial value.


    // Build the problem.
    Problem problem;
    ros::Rate rate(10);
    while(ros::ok()){
  /*
    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    CostFunction* cost_function =
        new AutoDiffCostFunction<F1, 1, 1>(new F1);
    problem.AddResidualBlock(cost_function, NULL, &qji_d_x);
  */
    problem.AddResidualBlock(new AutoDiffCostFunction<F1_v, 1, 8>(new F1_v),
                               NULL,
                               qji);
    //problem.SetParameterBlockConstant(qji);
    problem.AddResidualBlock(new AutoDiffCostFunction<F2_v, 1, 8>(new F2_v),
                               NULL,
                             qji);

    // Run the solver!
    Solver::Options options;
    LOG_IF(FATAL, !ceres::StringToMinimizerType(FLAGS_minimizer,
                                                &options.minimizer_type))
        << "Invalid minimizer: " << FLAGS_minimizer
        << ", valid options are: trust_region and line_search.";
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "qji_x : " << qji_k
              << " -> " << qji << "\n";
    //qji_d_x_k = qji_d_x;
    //qji_k = qji;
    ros::spinOnce();
    rate.sleep();

}
    return 0;

}
