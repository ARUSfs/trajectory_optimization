#include <rclcpp/rclcpp.hpp>
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/point_xy.hpp"
#include "Interpolate.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
using namespace std;
using namespace Eigen;

class TrajectoryOptimization : public rclcpp::Node
{
    public:
        TrajectoryOptimization();
    private:
        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr arussim_sub_;
        rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;

        void arussim_callback(common_msgs::msg::Trajectory::SharedPtr assim_msg);

        //Trajectory variable
        std::vector<common_msgs::msg::PointXY> pointsXY_;  
        std::vector<float> s_;                           // distance values of the points              
        std::vector<float> k_ ;                          // curvature values of the points
        std::vector<float> speed_profile_;               
        std::vector<float> acc_profile_;    

        //Auxiliar methods
        MatrixXd process_track_data(VectorXd x, VectorXd y, VectorXd twr, VectorXd twl);
        MatrixXd diff_col(MatrixXd E);
        VectorXd cumsum(VectorXd v);
        VectorXd gradient(VectorXd f);
        MatrixXd matrixH(VectorXd delx, VectorXd dely);
        MatrixXd matrixB(VectorXd xin, VectorXd yin, VectorXd delx, VectorXd dely);
        VectorXd solver(MatrixXd H, MatrixXd B);
        common_msgs::msg::Trajectory create_trajectory(VectorXd traj_x, VectorXd traj_y);

};