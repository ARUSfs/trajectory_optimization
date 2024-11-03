#include <rclcpp/rclcpp.hpp>
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/point_xy.hpp"
#include "common_msgs/msg/state.hpp"
#include "libInterpolate/Interpolate.hpp"
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
        double vx_;
        double vy_;
        double speed_;

        rclcpp::Subscription<common_msgs::msg::Trajectory>::SharedPtr arussim_sub_;
        rclcpp::Subscription<common_msgs::msg::State>::SharedPtr car_state_sub_;
        rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;

        void arussim_callback(common_msgs::msg::Trajectory::SharedPtr assim_msg);
        void car_state_callback(common_msgs::msg::State::SharedPtr car_state_msg);

        //Auxiliar methods
        common_msgs::msg::Trajectory create_trajectory(VectorXd traj_x, VectorXd traj_y, VectorXd s, VectorXd k, VectorXd speed_profile);
        MatrixXd get_distance_and_curvature_values(VectorXd traj_x, VectorXd traj_y);
        VectorXd generate_speed_profile(VectorXd s, VectorXd k);
};

