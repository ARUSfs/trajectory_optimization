#include "trajectory_optimization/trajectory_optimization_node.hpp"



TrajectoryOptimization::TrajectoryOptimization() : Node("trajectory_optimization")
{
    arussim_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        "/arussim_interface/fixed_trajectory", 10, std::bind(&TrajectoryOptimization::arussim_callback, this, std::placeholders::_1));
    trajectory_pub_ = this->create_publisher<common_msgs::msg::Trajectory>("/trajectory_optimizer/trajectory", 10);
}

void TrajectoryOptimization::arussim_callback(common_msgs::msg::Trajectory::SharedPtr assim_msg){
    
    std::vector<common_msgs::msg::PointXY> track_xy = assim_msg -> points;

    VectorXd x, y, twr, twl;      //Track width parameters need to be established

    for(int i = 0; i < track_xy.size(); i++){
        x(i) = track_xy[i].x;
        y(i) = track_xy[i].y;
    }

    //First, we process track data 
    Matrix<double, Dynamic, 6> track_data = TrajectoryOptimization::process_track_data(x, y, twr, twl);

    //Then, we form the matrices which will define the quadratic optimization problem
    VectorXd xin = track_data.col(2);
    VectorXd yin = track_data.col(3);
    VectorXd xout = track_data.col(4);
    VectorXd yout = track_data.col(5);
    
    VectorXd delx = xout - xin;
    VectorXd dely = yout - yin;
    
    int n = delx.size();

    MatrixXd H = TrajectoryOptimization::matrixH(delx, dely);
    MatrixXd B = TrajectoryOptimization::matrixB(xin, yin, delx, dely);

    //Solve the quadratic problem
    VectorXd resMCP = TrajectoryOptimization::solver(H,B);

    //Co-ordinates for the resultant curve
    VectorXd xresMCP = VectorXd::Zero(n);
    VectorXd yresMCP = VectorXd::Zero(n);

    for(int i = 0; i < n; i++){
        xresMCP(i) = xin(i) + resMCP(i)*delx(i);
        yresMCP(i) = yin(i) + resMCP(i)*dely(i);
    }
    
    common_msgs::msg::Trajectory trajectory_msg = TrajectoryOptimization::create_trajectory(xresMCP, yresMCP);
    trajectory_pub_ -> publish(trajectory_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryOptimization>());
    rclcpp::shutdown();
    return 0;
}






Matrix<double, Dynamic, 6> TrajectoryOptimization::process_track_data(VectorXd x, VectorXd y, VectorXd twr, VectorXd twl){

}

MatrixXd TrajectoryOptimization::matrixH(VectorXd delx, VectorXd dely){

}

MatrixXd TrajectoryOptimization::matrixB(VectorXd xin, VectorXd yin, VectorXd delx, VectorXd dely){

}

VectorXd TrajectoryOptimization::solver(MatrixXd H, MatrixXd B){

}

common_msgs::msg::Trajectory TrajectoryOptimization::create_trajectory(VectorXd traj_x, VectorXd traj_y){

}