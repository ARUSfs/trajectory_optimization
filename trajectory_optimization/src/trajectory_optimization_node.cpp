/**
 * @file trajectory_optimization_node.cpp
 * @author José Manuel Landero Plaza (josemlandero05@gmail.com)
 * @brief Trajectory Optimization node implementation for ARUS Team Driverless pipeline
 * @date 4-11-2024
 */
#include "trajectory_optimization/trajectory_optimization_node.hpp"
#include "trajectory_optimization/min_curvature_path.hpp"

//These parameters will be established in a configuration file
const double KAxMax = 3;
const double KAyMax = 1.5;
const double KVMax = 8;
const double KDMax = 0.7;

/**
 * @class TrajectoryOptimization
 * @brief TrajectoryOptimization class
 * 
 * This class generates optimized trajectory (minimal curvature) based on the trajectory received 
 * from path planning (right now, it takes trajectory from arussim_interface to allow testing)
 * 
 */
TrajectoryOptimization::TrajectoryOptimization() : Node("trajectory_optimization")
{
    arussim_sub_ = this->create_subscription<common_msgs::msg::Trajectory>(
        "/arussim_interface/fixed_trajectory", 10, std::bind(&TrajectoryOptimization::arussim_callback, this, std::placeholders::_1));
    car_state_sub_ = this->create_subscription<common_msgs::msg::State>(
        "/car_state/state", 1, std::bind(&TrajectoryOptimization::car_state_callback, this, std::placeholders::_1));
    trajectory_pub_ = this->create_publisher<common_msgs::msg::Trajectory>("/trajectory_optimizer/trajectory", 10);
}


/**
 * @brief Callback function for the ARUSSim topic
 * 
 * We provisionally receive trajectory from ARUSSim topic to allow early testing.
 * When a trajectory message is received, the callback extracts the track centerline 
 * (x,y) points from the message (the other parameters are empty) and executes all  
 * necessary computations to get the optimized trajectory full message 
 * (except acceleration profile at this moment)
 * 
 * @param assim_msg The trajectory received from ARUSSim
 */
void TrajectoryOptimization::arussim_callback(common_msgs::msg::Trajectory::SharedPtr assim_msg){
   
    std::vector<common_msgs::msg::PointXY> track_xy = assim_msg -> points;
    
    //Convert points message to vectors
    int n = track_xy.size();
    VectorXd x(n), y(n);
    for(int i = 0; i < n; i++){
        x(i) = track_xy[i].x;
        y(i) = track_xy[i].y;
    }

    //Generate track width vectors
    MatrixXd original_s_k = TrajectoryOptimization::get_distance_and_curvature_values(x, y);
    VectorXd original_k = original_s_k.col(1);
    VectorXd twr = TrajectoryOptimization::generate_track_width(original_k, KDMax);
    VectorXd twl = twr;

    //Get minimal curvature path
    MatrixXd optimized_trajectory = MinCurvaturepath::get_min_curvature_path(x, y, twr, twl);
    VectorXd traj_x = optimized_trajectory.col(0);
    VectorXd traj_y = optimized_trajectory.col(1);

    //Get accumulated distance and curvature at each point
    MatrixXd optimized_s_k = TrajectoryOptimization::get_distance_and_curvature_values(traj_x, traj_y);
    VectorXd s = optimized_s_k.col(0);
    VectorXd k = optimized_s_k.col(1);

    //Generate speed profile
    VectorXd speed_profile = TrajectoryOptimization::generate_speed_profile(s, k);

    
    //Create and publish trajectory message
    common_msgs::msg::Trajectory trajectory_msg = TrajectoryOptimization::create_trajectory_msg(traj_x, traj_y, s, k, speed_profile);
    trajectory_pub_ -> publish(trajectory_msg);
}


/**
 * @brief Callback function for the car_state topic
 * 
 * We extract vx and vy to calculate the car's current speed
 * 
 * @param car_state_msg Car state variables received from car_state
 */
void TrajectoryOptimization::car_state_callback(common_msgs::msg::State::SharedPtr car_state_msg){
    vx_ = car_state_msg -> vx;
    vy_ = car_state_msg -> vy;

    speed_ = sqrt(vx_*vx_ + vy_*vy_);
}


/**
 * @brief Generate track width limits on each point based on the trajectory's 
 * curvature to control the optimized trajectory's boundaries
 * 
 * @param  k Curvature of the given trajectory
 * @param  dmax Maximum track width
 * 
 * @return VectorXd Vector of track width allowed at each point (both left and rigth)
 */
VectorXd TrajectoryOptimization::generate_track_width(VectorXd k, double dmax){
    int n = k.size();
    VectorXd dist = VectorXd::Zero(n);

    double kk;
    for(int i = 0; i < n; i++){
        kk = k(seq(max(0, i-20), min(n-1, i+20))).mean();
        dist(i) = min(dmax, max(0.5, abs(kk*10)));
    }

    return dist;
}


/**
 * @brief Creates the trajectory message to publish
 * 
 * @param  traj_x x coordinates of the trajectory points
 * @param  traj_y y coordinates of the trajectory points
 * @param  s Accumulated distance at each point
 * @param  k Curvature at each point
 * @param  speed_profile Speed profile for the given trajectory
 * 
 * @return common_msgs::msg::Trajectory 
 */
common_msgs::msg::Trajectory TrajectoryOptimization::create_trajectory_msg(VectorXd traj_x, VectorXd traj_y, VectorXd s, VectorXd k, VectorXd speed_profile){
    common_msgs::msg::Trajectory traj_msg;

    for(int i = 0; i < traj_x.size(); i++){
        common_msgs::msg::PointXY p;
        p.x = traj_x(i);
        p.y = traj_y(i);
        traj_msg.points.push_back(p);
        traj_msg.s.push_back(s(i));
        traj_msg.k.push_back(k(i));
        traj_msg.speed_profile.push_back(speed_profile(i));
    }

    return traj_msg;
}


/**
 * @brief Calculates the accumulated distance (s) and curvature (k) 
 * at each point of the given trajectory
 * 
 * @param  traj_x x coordinates of the given trajectory points
 * @param  traj_y y coordinates of the given trajectory points
 * 
 * @return MatrixXd Matrix containing s and k: [s, k]
 */
MatrixXd TrajectoryOptimization::get_distance_and_curvature_values(VectorXd traj_x, VectorXd traj_y){
    //First, we get the accumulated distance at each point of the trajectory (s)
    double acum = 0;
    int n = traj_x.size();
    VectorXd s(n), xp(n), yp(n);
    s(0) = 0.;

    for(int i = 0; i < n-1; i++){
        xp(i) = traj_x(i+1) - traj_x(i);    
        yp(i) = traj_y(i+1) - traj_y(i);    
        acum += sqrt(xp(i)*xp(i) + yp(i)*yp(i));    // euclidean distance between consecutive points
        s(i+1) = acum;      
    }

    xp(n-1) = xp(n-2);      // we ensure s, xp and yp have the same data size 
    yp(n-1) = yp(n-2);      // by repeating the last X and Y differences

    //Then, we calculate the curvature at each point of the trajectory (k)
    VectorXd xpp(n), ypp(n), k(n);

    for(int i = 0; i < xp.size()-1; i++){
        xpp(i) = xp(i+1) - xp(i);
        ypp(i) = yp(i+1) - yp(i);
    }

    xpp(n-1) = xpp(n-2);
    ypp(n-1) = ypp(n-2);

    for(int i = 0; i < xpp.size() -1; i++){
        if(xp(i) != yp(i)){
            k(i) = (xp(i)*ypp(i) - xpp(i)*yp(i)) / pow((xp(i)*xp(i) + yp(i)*yp(i)), 1.5);
        } else {
            k(i) = 0.;
        }
    }

    MatrixXd res(n,2);
    res << s, k;

    return res;
}


/**
 * @brief Generates a speed profile for the trajectory
 * 
 * @param  s Accumulated distance at each point
 * @param  k Curvature at each point
 * 
 * @return VectorXd Speed profile vector
 */
VectorXd TrajectoryOptimization::generate_speed_profile(VectorXd s, VectorXd k){
    VectorXd speed_profile = VectorXd::Zero(s.size());
    speed_profile(0) = speed_;

    VectorXd v_grip(k.size());
    for(int i = 0; i < k.size(); i++){
        v_grip(i) = min(sqrt(KAyMax/abs(k(i)+0.0001)), KVMax);
    }

    VectorXd ds(speed_profile.size());
    for(int j = 1; j < speed_profile.size(); j++){
        ds(j) = s(j) - s(j-1);
    }

    for(int j = 1; j < speed_profile.size(); j++){
        speed_profile(j) = sqrt(speed_profile(j-1)*speed_profile(j-1) + 2*KAxMax*ds(j));
        if (speed_profile(j) > v_grip(j)){
            speed_profile(j) = v_grip(j);
        }
    }

    double v_max_braking;
    for(int j = speed_profile.size()-2; j > -1; j--){
        v_max_braking = sqrt(speed_profile(j+1)*speed_profile(j+1) + 2*KAxMax*ds(j));
        if(speed_profile(j) > v_max_braking){
            speed_profile(j) = v_max_braking;
        }
    }

    return speed_profile;
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryOptimization>());
    rclcpp::shutdown();
    return 0;
}