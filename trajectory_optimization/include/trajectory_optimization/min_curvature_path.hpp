#include <rclcpp/rclcpp.hpp>
#include "common_msgs/msg/trajectory.hpp"
#include "common_msgs/msg/point_xy.hpp"
#include "libInterpolate/Interpolate.hpp"
#include "qpmad/solver.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
using namespace std;
using namespace Eigen;

namespace MinCurvaturepath {
    //Methods declaration
    MatrixXd process_track_data(VectorXd x, VectorXd y, VectorXd twr, VectorXd twl);
    MatrixXd diff_col(MatrixXd E);
    VectorXd cumsum(VectorXd v);
    VectorXd gradient(VectorXd f);
    MatrixXd matrixH(VectorXd delx, VectorXd dely);
    MatrixXd matrixB(VectorXd xin, VectorXd yin, VectorXd delx, VectorXd dely);
    VectorXd qp_solver(MatrixXd H, MatrixXd B);

    //Main function for getting optimized trajectory
    MatrixXd get_min_curvature_path(std::vector<common_msgs::msg::PointXY> track_xy, VectorXd twr, VectorXd twl){
        int n = track_xy.size();
        VectorXd x(n), y(n);
        for(int i = 0; i < n; i++){
            x(i) = track_xy[i].x;
            y(i) = track_xy[i].y;
        }
        
        //First, we process track data 
        MatrixXd track_data = process_track_data(x, y, twr, twl);

        //Then, we form the matrices which will define the quadratic optimization problem
        VectorXd xin = track_data.col(2);
        VectorXd yin = track_data.col(3);
        VectorXd xout = track_data.col(4);
        VectorXd yout = track_data.col(5);
        
        VectorXd delx = xout - xin;
        VectorXd dely = yout - yin;

        MatrixXd H = matrixH(delx, dely);
        MatrixXd  B = matrixB(xin, yin, delx, dely);

        //Solve the quadratic problem
        VectorXd resMCP = qp_solver(H,B);

        //Co-ordinates for the resultant curve
        VectorXd xresMCP = VectorXd::Zero(n);
        VectorXd yresMCP = VectorXd::Zero(n);

        for(int i = 0; i < n; i++){
            xresMCP(i) = xin(i) + resMCP(i)*delx(i);
            yresMCP(i) = yin(i) + resMCP(i)*dely(i);
        }

        MatrixXd res(n,2);
        res << xresMCP, yresMCP;

        return res;
    }

    //Auxiliar methods
    MatrixXd process_track_data(VectorXd x, VectorXd y, VectorXd twr, VectorXd twl){
    /*interpolate data to get finer curve with equal distances between each segment
        higher no. of segments causes trajectory to follow the reference line*/
        int n = x.size();
        const int n_seg = 1000;

        MatrixXd path_x_y(n,2); path_x_y << x, y;

        MatrixXd difs2 = diff_col(path_x_y).array().square().matrix();
        VectorXd step_lengths = (difs2.col(0)+difs2.col(1)).cwiseSqrt();

        VectorXd temp_step = step_lengths;
        step_lengths.conservativeResize(step_lengths.size()+1);
        step_lengths << 0, 
                    temp_step;   // add the starting point

        VectorXd cumulative_len = cumsum(step_lengths); 
        VectorXd final_step_locks = VectorXd::LinSpaced(n_seg, 0, cumulative_len(cumulative_len.size()-1));
    
        int m = final_step_locks.size();
        
        MatrixXd final_path_x_y(m, 2);
        _1D::LinearInterpolator<double> interp;
        interp.setData(cumulative_len, x);
        for(int i = 0; i < m; i++){
            final_path_x_y(i, 0) = interp(final_step_locks(i));
        }

        interp.setData(cumulative_len, y);
        for(int i = 0; i < m; i++){
            final_path_x_y(i, 1) = interp(final_step_locks(i));
        }

        VectorXd xt = final_path_x_y.col(0);
        VectorXd yt = final_path_x_y.col(1);

        
        _1D::CubicSplineInterpolator<double> interp_c;
        interp_c.setData(cumulative_len, twr);
        VectorXd twrt(m);
        for(int i = 0; i < m; i++){
            twrt(i) = interp_c(final_step_locks(i));
        }

        interp_c.setData(cumulative_len, twl);
        VectorXd twlt(m);
        for(int i = 0; i < m; i++){
            twlt(i) = interp_c(final_step_locks(i));
        }

        //normal direction for each vertex
        VectorXd dx = gradient(xt); auto dx_a = dx.array();
        VectorXd dy = gradient(yt); auto dy_a = dy.array();
        VectorXd dL = (dx_a*dx_a + dy_a*dy_a).sqrt().matrix();

        //Offset data
        MatrixXd offset(m, 2);
        offset << -twrt, twlt;
        VectorXd xin = VectorXd::Zero(m); 
        VectorXd yin = VectorXd::Zero(m);
        VectorXd xout = VectorXd::Zero(m);
        VectorXd yout = VectorXd::Zero(m);

        VectorXd aux_x = (dy.array()/dL.array()).matrix();
        VectorXd aux_y = (dx.array()/dL.array()).matrix();

        for(int i = 0; i < m; i++){
            VectorXd xinv, yinv, xoutv, youtv;
            
            xinv = -offset(i,0)*aux_x + xt;    //get inner offset curve
            yinv = offset(i,0)*aux_y + yt;

            xoutv = -offset(i,1)*aux_x + xt;   //get outer offset curve
            youtv = offset(i,1)*aux_y + yt;

            xin(i) = xinv(i);
            yin(i) = yinv(i);
            xout(i) = xoutv(i);
            yout(i) = youtv(i);
        }
        
        MatrixXd track_data(m,6);
        track_data << xt, yt, xin, yin, xout, yout;

        return track_data;
    }

    MatrixXd diff_col(MatrixXd E) { 
        MatrixXd E1 = E.block(0, 0, E.rows()-1, E.cols());
        MatrixXd E2 = E.block(1, 0, E.rows()-1, E.cols());
        return E2 - E1;
    }

    VectorXd cumsum(VectorXd v){
        int n = v.size(); VectorXd res(n);

        res(0) = v(0);
        for(int i = 1; i < n; i++){
            res(i) = v.head(i+1).sum();
        }

        return res;
    }

    VectorXd gradient(VectorXd f){
        int n = f.size();
        VectorXd grad(n);

        grad(0) = f(1)-f(0);
        for(int i = 1; i < n-1; i++){
            grad(i) = (f(i+1)-f(i-1))/2;
        }
        grad(n-1) = f(n-1)-f(n-2);

        return grad;
    }

    MatrixXd matrixH(VectorXd delx, VectorXd dely){
        int n = delx.size();
        MatrixXd H = MatrixXd::Zero(n,n);

        for(int i = 1; i < n-1; i++ ){
            //First row
            H(i-1,i-1) = H(i-1,i-1) + pow(delx(i-1),2)     + pow(dely(i-1),2);
            H(i-1,i)   = H(i-1,i)   - 2*delx(i-1)*delx(i) - 2*dely(i-1)*dely(i);
            H(i-1,i+1) = H(i-1,i+1) + delx(i-1)*delx(i+1) + dely(i-1)*dely(i+1);

            //Second row
            H(i,i-1)   = H(i,i-1)   - 2*delx(i-1)*delx(i) - 2*dely(i-1)*dely(i);
            H(i,i)     = H(i,i )    + 4*pow(delx(i),2)    + 4*pow(dely(i),2);
            H(i,i+1)   = H(i,i+1)   - 2*delx(i)*delx(i+1) - 2*dely(i)*dely(i+1);

            //Third row
            H(i+1,i-1) = H(i+1,i-1) + delx(i-1)*delx(i+1) + dely(i-1)*dely(i+1);
            H(i+1,i)   = H(i+1,i)   - 2*delx(i)*delx(i+1) - 2*dely(i)*dely(i+1);
            H(i+1,i+1) = H(i+1,i+1) + pow(delx(i+1),2)    + pow(dely(i+1),2);
        }

        return H;
    }

    MatrixXd matrixB(VectorXd xin, VectorXd yin, VectorXd delx, VectorXd dely){
        int n = delx.size();
        MatrixXd B = MatrixXd::Zero(1,n);

        for(int i = 1; i < n-1; i++){
            B(0,i-1) = B(0,i-1) + 2*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i-1) + 2*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i-1);
            B(0,i)   = B(0,i)   - 4*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i)   - 4*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i);
            B(0,i+1) = B(0,i+1) + 2*(xin(i+1)+xin(i-1)-2*xin(i))*delx(i+1) + 2*(yin(i+1)+yin(i-1)-2*yin(i))*dely(i+1);
        }

        return B;
    }

    VectorXd qp_solver(MatrixXd H, MatrixXd B){
        //Define constraints
        int n = H.rows();
        VectorXd lb = VectorXd::Zero(n);
        VectorXd ub = VectorXd::Ones(n);

        //If start and end points are the same. 
        //Solver doesn't handle equality constraints, but we can implement it as two inequalities
        // beq <= Aeq*res <= beq
        MatrixXd Aeq = MatrixXd::Zero(1,n);
        Aeq(0) = 1;
        Aeq(n-1) = -1;
        double beq = 0;

        MatrixXd A(1, n); 
        A.row(0)= Aeq;
    

        VectorXd Alb(1), Aub(1);
        Alb << beq;
        Aub << beq;

        //Solver
        VectorXd res;
        qpmad::Solver solver;

        H << 2*H;
        qpmad::Solver::ReturnStatus status = solver.solve(res, H, B.transpose(), lb, ub, A, Alb, Aub);
        if (status != qpmad::Solver::OK)
        {
            std::cerr << "Error" << std::endl;
        }

        return res;
    }
};

