#ifndef _UR_KINEMATICS_H_
#define _UR_KINEMATICS_H_

#include<iostream>
#include<cmath>
#include<Eigen/Dense>
#include <array>

#define Eigen Eigen // 替换一下，防止Eigen库不一样

class Ur_Kinematics
{

public:

    double d[6] = {0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0};
    double a[6] = {0.0, 0.0, 0.42500, 0.39225, 0.0, 0.0};
    double alph[6] = {0.0, M_PI/2, 0.0, 0.0, M_PI/2, -M_PI/2};

   

    

    struct inverse_8theta
    {
        double th[6][8];
    };

    double now_theta[6];  // 从编码器读出的当前的关节角度 ！！！

    Eigen::MatrixXd AH(int n, double* theta);
    Eigen::Matrix4d forward(double* theta);
    inverse_8theta inverse(Eigen::Matrix4d desired_pos);
    

private:

   
    double default_theta = 100;                                 // 逆解无解时赋值的角度大小
    
};



          


#endif
