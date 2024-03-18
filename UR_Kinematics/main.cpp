#include"UR_Kinematics.h"
#include <stdio.h>
#include <iomanip>


double* dada()
{

    double *a = new double[6];
    
    //  = {1,2,3,4,5,6};
    return a;
}

int main()
{
    Ur_Kinematics kinematics;

    double theta[6] = {0.01, 1.11, 1.29, 2, 1.0, 0};


    Eigen::MatrixXd matrix44;

    matrix44 = kinematics.forward(theta);

    Ur_Kinematics::inverse_8theta inverse_theta;

    inverse_theta = kinematics.inverse(matrix44);


   
    for (int i = 0; i < 8; i++) {
        std::cout << "第" << std::setw(2) << i << "组解: ";

        for (int j = 0; j < 6; j++) {
            std::cout << std::setw(15) << std::right << inverse_theta.th[j][i] << ' ';
        }

        std::cout << std::endl;
    }

    return 0;

}
