#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lala_node");
    ros::NodeHandle nh;

    int QUADROTOR_N = 2;
    int QUADROTOR_NY = 11;
    
    Eigen::VectorXd vec(11);
    vec << 1,2,3,4,5,6,7,8,9,10,11;

    double array[QUADROTOR_N+1][QUADROTOR_NY];

    for (int i = 0; i < QUADROTOR_N+1; ++i){
        for (int j = 0; j < QUADROTOR_NY; ++j){
            array[i][j] = vec(j);
            std::cout<<("%f ", array[i][j]);
        }
        std::cout<<std::endl;
    }

    Eigen::MatrixXd mat(QUADROTOR_N+1, QUADROTOR_NY);

    mat << 1,2,3,4,5,6,7,8,9,10,11,
            1,2,3,4,5,6,7,8,9,10,11,
            1,2,3,4,5,6,7,8,9,10,11;
    
    std::cout<<mat<<std::endl;

    for (int i = 0; i < QUADROTOR_N+1; ++i){
        for (int j = 0; j < QUADROTOR_NY; ++j){
            array[i][j] = mat(i,j);
            std::cout<<("%f ", array[i][j]);
        }
        std::cout<<std::endl;
    }

    return 0;
}


