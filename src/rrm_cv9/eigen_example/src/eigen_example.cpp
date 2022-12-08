#include <ros/ros.h>
#include <Eigen/Geometry>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "eigen_example");

    // Inicializacia vektora po prvkoch
    Eigen::VectorXd v1(6);
    v1 << 0, 0, 0, M_PI/2, 0, 0;
    ROS_INFO_STREAM("v1 = \n" << v1);

    Eigen::VectorXd v3(6);
    v3 << 1, 1.6, 0, 0, 1, 0;
    ROS_INFO_STREAM("v2 = \n" << v3);

    Eigen::VectorXd v4(6);
    v4 << 0, 1, 0, 0, 1, M_PI/2;
    ROS_INFO_STREAM("v2 = \n" << v4);

    Eigen::VectorXd v5(6);
    v5 << 0, 1, M_PI/2, 0, 1, M_PI/2;
    ROS_INFO_STREAM("v2 = \n" << v5);

    Eigen::VectorXd v6(6);
    v6 << 0, 1, M_PI/2, 0.5, 1, M_PI/2;
    ROS_INFO_STREAM("v2 = \n" << v6);
    // Iniclizacia nuloveho vektora
    // Eigen::VectorXd v2 = Eigen::VectorXd::Zero(3);
    // ROS_INFO_STREAM("v2 = \n" << v2);

    // Inicializacia 4x4 matice po prvkoch
    Eigen::MatrixXd m1(6,6);
    m1 <<   1,0,0,0,0,0,
            0,1,0,0,0,0,
            0,0,2,0,0,0,
            1,1,1,1,1,1,
            0,1,2,3,4,5,
            0,0,2,6,12,20;
    ROS_INFO_STREAM("m1:\n" << m1);

    /*  Eigen::MatrixXd m2(9,9);
    m2 <<   0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,
            0,0,0,1,1,1,1,1,1,
            0,0,0,3,4,5,6,7,8,
            0,0,0,6,12,20,30,42,56;
            0,0,0,64,256,1024,4096,16384,65536,
            0,0,0,48,256,1280,6144,28672,131072,
            0,0,0,24,192,1280,7680,43008,229376;
    ROS_INFO_STREAM("m2:\n" << m2);*/

    // Vytvorenie identickej matice
    // Eigen::MatrixXd m2 = Eigen::MatrixXd::Identity(3,3);
    // ROS_INFO_STREAM("m2 = \n" << m2);

    // Vytvorenie nulovej matice
    // Eigen::MatrixXd m3 = Eigen::MatrixXd::Zero(3,3);
    // ROS_INFO_STREAM("m2 = \n" << m3);

    // sucet
    // ROS_INFO_STREAM("m1 + m2 = \n" << (m1 + m2));

    // nasobenie
    // ROS_INFO_STREAM("m1 * v1 = \n" << (m1 * v1));

    // inverzia matice
    // ROS_INFO_STREAM("m1^-1 = \n" << (m1.inverse()));

    // Skalarny sucin
    // ROS_INFO_STREAM("v1 . v2 = \n" << v1.dot(v2));

    // nasobenie + inverzia
    ROS_INFO_STREAM("m1.inverse() * v1 = \n" << (m1.inverse() * v1));
    ROS_INFO_STREAM("m3.inverse() * v1 = \n" << (m1.inverse() * v3));
    ROS_INFO_STREAM("m4.inverse() * v1 = \n" << (m1.inverse() * v4));
    ROS_INFO_STREAM("m5.inverse() * v1 = \n" << (m1.inverse() * v5));
    ROS_INFO_STREAM("m6.inverse() * v1 = \n" << (m1.inverse() * v6));
    // Vektorovy sucin
    // Eigen::Vector3d x1 = {1,2,3};
    // Eigen::Vector3d x2 = {1,0,0};
    // ROS_INFO_STREAM(" [1,2,3] x [1,0,0] = \n" << x1.cross(x2));
    return 0;
}
