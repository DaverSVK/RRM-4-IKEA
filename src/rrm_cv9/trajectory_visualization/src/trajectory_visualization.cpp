#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit_msgs/DisplayTrajectory.h>
#include <cmath>
#include <fstream>

int main(int argc, char **argv)
{
	std::ofstream myfile;
     myfile.open ("example.txt");
    // Vytvorenie node a publishera
    ros::init(argc, argv, "trajectory_visualization");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 1);
    double jerk_1 = {0};
    double jerk_3 = {0};

    // Sprava pre trajektoriu
    moveit_msgs::RobotTrajectory trajectory;
    // Mena klbov musia byt vyplnene
    trajectory.joint_trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    float A1[6] = {0, 0, 0, 0, 0, 0};
    float A3[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    Eigen::MatrixXd m1(6,6);
    m1 <<   1,0,0,0,0,0,
            0,1,0,0,0,0,
            0,0,2,0,0,0,
            1,4,16,64,256,1024,
            0,1,8,48,256,1280,
            0,0,2,24,192,1280;
    ROS_INFO_STREAM("m1:\n" << m1);

    Eigen::MatrixXd m2(7,7);
    m2 <<   1,0,0,0,0,0,0,
            0,1,0,0,0,0,0,
            0,0,2,0,0,0,0,
            1,1,1,1,1,1,1,
            1,4,16,64,256,1024,4096,
            0,1,8,48,256,1280,6144,
            0,0,2,24,192,1280,7680;
    ROS_INFO_STREAM("m2:\n" << m2);

    Eigen::VectorXd v1(6);
    v1 << 0, 0, 0, M_PI/2, 0, 0;
    ROS_INFO_STREAM("v1 = \n" << v1);

    Eigen::VectorXd v2(7);
    v2 << 0, 0, 0, M_PI/6, 0, 0, 0;
    ROS_INFO_STREAM("v2 = \n" << v2);
    v1 = (m1.inverse() * v1);
    ROS_INFO_STREAM("m1.inverse() * v1 = \n" << v1);
    v2 = (m2.inverse() * v2);
    ROS_INFO_STREAM("m2.inverse() * v2 = \n" << v2);

    // V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
    for (double t = 0; t <= 4; t += 0.05) {

        // Vytvorenie prejazdoveho bodu
        trajectory_msgs::JointTrajectoryPoint point;

        // Robot ma 6 klbov
        point.positions.resize(6);
        point.velocities.resize(6);
        point.accelerations.resize(6);

        // Klb 1
	    point.positions[0] = v1[3]*pow(t,3) + v1[4]*pow(t,4) + v1[5]*pow(t,5);
        point.velocities[0] = 3*v1[3]*pow(t,2) + 4*v1[4]*pow(t,3) + 5*v1[5]*pow(t,4);
        point.accelerations[0] = 6*v1[3]*pow(t,1) + 12*v1[4]*pow(t,2) + 20*v1[5]*pow(t,3);
        jerk_1 = 6*v1[3] + 24*v1[4]*pow(t,1) + 60*v1[5]*pow(t,2);

        // Klb 2
        point.positions[1] = 0;
        point.velocities[1] = 0;
        point.accelerations[1] =0;

        // Klb 3
        point.positions[2] = v2[0] + v2[1] * pow(t,1) + v2[2] * pow(t,2) + v2[3] * pow(t,3) + v2[4] * pow(t,4) + v2[5] * pow(t,5) + v2[6] * pow(t,6);
        point.velocities[2] =  v2[1] + 2*v2[2]* pow(t,1) + 3*v2[3] * pow(t,2) + 4*v2[4] * pow(t,3) + 5*v2[5]*pow(t,4) + 6*v2[6] * pow(t,5);
        point.accelerations[2] =  2*v2[2]* pow(t,1) + 6 * v2[3]* pow(t,1) + 12 * v2[4] * pow(t,2) + 20 * v2[5] * pow(t,3) + 30*v2[6] * pow(t,4);
        jerk_3 =  6*v2[3] + 24*v2[4]*pow(t,1) + 60*v2[5]*pow(t,2) + 120*v2[6] * pow(t,3);
        // Klb 4
        point.positions[3] = 0;
        point.velocities[3] = 0;
        point.accelerations[3] =0;

        // Klb 5
        point.positions[4] = 0;
        point.velocities[4] = 0;
        point.accelerations[4] = 0;

        // Klb 6
        point.positions[5] = 0;
        point.velocities[5] = 0;
        point.accelerations[5] =0;

        // Vlozenie casu prejazdu
        point.time_from_start = ros::Duration(t);

        // VLozenie bodu do trajektorie
        trajectory.joint_trajectory.points.push_back(point);

        myfile << t << ";";
        myfile << point.positions[0] << ";";
        myfile << point.velocities[0] << ";";
        myfile << point.accelerations[0] << ";";
        myfile <<  jerk_1 << ";";

        myfile << point.positions[2] << ";";
        myfile << point.velocities[2] << ";";
        myfile << point.accelerations[2] << ";";
        myfile <<  jerk_3 << ";";
        myfile << "\n";
    }

    // Sprava pre vizualizaciu
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Potrebne vyplnit nazov modelu
    display_trajectory.model_id = "abb_irb";

    // Vlozenie vypocitanej trajektorie
    display_trajectory.trajectory.push_back(trajectory);

    // Publikuj trajektoriu kazde 2 sekundy
    ros::Rate loop_rate(0.5);
    while (ros::ok()) {
        publisher.publish(display_trajectory);
        loop_rate.sleep();
    }
    myfile.close();
    return 0;
}
