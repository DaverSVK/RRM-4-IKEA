//
// Created by david on 5.12.2022.
//
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit_msgs/DisplayTrajectory.h>
#include <cmath>
#include <fstream>
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

Eigen::VectorXd MatrixMaker4(float tn, float tn1, float con1, float con2, float con3, float con4);
Eigen::VectorXd MatrixMaker5(float tn, float tn1, float tn2, float con1, float con2, float con3, float con4, float con5);
Eigen::VectorXd MatrixMaker6(float tn, float tn1, float con1, float con2, float con3, float con4, float con5, float con6);
std::vector<std::vector<double>> IKEAsolver(double x, double y, double z, double rx, double ry, double rz);

int main(int argc, char **argv)
{
    std::ofstream myfile;
    myfile.open ("example.txt");
    // Vytvorenie node a publishera
    ros::init(argc, argv, "trajectory_visualization_part2");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 1);
    std::vector<std::vector<double>> solutions;
    Eigen::VectorXd solution_final(6);
    double place_holder_solution = 0;
    double place_holder_compare_2 = 0;

    Eigen::VectorXd solution_previous(6);
    // Sprava pre trajektoriu
    moveit_msgs::RobotTrajectory trajectory;
    // Mena klbov musia byt vyplnene
    trajectory.joint_trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    float A1[6] = {0, 0, 0, 0, 0, 0};
    float A3[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};


    // V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
    for (double t = 0; t <= 9; t += 0.05) {
//iksolver
        Eigen::VectorXd U1 = MatrixMaker4(0,1,1.6,0,1,0);
        Eigen::VectorXd U2 = MatrixMaker4(1,2,0,0,M_PI/2,0);
        Eigen::VectorXd U3 = MatrixMaker5(3,4,5,0,0,0.5,0.5,0);
        Eigen::VectorXd U4 = MatrixMaker5(3,4,5,1,0,1,1.6,0);
        Eigen::VectorXd U5 = MatrixMaker6(3,5,M_PI/2,0,M_PI/2,0,M_PI/2,0);
        Eigen::VectorXd U6 = MatrixMaker4(5,9,0.5,0,0,0);

        Eigen::MatrixXd m_IKEA(9,6);
        m_IKEA <<   1,0,U1(0)+U1(1)*pow(t,1)+U1(2)*pow(t,2)+U1(3)*pow(t,3),0,M_PI/2,0,
                  1,0,1,0,M_PI/2,U2(0)+U2(1)*pow(t,1)+U2(2)*pow(t,2)+U2(3)*pow(t,3),
                  1,0,1,0,M_PI/2,M_PI/2,
                  1,U3(0)+U3(1)*pow(t,1)+U3(2)*pow(t,2)+U3(3)*pow(t,3)+U3(4)*pow(t,4),U4(0)+U4(1)*pow(t,1)+U4(2)*pow(t,2)+U4(3)*pow(t,3)+U4(4)*pow(t,4),0,M_PI/2,U5(0)+U5(1)*pow(t,1)+U5(2)*pow(t,2)+U5(3)*pow(t,3)+U5(4)*pow(t,4)+U5(5)*pow(t,5),
                  1,U3(0)+U3(1)*pow(t,1)+U3(2)*pow(t,2)+U3(3)*pow(t,3)+U3(4)*pow(t,4),U4(0)+U4(1)*pow(t,1)+U4(2)*pow(t,2)+U4(3)*pow(t,3)+U4(4)*pow(t,4),0,M_PI/2,U5(0)+U5(1)*pow(t,1)+U5(2)*pow(t,2)+U5(3)*pow(t,3)+U5(4)*pow(t,4)+U5(5)*pow(t,5),
                  1,U6(0)+U6(1)*pow(t,1)+U6(2)*pow(t,2)+U6(3)*pow(t,3),1.6,0,M_PI/2,0;
        ROS_INFO_STREAM("m_IKEA:\n" << m_IKEA);

        if(t<=1){
            solutions = IKEAsolver(m_IKEA(0,0),m_IKEA(0,1),m_IKEA(0,2),m_IKEA(0,3),m_IKEA(0,4),m_IKEA(0,5));
        }
        if(t>1 && t<=2){
            solutions = IKEAsolver(m_IKEA(1,0),m_IKEA(1,1),m_IKEA(1,2),m_IKEA(1,3),m_IKEA(1,4),m_IKEA(1,5));
        }
        if(t>2 && t<=3){
            solutions = IKEAsolver(m_IKEA(2,0),m_IKEA(2,1),m_IKEA(2,2),m_IKEA(2,3),m_IKEA(2,4),m_IKEA(2,5));
        }
        if(t>3 && t<=5){
            solutions = IKEAsolver(m_IKEA(3,0),m_IKEA(3,1),m_IKEA(3,2),m_IKEA(3,3),m_IKEA(3,4),m_IKEA(3,5));
        }
        if(t>5 && t<=9){
            solutions = IKEAsolver(m_IKEA(5,0),m_IKEA(5,1),m_IKEA(5,2),m_IKEA(5,3),m_IKEA(5,4),m_IKEA(5,5));
        }

        double place_holder_compare = abs(solutions[0][0]) + abs(solutions[0][1]) + abs(solutions[0][2]) + abs(solutions[0][3]) + abs(solutions[0][4]) + abs(solutions[0][5]);
//        ROS_INFO_STREAM("m_IKEA:\n" << place_holder_compare);
//        ROS_INFO_STREAM("m_IKEA:\n" << solutions.size());
        for (int i=0; i<solutions.size();i++) {
            double place_holder = 0;
            for (int j=0; j<6;j++) {
                place_holder += abs(abs(solution_previous(j))-abs(solutions[i][j]));
                ROS_INFO_STREAM("m_IKEA:\n" << place_holder);
            }
            if(place_holder_compare > place_holder){
                place_holder_compare = place_holder;
                solution_final << solutions[i][0],solutions[i][1],solutions[i][2],solutions[i][3],solutions[i][4],solutions[i][5];
            }

        }
        solution_previous << solution_final;
        ROS_INFO_STREAM("m_IKEA:\n" << solution_final);

        // Vytvorenie prejazdoveho bodu
        trajectory_msgs::JointTrajectoryPoint point;

        // Robot ma 6 klbov
        point.positions.resize(6);

        // Klb 1
        point.positions[0] = solution_final(0);

        // Klb 2
        point.positions[1] = solution_final(1);

        // Klb 3
        point.positions[2] = solution_final(2);

        // Klb 4
        point.positions[3] = solution_final(3);

        // Klb 5
        point.positions[4] = solution_final(4);

        // Klb 6
        point.positions[5] = solution_final(5);

        // Vlozenie casu prejazdu
        point.time_from_start = ros::Duration(t);

        // VLozenie bodu do trajektorie
        trajectory.joint_trajectory.points.push_back(point);

        myfile << t << ";";
        myfile << point.positions[0] << ";";
        myfile << point.positions[2] << ";";
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
Eigen::VectorXd MatrixMaker4(float tn, float tn1, float con1, float con2, float con3, float con4) {
    Eigen::VectorXd vector(4);
    vector << con1, con2, con3, con4;

    Eigen::MatrixXd m4(4,4);
    m4 <<   1,  pow(tn,1),    pow(tn,2),      pow(tn,3),
            0,  1,                  2*pow(tn,1),    3*pow(tn,2),
            1,  pow(tn1,1),   pow(tn1,2),     pow(tn1,3),
            0,  1,                  2*pow(tn1,1),   3*pow(tn1,2);
    vector = (m4.inverse() * vector);

    return vector;
}
Eigen::VectorXd MatrixMaker5(float tn, float tn1, float tn2, float con1, float con2, float con3, float con4, float con5) {
    Eigen::VectorXd vector(5);
    vector << con1, con2, con3, con4, con5;

    Eigen::MatrixXd m5(5,5);
    m5 <<   1, pow(tn,1), pow(tn,2),   pow(tn,3),   pow(tn,4),
            0, 1,               2*pow(tn,1), 3*pow(tn,2), 4*pow(tn,3),
            1, pow(tn1,1),pow(tn1,2),  pow(tn1,3),  pow(tn1,4),
            1, pow(tn2,1),pow(tn2,2),  pow(tn2,3),  pow(tn2,4),
            0, 1,               2*pow(tn2,1),3*pow(tn2,2),4*pow(tn2,3);
            vector = (m5.inverse() * vector);

    return vector;
}
Eigen::VectorXd MatrixMaker6(float tn, float tn1, float con1, float con2, float con3, float con4, float con5, float con6) {
    Eigen::VectorXd vector(6);
    vector << con1, con2, con3, con4, con5, con6;

    Eigen::MatrixXd m6(6,6);
    m6 <<   1,  pow(tn,1),pow(tn,2),      pow(tn,3),      pow(tn,4),    pow(tn,5),
            0,  1,              2*pow(tn,1),    3*pow(tn,2),    4*pow(tn,3),  5*pow(tn,4),
            0,  0,              2,                    6*pow(tn,1),  12*pow(tn,2),   20*pow(tn,3),
            1,  pow(tn1,1),pow(tn1,2),    pow(tn1,3),     pow(tn1,4),   pow(tn1,5),
            0,  1,              2*pow(tn1,1),   3*pow(tn1,2),   4*pow(tn1,3), 5*pow(tn1,4),
            0,  0,              2,                    6*pow(tn1,1), 12*pow(tn1,2),  20*pow(tn1,3);
            vector = (m6.inverse() * vector);

    return vector;
}

std::vector<std::vector<double>> IKEAsolver(double x, double y, double z, double rx, double ry, double rz) {
    ros::NodeHandle n;
    //x,Eigen::VectorXd y,Eigen::VectorXd z,Eigen::VectorXd dx,Eigen::VectorXd dy, Eigen::VectorXd dz

    // Vytvorenie modelu z urdf
    robot_model_loader::RobotModelLoader loader("robot_description");

    // Vyber move group a IK algoritmu
    robot_state::JointModelGroup* joint_model_group = loader.getModel()->getJointModelGroup("robot");
    const kinematics::KinematicsBaseConstPtr& solver = joint_model_group->getSolverInstance();


    // Vytvorenie cielovej polohy x, y, z, rx, ry, rz.
    Eigen::Affine3d target = Eigen::Translation3d(Eigen::Vector3d(x, y, z))*
                             Eigen::AngleAxisd(rx,Eigen::Vector3d::UnitX())*
                             Eigen::AngleAxisd(ry,Eigen::Vector3d::UnitY())*
                             Eigen::AngleAxisd(rz,Eigen::Vector3d::UnitZ());

    // Konverzia z Eigen do geometry_msgs
    geometry_msgs::Pose target_msg;
    tf::poseEigenToMsg(target, target_msg);

    // Premenne vystupujuce vo funkcii getPositionIK
    std::vector<geometry_msgs::Pose> ik_poses = {target_msg};   // Vlozenie cielu do pola
    std::vector<std::vector<double> > solutions;                // Vystup z funkcie s mnozinou rieseni
    std::vector<double> ik_seed_state = {0,0,0,0,0,0};          // Odhadovane riesenie - potrebne iba pre analyticke algoritmy
    kinematics::KinematicsResult result;                        // exit_code z funkcie (ci bol vypocet uspesny)

    // Vypocet inverznej kinematiky
    solver->getPositionIK(ik_poses,ik_seed_state, solutions, result, kinematics::KinematicsQueryOptions());

    // Overenie ci bol vypocet uspesny
    if (result.kinematic_error != kinematics::KinematicError::OK) {
        throw std::runtime_error("Unable to compute IK. Error: " + std::to_string(result.kinematic_error));
    }

    // Vypis riesenii
    for (const auto &solution: solutions) {
        ROS_INFO("Solution found: ");
        for (const auto &joint : solution) {
            ROS_INFO_STREAM(std::to_string(joint));
        }
    }

    return solutions;
}

