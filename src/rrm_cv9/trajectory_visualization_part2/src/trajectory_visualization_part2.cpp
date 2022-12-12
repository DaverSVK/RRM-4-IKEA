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
Eigen::VectorXd MatrixMaker6(float tn, float tn1, float tn2, float con1, float con2, float con3, float con4, float con5, float con6);
Eigen::VectorXd MatrixMaker66(float tn, float tn1, float con1, float con2, float con3, float con4, float con5, float con6);
Eigen::VectorXd MatrixMaker7(float tn, float tn1, float tn2, float con1, float con2, float con3, float con4, float con5, float con6, float con7);
Eigen::VectorXd MatrixMaker9(float tn, float tn1, float tn2, float con1, float con2, float con3, float con4, float con5, float con6, float con7, float con8, float con9);
std::vector<std::vector<double>> IKEAsolver(double x, double y, double z, double rx, double ry, double rz);

int main(int argc, char **argv)
{
    std::ofstream myfile;
    myfile.open ("example2.txt");
    // Vytvorenie node a publishera
    ros::init(argc, argv, "trajectory_visualization_part2");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<moveit_msgs::DisplayTrajectory>("trajectory", 1);
    std::vector<std::vector<double>> solutions;
    Eigen::VectorXd solution_final(6);
    double y_position_tool =0;
    double y_speeeeed_tool =0;
    double y_acceleration_tool =0;
    double z_position_tool =0;
    double z_speeeeed_tool =0;
    double z_acceleration_tool =0;
    double rz_position_tool =0;
    double rz_speeeeed_tool =0;
    double rz_acceleration_tool =0;
    Eigen::VectorXd solution_previous(6);
    // Sprava pre trajektoriu
    moveit_msgs::RobotTrajectory trajectory;
    // Mena klbov musia byt vyplnene
    trajectory.joint_trajectory.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};


    // V cykle sa vytvori trajektoria, kde pre ukazku kazdy klb bude mat hodnota q(t) = t*0.5
    for (double t = 0; t <= 9; t += 0.05) {
//iksolver
        Eigen::VectorXd U1 = MatrixMaker4(0,1,1.6,0,1,0);
        Eigen::VectorXd U2 = MatrixMaker4(1,2,0,0,M_PI/2,0);
        Eigen::VectorXd U3 = MatrixMaker5(3,4,5,0,0,0.5,0.5,0);
        Eigen::VectorXd U4 = MatrixMaker5(3,4,5,1,0,1,1.6,0);
        Eigen::VectorXd U5 = MatrixMaker6(3,4, 5,M_PI/2,0,M_PI/2,0,0,0);
        Eigen::VectorXd U6 = MatrixMaker4(5,9,0.5,0, 0, 0);

        Eigen::VectorXd URZ35 = MatrixMaker9(3,4, 5,M_PI/2,0,0,M_PI/2,0,0,0,0,0);
        Eigen::VectorXd URZ12 = MatrixMaker66(1,2,0,0,0,M_PI/2,0,0);
        Eigen::VectorXd UZ1 = MatrixMaker66(0,1,1.6,0,0,1,0,0);
        Eigen::VectorXd UZ35 = MatrixMaker7(3,4,5,1,0,0,1,1.6,0,0);
        Eigen::VectorXd UY9 = MatrixMaker66(5,9,0.5,0,0,0,0,0);
        Eigen::VectorXd UY35 = MatrixMaker7(3,4,5,0,0,0,0.5,0.5,0,0);

        Eigen::MatrixXd m_IKEA(6,6);
        m_IKEA <<   1,0,U1(0)+U1(1)*pow(t,1)+U1(2)*pow(t,2)+U1(3)*pow(t,3),0,M_PI/2,0,
                  1,0,1,0,M_PI/2,U2(0)+U2(1)*pow(t,1)+U2(2)*pow(t,2)+U2(3)*pow(t,3),
                  1,0,1,0,M_PI/2,M_PI/2,
                  1,U3(0)+U3(1)*pow(t,1)+U3(2)*pow(t,2)+U3(3)*pow(t,3)+U3(4)*pow(t,4),U4(0)+U4(1)*pow(t,1)+U4(2)*pow(t,2)+U4(3)*pow(t,3)+U4(4)*pow(t,4),0,M_PI/2,U5(0)+U5(1)*pow(t,1)+U5(2)*pow(t,2)+U5(3)*pow(t,3)+U5(4)*pow(t,4)+U5(5)*pow(t,5),
                  1,U3(0)+U3(1)*pow(t,1)+U3(2)*pow(t,2)+U3(3)*pow(t,3)+U3(4)*pow(t,4),U4(0)+U4(1)*pow(t,1)+U4(2)*pow(t,2)+U4(3)*pow(t,3)+U4(4)*pow(t,4),0,M_PI/2,U5(0)+U5(1)*pow(t,1)+U5(2)*pow(t,2)+U5(3)*pow(t,3)+U5(4)*pow(t,4)+U5(5)*pow(t,5),
                  1,U6(0)+U6(1)*pow(t,1)+U6(2)*pow(t,2)+U6(3)*pow(t,3),1.6,0,M_PI/2,0;
        ROS_INFO_STREAM("m_IKEA:\n" << m_IKEA);

        if(t<=1){
            solutions = IKEAsolver(m_IKEA(0,0),m_IKEA(0,1),m_IKEA(0,2),m_IKEA(0,3),m_IKEA(0,4),m_IKEA(0,5));
            z_speeeeed_tool =U1(1)+U1(2)*2*pow(t,1)+U1(3)*3*pow(t,2);
            z_position_tool =U1(0)+U1(1)*pow(t,1)+U1(2)*pow(t,2)+U1(3)*pow(t,3);
            z_acceleration_tool = UZ1(2)*2+UZ1(3)*6*pow(t,1)+UZ1(4)*12*pow(t,2)+UZ1(5)*20*pow(t,3);

            y_speeeeed_tool =0;
            y_position_tool =0;
            y_acceleration_tool = 0;

            rz_position_tool =0;
            rz_speeeeed_tool =0;
            rz_acceleration_tool =0;
        }
        if(t>1 && t<=2){
            solutions = IKEAsolver(m_IKEA(1,0),m_IKEA(1,1),m_IKEA(1,2),m_IKEA(1,3),m_IKEA(1,4),m_IKEA(1,5));
            z_speeeeed_tool =0;
            z_position_tool =1;
            z_acceleration_tool =0;

            y_speeeeed_tool =0;
            y_position_tool =0;
            y_acceleration_tool = 0;

            rz_position_tool =U2(0)+U2(1)*pow(t,1)+U2(2)*pow(t,2)+U2(3)*pow(t,3);
            rz_speeeeed_tool =U2(1)+U2(2)*2*pow(t,1)+U2(3)*3*pow(t,2);
            rz_acceleration_tool = URZ12(2)*2+URZ12(3)*6*pow(t,1)+URZ12(4)*12*pow(t,2)+URZ12(5)*20*pow(t,3);;
        }
        if(t>2 && t<=3){
            solutions = IKEAsolver(m_IKEA(2,0),m_IKEA(2,1),m_IKEA(2,2),m_IKEA(2,3),m_IKEA(2,4),m_IKEA(2,5));
            z_speeeeed_tool =0;
            z_position_tool =1;
            z_acceleration_tool=0;

            y_speeeeed_tool =0;
            y_position_tool =0;
            y_acceleration_tool = 0;

            rz_position_tool =M_PI/2;
            rz_speeeeed_tool =0;
            rz_acceleration_tool =0;
        }
        if(t>3 && t<=5){
            solutions = IKEAsolver(m_IKEA(3,0),m_IKEA(3,1),m_IKEA(3,2),m_IKEA(3,3),m_IKEA(3,4),m_IKEA(3,5));
            z_speeeeed_tool =U4(1)+U4(2)*2*pow(t,1)+U4(3)*3*pow(t,2)+U4(4)*4*pow(t,3);
            z_position_tool =U4(0)+U4(1)*pow(t,1)+U4(2)*pow(t,2)+U4(3)*pow(t,3)+U4(4)*pow(t,4);
            z_acceleration_tool=UZ35(2)*2+UZ35(3)*6*pow(t,1)+UZ35(4)*12*pow(t,2)+UZ35(5)*20*pow(t,3)+UZ35(6)*30*pow(t,4);

            y_speeeeed_tool =U3(1)+U3(2)*2*pow(t,1)+U3(3)*3*pow(t,2)+U3(4)*4*pow(t,3);
            y_position_tool =U3(0)+U3(1)*pow(t,1)+U3(2)*pow(t,2)+U3(3)*pow(t,3)+U3(4)*pow(t,4);
            y_acceleration_tool = UY35(2)*2+UY35(3)*6*pow(t,1)+UY35(4)*12*pow(t,2)+UY35(5)*20*pow(t,3)+UY35(6)*30*pow(t,4);

            rz_position_tool =U5(0)+U5(1)*pow(t,1)+U5(2)*pow(t,2)+U5(3)*pow(t,3)+U5(4)*pow(t,4)+U5(5)*pow(t,5);
            rz_speeeeed_tool =U5(1)+U5(2)*2*pow(t,1)+U5(3)*3*pow(t,2)+U5(4)*4*pow(t,3)+U5(5)*5*pow(t,4);
            rz_acceleration_tool = URZ35(2)*2+URZ35(3)*6*pow(t,1)+URZ35(4)*12*pow(t,2)+URZ35(5)*20*pow(t,3)+URZ35(6)*30*pow(t,4)+URZ35(7)*42*pow(t,5)+URZ35(8)*56*pow(t, 6);
        }
        if(t>5 && t<=9){
            solutions = IKEAsolver(m_IKEA(5,0),m_IKEA(5,1),m_IKEA(5,2),m_IKEA(5,3),m_IKEA(5,4),m_IKEA(5,5));
            z_speeeeed_tool =0;
            z_position_tool =1.6;
            z_acceleration_tool=0;

            y_speeeeed_tool =U6(1)+U6(2)*2*pow(t,1)+U6(3)*3*pow(t,2);
            y_position_tool =U6(0)+U6(1)*pow(t,1)+U6(2)*pow(t,2)+U6(3)*pow(t,3);
            y_acceleration_tool = UY9(2)*2+UY9(3)*6*pow(t,1)+UY9(4)*12*pow(t,2)+UY9(5)*20*pow(t,3);

            rz_position_tool =0;
            rz_speeeeed_tool =0;
            rz_acceleration_tool =0;
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
        myfile << y_position_tool << ";";
        myfile << y_speeeeed_tool << ";";
        myfile << y_acceleration_tool << ";";
        myfile << z_position_tool << ";";
        myfile << z_speeeeed_tool << ";";
        myfile << z_acceleration_tool << ";";
        myfile << rz_position_tool << ";";
        myfile << rz_speeeeed_tool << ";";
        myfile << rz_acceleration_tool << ";";
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


//___________MATICE____________________________________________________________________________________________________________________

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
Eigen::VectorXd MatrixMaker6(float tn, float tn1, float tn2, float con1, float con2, float con3, float con4, float con5, float con6) {
    Eigen::VectorXd vector(6);
    vector << con1, con2, con3, con4, con5, con6;

    Eigen::MatrixXd m6(6,6);
    m6 <<   1,  pow(tn,1),pow(tn,2),      pow(tn,3),      pow(tn,4),    pow(tn,5),
            0,  1,              2*pow(tn,1),    3*pow(tn,2),    4*pow(tn,3),  5*pow(tn,4),
            1,  pow(tn1,1),pow(tn1,2),    pow(tn1,3),     pow(tn1,4),   pow(tn1,5),
            0,  1,              2*pow(tn1,1),   3*pow(tn1,2),   4*pow(tn1,3), 5*pow(tn1,4),
            1,  pow(tn2,1),pow(tn2,2),    pow(tn2,3),     pow(tn2,4),   pow(tn2,5),
            0,  1,              2*pow(tn2,1),   3*pow(tn2,2),   4*pow(tn2,3), 5*pow(tn2,4);
            vector = (m6.inverse() * vector);

    return vector;
}

//________________________________________Matrix for Acc _____dont need but i tried different method_________________________
Eigen::VectorXd MatrixMaker66(float tn, float tn1, float con1, float con2, float con3, float con4, float con5, float con6) {
    Eigen::VectorXd vector(6);
    vector << con1, con2, con3, con4, con5, con6;

    Eigen::MatrixXd m6(6,6);
    m6 <<   1,  pow(tn,1),pow(tn,2),      pow(tn,3),      pow(tn,4),    pow(tn,5),
            0,  1,              2*pow(tn,1),    3*pow(tn,2),    4*pow(tn,3),  5*pow(tn,4),
            0,  0,              2,                    6*pow(tn,1),  12*pow(tn,2),   20*pow(tn,3),
            1,  pow(tn1,1),pow(tn1,2),      pow(tn1,3),      pow(tn1,4),    pow(tn1,5),
            0,  1,              2*pow(tn1,1),    3*pow(tn1,2),    4*pow(tn1,3),  5*pow(tn1,4),
            0,  0,              2,                    6*pow(tn1,1),  12*pow(tn1,2),   20*pow(tn1,3);

            vector = (m6.inverse() * vector);

    return vector;
}
Eigen::VectorXd MatrixMaker7(float tn, float tn1, float tn2, float con1, float con2, float con3, float con4, float con5, float con6, float con7) {
    Eigen::VectorXd vector(7);
    vector << con1, con2, con3, con4, con5, con6, con7;

    Eigen::MatrixXd m6(7,7);
    m6 <<   1,  pow(tn,1),pow(tn,2),      pow(tn,3),      pow(tn,4),    pow(tn,5),          pow(tn,6),
            0,  1,              2*pow(tn,1),    3*pow(tn,2),    4*pow(tn,3),  5*pow(tn,4),        6*pow(tn,5),
            0,  0,              2,                    6*pow(tn,1),  12*pow(tn,2),   20*pow(tn,3),      30*pow(tn,4),
            1,  pow(tn1,1),pow(tn1,2),      pow(tn1,3),      pow(tn1,4),    pow(tn1,5),          pow(tn1,6),
            1,  pow(tn2,1),pow(tn2,2),      pow(tn2,3),      pow(tn2,4),    pow(tn2,5),          pow(tn2,6),
            0,  1,              2*pow(tn2,1),    3*pow(tn2,2),    4*pow(tn2,3),  5*pow(tn2,4),        6*pow(tn2,5),
            0,  0,              2,                    6*pow(tn2,1),  12*pow(tn2,2),   20*pow(tn2,3),      30*pow(tn2,4);
            vector = (m6.inverse() * vector);

    return vector;
}

Eigen::VectorXd MatrixMaker9(float tn, float tn1, float tn2, float con1, float con2, float con3, float con4, float con5, float con6, float con7, float con8, float con9) {
    Eigen::VectorXd vector(9);
    vector << con1, con2, con3, con4, con5, con6, con7, con8, con9;

    Eigen::MatrixXd m6(9,9);
    m6 <<   1,  pow(tn,1),pow(tn,2),      pow(tn,3),      pow(tn,4),    pow(tn,5),          pow(tn,6),      pow(tn,7),             pow(tn, 8),
            0,  1,              2*pow(tn,1),    3*pow(tn,2),    4*pow(tn,3),  5*pow(tn,4),        6*pow(tn,5),       7*pow(tn,6),        8*pow(tn, 7),
            0,  0,              2,                    6*pow(tn,1),  12*pow(tn,2),   20*pow(tn,3),      30*pow(tn,4),       42*pow(tn,5),       56*pow(tn, 6),
            1,  pow(tn1,1),pow(tn1,2),      pow(tn1,3),      pow(tn1,4),    pow(tn1,5),          pow(tn1,6),      pow(tn1,7),             pow(tn1, 8),
            0,  1,              2*pow(tn1,1),    3*pow(tn1,2),    4*pow(tn1,3),  5*pow(tn1,4),        6*pow(tn1,5),       7*pow(tn1,6),        8*pow(tn1, 7),
            0,  0,              2,                    6*pow(tn1,1),  12*pow(tn1,2),   20*pow(tn1,3),      30*pow(tn1,4),       42*pow(tn1,5),       56*pow(tn1, 6),
            1,  pow(tn2,1),pow(tn2,2),      pow(tn2,3),      pow(tn2,4),    pow(tn2,5),          pow(tn2,6),      pow(tn2,7),             pow(tn2, 8),
            0,  1,              2*pow(tn2,1),    3*pow(tn2,2),    4*pow(tn2,3),  5*pow(tn2,4),        6*pow(tn2,5),       7*pow(tn2,6),        8*pow(tn2, 7),
            0,  0,              2,                    6*pow(tn2,1),  12*pow(tn2,2),   20*pow(tn2,3),      30*pow(tn2,4),       42*pow(tn2,5),       56*pow(tn2, 6);
    vector = (m6.inverse() * vector);

    return vector;
}



//_________IKEA SOLVER_____________________________________________________________________________________________
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

