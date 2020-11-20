#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#include <google/protobuf/util/json_util.h>

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

// Matrices - ForwardKinematics & DirectDynamics
#include <Eigen/Dense>

//-------------------------
//--- Library PINOCCHIO ---
//-------------------------
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
/*********************/

//-------------------
//--- MYO ARMBAND ---
//-------------------
#include <thread> // std::thread
#include "myolinux/myoclient.h"
#include "myolinux/serial.h"
#include <cinttypes>
/***************/

//---------------------
//------- XSENS -------
//---------------------
#include "mvn_xsens.h"
#include "mvn_stream.h"
//------------------//

namespace k_api = Kinova::Api;

#define IP_ADDRESS "192.168.1.10"

#define PORT 10000
#define PORT_REAL_TIME 10001

#define ACTUATOR_COUNT 7

float TIME_DURATION = 30.0f; // Duration of the example (seconds)

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};

// --------------------------------
// -- Quaternion to Euler angles --
// --------------------------------
struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};
// --------------------------------


// =============== MYO =======================
// For acquiring the DATA
std::vector<float> myo_emg, myo_acc, myo_gyr;
float OrientationScale =  16384.0f;
struct Quaternion myo_quat;
struct EulerAngles myo_angles;
bool running = true;

// Admittance VARIABLES
double sum_EMG = 0.0;
double norm_EMG = 0.0;
double max_EMG = 350.0;

double k = 0.0;
double min_k = 100.0;
double max_k = 600.0;

// FILTERING DATA: Mean Square Root
double window_size = 100.0;

std::vector<double> EMG_1 (100);
std::vector<double> EMG_2 (100);
std::vector<double> EMG_3 (100);
std::vector<double> EMG_4 (100);
std::vector<double> EMG_5 (100);
std::vector<double> EMG_6 (100);
std::vector<double> EMG_7 (100);
std::vector<double> EMG_8 (100);
std::vector<double> myo_RMS_EMG (8);

double sum_EMG_1, sum_EMG_2, sum_EMG_3, sum_EMG_4, sum_EMG_5, sum_EMG_6, sum_EMG_7, sum_EMG_8;
// ===========================================

// ================== XSENS ==================
bool new_goal = false;

// ----- From 9 to 3 Features
Eigen::MatrixXf PCA_Head_Shoulders(3,9);
Eigen::VectorXf ORI_Head_Shoulders(9);
Eigen::VectorXf out_PCA_Head_Shoulders(3);

// ===========================================

// --------------------------------
// -- Quaternion to Euler angles --
// --------------------------------
EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}


/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    return (start.QuadPart * 1000000) / frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)>
    create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}


void myo_reading(myolinux::myo::Client* myo_client){

    // Read EMG and IMU
    myo_client->setMode(myolinux::myo::EmgMode::SendEmg, myolinux::myo::ImuMode::SendData, myolinux::myo::ClassifierMode::Disabled);

    // EMG signals
    myo_client->onEmg([](myolinux::myo::EmgSample sample){
        myo_emg.clear();
        for (std::size_t i = 0; i < 8; i++) {
            myo_emg.push_back(static_cast<int>(sample[i]));
        }
    });

    // IMU signals
    myo_client->onImu([](myolinux::myo::OrientationSample ori, myolinux::myo::AccelerometerSample acc, myolinux::myo::GyroscopeSample gyr){

        myo_quat.x = ori[0]/OrientationScale;
        myo_quat.y = ori[1]/OrientationScale;
        myo_quat.z = ori[2]/OrientationScale;
        myo_quat.w = ori[3]/OrientationScale;
        myo_angles = ToEulerAngles(myo_quat);
        myo_angles.roll = myo_angles.roll*180/M_PI;
        myo_angles.pitch = myo_angles.pitch*180/M_PI;
        myo_angles.yaw = myo_angles.yaw*180/M_PI;

        myo_acc.clear();
        myo_acc.push_back(acc[0]);
        myo_acc.push_back(acc[1]);
        myo_acc.push_back(acc[2]);

        myo_gyr.clear();
        myo_gyr.push_back(gyr[0]);
        myo_gyr.push_back(gyr[1]);
        myo_gyr.push_back(gyr[2]);

    });
    // *******************************

    while(running){
        // MYO ARMBAND receiving data
        myo_client->listen();
    }
    std::cout << "=============================== MYO ===============================" << std::endl;
    std::cout << "myo - End of Reading!" << std::endl;
}


double myo_k(){

	// Root Mean Square
	for(int i = 0; i<window_size-1; i++){
		EMG_1[i] = EMG_1[i+1];
		EMG_2[i] = EMG_2[i+1];
		EMG_3[i] = EMG_3[i+1];
		EMG_4[i] = EMG_4[i+1];
		EMG_5[i] = EMG_5[i+1];
		EMG_6[i] = EMG_6[i+1];
		EMG_7[i] = EMG_7[i+1];
		EMG_8[i] = EMG_8[i+1];

	}

	EMG_1[window_size-1] = myo_emg[0];
	EMG_2[window_size-1] = myo_emg[1];
	EMG_3[window_size-1] = myo_emg[2];
	EMG_4[window_size-1] = myo_emg[3];
	EMG_5[window_size-1] = myo_emg[4];
	EMG_6[window_size-1] = myo_emg[5];
	EMG_7[window_size-1] = myo_emg[6];
	EMG_8[window_size-1] = myo_emg[7];

	sum_EMG_1 = 0.0;
	sum_EMG_2 = 0.0;
	sum_EMG_3 = 0.0;
	sum_EMG_4 = 0.0;
	sum_EMG_5 = 0.0;
	sum_EMG_6 = 0.0;
	sum_EMG_7 = 0.0;
	sum_EMG_8 = 0.0;

	for(int i = 0; i<window_size; i++){
		sum_EMG_1 += EMG_1[i]*EMG_1[i];
		sum_EMG_2 += EMG_2[i]*EMG_2[i];
		sum_EMG_3 += EMG_3[i]*EMG_3[i];
		sum_EMG_4 += EMG_4[i]*EMG_4[i];
		sum_EMG_5 += EMG_5[i]*EMG_5[i];
		sum_EMG_6 += EMG_6[i]*EMG_6[i];
		sum_EMG_7 += EMG_7[i]*EMG_7[i];
		sum_EMG_8 += EMG_8[i]*EMG_8[i];
	}

	myo_RMS_EMG[0] = sqrt((1/window_size)*sum_EMG_1);
	myo_RMS_EMG[1] = sqrt((1/window_size)*sum_EMG_2);
	myo_RMS_EMG[2] = sqrt((1/window_size)*sum_EMG_3);
	myo_RMS_EMG[3] = sqrt((1/window_size)*sum_EMG_4);
	myo_RMS_EMG[4] = sqrt((1/window_size)*sum_EMG_5);
	myo_RMS_EMG[5] = sqrt((1/window_size)*sum_EMG_6);
	myo_RMS_EMG[6] = sqrt((1/window_size)*sum_EMG_7);
	myo_RMS_EMG[7] = sqrt((1/window_size)*sum_EMG_8);

	sum_EMG = 0.0;
	for(int i = 0; i<8; i++){
		sum_EMG += myo_RMS_EMG[i];
	}

	norm_EMG = (sum_EMG-10)/max_EMG;

	k = (1 - norm_EMG)*min_k + norm_EMG*max_k;

	return k;

}

void xsens_reading(CMvnXsens* xsens_client){

    // ===========================
    // ------ PCA VARIABLES ------
    // ===========================
    double Head_roll = 0.0, Head_pitch = 0.0, Head_yaw = 0.0;
    double RightShoulder_roll = 0.0, RightShoulder_pitch = 0.0, RightShoulder_yaw = 0.0;
    double LeftShoulder_roll = 0.0, LeftShoulder_pitch = 0.0, LeftShoulder_yaw = 0.0;

    Eigen::VectorXf ORI_Head_Shoulders_degrees(9);
    Eigen::VectorXf H_RS_LS_mean(9), H_RS_LS_bias(3);

    H_RS_LS_mean << 3.4739, 14.9328, -24.5692, 13.1705, 3.3749, -29.8030, -4.8520, 4.7387, -27.7398;
    H_RS_LS_bias << -10, 1, 35;

    // ===========================

    // ===================================
    // ------ XSENS COLLECTING DATA ------
    // ===================================
    std::vector<TPoseEuler> euler_poses;
    bool ready = false;
    TBody body;

    int id_Head, id_RightShoulder, id_LeftShoulder;

    do{
	    if(xsens_client->is_body_available())
        {
            std::cout << "new body data available" << std::endl;
            xsens_client->get_scale_info(body);
            //std::cout << body << std::endl;

            // SEGMENTS To FIND: Head, RightShouolder & LeftShoulder
            for(int i=0;i<body.segments.size();i++){
                if(body.segments[i].name == "RightShoulder"){
                    id_RightShoulder = i;
                }else if(body.segments[i].name == "LeftShoulder"){
                    id_LeftShoulder = i;
                }else if(body.segments[i].name == "Head"){
                    id_Head = i;
                }
            }

		    while(running){
		    	// CHECK: xsens is receiving DATA
		        while(!ready){
			        if(xsens_client->is_data_available())
			          ready = true;
			        else
			          usleep(100000);
		        }
		        ready = false;

		        xsens_client->get_euler_poses(euler_poses);

		        for(unsigned int i=0;i<euler_poses.size();i++){
                    // === HEAD
                  	Head_roll = euler_poses[id_Head].rot_z;
                  	Head_pitch = euler_poses[id_Head].rot_x;
                  	Head_yaw = euler_poses[id_Head].rot_y;

                    // === RIGHT
                  	RightShoulder_roll = euler_poses[id_RightShoulder].rot_z;
                  	RightShoulder_pitch = euler_poses[id_RightShoulder].rot_x;
                  	RightShoulder_yaw = euler_poses[id_RightShoulder].rot_y;

                  	// === LEFT
                    LeftShoulder_roll = euler_poses[id_LeftShoulder].rot_z;
                  	LeftShoulder_pitch = euler_poses[id_LeftShoulder].rot_x;
                  	LeftShoulder_yaw = euler_poses[id_LeftShoulder].rot_y;

                  	/*
                  	std::cout << "Head: \n"
                  			  << "  roll  " << Head_roll << "\n"
                  			  << "  pitch " << Head_pitch << "\n"
                  			  << "  yaw   " << Head_yaw << "\n"  << std::endl;
                  	std::cout << "Right Shoulder: \n"
                  			  << "  roll  " << RightShoulder_roll << "\n"
                  			  << "  pitch " << RightShoulder_pitch << "\n"
                  			  << "  yaw   " << RightShoulder_yaw << "\n"  << std::endl;

                   	std::cout << "Left Shoulder: \n"
                   			  << "  roll  " << LeftShoulder_roll << "\n"
                   			  << "  pitch " << LeftShoulder_pitch << "\n"
                   			  << "  yaw   " << LeftShoulder_yaw << "\n"  << std::endl;
                   	*/

                  }

                  // ------ PCA - Head & Shoulders ORIENTATION ------
                  ORI_Head_Shoulders_degrees <<
                   Head_roll, Head_pitch, Head_yaw,
                   RightShoulder_roll, RightShoulder_pitch, RightShoulder_yaw,
                   LeftShoulder_roll, LeftShoulder_pitch, LeftShoulder_yaw;

                  ORI_Head_Shoulders =  ORI_Head_Shoulders_degrees - H_RS_LS_mean;
                  //std::cout << "H+S: ORI(9D): " << ORI_Head_Shoulders.transpose() << std::endl;

                  out_PCA_Head_Shoulders = PCA_Head_Shoulders*ORI_Head_Shoulders + H_RS_LS_bias;
                  std::cout << "H+S: PCA_out(3D): " << out_PCA_Head_Shoulders.transpose() << std::endl;
                  new_goal = true;
                  // ------------------------------------------------

			}
		}
    	new_goal = false;
	}while(running);
    std::cout << "============================== XSENS =============================" << std::endl;
    std::cout << "xsens - End of Reading!" << std::endl;
}

Eigen::Matrix4d gen3_ForwardKinematics(Eigen::VectorXd q_fk)
{
    Eigen::Matrix4d T_B1,T_12,T_23,T_34,T_45,T_56,T_67,T_7Tool,T_fk_BTool;

    T_B1 <<  cos(q_fk[0]), -sin(q_fk[0]),   0.0,      0.0,
            -sin(q_fk[0]), -cos(q_fk[0]),   0.0,      0.0,
                      0.0,           0.0,  -1.0,   0.1564,
                      0.0,           0.0,   0.0,      1.0;

    T_12 <<  cos(q_fk[1]), -sin(q_fk[1]),   0.0,      0.0,
                      0.0,           0.0,  -1.0,   0.0054,
             sin(q_fk[1]),  cos(q_fk[1]),   0.0,  -0.1284,
                      0.0,           0.0,   0.0,      1.0;

    T_23 <<  cos(q_fk[2]), -sin(q_fk[2]),   0.0,      0.0,
                      0.0,           0.0,   1.0,  -0.2104,
            -sin(q_fk[2]), -cos(q_fk[2]),   0.0,  -0.0064,
                      0.0,           0.0,   0.0,      1.0;

    T_34 <<  cos(q_fk[3]), -sin(q_fk[3]),   0.0,      0.0,
                      0.0,           0.0,  -1.0,   0.0064,
             sin(q_fk[3]),  cos(q_fk[3]),   0.0,  -0.2104,
                      0.0,           0.0,   0.0,      1.0;

    T_45 <<  cos(q_fk[4]), -sin(q_fk[4]),   0.0,      0.0,
                      0.0,           0.0,   1.0,  -0.2084,
            -sin(q_fk[4]), -cos(q_fk[4]),   0.0,  -0.0064,
                      0.0,           0.0,   0.0,      1.0;

    T_56 <<  cos(q_fk[5]),  -sin(q_fk[5]),  0.0,      0.0,
                      0.0,            0.0, -1.0,      0.0,
             sin(q_fk[5]),   cos(q_fk[5]),  0.0,  -0.1059,
                      0.0,            0.0,  0.0,      1.0;

    T_67 <<  cos(q_fk[6]),  -sin(q_fk[6]),  0.0,      0.0,
                      0.0,            0.0,  1.0,  -0.1059,
            -sin(q_fk[6]),  -cos(q_fk[6]),  0.0,      0.0,
                      0.0,            0.0,  0.0,      1.0;

    T_7Tool <<        1.0,            0.0,  0.0,      0.0,
                      0.0,           -1.0,  0.0,      0.0,
                      0.0,            0.0, -1.0,  -0.0595,
                      0.0,            0.0,  0.0,      1.0;

    T_fk_BTool = T_B1*T_12*T_23*T_34*T_45*T_56*T_67*T_7Tool;

    return T_fk_BTool;

}

Eigen::MatrixXd gen3_GeometricJacobian(Eigen::VectorXd q_fk)
{
    Eigen::Matrix4d T_B1,T_12,T_23,T_34,T_45,T_56,T_67,T_7Tool;

    Eigen::Vector3d z0, z1, z2, z3, z4, z5, z6, z7;
    Eigen::Vector4d p0, p1, p2, p3, p4, p5, p6, p7, pE;
    Eigen::Vector3d z1_p1, z2_p2, z3_p3, z4_p4, z5_p5, z6_p6, z7_p7;

    Eigen::MatrixXd J_Geo_pos_ori(6,7);

    // Forward Kinematics matrices

    T_B1 <<  cos(q_fk[0]), -sin(q_fk[0]),   0.0,      0.0,
            -sin(q_fk[0]), -cos(q_fk[0]),   0.0,      0.0,
                      0.0,           0.0,  -1.0,   0.1564,
                      0.0,           0.0,   0.0,      1.0;

    T_12 <<  cos(q_fk[1]), -sin(q_fk[1]),   0.0,      0.0,
                      0.0,           0.0,  -1.0,   0.0054,
             sin(q_fk[1]),  cos(q_fk[1]),   0.0,  -0.1284,
                      0.0,           0.0,   0.0,      1.0;

    T_23 <<  cos(q_fk[2]), -sin(q_fk[2]),   0.0,      0.0,
                      0.0,           0.0,   1.0,  -0.2104,
            -sin(q_fk[2]), -cos(q_fk[2]),   0.0,  -0.0064,
                      0.0,           0.0,   0.0,      1.0;

    T_34 <<  cos(q_fk[3]), -sin(q_fk[3]),   0.0,      0.0,
                      0.0,           0.0,  -1.0,   0.0064,
             sin(q_fk[3]),  cos(q_fk[3]),   0.0,  -0.2104,
                      0.0,           0.0,   0.0,      1.0;

    T_45 <<  cos(q_fk[4]), -sin(q_fk[4]),   0.0,      0.0,
                      0.0,           0.0,   1.0,  -0.2084,
            -sin(q_fk[4]), -cos(q_fk[4]),   0.0,  -0.0064,
                      0.0,           0.0,   0.0,      1.0;

    T_56 <<  cos(q_fk[5]),  -sin(q_fk[5]),  0.0,      0.0,
                      0.0,            0.0, -1.0,      0.0,
             sin(q_fk[5]),   cos(q_fk[5]),  0.0,  -0.1059,
                      0.0,            0.0,  0.0,      1.0;

    T_67 <<  cos(q_fk[6]),  -sin(q_fk[6]),  0.0,      0.0,
                      0.0,            0.0,  1.0,  -0.1059,
            -sin(q_fk[6]),  -cos(q_fk[6]),  0.0,      0.0,
                      0.0,            0.0,  0.0,      1.0;

    T_7Tool <<        1.0,            0.0,  0.0,      0.0,
                      0.0,           -1.0,  0.0,      0.0,
                      0.0,            0.0, -1.0,  -0.0595,
                      0.0,            0.0,  0.0,      1.0;

    // Geometric Jacobian Calculus
    z0 << 0, 0, 1;
    z1 = T_B1.block<3,3>(0,0)*z0;
    z2 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*z0;
    z3 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*z0;
    z4 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*T_34.block<3,3>(0,0)*z0;
    z5 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*T_34.block<3,3>(0,0)*T_45.block<3,3>(0,0)*z0;
    z6 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*T_34.block<3,3>(0,0)*T_45.block<3,3>(0,0)*T_56.block<3,3>(0,0)*z0;
    z7 = T_B1.block<3,3>(0,0)*T_12.block<3,3>(0,0)*T_23.block<3,3>(0,0)*T_34.block<3,3>(0,0)*T_45.block<3,3>(0,0)*T_56.block<3,3>(0,0)*T_67.block<3,3>(0,0)*z0;

    p0 << 0, 0, 0, 1;
    p1 = T_B1*p0;
    p2 = T_B1*T_12*p0;
    p3 = T_B1*T_12*T_23*p0;
    p4 = T_B1*T_12*T_23*T_34*p0;
    p5 = T_B1*T_12*T_23*T_34*T_45*p0;
    p6 = T_B1*T_12*T_23*T_34*T_45*T_56*p0;
    p7 = T_B1*T_12*T_23*T_34*T_45*T_56*T_67*p0;
    pE = T_B1*T_12*T_23*T_34*T_45*T_56*T_67*T_7Tool*p0;

    z1_p1 = z1.cross(pE.head<3>()-p1.head<3>());
    z2_p2 = z2.cross(pE.head<3>()-p2.head<3>());
    z3_p3 = z3.cross(pE.head<3>()-p3.head<3>());
    z4_p4 = z4.cross(pE.head<3>()-p4.head<3>());
    z5_p5 = z5.cross(pE.head<3>()-p5.head<3>());
    z6_p6 = z6.cross(pE.head<3>()-p6.head<3>());
    z7_p7 = z7.cross(pE.head<3>()-p7.head<3>());

    J_Geo_pos_ori << z1_p1, z2_p2, z3_p3, z4_p4, z5_p5, z6_p6, z7_p7,
                        z1,    z2,    z3,    z4,    z5,    z6,    z7;

    return J_Geo_pos_ori;

}

Eigen::MatrixXd JacobianTranspose_PseudoInverse(Eigen::MatrixXd Jacobian)
{
    // Compute SVD of the jacobian using Eigen functions
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jacobian.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::VectorXd singular_inv(svd.singularValues());

    for (int j = 0; j < singular_inv.size(); ++j){
        if (singular_inv[j] < 1e-8){
            singular_inv[j] = 0.0;
        }else{
            singular_inv[j] = 1.0/singular_inv[j];
        }
    }

    Eigen::MatrixXd J_inv(6,7);
    J_inv = svd.matrixV() * singular_inv.matrix().asDiagonal() * svd.matrixU().adjoint();

	//std::cout << "Jacobian: \n" << Jacobian << std::endl;
    //std::cout << "Jacobian-Transpose Inverse: \n" << J_inv << std::endl;

    return J_inv;
}

Eigen::VectorXd gen3_DirectDynamics(pinocchio::Model model, pinocchio::Data data, Eigen::VectorXd q_dD,Eigen::VectorXd dq_dD)
{

    Eigen::VectorXd q_pinocchio(11), tau_gravity_coriolis(7);

    // For PINOCCHIO: Continuous joints [cos(theta),sin(theta)] joints: {0,2,4,6}
    q_pinocchio << cos(q_dD[0]),sin(q_dD[0]),q_dD[1],cos(q_dD[2]),sin(q_dD[2]),q_dD[3],cos(q_dD[4]),sin(q_dD[4]),q_dD[5],cos(q_dD[6]),sin(q_dD[6]);

    // Gravity Compensated Torques
    pinocchio::computeGeneralizedGravity(model,data,q_pinocchio);
    Eigen::VectorXd & gravity = data.g;

    // Coriolis Matrix
    computeCoriolisMatrix(model,data,q_pinocchio,dq_dD);

    tau_gravity_coriolis = gravity + data.C*dq_dD;
    //tau_gravity_coriolis = gravity*1.05 + data.C*dq_dD;

    return tau_gravity_coriolis;

}

// =========================
//- Example core functions -
// =========================
void example_move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list())
    {
        if (action.name() == "Home")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
    }
    else
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
        }
        const auto promise_event = finish_future.get();
    }
}

bool torque_control_low_level(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config)
{
    bool return_status = true;

    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch(...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }


    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;

    std::vector<float> commands;

    auto servoing_mode = k_api::Base::ServoingModeInformation();

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    // ===== KINOVA Feedback
    Eigen::VectorXd q(7), dq(7), torque(7);

    // ===== Forward Kinematics
    Eigen::Matrix4d T;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;

    // ===== Direct Dynamics
    // Set up your own URDF file PATH
    // --- URDF file LAPTOP pwd
    const std::string urdf_filename = std::string("YOUR_src_PATH_HERE"+"urdf/GEN3_URDF_V12.urdf");

    // ===== Load the urdf model & Create data required by the algorithms
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename,model);
    pinocchio::Data data(model);

    // ===== Control
    Eigen::MatrixXd J(6,7), J_inv(6,7);
    Eigen::VectorXd v(6);

    double roll, pitch, yaw;
    Eigen::Matrix3d R_d;
    Eigen::Vector3d p_d;
    Eigen::Vector3d p_home(0.434, 0.001, 0.434);
    Eigen::VectorXd v_d(6);

    Eigen::Vector3d ep, eo;
    Eigen::VectorXd errorPose(6), errorVelocity(6);

    Eigen::VectorXd tau, control_signal, tau_compensation;

    // -- Joint Torques Limits
    Eigen::VectorXd gen3_JointTorquesLimits(7);
    gen3_JointTorquesLimits << 33, 33, 25, 25, 20, 20, 20;

    // -- Kp & Kd
    double kp_pos, kp_ori, kd_pos, kd_ori;
    Eigen::VectorXd Kp_vec(6), Kd_vec(6);
    Eigen::MatrixXd Kp(6,6), Kd(6,6);

    kp_pos = 1500;
    kp_ori = 150;
    Kp_vec << kp_pos,kp_pos,kp_pos,kp_ori,kp_ori,kp_ori;
    Kp = Kp_vec.asDiagonal();

    kd_pos = 2*sqrt(kp_pos);
    kd_ori = 2*sqrt(kp_ori);
    Kd_vec << kd_pos,kd_pos,kd_pos,kd_ori,kd_ori,kd_ori;
    Kd = Kd_vec.asDiagonal();

	// ===== Admittance
	// -- Wrench (force[x,y,z] + torque[x,y,z])
	Eigen::VectorXd wrench(6), wrench_force(3);
	Eigen::Vector3d accel_d_admittance, vel_d_admittance, pos_d_admittance;

	Eigen::Vector3d m_vec(2, 2, 2), b_vec(20, 20, 20), k_vec(200, 200, 200);
	Eigen::MatrixXd M(3,3), B(3,3), K(3,3);

    M = m_vec.asDiagonal();
    B = b_vec.asDiagonal();
    K = k_vec.asDiagonal();

	// ===== XSENS
	double dt = 0.001;
	Eigen::Vector3d xsens_xyz = p_home;
    // LIMIT x,y,z:  x [0.4, 0.7], y [-0.4, 0.4], z [0.1, 0.6]
	Eigen::Vector3d min_position_limits(0.4, -0.4, 0.1), max_position_limits(0.7, 0.4, 0.6);

    std::cout << "Initializing the arm for torque control ^^!" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());

            // Save the current actuator position, to avoid a following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);

        // Set actuatorS in torque mode now that the command is equal to measure
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
        for (int id = 1; id < ACTUATOR_COUNT+1; id++)
        {
            actuator_config->SetControlMode(control_mode_message, id);
        }

        std::cout << "Running torque control ^^ for " << TIME_DURATION << " seconds" << std::endl;

        // Real-time loop
        while (timer_count < (TIME_DURATION * 1000))
        {
            now = GetTickUs();
            if (now - last > 1000)
            {
                // =====================================
                // ---------- KINOVA Feedback ----------
                // =====================================

                // Obtaining gen3 ACTUAL joint positions, velocities & torques
             	for (int i = 0; i < ACTUATOR_COUNT; i++)
                {
                    q[i] = (M_PI/180)*base_feedback.actuators(i).position();
                    dq[i] = (M_PI/180)*base_feedback.actuators(i).velocity();
                    torque[i] = base_feedback.actuators(i).torque();
		        }
                /*
                std::cout << "-------------------------------------------------------- \n"
                          << "---------------------- KINOVA FEEDBACK ----------------- \n"
                          << "q [rad]: " << q.transpose() << "\n"
                          << "dq [rad/s]: " << dq.transpose() << "\n"
                          << "torque [Nm]: " << torque.transpose() << "\n"
                          << "-------------------------------------------------------- " << std::endl;
				*/

                // ======================================
                // ---------  Direct Dynamics  ----------
                // == Compensation: Gravity & Coriolis ==
                tau_compensation = gen3_DirectDynamics(model,data,q,dq);
                //std::cout << "Compen [Nm]: " << tau_compensation.transpose() << std::endl;

                // =====================================
                // -------------- CONTROL --------------
                // =====================================

                // ACTUAL: POSE & VELOCITY
                // POSE
                // --- Forward Kinematics ---
                T = gen3_ForwardKinematics(q);
                R << T.block<3,3>(0,0);
                p << T.block<3,1>(0,3);

                // VELOCITY
                // --- Geometric Jacobian ---
                J = gen3_GeometricJacobian(q);
                v = J*dq;


                // UPDATING GOAL IF XSENS DATA
                if(new_goal){

                	if(abs(out_PCA_Head_Shoulders[0]) > 10){
                		xsens_xyz[0] = xsens_xyz[0] + out_PCA_Head_Shoulders[0]*dt/3;
                	}

                	if(abs(out_PCA_Head_Shoulders[1]) > 10){
                		xsens_xyz[1] = xsens_xyz[1] + out_PCA_Head_Shoulders[1]*dt/3;
                	}

                    if(abs(out_PCA_Head_Shoulders[2]) > 5){
                    	xsens_xyz[2] = xsens_xyz[2] + out_PCA_Head_Shoulders[2]*dt;
                    }


					// LIMITING the obtained positions: x, y and z
                	// up LIMIT
                	for (int i = 0; i<max_position_limits.size(); i++){
                		if(xsens_xyz[i] > max_position_limits[i]){
                			xsens_xyz[i] = max_position_limits[i];
                		}
                	}

                	// down LIMIT
                	for (int i = 0; i<min_position_limits.size(); i++){
                		if(xsens_xyz[i] < min_position_limits[i]){
                			xsens_xyz[i] = min_position_limits[i];
                		}
                	}

                	std::cout << "x = "   << xsens_xyz[0]
                			  << ", y = " << xsens_xyz[1]
                			  << ", z = " << xsens_xyz[2] << std::endl;

                	new_goal = false;

                }else{

                	std::cout << "NOT RECEIVING XSENS DATA, SO NOT UPDATING GOAL" << std::endl;
                }


				// ====================================
				// ------------ ADMITTANCE ------------
				// ====================================

				// === Obtaining KINOVA EndEffector Wrench:
				// From joint torques using a pseudo inverse of Jacobian-Transpose
				J_inv = JacobianTranspose_PseudoInverse(J);
				wrench = J_inv * (torque + tau_compensation);
				wrench_force  << wrench[0], wrench[1], wrench[2];
				//std::cout << "Wrench: " << wrench.transpose() << std::endl;

				// === Stiffness acquisition from Myo Armband
				k = myo_k();
				// Represent the stiffness acquired as a MATRIX: K
				k_vec << k, k, k;
				K = k_vec.asDiagonal();


				// === Initial Conditions: accel & vel = 0, pos = actual position
				if (timer_count == 0){
					accel_d_admittance << 0, 0, 0;
					vel_d_admittance << 0, 0, 0;
					pos_d_admittance = p;
				}

				// ACCELERATION
				accel_d_admittance = M.inverse()*(wrench_force - B*vel_d_admittance - K*(pos_d_admittance-xsens_xyz));

				// VELOCITY
				vel_d_admittance = vel_d_admittance + accel_d_admittance*dt;

				// POSITION
                pos_d_admittance = pos_d_admittance + vel_d_admittance*dt;

				/*
				std::cout << "------------------------------------------------------- \n"
					  << "------------------------ ADMITTANCE ----------------------- \n"
					  << "accel [m/s²]: " << accel_d_admittance.transpose() << "\n"
					  << "vel [m/s]: " << vel_d_admittance.transpose() << "\n"
					  << "pos [m]: " << pos_d_admittance.transpose() << "\n"
					  << "----------------------------------------------------------- " << std::endl;
                */

                // DESIRED: POSE & VELOCITY
				// POSE
				// --- POSITION
				p_d = pos_d_admittance;

				// --- ORIENTATION
				R_d = R;

                // VELOCITY
				v_d << vel_d_admittance[0], vel_d_admittance[1], vel_d_admittance[2], 0, 0, 0;

                // ===============================
                // ------------ ERROR ------------
                // ===============================

                // ERROR POSE: POSITION + ORIENTATION
                ep = p_d - p;
                // MES ENDAVANT: ep = pos_d_admittance - p;
                eo = 0.5*(R.col(0).cross(R_d.col(0))+R.col(1).cross(R_d.col(1))+R.col(2).cross(R_d.col(2)));
                errorPose << ep[0], ep[1], ep[2], eo[0], eo[1], eo[2];

                // ERROR VELOCITY
                errorVelocity = v_d - v;

                // CONTROL SIGNAL OBTAINED
                control_signal = J.transpose()*(Kp*errorPose + Kd*errorVelocity);
                //std::cout << "Control Signal: " << control_signal.transpose() << std::endl;

				// ===================================
		        // ------ Control Signal Limits ------
		        // ===================================
				for (int i = 0; i < ACTUATOR_COUNT; i++)
				{
					if(control_signal[i]>gen3_JointTorquesLimits[i]){
						std::cout << "------------------------ Hi +!" << std::endl;
						control_signal[i] = gen3_JointTorquesLimits[i];
					}else if(control_signal[i]<-gen3_JointTorquesLimits[i]){
						std::cout << "------------------------ Hi -!" << std::endl;
						control_signal[i] = -gen3_JointTorquesLimits[i];
					}

				}

				// Controller OUTPUT with KINOVA compensation
				tau = control_signal + tau_compensation;

                //std::cout << "Tau: " << tau.transpose() << std::endl;

                // ===================================
		        // ------ Joint Torque Commands ------
		        // ===================================
				// Sending TAU values
                for (int i = 0; i < ACTUATOR_COUNT; i++)
                {
                    // -- Position
                    base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
                    // -- Torque
                    base_command.mutable_actuators(i)->set_torque_joint(tau[i]);
                }

                // Incrementing identifier ensures actuators can reject out of time frames
                base_command.set_frame_id(base_command.frame_id() + 1);
                if (base_command.frame_id() > 65535)
                    base_command.set_frame_id(0);

                for (int idx = 0; idx < ACTUATOR_COUNT; idx++)
                {
                    base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
                }

                try
                {
                    base_feedback = base_cyclic->Refresh(base_command, 0);

                }
                catch (k_api::KDetailedException& ex)
                {
                    std::cout << "Kortex exception: " << ex.what() << std::endl;

                    std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
                }
                catch (std::runtime_error& ex2)
                {
                    std::cout << "runtime error: " << ex2.what() << std::endl;
                }
                catch(...)
                {
                    std::cout << "Unknown error." << std::endl;
                }

                timer_count++;
                last = GetTickUs();
            }
        }

        running = false;

        std::cout << "Torque control ^^ completed" << std::endl;

        // Set actuatorS back in position
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        for (int id = 1; id < ACTUATOR_COUNT+1; id++)
        {
            actuator_config->SetControlMode(control_mode_message, id);
        }

        std::cout << "Torque control ^^ clean exit" << std::endl;

    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

int main(int argc, char **argv)
{

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };

	std::cout << "============================== KINOVA ==============================" << std::endl;
    std::cout << "Creating transport objects" << std::endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(IP_ADDRESS, PORT);

    std::cout << "Creating transport real time objects" << std::endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(IP_ADDRESS, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

    // ===============================
    // -- MYO ARMBAND CONFIGURATION --
    // ===============================
    // Configure SERIAL
    auto myo_client = new myolinux::myo::Client(myolinux::Serial{"/dev/ttyACM0", 115200});

    // Autoconnect to the first Myo device
    myo_client->connect();
    std::cout << "=============================== MYO ===============================" << std::endl;
    if (!myo_client->connected()) {
        std::cout << "not connected..." << std::endl;
        return 1;
    }else{
        std::cout << "connected..." << std::endl;
    }

    // Print device address
    std::cout << "MYO Adress: " << std::endl;
    myolinux::print_address(myo_client->address());

    // Read firmware version
    auto version = myo_client->firmwareVersion();
    std::cout << "MYO firmwareVersion: "
        << version.major << "."
        << version.minor << "."
        << version.patch << "."
        << version.hardware_rev << std::endl;

    // Vibrate
    myo_client->vibrate(myolinux::myo::Vibration::Medium);

    // Read name
    auto name = myo_client->deviceName();
    std::cout << "MYO deviceName: " << name << std::endl;

    // Set sleep mode (otherwise the device auto disconnects after a while)
    myo_client->setSleepMode(myolinux::myo::SleepMode::NeverSleep);

    // ===============================
    // ----- XSENS CONFIGURATION -----
    // ===============================
    CMvnXsens* xsens_client;
    xsens_client = new CMvnXsens("inertial_suit");
    xsens_client->connect("192.168.1.15",9763);
    std::cout << "============================== XSENS =============================" << std::endl;
    std::cout << "connected! :D" << std::endl;
    xsens_client->process_message_type(mvn_character_scale|mvn_pose_data_euler|mvn_pose_data_pos);

    // PCA - Head & Shoulders ORIENTATION
    // ----- From 9 to 3 Features
    PCA_Head_Shoulders <<
     0.281367, -0.950163,  0.012164, -0.023132, -0.088831, -0.059910, -0.024777, -0.049729,  0.052666,
     0.953276,  0.270681,  0.000661,  0.046338,  0.013967,  0.073624,  0.058201,  0.080021,  0.021032,
    -0.014491, -0.077182, -0.884052,  0.163077,  0.244108,  0.159708,  0.025017,  0.202624, -0.242722;
    std::cout << "MATRIX PCA_Head_Shoulders: \n" << PCA_Head_Shoulders << std::endl;

    // ===============================
    // ------------ CORE -------------
    // ===============================
    // Move arm to HOME position
    example_move_to_home_position(base);

    // THREADS
        // spawn new thread that calls myo_reading(myo_client)
    std::thread th1_myo (myo_reading, myo_client);
        // spawn new thread that calls torque_control_low_level(base, base_cyclic,actuator_config)
    std::thread th2_kinova (torque_control_low_level, base, base_cyclic, actuator_config);
       // spawn new thread that calls xsens_reading()
    std::thread th3_xsens (xsens_reading, xsens_client);

    std::cout << "myo_reading, xsens_reading and torque_control_low_level executed concurrently...\n";
    // ********************************

    // synchronize threads:
    th1_myo.join();                // pauses until first finishes

    // Close MYO CLIENT
    myo_client->disconnect();
    std::cout << "MYO client disconnected!" << std::endl;

    th2_kinova.join();               // pauses until second finishes
    th3_xsens.join();               // pauses until second finishes

    std::cout << "myo_reading, xsens_reading and torque_control_low_level completed.\n";


    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;

    // Destroy MYO CLIENT
    delete myo_client;

    // Destroy XSENS CLIENT
    delete xsens_client;
}
