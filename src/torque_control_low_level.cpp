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
// Library PINOCCHIO
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"


namespace k_api = Kinova::Api;

#define IP_ADDRESS "192.168.1.10"

#define PORT 10000
#define PORT_REAL_TIME 10001

#define ACTUATOR_COUNT 7

float TIME_DURATION = 15.0f; // Duration of the example (seconds)

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};

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

/**************************
 * Example core functions *
 **************************/
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

    // KINOVA Feedback
    Eigen::VectorXd q(7), dq(7), torque(7);

    // Forward Kinematics
    Eigen::Matrix4d T;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;

    // Direct Dynamics
    // Set up your own URDF file PATH
    const std::string urdf_filename = std::string("YOUR_src_PATH_HERE"+"urdf/GEN3_URDF_V12.urdf");
    // Load the urdf model & Create data required by the algorithms
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename,model);
    pinocchio::Data data(model);

    // Control
    Eigen::MatrixXd J(6,7);
    Eigen::VectorXd v(6);

    double roll, pitch, yaw;
    Eigen::Matrix3d R_d;
    Eigen::Vector3d p_d;
    Eigen::Vector3d p_home(0.447, 0.003, 0.431);
    Eigen::VectorXd v_d(6);

    Eigen::Vector3d ep, eo;
    Eigen::VectorXd errorPose(6), errorVelocity(6);

    Eigen::VectorXd tau, control_signal, tau_compensation;

    // Kp & Kd
    double kp_pos, kp_ori, kd_pos, kd_ori;
    Eigen::VectorXd Kp_vec(6), Kd_vec(6);
    Eigen::MatrixXd Kp(6,6), Kd(6,6);

    kp_pos = 1500;
    kp_ori = 150;
    Kp_vec << kp_pos,kp_pos,kp_pos,kp_ori,kp_ori,kp_ori;
    Kp = Kp_vec.asDiagonal();

    kd_pos = 2*1*sqrt(kp_pos);
    kd_ori = 2*1*sqrt(kp_ori);
    Kd_vec << kd_pos,kd_pos,kd_pos,kd_ori,kd_ori,kd_ori;
    Kd = Kd_vec.asDiagonal();

    // Joint Torques Limits
    Eigen::VectorXd gen3_JointTorquesLimits(7);
    gen3_JointTorquesLimits << 40, 40, 40, 40, 20, 20, 30;

    // == VARIABLES to provide DESIRED positions:
    // -- SINUS parameters
    double A = 0.08;     //  m
    double f = 0.5;      //  Hz
    double t = 0.0;      //  time
    
    // -- Desired x,y (position) and vx,vy (velocity)
    double x_d, y_d;
    double vx_d, vy_d;

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
        //std::cout << "CONTROL MESSAGE: " << control_mode_message.control_mode() << std::endl;
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

                // KINOVA Feedback: Obtaining gen3 ACTUAL joint positions, velocities & torques
             	for (int i = 0; i < ACTUATOR_COUNT; i++)
                {
                    q[i] = (M_PI/180)*base_feedback.actuators(i).position();
                    dq[i] = (M_PI/180)*base_feedback.actuators(i).velocity();
                    torque[i] = base_feedback.actuators(i).torque();
                }

                /*
                std::cout << "------------------------------------------------------- \n"
                          << "---------------------- KINOVA FEEDBACK ---------------- \n"
                          << "q [rad]: " << q.transpose() << "\n"
                          << "dq [rad/s]: " << dq.transpose() << "\n"
                          << "torque [Nm]: " << torque.transpose() << "\n"
                          << "------------------------------------------------------- " << std::endl;
                */

                // --------------------------------------
                // ---------  Direct Dynamics  ----------
                // -- Compensation: Gravity & Coriolis --
                tau_compensation = gen3_DirectDynamics(model,data,q,dq);

                //std::cout << "Compen [Nm]: " << tau_compensation.transpose() << std::endl;

                // -------------------------------------
                // -------------- CONTROL --------------
                // -------------------------------------

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

		// DESIRED: POSE & VELOCITY
                // POSE
                // --- POSITION
		x_d = p_home[0]+0.1 + A*sin(2*M_PI*f*t);
		y_d = p_home[1] + A*cos(2*M_PI*f*t);
		// Controlling XY to make a circumference and Z to stay in its home position
		p_d << x_d, y_d, p_home[2];

		// --- ORIENTATION
		roll = 0.833*M_PI;
		pitch = 0.31*M_PI;
		yaw = 0.06*M_PI;
		R_d = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
		    * Eigen::AngleAxisd(pitch,  Eigen::Vector3d::UnitY())
		    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
		//std::cout << R_d << std::endl << "is unitary: " << R_d.isUnitary() << std::endl;

                // VELOCITY
		vx_d = A*cos(2*M_PI*f*t);
		vy_d = -A*sin(2*M_PI*f*t);
                v_d << vx_d, vy_d, 0, 0, 0, 0;

                // Updating TIME for SINUS and COSINUS computation
                t = t + 0.001;

                // TO STOP THE ROBOT 2s BEFORE: setting actuatorS back in position
		if(t > TIME_DURATION-2.0)
                {
                    p_d = p_home;
                    v_d << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

                }

                // ---------------
                // ---- ERROR ----
                // ---------------
                // ERROR POSE: POSITION + ORIENTATION
                ep = p_d - p;
                eo = 0.5*(R.col(0).cross(R_d.col(0))+R.col(1).cross(R_d.col(1))+R.col(2).cross(R_d.col(2)));
                errorPose << ep[0], ep[1], ep[2], eo[0], eo[1], eo[2];

                // ERROR VELOCITY
                errorVelocity = v_d - v;

                // CONTROL SIGNAL OBTAINED
                control_signal = J.transpose()*(Kp*errorPose + Kd*errorVelocity);

		// ===================================
		// ------ Control Signal Limits ------
		// ===================================
		for (int i = 0; i < ACTUATOR_COUNT; i++)
		{
		    if(control_signal[i]>gen3_JointTorquesLimits[i]){
			std::cout << "------------------------ Hi +!" << std::endl;
			std::cout << "Control Signal: " << control_signal.transpose() << std::endl;
				control_signal[i] = gen3_JointTorquesLimits[i];
		    }else if(control_signal[i]<-gen3_JointTorquesLimits[i]){
				std::cout << "------------------------ Hi -!" << std::endl;
				std::cout << "Control Signal: " << control_signal.transpose() << std::endl;
				control_signal[i] = -gen3_JointTorquesLimits[i];
		    }
		}

		// Controller OUTPUT with KINOVA compensation
		tau = control_signal + tau_compensation;

                //std::cout << "Tau: " << tau.transpose() << std::endl;

                // ===================================
		// ------ Joint Torque Commands ------
		// ===================================
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

    // Example core
    example_move_to_home_position(base);
    auto isOk = torque_control_low_level(base, base_cyclic, actuator_config);
    if (!isOk)
    {
        std::cout << "There has been an unexpected error in torque_control_low_level() function." << endl;;
    }

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
}
