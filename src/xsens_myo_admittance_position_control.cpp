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
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <google/protobuf/util/json_util.h>

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

//***************
//* MYO ARMBAND *
//***************
 #include <thread>         // std::thread
#include "myolinux/myoclient.h"
#include "myolinux/serial.h"
#include <cinttypes>
/***************/

//------------------//
//----- XSENS -----//
//-----------------//
#include "mvn_xsens.h"
#include "mvn_stream.h"
//------------------//

namespace k_api = Kinova::Api;

#define IP_ADDRESS "192.168.1.10"

#define PORT 10000
#define PORT_REAL_TIME 10001

#define ACTUATOR_COUNT 7
#define DURATION 30           // Network timeout (seconds)

float time_duration = DURATION; // Duration of the example (seconds)

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};

//******************************
//* Quaternion to Euler angles *
//******************************
struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};
//****************************//


// =============== MYO =======================
// For acquiring the DATA
std::vector<float> myo_emg, myo_acc, myo_gyr;
float OrientationScale =  16384.0f;
struct Quaternion myo_quat;
struct EulerAngles myo_angles;
double myo = 0.0;
double sum_EMG = 0.0;
double norm_EMG = 0.0;
double max_EMG = 300.0;


// Admittance VARIABLES
float m = 0.05;
float b = 0.3;
float k = 15;

float min_k = 3;
float max_k = 15;
float dt = 0.001;


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

double offset = 60;     // Offset between MYO and KINOVA

// ================== XSENS ==================
double goal_RightElbow;
bool goal = false;
// ===========================================


// ======== Position Control =================
double angle_min = -80.0;       // degrees
double angle_max = 80.0;        // degrees
double pos_error_max = 30.0;    // degrees
//double vel_max = 150.0;     // degrees/second
double pos_toSend = 0;

bool running = true;
// ===========================================

//******************************
//* Quaternion to Euler angles *
//******************************
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


//****************************
//* Example related function *
//****************************
int64_t GetTickUs()
{
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    return (start.QuadPart * 1000000)/frequency.QuadPart;
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

//**************************
//* Example core functions *
//**************************
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
        /*
        std::cout << "Euler Angles: "
                	          <<  myo_angles.roll << ", "
                	          <<  myo_angles.pitch << ", "
                	          <<  myo_angles.yaw << std::endl;
        */
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

    std::vector<TJointAngles> joint_angles;
    std::vector<TPointPos> point_positions;
    std::vector<TTransform> transforms;
    bool ready = false;
    TBody body;
    int id_jRightElbow;

    do{
        if(xsens_client->is_body_available()){
            std::cout << "New body data available" << std::endl;
            xsens_client->get_scale_info(body);
            //std::cout << body << std::endl;

            // POINTS - Looking For jRightElbow value
            for(int i=0;i<body.points.size();i++){
                if(body.points[i].name == "jRightElbow" && body.points[i].point_id == 2){
                    id_jRightElbow = body.points[i].segment_id;
                }
            }

           while(running){

                while(!ready)
                  {
                    if(xsens_client->is_data_available())
                      ready = true;
                    else
                      usleep(100000);
                  }
                  ready = false;

                  xsens_client->get_joint_angles(joint_angles);

                for(unsigned int i=0;i<joint_angles.size();i++){

                    //std::cout << "Joint Angles: " << joint_angles[i] << std::endl;

                    if(CMvnXsens::get_segment(joint_angles[i].parent_segment_id) == id_jRightElbow){
                        goal = true;
                        goal_RightElbow = joint_angles[i].rot_z*180/M_PI;
                    }
                }

                xsens_client->get_point_positions(point_positions);
                build_skeleton_transforms(body,point_positions,joint_angles,transforms);
            }
        }
        goal = false;

    }while(running);

    std::cout << "============================== XSENS =============================" << std::endl;
    std::cout << "xsens - End of Reading!" << std::endl;
}


bool position_and_admittance_control_low_level(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    bool return_status = true;

    k_api::BaseCyclic::Feedback base_feedback;
    const k_api::BaseCyclic::ActuatorFeedback* base_feedback_test;
    k_api::BaseCyclic::Command  base_command;

    std::vector<float> pos_commands, vel_commands;

    float torque, torque_ini, accel, velocity, position, pos_ini, pos_error;
    float pos_goal = 0.0f;

    // #JOINT to control
    int n_act = 5;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;
    double t = 0.0;

    std::cout << "Initializing the arm for velocity low-level control example" << std::endl;
    try
    {

        // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to its current position
        for(int i = 0; i < ACTUATOR_COUNT; i++)
        {
            pos_commands.push_back(base_feedback.actuators(i).position());
            vel_commands.push_back(base_feedback.actuators(i).velocity()*M_PI/180); // In rad
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());

        }

        // Initialization values
        torque_ini = base_feedback.actuators(n_act).torque();
        pos_ini = base_feedback.actuators(n_act).position();
        pos_goal = pos_ini;
        pos_toSend = pos_commands[n_act];


        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);

        // Real-time loop
        while(timer_count < (time_duration * 1000)){

            now = GetTickUs();
            if(now - last > 1000) { // t(us) > 1 ms

                // --------------- POSITION GOAL [ORI] --------------- //
                if(goal){
                    pos_goal = goal_RightElbow;
                    //std::cout << "Goal: " << goal_RightElbow << std::endl;
                }else{
                    std::cout << "NOT RECEIVING XSENS DATA, SO NOT UPDATING GOAL" << std::endl;
                }
                // --------------------------------------------------- //

                // --------------- k CALCULATION [EMG] --------------- //
                // === Stiffness acquisition from Myo Armband
				k = myo_k();

                // ---------- ADMITTANCE + POSITION CONTROL ----------- //
                position = base_feedback.actuators(n_act).position();
                torque = base_feedback.actuators(n_act).torque()-torque_ini*cos(position-pos_ini);

                // Degrees in range -180 0 180
                if(position > 180){
                    position -= 360;
                }else if(position < -180){
                    position += 360;
                }

                // From degree to rad
                pos_error = (pos_goal - position)*M_PI/180;
                position = position*M_PI/180;

                // ERROR LIMIT
                if(pos_error > pos_error_max*M_PI/180 && k > 5){
                    pos_error = pos_error_max*M_PI/180;
                }else if(pos_error < -pos_error_max*M_PI/180 && k > 5){
                    pos_error = -pos_error_max*M_PI/180;
                }

                // ----- ACCELERATION ----- //
                accel = (torque - vel_commands[n_act]*b + pos_error*k)/m;

                if((pos_commands[n_act] < angle_min && accel < 0) ||
                   (pos_commands[n_act] > angle_max && accel > 0)){
                    accel = 0;
                }

                // ----- VELOCITY ----- //
                vel_commands[n_act] = vel_commands[n_act] + accel*dt;

                // ----- POSITION ----- //
                pos_commands[n_act] = position + vel_commands[n_act]*dt;
                // ---------------------------------------------------- //

                // ---- SENDING pos_commands[n_act] TO ROBOT JOINT ---- //
                // From rad to degrees
                pos_commands[n_act] = pos_commands[n_act]*180/M_PI;

                // Degrees in range -180 0 180
                if(pos_commands[n_act] > 180){
                    pos_commands[n_act] -= 360;
                }else if(pos_commands[n_act] < -180){
                    pos_commands[n_act] += 360;
                }

                /*
                std::cout << "Accel, Vel_command, Pos_command: " << std::endl
                          << accel << " rad/s²; "
                          << vel_commands[n_act] << " rad/s; "
                          << pos_commands[n_act] << " degree(s)" << std::endl;
                */

                if(pos_commands[n_act] < angle_min){
                    std::cout << "-------------------------- HELP!" << std::endl;
                    pos_toSend = angle_min;
                    vel_commands[n_act] = 0;
                }else if(pos_commands[n_act] > angle_max){
                    std::cout << "-------------------------- HELP2!" << std::endl;
                    pos_toSend = angle_max;
                    vel_commands[n_act] = 0;
                }else{
                    pos_toSend = pos_commands[n_act];
                }

                //std::cout << "Pos_commands: " << pos_commands[n_act]  << std::endl;
                //std::cout << "Pos_toSend: " << pos_toSend << std::endl;

                base_command.mutable_actuators(n_act)->set_position(fmod(pos_toSend, 360.0f));

                // ---------------------------------------------------- //

                // ----- SETTING FRAME ID ----- //
                // Incrementing identifier ensures actuators can reject out of time frames
                base_command.set_frame_id(base_command.frame_id() + 1);
                if (base_command.frame_id() > 65535)
                    base_command.set_frame_id(0);

                for (int idx = 0; idx < ACTUATOR_COUNT; idx++)
                {
                    base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
                }
                // ---------------------------- //

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
        std::cout << "XSENS + MYO: Admittance & Position Control example completed! :D" << std::endl;

    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Kortex error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Runtime error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set back the servoing mode to Single Level Servoing
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

int main(int argc, char **argv)
{
    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };

    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(IP_ADDRESS, PORT);

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


    // *******************************
    // ** MYO ARMBAND CONFIGURATION **
    // *******************************
    auto myo_client = new myolinux::myo::Client(myolinux::Serial{"/dev/ttyACM0", 115200});

    // Autoconnect to the first Myo device
    myo_client->connect();
    if (!myo_client->connected()) {
        std::cout << "not connected..." << std::endl;
        return 1;
    }else{
        std::cout << "---------------------- MYO connected! :)" << std::endl;
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

    // *******************************
    // *** MVN XSENS CONFIGURATION ***
    // *******************************
    CMvnXsens* xsens_client;
    xsens_client = new CMvnXsens("inertial_suit");
    xsens_client->connect("192.168.1.15",9763);
    std::cout << "---------------------- XSENS connected! :D" << std::endl;
    xsens_client->process_message_type(mvn_character_scale|mvn_joint_angles|mvn_pose_data_pos);


    // ********************************
    // ************* CORE *************
    // ********************************
    // Move arm to ready position
    example_move_to_home_position(base);

    // THREADS
    std::thread t1_kinova (position_and_admittance_control_low_level,base, base_cyclic);  // spawn new thread that calls position_and_admittance_control_low_level(base, base_cyclic)
    std::thread t2_myo (myo_reading, myo_client);       // spawn new thread that calls myo_reading()
    std::thread t3_xsens (xsens_reading, xsens_client); // spawn new thread that calls xsens_reading()


    std::cout << "main, myo_reading, xsens_reading and low_level_position_control executing concurrently...\n";
    // ********************************

    // synchronize threads:
    t2_myo.join();              // pauses until t2_myo finishes

    // Close MYO CLIENT
    myo_client->disconnect();
    std::cout << "MYO client disconnected!" << std::endl;

    t1_kinova.join();           // pauses until t1_kinova finishes
    t3_xsens.join();            // pauses until t3_xsens finishes


    std::cout << "myo_reading, xsens_reading and position_and_admittance_control_low_level completed.\n";

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
