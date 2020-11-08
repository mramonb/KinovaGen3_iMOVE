#include "mvn_xsens.h"
#include "mvn_stream.h"

#include <math.h>


int main(int argc, char *argv[])
{
  std::vector<TJointAngles> joint_angles;
  std::vector<TPointPos> point_positions;
  std::vector<TTransform> transforms;
  std::vector<TPoseEuler> euler_poses;
  std::vector<TPoseQuat> quaternion_poses;
  TCenterOfMass center_of_mass;
  std::vector<TMotionTracker> motion_trackers;
  std::vector<TAngularKinematics> angular_kinematics;
  std::vector<TLinearKinematics> linear_kinematics;
  bool done=false,ready=false;
  TBody body;

  try{
    CMvnXsens suit("inertial_suit");
    suit.connect("192.168.1.15",9763);
    suit.process_message_type(mvn_pose_data_euler|mvn_pose_data_quat|mvn_character_scale|mvn_joint_angles|mvn_pose_data_pos|mvn_linear_kinematics|mvn_angular_kinematics|mvn_motion_tracker|mvn_center_of_mass);


    do{

      if(suit.is_body_available())
      {
        std::cout << "new body data available" << std::endl;
        suit.get_scale_info(body);
        std::cout << body << std::endl;  

          // SEGMENTS
          for(int i=0;i<body.segments.size();i++){
            std::cout << "Body SEGMENTS: " << body.segments[i] << std::endl;
          }

          // POINTS
          for(int i=0;i<body.points.size();i++){
            std::cout << "Body POINTS: " << body.points[i] << std::endl;
          }


          for(unsigned int i=0;i<500;i++)
          {
            while(!ready)
            {
              if(suit.is_data_available())
                ready=true;
              else
                usleep(100000);
            }
            ready=false;
            std::cout << "new transforms ... !!!!!!!!!!!" << std::endl;
            suit.get_joint_angles(joint_angles);
            suit.get_point_positions(point_positions);

            build_skeleton_transforms(body,point_positions,joint_angles,transforms);


            for(unsigned int i=0;i<joint_angles.size();i++){
              std::cout << "Joint Angles: " << joint_angles[i] << std::endl;
            }

            suit.get_euler_poses(euler_poses);
            suit.get_quaternion_poses(quaternion_poses);
            suit.get_angular_kinematics(angular_kinematics);
            suit.get_center_of_mass(center_of_mass);
            suit.get_motion_tracker(motion_trackers);
            suit.get_linear_kinematics(linear_kinematics);


            done=true;
          }

      }
      usleep(10000);
    }while(!done);
  }catch(...){
  }
}
