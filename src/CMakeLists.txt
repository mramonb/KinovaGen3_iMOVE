
# ------ locate the necessary dependencies ------
FIND_PACKAGE(mvn_xsens REQUIRED)
FIND_PACKAGE(comm REQUIRED)
FIND_PACKAGE(iriutils REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)


# ------ add the necessary include directories ------
INCLUDE_DIRECTORIES(${mvn_xsens_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${comm_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${iriutils_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# application source files

# application header files

# --------- create the executable files ---------
## KINOVA
ADD_EXECUTABLE(torque_compensation_low_level torque_compensation_low_level.cpp)
ADD_EXECUTABLE(torque_control_low_level torque_control_low_level.cpp)
ADD_EXECUTABLE(admittance_torque_control_low_level admittance_torque_control_low_level.cpp)

## MYO
ADD_EXECUTABLE(myo_test myo_test.cpp)

## XSENS
ADD_EXECUTABLE(xsens_test xsens_test.cpp)

## PINOCCHIO
ADD_EXECUTABLE(pinocchio_test pinocchio_test.cpp)

# XSENS + MYO + PINOCCHIO + KINOVA
ADD_EXECUTABLE(joint6_xsens_myo_admittance_position_control_low_level joint6_xsens_myo_admittance_position_control_low_level.cpp)
ADD_EXECUTABLE(fullarm_xsens_myo_admittance_torque_control_low_level fullarm_xsens_myo_admittance_torque_control_low_level.cpp)


# link necessary libraries
TARGET_LINK_LIBRARIES(xsens_test ${mvn_xsens_LIBRARY})
TARGET_LINK_LIBRARIES(joint6_xsens_myo_admittance_position_control ${mvn_xsens_LIBRARY})
TARGET_LINK_LIBRARIES(fullarm_xsens_myo_admittance_torque_control_low_level ${mvn_xsens_LIBRARY})

TARGET_LINK_LIBRARIES(xsens_test ${comm_LIBRARY})
TARGET_LINK_LIBRARIES(joint6_xsens_myo_admittance_position_control ${comm_LIBRARY})
TARGET_LINK_LIBRARIES(fullarm_xsens_myo_admittance_torque_control_low_level ${comm_LIBRARY})

TARGET_LINK_LIBRARIES(xsens_test ${iriutils_LIBRARY})
TARGET_LINK_LIBRARIES(joint6_xsens_myo_admittance_position_control ${iriutils_LIBRARY})
TARGET_LINK_LIBRARIES(fullarm_xsens_myo_admittance_torque_control_low_level ${iriutils_LIBRARY})


