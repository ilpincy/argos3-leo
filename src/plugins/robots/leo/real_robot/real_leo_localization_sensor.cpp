#include "real_leo_localization_sensor.h"
#include <argos3/core/utility/math/general.h>
#include <tf2_ros/transform_listener.h> // Include tf2 headers
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For converting to RPY
#include <geometry_msgs/Pose.h> // Include for Pose
#include <ros/timer.h> // Add this include

// #include <argos3/core/simulator/entity/embodied_entity.h> // For CEmbodiedEntity
// #include <argos3/core/simulator/space/positional_indices/positional_index.h> // For GetIndex()
#include <iostream> // Include for std::cout

/****************************************/
/****************************************/

CRealLeoLocalizationSensor::CRealLeoLocalizationSensor(ros::NodeHandle& c_node_handle, std::string str_robot_frame, std::string str_map_frame) :
   CRealLeoROSDevice(c_node_handle),
   m_tfBuffer(),  // Initialize the buffer
   m_tfListener(m_tfBuffer), // Initialize the listener with the buffer
   m_strRobotFrame(str_robot_frame), // Initialize robot frame from parameter
   m_strMapFrame(str_map_frame)      // Initialize map frame from parameter
{

    ros::Duration(0.1).sleep();

    m_updateTimer = c_node_handle.createTimer(
        ros::Duration(0.2), // Example: 10 Hz timer
        &CRealLeoLocalizationSensor::Update, 
        this);
}

/****************************************/
/****************************************/

void CRealLeoLocalizationSensor::Update(const ros::TimerEvent& event) {

    std::cout << "[CRealLeoLocalizationSensor::Update] Starting Update function (Timer Callback)" << std::endl;

    // --- Get Robot Pose from TF2 ---
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Pose cPose ;


    try {
        // Store the successful transform
        m_lastTransformStamped = transformStamped;
        m_bLastTransformValid = true; // Set flag to true as we have a valid transform

        std::cout << "[CRealLeoLocalizationSensor::Update] Before lookupTransform: map_frame=" << m_strMapFrame << ", robot_frame=" << m_strRobotFrame << std::endl;
        transformStamped = m_tfBuffer.lookupTransform(m_strMapFrame, m_strRobotFrame, ros::Time::now() - ros::Duration(0.2), ros::Duration(5.0));
        std::cout << "[CRealLeoLocalizationSensor::Update] After successful lookupTransform" << std::endl;

        m_sReading.Position.Set(
            transformStamped.transform.translation.x,
            transformStamped.transform.translation.y,
            transformStamped.transform.translation.z);

        m_sReading.Orientation.Set(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w);

    } catch (tf2::TransformException &ex) {
        ROS_WARN("[CRealLeoLocalizationSensor::Update] Transform exception: %s", ex.what()); // Use ROS_WARN for warnings

        if (m_bLastTransformValid) {
            std::cout << "[CRealLeoLocalizationSensor::Update] Using last valid transform." << std::endl;
             // Use the last valid transform
            transformStamped = m_lastTransformStamped; // Re-use the stored transform

            m_sReading.Position.Set(
                transformStamped.transform.translation.x,
                transformStamped.transform.translation.y,
                transformStamped.transform.translation.z);

            m_sReading.Orientation.Set(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

        } else {
            std::cout << "[CRealLeoLocalizationSensor::Update] No last valid transform available. Outputting zero pose." << std::endl;
            // If no last transform, output zero pose or handle as needed.
            m_sReading.Position.Set(420.0, 69.0, 10.0);
            m_sReading.Orientation.Set(0.0, 0.0, 0.0, 1.0); // Identity quaternion
        }
    }

    std::cout << "[CRealLeoLocalizationSensor::Update] Ending Update function" << std::endl;
}
