#include "leo_posetwist_ros_actuator.h"
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {
    CLeoPoseTwistRosActuator::CLeoPoseTwistRosActuator() : cmd_vel_pub(nullptr)
    {
    }

    void argos::CLeoPoseTwistRosActuator::SetRobot(CComposableEntity &c_entity)
    {
        CLeoPoseTwistDefaultActuator::SetRobot(c_entity);
    }

    void argos::CLeoPoseTwistRosActuator::Init(TConfigurationNode &t_tree)
    {
        CLeoPoseTwistDefaultActuator::Init(t_tree);

        // Initialize ROS node
        LOG << "[INFO] Initializing ROS..." << std::endl;
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "leo_posetwist_ros_actuator");
        if (ros::master::check()) {
            ros::NodeHandle nh;

            // Create publisher if ROS connection was successful
            cmd_vel_pub = std::make_shared<ros::Publisher>(nh.advertise<geometry_msgs::Twist>("cmd_vel", 10));
        } else {
            LOG << "[INFO] Could not connect to ROS" << std::endl;
            cmd_vel_pub = nullptr;
        }
    }

    void argos::CLeoPoseTwistRosActuator::SetLinearVelocity(Real f_velocity)
    {
        CLeoPoseTwistDefaultActuator::SetLinearVelocity(f_velocity);
    }

    void argos::CLeoPoseTwistRosActuator::SetAngularVelocity(const CRadians &c_velocity)
    {
        CLeoPoseTwistDefaultActuator::SetAngularVelocity(c_velocity);
    }

    void argos::CLeoPoseTwistRosActuator::Update()
    {
        CLeoPoseTwistDefaultActuator::Update();

        if (cmd_vel_pub != nullptr) {

            // Create a Twist message to send to ROS
            geometry_msgs::Twist vel_msg;

            // Set linear velocity
            vel_msg.linear.x = m_fDesiredLinearVelocity; // Scaling this down for testing
            vel_msg.linear.y = 0.0;
            vel_msg.linear.z = 0.0;

            // Set angular velocity
            vel_msg.angular.x = 0.0;
            vel_msg.angular.y = 0.0;
            vel_msg.angular.z = m_cDesiredAngularVelocity.GetValue();

            // Publish the message
            cmd_vel_pub->publish(vel_msg);
        }

    }

    void argos::CLeoPoseTwistRosActuator::Reset()
    {
        CLeoPoseTwistDefaultActuator::Reset();
    }

}

REGISTER_ACTUATOR(CLeoPoseTwistRosActuator,
                  "leo_posetwist", "ros",
                  "Carlo Pinciroli [ilpincy@gmail.com]",
                  "1.0",
                  "The leo posetwist actuator.",
                  "This actuator controls the posetwist of a leo robot. For a\n"
                  "complete description of its usage, refer to the\n"
                  "ci_leo_posetwist_actuator.h file.\n\n"

                  "REQUIRED XML CONFIGURATION\n\n"

                  "  <controllers>\n"
                  "    ...\n"
                  "    <my_controller ...>\n"
                  "      ...\n"
                  "      <actuators>\n"
                  "        ...\n"
                  "        <leo_posetwist implementation=\"ros\" />\n"
                  "        ...\n"
                  "      </actuators>\n"
                  "      ...\n"
                  "    </my_controller>\n"
                  "    ...\n"
                  "  </controllers>\n\n"

                  "OPTIONAL XML CONFIGURATION\n\n"

                  "None.\n\n"
                  ,
                  "Usable"
   );
