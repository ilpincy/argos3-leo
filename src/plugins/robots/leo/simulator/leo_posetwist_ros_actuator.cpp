/**
 * @file <argos3/plugins/robots/leo/simulator/leo_posetwist_ros_actuator.cpp>
 *
 * @author Steve Marotta - <smarotta@cra.com>
 */

#include "leo_posetwist_ros_actuator.h"
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {
   
   /****************************************/
   /****************************************/

   CLeoPoseTwistROSActuator::CLeoPoseTwistROSActuator() :
      m_pcCmdVelPub(nullptr) {}

   /****************************************/
   /****************************************/

   void CLeoPoseTwistROSActuator::Init(TConfigurationNode &t_tree) {
      CLeoPoseTwistDefaultActuator::Init(t_tree);
      /* Initialize ROS node */
      LOG << "[INFO] CLeoPoseTwistROSActuator: Initializing ROS..." << std::endl;
      int argc = 0;
      char** argv = nullptr;
      ros::init(argc, argv, "leo_posetwist_ros_actuator");
      if (ros::master::check()) {
         ros::NodeHandle cNH;
         std::string strROSPrefix(m_pcLeoEntity->GetId());
         if (strROSPrefix == "0") {
            strROSPrefix = "main";
         }
         /* Create publisher if ROS connection was successful */
         m_pcCmdVelPub = std::make_shared<ros::Publisher>(
            cNH.advertise<geometry_msgs::Twist>(
               "/leo_" + strROSPrefix + "/cmd_vel", 10));
         LOG << "[INFO] CLeoPoseTwistROSActuator: ROS Successfully Initialized" << std::endl;
      } else {
         LOGERR << "[INFO] Could not connect to ROS" << std::endl;
         m_pcCmdVelPub = nullptr;
      }
      LOG.Flush();
      LOGERR.Flush();
   }

   /****************************************/
   /****************************************/

   void CLeoPoseTwistROSActuator::Update() {
      CLeoPoseTwistDefaultActuator::Update();
      if (m_pcCmdVelPub != nullptr) {
         /* Create a Twist message to send to ROS */
         geometry_msgs::Twist cVelMsg;
         /* Set linear velocity */
         cVelMsg.linear.x = m_fDesiredLinearVelocity / 10.0; // Scaling this down for testing
         cVelMsg.linear.y = 0.0;
         cVelMsg.linear.z = 0.0;
         /* Set angular velocity */
         cVelMsg.angular.x = 0.0;
         cVelMsg.angular.y = 0.0;
         cVelMsg.angular.z = m_cDesiredAngularVelocity.GetValue();
         /* Publish the message */
         m_pcCmdVelPub->publish(cVelMsg);
      }

   }

   /****************************************/
   /****************************************/

}

REGISTER_ACTUATOR(CLeoPoseTwistROSActuator,
                  "leo_posetwist", "ros",
                  "Steve Marotta [smarotta@cra.com]",
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
                  "None.\n\n",
                  "Usable"
   );
