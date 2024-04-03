/**
 * @file <argos3/plugins/robots/leo/simulator/leo_posetwist_ros_actuator.h>
 *
 * @author Steve Marotta - <smarotta@cra.com>
 */

#ifndef LEO_POSETWIST_ACTUATOR_ROS_H
#define LEO_POSETWIST_ACTUATOR_ROS_H

#include <string>
#include <map>
#include <memory>

namespace argos {
   class CLeoPoseTwistROSActuator;
}

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/actuator.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_posetwist_actuator.h>
#include <argos3/plugins/robots/leo/simulator/leo_entity.h>
#include "leo_posetwist_default_actuator.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace argos {
   
   class CLeoPoseTwistROSActuator : public CLeoPoseTwistDefaultActuator {

   public:

      CLeoPoseTwistROSActuator();
      virtual ~CLeoPoseTwistROSActuator() {}
      virtual void Init(TConfigurationNode& t_tree);
      virtual void Update();

   protected:

      std::shared_ptr<ros::Publisher> m_pcCmdVelPub;

   };

}

#endif
