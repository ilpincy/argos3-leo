/* Include the controller definition */
#include "leo_test_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_navigation_actuator.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_localization_sensor.h>

CLeoTestController::CLeoTestController() {}

void CLeoTestController::Init(TConfigurationNode& t_node) {

   leoPoseTwistActuator = GetActuator<CCI_LeoPoseTwistActuator>("leo_posetwist");
   leoOdometrySensor = GetSensor<CCI_LeoOdometrySensor>("leo_odometry");
   leoArTagSensor = GetSensor<CCI_LeoArTagSensor>("leo_ar_tag");

   leoNavigationActuator = GetActuator<CCI_LeoNavigationActuator>("leo_navigation");
   leoLocalizationSensor = GetSensor<CCI_LeoLocalizationSensor>("leo_localization");
}

void CLeoTestController::ControlStep() {
   static int counter = 0;
       std::cout<<"Localization readings: X:"<<leoLocalizationSensor->GetReading().Position.GetX()<<"  Y:"<<leoLocalizationSensor->GetReading().Position.GetY()<<"  Z:"<<leoLocalizationSensor->GetReading().Position.GetZ()<<"  "<<std::endl;
 
    if(counter % 400 == 50) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map"; pose_stamped.header.stamp = ros::Time::now(); pose_stamped.pose.position.x = -3.0; pose_stamped.pose.orientation.w = 1.0;

        leoNavigationActuator->setGoal(pose_stamped);
        std::cout<<"SENDING GOAL!!!!!!!!!!!" << std::endl;
    }

    if(counter % 400 == 250) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map"; pose_stamped.header.stamp = ros::Time::now(); pose_stamped.pose.position.x = -1.0; pose_stamped.pose.orientation.w = 1.0;

        leoNavigationActuator->setGoal(pose_stamped);
        std::cout<<"SENDING GOAL!!!!!!!!!!!" << std::endl;
    }
    
//    static int counter = 0;
//    if(counter == 0)
//       leoPoseTwistActuator->SetLinearVelocity(1.0);
//    if(counter >= 20)
//       leoPoseTwistActuator->SetLinearVelocity(0.0);
//    std::cout<<"Odometry readings: X:"<<leoOdometrySensor->GetReading().Position.GetX()<<"  Y:"<<leoOdometrySensor->GetReading().Position.GetY()<<"  Z:"<<leoOdometrySensor->GetReading().Position.GetZ()<<"  "<<std::endl;
//    for (const auto& marker : leoArTagSensor->GetReadings()){
//       std::cout<<"AR tag readings: Tag:"<<marker.TagId<<"  Confidence:"<<marker.Confidence<<"  Position: X:"<<marker.Position.GetX()<<"  Y:"<<marker.Position.GetY()<<"  Z:"<<marker.Position.GetZ()<<std::endl;
//    }
   ++counter;
}

REGISTER_CONTROLLER(CLeoTestController, "leo_test_controller")
