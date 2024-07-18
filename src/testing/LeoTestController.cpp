/* Include the controller definition */
#include "LeoTestController.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

CLeoTestController::CLeoTestController() {}

void CLeoTestController::Init(TConfigurationNode& t_node) {

   leoPoseTwistActuator = GetActuator<CCI_LeoPoseTwistActuator>("leo_posetwist");
   leoOdometrySensor = GetSensor<CCI_LeoOdometrySensor>("leo_odometry");
   leoArTagSensor = GetSensor<CCI_LeoArTagSensor>("leo_ar_tag");
}

void CLeoTestController::ControlStep() {
   static int counter = 0;
   if(counter == 0)
      leoPoseTwistActuator->SetLinearVelocity(1.0);
   if(counter >= 20)
      leoPoseTwistActuator->SetLinearVelocity(0.0);
   std::cout<<"Odometry readings: X:"<<leoOdometrySensor->GetReading().Position.GetX()<<"  Y:"<<leoOdometrySensor->GetReading().Position.GetY()<<"  Z:"<<leoOdometrySensor->GetReading().Position.GetZ()<<"  "<<std::endl;
   for (const auto& marker : leoArTagSensor->GetReading()){
      std::cout<<"AR tag readings: Tag:"<<marker.TagId<<"  Confidence:"<<marker.Confidence<<"  Position: X:"<<marker.Position.GetX()<<"  Y:"<<marker.Position.GetY()<<"  Z:"<<marker.Position.GetZ()<<std::endl;
   }
   ++counter;
}

REGISTER_CONTROLLER(CLeoTestController, "leo_test_controller")
