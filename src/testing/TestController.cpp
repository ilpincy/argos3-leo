/* Include the controller definition */
#include "TestController.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

CTestController::CTestController() {}

void CTestController::Init(TConfigurationNode& t_node) {

   leoPoseTwistActuator = GetActuator<CCI_LeoPoseTwistActuator>("leo_posetwist");
   leoOdometrySensor = GetSensor<CCI_LeoOdometrySensor>("leo_odometry");
}

void CTestController::ControlStep() {
   static int counter = 0;
   if(counter == 0)
      leoPoseTwistActuator->SetLinearVelocity(1.0);
   if(counter >= 20)
      leoPoseTwistActuator->SetLinearVelocity(0.0);
   std::cout<<"Odometry readings: X:"<<leoOdometrySensor->GetReading().Position.GetX()<<"  Y:"<<leoOdometrySensor->GetReading().Position.GetY()<<"  Z:"<<leoOdometrySensor->GetReading().Position.GetZ()<<"  "<<std::endl;
   ++counter;
}

REGISTER_CONTROLLER(CTestController, "test_controller")
