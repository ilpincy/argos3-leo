/* Include the controller definition */
#include "TestController.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

CTestController::CTestController() {}

void CTestController::Init(TConfigurationNode& t_node) {

   leoPoseTwistActuator = GetActuator<CCI_LeoPoseTwistActuator>("leo_posetwist");
}

void CTestController::ControlStep() {
    leoPoseTwistActuator->SetLinearVelocity(1.0);
}

REGISTER_CONTROLLER(CTestController, "test_controller")
