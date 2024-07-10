/* Include the controller definition */
#include "SimTestController.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

CSimTestController::CSimTestController() {}

void CSimTestController::Init(TConfigurationNode& t_node) {

   leoPoseTwistActuator = GetActuator<CCI_LeoPoseTwistActuator>("leo_posetwist");
}

void CSimTestController::ControlStep() {
    leoPoseTwistActuator->SetLinearVelocity(1.0);
}

REGISTER_CONTROLLER(CSimTestController, "sim_test_controller")