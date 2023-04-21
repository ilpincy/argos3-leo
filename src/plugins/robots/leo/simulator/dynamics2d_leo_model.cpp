/**
 * @file <argos3/plugins/robots/leo/simulator/dynamics2d_leo_model.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "dynamics2d_leo_model.h"
#include "leo_measures.h"

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_gripping.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   // static const Real LEO_MASS                = 0.4f;
   // static const Real LEO_MAX_FORCE           = 1.5f;
   // static const Real LEO_MAX_TORQUE          = 1.5f;

   /****************************************/
   /****************************************/

   CDynamics2DLeoModel::CDynamics2DLeoModel(CDynamics2DEngine& c_engine,
                                            CLeoEntity& c_entity) :
      CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
      m_cLeoEntity(c_entity) {
      // m_cDiffSteering(c_engine,
      //                 LEO_MAX_FORCE,
      //                 LEO_MAX_TORQUE,
      //                 LEO_WHEEL_DISTANCE,
      //                 c_entity.GetConfigurationNode()),
      // m_fCurrentWheelVelocity(m_cWheeledEntity.GetWheelVelocities()) {
      // /* Create the body with initial position and orientation */
      // cpBody* ptBody =
      //    cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
      //                   cpBodyNew(LEO_MASS,
      //                             cpMomentForCircle(LEO_MASS,
      //                                               0.0f,
      //                                               LEO_BASE_RADIUS + LEO_BASE_RADIUS,
      //                                               cpvzero)));
      // const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
      // ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      // CRadians cXAngle, cYAngle, cZAngle;
      // GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      // cpBodySetAngle(ptBody, cZAngle.GetValue());
      // /* Create the body shape */
      // cpShape* ptShape =
      //    cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
      //                    cpCircleShapeNew(ptBody,
      //                                     LEO_BASE_RADIUS,
      //                                     cpvzero));
      // ptShape->e = 0.0; // No elasticity
      // ptShape->u = 0.7; // Lots of friction
      // /* Constrain the actual base body to follow the diff steering control */
      // m_cDiffSteering.AttachTo(ptBody);
      // /* Set the body so that the default methods work as expected */
      // SetBody(ptBody, LEO_BASE_TOP);
   }

   /****************************************/
   /****************************************/

   CDynamics2DLeoModel::~CDynamics2DLeoModel() {
      // m_cDiffSteering.Detach();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DLeoModel::Reset() {
      CDynamics2DSingleBodyObjectModel::Reset();
      // m_cDiffSteering.Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DLeoModel::UpdateFromEntityStatus() {
      // /* Do we want to move? */
      // if((m_fCurrentWheelVelocity[LEO_LEFT_WHEEL] != 0.0f) ||
      //    (m_fCurrentWheelVelocity[LEO_RIGHT_WHEEL] != 0.0f)) {
      //    m_cDiffSteering.SetWheelVelocity(m_fCurrentWheelVelocity[LEO_LEFT_WHEEL],
      //                                     m_fCurrentWheelVelocity[LEO_RIGHT_WHEEL]);
      // }
      // else {
      //    /* No, we don't want to move - zero all speeds */
      //    m_cDiffSteering.Reset();
      // }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CLeoEntity, CDynamics2DLeoModel);

   /****************************************/
   /****************************************/

}
