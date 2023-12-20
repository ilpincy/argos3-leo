/**
 * @file <argos3/plugins/robots/leo/simulator/dynamics2d_leo_model.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "dynamics2d_leo_model.h"
#include "leo_measures.h"

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   static const Real LEO_MASS        = 0.4f;
   static const Real LEO_MAX_FORCE   = 1.5f;
   static const Real LEO_MAX_TORQUE  = 1.5f;
   static const Real LEO_BODY_LENGTH = 0.43716;
   static const Real LEO_BODY_WIDTH  = 0.44152;
   static const Real LEO_BODY_HEIGHT = 0.16309;

   /* The model is a simple rectangle anchored at the base center */
   /* NOTE: the points must be defined in a clockwise winding */
   static cpVect tBodyVertices[] = {
      cpv(-LEO_BODY_LENGTH / 2, -LEO_BODY_WIDTH / 2),
      cpv(-LEO_BODY_LENGTH / 2,  LEO_BODY_WIDTH / 2),
      cpv( LEO_BODY_LENGTH / 2,  LEO_BODY_WIDTH / 2),
      cpv( LEO_BODY_LENGTH / 2, -LEO_BODY_WIDTH / 2),
   };

   /****************************************/
   /****************************************/

   CDynamics2DLeoModel::CDynamics2DLeoModel(CDynamics2DEngine& c_engine,
                                            CLeoEntity& c_entity) :
      CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
      m_cLeoEntity(c_entity),
      m_cVelocityControl(c_engine, LEO_MAX_FORCE, LEO_MAX_TORQUE) {
      /* Create the body with initial position and orientation */
      cpBody* ptBody =
         cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                        cpBodyNew(LEO_MASS,
                                  cpMomentForPoly(LEO_MASS,
                                                  4,
                                                  tBodyVertices,
                                                  cpvzero)));
      const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
      ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      CRadians cXAngle, cYAngle, cZAngle;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      cpBodySetAngle(ptBody, cZAngle.GetValue());
      /* Create the body shape */
      cpShape* ptShape =
         cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                         cpPolyShapeNew(ptBody,
                                        4,
                                        tBodyVertices,
                                        cpvzero));
      ptShape->e = 0.0; // No elasticity
      ptShape->u = 0.7; // Lots of friction
      /* Constrain the actual base body to follow the control body */
      m_cVelocityControl.AttachTo(ptBody);
      /* Set the body so that the default methods work as expected */
      SetBody(ptBody, LEO_BODY_HEIGHT);
   }

   /****************************************/
   /****************************************/

   CDynamics2DLeoModel::~CDynamics2DLeoModel() {
      m_cVelocityControl.Detach();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DLeoModel::Reset() {
      CDynamics2DSingleBodyObjectModel::Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DLeoModel::UpdateFromEntityStatus() {
      argos::CRadians fRotationZ, fRotationX, fRotationY;
      m_cLeoEntity.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(fRotationZ, fRotationY, fRotationX);

      argos::Real fVelX = argos::Cos(fRotationZ)*m_cLeoEntity.GetLinearVelocity();
      argos::Real fVelY = argos::Sin(fRotationZ)*m_cLeoEntity.GetLinearVelocity();
      
      m_cVelocityControl.SetLinearVelocity(CVector2(fVelX, fVelY));
      m_cVelocityControl.SetAngularVelocity(m_cLeoEntity.GetAngularVelocity().GetValue());
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CLeoEntity, CDynamics2DLeoModel);

   /****************************************/
   /****************************************/

}
