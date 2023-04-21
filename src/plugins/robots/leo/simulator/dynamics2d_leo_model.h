/**
 * @file <argos3/plugins/robots/leo/simulator/dynamics2d_leo_model.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef DYNAMICS2D_LEO_MODEL_H
#define DYNAMICS2D_LEO_MODEL_H

namespace argos {
   class CDynamics2DDifferentialSteeringControl;
   class CDynamics2DGripper;
   class CDynamics2DGrippable;
   class CDynamics2DLeoModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include <argos3/plugins/robots/leo/simulator/leo_entity.h>

namespace argos {

   class CDynamics2DLeoModel : public CDynamics2DSingleBodyObjectModel {

   public:

      CDynamics2DLeoModel(CDynamics2DEngine& c_engine,
                          CLeoEntity& c_entity);
      virtual ~CDynamics2DLeoModel();

      virtual void Reset();

      virtual void UpdateFromEntityStatus();
      
   private:

      CLeoEntity& m_cLeoEntity;
   };

}

#endif
