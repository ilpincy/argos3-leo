/**
 * @file <argos3/plugins/robots/leo/simulator/qtopengl_leo.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "qtopengl_leo.h"
#include "leo_entity.h"
#include "leo_measures.h"
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>
#include <QImage>

namespace argos {

   /****************************************/
   /****************************************/

   CQTOpenGLLeo::CQTOpenGLLeo() :
      m_cBodyModel("leo.obj") {
   }

   /****************************************/
   /****************************************/

   CQTOpenGLLeo::~CQTOpenGLLeo() {
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLLeo::Draw(CLeoEntity& c_entity) {
      m_cBodyModel.Draw();
   }

   /****************************************/
   /****************************************/

   class CQTOpenGLOperationDrawLeoNormal : public CQTOpenGLOperationDrawNormal {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CLeoEntity& c_entity) {
         static CQTOpenGLLeo m_cModel;
         c_visualization.DrawRays(c_entity.GetControllableEntity());
         c_visualization.DrawEntity(c_entity.GetEmbodiedEntity());
         m_cModel.Draw(c_entity);
      }
   };

   class CQTOpenGLOperationDrawLeoSelected : public CQTOpenGLOperationDrawSelected {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CLeoEntity& c_entity) {
         c_visualization.DrawBoundingBox(c_entity.GetEmbodiedEntity());
      }
   };

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLOperationDrawLeoNormal, CLeoEntity);

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLOperationDrawLeoSelected, CLeoEntity);

   /****************************************/
   /****************************************/

}
