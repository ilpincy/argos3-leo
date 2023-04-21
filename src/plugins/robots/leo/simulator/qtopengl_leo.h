/**
 * @file <argos3/plugins/robots/leo/simulator/qtopengl_leo.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef QTOPENGL_LEO_H
#define QTOPENGL_LEO_H

namespace argos {
   class CQTOpenGLLeo;
   class CLeoEntity;
}

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_obj_model.h>

namespace argos {

   class CQTOpenGLLeo {

   public:

      CQTOpenGLLeo();

      virtual ~CQTOpenGLLeo();

      virtual void Draw(CLeoEntity& c_entity);

   private:

      CQTOpenGLObjModel m_cBodyModel;

   };

}

#endif
