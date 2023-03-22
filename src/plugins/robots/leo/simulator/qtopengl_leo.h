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

#include <QOpenGLTexture>

namespace argos {

   class CQTOpenGLLeo {

   public:

      CQTOpenGLLeo();

      virtual ~CQTOpenGLLeo();

      virtual void Draw(CLeoEntity& c_entity);

   protected:

      /** Sets a white plastic material */
      void SetWhitePlasticMaterial();
      /** Sets a blue plastic material */
      void SetBluePlasticMaterial();
      /** Sets a colored LED material */
      void SetLEDMaterial(GLfloat f_red,
                          GLfloat f_green,
                          GLfloat f_blue);

      /** Renders the base */
      void RenderBase();
      /** A single LED of the ring */
      void RenderLED();

   private:

      /** Start of the texture index
       * Order: top, bottom, side
       */
      QOpenGLTexture* m_pcTextures[3];

      /** Start of the display list index */
      GLuint m_unLists;

      /** Base display list */
      GLuint m_unBaseList;

      /** LED display list */
      GLuint m_unLEDList;

      /** Number of vertices to display the round parts
          (wheels, chassis, etc.) */
      GLuint m_unVertices;

   };

}

#endif
