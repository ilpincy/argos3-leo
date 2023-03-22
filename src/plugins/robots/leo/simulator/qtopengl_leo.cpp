/**
 * @file <argos3/plugins/robots/leo/simulator/qtopengl_leo.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "qtopengl_leo.h"
#include "leo_entity.h"
#include "leo_measures.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>
#include <QImage>

namespace argos {

   /****************************************/
   /****************************************/

   static QOpenGLTexture* MakeTexture(const QString& str_fname) {
      QString strPath =
         QString(ARGOS_INSTALL_PREFIX) +
         "/include/argos3/plugins/simulator/visualizations/qt-opengl/textures/" +
         str_fname;
      return new QOpenGLTexture(QImage(strPath));
   }

   /****************************************/
   /****************************************/

   CQTOpenGLLeo::CQTOpenGLLeo() :
      m_unVertices(40) {
      /* Reserve the needed textures */
      m_pcTextures[0] = MakeTexture("leo_texture_top.png");
      m_pcTextures[1] = MakeTexture("leo_texture_bottom.png");
      m_pcTextures[2] = MakeTexture("leo_texture_side.png");
      /* Reserve the needed display lists */
      m_unLists = glGenLists(2);
      /* Assign indices for better referencing (later) */
      m_unBaseList = m_unLists;
      m_unLEDList  = m_unLists + 1;
      /* Create the base display list */
      glNewList(m_unBaseList, GL_COMPILE);
      RenderBase();
      glEndList();
      /* Create the LED display list */
      glNewList(m_unLEDList, GL_COMPILE);
      RenderLED();
      glEndList();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLLeo::~CQTOpenGLLeo() {
      glDeleteLists(m_unLists, 2);
      delete m_pcTextures[0];
      delete m_pcTextures[1];
      delete m_pcTextures[2];
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLLeo::Draw(CLeoEntity& c_entity) {
      /* Place the body */
      glCallList(m_unBaseList);
      /* Place the LEDs */
      CLEDEquippedEntity& cLEDEquippedEntity = c_entity.GetLEDEquippedEntity();
      for(UInt32 i = 0; i < 3; i++) {
         const CColor& cColor = cLEDEquippedEntity.GetLED(i).GetColor();
         const CVector3& cOffset = cLEDEquippedEntity.GetLEDOffset(i);
         SetLEDMaterial(cColor.GetRed(), cColor.GetGreen(), cColor.GetBlue());
         glPushMatrix();
         glTranslatef(cOffset.GetX(), cOffset.GetY(), cOffset.GetZ());
         glCallList(m_unLEDList);
         glPopMatrix();
      }
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLLeo::SetLEDMaterial(GLfloat f_red, GLfloat f_green, GLfloat f_blue) {
      const GLfloat pfColor[]     = { f_red, f_green, f_blue, 1.0f };
      const GLfloat pfSpecular[]  = {  0.0f,    0.0f,   0.0f, 1.0f };
      const GLfloat pfShininess[] = {  0.0f                        };
      const GLfloat pfEmission[]  = { f_red, f_green, f_blue, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLLeo::RenderBase() {
      glEnable(GL_TEXTURE_2D);
      /* Used to precalculate the rotation */
      CVector2 cRot(1.0, CRadians::TWO_PI / m_unVertices);
      /* Top face is a disk */
      m_pcTextures[0]->bind();
      glBegin(GL_TRIANGLE_FAN);
      glNormal3f(0.0, 0.0, 1.0);
      glTexCoord2f(0.5, 0.5);
      glVertex3f(0.0, 0.0, LEO_BASE_TOP);
      CVector2 cVert(1.0, 0.0);
      glTexCoord2f(0.5 + 0.5 * cVert.GetX(),
                   0.5 + 0.5 * cVert.GetY());
      glVertex3f(LEO_BASE_RADIUS * cVert.GetX(),
                 LEO_BASE_RADIUS * cVert.GetY(),
                 LEO_BASE_TOP);
      for(GLuint i = 1; i <= m_unVertices; ++i) {
         cVert.Set(
            cVert.GetX() * cRot.GetX() - cVert.GetY() * cRot.GetY(),
            cVert.GetX() * cRot.GetY() + cVert.GetY() * cRot.GetX());
         glTexCoord2f(0.5 + 0.5 * cVert.GetX(),
                      0.5 + 0.5 * cVert.GetY());
         glVertex3f(LEO_BASE_RADIUS * cVert.GetX(),
                    LEO_BASE_RADIUS * cVert.GetY(),
                    LEO_BASE_TOP);
      }
      glEnd();
      /* Used to precalculate the rotation */
      cRot.FromPolarCoordinates(1.0, -CRadians::TWO_PI / m_unVertices);
      /* Bottom face is a disk */
      m_pcTextures[1]->bind();
      glBegin(GL_TRIANGLE_FAN);
      glNormal3f(0.0, 0.0, -1.0);
      glTexCoord2f(0.5, 0.5);
      glVertex3f(0.0, 0.0, 0.0);
      cVert.Set(1.0, 0.0);
      glTexCoord2f(0.5 + 0.5 * cVert.GetX(),
                   0.5 + 0.5 * cVert.GetY());
      glVertex3f(LEO_BASE_RADIUS * cVert.GetX(),
                 LEO_BASE_RADIUS * cVert.GetY(),
                 LEO_BASE_ELEVATION);
      for(GLuint i = 1; i <= m_unVertices; ++i) {
         cVert.Set(
            cVert.GetX() * cRot.GetX() - cVert.GetY() * cRot.GetY(),
            cVert.GetX() * cRot.GetY() + cVert.GetY() * cRot.GetX());
         glTexCoord2f(0.5 + 0.5 * cVert.GetX(),
                      0.5 + 0.5 * cVert.GetY());
         glVertex3f(LEO_BASE_RADIUS * cVert.GetX(),
                    LEO_BASE_RADIUS * cVert.GetY(),
                    LEO_BASE_ELEVATION);
      }
      glEnd();
      /* Side face is a cylinder */
      m_pcTextures[2]->bind();
      glBegin(GL_QUAD_STRIP);
      cVert.Set(1.0, 0.0);
      glNormal3f(cVert.GetX(), cVert.GetY(), 0.0);
      glTexCoord2f(0.5, 0.0);
      glVertex3f(LEO_BASE_RADIUS * cVert.GetX(),
                 LEO_BASE_RADIUS * cVert.GetY(),
                 LEO_BASE_ELEVATION);
      glTexCoord2f(0.5, 0.96875);
      glVertex3f(LEO_BASE_RADIUS * cVert.GetX(),
                 LEO_BASE_RADIUS * cVert.GetY(),
                 LEO_BASE_TOP);
      for(GLuint i = 1; i <= m_unVertices; ++i) {
         cVert.Set(
            cVert.GetX() * cRot.GetX() - cVert.GetY() * cRot.GetY(),
            cVert.GetX() * cRot.GetY() + cVert.GetY() * cRot.GetX());
         glNormal3f(cVert.GetX(), cVert.GetY(), 0.0);
         glTexCoord2f(0.5 + (GLfloat)i / m_unVertices, 0.0);
         glVertex3f(LEO_BASE_RADIUS * cVert.GetX(),
                    LEO_BASE_RADIUS * cVert.GetY(),
                    LEO_BASE_ELEVATION);
         glTexCoord2f(0.5 + (GLfloat)i / m_unVertices, 0.96875);
         glVertex3f(LEO_BASE_RADIUS * cVert.GetX(),
                    LEO_BASE_RADIUS * cVert.GetY(),
                    LEO_BASE_TOP);
      }
      glEnd();
      glDisable(GL_TEXTURE_2D);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLLeo::RenderLED() {
      /* Draw it as a tiny pyramid pointing to Z upwards
         (with no base, cause it's not visible anyway) */
      glBegin(GL_TRIANGLES);
      /* North */
      glVertex3f( 0.000,  0.000, 0.015);
      glVertex3f( 0.015, -0.015, 0.000);
      glVertex3f( 0.015,  0.015, 0.000);
      /* South */
      glVertex3f( 0.000,  0.000, 0.015);
      glVertex3f(-0.015,  0.015, 0.000);
      glVertex3f(-0.015, -0.015, 0.000);
      /* West */
      glVertex3f( 0.000,  0.000, 0.015);
      glVertex3f( 0.015,  0.015, 0.000);
      glVertex3f(-0.015,  0.015, 0.000);
      /* East */
      glVertex3f( 0.000,  0.000, 0.015);
      glVertex3f(-0.015, -0.015, 0.000);
      glVertex3f( 0.015, -0.015, 0.000);
      glEnd();
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
