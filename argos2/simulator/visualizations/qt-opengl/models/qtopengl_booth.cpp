/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**
 * @file <simulator/visualizations/models/qtopengl_booth.h>
 *
 * @author Manuele Brambilla - <mbrambilla@iridia.ulb.ac.be>
 */

#include "qtopengl_booth.h"
#include <argos2/common/utility/math/vector2.h>
#include <argos2/simulator/space/entities/booth_entity.h>
#include <argos2/simulator/space/entities/led_equipped_entity.h>
#include <argos2/simulator/space/entities/embodied_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   /* BACK SIDE */
   static const Real BACK_SIDE_HEIGHT         = 0.1f;
   static const Real BACK_SIDE_WIDTH          = 0.08f;
   static const Real BACK_SIDE_HALF_WIDTH     = BACK_SIDE_WIDTH * 0.5f;
   static const Real BACK_SIDE_DEPTH          = 0.03f;
   static const Real BACK_SIDE_HALF_DEPTH     = BACK_SIDE_DEPTH * 0.5f;
   
   /* LATERAL SIDE */
   static const Real LATERAL_SIDE_HEIGHT         = 0.1f;
   static const Real LATERAL_SIDE_WIDTH          = 0.1f;
   static const Real LATERAL_SIDE_HALF_WIDTH     = LATERAL_SIDE_WIDTH * 0.5f;
   static const Real LATERAL_SIDE_DEPTH          = 0.01f;
   static const Real LATERAL_SIDE_HALF_DEPTH     = LATERAL_SIDE_DEPTH * 0.5f;
   
   /* TRANSLATIONS (condering 0,0 the center of the box) */
   static const Real LEFT_SIDE_TRANS			 = 0.045f;
   static const Real BACK_SIDE_TRANS			 = -0.035f;
   static const Real RIGHT_SIDE_TRANS		     = -0.045f;
   static const Real LED_PANEL_TOP_TRANS		 = -0.0275f;

   /****************************************/
   /****************************************/

    CQTOpenGLBooth::CQTOpenGLBooth(){

       /* Reserve the needed display lists */
       m_unLists = glGenLists(4);
       m_unLeftSideList  = m_unLists;
       m_unBackSideList  = m_unLists+1;
       m_unRightSideList = m_unLists+2;
       m_unLEDList       = m_unLists+3;

	   /* Make LeftSide list */
       glNewList(m_unLeftSideList, GL_COMPILE);
       MakeLateralSide();
       glEndList();  
              
       /* Make BackSide list */
       glNewList(m_unBackSideList, GL_COMPILE);
       MakeBackSide();
       glEndList();
       
       /* Make RightSide list */
       glNewList(m_unRightSideList, GL_COMPILE);
       MakeLateralSide();
       glEndList();
       
       /* Make LED panel list */
       glNewList(m_unLEDList, GL_COMPILE);
       MakeLED();
       glEndList();

    }

   /****************************************/
   /****************************************/

   CQTOpenGLBooth::~CQTOpenGLBooth() {
      glDeleteLists(m_unLists, 4);
   }
   
   /****************************************/
   /****************************************/

   void CQTOpenGLBooth::Draw(CBoothEntity& c_entity) {
	  /* Place the LeftSide */
	  glPushMatrix();
	  glRotatef(90.0f, 0.0f, 0.0f, 1.0f);
      glTranslatef(0.0f, LEFT_SIDE_TRANS, 0.0f);
      glCallList(m_unLeftSideList);
      glPopMatrix();
      /* Place the BackSide */
      glPushMatrix();
      glTranslatef(0.0f, BACK_SIDE_TRANS, 0.0f);
      glCallList(m_unBackSideList);
      glPopMatrix();
      /* Place the RightSide */
      glPushMatrix();
	  glRotatef(90.0f, 0.0f, 0.0f, 1.0f);
      glTranslatef(0.0f, RIGHT_SIDE_TRANS, 0.0f);
      glCallList(m_unRightSideList);
      glPopMatrix();
      
       /* Draw the LEDs */
      GLfloat pfColor[]           = {   0.0f, 0.0f, 0.0f, 1.0f };
      const GLfloat pfSpecular[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      const GLfloat pfShininess[] = { 100.0f                   };
      const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);
      CLedEquippedEntity& cLEDEquippedEntity = c_entity.GetLEDEquippedEntity();
      for(UInt32 i = 0; i < cLEDEquippedEntity.GetAllLeds().size(); ++i) {
         glPushMatrix();
         /* Set the material */
         const CColor& cColor = cLEDEquippedEntity.GetLED(i).GetColor();
         pfColor[0] = cColor.GetRed();
         pfColor[1] = cColor.GetGreen();
         pfColor[2] = cColor.GetBlue();
         glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
         //const CVector3& cPosition = cLEDEquippedEntity.GetLED(i).GetPosition();
         /* Perform rototranslation */
         glTranslatef(0.0f, LED_PANEL_TOP_TRANS, 0.0f);
         /* Draw the LED */
         glCallList(m_unLEDList);
         glPopMatrix();
      }
      
   }
   
   /****************************************/
   /****************************************/

   void CQTOpenGLBooth::MakeLED() {
      /* The led panel is composed of two rectangles
       * The first is the internal face of the booth
       * while the second covers half of the depth 
       * of the back wall of the booth */
      
      //the +0.00.. is to make sure that a layer is visible on top of another
      
      // top face 
      glBegin(GL_QUADS);
      glNormal3f(0.0f, 0.0f, 1.0f);
      glVertex3f(-BACK_SIDE_HALF_WIDTH, -BACK_SIDE_HALF_DEPTH * 0.5f, BACK_SIDE_HEIGHT+0.00001f); 
      glVertex3f( BACK_SIDE_HALF_WIDTH, -BACK_SIDE_HALF_DEPTH * 0.5f, BACK_SIDE_HEIGHT+0.00001f);
      glVertex3f( BACK_SIDE_HALF_WIDTH,  BACK_SIDE_HALF_DEPTH * 0.5f, BACK_SIDE_HEIGHT+0.00001f);
      glVertex3f(-BACK_SIDE_HALF_WIDTH,  BACK_SIDE_HALF_DEPTH * 0.5f, BACK_SIDE_HEIGHT+0.00001f);
      glEnd();
      
      // front face
      glBegin(GL_QUADS);
      glNormal3f(0.0f, 0.0f, 1.0f);
      glVertex3f(-BACK_SIDE_HALF_WIDTH, BACK_SIDE_HALF_DEPTH* 0.5f+0.00001f, BACK_SIDE_HEIGHT);
      glVertex3f(BACK_SIDE_HALF_WIDTH,  BACK_SIDE_HALF_DEPTH* 0.5f+0.00001f, BACK_SIDE_HEIGHT);
      glVertex3f(BACK_SIDE_HALF_WIDTH,  BACK_SIDE_HALF_DEPTH* 0.5f+0.00001f, 0.0f);
      glVertex3f(-BACK_SIDE_HALF_WIDTH, BACK_SIDE_HALF_DEPTH* 0.5f+0.00001f, 0.0f);
      glEnd();
      
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLBooth::SetSideMaterial() {
      const GLfloat pfColor[]     = { 0.5f, 1.0f, 0.5f, 1.0f };
	  const GLfloat pfSpecular[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
	  const GLfloat pfShininess[] = { 0.0f                   };
	  const GLfloat pfEmission[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
	  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
	  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
	  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
	  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);
   }


   /****************************************/
   /****************************************/

   void CQTOpenGLBooth::MakeBackSide() {
	  /* Set material */
	  SetSideMaterial();
	  /* Top face */
      glBegin(GL_QUADS);
      glNormal3f(0.0f, 0.0f, 1.0f);
      glVertex3f(-BACK_SIDE_HALF_WIDTH, -BACK_SIDE_HALF_DEPTH, BACK_SIDE_HEIGHT);
      glVertex3f( BACK_SIDE_HALF_WIDTH, -BACK_SIDE_HALF_DEPTH, BACK_SIDE_HEIGHT);
      glVertex3f( BACK_SIDE_HALF_WIDTH,  BACK_SIDE_HALF_DEPTH, BACK_SIDE_HEIGHT);
      glVertex3f(-BACK_SIDE_HALF_WIDTH,  BACK_SIDE_HALF_DEPTH, BACK_SIDE_HEIGHT);
      glEnd();
      /* Bottom face */
      glBegin(GL_QUADS);
      glNormal3f(0.0f, 0.0f, -1.0f);
      glVertex3f(-BACK_SIDE_HALF_WIDTH, -BACK_SIDE_HALF_DEPTH, 0.0f);
      glVertex3f(-BACK_SIDE_HALF_WIDTH,  BACK_SIDE_HALF_DEPTH, 0.0f);
      glVertex3f( BACK_SIDE_HALF_WIDTH,  BACK_SIDE_HALF_DEPTH, 0.0f);
      glVertex3f( BACK_SIDE_HALF_WIDTH, -BACK_SIDE_HALF_DEPTH, 0.0f);
      glEnd();
      /* This part covers the faces (South, East, North, West) */
      glBegin(GL_QUAD_STRIP);
      /* Starting vertex */
      glNormal3f(0.0f, -1.0f, 0.0f);
      glVertex3f(-BACK_SIDE_HALF_WIDTH, -BACK_SIDE_HALF_DEPTH, BACK_SIDE_HEIGHT);
      glVertex3f(-BACK_SIDE_HALF_WIDTH, -BACK_SIDE_HALF_DEPTH, 0.0f);
      /* South face */
      glVertex3f( BACK_SIDE_HALF_WIDTH, -BACK_SIDE_HALF_DEPTH, BACK_SIDE_HEIGHT);
      glVertex3f( BACK_SIDE_HALF_WIDTH, -BACK_SIDE_HALF_DEPTH, 0.0f);
      /* East face */
      glNormal3f(1.0f, 0.0f, 0.0f);
      glVertex3f( BACK_SIDE_HALF_WIDTH,  BACK_SIDE_HALF_DEPTH, BACK_SIDE_HEIGHT);
      glVertex3f( BACK_SIDE_HALF_WIDTH,  BACK_SIDE_HALF_DEPTH, 0.0f);
      /* North face */
      glNormal3f(0.0f, 1.0f, 0.0f);
      glVertex3f(-BACK_SIDE_HALF_WIDTH,  BACK_SIDE_HALF_DEPTH, BACK_SIDE_HEIGHT);
      glVertex3f(-BACK_SIDE_HALF_WIDTH,  BACK_SIDE_HALF_DEPTH, 0.0f);
      /* West face */
      glNormal3f(-1.0f, 0.0f, 0.0f);
      glVertex3f(-BACK_SIDE_HALF_WIDTH, -BACK_SIDE_HALF_DEPTH, BACK_SIDE_HEIGHT);
      glVertex3f(-BACK_SIDE_HALF_WIDTH, -BACK_SIDE_HALF_DEPTH, 0.0f);
      glEnd();
   }
   
   /****************************************/
   /****************************************/

   void CQTOpenGLBooth::MakeLateralSide() {
	  /* Set material */
	  SetSideMaterial();
	  /* Top face */
      glBegin(GL_QUADS);
      glNormal3f(0.0f, 0.0f, 1.0f);
      glVertex3f(-LATERAL_SIDE_HALF_WIDTH, -LATERAL_SIDE_HALF_DEPTH, LATERAL_SIDE_HEIGHT);
      glVertex3f( LATERAL_SIDE_HALF_WIDTH, -LATERAL_SIDE_HALF_DEPTH, LATERAL_SIDE_HEIGHT);
      glVertex3f( LATERAL_SIDE_HALF_WIDTH,  LATERAL_SIDE_HALF_DEPTH, LATERAL_SIDE_HEIGHT);
      glVertex3f(-LATERAL_SIDE_HALF_WIDTH,  LATERAL_SIDE_HALF_DEPTH, LATERAL_SIDE_HEIGHT);
      glEnd();
      /* Bottom face */
      glBegin(GL_QUADS);
      glNormal3f(0.0f, 0.0f, -1.0f);
      glVertex3f(-LATERAL_SIDE_HALF_WIDTH, -LATERAL_SIDE_HALF_DEPTH, 0.0f);
      glVertex3f(-LATERAL_SIDE_HALF_WIDTH,  LATERAL_SIDE_HALF_DEPTH, 0.0f);
      glVertex3f( LATERAL_SIDE_HALF_WIDTH,  LATERAL_SIDE_HALF_DEPTH, 0.0f);
      glVertex3f( LATERAL_SIDE_HALF_WIDTH, -LATERAL_SIDE_HALF_DEPTH, 0.0f);
      glEnd();
      /* This part covers the faces (South, East, North, West) */
      glBegin(GL_QUAD_STRIP);
      /* Starting vertex */
      glNormal3f(0.0f, -1.0f, 0.0f);
      glVertex3f(-LATERAL_SIDE_HALF_WIDTH, -LATERAL_SIDE_HALF_DEPTH, LATERAL_SIDE_HEIGHT);
      glVertex3f(-LATERAL_SIDE_HALF_WIDTH, -LATERAL_SIDE_HALF_DEPTH, 0.0f);
      /* South face */
      glVertex3f( LATERAL_SIDE_HALF_WIDTH, -LATERAL_SIDE_HALF_DEPTH, LATERAL_SIDE_HEIGHT);
      glVertex3f( LATERAL_SIDE_HALF_WIDTH, -LATERAL_SIDE_HALF_DEPTH, 0.0f);
      /* East face */
      glNormal3f(1.0f, 0.0f, 0.0f);
      glVertex3f( LATERAL_SIDE_HALF_WIDTH,  LATERAL_SIDE_HALF_DEPTH, LATERAL_SIDE_HEIGHT);
      glVertex3f( LATERAL_SIDE_HALF_WIDTH,  LATERAL_SIDE_HALF_DEPTH, 0.0f);
      /* North face */
      glNormal3f(0.0f, 1.0f, 0.0f);
      glVertex3f(-LATERAL_SIDE_HALF_WIDTH,  LATERAL_SIDE_HALF_DEPTH, LATERAL_SIDE_HEIGHT);
      glVertex3f(-LATERAL_SIDE_HALF_WIDTH,  LATERAL_SIDE_HALF_DEPTH, 0.0f);
      /* West face */
      glNormal3f(-1.0f, 0.0f, 0.0f);
      glVertex3f(-LATERAL_SIDE_HALF_WIDTH, -LATERAL_SIDE_HALF_DEPTH, LATERAL_SIDE_HEIGHT);
      glVertex3f(-LATERAL_SIDE_HALF_WIDTH, -LATERAL_SIDE_HALF_DEPTH, 0.0f);
      glEnd();
   }

   /****************************************/
   /****************************************/

}
