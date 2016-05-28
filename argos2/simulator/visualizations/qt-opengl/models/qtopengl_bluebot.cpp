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
 * @file <simulator/visualizations/models/qtopengl_bluebot.h>
 *
 * @author Ali Emre Turgut - <ali.turgut@gmail.com>
 */

#include "qtopengl_bluebot.h"
#include <argos2/common/utility/math/vector2.h>
#include <argos2/common/utility/math/vector3.h>
#include <argos2/common/utility/logging/argos_log.h>
#include <argos2/simulator/space/entities/bluebot_entity.h>
#include <argos2/simulator/space/entities/led_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   /* All measures are in meters */

   static const Real BODY_RADIUS                 = 0.06f;

   static const Real CHASSIS_ELEVATION           = 0.005f;
   static const Real WHEEL_DIAMETER              = 0.05f;
   static const Real WHEEL_RADIUS                = WHEEL_DIAMETER * 0.5f;
   static const Real WHEEL_WIDTH                 = 0.03f;
   static const Real HALF_WHEEL_WIDTH            = WHEEL_WIDTH * 0.5f;
   static const Real INTERWHEEL_DISTANCE         = (BODY_RADIUS-WHEEL_WIDTH) * 2.0f;
   static const Real HALF_INTERWHEEL_DISTANCE    = INTERWHEEL_DISTANCE * 0.5f;

   static const Real BODY_ELEVATION              = WHEEL_DIAMETER + CHASSIS_ELEVATION;
   static const Real BODY_HEIGHT                 = 0.045f;

   static const Real HALF_CHASSIS_LENGTH         = BODY_RADIUS * 0.9f;
   static const Real HALF_CHASSIS_WIDTH          = HALF_INTERWHEEL_DISTANCE - HALF_WHEEL_WIDTH;

   /****************************************/
   /****************************************/

   CQTOpenGLBluebot::CQTOpenGLBluebot() :
      m_unVertices(40) {
      /* Reserve the needed display lists */
      m_unLists = glGenLists(3);

      /* Assign indices for better referencing (later) */
      m_unWheelList   = m_unLists;
      m_unChassisList = m_unLists + 1;
      m_unBodyList    = m_unLists + 2;

      /* Create the wheel display list */
      glNewList(m_unWheelList, GL_COMPILE);
      RenderWheel();
      glEndList();

      /* Create the body display list */
      glNewList(m_unBodyList, GL_COMPILE);
      RenderBody();
      glEndList();

      /* Create the chassis display list */
      glNewList(m_unChassisList, GL_COMPILE);
      RenderChassis();
      glEndList();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLBluebot::~CQTOpenGLBluebot() {
      glDeleteLists(m_unLists, 3);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLBluebot::Draw(CBluebotEntity& c_entity) {
      /* Place the chassis */
      glCallList(m_unChassisList);
      /* Place the body */
      glCallList(m_unBodyList);
      /* Place the wheels */
      glPushMatrix();
      glTranslatef(0.0f, HALF_INTERWHEEL_DISTANCE, 0.0f);
      glCallList(m_unWheelList);
      glPopMatrix();
      glPushMatrix();
      glTranslatef(0.0f, -HALF_INTERWHEEL_DISTANCE, 0.0f);
      glCallList(m_unWheelList);
      glPopMatrix();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLBluebot::SetBluePlasticMaterial() {
      const GLfloat pfColor[]     = {   0.0f, 0.0f, 1.0f, 1.0f };
      const GLfloat pfSpecular[]  = {   0.9f, 0.9f, 0.9f, 1.0f };
      const GLfloat pfShininess[] = { 100.0f                   };
      const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLBluebot::SetYellowPlasticMaterial() {
      const GLfloat pfColor[]     = {   1.0f, 1.0f, 0.0f, 1.0f };
      const GLfloat pfSpecular[]  = {   0.9f, 0.9f, 0.9f, 1.0f };
      const GLfloat pfShininess[] = { 100.0f                   };
      const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLBluebot::SetCircuitBoardMaterial() {
      const GLfloat pfColor[]     = { 0.0f, 0.0f, 1.0f, 1.0f };
      const GLfloat pfSpecular[]  = { 0.5f, 0.5f, 1.0f, 1.0f };
      const GLfloat pfShininess[] = { 10.0f                  };
      const GLfloat pfEmission[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,            pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS,           pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,            pfEmission);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLBluebot::RenderWheel() {
      /* Set material */
      SetYellowPlasticMaterial();
      /* Right side */
      CVector2 cVertex(WHEEL_RADIUS, 0.0f);
      CRadians cAngle(CRadians::TWO_PI / m_unVertices);
      CVector3 cNormal(-1.0f, -1.0f, 0.0f);
      cNormal.Normalize();
      glBegin(GL_POLYGON);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3f(cVertex.GetX(), -HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
      /* Left side */
      cVertex.Set(WHEEL_RADIUS, 0.0f);
      cNormal.Set(-1.0f, 1.0f, 0.0f);
      cNormal.Normalize();
      cAngle = -cAngle;
      glBegin(GL_POLYGON);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3f(cVertex.GetX(), HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
      /* Tire */
      cNormal.Set(1.0f, 0.0f, 0.0f);
      cVertex.Set(WHEEL_RADIUS, 0.0f);
      cAngle = -cAngle;
      glBegin(GL_QUAD_STRIP);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
         glVertex3f(cVertex.GetX(), -HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         glVertex3f(cVertex.GetX(),  HALF_WHEEL_WIDTH, WHEEL_RADIUS + cVertex.GetY());
         cVertex.Rotate(cAngle);
         cNormal.RotateY(cAngle);
      }
      glEnd();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLBluebot::RenderChassis() {
      /* Set material */
      SetBluePlasticMaterial();
      /* This part covers the bottom face (parallel to XY) */
      glBegin(GL_QUADS);
      /* Bottom face */
      glNormal3f(0.0f, 0.0f, -1.0f);
      glVertex3f( HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glVertex3f( HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glVertex3f(-HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glEnd();
      /* This part covers the faces (South, East, North, West) */
      glBegin(GL_QUAD_STRIP);
      /* Starting side */
      glNormal3f(-1.0f, 0.0f, 0.0f);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      /* South face */
      glVertex3f( HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f( HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      /* East face */
      glNormal3f(0.0f, -1.0f, 0.0f);
      glVertex3f( HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f( HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      /* North face */
      glNormal3f(1.0f, 0.0f, 0.0f);
      glVertex3f(-HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f(-HALF_CHASSIS_LENGTH,  HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      /* West face */
      glNormal3f(0.0f, 1.0f, 0.0f);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION + WHEEL_DIAMETER);
      glVertex3f(-HALF_CHASSIS_LENGTH, -HALF_CHASSIS_WIDTH, CHASSIS_ELEVATION);
      glEnd();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLBluebot::RenderBody() {
      /* Set material */
      SetBluePlasticMaterial();
      CVector2 cVertex(BODY_RADIUS, 0.0f);
      CRadians cAngle(-CRadians::TWO_PI / m_unVertices);
      /* Bottom part */
      glBegin(GL_POLYGON);
      glNormal3f(0.0f, 0.0f, -1.0f);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION);
         cVertex.Rotate(cAngle);
      }
      glEnd();
      /* Side surface */
      cAngle = -cAngle;
      CVector2 cNormal(1.0f, 0.0f);
      cVertex.Set(BODY_RADIUS, 0.0f);
      glBegin(GL_QUAD_STRIP);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3f(cNormal.GetX(), cNormal.GetY(), 0.0f);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + BODY_HEIGHT);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION);
         cVertex.Rotate(cAngle);
         cNormal.Rotate(cAngle);
      }
      glEnd();
      /* Top part */
      glBegin(GL_POLYGON);
      cVertex.Set(BODY_RADIUS, 0.0f);
      glNormal3f(0.0f, 0.0f, 1.0f);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glVertex3f(cVertex.GetX(), cVertex.GetY(), BODY_ELEVATION + BODY_HEIGHT);
         cVertex.Rotate(cAngle);
      }
      glEnd();
      /* Triangle to set the direction */
      SetYellowPlasticMaterial();
      glBegin(GL_TRIANGLES);
      glVertex3f( BODY_RADIUS * 0.7,               0.0f, BODY_ELEVATION + BODY_HEIGHT + 0.001f);
      glVertex3f(-BODY_RADIUS * 0.7,  BODY_RADIUS * 0.3, BODY_ELEVATION + BODY_HEIGHT + 0.001f);
      glVertex3f(-BODY_RADIUS * 0.7, -BODY_RADIUS * 0.3, BODY_ELEVATION + BODY_HEIGHT + 0.001f);
      glEnd();
   }

   /****************************************/
   /****************************************/

}
