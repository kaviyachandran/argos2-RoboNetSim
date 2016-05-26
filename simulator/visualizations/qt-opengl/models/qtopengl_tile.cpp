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
 * @file <simulator/visualizations/models/qtopengl_tile.h>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#include "qtopengl_tile.h"
#include <argos2/common/utility/math/vector2.h>
#include <argos2/simulator/space/entities/tile_entity.h>
#include <argos2/simulator/space/entities/led_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

    CQTOpenGLTile::CQTOpenGLTile() :
       m_unVertices(20){

       /* Reserve the needed display lists */
       m_unBaseList = glGenLists(1);
       m_unBodyList = m_unBaseList;

       /* Make body list */
       glNewList(m_unBodyList, GL_COMPILE);
       MakeBody();
       glEndList();

    }

   /****************************************/
   /****************************************/

   CQTOpenGLTile::~CQTOpenGLTile() {
      glDeleteLists(m_unBaseList, 1);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLTile::Draw(const CTileEntity& c_entity) {
      /* Draw the body */
      glPushMatrix();
      glScalef(c_entity.GetSize().GetX(), c_entity.GetSize().GetY(), c_entity.GetSize().GetZ());
      glCallList(m_unBodyList);
      glPopMatrix();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLTile::MakeBody() {
	     /* Since this shape can be stretched,
	         make sure the normal vectors are unit-long */
	      glEnable(GL_NORMALIZE);

	      /* Set the material */
	      const GLfloat pfColor[]     = { 0.0f, 1.0f, 0.0f, 1.0f };
	      const GLfloat pfSpecular[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
	      const GLfloat pfShininess[] = { 0.0f                   };
	      const GLfloat pfEmission[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
	      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
	      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
	      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
	      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);

	      /* Let's start the actual shape */

	      /* This part covers the top and bottom faces (parallel to XY) */
	      glBegin(GL_QUADS);
	      /* Bottom face */
	      glNormal3f(0.0f, 0.0f, -1.0f);
	      glVertex3f( 0.5f,  0.5f, -0.5f);
	      glVertex3f( 0.5f, -0.5f, -0.5f);
	      glVertex3f(-0.5f, -0.5f, -0.5f);
	      glVertex3f(-0.5f,  0.5f, -0.5f);
	      /* Top face */
	      glNormal3f(0.0f, 0.0f, 1.0f);
	      glVertex3f(-0.5f, -0.5f, 0.5f);
	      glVertex3f( 0.5f, -0.5f, 0.5f);
	      glVertex3f( 0.5f,  0.5f, 0.5f);
	      glVertex3f(-0.5f,  0.5f, 0.5f);
	      glEnd();
	      /* This part covers the faces (South, East, North, West) */
	      glBegin(GL_QUADS);
	      /* South face */
	          glNormal3f(0.0f, -1.0f, 0.0f);
	          glVertex3f(-0.5f, -0.5f,  0.5f);
	          glVertex3f(-0.5f, -0.5f, -0.5f);
	          glVertex3f( 0.5f, -0.5f, -0.5f);
	          glVertex3f( 0.5f, -0.5f,  0.5f);
	          /* East face */
	          glNormal3f(1.0f, 0.0f, 0.0f);
	          glVertex3f( 0.5f, -0.5f,  0.5f);
	          glVertex3f( 0.5f, -0.5f, -0.5f);
	          glVertex3f( 0.5f,  0.5f, -0.5f);
	          glVertex3f( 0.5f,  0.5f,  0.5f);
	          /* North face */
	          glNormal3f(0.0f, 1.0f, 0.0f);
	          glVertex3f( 0.5f,  0.5f,  0.5f);
	          glVertex3f( 0.5f,  0.5f, -0.5f);
	          glVertex3f(-0.5f,  0.5f, -0.5f);
	          glVertex3f(-0.5f,  0.5f,  0.5f);
	          /* West face */
	          glNormal3f(-1.0f, 0.0f, 0.0f);
	          glVertex3f(-0.5f,  0.5f,  0.5f);
	          glVertex3f(-0.5f,  0.5f, -0.5f);
	          glVertex3f(-0.5f, -0.5f, -0.5f);
	          glVertex3f(-0.5f, -0.5f,  0.5f);
	      glEnd();
	      /* The shape definitions is finished */

	      /* We don't need it anymore */
	      glDisable(GL_NORMALIZE);
   }

   /****************************************/
   /****************************************/

}
