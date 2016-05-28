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

#ifndef QTOPENGL_BLUEBOT_H
#define QTOPENGL_BLUEBOT_H

namespace argos {
   class CQTOpenGLBluebot;
   class CBluebotEntity;
}


#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos {

   class CQTOpenGLBluebot {

   public:

      CQTOpenGLBluebot();

      virtual ~CQTOpenGLBluebot();

      virtual void Draw(CBluebotEntity& c_entity);

   protected:

      /** Sets a blue plastic material */
      void SetBluePlasticMaterial();
      /** Sets a yellow plastic material */
      void SetYellowPlasticMaterial();
      /** Sets a circuit board material */
      void SetCircuitBoardMaterial();

      /** Renders a wheel */
      void RenderWheel();
      /** Renders the chassis */
      void RenderChassis();
      /** Renders the body */
      void RenderBody();

   private:

      /** Start of the display list index */
      GLuint m_unLists;

      /** E-puck wheel */
      GLuint m_unWheelList;

      /** Chassis display list */
      GLuint m_unChassisList;

      /** Body display list */
      GLuint m_unBodyList;

      /** Number of vertices to display the round parts
          (wheels, chassis, etc.) */
      GLuint m_unVertices;

   };

}

#endif
