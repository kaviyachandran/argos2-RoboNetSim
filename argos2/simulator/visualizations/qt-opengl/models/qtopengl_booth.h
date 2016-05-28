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

#ifndef QTOPENGL_BOOTH_H
#define QTOPENGL_BOOTH_H

namespace argos {
   class CQTOpenGLBooth;
   class CBoothEntity;
}

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos {

   class CQTOpenGLBooth {

   public:

      CQTOpenGLBooth();

      virtual ~CQTOpenGLBooth();

      virtual void Draw(CBoothEntity& c_entity);

   protected:
   
   /** Sets sides material */
      void SetSideMaterial();   
   /** 
    * Left, Back and Right are considered when looking the booth
    * from the top with the back close to the viewer.
    * Like this:
    *    L |_| R
    *       B    
    */
      void MakeLateralSide();
      void MakeBackSide();

   /** Make LED */
      void MakeLED();

   private:

	/** Start of the display list index */
      GLuint m_unLists;
	
    /** LeftSide list */  
      GLuint m_unLeftSideList;
      
    /** BackSide list */  
      GLuint m_unBackSideList;
      
    /** RightSide list */  
      GLuint m_unRightSideList;
      
    /** LED display list */
      GLuint m_unLEDList;

   };

}

#endif
