/* -*- Mode: C++ -*-
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
 * @file <argos2/simulator/physics_engines/dynamics2d_booth_entity.cpp>
 *
 * @author Manuele Brambilla - <mbrambilla@iridia.ulb.ac.be>
 */

#include <argos2/simulator/space/entities/embodied_entity.h>
#include "dynamics2d_booth_entity.h"
#include "dynamics2d_engine.h"

namespace argos {

   /****************************************/
   /****************************************/

   static const Real BOOTH_HALF_SIZE		= 0.05f;
   static const Real LATERAL_WALL_SIZE		= 0.01f;
   static const Real BACK_WALL_SIZE			= 0.03f;
   static const Real BOOTH_HEIGHT           = 0.1f;

   /****************************************/
   /****************************************/

   CDynamics2DBoothEntity::CDynamics2DBoothEntity(CDynamics2DEngine& c_engine,
                                              CBoothEntity& c_entity) :
      CDynamics2DEntity(c_engine, c_entity.GetEmbodiedEntity()),
      m_cBoothEntity(c_entity),
      m_ptLeftSideShape(NULL),
      m_ptBackSideShape(NULL),
      m_ptRightSideShape(NULL){
      /* Create a polygonal object in the physics space */
	  /* Note that the booth is considered to be non-movable,
	   * if you want to move it, implement the necessary code*/

      /* Start defining the vertices
         NOTE: points must be defined in a clockwise winding
      */

	   // left side
	   cpVect tLeftSideVertices[] = {
			   cpv(-BOOTH_HALF_SIZE                  ,  BOOTH_HALF_SIZE), // -5, 5
			   cpv(-BOOTH_HALF_SIZE+LATERAL_WALL_SIZE,  BOOTH_HALF_SIZE), // -4, 5
			   cpv(-BOOTH_HALF_SIZE+LATERAL_WALL_SIZE, -BOOTH_HALF_SIZE), // -4,-5
			   cpv(-BOOTH_HALF_SIZE                  , -BOOTH_HALF_SIZE)  // -5,-5
	   };

	   // back side
	   cpVect tBackSideVertices[] = {
			   cpv(-BOOTH_HALF_SIZE+LATERAL_WALL_SIZE, -BOOTH_HALF_SIZE+BACK_WALL_SIZE), // -4,-2
			   cpv( BOOTH_HALF_SIZE-LATERAL_WALL_SIZE, -BOOTH_HALF_SIZE+BACK_WALL_SIZE), //  4,-2
			   cpv( BOOTH_HALF_SIZE-LATERAL_WALL_SIZE, -BOOTH_HALF_SIZE               ), //  4,-5
			   cpv(-BOOTH_HALF_SIZE+LATERAL_WALL_SIZE, -BOOTH_HALF_SIZE				  )  // -4,-5
	   };

	   // right side
	   cpVect tRightSideVertices[] = {
			   cpv( BOOTH_HALF_SIZE-LATERAL_WALL_SIZE, BOOTH_HALF_SIZE), //4, 5
			   cpv( BOOTH_HALF_SIZE					 , BOOTH_HALF_SIZE), //5, 5
			   cpv( BOOTH_HALF_SIZE					 ,-BOOTH_HALF_SIZE), //5,-5
			   cpv( BOOTH_HALF_SIZE-LATERAL_WALL_SIZE,-BOOTH_HALF_SIZE)  //4,-5
	   };

      const CVector3& cPosition = GetEmbodiedEntity().GetPosition();
      CRadians cXAngle, cYAngle, cZAngle;
      GetEmbodiedEntity().GetOrientation().ToEulerAngles(cZAngle, cYAngle, cXAngle);

      /* Manually rotate the vertices */
	  cpVect tRot = cpvforangle(cZAngle.GetValue());
	  //left
	  tLeftSideVertices[0] = cpvrotate(tLeftSideVertices[0], tRot);
	  tLeftSideVertices[1] = cpvrotate(tLeftSideVertices[1], tRot);
	  tLeftSideVertices[2] = cpvrotate(tLeftSideVertices[2], tRot);
	  tLeftSideVertices[3] = cpvrotate(tLeftSideVertices[3], tRot);
	  //back
	  tBackSideVertices[0] = cpvrotate(tBackSideVertices[0], tRot);
	  tBackSideVertices[1] = cpvrotate(tBackSideVertices[1], tRot);
	  tBackSideVertices[2] = cpvrotate(tBackSideVertices[2], tRot);
	  tBackSideVertices[3] = cpvrotate(tBackSideVertices[3], tRot);
	  //right
	  tRightSideVertices[0] = cpvrotate(tRightSideVertices[0], tRot);
	  tRightSideVertices[1] = cpvrotate(tRightSideVertices[1], tRot);
	  tRightSideVertices[2] = cpvrotate(tRightSideVertices[2], tRot);
	  tRightSideVertices[3] = cpvrotate(tRightSideVertices[3], tRot);


      /* Create the geometry */
	  /* The positions are already relative to the center of the entity
	   * so we just need to move them wrt the actual position of the
	   * complete object
	   */

	  //left
	  m_ptLeftSideShape=
			  cpSpaceAddStaticShape(m_cEngine.GetPhysicsSpace(),
					  cpPolyShapeNew(m_cEngine.GetGroundBody(),
							  4,
							  tLeftSideVertices,
							  cpv(cPosition.GetX(), cPosition.GetY())));
	  /* This object is normal */
	  m_ptLeftSideShape->collision_type = CDynamics2DEngine::SHAPE_NORMAL;
	  m_ptLeftSideShape->data = reinterpret_cast<void*>(&c_entity);
	  /* No elasticity */
	  m_ptLeftSideShape->e = 0.0;
	  /* Little contact friction to help sliding away */
	  m_ptLeftSideShape->u = 0.1;

	  //back
	  m_ptBackSideShape=
			  cpSpaceAddStaticShape(m_cEngine.GetPhysicsSpace(),
					  cpPolyShapeNew(m_cEngine.GetGroundBody(),
							  4,
							  tBackSideVertices,
							  cpv(cPosition.GetX(), cPosition.GetY())));
	  /* This object is normal */
	  m_ptBackSideShape->collision_type = CDynamics2DEngine::SHAPE_NORMAL;
	  m_ptBackSideShape->data = reinterpret_cast<void*>(&c_entity);
	  /* No elasticity */
	  m_ptBackSideShape->e = 0.0;
	  /* Little contact friction to help sliding away */
	  m_ptBackSideShape->u = 0.1;

	  //right
	  m_ptRightSideShape =
			  cpSpaceAddStaticShape(m_cEngine.GetPhysicsSpace(),
					  cpPolyShapeNew(m_cEngine.GetGroundBody(),
							  4,
							  tRightSideVertices,
							  cpv(cPosition.GetX(), cPosition.GetY())));
	  /* This object is normal */
	  m_ptRightSideShape->collision_type = CDynamics2DEngine::SHAPE_NORMAL;
	  m_ptRightSideShape->data = reinterpret_cast<void*>(&c_entity);
	  /* No elasticity */
	  m_ptRightSideShape->e = 0.0;
	  /* Little contact friction to help sliding away */
	  m_ptRightSideShape->u = 0.1;
      
   }
   
   /****************************************/
   /****************************************/

   CDynamics2DBoothEntity::~CDynamics2DBoothEntity() {
      cpSpaceRemoveStaticShape(m_cEngine.GetPhysicsSpace(), m_ptLeftSideShape);
      cpSpaceRemoveStaticShape(m_cEngine.GetPhysicsSpace(), m_ptBackSideShape);
      cpSpaceRemoveStaticShape(m_cEngine.GetPhysicsSpace(), m_ptRightSideShape);

      cpSpaceReindexStatic(m_cEngine.GetPhysicsSpace());

      cpShapeFree(m_ptLeftSideShape);
      cpShapeFree(m_ptBackSideShape);
      cpShapeFree(m_ptRightSideShape);


   }

   /****************************************/
   /****************************************/

   bool CDynamics2DBoothEntity::CheckIntersectionWithRay(Real& f_t_on_ray,
                                                       const CRay& c_ray) const {
	   cpSegmentQueryInfo tInfo;
	   bool return_value = false;
	   f_t_on_ray = 1.0f; // set to 1 initially in order to avoid comparisons with 0 (which is always smaller than anything)

	   /* we need to do check all the three sides
	    * and return only at the end because we need
	    * to understand which is the closest side to the
	    * ray starting point. This is necessary to give back
	    * the correct distance. */



	   // check left side
	   if(cpShapeSegmentQuery(m_ptLeftSideShape,
			   cpv(c_ray.GetStart().GetX(), c_ray.GetStart().GetY()),
			   cpv(c_ray.GetEnd().GetX()  , c_ray.GetEnd().GetY()  ),
			   &tInfo)) {
		   CVector3 cIntersectionPoint;
		   c_ray.GetPoint(cIntersectionPoint, tInfo.t);
		   if((cIntersectionPoint.GetZ() >= GetEmbodiedEntity().GetPosition().GetZ()) &&
				   (cIntersectionPoint.GetZ() <= GetEmbodiedEntity().GetPosition().GetZ() + BOOTH_HEIGHT) ) {
			   f_t_on_ray = tInfo.t;
			   return_value = true;
		   }
	   }

	   // check back side
	   if(cpShapeSegmentQuery(m_ptBackSideShape,
			   cpv(c_ray.GetStart().GetX(), c_ray.GetStart().GetY()),
			   cpv(c_ray.GetEnd().GetX()  , c_ray.GetEnd().GetY()  ),
			   &tInfo)) {
		   CVector3 cIntersectionPoint;
		   c_ray.GetPoint(cIntersectionPoint, tInfo.t);
		   if((cIntersectionPoint.GetZ() >= GetEmbodiedEntity().GetPosition().GetZ()) &&
				   (cIntersectionPoint.GetZ() <= GetEmbodiedEntity().GetPosition().GetZ() + BOOTH_HEIGHT) ) {
			   if (tInfo.t < f_t_on_ray)
				   f_t_on_ray = tInfo.t;
			   return_value = true;
		   }
	   }

	   if(cpShapeSegmentQuery(m_ptRightSideShape,
			   cpv(c_ray.GetStart().GetX(), c_ray.GetStart().GetY()),
			   cpv(c_ray.GetEnd().GetX()  , c_ray.GetEnd().GetY()  ),
			   &tInfo)) {
		   CVector3 cIntersectionPoint;
		   c_ray.GetPoint(cIntersectionPoint, tInfo.t);
		   if((cIntersectionPoint.GetZ() >= GetEmbodiedEntity().GetPosition().GetZ()) &&
				   (cIntersectionPoint.GetZ() <= GetEmbodiedEntity().GetPosition().GetZ() + BOOTH_HEIGHT) ) {
			   if (tInfo.t < f_t_on_ray)
			       f_t_on_ray = tInfo.t;
			   return_value = true;
		   }
	   }

	   return return_value;

   }

   /****************************************/
   /****************************************/

   bool CDynamics2DBoothEntity::MoveTo(const CVector3& c_position,
                                         const CQuaternion& c_orientation,
                                         bool b_check_only) {
   /* The booth is not movable, so you can't move it :-) */
	  LOGERR << "[CDynamics2DBoothEntity::MoveTo] WARNING! The booth is currently implemented as not movable only." << std::endl;
      return 1;
   }

   /****************************************/
   /****************************************/

   void CDynamics2DBoothEntity::Reset() {
   }

   /****************************************/
   /****************************************/

   void CDynamics2DBoothEntity::UpdateEntityStatus() {
      /* Update components */
      m_cBoothEntity.UpdateComponents();
   }

   /****************************************/
   /****************************************/

}
