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
 * @file <argos2/simulator/physics_engines/kinematics2d/kinematics2d_collision_rectangle.cpp>
 *
 * @author Vito Trianni - <vito.trianni@istc.cnr.it>
 */

#include "kinematics2d_collision_rectangle.h"
#include "kinematics2d_engine.h"
#include <argos2/common/utility/math/segment.h>
#include <argos2/common/utility/math/ray.h>
#include <argos2/simulator/space/entities/embodied_entity.h>

namespace argos {

  /****************************************/
  /****************************************/

  CKinematics2DCollisionRectangle::CKinematics2DCollisionRectangle(CKinematics2DEngine& c_engine, CEmbodiedEntity& c_entity, const CVector3& c_half_size) :
    CKinematics2DEntity(c_engine, c_entity),
    m_cHalfSize(c_half_size.ProjectOntoXY()/2) {
    m_bEnabled = false;
    m_tCollisionType = KINEMATICS2D_COLLISION_RECTANGLE;

    m_pcVertices[0] = CVector2( m_cHalfSize.GetX(), m_cHalfSize.GetY());
    m_pcVertices[1] = CVector2(-m_cHalfSize.GetX(), m_cHalfSize.GetY());
    m_pcVertices[2] = CVector2(-m_cHalfSize.GetX(),-m_cHalfSize.GetY());
    m_pcVertices[3] = CVector2( m_cHalfSize.GetX(),-m_cHalfSize.GetY());

    CMatrix2x2 rotation;
    rotation.FromAngle(m_cOrientation);

    m_pcVertices[0] = rotation*m_pcVertices[0] + m_cPosition;
    m_pcVertices[1] = rotation*m_pcVertices[1] + m_cPosition;
    m_pcVertices[2] = rotation*m_pcVertices[2] + m_cPosition;
    m_pcVertices[3] = rotation*m_pcVertices[3] + m_cPosition;
  }


  /****************************************/
  /****************************************/

  bool CKinematics2DCollisionRectangle::CheckIntersectionWithRay(Real& f_distance, const CRay& c_ray) const {
    CSegment c_segment = c_ray.ProjectOntoXY();
    Real inv_segment_length = 1/c_segment.GetLength();
    Real min_distance = (c_segment.GetStart() - m_cPosition).Length()*inv_segment_length;
    bool b_intersect = false;
    for( UInt32 i = 0; i < 4; i++ ) {
      CSegment c_side(m_pcVertices[i], m_pcVertices[(i+1)%4]);

      CVector2 c_intersect;
      if( c_segment.Intersect(c_side, c_intersect) ) {
	min_distance = Min(min_distance, (c_segment.GetStart() - c_intersect).Length()*inv_segment_length);
	b_intersect = true;
      }
    }

    if( b_intersect ) f_distance = min_distance;
    return b_intersect;
  }
  
  /****************************************/
  /****************************************/
  
  bool CKinematics2DCollisionRectangle::MoveTo(const CVector3& c_position, const CQuaternion& c_orientation, bool b_check_only ) {
    CKinematics2DEntity::MoveTo(c_position, c_orientation, b_check_only );
    
    m_pcVertices[0] = CVector2( m_cHalfSize.GetX(), m_cHalfSize.GetY());
    m_pcVertices[1] = CVector2(-m_cHalfSize.GetX(), m_cHalfSize.GetY());
    m_pcVertices[2] = CVector2(-m_cHalfSize.GetX(),-m_cHalfSize.GetY());
    m_pcVertices[3] = CVector2( m_cHalfSize.GetX(),-m_cHalfSize.GetY());

    CMatrix2x2 rotation;
    rotation.FromAngle(m_cOrientation);

    m_pcVertices[0] = rotation*m_pcVertices[0] + m_cPosition;
    m_pcVertices[1] = rotation*m_pcVertices[1] + m_cPosition;
    m_pcVertices[2] = rotation*m_pcVertices[2] + m_cPosition;
    m_pcVertices[3] = rotation*m_pcVertices[3] + m_cPosition;

    return m_cEngine.CollisionsDetection(this);
  }


  /****************************************/
  /****************************************/
  
  void CKinematics2DCollisionRectangle::Reset() {
    CKinematics2DEntity::Reset();
    
    m_pcVertices[0] = CVector2( m_cHalfSize.GetX(), m_cHalfSize.GetY());
    m_pcVertices[1] = CVector2(-m_cHalfSize.GetX(), m_cHalfSize.GetY());
    m_pcVertices[2] = CVector2(-m_cHalfSize.GetX(),-m_cHalfSize.GetY());
    m_pcVertices[3] = CVector2( m_cHalfSize.GetX(),-m_cHalfSize.GetY());

    CMatrix2x2 rotation;
    rotation.FromAngle(m_cOrientation);

    m_pcVertices[0] = rotation*m_pcVertices[0] + m_cPosition;
    m_pcVertices[1] = rotation*m_pcVertices[1] + m_cPosition;
    m_pcVertices[2] = rotation*m_pcVertices[2] + m_cPosition;
    m_pcVertices[3] = rotation*m_pcVertices[3] + m_cPosition;
  }


  /****************************************/
  /****************************************/

}
