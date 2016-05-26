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
 * @file <argos2/simulator/physics_engines/dynamics2d_booth_entity.h>
 *
 * @author Manuele Brambilla - <mbrambilla@iridia.ulb.ac.be>
 */

#ifndef DYNAMICS2D_BOOTH_ENTITY_H
#define DYNAMICS2D_BOOTH_ENTITY_H

#include <argos2/simulator/physics_engines/dynamics2d/dynamics2d_entity.h>
#include <argos2/simulator/space/entities/booth_entity.h>

namespace argos {

   class CDynamics2DBoothEntity : public CDynamics2DEntity {

   public:

      CDynamics2DBoothEntity(CDynamics2DEngine& c_engine,
                           CBoothEntity& c_entity);
      virtual ~CDynamics2DBoothEntity();

      virtual bool CheckIntersectionWithRay(Real& f_t_on_ray,
                                            const CRay& c_ray) const;

      virtual bool MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation,
                          bool b_check_only = false);

      virtual void Reset();

      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus() {}

   private:

      cpFloat  m_fMass;
      CBoothEntity& m_cBoothEntity;
      Real     m_fHalfHeight;
      cpShape* m_ptLeftSideShape;
      cpShape* m_ptBackSideShape;
      cpShape* m_ptRightSideShape;
      cpConstraint* m_ptLinearFriction;
      cpConstraint* m_ptAngularFriction;

   };

}

#endif
