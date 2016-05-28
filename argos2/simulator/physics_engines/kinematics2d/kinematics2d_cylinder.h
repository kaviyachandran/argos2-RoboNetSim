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
 * @file <argos2/simulator/physics_engines/kinematics2d_cylinder.h>
 *
 * @author Vito Trianni - <vito.trianni@istc.cnr.it>
 */

#ifndef KINEMATICS2D_CYLINDER_H
#define KINEMATICS2D_CYLINDER_H

namespace argos {
  class CKinematics2DEngine;
  class CKinematics2DCylinder;
}

#include <argos2/simulator/physics_engines/kinematics2d/kinematics2d_collision_circle.h>
#include <argos2/simulator/space/entities/cylinder_entity.h>

namespace argos {
  
  class CKinematics2DCylinder : public CKinematics2DCollisionCircle {

  public:
    
    CKinematics2DCylinder(CKinematics2DEngine& c_engine, CCylinderEntity& c_cylinder);
    virtual ~CKinematics2DCylinder() {}
      
  private:

    CCylinderEntity& m_cCylinderEntity;
  };

}

#endif
