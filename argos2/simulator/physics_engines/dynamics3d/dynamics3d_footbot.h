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
 * @file <argos2/simulator/physics_engines/dynamics3d/dynamics3d_footbot.h>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 * @author Gabriele Valentini - <gvalentini@iridia.ulb.ac.be>
 */

#ifndef DYNAMICS3D_FOOTBOT_H
#define DYNAMICS3D_FOOTBOT_H

namespace argos {
   class CDynamics3DEngine;
   class CDynamics3DFootBot;
}

#include <argos2/simulator/physics_engines/dynamics3d/dynamics3d_entity.h>
#include <argos2/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>
#include <argos2/simulator/space/entities/footbot_entity.h>

namespace argos {

   struct SDynamics3DFootBotODEDebug {
      /* Chassis */
      CVector3    m_cChassisPos;
      Real        m_fTrailOffsetZ;
      Real        m_fBaseOffsetZ;

      /* Wheels */
      CVector3    m_cWheelPos[4];
      CQuaternion m_cWheelRot[4];
      
      /* Turret */
      CVector3    m_cTurretPos;
      CVector3    m_cGripperPos;
   };

   class CDynamics3DFootBot : public CDynamics3DEntity {

   public:
      
      CDynamics3DFootBot(CDynamics3DEngine& c_engine,
                         CFootBotEntity& c_footbot);
      virtual ~CDynamics3DFootBot() {}
      
      virtual void Reset();

      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus();

      void GetODEData(SDynamics3DFootBotODEDebug& s_ode_data) const;

   private:

      void Displace(CVector3 c_pos, CQuaternion c_rot);
      void Debug();
      void TurretPassiveToActive();
      void TurretActiveToPassive();
      void LockTurret();
      void UnlockTurret();
   public:

      CFootBotEntity&               m_cFootBotEntity;
      CWheeledEntity<2>&            m_cWheeledEntity;
      CGripperEquippedEntity&       m_cGripperEntity;

      CVector3                      m_cFromARGoSToChassisCOM;
      CVector3                      m_cFromARGoSToWheelCOM[4];
      CVector3                      m_cFromARGoSToTurretCOM;

      dBodyID                       m_tWheelBody[4];
      dBodyID                       m_tTurretBody; 

      dGeomID                       m_tBaseGeom;
      dGeomID                       m_tChassisGeom;
      dGeomID                       m_tWheelGeom[4];
      dGeomID                       m_tTurretGeom;
      dGeomID                       m_tGripperGeom;

      SDynamics3DEngineGripperData  m_sGripperData;
      SDynamics3DEngineGeomData     m_sGripperGeomData;
      SDynamics3DEngineGeomData     m_sTurretGeomData;

      dJointID                      m_tWheelJoint[4];
      dJointID                      m_tTurretJoint;

      Real                          m_fCurrentWheelVelocityFromSensor[2];
      Real                          m_fPreviousTurretAngleError;
      UInt8                         m_unLastTurretMode;
   };

}

#endif
