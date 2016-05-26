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
 * @file <argos2/simulator/physics_engines/dynamics3d_eyebot.cpp>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 * @author Gabriele Valentini - <gvalentini@iridia.ulb.ac.be>
 */

#include "dynamics3d_footbot.h"
#include <argos2/simulator/space/entities/embodied_entity.h>
#include <argos2/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

namespace argos {

   /* Definition of useful constants */
   static const Real FB_CHASSIS_MASS                   = 0.4f;
   static const Real FB_CHASSIS_LX                     = 0.1296f; //.1496
   static const Real FB_CHASSIS_LY                     = 0.0916f;
   static const Real FB_CHASSIS_LZ                     = 0.0564f;
   static const Real FB_CHASSIS_OZ                     = 0.02f;

   static const Real FB_BASE_MASS                      = 0.4f;
   static const Real FB_BASE_HEIGHT                    = 0.0055f;
   static const Real FB_BASE_RADIUS                    = 0.085036758f;

   static const Real FB_WHEEL_MASS                     = 0.1f;
   static const Real FB_WHEEL_MAX_FORCE                = 1.0f;
   static const Real FB_WHEEL_RADIUS                   = 0.029f;
   static const Real FB_WHEEL_OZ                       = FB_WHEEL_RADIUS;
   static const Real FB_WHEEL_DISTANCE                 = FB_CHASSIS_LY;

   static const Real FB_TURRET_PD_P_CONSTANT           = 0.4f;
   static const Real FB_TURRET_PD_D_CONSTANT           = 0.2f;
   static const Real FB_TURRET_MASS                    = 0.4f;
   static const Real FB_TURRET_MAX_FORCE               = 0.1f;
   static const Real FB_TURRET_HEIGHT                  = 0.0255f;
   static const Real FB_TURRET_RADIUS                  = FB_BASE_RADIUS;
   static const Real FB_TURRET_OZ                      = FB_CHASSIS_OZ + FB_CHASSIS_LZ + FB_BASE_HEIGHT  
                                                         + 0.5f*FB_TURRET_HEIGHT;

   static const Real FB_GRIPPER_MASS                   = 0.001f;
   static const Real FB_GRIPPER_LX                     = 0.02f;
   static const Real FB_GRIPPER_LY                     = 0.02f;
   static const Real FB_GRIPPER_LZ                     = 0.02f;
   static const Real FB_GRIPPER_OX                     = FB_TURRET_RADIUS;   
   static const Real FB_GRIPPER_OZ                     = FB_CHASSIS_OZ + FB_CHASSIS_LZ + FB_BASE_HEIGHT
                                                         + FB_TURRET_HEIGHT - 0.5f * FB_GRIPPER_LZ;

   enum EFootBotWheels {
      FB_LEFT_WHEEL  = 0,
      FB_RIGHT_WHEEL = 1,
      FB_FRONT_WHEEL = 2,
      FB_BACK_WHEEL  = 3
   };
   
   enum ETurretModes {
      FB_TURRET_OFF,
      FB_TURRET_PASSIVE,
      FB_TURRET_SPEED_CONTROL,
      FB_TURRET_POSITION_CONTROL,
   };


   /****************************************/
   /****************************************/
   CDynamics3DFootBot::CDynamics3DFootBot(CDynamics3DEngine& c_engine,
                                          CFootBotEntity& c_footbot) :
      CDynamics3DEntity(c_engine, c_footbot.GetEmbodiedEntity()),
      m_cFootBotEntity(c_footbot),
      m_cWheeledEntity(m_cFootBotEntity.GetWheeledEntity()),
      m_cGripperEntity(m_cFootBotEntity.GetGripperEquippedEntity()),
      m_sGripperData(m_cFootBotEntity.GetGripperEquippedEntity()),
      m_sGripperGeomData(GEOM_MAGNETIC_GRIPPER, &m_sGripperData),
      m_sTurretGeomData(GEOM_GRIPPABLE),
      m_fPreviousTurretAngleError(0.0f),
      m_unLastTurretMode(m_cFootBotEntity.GetTurretMode()) {

      /* Here we build the foot-bot in the origin (0,0,0) with a null rotation (0,0,0,0). 
       * At the end we call the Displace function to put the foot-bot in the right place in the world. 
       * Every parameter expressed in global coordinates (Joint axis, anchor..) is set here for 
       * clarity but must be setted, according to ARGoS position and orientation, in the Displace
       * function. */     
      UInt8 i;

      /* Chassis */
      dMass tChassisMass;
      dMassSetZero(&tChassisMass);
      dMassSetBoxTotal(&tChassisMass, FB_CHASSIS_MASS, FB_CHASSIS_LX, FB_CHASSIS_LY, FB_CHASSIS_LZ);
      dMassTranslate(&tChassisMass, 0.0f, 0.0f, FB_CHASSIS_OZ + 0.5f * FB_CHASSIS_LZ);
      dMassAdd(&m_tMass, &tChassisMass);
      m_tChassisGeom = dCreateBox(m_tEntitySpace, FB_CHASSIS_LX, FB_CHASSIS_LY, FB_CHASSIS_LZ);
      dGeomSetBody(m_tChassisGeom, m_tBody);

      /* Base */
      dMass tBaseMass;
      dMassSetZero(&tBaseMass);
      dMassSetCylinderTotal(&tBaseMass, FB_BASE_MASS, 3, FB_BASE_RADIUS, FB_BASE_HEIGHT);
      dMassTranslate(&tBaseMass, 0.0f, 0.0f, FB_CHASSIS_OZ + FB_CHASSIS_LZ + 0.5f*FB_BASE_HEIGHT);
      dMassAdd(&m_tMass, &tBaseMass);
      m_tBaseGeom = dCreateCylinder(m_tEntitySpace,FB_BASE_RADIUS, FB_BASE_HEIGHT);
      dGeomSetBody(m_tBaseGeom, m_tBody);

      dGeomSetOffsetPosition(m_tChassisGeom,
                             -m_tMass.c[0],
                             -m_tMass.c[1],
                             FB_CHASSIS_OZ + 0.5f * FB_CHASSIS_LZ - m_tMass.c[2]);
      dGeomSetOffsetPosition(m_tBaseGeom,
                             -m_tMass.c[0],
                             -m_tMass.c[1],
                             FB_CHASSIS_OZ + FB_CHASSIS_LZ + 0.5f * FB_BASE_HEIGHT - m_tMass.c[2]);
      m_cFromARGoSToChassisCOM.Set(m_tMass.c[0], m_tMass.c[1], m_tMass.c[2]);
      dMassTranslate(&m_tMass, -m_tMass.c[0], -m_tMass.c[1], -m_tMass.c[2]);
      dBodySetMass(m_tBody, &m_tMass);

      /* Create wheel bodies */
      dMass tWheelMass;
      m_cFromARGoSToWheelCOM[FB_LEFT_WHEEL ].Set( 0.0f, 0.5f * FB_CHASSIS_LY, FB_WHEEL_OZ);
      m_cFromARGoSToWheelCOM[FB_RIGHT_WHEEL].Set( 0.0f,-0.5f * FB_CHASSIS_LY, FB_WHEEL_OZ);
      m_cFromARGoSToWheelCOM[FB_FRONT_WHEEL].Set( 0.5f * FB_CHASSIS_LX, 0.0f, FB_WHEEL_OZ);
      m_cFromARGoSToWheelCOM[FB_BACK_WHEEL ].Set(-0.5f * FB_CHASSIS_LX, 0.0f, FB_WHEEL_OZ);
      for (i=0; i<4; ++i) {
         dMassSetZero(&tWheelMass);
         m_tWheelBody[i] = dBodyCreate(m_tEngineWorld);
         dMassSetSphereTotal(&tWheelMass, FB_WHEEL_MASS, FB_WHEEL_RADIUS);
         dBodySetMass(m_tWheelBody[i], &tWheelMass);
         m_tWheelGeom[i] = dCreateSphere(m_tEntitySpace, FB_WHEEL_RADIUS);
         dGeomSetBody (m_tWheelGeom[i], m_tWheelBody[i]);
         dBodySetPosition(m_tWheelBody[i],
                          m_cFromARGoSToWheelCOM[i].GetX(),
                          m_cFromARGoSToWheelCOM[i].GetY(), 
                          m_cFromARGoSToWheelCOM[i].GetZ());
      }

      /* Attach the driving wheels to the chassis */
      for (i=0; i<2; ++i) {
         const dReal* pfAnchor = dBodyGetPosition(m_tWheelBody[i]);
         m_tWheelJoint[i] = dJointCreateHinge(m_tEngineWorld, 0);
         dJointAttach(m_tWheelJoint[i], m_tBody, m_tWheelBody[i]);
         dJointSetHingeAnchor(m_tWheelJoint[i], pfAnchor[0], pfAnchor[1], pfAnchor[2]);
         dJointSetHingeAxis(m_tWheelJoint[i], 0.0f, -1.0f, 0.0f);
         dJointSetHingeParam(m_tWheelJoint[i], dParamVel, 0.0f);
         dJointSetHingeParam(m_tWheelJoint[i], dParamFMax, FB_WHEEL_MAX_FORCE);
      }

      /* Attach the support wheels to the chassis. */
      for (i=2; i<4; ++i) {
         const dReal* pfAnchor = dBodyGetPosition(m_tWheelBody[i]);
         m_tWheelJoint[i] = dJointCreateBall(m_tEngineWorld, 0);
         dJointAttach(m_tWheelJoint[i], m_tBody, m_tWheelBody[i]);
         dJointSetBallAnchor(m_tWheelJoint[i], pfAnchor[0], pfAnchor[1], pfAnchor[2]);
      }

      /* Create turret body. */
      dMass tTurretMass, tTurretAndGripperMass;
      dMassSetZero(&tTurretAndGripperMass);
      m_tTurretBody = dBodyCreate(m_tEngineWorld);
      dMassSetZero(&tTurretMass);
      dMassSetCylinderTotal(&tTurretMass, FB_TURRET_MASS, 3, FB_TURRET_RADIUS, FB_TURRET_HEIGHT);
      dMassTranslate(&tTurretMass,0.0f, 0.0f, FB_TURRET_OZ);
      dMassAdd(&tTurretAndGripperMass, &tTurretMass);
      m_tTurretGeom = dCreateCylinder(m_tEntitySpace, FB_TURRET_RADIUS, FB_TURRET_HEIGHT);
      dGeomSetData(m_tTurretGeom, &m_sTurretGeomData);
      dGeomSetBody(m_tTurretGeom, m_tTurretBody);

      /* Add gripper Geom */
      dMass tGripperMass;
      dMassSetZero(&tGripperMass);
      dMassSetBoxTotal(&tGripperMass, FB_GRIPPER_MASS, FB_GRIPPER_LX, FB_GRIPPER_LY, FB_GRIPPER_LZ);
      dMassTranslate(&tGripperMass, FB_GRIPPER_OX, 0.0f, FB_GRIPPER_OZ);
      dMassAdd(&tTurretAndGripperMass, &tGripperMass);
      m_tGripperGeom = dCreateBox(m_tEntitySpace, FB_GRIPPER_LX, FB_GRIPPER_LY, FB_GRIPPER_LZ);
      dGeomSetData(m_tGripperGeom, &m_sGripperGeomData);
      dGeomSetBody(m_tGripperGeom, m_tTurretBody);

      dGeomSetOffsetPosition(m_tTurretGeom,
                             -tTurretAndGripperMass.c[0],
                             -tTurretAndGripperMass.c[1],
                             FB_TURRET_OZ - tTurretAndGripperMass.c[2]);
      dGeomSetOffsetPosition(m_tGripperGeom,
                             FB_GRIPPER_OX - tTurretAndGripperMass.c[0],
                             -tTurretAndGripperMass.c[1],
                             FB_GRIPPER_OZ - tTurretAndGripperMass.c[2]);
      m_cFromARGoSToTurretCOM.Set(tTurretAndGripperMass.c[0], 
                                  tTurretAndGripperMass.c[1], 
                                  tTurretAndGripperMass.c[2]);
      dMassTranslate(&tTurretAndGripperMass, 
                     -tTurretAndGripperMass.c[0], 
                     -tTurretAndGripperMass.c[1], 
                     -tTurretAndGripperMass.c[2]);
      dBodySetMass(m_tTurretBody, &tTurretAndGripperMass);
      dBodySetPosition(m_tTurretBody, 
                       m_cFromARGoSToTurretCOM.GetX(), 
                       m_cFromARGoSToTurretCOM.GetY(), 
                       m_cFromARGoSToTurretCOM.GetZ());
      
      /* Attach the turret to the chassis. */
      dVector3 tAnchor; 
      dBodyGetRelPointPos(m_tTurretBody, 0.0f, 0.0f, - 0.5f*FB_TURRET_HEIGHT, tAnchor);
      m_tTurretJoint = dJointCreateHinge(m_tEngineWorld, 0);
      dJointAttach(m_tTurretJoint, m_tBody, m_tTurretBody);
      dJointSetHingeAnchor(m_tTurretJoint, tAnchor[0], tAnchor[1], tAnchor[2]);
      dJointSetHingeAxis(m_tTurretJoint, 0.0f, 0.0f, 1.0f);
      dJointSetHingeParam(m_tTurretJoint, dParamVel, 0.0f);
      dJointSetHingeParam(m_tTurretJoint, dParamFMax, FB_TURRET_MAX_FORCE); 
      TurretPassiveToActive();
      
      /* Place the foot-bot in the world with the ARGoS position and orientation */
      Displace(GetEmbodiedEntity().GetPosition(), GetEmbodiedEntity().GetOrientation());
   }


   /****************************************/
   /****************************************/

   void CDynamics3DFootBot::Reset() {
      /* Reset body, forces, position, speed */
      CDynamics3DEntity::Reset();
      /* Reset foot-bot specific stuff */
      /* Release gripped objects */
      if(m_cGripperEntity.IsUnlocked() && m_sGripperData.BodyGripped) {
         dJointDestroy(m_sGripperData.GrippingJointID);
         m_sGripperData.BodyGripped = false;
      }
      /* Zero speed and applied forces of wheels */
      dJointSetHingeParam(m_tWheelJoint[FB_LEFT_WHEEL], dParamVel, 0.0f);
      dJointSetHingeParam(m_tWheelJoint[FB_RIGHT_WHEEL],dParamVel, 0.0f);
      /* Zero speed and applied forces of turret */
      dJointSetHingeParam(m_tTurretJoint, dParamVel, 0.0f);            
      /* Switch to turret active mode if needed */
      if(m_unLastTurretMode == FB_TURRET_PASSIVE) {
         TurretPassiveToActive();
      }
      m_unLastTurretMode = FB_TURRET_OFF;
      /* Reset body position and orientation */
      Displace(GetEmbodiedEntity().GetInitPosition(), GetEmbodiedEntity().GetInitOrientation());

      dBodyEnable(m_tBody);
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBot::UpdateFromEntityStatus() {
      /* Get wheel speeds from entity */
      m_cWheeledEntity.GetSpeed(m_fCurrentWheelVelocityFromSensor);
      const Real fChassisAngularVelocity =
         (m_fCurrentWheelVelocityFromSensor[FB_RIGHT_WHEEL] -
          m_fCurrentWheelVelocityFromSensor[FB_LEFT_WHEEL]) / 
         FB_WHEEL_DISTANCE;

      /* Do we want to move? */
      if((m_fCurrentWheelVelocityFromSensor[FB_LEFT_WHEEL] != 0.0f) ||
         (m_fCurrentWheelVelocityFromSensor[FB_RIGHT_WHEEL] != 0.0f)) {
         Real fLeftWheelVelocity, fRightWheelVelocity;
         fLeftWheelVelocity  = m_fCurrentWheelVelocityFromSensor[FB_LEFT_WHEEL] / FB_WHEEL_RADIUS; 
         fRightWheelVelocity = m_fCurrentWheelVelocityFromSensor[FB_RIGHT_WHEEL] / FB_WHEEL_RADIUS;
         dJointSetHingeParam(m_tWheelJoint[FB_LEFT_WHEEL], dParamVel, fLeftWheelVelocity);
         dJointSetHingeParam(m_tWheelJoint[FB_RIGHT_WHEEL],dParamVel, fRightWheelVelocity);
      }
      else {
         /* No, we don't want to move - zero all speeds */
         dJointSetHingeParam(m_tWheelJoint[FB_LEFT_WHEEL], dParamVel, 0.0f);
         dJointSetHingeParam(m_tWheelJoint[FB_RIGHT_WHEEL],dParamVel, 0.0f);
      }

      /* Update turret structures if the state changed state in the last step */
      if(m_cFootBotEntity.GetTurretMode() != m_unLastTurretMode) {
         /* Manage the thing like a state machine */
         switch(m_unLastTurretMode) {
           case FB_TURRET_PASSIVE:
               switch(m_cFootBotEntity.GetTurretMode()) {
                  case FB_TURRET_OFF:
                  case FB_TURRET_POSITION_CONTROL:
                  case FB_TURRET_SPEED_CONTROL:
                     TurretPassiveToActive();
                     break;
                  case FB_TURRET_PASSIVE:
                     break;
               }
               break;
            case FB_TURRET_OFF:
            case FB_TURRET_SPEED_CONTROL:
            case FB_TURRET_POSITION_CONTROL:
               if(m_cFootBotEntity.GetTurretMode() == FB_TURRET_PASSIVE) {
                  TurretActiveToPassive();
               }
               break;
         }
         /* Save the current mode for the next time step */
         m_unLastTurretMode = m_cFootBotEntity.GetTurretMode();
      }

      /* Update the turret data */
      switch(m_unLastTurretMode) {

         /* Position control mode is implemented using a PD controller */
         case FB_TURRET_POSITION_CONTROL: {
            Real fTurretAngularVelocity;
            const dReal* ptOrientation = dBodyGetQuaternion(m_tBody);
            CQuaternion cOrient(ptOrientation[0],
                                ptOrientation[1],
                                ptOrientation[2],
                                ptOrientation[3]);
            CRadians cAngleX, cAngleY, cAngleZ;
            cOrient.ToEulerAngles(cAngleZ, cAngleY, cAngleX);

            Real fActualTurretRot = dJointGetHingeAngle(m_tTurretJoint);
            fTurretAngularVelocity = fChassisAngularVelocity +
               (FB_TURRET_PD_P_CONSTANT * (m_cFootBotEntity.GetTurretRotation().GetValue() - fActualTurretRot) +
                FB_TURRET_PD_D_CONSTANT * (m_cFootBotEntity.GetTurretRotation().GetValue() 
                                           - fActualTurretRot - m_fPreviousTurretAngleError)) *
               m_cEngine.GetInverseSimulationClockTick();
            m_fPreviousTurretAngleError = m_cFootBotEntity.GetTurretRotation().GetValue() - fActualTurretRot;
            
            if(fTurretAngularVelocity == 0.0f) LockTurret();
            else UnlockTurret();
            dJointSetHingeParam(m_tTurretJoint, dParamVel, fTurretAngularVelocity);            
            break;
         }
         case FB_TURRET_SPEED_CONTROL: {
            /* Angular velocity of the turret is given by the rotational speed of the turret 
             * plus the rotational speed of the chassis */
            Real fTurretAngularVelocity;
            fTurretAngularVelocity = fChassisAngularVelocity + m_cFootBotEntity.GetTurretRotationSpeed();
            if(fTurretAngularVelocity == 0.0f) LockTurret();
            else UnlockTurret();
            dJointSetHingeParam(m_tTurretJoint, dParamVel, fTurretAngularVelocity);            
         }            
            break;
         case FB_TURRET_OFF:
         case FB_TURRET_PASSIVE:
            break;
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBot::UpdateEntityStatus() {
      /* Update foot-bot position and orientation */
      dVector3 tChassisCOMToARGoS;
      dBodyGetRelPointPos(m_tBody,
                          -m_cFromARGoSToChassisCOM.GetX(),
                          -m_cFromARGoSToChassisCOM.GetY(),
                          -m_cFromARGoSToChassisCOM.GetZ(),
                          tChassisCOMToARGoS);
      GetEmbodiedEntity().SetPosition(
         CVector3(tChassisCOMToARGoS[0],
                  tChassisCOMToARGoS[1],
                  tChassisCOMToARGoS[2]));
      const dReal* ptOrientation = dBodyGetQuaternion(m_tBody);
      CQuaternion cOrient(ptOrientation[0],
                          ptOrientation[1],
                          ptOrientation[2],
                          ptOrientation[3]);
      GetEmbodiedEntity().SetOrientation(cOrient);
      
      /* Update turret rotation*/
      m_cFootBotEntity.SetTurretRotation(CRadians(dJointGetHingeAngle(m_tTurretJoint)));

      /* Update the components */
      m_cFootBotEntity.UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBot::Displace(CVector3 c_pos, CQuaternion c_rot) {
      /* Displace chassis */
      dQuaternion tQuat = { c_rot.GetW(), c_rot.GetX(), c_rot.GetY(), c_rot.GetZ()};
      CVector3 cFromARGoSToChassisCOMRot = m_cFromARGoSToChassisCOM;       
      cFromARGoSToChassisCOMRot.Rotate(c_rot);
      CVector3 cChassisCOM = c_pos + cFromARGoSToChassisCOMRot;
      dBodySetPosition(m_tBody, cChassisCOM.GetX(), cChassisCOM.GetY(), cChassisCOM.GetZ());
      dBodySetQuaternion(m_tBody, tQuat);

      /* Displace wheels */
      for(UInt8 i=0; i<4; ++i) {
         CVector3 cFromARGoSToWheelCOMRot = m_cFromARGoSToWheelCOM[i];       
         cFromARGoSToWheelCOMRot.Rotate(c_rot);
         CVector3 cWheelCOM = c_pos + cFromARGoSToWheelCOMRot;
         dBodySetPosition(m_tWheelBody[i], cWheelCOM.GetX(), cWheelCOM.GetY(), cWheelCOM.GetZ());
         dBodySetQuaternion(m_tWheelBody[i], tQuat);
         if(i<2) {
            CVector3 cAxis(0.0f, -1.0f, 0.0f);
            cAxis.Rotate(c_rot);
            dJointSetHingeAnchor(m_tWheelJoint[i], cWheelCOM.GetX(), cWheelCOM.GetY(), cWheelCOM.GetZ());
            dJointSetHingeAxis(m_tWheelJoint[i], cAxis.GetX(), cAxis.GetY(), cAxis.GetZ());
         }
         else {
            dJointSetBallAnchor(m_tWheelJoint[i], cWheelCOM.GetX(), cWheelCOM.GetY(), cWheelCOM.GetZ());
         }
      }

      /* Displace turret */
      CVector3 cFromARGoSToTurretCOMRot = m_cFromARGoSToTurretCOM;       
      cFromARGoSToTurretCOMRot.Rotate(c_rot);
      CVector3 cTurretCOM = c_pos + cFromARGoSToTurretCOMRot;
      dBodySetPosition(m_tTurretBody, cTurretCOM.GetX(), cTurretCOM.GetY(), cTurretCOM.GetZ());
      dBodySetQuaternion(m_tTurretBody, tQuat);
      dVector3 tAnchor; 
      dBodyGetRelPointPos(m_tTurretBody, 0.0f, 0.0f, - 0.5f*FB_TURRET_HEIGHT, tAnchor);
      dJointSetHingeAnchor(m_tTurretJoint, tAnchor[0], tAnchor[1], tAnchor[2]);
      CVector3 cAxis(0.0f, 0.0f, 1.0f);
      cAxis.Rotate(c_rot);
      dJointSetHingeAxis(m_tTurretJoint, cAxis.GetX(), cAxis.GetY(), cAxis.GetZ());
      TurretPassiveToActive();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBot::GetODEData(SDynamics3DFootBotODEDebug& s_ode_data) const {
      /* Chassis*/
      s_ode_data.m_cChassisPos.Set(m_cFromARGoSToChassisCOM.GetX(),
                                   m_cFromARGoSToChassisCOM.GetY(),
                                   m_cFromARGoSToChassisCOM.GetZ());
   
      const dReal* ptTmp;
      ptTmp = dGeomGetOffsetPosition(m_tChassisGeom);
      s_ode_data.m_fTrailOffsetZ = ptTmp[2];
      ptTmp = dGeomGetOffsetPosition(m_tBaseGeom);
      s_ode_data.m_fBaseOffsetZ  = ptTmp[2];

      /* Wheels */
      const dReal* ptChassisOrientation = dBodyGetQuaternion(m_tBody);
      for(UInt32 i=0;i<4;++i) {
         const dReal* ptWheelPositionGlobal = dBodyGetPosition(m_tWheelBody[i]);
         dVector3 ptWheelPositionRelative;
         dBodyGetPosRelPoint(m_tBody,
                             ptWheelPositionGlobal[0],
                             ptWheelPositionGlobal[1],
                             ptWheelPositionGlobal[2],
                             ptWheelPositionRelative);
         s_ode_data.m_cWheelPos[i].Set(ptWheelPositionRelative[0], 
                                       ptWheelPositionRelative[1], 
                                       ptWheelPositionRelative[2]);
         s_ode_data.m_cWheelPos[i] += m_cFromARGoSToChassisCOM;
         const dReal* ptWheelOrientation = dBodyGetQuaternion(m_tWheelBody[i]);
         CQuaternion cWheelOrient(ptWheelOrientation[0], 
                                  ptWheelOrientation[1], 
                                  ptWheelOrientation[2], 
                                  ptWheelOrientation[3]);
         CQuaternion cChassisOrient(ptChassisOrientation[0], 
                                    ptChassisOrientation[1], 
                                    ptChassisOrientation[2], 
                                    ptChassisOrientation[3]);
         cChassisOrient.Inverse();
         s_ode_data.m_cWheelRot[i] = cWheelOrient * cChassisOrient;
      }
      
      /* Turret */
      const dReal* ptTurretOrientation = dBodyGetQuaternion(m_tTurretBody);
      CQuaternion cOrient(ptTurretOrientation[0], 
                          ptTurretOrientation[1], 
                          ptTurretOrientation[2], 
                          ptTurretOrientation[3]);
      CRadians cAngleX, cAngleY, cAngleZ;
      cOrient.ToEulerAngles(cAngleZ, cAngleY, cAngleX);


      s_ode_data.m_cTurretPos.Set(m_cFromARGoSToTurretCOM.GetX(),
                                  m_cFromARGoSToTurretCOM.GetY(),
                                  m_cFromARGoSToTurretCOM.GetZ());

      CVector3 cGripPos = CVector3(FB_GRIPPER_OX, 0.0f, FB_GRIPPER_OZ).RotateZ(CRadians(dJointGetHingeAngle(m_tTurretJoint)));
      s_ode_data.m_cGripperPos.Set(cGripPos.GetX(),
                                   cGripPos.GetY(),
                                   cGripPos.GetZ());
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBot::TurretPassiveToActive() {
      LockTurret();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBot::TurretActiveToPassive() {
      UnlockTurret();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBot::LockTurret() {
      dReal tActualAngle;
      
      tActualAngle = dJointGetHingeAngle(m_tTurretJoint);
      UnlockTurret();
      dJointSetHingeParam(m_tTurretJoint, dParamHiStop, tActualAngle + 0.05);
      dJointSetHingeParam(m_tTurretJoint, dParamLoStop, tActualAngle - 0.05);

      // dJointSetHingeParam(m_tTurretJoint, dParamFMax, 0.00001); 

   }

   /****************************************/
   /****************************************/

   void CDynamics3DFootBot::UnlockTurret() {
      dJointSetHingeParam(m_tTurretJoint, dParamHiStop, dInfinity);
      dJointSetHingeParam(m_tTurretJoint, dParamLoStop, -dInfinity);      

      // dJointSetHingeParam(m_tTurretJoint, dParamFMax, 100); 
   }

   /****************************************/
   /****************************************/

}

