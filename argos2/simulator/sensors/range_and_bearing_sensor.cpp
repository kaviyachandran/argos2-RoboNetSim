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
 * @file <argos2/simulator/sensors/foot-bot/footbot_range_and_bearing_sensor.cpp>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#include "range_and_bearing_sensor.h"
#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/entities/composable_entity.h>
#include <argos2/common/utility/math/angles.h>

namespace argos {

   /****************************************/
   /****************************************/

   CRangeAndBearingSensor::CRangeAndBearingSensor() :
      m_cSpace(CSimulator::GetInstance().GetSpace()),
      m_cEmbodiedSpaceHash(m_cSpace.GetEmbodiedEntitiesSpaceHash()),
      m_cRABEquippedSpaceHash(m_cSpace.GetRABEquippedEntitiesSpaceHash()),
      m_pcEntity(NULL),
      m_pcRABEquippedEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcRNG(NULL),
      m_bShowRays(false),
      m_bCheckOcclusions(true){}

   /****************************************/
   /****************************************/

   void CRangeAndBearingSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_RangeAndBearingSensor::Init(t_tree);
         /* Show rays? */
         GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
         GetNodeAttributeOrDefault(t_tree, "check_occlusions", m_bCheckOcclusions, m_bCheckOcclusions);
         /* Distance noise std dev */
         GetNodeAttributeOrDefault<Real>(t_tree, "noise_std_dev", m_fDistanceNoiseStdDev, 0.0f);
         if(m_fDistanceNoiseStdDev > 0.0f) {
            m_pcRNG = CARGoSRandom::CreateRNG("argos");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in foot-bot range and bearing sensor.", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CRangeAndBearingSensor::SetEntity(CEntity& c_entity) {
      /* Treat the entity as composable */
      CComposableEntity* pcComposableEntity = dynamic_cast<CComposableEntity*>(&c_entity);
      if(pcComposableEntity != NULL) {
         /* The entity is composable, does it have the required components? */
         if(pcComposableEntity->HasComponent("rab_equipped_entity") &&
            pcComposableEntity->HasComponent("embodied_entity") &&
            pcComposableEntity->HasComponent("controllable_entity")) {
            /* Yes, it does */
            m_pcRABEquippedEntity = &(pcComposableEntity->GetComponent<CRABEquippedEntity>("rab_equipped_entity"));
            m_pcEmbodiedEntity = &(pcComposableEntity->GetComponent<CEmbodiedEntity>("embodied_entity"));
            m_pcControllableEntity = &(pcComposableEntity->GetComponent<CControllableEntity>("controllable_entity"));
            m_pcEntity = &c_entity;
         }
         else {
            /* No, error */
            THROW_ARGOSEXCEPTION("Cannot associate a range and bearing sensor to an entity of type \"" << c_entity.GetTypeDescription() << "\"");
         }
      }
   }

   /****************************************/
   /****************************************/

   void CRangeAndBearingSensor::Update() {
      /* Get robot position */
      const CVector3& cRobotPosition = m_pcEmbodiedEntity->GetPosition();
      /* Get robot orientation */
      CRadians cTmp1, cTmp2, cOrientationZ;
      m_pcEmbodiedEntity->GetOrientation().ToEulerAngles(cOrientationZ, cTmp1, cTmp2);
      /* Buffer for calculating the message--robot distance */
      CVector3 cVectorToMessage;
      CVector3 cVectorRobotToMessage;
      Real fMessageDistance;
      /* Buffer for the received packet */
      TRangeAndBearingReceivedPacket tPacket;
      /* Initialize the occlusion check ray start to the position of the robot */
      CRay cOcclusionCheckRay;
      cOcclusionCheckRay.SetStart(cRobotPosition);
      /* Buffer to store the intersection data */
      CSpace::SEntityIntersectionItem<CEmbodiedEntity> sIntersectionData;
      /* Ignore the sensing robot when checking for occlusions */
      TEmbodiedEntitySet tIgnoreEntities;
      tIgnoreEntities.insert(m_pcEmbodiedEntity);
      /* Buffer to store the list of RAB entitities to check */
      TRABEquippedEntitySet tRABEntities;
      /*
       * 1. Go through the RAB entities in the buckets of the space hash covered by the bounding box of the robot
       * 2. Then, for each of them found:
       *    a) check that the RAB entity is compatible with that of this robot
       *    b) Check if there is an occlusion
       *    c) If there isn't, get the info and set reading for that robot
       */
      CEmbodiedEntity::SBoundingBox& sBBox = m_pcEmbodiedEntity->GetBoundingBox();
      SInt32 nBBoxMinI, nBBoxMinJ, nBBoxMinK;
      SInt32 nBBoxMaxI, nBBoxMaxJ, nBBoxMaxK;
      m_cRABEquippedSpaceHash.SpaceToHashTable(nBBoxMinI, nBBoxMinJ, nBBoxMinK, sBBox.MinCorner);
      m_cRABEquippedSpaceHash.SpaceToHashTable(nBBoxMaxI, nBBoxMaxJ, nBBoxMaxK, sBBox.MaxCorner);
      for(SInt32 i = nBBoxMinI; i <= nBBoxMaxI; ++i) {         
         for(SInt32 j = nBBoxMinJ; j <= nBBoxMaxJ; ++j) {         
            for(SInt32 k = nBBoxMinK; k <= nBBoxMaxK; ++k) {
               m_cRABEquippedSpaceHash.CheckCell(i, j, k, tRABEntities);
            }
         }
      }
      /* Go through the items */
      for(TRABEquippedEntitySet::iterator it = tRABEntities.begin();
          it != tRABEntities.end();
          ++it) {
         /* Create a reference to the RAB entity to check */
         CRABEquippedEntity& cRABEntity = **it;
         /* Check the RAB equipped entity is not this robot (avoid self-messaging) */
         if((&cRABEntity != m_pcRABEquippedEntity) &&
            (cRABEntity.GetMsgSize() == m_pcRABEquippedEntity->GetMsgSize())) {
            /*
             * The message is considered received if:
             * 1. the source of the message is not beyond the range;
             * 2. there are no occlusions between the message source and this RAB entity.
             */
            cVectorToMessage = cRABEntity.GetPosition();
            cVectorRobotToMessage = cVectorToMessage;
            cVectorRobotToMessage -= cRobotPosition;
            /* If noise was setup, add it */
            if(m_fDistanceNoiseStdDev > 0.0f) {
               cVectorRobotToMessage += CVector3(
                  m_pcRNG->Gaussian(m_fDistanceNoiseStdDev),
                  m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                  m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE));
            }
            /* Check that the distance is lower than the range */
            fMessageDistance = cVectorRobotToMessage.Length();
            if(fMessageDistance < cRABEntity.GetRange()) {
               /* Set the ray end */
               cOcclusionCheckRay.SetEnd(cVectorToMessage);
               /* Check occlusion between robot and message location */
               if((! m_bCheckOcclusions) ||
                  (! m_cSpace.GetClosestEmbodiedEntityIntersectedByRay(sIntersectionData,
                                                                       cOcclusionCheckRay,
                                                                       tIgnoreEntities)) ||
                  (cRABEntity.GetId() == sIntersectionData.IntersectedEntity->GetId())
                  ) {
                  /* The message is not occluded */
                  if(m_bShowRays) m_pcControllableEntity->AddCheckedRay(false, cOcclusionCheckRay);
                  /* Set the reading */
                  tPacket.RobotId = cRABEntity.GetId();
                  tPacket.Id = ++m_unLatestPacketId;
                  cVectorRobotToMessage.ToSphericalCoordsHorizontal(tPacket.Range,
                                                                    tPacket.BearingVertical,
                                                                    tPacket.BearingHorizontal);
                  /* Convert the reading to cm */
                  tPacket.Range *= 100.0f;
                  tPacket.BearingHorizontal -= cOrientationZ;
                  tPacket.BearingHorizontal.SignedNormalize();
                  tPacket.BearingVertical.SignedNormalize();
                  cRABEntity.GetData(tPacket.Data);
                  m_tLastReceivedPackets[cRABEntity.GetId()] = tPacket;
               }
               else {
                  /* The message is occluded */
                  if(m_bShowRays) {
                     m_pcControllableEntity->AddCheckedRay(true, cOcclusionCheckRay);
                     m_pcControllableEntity->AddIntersectionPoint(cOcclusionCheckRay, sIntersectionData.TOnRay);
                  }
               }
            }
         }
      }
   }

   /****************************************/
   /****************************************/

   void CRangeAndBearingSensor::Reset() {
      m_unLatestPacketId = 0;
      ClearRABReceivedPackets();
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CRangeAndBearingSensor,
                   "range_and_bearing", "default",
                   "The Swarmanoid range and bearing sensor",
                   "Carlo Pinciroli [cpinciro@ulb.ac.be]",
                   "This sensor accesses the foot-bot range and bearing sensor. For a complete\n"
                   "description of its usage, refer to the common interface.\n"
                   "In this implementation, the readings are calculated under the assumption that\n"
                   "the foot-bot is always parallel to the XY plane, i.e., it rotates only around\n"
                   "the Z axis. This implementation is faster than a 3D one and should be used\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <range_and_bearing implementation=\"default\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "It is possible to draw the rays shot by the distance scanner in the OpenGL\n"
                   "visualization. This can be useful for sensor debugging but also to understand\n"
                   "what's wrong in your controller. In OpenGL, the rays are drawn in cyan when\n"
                   "they are not obstructed and in purple when they are. In case a ray is\n"
                   "obstructed, a black dot is drawn where the intersection occurred.\n"
                   "Additionally, the occlusion check for the range and bearing can be disabled\n"
                   "completely. This is useful, for example, in collective transport, where you\n"
                   "want the robot to communicate as it happens in reality (robots are very close\n"
                   "and hence the IR beam is always received by some of the sensors) but the\n"
                   "simulation would discard messages because occlusions are computed.\n"
                   "The show_rays functionality is off by default, whereas the check occlusion\n"
                   "is on. To switch on or off these functionalities, add the attribute\n"
                   "'show_rays=\"true\"' and/or 'check_occlusions=\"false\"' in the XML\n"
                   "as in this example:\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <range_and_bearing implementation=\"default\"\n"
                   "                           show_rays=\"true\"\n"
                   "                           check_occlusions=\"false\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "Finally, you can add noise to the readings. Each reading corresponds to a\n"
                   "unobstructed RAB device within the communication range. For each of these\n"
                   "devices, the vector V connecting the device to the robot is calculated. Noise is\n"
                   "a random 3D vector R added to V. R is calculated as follows: the elevation and\n"
                   "azimuth angles are drawn at random from a uniform distribution over [0:2pi],\n"
                   "while the length is taken from a gaussian distribution with zero mean and a\n"
                   "standard deviation chosen by the user. By default, the standard deviation is\n"
                   "zero, which means that no noise is added. If the standard deviation is set to\n"
                   "any value greater than zero, noise is added to the readings. This noise impacts\n"
                   "on the distance and angle of each returned reading, but not on its playload. To\n"
                   "add noise to the sensor, you must set the attribute \"noise_std_dev\" as shown:\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <range_and_bearing implementation=\"default\"\n"
                   "                           noise_std_dev=\"1\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n",
                   "Usable"
      );

}
