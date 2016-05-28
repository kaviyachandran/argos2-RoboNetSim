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
 * @file <argos2/simulator/sensors/eye-bot/eyebot_static_pan_and_tilt_camera_sensor.h>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#ifndef EYEBOT_STATIC_PAN_AND_TILT_CAMERA_SENSOR_H
#define EYEBOT_STATIC_PAN_AND_TILT_CAMERA_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CEyeBotStaticPanAndTiltCameraSensor;
   class CSpace;
}

#include <argos2/common/control_interface/swarmanoid/eyebot/ci_eyebot_pan_and_tilt_camera_sensor.h>
#include <argos2/simulator/sensors/eye-bot/eyebot_sensor.h>
#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/space.h>
#include <argos2/simulator/space/entities/eyebot_entity.h>
#include <argos2/simulator/space/entities/embodied_entity.h>
#include <argos2/simulator/space/entities/led_equipped_entity.h>
#include <argos2/common/utility/argos_random.h>

namespace argos {

   class CEyeBotStaticPanAndTiltCameraSensor : public CEyeBotSensor,
                                               public CCI_EyeBotPanAndTiltCameraSensor {

   public:

      CEyeBotStaticPanAndTiltCameraSensor();
      virtual ~CEyeBotStaticPanAndTiltCameraSensor() {}

      inline virtual void SetEntity(CEntity& c_entity) {
         CEyeBotSensor::SetEntity(c_entity);
         m_pcLEDEntity = &(GetEntity().GetLEDEquippedEntity());
      }

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();
      virtual void Reset();

      virtual void Destroy();

      inline virtual void Enable() {
         m_bEnabled = true;
      }

      inline virtual void Disable() {
         m_bEnabled = false;
         /* Erase the last detected blobs */
         while(!m_sCameraReadings.BlobList.empty()) {
            delete m_sCameraReadings.BlobList.back();
            m_sCameraReadings.BlobList.pop_back();
         }
      }

   private:

      struct SCellCoords {
         SInt32 I, J, K;

         SCellCoords(SInt32 n_i, SInt32 n_j, SInt32 n_k) :
            I(n_i), J(n_j), K(n_k) {}
      };

      template <typename Element>
      void CalculateCellsToAnalyze(CAbstractSpaceHash<Element>& c_space_hash,
                                   std::vector<SCellCoords>& vec_cell_list,
                                   Real f_height) {
         /* We make the assumption that the rotation is only along Z */
         SInt32 nRangeOnX, nRangeOnY;
         SInt32 nI, nJ;
         Real fHeight;
         SInt32 nCameraElevation =
            c_space_hash.SpaceToHashTable(
               m_cSpace.GetArenaSize().GetZ() - f_height, 2);
         /* Move from the floor to the bottom of the eye-bot */
         for(SInt32 nK = 0; nK <= nCameraElevation; ++nK) {
            /* Calculate the range on the XY plane at height nK */
            fHeight = m_cSpace.GetArenaSize().GetZ() - f_height - c_space_hash.HashTableToSpace(nK, 2);
            nRangeOnX = c_space_hash.SpaceToHashTable(m_fApertureSlope * fHeight, 0);
            nRangeOnY = c_space_hash.SpaceToHashTable(m_fApertureSlope * fHeight, 1);
            /* Go through XY according to the range
               The next for loops sweep a square area.
            */
            for(nJ = -nRangeOnY; nJ <= nRangeOnY; ++nJ) {
               for(nI = -nRangeOnX; nI <= nRangeOnX; ++nI) {
                  /* Add the cell to the list of those to visit */
                  vec_cell_list.push_back(SCellCoords(nI, nJ, nK));
               }
            }
         }
      }

      void CalculateBlobs();

   private:

      bool m_bEnabled;
      CSpace& m_cSpace;
      CSpaceHash<CEmbodiedEntity, CEmbodiedEntitySpaceHashUpdater>& m_cEmbodiedSpaceHash;
      CSpaceHash<CLedEntity, CLEDEntitySpaceHashUpdater>& m_cLEDSpaceHash;
      CLedEquippedEntity* m_pcLEDEntity;
      CRadians m_cApertureAngle;
      Real m_fApertureSlope;
      CARGoSRandom::CRNG* m_pcRNG;
      Real m_fBlobDistanceNoiseStdDev;

      bool m_bShowRays;

      std::vector<SCellCoords> m_sLEDSHCellsToVisit;
      std::vector<SCellCoords> m_sEmbodiedSHCellsToVisit;


   };

}

#endif
