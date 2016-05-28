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

#include "eyebot_static_pan_and_tilt_camera_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   static const Real EYEBOT_HEIGHT = 0.566f;

   /****************************************/
   /****************************************/

   CEyeBotStaticPanAndTiltCameraSensor::CEyeBotStaticPanAndTiltCameraSensor() :
      m_bEnabled(false),
      m_cSpace(CSimulator::GetInstance().GetSpace()),
      m_cEmbodiedSpaceHash(m_cSpace.GetEmbodiedEntitiesSpaceHash()),
      m_cLEDSpaceHash(m_cSpace.GetLEDEntitiesSpaceHash()),
      m_fApertureSlope(0.0f),
      m_bShowRays(false) {
   }

   /****************************************/
   /****************************************/

   void CEyeBotStaticPanAndTiltCameraSensor::Init(TConfigurationNode& t_tree) {
      try {
         CCI_EyeBotPanAndTiltCameraSensor::Init(t_tree);
         /* Get the aperture */
         CDegrees cAperture;
         GetNodeAttributeOrDefault(t_tree, "aperture", cAperture, CDegrees(80.0f));
         m_cApertureAngle = ToRadians(cAperture);
         m_fApertureSlope = Tan(m_cApertureAngle);
         /* Precalculate the list of cells to analyze */
         CalculateCellsToAnalyze(m_cLEDSpaceHash, m_sLEDSHCellsToVisit, EYEBOT_HEIGHT);
         CalculateCellsToAnalyze(m_cEmbodiedSpaceHash, m_sEmbodiedSHCellsToVisit, EYEBOT_HEIGHT);
         /* Show rays? */
         GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
         /* Blob distance noise std dev */
         GetNodeAttributeOrDefault<Real>(t_tree, "blob_noise_std_dev", m_fBlobDistanceNoiseStdDev, 0.0f);
         if(m_fBlobDistanceNoiseStdDev > 0.0f) {
            m_pcRNG = CARGoSRandom::CreateRNG("argos");
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Initialization error in eye-bot static pan and tilt camera.", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CEyeBotStaticPanAndTiltCameraSensor::Update() {
      /* Compute only if enabled */
      if(m_bEnabled) {
         /* Erase the last detected blobs */
         while(!m_sCameraReadings.BlobList.empty()) {
            delete m_sCameraReadings.BlobList.back();
            m_sCameraReadings.BlobList.pop_back();
         }
         CalculateBlobs();
      }
   }

   /****************************************/
   /****************************************/

   void CEyeBotStaticPanAndTiltCameraSensor::Reset() {
      /* By default, the camera is not enabled */
      Disable();
      /* Reset the sensor updating counter */
      m_sCameraReadings.Counter = 0;
   }

   /****************************************/
   /****************************************/

   void CEyeBotStaticPanAndTiltCameraSensor::Destroy() {
      Disable();
      m_sCameraReadings.Counter = 0;
   }

   /****************************************/
   /****************************************/

   void CEyeBotStaticPanAndTiltCameraSensor::CalculateBlobs() {
      /* Get the position of the eye-bot */
      const CVector3& cEyeBotPosition = GetEntity().GetEmbodiedEntity().GetPosition();
      /* We make the assumption that the eye-bot is rotated only around Z
         and that the camera is static, pointed downwards */
      /* Get the eye-bot orientation */
      CRadians cTmp1, cTmp2, cOrientationZ;
      GetEntity().GetEmbodiedEntity().GetOrientation().ToEulerAngles(cOrientationZ, cTmp1, cTmp2);
      /* Initialize the occlusion check ray start to the bottom of the eye-bot */
      CRay cOcclusionCheckRay;
      cOcclusionCheckRay.SetStart(CVector3(cEyeBotPosition.GetX(), cEyeBotPosition.GetY(), cEyeBotPosition.GetZ()));
      /* Buffer to store the intersection data */
      CSpace::SEntityIntersectionItem<CEmbodiedEntity> sIntersectionData;
      /* Ignore the sensing robot when checking for occlusions */
      TEmbodiedEntitySet tIgnoreEntities;
      tIgnoreEntities.insert(&GetEntity().GetEmbodiedEntity());
      /* Get the center of the eye-bot (in the space hash) on XY */
      SInt32 nCenterI = m_cLEDSpaceHash.SpaceToHashTable(cEyeBotPosition.GetX(), 0);
      SInt32 nCenterJ = m_cLEDSpaceHash.SpaceToHashTable(cEyeBotPosition.GetY(), 1);
      /* Buffer to store the vector connecting the eye-bot to the LED */
      CVector3 cEyeBotToLED;
      Real fLEDDistance;
      /* Buffer for calculating the LED--eye-bot distance */
      CVector2 cLEDDistance;
      /* Go through the precalculated list of cells to visit */
      for(UInt32 unCell = 0; unCell < m_sLEDSHCellsToVisit.size(); ++unCell) {
         /* Get a reference to the current cell to visit */
         SCellCoords& sCell = m_sLEDSHCellsToVisit[unCell];
         /* Clear the list of candidate LEDs */
         TLedEntitySet tVisibleLEDs;
         /* Visit the cell to collect candidate LEDs */
         if(m_cLEDSpaceHash.CheckCell(sCell.I+nCenterI,
                                      sCell.J+nCenterJ,
                                      sCell.K,
                                      tVisibleLEDs)) {
            /* New candidate LEDs found
               Check for occlusions among the candidate elements */
            for(TLedEntitySet::iterator itLED = tVisibleLEDs.begin();
                itLED != tVisibleLEDs.end(); ++itLED) {
               /* Get a reference to this LED */
               CLedEntity& cLED = **itLED;
               /* A robot cannot see its own LEDs, so check to whom they belong */
               if(&cLED.GetParent() != m_pcLEDEntity) {
                  /* The LED is visible if it's not black */
                  if(cLED.GetColor() != CColor::BLACK) {
                     /* Set the end point of the occlusion check ray to the position of the LED
                        Now we have a ray connecting the eye-bot bottom to the candidate LED */
                     cOcclusionCheckRay.SetEnd(cLED.GetPosition());
                     /* The LED is visible if is inside the viewing cone */
                     cOcclusionCheckRay.ToVector(cEyeBotToLED);
                     /* Create the distance vector from the center of the robot to the LED */
                     cLEDDistance.Set(cEyeBotToLED.GetX(),
                                      cEyeBotToLED.GetY());
                     fLEDDistance = cLEDDistance.Length();
                     if(cEyeBotToLED.GetZ() < 0 && Abs(ATan2(fLEDDistance, -cEyeBotToLED.GetZ())) < m_cApertureAngle) {
                        /* Is there an embodied entity along this ray?
                           One is enough, so just tell me closest */
                        if(! m_cSpace.GetClosestEmbodiedEntityIntersectedByRay(sIntersectionData,
                                                                               cOcclusionCheckRay,
                                                                               tIgnoreEntities)) {
                           /* The LED is not obstructed */
                           if(m_bShowRays) GetEntity().GetControllableEntity().AddCheckedRay(false, cOcclusionCheckRay);
                           /* This LED is OK, create a reading for it */
                           /* If noise was setup, add it */
                           if(m_fBlobDistanceNoiseStdDev > 0.0f) {
                              cLEDDistance += CVector2(
                                 m_pcRNG->Gaussian(m_fBlobDistanceNoiseStdDev),
                                 m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE));
                           }
                           /* Add the blob */
                           m_sCameraReadings.BlobList.push_back(
                              new SBlob(
                                 cLED.GetColor(),
                                 (cLEDDistance.Angle() - cOrientationZ).SignedNormalize(),
                                 cLEDDistance.Length() * 100.0f,
                                 0.0f, // TODO: transform cm into pixel
                                 0.0f // TODO: calculate area
                                 ));
                           m_sCameraReadings.Counter++;
                        }
                        else {
                           if(m_bShowRays) {
                              GetEntity().GetControllableEntity().AddCheckedRay(true, cOcclusionCheckRay);
                              GetEntity().GetControllableEntity().AddIntersectionPoint(cOcclusionCheckRay, sIntersectionData.TOnRay);
                           }
                        }
                     }
                  }
               }
            }
         }
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CEyeBotStaticPanAndTiltCameraSensor,
                   "eyebot_pan_and_tilt_camera", "static",
                   "The eye-bot pan_and_tilt camera sensor (static and optimized for 2D)",
                   "Carlo Pinciroli [cpinciro@ulb.ac.be]",
                   "This sensor accesses the eye-bot pan-and-tilt camera sensor. For a\n"
                   "complete description of its usage, refer to the common interface.\n"
                   "In this implementation, the readings are calculated under two assumptions: (i)\n"
                   "the eye-bot is always parallel to the XY plane, i.e., it rotates only around\n"
                   "the Z axis, and (ii) the camera is always pointed towards the ground and\n"
                   "parallel to the Z axis. This implementation is faster than a complete 3D one\n"
                   "and should be used only when the assumptions hold.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <eyebot_pan_and_tilt_camera implementation=\"static\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "It is possible to draw the rays shot by the camera in the OpenGL\n"
                   "visualization. This can be useful for sensor debugging but also to understand\n"
                   "what's wrong in your controller. In OpenGL, the rays are drawn in cyan when\n"
                   "they are not obstructed and in purple when they are. In case a ray is\n"
                   "obstructed, a black dot is drawn where the intersection occurred.\n"
                   "To turn this functionality on, add the attribute 'show_rays=\"true\"' in the\n"
                   "XML as in this example:\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <eyebot_pan_and_tilt_camera implementation=\"static\"\n"
                   "                                    show_rays=\"true\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "In addition, it is possible to tweak the camera aperture. In ARGoS, the camera\n"
                   "aperture is defined as half of the viewing cone tip angle. The real eye-bot\n"
                   "camera has an aperture of 80 degrees. For testing purposes, you can change this\n"
                   "value with the \"aperture\" attribute as in this example (the passed value is in\n"
                   "degrees):\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <eyebot_pan_and_tilt_camera implementation=\"rot_z_only\"\n"
                   "                                    aperture=\"40\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "Finally, you can add noise to the camera readings. Currently, the camera only\n"
                   "returns colored blobs, and you can add noise to their data. Each blob\n"
                   "corresponds to a visible LED in the environment. For each visible LED, the\n"
                   "vector V connecting the LED to the robot is calculated. Noise is a random 3D\n"
                   "vector R added to V. R is calculated as follows: the elevation and azimuth\n"
                   "angles are drawn at random from a uniform distribution over [0:2pi], while the\n"
                   "length is taken from a gaussian distribution with zero mean and a standard\n"
                   "deviation chosen by the user. By default, the standard deviation is zero, which\n"
                   "means that no noise is added. If the standard deviation is set to any value\n"
                   "greater than zero, noise is added to the blob data. This noise impacts on the\n"
                   "distance and angle of each returned blob, but not on its color. Also, currently\n"
                   "all visible LEDs are returned, while on real cameras it may happen that sometimes\n"
                   "a LED does not result in a blob. To add noise to the camera, you must set the\n"
                   "attribute \"blob_noise_std_dev\" as shown:\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <eyebot_pan_and_tilt_camera implementation=\"rot_z_only\"\n"
                   "                                    blob_noise_std_dev=\"1\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n",
                   "Usable"
      );

}
