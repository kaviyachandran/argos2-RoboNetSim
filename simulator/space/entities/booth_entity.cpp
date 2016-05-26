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
 * @file <simulator/space/entities/booth.cpp>
 *
 * @author Manuele Brambilla - <mbrambilla@iridia.ulb.ac.be>
 */

#include "booth_entity.h"
#include "embodied_entity.h"
#include "controllable_entity.h"
#include "led_equipped_entity.h"
#include <argos2/common/utility/math/matrix3x3.h>

namespace argos {

   /****************************************/
   /****************************************/

   static const Real BOOTH_WIDTH                    = 0.1f;
   static const Real BOOTH_HEIGHT                   = 0.1f;
   static const Real BOOTH_DEPTH                    = 0.1f;
   static const Real BOOTH_LATERAL_WALL_THICKNESS   = 0.01f;
   static const Real BOOTH_BACK_WALL_THICKNESS      = 0.03f;

   static const Real BOOTH_LED_X            = 0.0f;
   static const Real BOOTH_LED_Y            = -0.019f; // it should be 20, but with 19 it doesn't overlap with the back side
   static const Real BOOTH_LED_Z            = 0.05f;
      
   /****************************************/
   /****************************************/

   class CBoothEmbodiedEntity : public CEmbodiedEntity {

   public:

      CBoothEmbodiedEntity(CBoothEntity* pc_parent) :
         CEmbodiedEntity(pc_parent) {
         m_cHalfSize.SetX(BOOTH_WIDTH  * 0.5f);
         m_cHalfSize.SetY(BOOTH_DEPTH  * 0.5f);
         m_cHalfSize.SetZ(BOOTH_HEIGHT * 0.5f);
      }

   protected:

      virtual void CalculateBoundingBox() {
         m_cCenterPos = GetPosition();
         m_cCenterPos.SetZ(m_cCenterPos.GetZ() + m_cHalfSize.GetZ());
         m_cOrientationMatrix.FromQuaternion(GetOrientation());
         CalculateBoundingBoxFromHalfSize(GetBoundingBox(),
                                          m_cHalfSize,
                                          m_cCenterPos,
                                          m_cOrientationMatrix);
      }

   private:

      CVector3 m_cHalfSize;
      CVector3 m_cCenterPos;
      CMatrix3x3 m_cOrientationMatrix;

   };

   /****************************************/
   /****************************************/

   CBoothEntity::CBoothEntity() :
      CComposableEntity(NULL),
      m_pcEmbodiedEntity(new CBoothEmbodiedEntity(this)),
      m_pcControllableEntity(new CControllableEntity(this)),
      m_pcLEDEquippedEntity(new CLedEquippedEntity(this)) {
      m_pcLEDEquippedEntity->AddLed(CVector3());
   }

   /****************************************/
   /****************************************/

   CBoothEntity::~CBoothEntity() {
      delete m_pcEmbodiedEntity;
      delete m_pcControllableEntity;
      delete m_pcLEDEquippedEntity;
   }

   /****************************************/
   /****************************************/

   void CBoothEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CEntity::Init(t_tree);
         /* Init components */
         m_pcEmbodiedEntity->Init(t_tree);
         m_pcControllableEntity->Init(t_tree);
         m_pcLEDEquippedEntity->Init(t_tree);
         //TODO: check, we set only the leds, we don't need to update the other components
         UpdateComponents();

      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CBoothEntity::Reset() {
      /* Reset all components */
      m_pcEmbodiedEntity->Reset();
      m_pcControllableEntity->Reset();
      m_pcLEDEquippedEntity->Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CBoothEntity::Destroy() {
      m_pcEmbodiedEntity->Destroy();
      m_pcControllableEntity->Destroy();
      m_pcLEDEquippedEntity->Destroy();
   }

   /****************************************/
   /****************************************/

   CEntity& CBoothEntity::GetComponent(const std::string& str_component) {
      if(str_component == "embodied_entity") {
         return *m_pcEmbodiedEntity;
      }
      else if(str_component == "controllable_entity") {
         return *m_pcControllableEntity;
      }
      else if(str_component == "led_equipped_entity") {
         return *m_pcLEDEquippedEntity;
      }
      else {
         THROW_ARGOSEXCEPTION("A booth does not have a component of type \"" << str_component << "\"");
      }
   }

   /****************************************/
   /****************************************/

   bool CBoothEntity::HasComponent(const std::string& str_component) {
      return (str_component == "embodied_entity"       ||
              str_component == "controllable_entity"   ||
              str_component == "led_equipped_entity" );
   }

   /****************************************/
   /****************************************/

   void CBoothEntity::UpdateComponents() {
      SetLedPosition();
      //TODO: are these updates necessary since the booth doesn't move?
      m_pcEmbodiedEntity->UpdateBoundingBox();
   }

   /****************************************/
   /****************************************/
   
   //TODO: check
#define SET_RING_LED_POSITION(IDX)                                          \
   cLEDPosition.Set(BOOTH_LED_X, BOOTH_LED_Y, BOOTH_LED_Z ); \
   cLEDPosition.Rotate(m_pcEmbodiedEntity->GetOrientation());               \
   cLEDPosition += cEntityPosition;                                         \
   m_pcLEDEquippedEntity->SetLedPosition(IDX, cLEDPosition);
   
   void CBoothEntity::SetLedPosition() {
      /* Set LED positions */
      const CVector3& cEntityPosition = GetEmbodiedEntity().GetPosition();
      CVector3 cLEDPosition;
      CRadians cLEDAngle;
      SET_RING_LED_POSITION(0);
   }

   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CBoothEntity,
                   "booth",
                   "The booth entity.",
                   "Manuele Brambilla [mbrambilla@iridia.ulb.ac.be]",
                   "The booth is a simple box that can interact with an e-puck.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <booth id=\"bt0\"\n"
                   "              position=\"0.4,2.3,0.25\"\n"
                   "              orientation=\"45,90,0\"\n"
                   "              controller=\"mycntrl\" />\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "The 'id' attribute is necessary and must be unique among the entities. If two\n"
                   "entities share the same id, initialization aborts.\n"
                   "The 'position' attribute specifies the position of the bottom point of the\n"
                   "booth in the arena. When the booth is untranslated and unrotated, the\n"
                   "pucktom point is in the origin and it is defined as the middle point between\n"
                   "the two walls on the XY plane and the lowest point of the booth on the Z\n"
                   "axis, that is the point where the walls touch the floor. The attribute values\n"
                   "are in the X,Y,Z order.\n"
                   "The 'orientation' attribute specifies the orientation of the booth. All\n"
                   "rotations are performed with respect to the bottom point. The order of the\n"
                   "angles is Z,Y,X, which means that the first number corresponds to the rotation\n"
                   "around the Z axis, the second around Y and the last around X. This reflects\n"
                   "the internal convention used in ARGoS, in which rotations are performed in\n"
                   "that order. Angles are expressed in degrees. When the booth is unrotated, it\n"
                   "is oriented along the X axis.\n"
                   "The 'controller' attribute is used to assign a controller to the booth. The\n"
                   "value of the attribute must be set to the id of a previously defined\n"
                   "controller. Controllers are defined in the <controllers> XML subtree.\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "None for the time being.\n",
                   "Under development"
      );

}
