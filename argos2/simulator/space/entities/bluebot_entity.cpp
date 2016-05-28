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
 * @file <simulator/space/entities/bluebot_entity.cpp>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#include "bluebot_entity.h"
#include "embodied_entity.h"
#include "controllable_entity.h"
#include "led_equipped_entity.h"
#include "gripper_equipped_entity.h"
#include "distance_scanner_equipped_entity.h"
#include "rab_equipped_entity.h"
#include <argos2/common/utility/math/matrix3x3.h>

namespace argos {

   /****************************************/
   /****************************************/

   static const Real BLUEBOT_RADIUS                   = 0.06f;
   static const Real BLUEBOT_HEIGHT                   = 0.10f;

   static const Real BLUEBOT_INTERWHEEL_DISTANCE      = BLUEBOT_RADIUS * 2.0f;
   static const Real BLUEBOT_HALF_INTERWHEEL_DISTANCE = BLUEBOT_INTERWHEEL_DISTANCE * 0.5f;

   /****************************************/
   /****************************************/

   class CBluebotEmbodiedEntity : public CEmbodiedEntity {

   public:

      CBluebotEmbodiedEntity(CBluebotEntity* pc_parent) :
         CEmbodiedEntity(pc_parent) {
         m_cHalfSize.SetX(BLUEBOT_RADIUS);
         m_cHalfSize.SetY(BLUEBOT_RADIUS);
         m_cHalfSize.SetZ(BLUEBOT_HEIGHT * 0.5f);
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

   CBluebotEntity::CBluebotEntity() :
      CComposableEntity(NULL),
      m_pcEmbodiedEntity(new CBluebotEmbodiedEntity(this)),
      m_pcControllableEntity(new CControllableEntity(this)),
      m_pcWheeledEntity(new CWheeledEntity<2>(this)) {
      /* Left wheel position */
      m_pcWheeledEntity->SetWheelPosition(0, CVector3(0.0f,  BLUEBOT_HALF_INTERWHEEL_DISTANCE, 0.0f));
      /* Right wheel position */
      m_pcWheeledEntity->SetWheelPosition(1, CVector3(0.0f, -BLUEBOT_HALF_INTERWHEEL_DISTANCE, 0.0f));
   }

   /****************************************/
   /****************************************/

   CBluebotEntity::~CBluebotEntity() {
      delete m_pcEmbodiedEntity;
      delete m_pcControllableEntity;
      delete m_pcWheeledEntity;
   }

   /****************************************/
   /****************************************/

   void CBluebotEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CEntity::Init(t_tree);
         /* Init components */
         m_pcEmbodiedEntity->Init(t_tree);
         m_pcControllableEntity->Init(t_tree);
         m_pcWheeledEntity->Init(t_tree);
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CBluebotEntity::Reset() {
      /* Reset all components */
      m_pcEmbodiedEntity->Reset();
      m_pcControllableEntity->Reset();
      m_pcWheeledEntity->Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   void CBluebotEntity::Destroy() {
      m_pcEmbodiedEntity->Destroy();
      m_pcControllableEntity->Destroy();
      m_pcWheeledEntity->Destroy();
   }

   /****************************************/
   /****************************************/

   CEntity& CBluebotEntity::GetComponent(const std::string& str_component) {
      if(str_component == "embodied_entity") {
         return *m_pcEmbodiedEntity;
      }
      else if(str_component == "controllable_entity") {
         return *m_pcControllableEntity;
      }
      else if(str_component == "wheeled_entity<2>") {
         return *m_pcWheeledEntity;
      }
      else {
         THROW_ARGOSEXCEPTION("An e-puck does not have a component of type \"" << str_component << "\"");
      }
   }

   /****************************************/
   /****************************************/

   bool CBluebotEntity::HasComponent(const std::string& str_component) {
      return (str_component == "embodied_entity"       ||
              str_component == "controllable_entity"   ||
              str_component == "wheeled_entity<2>");
   }

   /****************************************/
   /****************************************/

   void CBluebotEntity::UpdateComponents() {
      m_pcEmbodiedEntity->UpdateBoundingBox();
   }

   /****************************************/
   /****************************************/
   
   REGISTER_ENTITY(CBluebotEntity,
                   "bluebot",
                   "The bluebot robot.",
                   "Ali Emre Turgut [ali.turgut@gmail.com]",
                   "The bluebot is a simple wheeled robot.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <e-puck id=\"fb0\"\n"
                   "              position=\"0.4,2.3,0.25\"\n"
                   "              orientation=\"45,90,0\"\n"
                   "              controller=\"mycntrl\" />\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "The 'id' attribute is necessary and must be unique among the entities. If two\n"
                   "entities share the same id, initialization aborts.\n"
                   "The 'position' attribute specifies the position of the pucktom point of the\n"
                   "e-puck in the arena. When the robot is untranslated and unrotated, the\n"
                   "pucktom point is in the origin and it is defined as the middle point between\n"
                   "the two wheels on the XY plane and the lowest point of the robot on the Z\n"
                   "axis, that is the point where the wheels touch the floor. The attribute values\n"
                   "are in the X,Y,Z order.\n"
                   "The 'orientation' attribute specifies the orientation of the e-puck. All\n"
                   "rotations are performed with respect to the pucktom point. The order of the\n"
                   "angles is Z,Y,X, which means that the first number corresponds to the rotation\n"
                   "around the Z axis, the second around Y and the last around X. This reflects\n"
                   "the internal convention used in ARGoS, in which rotations are performed in\n"
                   "that order. Angles are expressed in degrees. When the robot is unrotated, it\n"
                   "is oriented along the X axis.\n"
                   "The 'controller' attribute is used to assign a controller to the e-puck. The\n"
                   "value of the attribute must be set to the id of a previously defined\n"
                   "controller. Controllers are defined in the <controllers> XML subtree.\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "None for the time being.\n",
                   "Under development"
      );

}
