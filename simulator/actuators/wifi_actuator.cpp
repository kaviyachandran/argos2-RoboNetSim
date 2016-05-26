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
 * @file <argos2/simulator/actuators/wifi_actuator.cpp>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#include "wifi_actuator.h"
#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/entities/composable_entity.h>

#include <argos2/simulator/space/entities/wifi_equipped_entity.h>
namespace argos {

   /****************************************/
   /****************************************/

   CWiFiActuator::CWiFiActuator() :
      m_cSpace(CSimulator::GetInstance().GetSpace()),
      m_pcEntity(NULL),
      m_pcWiFiEquippedEntity(NULL) {}

   /****************************************/
   /****************************************/

   void CWiFiActuator::Init(TConfigurationNode& t_node) {
    try {
      GetNodeAttribute(t_node, "range", m_fRange);
      GetNodeAttribute(t_node, "probability", m_fProbability);
    }
    catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing range and bearing actuator", ex);
    }
   }

   /****************************************/
   /****************************************/

   void CWiFiActuator::SetEntity(CEntity& c_entity) {
      /* Treat the entity as composable */
      CComposableEntity* pcComposableEntity = dynamic_cast<CComposableEntity*>(&c_entity);
      if(pcComposableEntity != NULL) {
         /* The entity is composable, does it have the required component? */
         if(pcComposableEntity->HasComponent("wifi_equipped_entity")) {
            /* Yes, it does */
            m_pcWiFiEquippedEntity = &(pcComposableEntity->GetComponent<CWiFiEquippedEntity>("wifi_equipped_entity"));
            m_pcEntity = &c_entity;
	    /* Setting the mode of the underlying wifi_entity */
	    m_pcWiFiEquippedEntity->SetMode(argos::STAND_ALONE);
	    m_pcWiFiEquippedEntity->SetRange(m_fRange);
	    m_pcWiFiEquippedEntity->SetProbability(m_fProbability);
         }
         else {
            /* No, error */
            THROW_ARGOSEXCEPTION("Cannot associate a wifi actuator to an entity of type \"" << c_entity.GetTypeDescription() << "\"");
         }
      }
   }

   /****************************************/
   /****************************************/

   void CWiFiActuator::Update() {
     /*Clear all the messages that has not yet been sent*/
     m_pcWiFiEquippedEntity->ClearMessages();
     /*Deliver the messages to the medium*/
     m_pcWiFiEquippedEntity->SendMessages(m_tMessages);
     /*Once the messages are delivered down delete them*/
     m_tMessages.clear();
   }

   /****************************************/
   /****************************************/

   void CWiFiActuator::Reset() {
   }

   /****************************************/
   /****************************************/

   void CWiFiActuator::SendMessageTo(const std::string& str_recipient,
                                     const std::string& str_payload,
                                     int f_delay) {

     /*Build the packet (Cinus)*/
     std::string strSender(m_pcEntity->GetId());
     CMessage tMessage(strSender, str_recipient,str_payload,f_delay);
     /*Put the message in the delivery "queue"*/
     m_tMessages.push_back(tMessage);
   }

   /****************************************/
   /****************************************/

   void
   CWiFiActuator::BroadcastMessage(const std::string& str_payload,
                                        int f_delay)
   {
     this->SendMessageTo("-1", str_payload, f_delay);
   }

   /****************************************/
   /****************************************/

  /*Registering the Wifi Actuator*/
  REGISTER_ACTUATOR(CWiFiActuator,
		    "wifi","default",
		    "The wifi actuator",
		    "Marco Cinus [marco@idsia.ch]",
		    "This actuator access the wifi actuator of a wifi equipped entity (only footbots at this time)\n"
		    "For a complete description of its usage refer to the common interface\n"
		    "In this implementation the actuator is sending a message to all wifi-equipped entities\n"
		    "present in the space. Two sendings behaviors are provided for the sender, a broadcast message, which is \n"
		    "received and processed by all entities, and a unicast message, which is processed only by the recipient\n"
		    "There's the possibility to limit the range of the wifi by changing the range attribute\n"
		    "Any real number greater or equal than 0.0f is valid. In case the range is setted to 0.0f\n"
		    "the transmission range is infinte and every wifi-equipped enabled entity will\n"
		    "process the messages.\n"
		    "REQUIRED XML CONFIGURATION\n\n"
                     "  <controllers>\n"
                     "    ...\n"
                     "    <my_controller ...>\n"
                     "      ...\n"
                     "      <actuators>\n"
                     "        ...\n"
                     "        <wifi implementation=\"default\" range=\"0.0f\"/>\n"
                     "        ...\n"
                     "      </actuators>\n"
                     "      ...\n"
                     "    </my_controller>\n"
                     "    ...\n"
                     "  </controllers>\n\n",
		    "UNDER DEVELOPMENT"
		    );
}
