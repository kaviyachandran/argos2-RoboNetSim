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
 * @file <argos2/simulator/actuators/wifi_actuator_ns3.cpp>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#include "wifi_actuator_dual.h"
#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/entities/composable_entity.h>

#include <argos2/simulator/space/entities/wifi_equipped_entity.h>
namespace argos {

   /****************************************/
   /****************************************/

   CWiFiActuatorDual::CWiFiActuatorDual() :
      m_cSpace(CSimulator::GetInstance().GetSpace()),
      m_pcEntity(NULL),
      m_pcWiFiEquippedEntity(NULL) {}

   /****************************************/
   /****************************************/

   void CWiFiActuatorDual::Init(TConfigurationNode& t_node) {
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

   void CWiFiActuatorDual::SetEntity(CEntity& c_entity) {
      /* Treat the entity as composable */
      CComposableEntity* pcComposableEntity = dynamic_cast<CComposableEntity*>(&c_entity);
      if(pcComposableEntity != NULL) {
         /* The entity is composable, does it have the required component? */
         if(pcComposableEntity->HasComponent("wifi_equipped_entity")) {
            /* Yes, it does */
            m_pcWiFiEquippedEntity = &(pcComposableEntity->GetComponent<CWiFiEquippedEntity>("wifi_equipped_entity"));
            m_pcEntity = &c_entity;
	    /* Setting the mode of the underlying wifi_entity */
	    m_pcWiFiEquippedEntity->SetMode(argos::DUAL);
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

   void CWiFiActuatorDual::Update() {
		 /*Clear all the messages that has not yet been sent*/
		 m_pcWiFiEquippedEntity->ClearMessages();
		 /*Deliver the messages to the medium*/
		 /// first the extern messages
		 m_pcWiFiEquippedEntity->SendMessagesOverSocket(m_tMessagesExtern);
		 m_pcWiFiEquippedEntity->SendMessagesLocal(m_tMessagesLocal);
		 /*Once the messages are delivered down delete them*/
		 m_tMessagesExtern.clear();
		 m_tMessagesLocal.clear();

   }

   /****************************************/
   /****************************************/

   void CWiFiActuatorDual::Reset() {
   }

   /****************************************/
   /****************************************/

 // ******* 
 // Added by Eduardo
 // using a string limits the scope of applications
 // !!!! you can not send binary data !!!!

   void CWiFiActuatorDual::
   SendBinaryMessageTo(const std::string& str_recipient,
		       const char *payload,
		       size_t len,
		       int f_delay)
   {
     SendBinaryMessageTo_Local(str_recipient,
			       payload,
			       len,f_delay);

   }
  void CWiFiActuatorDual::
   SendBinaryMessageTo_Local(const std::string& str_recipient,
		       const char *payload,
		       size_t len,
		       int f_delay)
   {

      //printf("sending packet with binary payload\n");
      fflush(stdout);
     /*Build the packet (Cinus)*/
     std::string strSender(m_pcEntity->GetId());
     CMessage tMessage(strSender, str_recipient, payload, len, f_delay);
     /*Put the message in the delivery "queue"*/
     m_tMessagesLocal.push_back(tMessage);

   }
   void CWiFiActuatorDual::
   SendBinaryMessageTo_Extern(const std::string& str_recipient,
		       const char *payload,
		       size_t len,
		       int f_delay)
   {

      //printf("sending packet with binary payload\n");
      fflush(stdout);
     /*Build the packet (Cinus)*/
     std::string strSender(m_pcEntity->GetId());
     CMessage tMessage(strSender, str_recipient, payload, len, f_delay);
     /*Put the message in the delivery "queue"*/
     m_tMessagesExtern.push_back(tMessage);
   }

  
   void CWiFiActuatorDual::SendMessageTo(const std::string& str_recipient,
                                     const std::string& str_payload,
                                     int f_delay)
   {
     SendMessageTo_Local(str_recipient,
			 str_payload,
			 f_delay);

   }

  void CWiFiActuatorDual::SendMessageTo_Local(const std::string& str_recipient,
                                     const std::string& str_payload,
                                     int f_delay) {

      //printf("sending packet with str payload\n");
      fflush(stdout);
     /*Build the packet (Cinus)*/
     std::string strSender(m_pcEntity->GetId());
     CMessage tMessage(strSender, str_recipient,str_payload,f_delay);
     /*Put the message in the delivery "queue"*/
     m_tMessagesLocal.push_back(tMessage);
   }

  void
  CWiFiActuatorDual::
  SendMessageTo_Extern(const std::string& str_recipient,
		      const std::string& str_payload,
		      int f_delay) {

     /*Build the packet (Cinus)*/
     std::string strSender(m_pcEntity->GetId());
     CMessage tMessage(strSender, str_recipient,str_payload,f_delay);
     /*Put the message in the delivery "queue"*/
     m_tMessagesExtern.push_back(tMessage);
   }

   /****************************************/
   /****************************************/

   void
   CWiFiActuatorDual::BroadcastMessage(const std::string& str_payload,
                                        int f_delay)
   {
     BroadcastMessage_Local(str_payload, f_delay);
   }

  
   void
   CWiFiActuatorDual::BroadcastMessage_Local(const std::string& str_payload,
                                        int f_delay) {
     this->SendMessageTo_Local("-1", str_payload);
   }

  void
   CWiFiActuatorDual::BroadcastMessage_Extern(const std::string& str_payload,
					     int f_delay) {
     this->SendMessageTo_Extern("-1", str_payload);
   }


   /****************************************/
   /****************************************/

  /*Registering the Wifi Actuator*/
  REGISTER_ACTUATOR(CWiFiActuatorDual,
		    "wifi","dual",
		    "The wifi actuator",
		    "Marco Cinus [marco@idsia.ch]",
		    "This actuator access the wifi actuator of a wifi equipped entity (only footbots at this time)\n"
		    "For a complete description of its usage refer to the common interface\n"
		    "In this implementation the actuator is sending a message to all wifi-equipped entities\n"
		    "present in the space. Two system are provided for the sender, a broadcast message, which is \n"
		    "received and processed by all entities, and a unicast message, which is processed only by the recipient\n"
		    "REQUIRED XML CONFIGURATION\n\n"
                     "  <controllers>\n"
                     "    ...\n"
                     "    <my_controller ...>\n"
                     "      ...\n"
                     "      <actuators>\n"
                     "        ...\n"
                     "        <wifi implementation=\"extern\" />\n"
                     "        ...\n"
                     "      </actuators>\n"
                     "      ...\n"
                     "    </my_controller>\n"
                     "    ...\n"
                     "  </controllers>\n\n",
		    "UNDER DEVELOPMENT"
		    );

}
