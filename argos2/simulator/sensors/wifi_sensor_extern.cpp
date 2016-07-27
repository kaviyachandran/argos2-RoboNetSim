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
 * @file <argos2/simulator/sensors/wifi_sensor.cpp>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#include "wifi_sensor_extern.h"
#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/entities/composable_entity.h>

#include <algorithm>
namespace argos {

  /****************************************/
  /****************************************/

  CWiFiSensorExtern::CWiFiSensorExtern() :
    m_cSpace(CSimulator::GetInstance().GetSpace()),
    m_pcEntity(NULL),
    m_pcWiFiEquippedEntity(NULL) {}

  /****************************************/
  /****************************************/

  void CWiFiSensorExtern::Init(TConfigurationNode& t_tree) {
    /* INITIALIZATION GOES HERE, NOT IN THE CONSTRUCTOR */
    m_tMessages.resize(0);

    /* Random number generator*/
    m_pcRNG = CARGoSRandom::CreateRNG("argos");	// Michal (tricky): just need to initialize it here, as I want to repeat exactly the same randomness as using the 'default' wifi implementation...
  }

  /****************************************/
  /****************************************/

  void CWiFiSensorExtern::SetEntity(CEntity& c_entity) {
    /* Treat the entity as composable */
    CComposableEntity* pcComposableEntity = dynamic_cast<CComposableEntity*>(&c_entity);
    if(pcComposableEntity != NULL) {
      /* The entity is composable, does it have the required component? */
      if(pcComposableEntity->HasComponent("wifi_equipped_entity")) {
	/* Yes, it does */
	m_pcWiFiEquippedEntity = &(pcComposableEntity->GetComponent<CWiFiEquippedEntity>("wifi_equipped_entity"));
	m_pcEntity = &c_entity;
      }
      else {
	/* No, error */
	THROW_ARGOSEXCEPTION("Cannot associate a wifi sensor to an entity of type \"" << c_entity.GetTypeDescription() << "\"");
      }
    }
  }

  /****************************************/
  /****************************************/

  void CWiFiSensorExtern::Update() {
	m_tMessages = m_pcWiFiEquippedEntity->GetAllMessages();
  }

  /****************************************/
  /****************************************/

  void CWiFiSensorExtern::Reset() {
    /* RESTORE STATUS OF THE SENSOR TO ITS STATUS RIGHT AFTER Init() WAS CALLED */
  }

  /****************************************/
  /****************************************/

  void CWiFiSensorExtern::GetReceivedMessages(TMessageList& t_messages) {
    //printf("Retrieving msgs\n");
    //fflush(stdout);

    /*Resizing the vector, I'm sure to have enough space to store all the messages*/
    t_messages.resize(m_tMessages.size());
    /*Delivering the messages to the caller*/
    std::copy(m_tMessages.begin(),m_tMessages.end(),t_messages.begin());

    /*Remove the delivered up messages*/
    m_tMessages.clear();

    //printf("msgs retrieved\n");
    //fflush(stdout);
  }

  /*Added by michal*/
  void CWiFiSensorExtern::GetPositionInfo(CVector3& position){
	  position = m_pcWiFiEquippedEntity->GetPosition();
  }
  void CWiFiSensorExtern::GetOrientationInfo(CQuaternion& orientation){
	  orientation = m_pcWiFiEquippedEntity->GetOrientation();
  }

  /****************************************/
  /****************************************/
  REGISTER_SENSOR(CWiFiSensorExtern,
		  "wifi", "extern",
		  "The Swarmanoid wifi sensor",
		  "Marco Cinus [marco@idsia.ch]",
		  "This sensor accesses the foot-bot wifi sensor. For a complete\n"
		  "description of its usage, refer to the common interface.\n"
		  "In this implementation, a footbot receives all messages which are a unicast\n"
		  "message directed to it or a broadcast message. There's no range implemented\n"
		  "REQUIRED XML CONFIGURATION\n\n"
		  "  <controllers>\n"
		  "    ...\n"
		  "    <my_controller ...>\n"
		  "      ...\n"
		  "      <sensors>\n"
		  "        ...\n"
		  "        <wifi implementation=\"extern\" />\n"
		  "        ...\n"
		  "      </sensors>\n"
		  "      ...\n"
		  "    </my_controller>\n"
		  "    ...\n"
		  "  </controllers>\n\n",
		  "UNDER DEVELOPMENT");
    REGISTER_SENSOR(CWiFiSensorExternNamed,
		  "wifiextern", "default",
		  "The Swarmanoid wifi sensor",
		  "Marco Cinus [marco@idsia.ch]",
		  "This sensor accesses the foot-bot wifi sensor. For a complete\n"
		  "description of its usage, refer to the common interface.\n"
		  "In this implementation, a footbot receives all messages which are a unicast\n"
		  "message directed to it or a broadcast message. There's no range implemented\n"
		  "REQUIRED XML CONFIGURATION\n\n"
		  "  <controllers>\n"
		  "    ...\n"
		  "    <my_controller ...>\n"
		  "      ...\n"
		  "      <sensors>\n"
		  "        ...\n"
		  "        <wifi implementation=\"extern\" />\n"
		  "        ...\n"
		  "      </sensors>\n"
		  "      ...\n"
		  "    </my_controller>\n"
		  "    ...\n"
		  "  </controllers>\n\n",
		  "UNDER DEVELOPMENT");
}
