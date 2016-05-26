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

#include "wifi_sensor.h"
#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/entities/composable_entity.h>

#include <algorithm>
namespace argos {

  /****************************************/
  /****************************************/

  CWiFiSensor::CWiFiSensor() :
    m_cSpace(CSimulator::GetInstance().GetSpace()),
    m_pcEntity(NULL),
    m_pcWiFiEquippedEntity(NULL),
    ProbRange(0.0f, 1.0f){}

  /****************************************/
  /****************************************/

  void CWiFiSensor::Init(TConfigurationNode& t_tree) {
    /* INITIALIZATION GOES HERE, NOT IN THE CONSTRUCTOR */
    m_tMessages.resize(0);
    /* Random number generator*/
    m_pcRNG = CARGoSRandom::CreateRNG("argos");
  }

  /****************************************/
  /****************************************/

  void CWiFiSensor::SetEntity(CEntity& c_entity) {
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

  void CWiFiSensor::Update() {
    if( m_pcWiFiEquippedEntity->GetRange() != 0.0f){
      UpdateWithRange();
    }else{
      UpdateWithNoRange();
    }
  }

  /****************************************/
  /****************************************/

  void CWiFiSensor:: UpdateWithNoRange()
  {
    /*Start Marco*/
    //std::cerr << "CWiFISensor Update()" << std::endl;
    CSpace::TAnyEntityMap& tEntityMap = m_cSpace.GetEntitiesByType("wifi_equipped_entity");

    TMessageList t_temporaryMsgs;
    for(CSpace::TAnyEntityMap::iterator it = tEntityMap.begin();
	it != tEntityMap.end();
	++it){
      CWiFiEquippedEntity& cWifiEntity = *(any_cast<CWiFiEquippedEntity*>(it->second));
      /*Avoiding self messaging*/
      if(&cWifiEntity != m_pcWiFiEquippedEntity){
	  t_temporaryMsgs = cWifiEntity.GetAllMessages();
	  for(TMessageList::iterator jt = t_temporaryMsgs.begin();jt != t_temporaryMsgs.end();++jt){
	    //std::cerr << jt->Payload << std::endl;
	    if(jt->Recipient == m_pcEntity->GetId() || jt->Recipient == "-1"){
	      m_tMessages.push_back(*jt);
	    }
	  }
      }
    }
  }

  /****************************************/
  /****************************************/

  void  CWiFiSensor::UpdateWithRange()
  {
	  CSpace::TAnyEntityMap& tEntityMap = m_cSpace.GetEntitiesByType("wifi_equipped_entity");

	  /*Get the robot position*/
	  const CVector3& cRobotPosition = m_pcWiFiEquippedEntity->GetPosition();
	  /* Buffer for calculating the message--robot distance */
	  CVector3 cVectorToMessage;
	  CVector3 cVectorRobotToMessage;
	  Real fMessageDistance;
	  TMessageList t_temporaryMsgs;
	  for(CSpace::TAnyEntityMap::iterator it = tEntityMap.begin();
	  it != tEntityMap.end();
	  ++it){
		  CWiFiEquippedEntity& cWifiEntity = *(any_cast<CWiFiEquippedEntity*>(it->second));
		  /*Avoiding self messaging*/
		  if(&cWifiEntity != m_pcWiFiEquippedEntity){
			  cVectorToMessage = cWifiEntity.GetPosition();
			  cVectorRobotToMessage = cVectorToMessage - cRobotPosition;
			  /* Check that the distance is lower than the range */
			  fMessageDistance = cVectorRobotToMessage.Length();
			  //	std::cerr << "Distance is: " << fMessageDistance << std::endl;
			  if(fMessageDistance < m_pcWiFiEquippedEntity->GetRange()){
				  t_temporaryMsgs = cWifiEntity.GetAllMessages();
				  Real probability = m_pcWiFiEquippedEntity->GetProbability();
				  for(TMessageList::iterator jt = t_temporaryMsgs.begin();jt != t_temporaryMsgs.end();++jt){
					  //std::cerr << jt->Payload << std::endl;
					  if(jt->Recipient == m_pcEntity->GetId() || jt->Recipient == "-1"){
						  if ( m_pcRNG->Uniform(ProbRange) < (probability) ) {
							  // Receive with a certain probability of success
							  m_tMessages.push_back(*jt);
						  }
					  }
				  }
			  }
		  }
	  }
  }
  /****************************************/
  /****************************************/

  void CWiFiSensor::Reset() {
    /* RESTORE STATUS OF THE SENSOR TO ITS STATUS RIGHT AFTER Init() WAS CALLED */
  }

  /****************************************/
  /****************************************/

  void CWiFiSensor::GetReceivedMessages(TMessageList& t_messages) {

    /*Resizing the vector, I'm sure to have enough space to store all the messages*/
    t_messages.resize(m_tMessages.size());
    /*Delivering the messages to the caller*/
    std::copy(m_tMessages.begin(),m_tMessages.end(),t_messages.begin());

    /*Remove the delivered up messages*/
    m_tMessages.clear();
  }

  /*Added by michal*/
  void CWiFiSensor::GetPositionInfo(CVector3& position){
	  position = m_pcWiFiEquippedEntity->GetPosition();
  }
  void CWiFiSensor::GetOrientationInfo(CQuaternion& orientation){
	  orientation = m_pcWiFiEquippedEntity->GetOrientation();
  }

  /****************************************/
  /****************************************/
  REGISTER_SENSOR(CWiFiSensor,
		  "wifi", "default",
		  "The Swarmanoid wifi sensor",
		  "Marco Cinus [marco@idsia.ch]",
		  "This sensor accesses the foot-bot wifi sensor. For a complete\n"
		  "description of its usage, refer to the common interface.\n"
		  "In this implementation, a footbot receives all messages from the robots\n"
		  "which are in the transmission range. The Range is setted as a parameter\n"
		  "of the wifi actuator. Refer to it's documentation for more details.\n"
		  "REQUIRED XML CONFIGURATION\n\n"
		  "  <controllers>\n"
		  "    ...\n"
		  "    <my_controller ...>\n"
		  "      ...\n"
		  "      <sensors>\n"
		  "        ...\n"
		  "        <wifi implementation=\"default\" />\n"
		  "        ...\n"
		  "      </sensors>\n"
		  "      ...\n"
		  "    </my_controller>\n"
		  "    ...\n"
		  "  </controllers>\n\n",
		  "UNDER DEVELOPMENT");
}
