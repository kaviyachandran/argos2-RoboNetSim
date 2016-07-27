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

#include "wifi_sensor_dual.h"
#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/entities/composable_entity.h>

#include <algorithm>
namespace argos {

  /****************************************/
  /****************************************/

  CWiFiSensorDual::CWiFiSensorDual() :
    m_cSpace(CSimulator::GetInstance().GetSpace()),
    m_pcEntity(NULL),
    m_pcWiFiEquippedEntity(NULL),
      ProbRange(0.0f, 1.0f){}

  /****************************************/
  /****************************************/

  void CWiFiSensorDual::Init(TConfigurationNode& t_tree) {
    /* INITIALIZATION GOES HERE, NOT IN THE CONSTRUCTOR */
    m_tMessagesLocal.resize(0);
    m_tMessagesExtern.resize(0);

    /* Random number generator*/
    m_pcRNG = CARGoSRandom::CreateRNG("argos");	// Michal (tricky): just need to initialize it here, as I want to repeat exactly the same randomness as using the 'default' wifi implementation...
  }

  /****************************************/
  /****************************************/

  void CWiFiSensorDual::SetEntity(CEntity& c_entity) {
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

  void CWiFiSensorDual::Update()
  {
    /// first, the local messages are caught using the updateWith(No)Range
    if( m_pcWiFiEquippedEntity->GetRange() != 0.0f){
      UpdateWithRange();
    }else{
      UpdateWithNoRange();
    }
    /// then, the extern ones
    m_tMessagesExtern = m_pcWiFiEquippedEntity->GetMessagesFromSocket();
  }

  /****************************************/
  /****************************************/

  void CWiFiSensorDual::Reset() {
    /* RESTORE STATUS OF THE SENSOR TO ITS STATUS RIGHT AFTER Init() WAS CALLED */
  }

  /****************************************/
  /****************************************/

  void CWiFiSensorDual::GetReceivedMessages(TMessageList& t_messages)
  {
    //printf("Retrieving msgs\n");
    //fflush(stdout);

    /*Resizing the vector, I'm sure to have enough space to store all the messages*/
    t_messages.resize(m_tMessagesLocal.size() + m_tMessagesExtern.size());
    /*Delivering the messages to the caller*/
    std::copy(m_tMessagesLocal.begin(),m_tMessagesLocal.end(),
	      t_messages.begin());
    std::copy(m_tMessagesExtern.begin(),m_tMessagesExtern.end(),
	      t_messages.begin() + m_tMessagesLocal.size());
    

    /*Remove the delivered up messages*/
    m_tMessagesLocal.clear();
    m_tMessagesExtern.clear();
    

  }

  void
  CWiFiSensorDual::GetReceivedMessages_Local(TMessageList& t_messages)
  {
    //printf("Retrieving msgs\n");
    //fflush(stdout);

    /*Resizing the vector, I'm sure to have enough space to store all the messages*/
    t_messages.resize(m_tMessagesLocal.size());
    /*Delivering the messages to the caller*/
    std::copy(m_tMessagesLocal.begin(),m_tMessagesLocal.end(),
	      t_messages.begin());

    /*Remove the delivered up messages*/
    m_tMessagesLocal.clear();
   
  }

  void
  CWiFiSensorDual::GetReceivedMessages_Extern(TMessageList& t_messages)
  {
    //printf("Retrieving msgs\n");
    //fflush(stdout);

    /*Resizing the vector, I'm sure to have enough space to store all the messages*/
    t_messages.resize(m_tMessagesExtern.size());
    /*Delivering the messages to the caller*/
    std::copy(m_tMessagesExtern.begin(),m_tMessagesExtern.end(),
	      t_messages.begin());

    /*Remove the delivered up messages*/
    m_tMessagesExtern.clear();
   
  }


  void CWiFiSensorDual:: UpdateWithNoRange()
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
	t_temporaryMsgs = cWifiEntity.GetMessagesLocal();
	for(TMessageList::iterator jt = t_temporaryMsgs.begin();jt != t_temporaryMsgs.end();++jt){
	  //std::cerr << jt->Payload << std::endl;
	  if(jt->Recipient == m_pcEntity->GetId() || jt->Recipient == "-1"){
	    m_tMessagesLocal.push_back(*jt);
	  }
	}
      }
    }
  }

  /****************************************/
  /****************************************/

  void  CWiFiSensorDual::UpdateWithRange()
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
	  t_temporaryMsgs = cWifiEntity.GetMessagesLocal();
	  Real probability = m_pcWiFiEquippedEntity->GetProbability();
	  for(TMessageList::iterator jt = t_temporaryMsgs.begin();jt != t_temporaryMsgs.end();++jt){
	    //std::cerr << jt->Payload << std::endl;
	    if(jt->Recipient == m_pcEntity->GetId() || jt->Recipient == "-1"){
	      if ( m_pcRNG->Uniform(ProbRange) < (probability) ) {
		// Receive with a certain probability of success
		m_tMessagesLocal.push_back(*jt);
	      }
	    }
	  }
	}
      }
    }
  }
  /****************************************/
  /****************************************/


  /*Added by michal*/
  void CWiFiSensorDual::GetPositionInfo(CVector3& position){
	  position = m_pcWiFiEquippedEntity->GetPosition();
  }
  void CWiFiSensorDual::GetOrientationInfo(CQuaternion& orientation){
	  orientation = m_pcWiFiEquippedEntity->GetOrientation();
  }

  /****************************************/
  /****************************************/
  REGISTER_SENSOR(CWiFiSensorDual,
		  "wifi", "dual",
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
