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

#include "wifi_equipped_entity.h"

#include <argos2/simulator/space/entities/wifi_equipped_entity.h>
#include <argos2/simulator/physics_engines/physics_engine.h>
#include <argos2/simulator/space/space.h>
#include <argos2/simulator/space/entities/wifi_medium_entity.h> /*Contains the function for the socket comunication*/
#include <argos2/common/control_interface/ci_wifi_sensor.h> /*Containts the message definition*/
namespace argos {
  UInt32 CWiFiEquippedEntity::m_unStaticNumericIdCounter = 0;
  TIdConversion CWiFiEquippedEntity::m_tIdConversionMap;

  void CWiFiEquippedEntity::Init(TConfigurationNode& t_tree)
  {
    CEntity::Init(t_tree);
    // seems that every footbot is asssumed to be wifi equipped,
    // here we allow to indicate that the wifi is "disabled"
    bool has_wifi=true;
    if( NodeAttributeExists(t_tree, "wifi_enabled") )
      GetNodeAttribute(t_tree, "wifi_enabled",has_wifi);
    if( has_wifi )
    {
      printf("Node %s has wifi\n", GetId().c_str());
      m_unNumericId = m_unStaticNumericIdCounter++;
      m_tIdConversionMap.insert(TIdConversion::value_type(GetId(), m_unNumericId));
    }
    //GetNodeAttributeOrDefault(t_tree, "wifi_id",WiFiId ,m_unNumericId);
    //m_tIdConversionMap.insert(TIdConversion::value_type(GetId(), m_unNumericId));
    m_tIdConversionMap.insert(TIdConversion::value_type("broadcast",-1));
    m_tIdConversionMap.insert(TIdConversion::value_type("-1",-1));
  }

  /****************************************/
  /****************************************/

  void CWiFiEquippedEntity::SendMessagesOverSocket(const TMessageList& t_message_list)
  {
    char n_numMsg = t_message_list.size();
    ClientSendDataToSocket(m_tSocket, &n_numMsg, 1);
    //char **pMsg;
    char *pMsg = new char[MAX_MSG_SIZE];
    SInt32 nMsgSize;
    for(TMessageList::const_iterator it = t_message_list.begin();
	it != t_message_list.end();
	it++){
      /*Converting the string id from the robots to the numeric Id*/
      SInt32 nSender = m_tIdConversionMap[it->Sender];
      SInt32 nRecipient =  m_tIdConversionMap[it->Recipient];
      ns3Real fCurrentTimeSlice = m_cSpace.GetSimulationClock() / CPhysicsEngine::GetInverseSimulationClockTick();
      /*Build the packet*/
      // printf("Generating packet - payload size %d\n", (int) it->Payload.size());
      //fflush(stdout);
      GenerateMsg(pMsg, nMsgSize, nSender, nRecipient, 
		  &it->Payload[0], it->Payload.size(),
		  fCurrentTimeSlice,it->Delay);
      /*Send the packet*/

      //printf("packet generated msgSize %d\n", nMsgSize);
      //fflush(stdout);
      ClientSendDataToSocket(m_tSocket, pMsg, nMsgSize);
    }
    delete pMsg;
  }

  /****************************************/
  /****************************************/

  void CWiFiEquippedEntity::SendMessagesLocal(const TMessageList& t_message_list)
  {
    /*Resizing the vector the hold the right number of elements*/
    m_tMessages.resize(t_message_list.size());
    /*Perform the copy of the array*/
    std::copy(t_message_list.begin(),t_message_list.end(), m_tMessages.begin());
  }

  /****************************************/
  /****************************************/


  TMessageList CWiFiEquippedEntity::GetMessagesFromSocket(void)
  {
	  TMessageList tReceivedMessages;
	  char *pMsg = new char[MAX_MSG_SIZE];
	  SInt32 nMsgSize;
	  SInt32 nMaxData(MAX_MSG_SIZE);
	  while(ClientReceiveDataFromSocketNonBlocking(m_tSocket, &pMsg, nMsgSize, nMaxData)>0){
		  TApplicationPacket *tAppl = (TApplicationPacket *)pMsg;
		  char *pMessage = pMsg + sizeof(TApplicationPacket);

		  if(tAppl->type != APPLICATION_DATA)
		  {
			  fprintf(stderr, "WRONG PACKET TYPE: %d\n", tAppl->type);
			  //exit(-1);
		  }
		  else
		  {
			  char *pFrom = new char[8];
			  sprintf(pFrom, "%d", tAppl->from);
			  // commented by ef
			  //std::string str_content(pMessage,nMsgSize-sizeof(TApplicationPacket));
//			  CMessage cMsg(pFrom, GetId(), str_content);
//			  // end commend here
			  CMessage cMsg(pFrom, GetId(), pMessage,
					nMsgSize-sizeof(TApplicationPacket));
			  //printf("Received msg of size %d\n", nMsgSize);
			  //fflush(stdout);
			  //std::cerr << cMsg.Sender << " sended: " << cMsg.Payload << " to " << cMsg.Recipient << std::endl;
			  tReceivedMessages.push_back(cMsg);
			  delete pFrom;
		  }
	  }
	  delete pMsg;
	  return tReceivedMessages;
  }

  /****************************************/
  /****************************************/

  TMessageList CWiFiEquippedEntity::GetMessagesLocal(void){
    return m_tMessages;
  }

  /****************************************/
  /****************************************/

  void CWiFiEquippedEntity::SendMessages(const TMessageList& t_message_list)
  {
    switch(m_tMode){
    case STAND_ALONE:
      SendMessagesLocal(t_message_list);
      break;
    case EXTERN:
      SendMessagesOverSocket(t_message_list);
      break;
    default:
      std::cerr << "UNKNOWN MODE FOR WIFI EQUIPPED ENTITY\n";
      break;
    }
  }

  /****************************************/
  /****************************************/

  TMessageList CWiFiEquippedEntity::GetAllMessages(void)
  {
    switch(m_tMode){
    case STAND_ALONE:
      return this->GetMessagesLocal();
    case EXTERN:
      return this->GetMessagesFromSocket();
    default:
      std::cerr << "UNKNOWN MODE FOR WIFI EQUIPPED ENTITY\n";
      break;
    }
    return TMessageList();
  }

  /****************************************/
  /****************************************/

  void CWiFiEquippedEntity::SetMode(TMode const t_mode)
  {
    if(m_bModeSet){
      std::cerr << "WiFiEquippedEntity Mode already setted\n";
    }else{
      m_bModeSet = true;
      m_tMode = t_mode;
      if(m_tMode == EXTERN){
	InitSocketWithExternalApplication();
      }
    }

  }

  /****************************************/
  /****************************************/

  void CWiFiEquippedEntity::InitSocketWithExternalApplication()
  {
    SInt32 n_portBase = 9999; /*Should be acquired from xml or made accessibile through wifi-medium*/
    UInt8 const *punServerHost = (UInt8 const*)"127.0.0.1";/*Should be acquired from xml or made accessibile through wifi-medium*/
    OpenBlockingUDPSocketClientSide(m_tSocket,n_portBase + (m_unNumericId+1), (const char*)punServerHost);
    char *pMsg = new char[MAX_MSG_SIZE];
    SInt32 nMsgSize;
    BuildRobotStartPacket(&pMsg, nMsgSize, m_unNumericId, "WiFi-Equipped-Robot");
    ClientSendDataToSocket(m_tSocket, pMsg, nMsgSize);
  }
}
