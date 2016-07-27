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
 * @file <common/control_interface/ci_wifi_sensor.h>
 *
 * @brief This file provides the definition of the  wifi sensor.
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#ifndef CCI_WIFI_SENSOR_H
#define CCI_WIFI_SENSOR_H

/* To avoid dependency problems when including */
namespace argos {
  class CCI_WiFiSensor;
}

#include <argos2/common/control_interface/ci_sensor.h>
#include <argos2/common/control_interface/ci_wifi_actuator.h>
/* 3D vector definition */
#include <argos2/common/utility/math/vector3.h>
/* Quaternion definition */
#include <argos2/common/utility/math/quaternion.h>

namespace argos {


  /*Struct to hold a wifi message*/
  struct CMessage {
    std::string Sender;
    std::string Recipient;
    std::vector<char> Payload;
    int Delay;
//    size_t PayloadLen;

  CMessage() : Sender("0"),
      Recipient("0"){}

  CMessage(const std::string& str_sender,
     const std::string& str_recipient,
     const std::string& str_payload,
     int f_delay) :
    Sender(str_sender),
      Recipient(str_recipient),
      Delay(f_delay)
      {
  for(unsigned int i=0; (unsigned int)i< str_payload.size(); i++)
    Payload.push_back(str_payload[i]);
//  std::copy( str_payload.begin(), str_payload.end(),
//      Payload.begin()); 
      }
  CMessage(const std::string& str_sender,
     const std::string& str_recipient,
     const char *payload,
     size_t len,
     int f_delay) :
    Sender(str_sender),
      Recipient(str_recipient),
      Delay(f_delay)
      {
  for(unsigned int i=0; (unsigned int)i<len; i++)
    Payload.push_back(payload[i]);
//  std::copy( payload, payload + len, 
//       Payload.begin());
      }
#if 0
  CMessage (const CMessage &m) 
  {
    /// copy the beast
    /// BTW, there's a ugly performance bug in 
    /// GetAllMessages(), which is copying all messages (including their
    /// payloads)
    Sender = m.Sender;
    Recipient = m.Recipient;
    PayloadLen = m.PayloadLen;
    if( PayloadLen )
    {
      Payload = (char*)malloc(PayloadLen);
      memcpy(Payload, m.Payload, PayloadLen);
    }
  }
  CMessage & operator= (const CMessage & other)
  {
    if(&other == this)
      return *this;
    
      printf("copy\n");
      fflush(stdout);
    Sender = other.Sender;
    Recipient = other.Recipient;
    PayloadLen = other.PayloadLen;
    if( Payload != NULL)
      free(Payload);

    printf("about to copy %d\n", PayloadLen);
    if( PayloadLen )
    {
      Payload = (char*)malloc(PayloadLen);
      printf("copying %d\n", PayloadLen);
      fflush(stdout);
      memcpy(Payload, other.Payload, PayloadLen);

      printf("copyed %d\n", PayloadLen);
      fflush(stdout);
    }
    else
      Payload = NULL;
    return *this;
  }
#endif
  ~CMessage()
  {
#if 0
    if( Payload && PayloadLen)
      free(Payload);
#endif
  }
  };


  typedef std::vector<CMessage> TMessageList;



  class CCI_WiFiSensor : virtual public CCI_Sensor {

  public:

    CCI_WiFiSensor() {}
    virtual ~CCI_WiFiSensor() {}

    virtual void GetReceivedMessages(TMessageList& t_messages) = 0;
    virtual void GetReceivedMessages_Local(TMessageList& t_messages) = 0;
    virtual void GetReceivedMessages_Extern(TMessageList& t_messages) = 0;

    /*Added by michal*/
    virtual void GetPositionInfo(CVector3& position) = 0; // Possibility to obtain coordinates via this sensor (thus, may also work as a virtual, ideal GPS-like device)
    virtual void GetOrientationInfo(CQuaternion& orientation) = 0;  // Similarly, obtain the robot's orientation

  };

}

#endif