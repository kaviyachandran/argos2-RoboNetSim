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
 * @file <argos2/simulator/actuators/wifi_actuator.h>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#ifndef WIFI_ACTUATOR_H
#define WIFI_ACTUATOR_H

namespace argos {
  class CWiFiActuator;
  class CControllableEntity;
}

#include <argos2/simulator/actuators/simulated_actuator.h>
#include <argos2/common/control_interface/ci_wifi_actuator.h>
#include <argos2/simulator/space/entities/wifi_equipped_entity.h>
#include <argos2/simulator/space/space.h>

#include <argos2/common/control_interface/ci_wifi_sensor.h>


namespace argos {

  class CWiFiActuator : public CSimulatedActuator,
            public CCI_WiFiActuator {

  public:

    CWiFiActuator();
    virtual ~CWiFiActuator() {}

    virtual void Init(TConfigurationNode& t_node);

    inline virtual CEntity& GetEntity() {
      return *m_pcEntity;
    }
    virtual void SetEntity(CEntity& c_entity);

    virtual void Update();
    virtual void Reset();

    virtual void SendMessageTo(const std::string& str_recipient,
                   const std::string& str_payload,
                   int f_delay = 0);

    virtual void SendBinaryMessageTo(const std::string& str_recipient,
                   const char *payload,
                   size_t len,
                     int f_delay = 0);
    virtual void BroadcastMessage(const std::string& str_payload,
                  int f_delay = 0);

    virtual void SendMessageTo_Local(const std::string& str_recipient,
                   const std::string& str_payload,
                   int f_delay = 0)
    {}

    virtual void SendBinaryMessageTo_Local(const std::string& str_recipient,
                   const char *payload,
                   size_t len,
                     int f_delay = 0)
    {}
    virtual void BroadcastMessage_Local(const std::string& str_payload,
                  int f_delay = 0)
    {}
    
    virtual void SendMessageTo_Extern(const std::string& str_recipient,
                   const std::string& str_payload,
                   int f_delay = 0)
    {}

    virtual void SendBinaryMessageTo_Extern(const std::string& str_recipient,
                   const char *payload,
                   size_t len,
                        int f_delay = 0)
    {}
    virtual void BroadcastMessage_Extern(const std::string& str_payload,
                  int f_delay = 0)
    {}


  private:

    CSpace& m_cSpace;
    CEntity* m_pcEntity;
    CWiFiEquippedEntity* m_pcWiFiEquippedEntity;
    Real m_fRange;
    Real m_fProbability;    // added by Michal: a probability of successful transmission in the 'disc probability model'
    //My added stuff (Cinus)
    TMessageList m_tMessages;

    UInt8 tstTempCounter;
    UInt8 tstCounterLimit;
  };

}

#endif