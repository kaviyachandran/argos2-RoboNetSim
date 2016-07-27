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

#ifndef WIFI_ACTUATOR_EXTERN_H
#define WIFI_ACTUATOR_EXTERN_H

namespace argos {
  class CWiFiActuatorExtern;
  class CControllableEntity;
  class CSpace;
}

#include <argos2/simulator/actuators/simulated_actuator.h>
#include <argos2/common/control_interface/ci_wifi_actuator.h>
#include <argos2/simulator/space/entities/wifi_equipped_entity.h>
#include <argos2/simulator/space/space.h>

#include <argos2/common/control_interface/ci_wifi_sensor.h>


namespace argos {

  class CWiFiActuatorExtern : public CSimulatedActuator,
			   public CCI_WiFiActuator {

  public:

    CWiFiActuatorExtern();
    virtual ~CWiFiActuatorExtern() {}

    virtual void Init(TConfigurationNode& t_tree);

    inline virtual CEntity& GetEntity() {
      return *m_pcEntity;
    }
    virtual void SetEntity(CEntity& c_entity);

    virtual void Update();
    virtual void Reset();

    void SendBinaryMessageTo(const std::string& str_recipient,
			       const char *payload,
			       size_t len,
			       int f_delay = 0);

    virtual void SendMessageTo(const std::string& str_recipient,
			       const std::string& str_payload,
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

    //My added stuff (Cinus)
    TMessageList m_tMessages;

    UInt8 tstTempCounter;
    UInt8 tstCounterLimit;
  };
  
  class CWiFiActuatorExternNamed : public CWiFiActuatorExtern {
  public:
    CWiFiActuatorExternNamed()
    {
      CWiFiActuatorExtern();
    }
    virtual ~CWiFiActuatorExternNamed() {}

    virtual void Init(TConfigurationNode& t_tree)
    {
      CWiFiActuatorExtern::Init(t_tree);
    }

    inline virtual CEntity& GetEntity() {
      
      return CWiFiActuatorExtern::GetEntity();
    }
    virtual void SetEntity(CEntity& c_entity)
    {
      CWiFiActuatorExtern::SetEntity(c_entity);
    }

    virtual void Update()
    {
      CWiFiActuatorExtern::Update();
    }
    virtual void Reset()
    {
      CWiFiActuatorExtern::Reset();
    }
    

  };
    
}

#endif
