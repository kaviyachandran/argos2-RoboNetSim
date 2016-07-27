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
 * @file <argos2/simulator/sensors/wifi_sensor.h>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#ifndef WIFI_SENSOR_EXTERN_H
#define WIFI_SENSOR_EXTERN_H

namespace argos {
  class CWiFiSensorExtern;
  class CControllableEntity;
}

#include <argos2/simulator/sensors/simulated_sensor.h>
#include <argos2/common/control_interface/ci_wifi_sensor.h>
#include <argos2/simulator/space/entities/wifi_equipped_entity.h>
#include <argos2/simulator/space/space.h>

namespace argos {

  class CWiFiSensorExtern : public CSimulatedSensor,
		      public CCI_WiFiSensor {

  public:

    CWiFiSensorExtern();
    virtual ~CWiFiSensorExtern() {}

    virtual void Init(TConfigurationNode& t_tree);

    inline virtual CEntity& GetEntity() {
      return *m_pcEntity;
    }
    virtual void SetEntity(CEntity& c_entity);

    virtual void Update();
    virtual void Reset();

    void GetReceivedMessages(TMessageList& t_messages);
    void GetReceivedMessages_Local(TMessageList& t_messages){}
    void GetReceivedMessages_Extern(TMessageList& t_messages){}
    

    /*Added by michal*/
    void GetPositionInfo(CVector3& position); // Possibility to obtain coordinates via this sensor (thus, may also work as a virtual, ideal GPS-like device)
    virtual void GetOrientationInfo(CQuaternion& orientation);	// Similarly, obtain the robot's orientation

  private:

    CSpace& m_cSpace;
    CEntity* m_pcEntity;
    CWiFiEquippedEntity* m_pcWiFiEquippedEntity;
    /*Added by Marco*/
    TMessageList m_tMessages;
    /*Added bby Michal, not used here*/
    CARGoSRandom::CRNG* m_pcRNG;

  };

  class CWiFiSensorExternNamed : public CWiFiSensorExtern {
  public:
    CWiFiSensorExternNamed()
    {
      CWiFiSensorExtern();
    }
    virtual ~CWiFiSensorExternNamed() {}

    virtual void Init(TConfigurationNode& t_tree)
    {
      CWiFiSensorExtern::Init(t_tree);
    }

    inline virtual CEntity& GetEntity() {
      
      return CWiFiSensorExtern::GetEntity();
    }
    virtual void SetEntity(CEntity& c_entity)
    {
      CWiFiSensorExtern::SetEntity(c_entity);
    }

    virtual void Update()
    {
      CWiFiSensorExtern::Update();
    }
    virtual void Reset()
    {
      CWiFiSensorExtern::Reset();
    }
    

  };

}

#endif
