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
 * @file <argos2/simulator/space/entities/wifi_equipped_entity.h>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#ifndef WiFi_EQUIPPED_ENTITY_H
#define WiFi_EQUIPPED_ENTITY_H


#include <argos2/simulator/space/entities/entity.h>
#include <argos2/common/utility/math/vector3.h>
#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/space.h>
#include <argos2/common/control_interface/ci_wifi_sensor.h> /*Contains the CMessage definition*/
#include <argos2/simulator/space/entities/wifi_medium_entity.h> /*Contains the function for the socket comunication*/

#include <algorithm>
#include <map>

namespace argos {

  typedef enum {
    STAND_ALONE,
    EXTERN,
    DUAL,
  } TMode;

  typedef std::map<std::string, SInt32> TIdConversion;

  class CWiFiEquippedEntity : public CEntity {

  public:

  CWiFiEquippedEntity(CEntity* pc_parent) : CEntity(pc_parent),m_cSpace(CSimulator::GetInstance().GetSpace()),m_fRange(0.0f) {
      //m_unNumericId = m_unStaticNumericIdCounter++;
      m_bModeSet = false;/*Added by Marco*/
      m_tMode = STAND_ALONE;
      m_externInitialized = false;
    }

    virtual ~CWiFiEquippedEntity() {}
    virtual void Init(TConfigurationNode& t_tree); //Added by Marco
    virtual void Reset() {}
    virtual void Destroy() {
    	m_unStaticNumericIdCounter--;
    }

    inline virtual void Accept(CEntityVisitor& visitor) {
      visitor.Visit(*this);
    }

    inline virtual std::string GetTypeDescription() const {
      return "wifi_equipped_entity";
    }

    /*Added by marco*/
    inline const CVector3& GetPosition() const {
      return m_cPosition;
    }
    inline void SetPosition(const CVector3& c_position) {
      m_cPosition = c_position;
    }
    /*Added by michal*/
    inline const CQuaternion& GetOrientation() const {
      return m_cOrientation;
    }
    inline void SetOrientation(const CQuaternion& c_orientation) {
    	m_cOrientation = c_orientation;
    }

    void SendMessages(const TMessageList& t_message_list);
    TMessageList GetAllMessages(void);

    void ClearMessages(void)
    {
      m_tMessages.clear();
    }

    inline const TMode GetMode() const 
    {
      return m_tMode;
    }
    void SetMode(TMode const t_mode);

    inline UInt32 GetNumericId(void) const {
      return m_unNumericId;
    }
    inline Real GetRange() const {
      return m_fRange;
    }

    inline void SetRange(Real f_range) {
      m_fRange = f_range;
    }

    inline Real GetProbability() const {
      return m_fProbability;
    }

    inline void SetProbability(Real f_prob) {
    	m_fProbability = f_prob;
    }

    inline static SInt32 GetNumericId(std::string str_id){
	    return m_tIdConversionMap[str_id];
    }
    /*There are 2 implementation of the entity... 1 is stand alone, the other is
      collegated with the ns3 network simulator, for the end user it will always call
      the same function SendMessages but depending on the implementation type one there'll
      be a dispatching to the right function*/
    void SendMessagesOverSocket(const TMessageList& t_message_list);
    void SendMessagesLocal(const TMessageList& t_message_list);
    /*There are 2 implementation of the entity... 1 is stand alone, the other is
      collegated with the ns3 network simulator, for the end user it will always call
      the same function GetAllMessages but depending on the implementation type one there'll
      be a dispatching to the right function*/
    TMessageList GetMessagesFromSocket(void);
    TMessageList GetMessagesLocal(void);

    

  protected:
    CVector3 m_cPosition;
    CQuaternion m_cOrientation;
  private:



    /*The function for the opening of the socket is private so that it can only be
      called inside the class*/
    void InitSocketWithExternalApplication(void);

    TMessageList m_tMessages; /*Local copy of the messages sentz by the actuator*/
    TMode m_tMode;
    UInt32 m_unNumericId;/*The id of the entity, used when sending msg to ns3 as recipient*/
    TSocket m_tSocket;
    bool m_bModeSet;
    bool m_externInitialized;
    CSpace& m_cSpace;
    Real m_fRange;
    Real m_fProbability;	// added by Michal
    /*End Added by Marco*/

    static UInt32 m_unStaticNumericIdCounter;
    static TIdConversion m_tIdConversionMap;
  };

}

#endif
