#ifndef _FOOTBOTMISAGENT_H_
#define _FOOTBOTMISAGENT_H_

#include <iostream>
#include <fstream>
#include <argos2/common/control_interface/ci_controller.h>
#include <argos2/common/utility/logging/argos_log.h>
#include <argos2/common/utility/argos_random.h>
#include <argos2/common/utility/datatypes/color.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_wheels_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_leds_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_beacon_actuator.h>
#include <argos2/common/utility/argos_random.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <math.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_encoder_sensor.h>
#include <navigation/client/nav_client.h>

using namespace argos;
using namespace std;

#include <argos2/common/utility/datatypes/datatypes.h>

#define MAX_UDP_SOCKET_BUFFER_SIZE 1500

class FootbotMissionAgents: public CCI_Controller
{
  private:
    UInt32 RandomSeed;
    std::string m_MyIdStr;
    UInt64 m_Steps;
    CARGoSRandom::CRNG* m_randomGen;
    UInt64 m_sendPackets;
    UInt8 m_myID;
    RVONavClient *m_navClient;

    CCI_WiFiSensor* m_pcWifiSensor;
    CCI_WiFiActuator* m_pcWifiActuator;

    CCI_FootBotLedsActuator* m_pcLEDs;

    //  msg buffer
    char *m_socketMsg;
    uint64_t m_lastTxTime;

    UInt8 getNumberOfNeighbors();

    size_t makeProfileMsg();
    //void parseMessage(size_t );
   
    
  public:

    /* Class constructor. */
    FootbotMissionAgents();

    /* Class destructor. */
    virtual ~FootbotMissionAgents() {
    }

    virtual void Init(TConfigurationNode& t_tree);

    virtual void ControlStep();
    virtual void Destroy();
    virtual bool IsControllerFinished() const;

    static std::string getTimeStr();
    UInt64 getTime();
   // void sendStringPacketTo(int dest, const string msg);
    CVector3 randomWaypoint();
      

};

#endif
