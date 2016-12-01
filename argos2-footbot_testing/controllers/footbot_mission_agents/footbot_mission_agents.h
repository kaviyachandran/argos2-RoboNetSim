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
#include <unistd.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_encoder_sensor.h>
#include <argos2/common/utility/datatypes/datatypes.h>
#include <navigation/client/nav_client.h>
#include <fstream>
#include <deque>


#define PI 3.14159265

using namespace argos;
using namespace std;


#define MAX_UDP_SOCKET_BUFFER_SIZE 1500



class FootbotMissionAgents: public CCI_Controller
{
 
 public:

    struct SStateData {
      /* possible states in which the controller can be */
      enum EState {
         STATE_RESTING = 0,
         STATE_EXPLORING
      } State;

      SStateData();

      RobotNavState target_state;
      Real wait_time;
      CVector3 goal_pos;
    };


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
    
    /* The controller state information */
    SStateData statedata;

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

    void Rest();
    void Explore();
    CVector3 randomWaypoint();

    static std::string getTimeStr();
    UInt64 getTime();
    
    
};

#endif
