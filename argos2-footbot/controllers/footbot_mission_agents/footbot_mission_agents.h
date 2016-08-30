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
#include <navigation/client/nav_client.h>
#include <include/constants.hpp>
#include <fstream>

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

    size_t create_message_torelay(char* message);
    bool reachedTarget;
    RobotNavState target_state;
    uint8_t time_counter;
    uint8_t neighbour_agents_number;
    void parse_message(vector<char>& received_message);
    void getData();
    
    // file to store data
    ofstream data_file;
    //ofstream sent_message_file;

    string filename;
    //string sent_file;
    
    struct Agent_profile_message
   {
        uint32_t message_size;
        uint8_t agent_id;
        double agent_current_x;
        double agent_current_y;
        uint64_t time_message_sent;
        // Neighbors are mission agents
        uint8_t number_neighbors;
        uint64_t time_last_data_transmitted;
    };  

    struct Agent_profile_message profile_message;
    uint64_t generated_data_size;
    size_t getData(char* ptr);

    

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
