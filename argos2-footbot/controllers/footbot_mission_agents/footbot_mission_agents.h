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
#include <fstream>
#include <deque>


#define PI 3.14159265

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
    
    /// constant number of targets
    uint8_t number_of_targets; 
    uint16_t predicted_timesteps;
    uint8_t interval;
    vector<double> target_positions;
    uint32_t discarded_data_count;

    size_t create_message_torelay(char* message);
    bool reachedTarget;
    RobotNavState target_state;
    
    uint8_t neighbour_agents_number;
    void parse_message(vector<char>& received_message);
    void getData();
    
    // file to store data
    ofstream data_file;
    string filename;
    
    ofstream generated_data_info;
    string generated_data_file;
    
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
        double target_pos_x;
        double target_pos_y;
        uint64_t data_available;
    };  

    struct Agent_profile_message profile_message;
    uint64_t generated_data_size;
    size_t getData(char* ptr);
    uint16_t time_one_run; /// time taken for one single run for a relay

    // Map to store the id of relay and at what time step info about agent is sent
    // to make sure info is only sent every 20 seconds though they are in contact
    map<uint8_t, uint32_t> relays_met;
    deque<double> fake_data;

    //time agent waits at target position collecting data
    uint32_t wait_time;

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

    //vector<double>  calculated_positions(uint16_t timesteps_number,uint8_t interval);
    
    
    void Testing();
};

#endif
