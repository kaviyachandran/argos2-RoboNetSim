#ifndef _FOOTBOTRELAY_H_
#define _FOOTBOTRELAY_H_

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
#include <string>
#include <termios.h>
#include <math.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_encoder_sensor.h>
#include <navigation/client/nav_client.h>
#include <map>
#include <set>
#include <vector>
#include <queue>
#include <sstream>
#include <include/constants.hpp>
#include <iterator>   
#include <random>
#include <deque>

#include <spiralPattern.h>

#define MAX_UDP_SOCKET_BUFFER_SIZE 1500


using namespace argos;
using namespace std;



#include <argos2/common/utility/datatypes/datatypes.h>

class FootbotRelay: public CCI_Controller
{

 public:

    struct dataWrite
    {
      ofstream data_file;
      string filename;
    };

    struct Position
    {
        double x;
        double y;
    };

    struct SAgentData
    {
        /*** Profile message details ***/
        Position current_location;
        Position goal_location;
        uint64_t time_last_visited;  // time when profile message is sent
        uint64_t time_last_data_collected;
        uint64_t data_available; // size of data to be transmitted
        uint8_t id;
	     //uint8_t number_of_neighbours;

        /*** Details of data transmitted ***/
        uint64_t transmitted_data_time;
        uint32_t transmitted_data_size;

        bool IsGoalSet;
        bool IsAgentStatusAvailable;

        uint8_t NUMBER_OF_AGENT;

        SAgentData();
        void Init(TConfigurationNode& t_node);
        
    };
    
   
    struct SStateData
    {   

      enum EState {
         STATE_SEARCH = 0,  // this is the state when relay knows agents position 
         STATE_EXPLORE, // when no information about agent is available
         STATE_DATA_GATHERING, // when agent is detected
         STATE_RETURN_TO_BASESTATION // moving to base station
        } State;

      enum Search {
        To_AGENT = 0,
        SPIRAL
      } SearchState;

      /**** received data identifier 0 - agent profile data, 1- agent collected data, 2- relay profile data ****/
      enum ReceivedDataType {
         AGENT_PROFILE_DATA = 0,
         AGENT_COLLECTED_DATA
         //RELAY_PROFILE_DATA
      } ReceivedData;

      enum SentDataType {
        RELAY_HELLO_TO_AGENT = 0,
        RELAY_SENDING_ACCEPTANCE_TO_AGENT,
	// RELAY_TO_RELAY
        RELAY_TO_BASESTATION
        
      } SentData;

      //Position base_station;
      uint8_t NUMBER_OF_BASESTATION;
      vector<Position> base_station;
      uint8_t NUMBER_OF_RELAY;
      
      bool IsGoalSet;
      bool IsAgentDetected;
      

      //time limit to come back to BS
      uint64_t time_limit;
      uint64_t time_for_each_agent;
      uint64_t min_data_size;
      uint64_t collected_data_size;


      //spiral line segments motion variables
      
      SStateData();
      void Init(TConfigurationNode& t_node);
    };

    /*struct SpiralData
    { 
      uint8_t spiralCondition; // corner or edge or centre
      uint8_t sequence;
      uint8_t spiral_count;
      bool clockwise;
      vector<bool> possibleDirections;
      Position spiralPos;
      SpiralData();
    };*/

private:

    UInt32 RandomSeed;
    std::string m_MyIdStr;

    UInt64 m_Steps;
    UInt8 m_myID;
    
    uint64_t search_time; 
    
    CARGoSRandom::CRNG* m_randomGen;
    //UInt64 m_sendPackets;
    Position startPos;
    
    float size_x;
    float size_y;
    float speed;

    RVONavClient *m_navClient;
    RobotNavState target_state;

    CCI_WiFiSensor* m_pcWifiSensor;
    CCI_WiFiActuator* m_pcWifiActuator;

    CCI_WiFiSensor* m_pcWifiSensorLongRange;
    CCI_WiFiActuator* m_pcWifiActuatorLongRange;
    
    CCI_FootBotLedsActuator* m_pcLEDs;
    
    SAgentData agentData;
    SStateData stateData;
    //SpiralData spiralData;
    
    CVector3 getSpiralTargetPos(double x, double y);

    // object of class B
    SpiralPattern spiral;
    SpiralPattern::SpiralMotion* spiralData;
    uint8_t counter = 0;
    
public:

    /* Class constructor. */
    FootbotRelay();

    /* Class destructor. */
    virtual ~FootbotRelay() {
    }

    virtual void Init(TConfigurationNode& t_tree);

    virtual void ControlStep();
    virtual void Destroy();
    virtual bool IsControllerFinished() const;

    uint64_t getTimeLimit(float,float);
    void UpdateState();

    static std::string getTimeStr();
    UInt64 getTime();
    
    void SendData(uint8_t,uint8_t);
    void ParseMessage(std::vector<char> &v,uint8_t id);

    void Search();
    void Explore();
    void DataGather();
    void Return();

     /*** communication ***/
    size_t HelloToAgent(char* out_to_agent);
    size_t AcceptanceToAgent(char* out_to_agent);
    size_t ToBaseStation();

    void ParseAgentProfile(vector<char> &incoming_agent_message);
    void ParseAgentCollectedData(vector<char> &incoming_agent_message);
    
    dataWrite relayPositions;
};

#endif
