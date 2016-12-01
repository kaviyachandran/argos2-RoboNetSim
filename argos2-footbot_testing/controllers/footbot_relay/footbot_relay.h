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
#include <boost/tokenizer.hpp>
#include <vector>
#include <queue>
#include <sstream>
#include <include/constants.hpp>
#include <iterator>   
#include <random>
#include <deque>
#include <boost/bimap/bimap.hpp>
#include <boost/bimap/support/lambda.hpp>
#include <boost/bimap.hpp>



using namespace argos;
using namespace std;
using namespace boost;


#include <argos2/common/utility/datatypes/datatypes.h>

class FootbotRelay: public CCI_Controller
{
  private:
    UInt32 RandomSeed;
    std::string m_MyIdStr;
    UInt64 m_Steps;
    CARGoSRandom::CRNG* m_randomGen;
    //UInt64 m_sendPackets;
    UInt8 m_myID;
    
    RVONavClient *m_navClient;
    RobotNavState target_state;
    CCI_WiFiSensor* m_pcWifiSensor;
    CCI_WiFiActuator* m_pcWifiActuator;

    CCI_WiFiSensor* m_pcWifiSensorLongRange;
    CCI_WiFiActuator* m_pcWifiActuatorLongRange;
    
    CCI_FootBotLedsActuator* m_pcLEDs;

    UInt8 NumberOfBaseStation;
    UInt8 NumberOfMissionAgents;

    vector<std::string> strVector;
    bool changePos;
    bool send_message_to_relay;
    bool calculate_pos;
    
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep;
    
    vector<CVector3> targetPos;
    CVector3 position;
    UInt8 counter;
    char *m_incomingMsg;
    void parse_relay_message(std::vector<char> &v);
    void parse_agent_message(std::vector<char> &v);
    
    char *m_relaySocketMsg;
    
    UInt8 getNeighbourInfo();

    size_t createProfileMessage(char* msg);
    size_t createMessageToMissionAgents(char* m);
    size_t send_collected_data(char *data);
    size_t get_data_from_missionagents(char* data);
    Constants constant; 

    uint8_t neighbour_count;
    int min,max;

    // agent ids are stored in the order of response 
    //from agents to relays and served FIFO
    
    std::queue<uint8_t> agent_ids;
    std::set<uint8_t> check_set;
    
   struct Relay_profile_message
   {
        uint32_t message_size;
        uint8_t relay_id;
        double relay_current_x;
        double relay_current_y;
        uint64_t time_message_sent;
        // Neighbors are mission agents
        uint8_t number_neighbors;
        uint8_t target_basestation;
        uint64_t last_served_time;
        uint8_t last_served_basestation;
    };  
  
    struct Relay_profile_message relay_message;
    struct Relay_profile_message received_relay_message;
    //static int number_of_times = 0;
    
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
        uint64_t timestep;
        double target_pos_x;
        double target_pos_y;
        uint64_t data_available;
    };  
    

    struct Agent_details
    {
        
        bool agent_status; /// 1 -> agent is met at the location, 0 -> agent is lost
        double goal_pos_x;
        double goal_pos_y;
        double current_x;
        double current_y;
    };
    
    struct Agent_details agent_details;

    typedef boost::bimap<uint8_t, uint64_t> bmtype;
    bmtype agent_time_details;
    map<uint8_t, Agent_details > agent_details_map;
    
    uint64_t max_time_interval_to_visit_bs;
    uint64_t time_since_visited_basestation;
    /// calculates target positions for relay depending on the starting position
   
    vector<double> breadth_size;
    vector<double> length_size;
    double max_agent_range;
    deque<double> calculate_target();
    deque<double> relay_target_positions;

    struct Agent_profile_message agent_message;

    struct Agent_data_info
    {
        uint64_t time_data_sent;
        uint32_t data_size;
        //vector<char> fake_data;
    };
    
    uint32_t data_to_BS_size;
    map<uint8_t, vector<Agent_data_info> > data_from_agents;
    struct Agent_data_info agent_data;
    uint8_t number_of_positions;
    
    //vector<uint8_t> target_odd;
    //7vector<uint8_t> target_even;
    
    map<uint8_t,vector<double> > baseStationPosition;
    vector<double> getBaseStationPositions(TConfigurationNode node);
    vector<double> getValues(TConfigurationNode node, double d);
    
    

    bool direction;
    // vector to store the agents met during each run and collected real data
    vector<uint8_t> visited_agents;


    ofstream data_file;
    string filename;

    ofstream received_message_file;
    string received_file;

    ofstream meeting_data_file;
    string meeting_file;

    //vector<uint8_t> agents;
    //vector<uint8_t> data_exchange_agents;
    
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

    static std::string getTimeStr();
    UInt64 getTime();

    void InitialiseAgentTable(uint8_t nagent);
    
    //void sendStringPacketTo(int dest, const string msg);
    //CVector3 getWaypoint(positionMap& mapP);
    //map<int, CVector3>& parseMessage(string str_msg);

};

#endif
