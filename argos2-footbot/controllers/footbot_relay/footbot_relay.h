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
#include <boost/tokenizer.hpp>
#include <vector>
#include <sstream>
#include <include/constants.hpp>
#include <iterator>   

using namespace argos;
using namespace std;
using namespace boost;
using namespace constants;

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

    vector<std::string> strVector;
    bool changePos;
    

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep;
    
    vector<CVector3> targetPos;
    CVector3 position;
    UInt8 counter;
    char *m_incomingMsg;
    void parseMessage(size_t );

    char *m_socketMsg;
    
    UInt8 getNeighbourInfo();

    size_t createProfileMessage();
    size_t messageSize;
    
    
   struct LastserviceInfo
    {
        uint8_t lastserved_station_id;
        uint64_t previousTxTime;
        double initialX, initialY;
    };
    
    struct LastserviceInfo lastserviceInfo;
    
    // Map with id and position of neighbours.
    typedef std::map<uint8_t,std::vector<double> > NeighbourMap;
    
    NeighbourMap relay_neighbour;

    /// variables used for received message

    struct LastserviceInfo received_lastserviceInfo;
    uint8_t targetBaseStation;
    NeighbourMap received_neighbour_info;

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
    
    //void sendStringPacketTo(int dest, const string msg);
    //CVector3 getWaypoint(positionMap& mapP);
    //map<int, CVector3>& parseMessage(string str_msg);

};

#endif
