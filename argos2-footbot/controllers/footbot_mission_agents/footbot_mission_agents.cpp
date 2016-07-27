#include "footbot_mission_agents.h"


#ifndef FOOTBOT_SIM
#else
#include <argos2/simulator/physics_engines/physics_engine.h>
#include <argos2/simulator/simulator.h>
#endif


#define __USE_DEBUG_COMM 1
#if __USE_DEBUG_COMM
#define DEBUGCOMM(m, ...) \
{\
  fprintf(stderr, "%.2f DEBUGCOMM[%d]: " m,\
	  (float) m_Steps,\
	  (int) m_myID, \
          ## __VA_ARGS__);\
  fflush(stderr);\
}
#else
#define DEBUGCOMM(m, ...)
#endif


FootbotMissionAgents::FootbotMissionAgents() :
  RandomSeed(12345),
  m_Steps(0),
  m_randomGen(0),
  reachedTarget(false),
  time_counter(0),
  target_state(STATE_ARRIVED_AT_TARGET),
  neighbour_agents_number(0)
{
}


  void 
FootbotMissionAgents::Init(TConfigurationNode& t_node) 
{
  /// The first thing to do, set my ID
#ifdef FOOTBOT_SIM
  m_myID = 
    atoi(GetRobot().GetRobotId().substr(3).c_str());
#else
  m_myID = 
    atoi(GetRobot().GetRobotId().substr(7).c_str());
#endif
  printf("MyID %d\n", m_myID);

  /// Random
  GetNodeAttributeOrDefault(t_node, "RandomSeed", RandomSeed, RandomSeed);


  if( m_randomGen == NULL )
    {
      CARGoSRandom::CreateCategory("rwp",
           RandomSeed+m_myID);
      m_randomGen = CSimulator::GetInstance().GetRNG();
    }


  //////////////////////////////////////////////////////////////
  // Initialize things required by the communications
  //////////////////////////////////////////////////////////////
  m_pcWifiSensor = dynamic_cast<CCI_WiFiSensor* >(GetRobot().GetSensor("wifi"));
  m_pcWifiActuator = dynamic_cast<CCI_WiFiActuator* >(GetRobot().GetActuator("wifi"));

  //Led actuator
  m_pcLEDs   = dynamic_cast<CCI_FootBotLedsActuator*>(GetRobot().GetActuator("footbot_leds"));
   
  /// create the client and pass the configuration tree (XML) to it
  m_navClient = new RVONavClient(m_myID, GetRobot());
  m_navClient->init(t_node);

  /// start the navigation client
  m_navClient->start();
  m_pcLEDs->SetAllColors(CColor::GREEN);

  /// initialize buffer
  m_socketMsg = new char[MAX_UDP_SOCKET_BUFFER_SIZE];


}


/*void
FootbotDiffusionExample::sendStringPacketTo(int dest, const string msg)
{
  std::ostringstream str(ostringstream::out);
  m_sendPackets++;
  std::ostringstream str_tmp(ostringstream::out);
  str_tmp << "fb_" << dest;
  string str_Dest = str_tmp.str();
  
  str << "Hi I'm " << (int) m_myID << " and I say \"" << msg << "\" to " << str_Dest;
  std::cout << str.str() << std::endl;
  m_pcWifiActuator->SendMessageTo(str_Dest, str.str());
}*/


/// fills the buffer with a new msg content,
/// taking into account the current packet size
/// valid for both simulation and real
size_t
FootbotMissionAgents::create_message_torelay(char* mes_ptr)
{ 
  long unsigned int initial_address = (long unsigned int)&(*mes_ptr);
  // id --> 00 for agent sending profile data
  uint8_t identifier = 0;
  memcpy(mes_ptr, &identifier, sizeof(identifier));
  mes_ptr += sizeof(identifier);

  /// put id (1)
  /// #1:  id  - uint8_t
  profile_message.agent_id = (uint8_t) m_myID;
  /// copy to the buffer
  memcpy(mes_ptr, &profile_message.agent_id, sizeof(profile_message.agent_id));
  mes_ptr += sizeof(profile_message.agent_id);
  
  /// put timestamp (8)
  /// #2: timestamp - uint64_t
  profile_message.time_message_sent = getTime();
  memcpy(mes_ptr, &profile_message.time_message_sent, sizeof(profile_message.time_message_sent));
  mes_ptr += sizeof(profile_message.time_message_sent);
  
  /// put position (x,y) (16)
  /// #3: x, y : double, double 
  profile_message.agent_current_x = (double) m_navClient->currentPosition().GetX();
  profile_message.agent_current_y = (double) m_navClient->currentPosition().GetY();
  memcpy(mes_ptr,&profile_message.agent_current_x, sizeof(profile_message.agent_current_x));
  mes_ptr += sizeof(profile_message.agent_current_x);
  
  memcpy(mes_ptr,&profile_message.agent_current_y, sizeof(profile_message.agent_current_y));
  mes_ptr += sizeof(profile_message.agent_current_y);
  
  /// put timestamp (8)
  /// #4: timestamp - uint64_t
  profile_message.time_last_data_transmitted = m_lastTxTime;
  memcpy(mes_ptr, &profile_message.time_last_data_transmitted, sizeof(profile_message.time_last_data_transmitted));
  mes_ptr += sizeof(profile_message.time_last_data_transmitted);
  
  m_lastTxTime = getTime();

  /// put numberofneighbors (1)
  /// #5: n_neighbors: uint8_t
  profile_message.number_neighbors = neighbour_agents_number;
  memcpy(mes_ptr, &profile_message.number_neighbors, sizeof(profile_message.number_neighbors));
  mes_ptr += sizeof(profile_message.number_neighbors);
  
  long unsigned int final_address = (long unsigned int)&(*mes_ptr);
  
  DEBUGCOMM("Composed MSG of size %d\n", final_address-initial_address);
  return (size_t)(final_address-initial_address);
}

UInt8
FootbotMissionAgents::getNumberOfNeighbors()
{
  return 0;
}

CVector3
FootbotMissionAgents::randomWaypoint()
{
  /// generate point in the square (1,5) x (1,5)
  Real x = m_randomGen->Uniform(CRange<Real>(1,5));
  Real y = m_randomGen->Uniform(CRange<Real>(1,5));
  CVector3 random_point(x,y,0);
  return random_point;
}

void
FootbotMissionAgents::getData()
  {
    char agent_data[MAX_UDP_SOCKET_BUFFER_SIZE];
    char* data_ptr = agent_data;
    long unsigned int initial_address =  (long unsigned int)&(*data_ptr);
  
    for(int i =0 ;i < 1000; i++)
      {
    
        uint8_t a = 20;
        memcpy(data_ptr,&a,sizeof(a));
        data_ptr = data_ptr + sizeof(a);
      }
    
    long unsigned int final_address =  (long unsigned int)&(*data_ptr);
    size_t mes_size = (final_address-initial_address);
    DEBUGCOMM("Message to be sent to Relay %d\n", mes_size);  
  }


void 
FootbotMissionAgents::parse_message(std::vector<char> &m_incomingMsg)
{
char *cntptr = (char*)&m_incomingMsg[0];
uint8_t sender_identifier = uint8_t(cntptr[0]);
cntptr = cntptr + sizeof(sender_identifier);

if(sender_identifier == 2)
{ 
  DEBUGCOMM("Received message from relay"); 
  uint8_t relay_id = cntptr[0];
  cntptr += sizeof(relay_id); 

  // response to relay
  size_t mes_size = create_message_torelay(m_socketMsg);
  DEBUGCOMM("sending %d bytes to relay in range\n", mes_size);
  std::ostringstream str_tmp(ostringstream::out);
  str_tmp << "fb_" << relay_id;
  string str_Dest = str_tmp.str();
  m_pcWifiActuator->SendBinaryMessageTo(str_Dest.c_str(),m_socketMsg,mes_size);
}
else if(sender_identifier == 3)
{ 
  DEBUGCOMM("Relay requests data\n");
  getData();
}
else if(sender_identifier == 0)
{
 // response to mission agent
  neighbour_agents_number+=1;
  DEBUGCOMM("Number of neighbours %d\n", neighbour_agents_number); 
}

}


  void 
FootbotMissionAgents::ControlStep() 
{
  m_Steps+=1;
  neighbour_agents_number = 0;
  
  /*** send message to other agents ***/
  char hello_message[5];
  char *hello_msg_ptr = hello_message;
  uint8_t mes_type_id = 0;
  memcpy(hello_msg_ptr,&mes_type_id,sizeof(mes_type_id));
  hello_msg_ptr += sizeof(mes_type_id);
  m_pcWifiActuator->SendBinaryMessageTo("-1",hello_message,1);

  if(!reachedTarget)
    {
      CVector3 randomPoint = randomWaypoint();
      m_navClient->setTargetPosition( randomPoint );
      printf("Robot [%d] selected random point %.2f %.2f\n",
       m_myID,randomPoint.GetX(),randomPoint.GetY());
      reachedTarget = true;
    }
  else if(m_navClient->state() == target_state)
    { 
      //if(time_counter == 20)
      reachedTarget = false;
      //time_counter = 0;
    }

 //searching for the received msgs
  TMessageList t_incomingMsgs;
  m_pcWifiSensor->GetReceivedMessages(t_incomingMsgs);
  for(TMessageList::iterator it = t_incomingMsgs.begin(); it!=t_incomingMsgs.end();it++)
    {
      /// parse msg
      DEBUGCOMM("Received %lu bytes to incoming buffer\n", it->Payload.size());
      parse_message(it->Payload);
    }

  
  ///  must call this two methods from navClient in order to
  ///  update the navigation controller
  m_navClient->setTime(getTime());
  m_navClient->update();
  m_pcLEDs->SetAllColors(CColor::GREEN);
}


  void 
FootbotMissionAgents::Destroy() 
{
  DEBUG_CONTROLLER("FootbotDiffusionExample::Destroy (  )\n");
}

/**************************************/

bool 
FootbotMissionAgents::IsControllerFinished() const 
{
  return false;
}


std::string
FootbotMissionAgents::getTimeStr()
{
#ifndef FOOTBOT_SIM
  char buffer [80];
  timeval curTime;
  gettimeofday(&curTime, NULL);
  int milli = curTime.tv_usec / 1000;
  strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));
  char currentTime[84] = "";
  sprintf(currentTime, "%s:%d", buffer, milli);
  std::string ctime_str(currentTime);
  return ctime_str;
#else
  return "mytime";
#endif
}


/// returns time in milliseconds
  UInt64 
FootbotMissionAgents::getTime()
{
#ifndef FOOTBOT_SIM
  struct timeval timestamp;
  gettimeofday(&timestamp, NULL);

  UInt64 ms1 = (UInt64) timestamp.tv_sec;
  ms1*=1000;

  UInt64 ms2 = (UInt64) timestamp.tv_usec;
  ms2/=1000;

  return (ms1+ms2);
#else
  return m_Steps * CPhysicsEngine::GetSimulationClockTick() * 1000;
#endif
}


  
REGISTER_CONTROLLER(FootbotMissionAgents, "footbot_mission_agent_controller")

