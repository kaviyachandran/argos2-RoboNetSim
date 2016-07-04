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
  m_randomGen(0)
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
FootbotMissionAgents::makeProfileMsg()
{
  uint32_t bcnt = 0;
  char *cntptr = m_socketMsg;

  /// put identifier (1)
  /// #6: identifier - 0 (Mission Agent), 1 (Relay)
  /// Identifier is used to find the source of message 

  uint8_t identifier = 0;
  memcpy(cntptr, &identifier, sizeof(identifier));
  cntptr = cntptr + sizeof(identifier);
  bcnt = bcnt + sizeof(identifier);

  /// put id (1)
  /// #1:  id  - uint8_t
  uint8_t robot_id = (uint8_t) m_myID;
  /// copy to the buffer
  memcpy(cntptr, &robot_id, sizeof(robot_id));
  cntptr += sizeof(robot_id);
  bcnt += sizeof(robot_id);
  
  /// put timestamp (8)
  /// #2: timestamp - uint64_t
  uint64_t tstamp = getTime();
  memcpy(cntptr, &tstamp, sizeof(tstamp));
  cntptr += sizeof(tstamp);
  bcnt += sizeof(tstamp);
  
  /// put position (x,y) (16)
  /// #3: x, y : double, double 
  double nx = (double) m_navClient->currentPosition().GetX();
  double ny = (double) m_navClient->currentPosition().GetY();
  memcpy(cntptr,&nx, sizeof(nx));
  cntptr += sizeof(nx);
  bcnt += sizeof(nx);
  memcpy(cntptr,&ny, sizeof(ny));
  cntptr += sizeof(ny);
  bcnt += sizeof(ny);

  /// put timestamp (8)
  /// #4: timestamp - uint64_t
  uint64_t tlasttransmission = m_lastTxTime;
  memcpy(cntptr, &tlasttransmission, sizeof(tlasttransmission));
  cntptr += sizeof(tlasttransmission);
  bcnt += sizeof(tlasttransmission);
  m_lastTxTime = getTime();

  /// put numberofneighbors (1)
  /// #5: n_neighbors: uint8_t
  uint8_t n_neigh = getNumberOfNeighbors();
  memcpy(cntptr, &n_neigh, sizeof(n_neigh));
  cntptr += sizeof(n_neigh);
  bcnt += sizeof(n_neigh);
  
  

  *cntptr = '\0';
  cntptr+=1;
  bcnt+=1;
  
  DEBUGCOMM("Composed MSG of size %d\n", bcnt);
  return (size_t)bcnt;
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
 /* std::cout << "new random waypoint ("
      << random_point.GetX()
      << ", " <<  random_point.GetY()
      << ")" << std::endl;*/
  return random_point;
}

  void 
FootbotMissionAgents::ControlStep() 
{
  m_Steps+=1;

  /* do whatever */

  /// every two seconds
  if( m_Steps % 20 == 0)
    {
  /// This profile message is sent to relay robots using short communication range 
      size_t psize = makeProfileMsg();
      DEBUGCOMM("Sending MSG of size %d\n", 
		psize); 
	  m_pcWifiActuator->SendBinaryMessageTo("-1",
						(char*)m_socketMsg,
						psize);

      printf("Hello. I'm %d - my current position (%.2f, %.2f) \n",
       (int) m_myID,
       m_navClient->currentPosition().GetX(),
       m_navClient->currentPosition().GetY());

      /// send network packet
     if( m_myID != 1)
  {
    //printf("Hello. I'm %d - sending network packet\n", (int) m_myID);
    //sendStringPacketTo(1, "hello");
  } 
    }

  if( m_Steps % 50 == 0 && m_Steps > 1)
    {
      CVector3 randomPoint = randomWaypoint();
      m_navClient->setTargetPosition( randomPoint );
      printf("Robot [%d] selected random point %.2f %.2f\n",
       m_myID,
       randomPoint.GetX(),
       randomPoint.GetY());
    }


 //searching for the received msgs
  TMessageList t_incomingMsgs;
  m_pcWifiSensor->GetReceivedMessages(t_incomingMsgs);
  for(TMessageList::iterator it = t_incomingMsgs.begin(); it!=t_incomingMsgs.end();it++)
    {
      std::string str_msg(it->Payload.begin(),
        it->Payload.end());
      std::cout << "[" << (int) m_myID << "] Received packet: "
      << str_msg << std::endl;
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

