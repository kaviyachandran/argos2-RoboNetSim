#include "footbot_navigator_robonetsim_example.h"


#ifndef FOOTBOT_SIM
#else
#include <argos2/simulator/physics_engines/physics_engine.h>
#include <argos2/simulator/simulator.h>
#endif


FootbotNavigatorRobonetSimExample::FootbotNavigatorRobonetSimExample() :
  RandomSeed(12345),
  m_Steps(0),
  m_randomGen(0)
{
}


  void 
FootbotNavigatorRobonetSimExample::Init(TConfigurationNode& t_node) 
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
   
  /// create the client and pass the configuration tree (XML) to it
  m_navClient = new RVONavClient(m_myID, GetRobot());
  m_navClient->init(t_node);

  /// start the navigation client
  m_navClient->start();

}


void
FootbotNavigatorRobonetSimExample::sendStringPacketTo(int dest, const string msg)
{
  std::ostringstream str(ostringstream::out);
  m_sendPackets++;
  std::ostringstream str_tmp(ostringstream::out);
  str_tmp << "fb_" << dest;
  string str_Dest = str_tmp.str();
  
  str << "Hi I'm " << (int) m_myID << " and I say \"" << msg << "\" to " << str_Dest;
  std::cout << str.str() << std::endl;
  m_pcWifiActuator->SendMessageTo(str_Dest, str.str());
}


CVector3
FootbotNavigatorRobonetSimExample::randomWaypoint()
{
  /// generate point in the square (1,5) x (1,5)
  Real x = m_randomGen->Uniform(CRange<Real>(1,5));
  Real y = m_randomGen->Uniform(CRange<Real>(1,5));
  CVector3 random_point(x,y,0);
  std::cout << "new random waypoint ("
	    << random_point.GetX()
	    << ", " <<  random_point.GetY()
	    << ")" << std::endl;
  return random_point;
}

  void 
FootbotNavigatorRobonetSimExample::ControlStep() 
{
  m_Steps+=1;

  /* do whatever */

  /// every two seconds
  if( m_Steps % 20 == 0)
    {
      printf("Hello. I'm %d - my current position (%.2f, %.2f) \n",
	     (int) m_myID,
	     m_navClient->currentPosition().GetX(),
	     m_navClient->currentPosition().GetY());

      /// send network packet
      if( m_myID != 1)
	{
	  printf("Hello. I'm %d - sending network packet\n", (int) m_myID);
	  sendStringPacketTo(1, "hello");
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
}


  void 
FootbotNavigatorRobonetSimExample::Destroy() 
{
  DEBUG_CONTROLLER("FootbotNavigatorRobonetSimExample::Destroy (  )\n");
}

/**************************************/

bool 
FootbotNavigatorRobonetSimExample::IsControllerFinished() const 
{
  return false;
}


std::string
FootbotNavigatorRobonetSimExample::getTimeStr()
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
FootbotNavigatorRobonetSimExample::getTime()
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


  
REGISTER_CONTROLLER(FootbotNavigatorRobonetSimExample, "footbot_navigator")

