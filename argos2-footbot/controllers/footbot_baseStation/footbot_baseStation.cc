#include "footbot_baseStation.h"


#ifndef FOOTBOT_SIM
#else
#include <argos2/simulator/physics_engines/physics_engine.h>
#include <argos2/simulator/simulator.h>
#endif


FootbotBaseStation::FootbotBaseStation() :
  RandomSeed(12345),
  m_Steps(0),
  m_randomGen(0)
{
}


  void 
FootbotBaseStation::Init(TConfigurationNode& t_node) 
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
  m_pcWifiSensor =   dynamic_cast<CCI_WiFiSensor*>(GetRobot().GetSensor("wifi"));
  m_pcWifiActuator = dynamic_cast<CCI_WiFiActuator*>(GetRobot().GetActuator("wifi"));

  m_pcLEDs   = dynamic_cast<CCI_FootBotLedsActuator*>(GetRobot().GetActuator("footbot_leds"));
   
  /// create the client and pass the configuration tree (XML) to it
  m_navClient = new RVONavClient(m_myID, GetRobot());
  m_navClient->init(t_node);

  /// start the navigation client
  //m_navClient->start();
  
  /// Sets color to all leds
  m_pcLEDs->SetAllColors(CColor::RED);

}


void
FootbotBaseStation::broadcastStringPacket(const CVector3& baseposition)
{
  std::ostringstream stringMessage(ostringstream::out);
  m_sendPackets++;
  std::ostringstream str_tmp(ostringstream::out);
  //str_tmp << "fb_" << dest;
  //string str_Dest = str_tmp.str();
  
  stringMessage << "BaseStation : " << int(m_myID) << " Position : " << (Real)baseposition.GetX() << ","<<(Real)baseposition.GetY();
  std::cout << stringMessage.str() << std::endl;
 
}


  void 
FootbotBaseStation::ControlStep() 
{
  m_Steps+=1;

 
  
   TMessageList t_incomingMsgs;
   m_pcWifiSensor->GetReceivedMessages_Local(t_incomingMsgs);
   for(TMessageList::iterator it = t_incomingMsgs.begin(); it!=t_incomingMsgs.end();it++){
	   
	   printf("%d received packet from \n", m_myID); 
    }
  
  ///  must call this two methods from navClient in order to
  ///  update the navigation controller
  m_pcLEDs->SetAllColors(CColor::RED);
  m_navClient->setTime(getTime());
  //m_navClient->update();
}


  void 
FootbotBaseStation::Destroy() 
{
  DEBUG_CONTROLLER("FootbotBaseStation::Destroy (  )\n");
}

/**************************************/

bool 
FootbotBaseStation::IsControllerFinished() const 
{
  return false;
}




/// returns time in milliseconds
  UInt64 
FootbotBaseStation::getTime()
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


  
REGISTER_CONTROLLER(FootbotBaseStation, "footbot_baseStation_controller")
