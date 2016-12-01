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
  m_Steps(0)
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
  //GetNodeAttributeOrDefault(t_node, "optimalSpeed", optimal_speed, optimal_speed);
  //GetNodeAttributeOrDefault(t_node, "targetMinPointDistance", min_distance_to_target, min_distance_to_target);
  

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

  /// initialise goal position
  statedata.goal_pos = randomWaypoint();
}

CVector3
FootbotMissionAgents::randomWaypoint()
{
  /// generate point in the square (1,10) x (1,10)
  Real x = m_randomGen->Uniform(CRange<Real>(-4.5,4.5));
  Real y = m_randomGen->Uniform(CRange<Real>(-4.5,4.5));
  CVector3 random_point(x,y,0);
  return random_point;
}


FootbotMissionAgents::SStateData::SStateData():
  State(STATE_EXPLORING),
  wait_time(0.0),
  target_state(STATE_ARRIVED_AT_TARGET) {}






 
void 
FootbotMissionAgents::ControlStep() 
{ 
  m_Steps+=1;
  DEBUGCOMM("current state %d\n", statedata.State);
  switch(statedata.State) {
      case SStateData::STATE_RESTING: {
         Rest();
         break;
      }
      case SStateData::STATE_EXPLORING: {
         Explore();
         break;
      }
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
   }

  ///  must call this two methods from navClient in order to
  ///  update the navigation controller
  m_navClient->setTime(getTime());
  m_navClient->update();
  m_pcLEDs->SetAllColors(CColor::GREEN);
}

void
FootbotMissionAgents::Explore()
{ 
  if(m_navClient->state() == statedata.target_state)
  {
    statedata.State = SStateData::STATE_RESTING;
    statedata.wait_time = m_randomGen->Uniform(CRange<Real>(5,10))*10;
    DEBUGCOMM("assigned wait_time %f\n",statedata.wait_time);
    m_navClient->stop();
  }

}


void
FootbotMissionAgents::Rest()
{ 
  
  if(statedata.wait_time > 0)
  { 
    statedata.wait_time = statedata.wait_time-1;
    DEBUGCOMM("waiting time %f\n",statedata.wait_time);
  }
  
  else if(statedata.wait_time <= 0)
  { 
    statedata.State = SStateData::STATE_EXPLORING;
    statedata.goal_pos = randomWaypoint();
    
    DEBUGCOMM("waiting 0 state %d\n", statedata.State);
    m_navClient->start();
    m_navClient->setTargetPosition( statedata.goal_pos );
  }
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

