#include "footbot_relay.h"


#ifndef FOOTBOT_SIM
#else
#include <argos2/simulator/physics_engines/physics_engine.h>
#include <argos2/simulator/simulator.h>
#endif


#define MAX_UDP_SOCKET_BUFFER_SIZE 1500

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


FootbotRelay::FootbotRelay() :
  RandomSeed(12345),
  m_Steps(0),
  m_randomGen(0),
  sep(" :,"),
  counter(0),
  NumberOfBaseStation(2),
  changePos(true),
  target_state(STATE_ARRIVED_AT_TARGET)
{
}




  void 
FootbotRelay::Init(TConfigurationNode& t_node) 
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
  GetNodeAttributeOrDefault(t_node, "NumberOfBaseStation", NumberOfBaseStation, NumberOfBaseStation);

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

  //  m_pcWifiSensorLongRange = dynamic_cast<CCI_WiFiSensor* >(GetRobot().GetSensor("wifilongrange"));
  //  m_pcWifiActuatorLongRange = dynamic_cast<CCI_WiFiActuator* >(GetRobot().GetActuator("wifilongrange"));

  //Led actuator
   m_pcLEDs   = dynamic_cast<CCI_FootBotLedsActuator*>(GetRobot().GetActuator("footbot_leds"));
   
  /// create the client and pass the configuration tree (XML) to it
  m_navClient = new RVONavClient(m_myID, GetRobot());
  m_navClient->init(t_node);
  /// start the navigation client
  m_navClient->start(); 
 
  m_pcLEDs->SetAllColors(CColor::MAGENTA);

  m_incomingMsg = new char[MAX_UDP_SOCKET_BUFFER_SIZE];
 }

/*map<int, CVector3>& FootbotRelay::parseMessage(string msg)
{ 
  
  printf("in");
  tokenizer tokens(msg, sep);
  for (tokenizer::iterator tok_iter = tokens.begin();
       tok_iter != tokens.end(); ++tok_iter){
    strVector.push_back((std::string(*tok_iter)));

    }
  printf("out" );
   Real x,y ;
   int id;
   
   cout << "parsed" << endl;
   cout << strVector[1] << endl;
   cout << strVector[3] << endl;
   cout << strVector[4] << endl;

   stm << strVector[1] << " " << strVector[3] << " "<< strVector[4];
   stm >> id >> x >> y; 
   
   CVector3 tempPos(x,y,0);
   mapPos.insert(std::pair<int,CVector3 >(id,tempPos));
   return mapPos;
}*/

/*void
FootbotRelay::sendStringPacketTo(int dest, const string msg)
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

/*CVector3
FootbotRelay::getWaypoint(positionMap& mapPos)
{
  /// generate point in the square (1,5) x (1,5)
  
  CVector3 target_point = mapPos[mapCounter];
  mapCounter = mapCounter + 1;
  if(mapCounter == NumberOfBaseStation)
    mapCounter = 0;
  return target_point;
} */

void
FootbotRelay::parseMsg(size_t len)
{
  uint32_t bcnt = 0;
  char *cntptr = m_incomingMsg;
  /// read id (1)
  /// #1:  id  - uint8_t
  uint8_t robot_id;
  memcpy(&robot_id, cntptr, sizeof(robot_id));
  cntptr += sizeof(robot_id);
  bcnt += sizeof(robot_id);
  DEBUGCOMM("Read id %d from msg\n", robot_id);

  
}

  void 
FootbotRelay::ControlStep() 
{ 
  
  m_Steps+=1;
  string message;
  /* do whatever */
  
 //if (mapPos.size() <= 2)
  //{  
      //searching for the received msgs
  TMessageList t_incomingMsgs;

  m_pcWifiSensor->GetReceivedMessages(t_incomingMsgs);
  for(TMessageList::iterator it = t_incomingMsgs.begin(); it!=t_incomingMsgs.end();it++)
    {
      /// parse msg
      std::copy(it->Payload.begin(),it->Payload.end() , m_incomingMsg);
      DEBUGCOMM("Copied %d bytes to incoming buffer\n", it->Payload.size());
      parseMsg(it->Payload.size());
    }
  
  
  #if 0
  m_pcWifiSensorLongRange->GetReceivedMessages(t_incomingMsgs);

  /// Parsing received  message to get base station position
  for(TMessageList::iterator it = t_incomingMsgs.begin(); it!=t_incomingMsgs.end();it++)
    {      
      Real x = 0.0;
      Real y = 0.0;
      int id = 0;
      std::stringstream stm;

      std::string str_msg(it->Payload.begin(),
        it->Payload.end());
       
     std::cout << "[" << (int) m_myID << "] Received packet " << std::endl;
    message = str_msg;
    if(str_msg.find("BaseStation") != string::npos && basestation_id.size() < NumberOfBaseStation)
      {  
         //printf("inside base station");
        // positionMap temp = parseMessage(message);
         
         typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
         boost::char_separator<char> sep(" :,");
        

         tokenizer tokens(str_msg, sep);
         for (tokenizer::iterator tok_iter = tokens.begin();
         tok_iter != tokens.end(); ++tok_iter)
         {
         strVector.push_back((std::string(*tok_iter))); 
         cout << *tok_iter << endl;
         } 

  stm << strVector[1] << " " << strVector[3] << " "<< strVector[4];
   stm >> id >> x >> y; 

   cout << "after parsing : " << " " << strVector[1] << ":" << strVector[3] << "," << strVector[4] << endl;
   cout << "values of var : " << " " << id << ":" << x << "," << y << endl;
   strVector.clear();
   UInt8 curr_size = basestation_id.size();
   basestation_id.insert(id);
   if(basestation_id.size() > curr_size)
   {
    CVector3 tempPos(x,y,0);
    targetPos.push_back(tempPos);
    cout << "after inserting" << endl;
   
    cout << tempPos.GetX() << " "<< tempPos.GetY() << endl;
   }
   
   //targetPos = tempPos;
    
   

  
         //basePositions.insert(temp.begin(),temp.end());
  /* cout << "Below are the base pos";
   for(map<int, CVector3>::iterator itr = mapPos.begin(); itr != mapPos.end(); ++it)
   {
    CVector3 a = itr->second;
    cout<< a.GetX() << " " << a.GetY() << endl;
   } */
      }
      }
   
#endif
 if(changePos && targetPos.size() > 0 )
    { 
      m_navClient->setTargetPosition( targetPos[counter] );
      //CVector3 targetPos(mapPos[mapCounter]); 
      if(counter == NumberOfBaseStation-1)
        { counter = 0; }
      else{
        counter = counter + 1;
      }
      

      CVector3 temp = targetPos[counter];
      printf("Id [%d] Robot [%d] selected random point %.2f %.2f\n",
       counter,
       m_myID,
       temp.GetX(),
       temp.GetY() ); 
      changePos = false;
      cout << "state of the robot" <<  m_navClient->state() << endl;
    } 
 else if(m_navClient->state() == target_state )
 {
  cout << "inside" << endl;
  changePos = true;
 }

  

 
 
 //target_state = 3;
 
 
/*if(changePos )
    { 
      
      CVector3 targetPos(mapPos[mapCounter]); 
      if(mapCounter == NumberOfBaseStation)
        { mapCounter = 1; }
      mapCounter = mapCounter + 1;
      m_navClient->setTargetPosition( targetPos );
      
      printf("Robot [%d] selected random point %.2f %.2f\n",
       m_myID,
       targetPos.GetX(),
       targetPos.GetY()); 
      changePos = false;
    } 
 

 target_state = STATE_ARRIVED_AT_TARGET;
 if (m_navClient->state() == target_state )
 {
   changePos = true;
   //m_navClient->stop();
 } */

  m_pcLEDs->SetAllColors(CColor::MAGENTA);
  m_navClient->setTime(getTime());
  m_navClient->update();
}


  void 
FootbotRelay::Destroy() 
{
  DEBUG_CONTROLLER("FootbotRelay::Destroy (  )\n");
}

/**************************************/

bool 
FootbotRelay::IsControllerFinished() const 
{
  return false;
}


std::string
FootbotRelay::getTimeStr()
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
FootbotRelay::getTime()
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


  
REGISTER_CONTROLLER(FootbotRelay, "footbot_relay_controller")

