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
  DEBUGCOMM("MyID %d\n", m_myID);
 
  /// Random
  GetNodeAttributeOrDefault(t_node, "RandomSeed", RandomSeed, RandomSeed);
  
  string speed_string;
  if (NodeExists(t_node, "optimalSpeed")) 
  {
    GetNodeText(GetNode(t_node, "optimalSpeed"), speed_string);
    sscanf(speed_string.c_str(), "%f", &speed);
  }
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
  
  CVector3 temp_goal = randomWaypoint();
  /// initialise goal position
  stateData.goal_loc.x = temp_goal[0];
  stateData.goal_loc.y = temp_goal[1];

  stateData.ttl = 2500; // time taken to cover two times the length of diagonal
  DEBUGCOMM("time for a data packet%d \n",stateData.ttl);
  stateData.id = (uint8_t) m_myID;

  agentPositions.filename = "position" + to_string(m_myID)+".csv";
  agentPositions.data_file.open(agentPositions.filename, ios::out | ios::ate);

  dataInformation.filename = "data" + to_string(m_myID)+".csv";
  dataInformation.data_file.open(dataInformation.filename, ios::out | ios::ate);
}

uint32_t 
FootbotMissionAgents::getTimeLimit(float x, float y)
{   
  UInt32 temp_time;
    // time limit ---> time taken to cover twice the length of diagonal
  if(x == y)
  {
    temp_time = (2*x*sqrt(2)*10)/(2*speed); // speed of relay in m/timestep, 2*x = X full length.
  }
  else
  {
    temp_time = (2*sqrt(pow((x),2) + pow((y),2))*10)/(2*speed);
  }
  DEBUGCOMM("time calculated %d \n", temp_time);
  return temp_time;
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
  discarded_data_count(0),
  target_state(STATE_ARRIVED_AT_TARGET) {}

FootbotMissionAgents::SRelayData::SRelayData():
  time_profile_data_sent(0),
  time_gathered_data_sent(0) {}

void 
FootbotMissionAgents::ParseRelayMessage(std::vector<char> &m_incomingMsg)
{
    char *cntptr = (char*)&m_incomingMsg[0];
    uint8_t sender_identifier = uint8_t(cntptr[0]);
    cntptr = cntptr + sizeof(sender_identifier);
    DEBUGCOMM("relay message type %c \n",uint8_t(sender_identifier)); 
    
    SRelayData tempRelayData;
    bool send = false;
    tempRelayData.id = cntptr[0];
    cntptr += sizeof(tempRelayData.id); 
    DEBUGCOMM("Received message from relay %d \n",tempRelayData.id); 
    

    if(relayMap.count(tempRelayData.id) != 0)
    { 

      uint64_t time_last_met = relayMap[tempRelayData.id].time_profile_data_sent;
      if((m_Steps-time_last_met) >= 80) // more than or equal to 8 seconds. In 1 second it travels 0.1 m.
       { // 10 seconds = 20*10 timesteps
         relayMap[tempRelayData.id].time_profile_data_sent = m_Steps;
         send = true;
       }
       cout << "time last met " << time_last_met << " Should the agent send message " << send << endl;
    }
    else
    {
      send = true;
      tempRelayData.time_profile_data_sent = m_Steps;
      relayMap.insert(pair<uint8_t, SRelayData>(tempRelayData.id,tempRelayData));
      cout << "agent met a relay first time " << endl;
    }

  
  
  if(send)
    {   
        stateData.SentData = SStateData::PROFILE_DATA;
        // response to relay
        SendData(stateData.SentData, tempRelayData.id);
    }
}

void 
FootbotMissionAgents::ParseRelayAcceptance(std::vector<char> &m_incomingMsg)
{
  char *cntptr = (char*)&m_incomingMsg[0];
  uint8_t relay_id;

  uint8_t sender_identifier = uint8_t(cntptr[0]);
  cntptr = cntptr + sizeof(sender_identifier);
  DEBUGCOMM("relay message type %d \n", sender_identifier); 

  relay_id = (uint8_t)cntptr[0];
  cntptr += sizeof(relay_id); 
  DEBUGCOMM("Received message from relay %d \n",relay_id); 
  
  // Relay can meet the same agent twice during one run
 /* if(relayMap[relay_id].time_gathered_data_sent >= stateData.ttl/2)
  {
    stateData.SentData = SStateData::COLLECTED_DATA;
    // response to relay
    SendData(stateData.SentData, relay_id);
  } */
  DEBUGCOMM("%d\n", relayMap[relay_id].time_gathered_data_sent);
  stateData.SentData = SStateData::COLLECTED_DATA;
    // response to relay
  SendData(stateData.SentData, relay_id);
}

size_t 
FootbotMissionAgents::SendProfileData(char* mes_ptr, uint8_t type_of_message, uint8_t relay_id)
{ 
  /* Order of message 
      message_id, message_size, agent_id, time_data_sent, agent_current_loc,
      time_last_data_transmitted, number_of_neighbors, future_pos
  */

  long unsigned int initial_address = (long unsigned int)&(*mes_ptr);
  uint32_t mes_size = 0;
  
  // id --> 0 for agent sending profile data (1)
  
  memcpy(mes_ptr, &type_of_message, sizeof(type_of_message));
  mes_ptr += sizeof(type_of_message);
  DEBUGCOMM("%d %d \n",type_of_message,sizeof(type_of_message));
  //profile_message.message_size = mes_size;
 
  /// put id (1)
  /// #1:  id  - uint8_t
  
  /// copy to the buffer
  memcpy(mes_ptr, &stateData.id, sizeof(stateData.id));
  mes_ptr += sizeof(stateData.id);
  DEBUGCOMM("%d %d\n",stateData.id, sizeof(stateData.id));
  
  /// put timestamp (8)
  /// #2: timestamp - uint64_t
  uint64_t time_message_sent = relayMap[relay_id].time_profile_data_sent;
  memcpy(mes_ptr, &time_message_sent, sizeof(time_message_sent));
  mes_ptr += sizeof(time_message_sent);
  DEBUGCOMM("%d %d\n",time_message_sent, sizeof(time_message_sent));
 
  /// put position (x,y) (16)
  /// #3: x, y : double, double 
  stateData.current_loc.x = (double) m_navClient->currentPosition().GetX();
  stateData.current_loc.y = (double) m_navClient->currentPosition().GetY();
  memcpy(mes_ptr,&stateData.current_loc.x, sizeof(stateData.current_loc.x));
  mes_ptr += sizeof(stateData.current_loc.x);
  DEBUGCOMM("%f %d\n",stateData.current_loc.x, sizeof(stateData.current_loc.x));

  memcpy(mes_ptr,&stateData.current_loc.y, sizeof(stateData.current_loc.y));
  mes_ptr += sizeof(stateData.current_loc.y);
  DEBUGCOMM("%f %d\n", stateData.current_loc.y, sizeof(stateData.current_loc.y));
  /// put timestamp (8)
  /// #4: timestamp - uint64_t
  uint64_t time_last_data_transmitted = relayMap[relay_id].time_gathered_data_sent;
  memcpy(mes_ptr, &time_last_data_transmitted, sizeof(time_last_data_transmitted));
  mes_ptr += sizeof(time_last_data_transmitted);
  DEBUGCOMM("%d %d\n",time_last_data_transmitted, sizeof(time_last_data_transmitted));
  /*/// put numberofneighbors (1)
  /// #5: n_neighbors: uint8_t
  profile_message.number_neighbors = neighbour_agents_number;
  memcpy(mes_ptr, &profile_message.number_neighbors, sizeof(profile_message.number_neighbors));
  mes_ptr += sizeof(profile_message.number_neighbors);
  
  /// timestep
  memcpy(mes_ptr, &m_myID, sizeof(m_Steps));
  mes_ptr += sizeof(m_Steps); */
  
 /// #6:  future target position (16)
  double target_pos_x = stateData.goal_loc.x;
  double target_pos_y = stateData.goal_loc.y;
  
  memcpy(mes_ptr, &target_pos_x, sizeof(target_pos_x));
  mes_ptr = mes_ptr + sizeof(target_pos_x); 
  DEBUGCOMM("%f %d\n",target_pos_x, sizeof(target_pos_x));

  memcpy(mes_ptr, &target_pos_y, sizeof(target_pos_y));
  mes_ptr = mes_ptr + sizeof(target_pos_y); 
  DEBUGCOMM("%f %d\n",target_pos_y, sizeof(target_pos_y));

  ///#7: size of fake data generated (8)
  uint64_t data_available = sizeof(uint8_t)*stateData.data_generated.size();
  memcpy(mes_ptr, &data_available, sizeof(data_available));
  mes_ptr = mes_ptr + sizeof(data_available);
  DEBUGCOMM("%d %d\n", data_available, sizeof(data_available));

  long unsigned int final_address = (long unsigned int)&(*mes_ptr);
  
  DEBUGCOMM("Composed MSG of size %d\n", final_address-initial_address);
  mes_size = (final_address-initial_address); 
  
  /// 8.Message size (4)
  memcpy(mes_ptr, &mes_size,sizeof(mes_size));
  mes_ptr = mes_ptr + sizeof(mes_size);
  mes_size = mes_size + sizeof(mes_size);
  DEBUGCOMM("%d %d\n",mes_size, sizeof(mes_size));
  DEBUGCOMM("%d message_size \n", mes_size);
  
  
  return (size_t)mes_size;
}

size_t 
FootbotMissionAgents::SendCollectedData(char* data_ptr,uint8_t type_of_message,uint8_t relay_id)
{ 

  DEBUGCOMM("Relay %d requests data\n",relay_id);
  
  long unsigned int initial_address =  (long unsigned int)&(*data_ptr);
    
  memcpy(data_ptr, &type_of_message, sizeof(type_of_message));
  data_ptr += sizeof(type_of_message);
    
  memcpy(data_ptr,&stateData.id ,sizeof(stateData.id));
  data_ptr = data_ptr + sizeof(stateData.id);

  //uint64_t time_data_sent = getTime();
  relayMap[relay_id].time_gathered_data_sent = m_Steps;
  uint64_t time_data_sent = m_Steps;
  memcpy(data_ptr,&time_data_sent,sizeof(time_data_sent));
  data_ptr = data_ptr + sizeof(time_data_sent);

  uint32_t d_size = stateData.data_generated.size()*sizeof(uint8_t);
  memcpy(data_ptr,&d_size,sizeof(d_size));
  data_ptr = data_ptr + sizeof(d_size);

  cout << "Size of data generated and send to relay " << d_size << endl;

  DEBUGCOMM("Size of data sent %d\n",stateData.data_generated.size()*sizeof(uint8_t));
    //memcpy(data_ptr, fake_data.data(), fake_data.size()*sizeof(uint8_t));
    //data_ptr = data_ptr + fake_data.size()*sizeof(uint8_t); 
    
  long unsigned int final_address =  (long unsigned int)&(*data_ptr);
    
  size_t data_size = (final_address-initial_address);

  DEBUGCOMM("Generated size of Data to be sent to Relay %d\n", data_size); 

  //uint32_t data_size = sizeof(double)*fake_data.size();
  
  //DEBUGCOMM("sending %d bytes to relay %s in range\n", data_size,str_Dest.c_str());
  //generated_data_info << m_Steps << "," << fake_data.size() << "," << relay_id << "\n";
  dataInformation.data_file << stateData.data_generated.size() << "," <<  stateData.discarded_data_count << "\n";
  
  // time last data transmitted
  //m_lastTxTime = getTime();
  stateData.data_generated.clear();
  stateData.discarded_data_count = 0;
  //fake_data.clear();
  return size_t(data_size);
}

void 
FootbotMissionAgents::SendData(uint8_t type_of_message, uint8_t relay_id)
{ 
  char relay_socket_msg[MAX_UDP_SOCKET_BUFFER_SIZE];
  size_t psize;

  switch(type_of_message) {
      case SStateData::PROFILE_DATA: {
         
         psize = SendProfileData(relay_socket_msg,type_of_message,relay_id);
         /*char *p = relay_socket_msg;
         
         for(int i=0;i < psize; i++)
        {
          DEBUGCOMM("Sending %x\n",*p++);
        }*/
        break;
      }
      case SStateData::COLLECTED_DATA: {
         psize = SendCollectedData(relay_socket_msg,type_of_message,relay_id); // explore is not necessary for one-one-one case
         break;
      }
      /* case SStateData::AGENT_TO_AGENT: {
         ParseNeighborData(); // Check argos2-footbot to decide when the relay should communicate
         break;
      } */
     
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
  }
   
   std::ostringstream str_tmp(ostringstream::out);
   str_tmp << "fb_" << relay_id;
   string str_Dest = str_tmp.str();
   m_pcWifiActuator->SendBinaryMessageTo(str_Dest,relay_socket_msg,psize); 
}



void 
FootbotMissionAgents::ParseMessage(uint8_t received_data_id, vector<char> &incoming_agent_message)
{
  switch(received_data_id) {
      case SStateData::RELAY_HELLO_MESSAGE: {
         ParseRelayMessage(incoming_agent_message);
         break;
      }
      case SStateData::RELAY_ACCEPTANCE_FOR_DATA: {
         ParseRelayAcceptance(incoming_agent_message); // explore is not necessary for one-one-one case
         break;
      }
      /* case SStateData::AGENT_DATA: {
         ParseNeighborData(); // Check argos2-footbot to decide when the relay should communicate
         break;
      } */
     
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
  } 
}

void   
FootbotMissionAgents::updateState()
{  

  DEBUGCOMM("In agents \n");
  /**** Check received messages ****/
  TMessageList t_incomingMsgs;
  m_pcWifiSensor->GetReceivedMessages(t_incomingMsgs);
  for(TMessageList::iterator it = t_incomingMsgs.begin(); it!=t_incomingMsgs.end();it++)
    {
      /// parse msg
    DEBUGCOMM("Received %lu bytes to incoming buffer \n", it->Payload.size());
      
    vector<char> check_message = it->Payload;
    DEBUGCOMM("Identifier of received message [extern] %d\n",uint8_t(check_message[0]));
    ParseMessage((uint8_t)check_message[0],it->Payload);
    }

}

void 
FootbotMissionAgents::generateData()
{  
   DEBUGCOMM("Generating Data %d \n", stateData.data_generated.size());
   uint8_t temp_data = 1;
   stateData.data_generated.push_back(temp_data);
   DEBUGCOMM("Generating Data after insertion %d \n", stateData.data_generated.size());
   
   if(stateData.data_generated.size() > stateData.ttl)
   { 
     stateData.data_generated.pop_front();
     stateData.discarded_data_count++;
   }
}
 
void 
FootbotMissionAgents::ControlStep() 
{ 
  m_Steps+=1;

  if(m_Steps % 5 == 0)
  {
    agentPositions.data_file << m_navClient->currentPosition().GetX() << "," << m_navClient->currentPosition().GetY() << "\n";
  }
  //DEBUGCOMM("current state %d\n", statedata.State);
  switch(stateData.State) {
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

   updateState();

   generateData();

  ///  must call this two methods from navClient in order to
  ///  update the navigation controller
  m_navClient->setTime(getTime());
  m_navClient->update();
  m_pcLEDs->SetAllColors(CColor::GREEN);
}

void
FootbotMissionAgents::Explore()
{ 
  if(m_navClient->state() == stateData.target_state)
  {
    stateData.State = SStateData::STATE_RESTING;
    stateData.wait_time = m_randomGen->Uniform(CRange<Real>(5,10))*10;
    //DEBUGCOMM("assigned wait_time %f\n",statedata.wait_time);
    m_navClient->stop();
  }

}


void
FootbotMissionAgents::Rest()
{ 
  
  if(stateData.wait_time > 0)
  { 
    stateData.wait_time = stateData.wait_time-1;
    //DEBUGCOMM("waiting time %f\n",statedata.wait_time);
  }
  
  else if(stateData.wait_time <= 0)
  { 
    stateData.State = SStateData::STATE_EXPLORING;
    CVector3 temp_goal = randomWaypoint();
    stateData.goal_loc.x = temp_goal[0];
    stateData.goal_loc.y = temp_goal[1];
    //goalPositions.data_file << stateData.goal_loc.x << "," << stateData.goal_loc.y << "\n";
    DEBUGCOMM("waiting 0 state %d\n", stateData.State);
    m_navClient->start();
    m_navClient->setTargetPosition( temp_goal );
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
  sDEBUGCOMM(currentTime, "%s:%d", buffer, milli);
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

