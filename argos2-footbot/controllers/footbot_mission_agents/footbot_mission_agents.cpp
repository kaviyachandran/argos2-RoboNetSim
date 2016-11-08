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
  reachedTarget(true),
  predicted_timesteps(360),
  target_state(STATE_ARRIVED_AT_TARGET),
  neighbour_agents_number(0),
  interval(10),
  time_one_run(2540),
  wait_time(0),
  discarded_data_count(0)
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

  /// initialize buffer
  m_socketMsg = new char[MAX_UDP_SOCKET_BUFFER_SIZE];

  //initialise target position list with few(10) random positions
  for(int i =0; i<5; i++)
  {
    CVector3 temp = randomWaypoint();
    target_positions.push_back(temp[0]);
    target_positions.push_back(temp[1]);
  }
 
   

   // file to save the data
  filename = "data_"+ to_string(m_myID)+".csv";
  data_file.open(filename,ios::out | ios::ate | ios::app);
  
  //file to save info related to generated data
  generated_data_file = "generated_data" + to_string(m_myID)+".csv";
  generated_data_info.open(generated_data_file,ios::out | ios::ate | ios::app);
  generated_data_info << "Time Step " << "Data Size " << "Relay Id " << "\n";

  //sent_file = "sent_data.csv";
  //sent_message_file.open(sent_file);
  //sent_message_file << "Time_step" << "," <<"Sender" << "," <<"Receiver" << "\n";

}


size_t
FootbotMissionAgents::create_message_torelay(char* mes_ptr)
{ 
  long unsigned int initial_address = (long unsigned int)&(*mes_ptr);
  uint32_t mes_size = 0;
  // id --> 00 for agent sending profile data
  char identifier = 'a';
  memcpy(mes_ptr, &identifier, sizeof(identifier));
  mes_ptr += sizeof(identifier);
  
  
  profile_message.message_size = mes_size;
  memcpy(mes_ptr, &profile_message.message_size,sizeof(profile_message.message_size));
  mes_ptr = mes_ptr + sizeof(profile_message.message_size);
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
  
  

  /// put numberofneighbors (1)
  /// #5: n_neighbors: uint8_t
  profile_message.number_neighbors = neighbour_agents_number;
  memcpy(mes_ptr, &profile_message.number_neighbors, sizeof(profile_message.number_neighbors));
  mes_ptr += sizeof(profile_message.number_neighbors);
  
  /// timestep
  memcpy(mes_ptr, &m_myID, sizeof(m_Steps));
  mes_ptr += sizeof(m_Steps);
  
 
  long unsigned int final_address = (long unsigned int)&(*mes_ptr);
  DEBUGCOMM("before vector %d\n", final_address-initial_address); 

  /// #6: 20 future target position 
  profile_message.target_pos_x = target_positions[0];
  profile_message.target_pos_y = target_positions[1];
  
  memcpy(mes_ptr, &profile_message.target_pos_x, sizeof(profile_message.target_pos_x));
  mes_ptr = mes_ptr + sizeof(profile_message.target_pos_x); 
  
  memcpy(mes_ptr, &profile_message.target_pos_y, sizeof(profile_message.target_pos_y));
  mes_ptr = mes_ptr + sizeof(profile_message.target_pos_y); 
  
  profile_message.data_available = sizeof(double)*fake_data.size();
  memcpy(mes_ptr, &profile_message.data_available, sizeof(profile_message.data_available));
  mes_ptr = mes_ptr + sizeof(profile_message.data_available);

  final_address = (long unsigned int)&(*mes_ptr);
  
  DEBUGCOMM("Composed MSG of size %d\n", final_address-initial_address);
  mes_size = (final_address-initial_address);
  
  return (size_t)mes_size;
}

UInt8
FootbotMissionAgents::getNumberOfNeighbors()
{
  return 0;
}




CVector3
FootbotMissionAgents::randomWaypoint()
{
  /// generate point in the square (1,10) x (1,10)
  Real x = m_randomGen->Uniform(CRange<Real>(-9,9));
  Real y = m_randomGen->Uniform(CRange<Real>(-9,9));
  CVector3 random_point(x,y,0);
  return random_point;
}

size_t
FootbotMissionAgents::getData(char* data_ptr)
  {
    long unsigned int initial_address =  (long unsigned int)&(*data_ptr);
    
    char mes_type_id = 'b';
    memcpy(data_ptr,&mes_type_id,sizeof(mes_type_id));
    data_ptr = data_ptr + sizeof(mes_type_id);
    
    memcpy(data_ptr,&m_myID,sizeof(m_myID));
    data_ptr = data_ptr + sizeof(m_myID);

    //uint64_t time_data_sent = getTime();
    uint64_t time_data_sent = m_Steps;
    memcpy(data_ptr,&time_data_sent,sizeof(time_data_sent));
    data_ptr = data_ptr + sizeof(time_data_sent);

    uint32_t d_size = fake_data.size()*sizeof(uint8_t);
    memcpy(data_ptr,&d_size,sizeof(d_size));
    data_ptr = data_ptr + sizeof(d_size);

    cout << "Size of data generated and send to relay " << d_size << endl;

    DEBUGCOMM("Size of data sent %d\n",fake_data.size()*sizeof(uint8_t));
    //memcpy(data_ptr, fake_data.data(), fake_data.size()*sizeof(uint8_t));
    //data_ptr = data_ptr + fake_data.size()*sizeof(uint8_t); 
    
    long unsigned int final_address =  (long unsigned int)&(*data_ptr);
    
    generated_data_size = (final_address-initial_address);

    DEBUGCOMM("Generated size of Data to be sent to Relay %d\n", generated_data_size); 
    return (size_t)generated_data_size; 
  }


void 
FootbotMissionAgents::parse_message(std::vector<char> &m_incomingMsg)
{
char *cntptr = (char*)&m_incomingMsg[0];
char sender_identifier = char(cntptr[0]);
cntptr = cntptr + sizeof(sender_identifier);
DEBUGCOMM("relay message type %c \n",char(sender_identifier)); 

if(sender_identifier == 'c')
{ 
  bool send = false;
  uint8_t relay_id = cntptr[0];
  cntptr += sizeof(relay_id); 
  DEBUGCOMM("Received message from relay %d \n",relay_id); 
  
  if(relays_met.count(relay_id) != 0)
    {
      uint32_t time_last_met = relays_met[relay_id];
      if((m_Steps-time_last_met) >= 200)
       { // 10 seconds = 20*10 timesteps
         relays_met[relay_id] = m_Steps;
         send = true;
       }
       cout << "time last met " << time_last_met << " Should the agent send message " << send << endl;
    }
  else
    {
      send = true;
      relays_met.insert(pair<uint8_t, uint32_t>(relay_id,m_Steps));
      cout << "agent met a relay first time " << endl;
    }

  
  
  if(send)
    {
        // response to relay
        size_t mes_size = create_message_torelay(m_socketMsg);
        DEBUGCOMM("sending %d bytes to relay %d in range\n", mes_size, relay_id);
        std::ostringstream str_tmp(ostringstream::out);
        str_tmp << "fb_" << relay_id;
        string str_Dest = str_tmp.str();
        m_pcWifiActuator->SendBinaryMessageTo(str_Dest.c_str(),m_socketMsg,mes_size);
        
    }
}
else if(sender_identifier == 'd')
{ 
  
  uint8_t relay_id = cntptr[0];
  cntptr += sizeof(relay_id); 
  DEBUGCOMM("Received message from relay %d \n",relay_id); 


  char agent_data[MAX_UDP_SOCKET_BUFFER_SIZE];
  DEBUGCOMM("Relay %d requests data\n",relay_id);
  
  size_t data_size = getData(agent_data);
  
  
  std::ostringstream str_tmp(ostringstream::out);
  str_tmp << "fb_" << relay_id;
  string str_Dest = str_tmp.str();

  //uint32_t data_size = sizeof(double)*fake_data.size();
  
  //DEBUGCOMM("sending %d bytes to relay %s in range\n", data_size,str_Dest.c_str());
  generated_data_info << m_Steps << "," << fake_data.size() << "," << relay_id << "\n";
  m_pcWifiActuator->SendBinaryMessageTo(str_Dest.c_str(),agent_data,data_size);
  
  // time last data transmitted
  m_lastTxTime = getTime();
  fake_data.clear();

}

else if(sender_identifier == 'm')
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
  
  
  
  if(fake_data.size() > time_one_run)
  { 
    //uint32_t d_size = sizeof(double)*
    //cout << "POPPING " << endl;
    generated_data_info << m_Steps << "," << discarded_data_count << ","<< "discared data " << "\n";
    //fake_data.pop_front();
    discarded_data_count++;
  }
  else
  {
    uint8_t temp = 1;
    fake_data.push_back(temp);
  }

  /*** Saving the waypoints ***/
  if(m_Steps % 5 == 0 && m_Steps > 2)
  {
    data_file << m_navClient->currentPosition().GetX() << "," << m_navClient->currentPosition().GetY() << "\n";
    
  }
 
  //if(wait_time == m_Steps)
  //{
    if(reachedTarget)
      { 
        //DEBUGCOMM("Wait time %d \n", wait_time);
        if(wait_time > 0)
        { 
          wait_time = wait_time-1;
         
          m_navClient->stop();
          
        }
        else if(wait_time == 0)
        { 
        m_navClient->start(); 
        CVector3 randomPoint(target_positions[0],target_positions[1],0);
        m_navClient->setTargetPosition( randomPoint );
        printf("Robot [%d] selected random point %.2f %.2f\n",
         m_myID,randomPoint.GetX(),randomPoint.GetY());
        reachedTarget = false;
        }
      }
    else if(m_navClient->state() == target_state)
      { 
        //DEBUGCOMM("inside target Wait time %d \n", wait_time);
        CVector3 temp = randomWaypoint();

        // removing the target which is already reached
        target_positions.erase(target_positions.begin(),target_positions.begin()+1);
        target_positions.erase(target_positions.begin(),target_positions.begin()+1);
        
        target_positions.push_back(temp[0]);
        target_positions.push_back(temp[1]);
        
        // Once the agent reaches target it has to wait there for few seconds (collecting data)
        reachedTarget = true;
        Real temp_rand = m_randomGen->Uniform(CRange<Real>(5,10));
        wait_time = (uint8_t)temp_rand*20;
        //wait_time = 450;
        cout << "random " << (uint8_t)temp_rand*10 << "time step " << m_Steps << "wait_ time " << wait_time << endl; 
        DEBUGCOMM("temp, Wait time, time_step %f %d %lu", (uint8_t)temp_rand*10, wait_time, m_Steps);
        
      }
  //}

   /*** send message to other agents ***/
  if(m_Steps%10 == 0)
  {
      char hello_message[5];
      char *hello_msg_ptr = hello_message;
      uint8_t mes_type_id = 'm';
      memcpy(hello_msg_ptr,&mes_type_id,sizeof(mes_type_id));
      hello_msg_ptr += sizeof(mes_type_id);
      m_pcWifiActuator->SendBinaryMessageTo("-1",hello_message,1);
  }

 //searching for the received msgs
  TMessageList t_incomingMsgs;
  m_pcWifiSensor->GetReceivedMessages(t_incomingMsgs);
  for(TMessageList::iterator it = t_incomingMsgs.begin(); it!=t_incomingMsgs.end();it++)
    {
      /// parse msg
    DEBUGCOMM("Received %lu bytes to incoming buffer \n", it->Payload.size());
      
    vector<char> check_message = it->Payload;
    DEBUGCOMM("Identifier of received message [extern] %c\n",char(check_message[0]));
    
    if((char)check_message[0] == 'c' || (char)check_message[0] == 'd' || (char)check_message[0] == 'm')
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

