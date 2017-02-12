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
	target_state(STATE_ARRIVED_AT_TARGET),
	search_time(0)
	{}



FootbotRelay::SStateData::SStateData()
{
	IsGoalSet = false;
	
	IsAgentDetected = false; 
	collected_data_size = 0;
	State = SStateData::STATE_SEARCH;
	SearchState = SStateData::To_AGENT;
  MovingToBaseStation = false;
  
  
	/****Calculate time limit and min data size ****/
	
}

/*FootbotRelay::SpiralData::SpiralData()
{
  sequence = 0;
  spiral_count = 1; // Agent range
  possibleDirections.assign(4,true); // assign 4 directions to true initially
  clockwise = true;
}*/

FootbotRelay::SAgentData::SAgentData()
{
	IsGoalSet = false;
  IsDataReceived = false;
}

void
FootbotRelay::SStateData::Init(TConfigurationNode& t_node)
{
	
	printf(" number of base station  %d \n", NUMBER_OF_BASESTATION);
	Position loc; 
	
	//NUMBER_OF_BASESTATION ;
	for(int i=0; i< NUMBER_OF_BASESTATION; i++)
	{   
		ostringstream temp_str;
		temp_str << "base_station" <<  i+1;

		GetNodeAttribute(GetNode(t_node, temp_str.str()), "x", loc.x);
		GetNodeAttribute(GetNode(t_node, temp_str.str()), "y", loc.y);
		base_station.emplace(i+1,(loc));	
		printf("Base Station position %f %f", loc.x, loc.y);
	}

}

void
FootbotRelay::SAgentData::Init(TConfigurationNode& t_node)
{
	
	Position loc; 
	//NUMBER_OF_BASESTATION ;
	for(int i=0; i< NUMBER_OF_AGENT; i++)
	{   
		ostringstream temp_str;
		temp_str << "agent" << i+1;
		
		GetNodeAttribute(GetNode(t_node, temp_str.str()), "x", current_location.x);
		GetNodeAttribute(GetNode(t_node, temp_str.str()), "y", current_location.y);
		printf("agent pos %f %f", current_location.x, current_location.y);	

	}

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

	string temp;
	GetNodeText(GetNode(t_node, "destinationAreaX"), temp);
	size_x = atof(temp.c_str());

	GetNodeText(GetNode(t_node, "destinationAreaY"),temp);
	size_y = atof(temp.c_str());

	GetNodeText(GetNode(t_node, "numberofrelay"), temp);
	stateData.NUMBER_OF_RELAY = atoi(temp.c_str());

	GetNodeText(GetNode(t_node, "numberofBS"), temp);
	stateData.NUMBER_OF_BASESTATION = atoi(temp.c_str());

  printf("Number of base_station: %d \n ", stateData.NUMBER_OF_BASESTATION);
	
  GetNodeText(GetNode(t_node, "numberofAgent"), temp);
	agentData.NUMBER_OF_AGENT = atoi(temp.c_str());


	string speed_string;
	if (NodeExists(t_node, "optimalSpeed")) 
  {
    GetNodeText(GetNode(t_node, "optimalSpeed"), speed_string);
    sscanf(speed_string.c_str(), "%f", &speed);
  }
	
      
    
	if( m_randomGen == NULL )
		{
			CARGoSRandom::CreateCategory("rwp",
					 RandomSeed+m_myID);
			m_randomGen = CSimulator::GetInstance().GetRNG();
		}


	//////////////////////////////////////////////////////////////
	// Initialize things required for communications
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
	
  //printf("Random num %d\n", m_randomGen->Uniform(CRange<UInt32>(1,50)));
  //agentData.time_last_visited = m_randomGen->Uniform(CRange<UInt32>(1,50));
	
  initialiseReturnState = true;
  initialiseDataGather = true;
  initialiseSearchState = true;

	stateData.time_limit = 2*getTimeLimit(size_x,size_y);
  stateData.min_data_size = 0.4*stateData.time_limit*(agentData.NUMBER_OF_AGENT/stateData.NUMBER_OF_RELAY); // 40 % of data from each agent assigned to it
  stateData.time_for_each_agent = ((stateData.time_limit - (stateData.time_limit/2)) / agentData.NUMBER_OF_AGENT);

  printf("Initialising agent and base station positions \n");
	/****Initialising Agent and Base Station positions ****/
	stateData.Init(GetNode(t_node, "state"));
	agentData.Init(GetNode(t_node, "agent"));
  
  // Records Positions of Agents
  relayPositions.filename = "position" + to_string(m_myID)+".csv";
  relayPositions.data_file.open(relayPositions.filename, ios::out | ios::ate);

 // Records the time and position at which a relay meets the agent
  timeStepToMeet.filename = "timestep"+ to_string(m_myID)+".csv";
  timeStepToMeet.data_file.open(timeStepToMeet.filename, ios::out | ios::ate);
   
}

uint64_t 
FootbotRelay::getTimeLimit(float x, float y)
{   
	UInt64 temp_time;
    // time limit ---> time taken to cover twice the length of diagonal
	if(x == y)
	{
		temp_time = (2*x*sqrt(2)*10)/speed; // speed of relay in m/timestep, 2*x = X full length.
	}
	else
	{
		temp_time = (2*sqrt(pow((x),2) + pow((y),2))*10)/speed;
	}
	return temp_time;
}



size_t 
FootbotRelay::HelloToAgent(uint8_t identifier, char* out_to_agent)
{  
	/*** This message is sent to detect agent in range ***/

	long unsigned int initial_address =  (long unsigned int)&(*out_to_agent);
  
   	/// identifier - 0 relay to agent hello message 
    //uint8_t identifier = 0; 
    printf("Size of character %d\n", sizeof(identifier));
    memcpy(out_to_agent,&identifier,sizeof(identifier));
    out_to_agent = out_to_agent + sizeof(identifier);
    
    uint8_t id = (uint8_t)m_myID;
    memcpy(out_to_agent,&id,sizeof(id));
    out_to_agent = out_to_agent + sizeof(id);
    
    long unsigned int final_address = (long unsigned int)&(*out_to_agent);
    return size_t(final_address-initial_address);
	
}

size_t 
FootbotRelay::AcceptanceToAgent(uint8_t identifier,char* out_to_agent)
{
	/*** This message is sent as acceptance to collect data from agent in range ***/
	printf("Creating message to request data from agent\n");
  long unsigned int initial_address =  (long unsigned int)&(*out_to_agent);

	//uint8_t identifier = 1;
	memcpy(out_to_agent,&identifier,sizeof(identifier));
	out_to_agent += sizeof(identifier);

	uint8_t id_relay = m_myID;
	memcpy(out_to_agent,&id_relay,sizeof(id_relay));
	out_to_agent+= sizeof(id_relay);
    
	long unsigned int final_address =  (long unsigned int)&(*out_to_agent);

  return size_t(final_address-initial_address);

}

size_t
FootbotRelay::ToBaseStation(uint8_t identifier,char* data_to_basestation_ptr)
{
  long unsigned int initial_address =  (long unsigned int)&(*data_to_basestation_ptr);
  printf("Creating message to send data to base station\n");

  //char identifier = 'x';
  memcpy(data_to_basestation_ptr, &identifier, sizeof(identifier));
  data_to_basestation_ptr = data_to_basestation_ptr + sizeof(identifier);
  printf("Identifier data to base station %d\n", identifier);


  uint8_t relay_id = m_myID;
  memcpy(data_to_basestation_ptr, &relay_id, sizeof(relay_id));
  data_to_basestation_ptr = data_to_basestation_ptr + sizeof(relay_id);
  printf("Relay Id %d\n", relay_id);

  if(stateData.collected_data_size > 0)
  {
    uint8_t id = agentData.id;
    memcpy(data_to_basestation_ptr, &id, sizeof(id));
    data_to_basestation_ptr = data_to_basestation_ptr + sizeof(id);
    printf("Agent Id:  %d\n", id);

    uint64_t time_last_data_collected = agentData.transmitted_data_time;
    memcpy(data_to_basestation_ptr, &time_last_data_collected, sizeof(time_last_data_collected));
    data_to_basestation_ptr = data_to_basestation_ptr + sizeof(time_last_data_collected);
    printf("Time Message Sent:  %d\n", time_last_data_collected);

    uint32_t agent_message_size = agentData.transmitted_data_size;
    memcpy(data_to_basestation_ptr, &agent_message_size, sizeof(agent_message_size));
    data_to_basestation_ptr = data_to_basestation_ptr + sizeof(agent_message_size);
    printf("Agent Message Size:  %d\n", agent_message_size);

  }
  
  long unsigned int final_address =  (long unsigned int)&(*data_to_basestation_ptr);
  uint32_t data_to_BS_size = (final_address-initial_address); 
  
  /// 8.Message size (4)
  memcpy(data_to_basestation_ptr, &data_to_BS_size,sizeof(data_to_BS_size));
  data_to_basestation_ptr = data_to_basestation_ptr + sizeof(data_to_BS_size);
  data_to_BS_size = data_to_BS_size + sizeof(data_to_BS_size);
  
  printf("%d message_size \n", data_to_BS_size);
  stateData.IsDataSentToBaseStation = true;
  return size_t(data_to_BS_size);
}

void 
FootbotRelay::ParseAgentProfile(vector<char> &incoming_agent_message)
{
  	char* agent_mes_ptr = (char*)&incoming_agent_message[0];
  	
  //	printf("parsing agent message\n");
  
  	uint8_t mes_type = (uint8_t)agent_mes_ptr[0];
  	agent_mes_ptr = agent_mes_ptr + sizeof(mes_type);

    //neighbour_count = neighbour_count + 1;
    
	 //printf("profile message from agent\n");
  	
  	// Agent id
  	agentData.id = (uint8_t)agent_mes_ptr[0];
  	agent_mes_ptr = agent_mes_ptr + sizeof(agentData.id);
  	//printf("agent id %d\n",agentData.id);
    
    // Time when message is sent
   
    memcpy(&agentData.time_last_visited, agent_mes_ptr, sizeof(agentData.time_last_visited));
   // printf("time sent %u\n",agentData.time_last_visited);
    agent_mes_ptr+=sizeof(agentData.time_last_visited);

    // Agent pos
    memcpy(&agentData.current_location.x, agent_mes_ptr, sizeof(agentData.current_location.x));
    agent_mes_ptr+= sizeof(agentData.current_location.x);

    memcpy(&agentData.current_location.y, agent_mes_ptr, sizeof(agentData.current_location.y));
    agent_mes_ptr+= sizeof(agentData.current_location.y);
    
    printf("Agent Position %f %f \n", agentData.current_location.x, agentData.current_location.y);


    // last time when hte data is transmitted
    memcpy(&agentData.time_last_data_collected,agent_mes_ptr,sizeof(agentData.time_last_data_collected));
    agent_mes_ptr+= sizeof(agentData.time_last_data_collected);
    //printf("LAST DATA transmitted %u\n",agentData.time_last_data_collected);

    // number of neighbours
   /* agent_message.number_neighbors = (uint8_t)agent_mes_ptr[0];
    agent_mes_ptr = agent_mes_ptr+ sizeof(agent_message.number_neighbors);
    //printf("number of neighbours for agent %d\n",agent_message.number_neighbors);
  
	 //timestep
    memcpy(&agent_message.timestep, agent_mes_ptr, sizeof(agent_message.timestep));
    agent_mes_ptr = agent_mes_ptr + sizeof(agent_message.timestep);
    //printf("timestep %d\n", agent_message.timestep);
     */
    // future position
    agentData.goal_location.x = 0.0;
    agentData.goal_location.y = 0.0;

    memcpy(&agentData.goal_location.x, agent_mes_ptr, sizeof(agentData.goal_location.x));
    agent_mes_ptr = agent_mes_ptr + sizeof(agentData.goal_location.x);
    printf("target X %f\n", agentData.goal_location.x);

    memcpy(&agentData.goal_location.y, agent_mes_ptr, sizeof(agentData.goal_location.y));
    agent_mes_ptr = agent_mes_ptr + sizeof(agentData.goal_location.y);
    printf("target Y %f\n", agentData.goal_location.y);
    agentData.IsGoalSet = true;

    // amount of data available from the agent
    memcpy(&agentData.data_available, agent_mes_ptr, sizeof(agentData.data_available));
    agent_mes_ptr = agent_mes_ptr + sizeof(agentData.data_available);
    printf("Amount of data available %lu\n", agentData.data_available);
    
    uint32_t message_size;
  	memcpy(&message_size, agent_mes_ptr, sizeof(message_size));
    agent_mes_ptr = agent_mes_ptr+sizeof(message_size);
    //printf("message_size %d\n",message_size);

    if(agentData.data_available > 0)
    {
       stateData.IsAgentDetected = true;
    }
    
    timeStepToMeet.data_file << m_Steps << "," << m_navClient->currentPosition().GetX() << "," << m_navClient->currentPosition().GetY() << "\n";
    printf("done parsing agent message\n");
}

void 
FootbotRelay::ParseAgentCollectedData(vector<char> &incoming_agent_message)
{   
	char* agent_mes_ptr = (char*)&incoming_agent_message[0];
    
    uint8_t mes_type = (uint8_t)agent_mes_ptr[0];
  	agent_mes_ptr = agent_mes_ptr + sizeof(mes_type);

	  uint8_t temp_id;
    memcpy(&temp_id,agent_mes_ptr,sizeof(temp_id));
    agent_mes_ptr = agent_mes_ptr + sizeof(temp_id);

    memcpy(&agentData.transmitted_data_time, agent_mes_ptr, sizeof(agentData.transmitted_data_time));
    agent_mes_ptr = agent_mes_ptr + sizeof(agentData.transmitted_data_time);

	//neighbour_count = neighbour_count + 1;
  	printf("Received data from agents\n");
      /// Receiving and storing data
  	 
  	memcpy(&agentData.transmitted_data_size,agent_mes_ptr,sizeof(agentData.transmitted_data_size));
  	agent_mes_ptr = agent_mes_ptr + sizeof(agentData.transmitted_data_size);
  	
    agentData.IsDataReceived = true;
  	printf("Relay knows the size of data: %d sent \n",(agentData.transmitted_data_size));
}

void 
FootbotRelay::ParseMessage(vector<char> &incoming_agent_message, uint8_t received_data_id)
{
	switch(received_data_id) {
      case SStateData::AGENT_PROFILE_DATA: {
         ParseAgentProfile(incoming_agent_message);
         break;
      }
      case SStateData::AGENT_COLLECTED_DATA: {
         ParseAgentCollectedData(incoming_agent_message); // explore is not necessary for one-one-one case
         break;
      }
      /* case SStateData::RELAY_PROFILE_DATA: {
         ParseRelayProfile(); // Check argos2-footbot to decide when the relay should communicate
         break;
      } */
     
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
	}	
}

void
FootbotRelay::SendData(uint8_t send_data_id, uint8_t id)
{ 
  std::ostringstream str_tmp(ostringstream::out);
  str_tmp << "fb_" << id;
  string str_Dest = str_tmp.str();
	
  switch(send_data_id) {
      
      case SStateData::RELAY_HELLO_TO_AGENT: {
         
         char agent_socket_msg[20];
       	 size_t mes_size =  HelloToAgent(send_data_id,agent_socket_msg);
         m_pcWifiActuator->SendBinaryMessageTo_Extern("-1",agent_socket_msg,mes_size);
         break;
      }
      
      case SStateData::RELAY_SENDING_ACCEPTANCE_TO_AGENT: {
         
         char agent_data_msg[20];
         size_t mes_size = AcceptanceToAgent(send_data_id,agent_data_msg); 

         m_pcWifiActuator->SendBinaryMessageTo_Extern(str_Dest.c_str(),agent_data_msg,mes_size);
         break;
      }
      case SStateData::RELAY_TO_BASESTATION: {
         
         char base_socket_msg[MAX_UDP_SOCKET_BUFFER_SIZE*2];
         size_t collected_data_size = ToBaseStation(send_data_id,base_socket_msg); 

          cout << "Base Station Target " << str_Dest << endl;
          m_pcWifiActuator->SendBinaryMessageTo_Extern(str_Dest.c_str(),base_socket_msg,collected_data_size);
          break;
      }
      /*case SStateData::RELAY_TO_RELAY: {
         InformationToRelay(); 
         break;
      }*/
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
	}
}

void
FootbotRelay::UpdateState()
{	
	printf("In relay state %d\n", stateData.SentData);
	if(m_Steps%10 == 0 && not(stateData.MovingToBaseStation))
	{   
		stateData.SentData = SStateData::RELAY_HELLO_TO_AGENT;
		SendData(stateData.SentData, 0);
	}

	/*if(search_time >= stateData.time_for_each_agent)
	{   
		printf("time_for_each_agent exceeded \n");
		m_navClient->stop();
	}*/

	if(stateData.IsAgentDetected)
	{ 
    printf("Agent Detected \n");
		stateData.State = SStateData::STATE_DATA_GATHERING;
    stateData.IsGoalSet = false;
		//spiralData->sequence = 0;
		search_time = 0;
		
	}
  else
  {
    if(agentData.IsDataReceived)
    { 
      initialiseDataGather = true;
      stateData.State = SStateData::STATE_RETURN_TO_BASESTATION;
      printf("I am in state base station \n");
      stateData.IsGoalSet = false;
      agentData.IsDataReceived = false;
    }

    else if(not stateData.MovingToBaseStation)
    { 
      printf("I am fucking here \n");
      stateData.State = SStateData::STATE_SEARCH;
      if(initialiseSearchState)
      {
        stateData.SearchState = SStateData::To_AGENT;
        initialiseReturnState = true;
        initialiseSearchState = false;
      }
    }
  }
}

void 
FootbotRelay::ControlStep() 
{   
	m_Steps+=1;
  if(m_Steps % 5 == 0)
  {
    relayPositions.data_file << m_navClient->currentPosition().GetX() << "," << m_navClient->currentPosition().GetY() << "\n";
  }
	UpdateState();

    printf("Position %f %f\n", m_navClient->currentPosition().GetX(), m_navClient->currentPosition().GetY());
    printf("Agent State %d \n", stateData.State);
	
	switch(stateData.State) {
      case SStateData::STATE_SEARCH: {
         printf("IN SEARCH STATE \n");
         Search();
         break;
      }
      case SStateData::STATE_EXPLORE: {
         //Explore(); // explore is not necessary for one-one-one case
         break;
      }
      case SStateData::STATE_DATA_GATHERING: {
         printf("IN DataGather STATE \n");
         DataGather(); // Check argos2-footbot to decide when the relay should communicate
         break;
      }
      case SStateData::STATE_RETURN_TO_BASESTATION: {
         printf("IN Return STATE \n");
         Return(); // Once Reached BS send collected data
         break;
      }
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
   }

    /********  Receive messages to and from mission agents ********/
	
  TMessageList message_from_agents;
	m_pcWifiSensor->GetReceivedMessages_Extern(message_from_agents);
	for(TMessageList::iterator it = message_from_agents.begin(); it!=message_from_agents.end();it++)
	{   
		//send_message_to_relay = true;
		printf("Received %lu bytes to incoming buffer\n", it->Payload.size());

		vector<char> check_message = it->Payload;
		printf("Identifier of received message [extern] %c\n",uint8_t(check_message[0]));
		ParseMessage(it->Payload, uint8_t(check_message[0]));
		string sender = it->Sender;
		/*if((char)check_message[0] == 'a')
		{   
			//agents.push_back(atoi((it->Sender).c_str));
			meeting_data_file << sender << " ";
			//parse_agent_message(it->Payload);
			//uint8_t received_from = stoi(it->Sender)+1;
			received_message_file << m_Steps << "," << check_message[0] << "," << sender << "," << (m_navClient->currentPosition().GetX()) << "," << (m_navClient->currentPosition().GetY()) << "\n";
			//received_message_file << m_Steps << "," << check_message[0] << "," << it->Sender << "\n";
		}

		else if((char)check_message[0] == 'b')
		{
			meeting_data_file << "d_"+(sender) << " ";
			//data_exchange_agents.push_back(atoi((it->Sender).c_str));
			received_message_file << m_Steps << "," << check_message[0] << "," << sender << "," << (m_navClient->currentPosition().GetX()) << "," << (m_navClient->currentPosition().GetY()) << "\n";
		} */
    }

	m_pcLEDs->SetAllColors(CColor::MAGENTA);
	m_navClient->setTime(getTime());
	m_navClient->update();
	
	//cout <<"MyId: " << m_myID << "target: " << counter << endl;
	
}

void
FootbotRelay::DataGather()
{
	//send the acceptance to collect data after checking the available data size.
	// move towards agent's current location Potential field -- agent should attract until it sends data and once data is sent agent should repel
	stateData.SentData = SStateData::RELAY_SENDING_ACCEPTANCE_TO_AGENT;
  
  if(initialiseDataGather)
  {  
    SendData(stateData.SentData, agentData.id);
    if(not stateData.IsGoalSet)
    { 
      printf("Agent Location set as target\n");
      CVector3 targetPos(agentData.current_location.x,agentData.current_location.y,0.0);
      m_navClient->setTargetPosition(targetPos);
      stateData.IsGoalSet = true;
      initialiseDataGather = false;
    }
  }
  
  if(m_navClient->state() == target_state)
  {  
     printf("Reached Agent Location\n");
     stateData.collected_data_size = stateData.collected_data_size + agentData.data_available;
     agentData.data_available = 0;
     stateData.MovingToBaseStation = true;
     stateData.IsGoalSet = false;
     stateData.IsAgentDetected = false;
  }
  printf("Received gathered data \n");
}

void
FootbotRelay::Return()
{   
    // initialising Return State -> set target position as position of one of the base station

    Position base_loc = stateData.base_station[1];

    if(initialiseReturnState)
    {
      
      printf("Target set to base location\n");
      CVector3 targetPos(base_loc.x,base_loc.y,0.0);
      m_navClient->setTargetPosition(targetPos);
      stateData.IsGoalSet = true;
      initialiseReturnState = false;
    }
    

    Position currentL;
    currentL.x = m_navClient->currentPosition().GetX();
    currentL.y = m_navClient->currentPosition().GetY();

    double dist = spiral.findDistance(&base_loc, &currentL);
    printf("Distance: %f \n", dist);
    printf("Navigation state of agent %d\n", m_navClient->state());

    
    if(stateData.IsGoalSet && dist <= 0.35) // if an object is stationary(Base Station) at that target location, it doesn't go beyond 0.28.0.35
    {
         // send data once reached base Station
        printf("Reached Base Station\n");
        stateData.SentData = SStateData::RELAY_TO_BASESTATION;
        SendData(stateData.SentData,1);
        m_navClient->stop();
        stateData.IsGoalSet = false;
    }
      
    
    else if(stateData.IsDataSentToBaseStation)
    {  
       printf("Data Sent to Base Station\n");
       m_navClient->start();
       stateData.State = SStateData::STATE_SEARCH;
       stateData.IsDataSentToBaseStation = false;
       stateData.MovingToBaseStation = false;
       initialiseSearchState = true;
    }
    
    //if(not stateData.IsGoalSet)
}

void
FootbotRelay::Search()
{ 
  search_time ++;
  printf("Searching in mode %d \n", stateData.SearchState);
  
  if(stateData.IsGoalSet)
  {
  if(m_navClient->state() == target_state)
	  { 
      printf("Reached target %f, %f", m_navClient->currentPosition().GetX(), m_navClient->currentPosition().GetY());
	  	if(stateData.SearchState == SStateData::To_AGENT)
	  	{
	  		stateData.SearchState = SStateData::SPIRAL;

        spiralData = spiral.calculatePositions(m_navClient->currentPosition().GetX(), m_navClient->currentPosition().GetY());
        printf("condition %d\n", spiralData->spiralCondition);
        for(int i =0 ;i< 4 ; i++)
        {
          printf("direction : %d\n",spiralData->possibleDirections[i]);
        }
	  	}
	    stateData.IsGoalSet = false;
	  }
  }
  else 
  {
    if(stateData.SearchState == SStateData::To_AGENT)
	  { 
      
			if(not agentData.IsGoalSet)
			{ 
        printf("agent current_location %f  %f \n", agentData.current_location.x,agentData.current_location.y);
				CVector3 targetPos(agentData.current_location.x,agentData.current_location.y,0.0);
				m_navClient->setTargetPosition(targetPos);
				startPos.x = targetPos[0];
        startPos.y = targetPos[1];
			}
			else if(agentData.IsGoalSet)
			{
        printf("agent goal_location %f  %f \n", agentData.goal_location.x,agentData.goal_location.y);
				CVector3 targetPos(agentData.goal_location.x,agentData.goal_location.y,0.0);
				m_navClient->setTargetPosition(targetPos);
				agentData.IsGoalSet = false;
        startPos.x = targetPos[0];
        startPos.y = targetPos[1];
			}

      
      stateData.IsGoalSet = true;
	  }
	  else if(stateData.SearchState == SStateData::SPIRAL)
	  {   
        spiralData->spiralPos.x = m_navClient->currentPosition().GetX();
        spiralData->spiralPos.y = m_navClient->currentPosition().GetY();

        if(not spiralData->endCondition)
        { 
          printf("position when cond false %f %f \n", spiralData->spiralPos.x,spiralData->spiralPos.y);
          spiralData = spiral.assignQuadrant(spiralData);
          printf("Assigned Position %f %f \n", spiralData->spiralPos.x, spiralData->spiralPos.y);
          spiralData = spiral.calculatePositions(spiralData->spiralPos.x,spiralData->spiralPos.y);
          printf("Sequence: %d  Increment: %f spiralCondition: %d \n", spiralData->sequence, spiralData->spiral_count, spiralData->spiralCondition);
          spiralData->endCondition = true;
        }
        else
        {
          spiralData->spiralPos.x = m_navClient->currentPosition().GetX();
      	  spiralData->spiralPos.y = m_navClient->currentPosition().GetY();
          spiralData = spiral.getPosition(spiralData);
        }
        
        CVector3 targetPos(spiralData->spiralPos.x,spiralData->spiralPos.y,0.0);
        printf("Pos by spiral %f %f \n", targetPos[0], targetPos[1]);
        m_navClient->setTargetPosition(targetPos);
        stateData.IsGoalSet = true;
	  }
  }
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

