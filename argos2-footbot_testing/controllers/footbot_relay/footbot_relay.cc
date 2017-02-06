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
  IsAgentDetected = false;
  
  
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
		base_station.push_back(loc);	
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

  DEBUGCOMM("Number of base_station: %d \n ", stateData.NUMBER_OF_BASESTATION);
	
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
	

	stateData.time_limit = 2*getTimeLimit(size_x,size_y);
  stateData.min_data_size = 0.4*stateData.time_limit*(agentData.NUMBER_OF_AGENT/stateData.NUMBER_OF_RELAY); // 40 % of data from each agent assigned to it
  stateData.time_for_each_agent = ((stateData.time_limit - (stateData.time_limit/2)) / agentData.NUMBER_OF_AGENT);

  DEBUGCOMM("Initialising agent and base station positions \n");
	/****Initialising Agent and Base Station positions ****/
	stateData.Init(GetNode(t_node, "state"));
	agentData.Init(GetNode(t_node, "agent"));

  relayPositions.filename = "position" + to_string(m_myID)+".csv";
  relayPositions.data_file.open(relayPositions.filename, ios::out | ios::ate);
   
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

float checkDistance(double c_x,double c_y,double t_x,double t_y)
{
  return (sqrt(pow((t_x-c_x),2)+sqrt(pow((t_y-c_y),2))));
}

/*CVector3 
FootbotRelay::getSpiralTargetPos(double x, double y)
{   
    // Sending Hello message to Agent
  if(not stateData.possibleDirections[stateData.sequence])
  { 
    if(stateData.sequence % 2 ==0)
    {
      x = stateData.lastPossiblePosition[stateData.sequence]; // 0,2 -> along x direction
    }
    else
    {
      y = stateData.lastPossiblePosition[stateData.sequence];
    }
  }
  else
  {
	 switch(stateData.sequence) 
	 {
      case SStateData::ALONG_POSX: {
      	if((x+stateData.spiral_count) < size_x)
      	{ 
          printf("In pox x \n");
       		x = x+stateData.spiral_count; // 1.0 -> range of mission agents
          printf("+ x %f\n", x );
      	}
        else
        {
          stateData.possibleDirections[stateData.sequence] = false;
        }
       	stateData.sequence+=1;
        break;
      }
      case SStateData::ALONG_POSY: {
        if((y+stateData.spiral_count) < size_y)
        { 
          printf("In pox y \n");
       		y = y+stateData.spiral_count; // 1.0 -> range of mission agents
          printf("+ Y %f\n", y );
        }
        else
        {
          stateData.possibleDirections[stateData.sequence] = false;
        }
       	stateData.sequence+=1;
        break;
      }
      case SStateData::ALONG_NEGX: {
      	if((x-stateData.spiral_count) > -size_x) 
      	{ 
          printf("In neg x \n");
       		x = x-stateData.spiral_count; // 1.0 -> range of mission agents
          printf("- x %f\n", x );
      	}
        else
        {
          stateData.possibleDirections[stateData.sequence] = false;
        }
       	stateData.sequence+=1;
        break;
      }
      case SStateData::ALONG_NEGY: {
        if((y-stateData.spiral_count) > -size_y) 
        { 
          printf("In neg y \n");
       		y = y-stateData.spiral_count; // 1.0 -> range of mission agents
          printf("- Y %f\n", y );
        }
        else
        {
          stateData.possibleDirections[stateData.sequence] = false;
        }
       	stateData.sequence = 0;
        break;
      }
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }

    }
  }
    if(stateData.sequence%2 == 0)
    {
        stateData.spiral_count += (1.0); // considering 1 metre as agent range else (spiral count new = spiral count old + agent range * spiral count old) 
    }
    CVector3 temp(x,y,0); 

    return temp;
} */

size_t 
FootbotRelay::HelloToAgent(char* out_to_agent)
{  
	/*** This message is sent to detect agent in range ***/

	long unsigned int initial_address =  (long unsigned int)&(*out_to_agent);
  
   	/// identifier - 0 relay to agent hello message 
    uint8_t identifier = 0; 
    DEBUGCOMM("Size of character %d\n", sizeof(identifier));
    memcpy(out_to_agent,&identifier,sizeof(identifier));
    out_to_agent = out_to_agent + sizeof(identifier);
    
    uint8_t id = (uint8_t)m_myID;
    memcpy(out_to_agent,&id,sizeof(id));
    out_to_agent = out_to_agent + sizeof(id);
    
    long unsigned int final_address = (long unsigned int)&(*out_to_agent);
    return size_t(final_address-initial_address);
	
}

size_t 
FootbotRelay::AcceptanceToAgent(char* out_to_agent)
{
	/*** This message is sent as acceptance to collect data from agent in range ***/
	DEBUGCOMM("Creating message to request data from agent\n");
    long unsigned int initial_address =  (long unsigned int)&(*out_to_agent);

	uint8_t identifier = 1;
	memcpy(out_to_agent,&identifier,sizeof(identifier));
	out_to_agent += sizeof(identifier);

	uint8_t id_relay = m_myID;
	memcpy(out_to_agent,&id_relay,sizeof(id_relay));
	out_to_agent+= sizeof(id_relay);
    
	long unsigned int final_address =  (long unsigned int)&(*out_to_agent);

    return size_t(final_address-initial_address);

}


void 
FootbotRelay::ParseAgentProfile(vector<char> &incoming_agent_message)
{
  	char* agent_mes_ptr = (char*)&incoming_agent_message[0];
  	
  	

  	DEBUGCOMM("parsing agent message\n");
  
  	uint8_t mes_type = (uint8_t)agent_mes_ptr[0];
  	agent_mes_ptr = agent_mes_ptr + sizeof(mes_type);

    //neighbour_count = neighbour_count + 1;
    
	DEBUGCOMM("profile message from agent\n");
  	
  	// Agent id
  	agentData.id = (uint8_t)agent_mes_ptr[0];
  	agent_mes_ptr = agent_mes_ptr + sizeof(agentData.id);
  	DEBUGCOMM("agent id %d\n",agentData.id);
    
    // Time when message is sent
   
    memcpy(&agentData.time_last_visited, agent_mes_ptr, sizeof(agentData.time_last_visited));
    DEBUGCOMM("time sent %u\n",agentData.time_last_visited);
    agent_mes_ptr+=sizeof(agentData.time_last_visited);

    // Agent pos
    memcpy(&agentData.current_location.x, agent_mes_ptr, sizeof(agentData.current_location.x));
    agent_mes_ptr+= sizeof(agentData.current_location.x);

    memcpy(&agentData.current_location.y, agent_mes_ptr, sizeof(agentData.current_location.y));
    agent_mes_ptr+= sizeof(agentData.current_location.y);
    
    DEBUGCOMM("Agent Position %f %f \n", agentData.current_location.x, agentData.current_location.y);


    // last time when hte data is transmitted
    memcpy(&agentData.time_last_data_collected,agent_mes_ptr,sizeof(agentData.time_last_data_collected));
    agent_mes_ptr+= sizeof(agentData.time_last_data_collected);
    DEBUGCOMM("LAST DATA transmitted %u\n",agentData.time_last_data_collected);

    // number of neighbours
   /* agent_message.number_neighbors = (uint8_t)agent_mes_ptr[0];
    agent_mes_ptr = agent_mes_ptr+ sizeof(agent_message.number_neighbors);
    //DEBUGCOMM("number of neighbours for agent %d\n",agent_message.number_neighbors);
  
	 //timestep
    memcpy(&agent_message.timestep, agent_mes_ptr, sizeof(agent_message.timestep));
    agent_mes_ptr = agent_mes_ptr + sizeof(agent_message.timestep);
    //DEBUGCOMM("timestep %d\n", agent_message.timestep);
     */
    // future position
    agentData.goal_location.x = 0.0;
    agentData.goal_location.y = 0.0;

    memcpy(&agentData.goal_location.x, agent_mes_ptr, sizeof(agentData.goal_location.x));
    agent_mes_ptr = agent_mes_ptr + sizeof(agentData.goal_location.x);
    DEBUGCOMM("target X %f\n", agentData.goal_location.x);

    memcpy(&agentData.goal_location.y, agent_mes_ptr, sizeof(agentData.goal_location.y));
    agent_mes_ptr = agent_mes_ptr + sizeof(agentData.goal_location.y);
    DEBUGCOMM("target Y %f\n", agentData.goal_location.y);
    agentData.IsGoalSet = true;

    // amount of data available from the agent
    memcpy(&agentData.data_available, agent_mes_ptr, sizeof(agentData.data_available));
    agent_mes_ptr = agent_mes_ptr + sizeof(agentData.data_available);
    DEBUGCOMM("Amount of data available %lu\n", agentData.data_available);
    
    uint32_t message_size;
  	memcpy(&message_size, agent_mes_ptr, sizeof(message_size));
    agent_mes_ptr = agent_mes_ptr+sizeof(message_size);
    DEBUGCOMM("message_size %d\n",message_size);

    stateData.IsAgentDetected = true;
    DEBUGCOMM("done parsing agent message\n");
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
  	DEBUGCOMM("Received data from agents\n");
      /// Receiving and storing data
  	 
  	memcpy(&agentData.transmitted_data_size,agent_mes_ptr,sizeof(agentData.transmitted_data_size));
  	agent_mes_ptr = agent_mes_ptr + sizeof(agentData.transmitted_data_size);
  	
  	DEBUGCOMM("Relay knows the size of data: %d sent \n",(agentData.transmitted_data_size));
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
	switch(send_data_id) {
      
      case SStateData::RELAY_HELLO_TO_AGENT: {
         
         char agent_socket_msg[20];
       	 size_t mes_size =  HelloToAgent(agent_socket_msg);
         m_pcWifiActuator->SendBinaryMessageTo_Extern("-1",agent_socket_msg,mes_size);
         break;
      }
      
      case SStateData::RELAY_SENDING_ACCEPTANCE_TO_AGENT: {
         
         char agent_data_msg[20];
         size_t mes_size = AcceptanceToAgent(agent_data_msg); 

         std::ostringstream str_tmp(ostringstream::out);
		     str_tmp << "fb_" << id;
		     string str_Dest = str_tmp.str();
		     m_pcWifiActuator->SendBinaryMessageTo_Extern(str_Dest.c_str(),agent_data_msg,mes_size);

         break;
      }
      case SStateData::RELAY_TO_BASESTATION: {
         //ToBaseStation(); 
         break;
      }
      /*case SStateData::RELAY_TO_RELAY: {
         Return(); 
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
	DEBUGCOMM("In relay state %d\n", stateData.SentData);
	if(m_Steps%10 == 0)
	{   
		stateData.SentData = SStateData::RELAY_HELLO_TO_AGENT;
		SendData(stateData.SentData, 0);
	}

	/*if(search_time >= stateData.time_for_each_agent)
	{   
		DEBUGCOMM("time_for_each_agent exceeded \n");
		m_navClient->stop();
	}*/

	if(stateData.IsAgentDetected)
	{ 
    DEBUGCOMM("Agent Detected \n");
		stateData.State = SStateData::STATE_DATA_GATHERING;
		//spiralData->sequence = 0;
		search_time = 0;
		stateData.SearchState = SStateData::To_AGENT;
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

    DEBUGCOMM("Position %f %f\n", m_navClient->currentPosition().GetX(), m_navClient->currentPosition().GetY());
	
	switch(stateData.State) {
      case SStateData::STATE_SEARCH: {
         Search();
         break;
      }
      case SStateData::STATE_EXPLORE: {
         //Explore(); // explore is not necessary for one-one-one case
         break;
      }
      case SStateData::STATE_DATA_GATHERING: {
         DataGather(); // Check argos2-footbot to decide when the relay should communicate
         break;
      }
      case SStateData::STATE_RETURN_TO_BASESTATION: {
         //Return(); // Once Reached BS send collected data
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
		DEBUGCOMM("Received %lu bytes to incoming buffer\n", it->Payload.size());

		vector<char> check_message = it->Payload;
		DEBUGCOMM("Identifier of received message [extern] %c\n",char(check_message[0]));
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
	SendData(stateData.SentData, agentData.id);
	DEBUGCOMM("Received gathered data \n");
  stateData.IsAgentDetected = false;
	stateData.State = SStateData::STATE_SEARCH;
}

void
FootbotRelay::Search()
{ 
  search_time ++;
  DEBUGCOMM("Searching in mode %d \n", stateData.SearchState);
  
  if(stateData.IsGoalSet)
  {
  if(m_navClient->state() == target_state)
	  { 
      DEBUGCOMM("Reached target %f, %f", m_navClient->currentPosition().GetX(), m_navClient->currentPosition().GetY());
	  	if(stateData.SearchState == SStateData::To_AGENT)
	  	{
	  		stateData.SearchState = SStateData::SPIRAL;

        spiralData = spiral.calculatePositions(m_navClient->currentPosition().GetX(), m_navClient->currentPosition().GetY());
        printf("condition %d\n", spiralData->spiralCondition);
        for(int i =0 ;i< 4 ; i++)
        {
          printf("direction : %s\n",spiralData->possibleDirections[i]);
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
        DEBUGCOMM("agent current_location %f  %f \n", agentData.current_location.x,agentData.current_location.y);
				CVector3 targetPos(agentData.current_location.x,agentData.current_location.y,0.0);
				m_navClient->setTargetPosition(targetPos);
				startPos.x = targetPos[0];
        startPos.y = targetPos[1];
			}
			else if(agentData.IsGoalSet)
			{
        DEBUGCOMM("agent goal_location %f  %f \n", agentData.goal_location.x,agentData.goal_location.y);
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
        DEBUGCOMM("Pos by spiral %f %f \n", targetPos[0], targetPos[1]);
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

