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
	NumberOfBaseStation(3),
	changePos(true),
	target_state(STATE_ARRIVED_AT_TARGET),
	min(1)
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
	
	/// Stores Base Station positions
	for (size_t i=1; i <= NumberOfBaseStation ; i++)
	{
	 	string str = "BaseStation" + to_string(i);
		TConfigurationNode node = GetNode(t_node, str);	
		
		vector<double> basePos = getBaseStationPositions(node);
		baseStationPosition.insert(pair<uint8_t, vector<double> >(i,basePos));
 		
	}
  
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
	//m_relaySocketMsg = new char[MAX_UDP_SOCKET_BUFFER_SIZE];
	max = NumberOfBaseStation;
	//cout << "number of times q" << number_of_times++ << endl;
 }


vector<double>
FootbotRelay::getBaseStationPositions(TConfigurationNode node)
{ 
	vector<double> pos;
	string temp;
	GetNodeAttribute(node, "x", temp);
	pos.push_back(stod(temp));
	GetNodeAttribute(node, "y", temp);
	pos.push_back(stod(temp));
	return pos;
}



uint8_t FootbotRelay::getNeighbourInfo()
{
	return 0;
}

size_t FootbotRelay::createProfileMessage(char *outptr)
{   
	long unsigned int initial_address =  (long unsigned int)&(*outptr);
  	printf("sending message");
    printf("beginning %lu\n", initial_address);
    memcpy(outptr, &relay_message.message_size, sizeof(relay_message.message_size));
    outptr = outptr + sizeof(relay_message.message_size);
   /// Id of relay
   /// Id - uint8_t
	relay_message.relay_id = (uint8_t)m_myID;
    memcpy(outptr, &relay_message.relay_id, sizeof(relay_message.relay_id));
	outptr = outptr + sizeof(relay_message.relay_id);
	printf("Relay Id %d\n", relay_message.relay_id);

	/// Relay Position - (x,y in double)
	relay_message.relay_current_x = (double) m_navClient->currentPosition().GetX();
	relay_message.relay_current_y = (double) m_navClient->currentPosition().GetY();  
       
	memcpy(outptr, &relay_message.relay_current_x, sizeof(relay_message.relay_current_x));
	outptr = outptr + sizeof(relay_message.relay_current_x);
	
  	memcpy(outptr, &relay_message.relay_current_y, sizeof(relay_message.relay_current_y));
	outptr = outptr + sizeof(relay_message.relay_current_y);
	
	printf("Relay pos x %f\n", relay_message.relay_current_x);
	printf("Relay pos y %f\n", relay_message.relay_current_y);

	/// Current time - uint64
	relay_message.time_message_sent = (uint64_t)getTime();
	memcpy(outptr, &relay_message.time_message_sent, sizeof(relay_message.time_message_sent));
	outptr = outptr + sizeof(relay_message.time_message_sent);
	printf("time mes sent %u\n", relay_message.time_message_sent);

	
	// neighbours- uint8_t
	relay_message.number_neighbors = (uint8_t)4;
	memcpy(outptr, &relay_message.number_neighbors, sizeof(relay_message.number_neighbors));
	outptr = outptr + sizeof(relay_message.number_neighbors);
    printf("Neighbour %d\n", relay_message.number_neighbors);

	memcpy(outptr, &relay_message.target_basestation, sizeof(relay_message.target_basestation));
	outptr = outptr + sizeof(relay_message.target_basestation);
	printf("target BS %d\n", relay_message.target_basestation);

	memcpy(outptr, &relay_message.last_served_time, sizeof(relay_message.last_served_time));
	outptr = outptr + sizeof(relay_message.last_served_time);
	printf("Last srerved time %u\n", relay_message.last_served_time);

    memcpy(outptr, &relay_message.last_served_basestation, sizeof(relay_message.last_served_basestation));
	outptr = outptr + sizeof(relay_message.last_served_basestation);
	printf("last served BS %d\n", relay_message.last_served_basestation);
	

	long unsigned int final_address = (long unsigned int)&(*outptr);

	relay_message.message_size = final_address-initial_address;
	printf("Size of message sent: %d\n",relay_message.message_size);
	return size_t(relay_message.message_size);
} 

size_t 
FootbotRelay::createMessageToMissionAgents(char* out_to_agent)
{ 
  long unsigned int initial_address =  (long unsigned int)&(*out_to_agent);
  
  /// identifier - 10 (relay) 0(agent) to identify relay message 
  uint8_t identifier = 10; 
  memcpy(out_to_agent,&identifier,sizeof(identifier));
  out_to_agent = out_to_agent + sizeof(identifier);
  
  uint8_t id = m_myID;
  memcpy(out_to_agent,&id,sizeof(id));
  out_to_agent = out_to_agent + sizeof(id);
  
  long unsigned int final_address = (long unsigned int)&(*out_to_agent);
  return size_t(final_address-initial_address);
}

void
FootbotRelay::parse_agent_message(vector<char> &incoming_agent_message)
{
  char* agent_mes_ptr = (char*)&incoming_agent_message[0];
  
  DEBUGCOMM("parsing agent message\n");

  uint8_t mes_type = (uint8_t)agent_mes_ptr[0];
  agent_mes_ptr = agent_mes_ptr + sizeof(mes_type);

  if(mes_type == 0)
  {  
  	// Agent id
  	uint8_t id = agent_mes_ptr[0];
  	agent_mes_ptr = agent_mes_ptr + sizeof(id);
  	if(check_set.find(id) == check_set.end())
  	{
  		check_set.insert(id);
  		agent_ids.push(id);
  	}
  	
    
    // Time when message is sent
    uint64_t message_sent_time;
    memcpy(&message_sent_time, agent_mes_ptr, sizeof(message_sent_time));
    agent_mes_ptr+=sizeof(message_sent_time);

    // Agent pos
    double agent_x, agent_y;
    memcpy(&agent_x, agent_mes_ptr, sizeof(agent_x));
    agent_mes_ptr+= sizeof(agent_x);

    memcpy(&agent_y, agent_mes_ptr, sizeof(agent_y));
    agent_mes_ptr+= sizeof(agent_y);

    // last time when hte data is transmitted
    uint64_t last_data_tansmission_time;
    memcpy(&last_data_tansmission_time,agent_mes_ptr,sizeof(last_data_tansmission_time));
    agent_mes_ptr+= sizeof(last_data_tansmission_time);

    // number of neighbours
    uint8_t neighbour_number = agent_mes_ptr[0];
    agent_mes_ptr = agent_mes_ptr+ sizeof(neighbour_number);

    DEBUGCOMM("done parsing agent message\n");
	
  }
 /* else if(mes_type == 1)
  {
      /// Receiving and storing data

  } */
}

void
FootbotRelay::parse_relay_message(std::vector<char> &incomingMsg)
{   

	char *cntptr = (char*)&incomingMsg[0];
    /*char *p = cntptr;
		for(int i=0;i < 26; i++)
		{
			printf("Received %x\n",*p++);
		}*/
	
	/// read size
	memcpy(&received_relay_message.message_size, cntptr, sizeof(received_relay_message.message_size));
	cntptr += sizeof(received_relay_message.message_size);
	printf("%d\n", received_relay_message.message_size);
	/// read id (1)
	/// #1:  id  - uint8_t
	received_relay_message.relay_id = uint8_t(cntptr[0]);
	cntptr += sizeof(received_relay_message.relay_id);
	DEBUGCOMM("Read id %d from msg\n", received_relay_message.relay_id);
			
	///read pos (2)
	memcpy(&received_relay_message.relay_current_x, cntptr, sizeof(received_relay_message.relay_current_x));
	cntptr = cntptr + sizeof(received_relay_message.relay_current_x);
	
  	memcpy(&received_relay_message.relay_current_y, cntptr, sizeof(received_relay_message.relay_current_y));
	cntptr = cntptr + sizeof(received_relay_message.relay_current_y);
	

	/// message sent time - uint64
	memcpy(&received_relay_message.time_message_sent, cntptr, sizeof(received_relay_message.time_message_sent));
	cntptr = cntptr + sizeof(received_relay_message.time_message_sent);
	
	// neighbours- uint8_t
	received_relay_message.number_neighbors = cntptr[0];
	cntptr = cntptr + sizeof(received_relay_message.number_neighbors);
    
    received_relay_message.target_basestation = cntptr[0];
	cntptr = cntptr + sizeof(received_relay_message.target_basestation);
	
	memcpy(&received_relay_message.last_served_time,cntptr, sizeof(received_relay_message.last_served_time));
	cntptr = cntptr + sizeof(received_relay_message.last_served_time);
	
	received_relay_message.last_served_basestation = (uint8_t)cntptr[0];
  	cntptr = cntptr + sizeof(received_relay_message.last_served_basestation);
}

size_t
FootbotRelay::get_data_from_missionagents(char *msg_ptr,uint8_t agent_id)
{
    DEBUGCOMM("Creating message to request data from agent\n");
    long unsigned int initial_address =  (long unsigned int)&(*msg_ptr);


	uint8_t identifier = 3;
	memcpy(msg_ptr,&identifier,sizeof(identifier));
	msg_ptr += sizeof(identifier);

	check_set.erase(agent_id);
	
	memcpy(msg_ptr,&agent_id,sizeof(agent_id));
	msg_ptr+= sizeof(agent_id);
    
	long unsigned int final_address =  (long unsigned int)&(*msg_ptr);

    return size_t(final_address-initial_address);
	
}


void 
FootbotRelay::ControlStep() 
{ 

	bool pass=true;
	
	if(changePos)
	{ 
		while(pass)
		{   
		int min = 1, max = 3;
		int random_integer = min + (rand() % (int)(max - min + 1));
		
		if(counter!=random_integer)
			{
				counter = random_integer;
				pass = false;
	    	}
		
		}
		cout <<"MyId: " << m_myID << "target: " << counter << endl;
		vector<double> tempBasePos = baseStationPosition[counter];
		relay_message.target_basestation = uint8_t(counter);
		CVector3 targetPos(tempBasePos[0], tempBasePos[1], 0);
		m_navClient->setTargetPosition(targetPos);
		/*if(counter == NumberOfBaseStation)
			{ counter = 1; }
		else
			{ counter = counter + 1; }*/
		changePos = false;
	}

	else if(m_navClient->state() == target_state)
	{
		changePos = true;
		relay_message.last_served_basestation = uint8_t(counter);
		relay_message.last_served_time = (uint64_t)getTime();
	}

	m_Steps+=1;

    /// Sending messages to other Relays
	if(m_Steps % 20 == 0)
	{
	  /// This profile message is sent to other relay robots using long communication range 
		char m_relaySocketMsg[MAX_UDP_SOCKET_BUFFER_SIZE];
		size_t psize = createProfileMessage(m_relaySocketMsg);
		DEBUGCOMM("Sending MSG of size %lu\n",psize); 
		
		//char *p = m_relaySocketMsg;
		/*for(int i=0;i < 41; i++)
		{
			printf("Sending %x\n",*p++);
		}*/
		
		m_pcWifiActuator->SendBinaryMessageTo_Local("-1",m_relaySocketMsg,psize); 
		

		//char data[9]={2,3,4,5,6,7,8,0,2};
		//m_pcWifiActuator->SendBinaryMessageTo("-1",data,9); 
	}
	
	TMessageList t_incomingMsgs;
    /// Receives Relay messages
	m_pcWifiSensor->GetReceivedMessages_Local(t_incomingMsgs);
	for(TMessageList::iterator it = t_incomingMsgs.begin(); it!=t_incomingMsgs.end();it++)
	{
		/// parse msg
		DEBUGCOMM("Received %lu bytes to incoming buffer\n", it->Payload.size());
		parse_relay_message(it->Payload);
	}

    
    /********  Send and Receive messages to and from mission agents ********/
	


	if(m_Steps % 10 == 0)
	{   
		char agent_socket_msg[20];
		size_t mes_size = createMessageToMissionAgents(agent_socket_msg);
		m_pcWifiActuator->SendBinaryMessageTo_Extern("-1",agent_socket_msg,mes_size);
	}	

   
	TMessageList message_from_agents;
	m_pcWifiSensor->GetReceivedMessages_Extern(message_from_agents);
	for(TMessageList::iterator it = message_from_agents.begin(); it!=message_from_agents.end();it++)
	{
		DEBUGCOMM("Received %lu bytes to incoming buffer\n", it->Payload.size());
		parse_agent_message(it->Payload);
		
	}

	
	
		for(size_t i=0;i<agent_ids.size();i++)
	{   
		uint8_t agent_id = agent_ids.front();
		agent_ids.pop();
		char data_request_message[MAX_UDP_SOCKET_BUFFER_SIZE];
		size_t mes_size = get_data_from_missionagents(data_request_message, agent_id);
        
        std::ostringstream str_tmp(ostringstream::out);
  		str_tmp << "fb_" << agent_id;
  		string str_Dest = str_tmp.str();

      	m_pcWifiActuator->SendBinaryMessageTo_Extern(str_Dest.c_str(),data_request_message,mes_size);
	}



	m_pcLEDs->SetAllColors(CColor::MAGENTA);
	m_navClient->setTime(getTime());
	m_navClient->update();
	cout <<"MyId: " << m_myID << "target: " << counter << endl;
	
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

