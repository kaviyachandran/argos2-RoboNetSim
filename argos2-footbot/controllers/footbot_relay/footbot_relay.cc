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
	counter(1),
	NumberOfBaseStation(4),
	changePos(true),
	target_state(STATE_ARRIVED_AT_TARGET),
	min(1),
	neighbour_count(0),
	max_agent_range(2.0),
	calculate_pos(true),
	send_message_to_relay(false)   //// Positions for 3 minutes -> 3*60*20 but positions 
	{                                    //// are calculated only every 10 timestep

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
	GetNodeAttributeOrDefault(t_node, "Agentrange", max_agent_range, max_agent_range);
	
	/// Stores Base Station positions
	for (size_t i=1; i <= NumberOfBaseStation ; i++)
	{
	 	string str = "BaseStation" + to_string(i);
		TConfigurationNode node = GetNode(t_node, str);	
		
		vector<double> basePos = getBaseStationPositions(node);
		baseStationPosition.insert(pair<uint8_t, vector<double> >(i,basePos));
 		
	}
    
    TConfigurationNode node = GetNode(t_node, "Length");
    length_size = getValues(node,max_agent_range);
    node = GetNode(t_node, "Breadth");
    breadth_size = getValues(node,max_agent_range);
    

    
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
    

	// file to save the data
  	filename = "data_"+ to_string(m_myID)+".csv";
  	data_file.open(filename, ios::out | ios::ate | ios::app);
    
    received_file = "received_data_"+ to_string(m_myID)+".csv";
  	received_message_file.open(received_file, ios::out | ios::ate | ios::app);
  	received_message_file << "Time_step" << "," << "Message_type" << "," <<"Received_From"  << "," << "Position" << "\n"; 
 
  	meeting_file = "meeting_data_"+to_string(m_myID)+".csv";
  	meeting_data_file.open(meeting_file, ios::out | ios::ate | ios::app);
    
  	meeting_data_file << "Timestep" << "," << "Starting point"<< "," << "Target"<< "," << "Agents" << "," << "Reached_timestep" << "\n";
 	
 	target_odd = {1,3};
 	target_even = {2,4}; 

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

vector<double> 
FootbotRelay::getValues(TConfigurationNode node, double agent_range)
{ 
	vector<double> size;
	string temp;
	GetNodeAttribute(node, "origin", temp);
	size.push_back(stod(temp)+agent_range);
	GetNodeAttribute(node, "destination", temp);
	size.push_back(stod(temp)-agent_range);
	return size;
}

uint8_t FootbotRelay::getNeighbourInfo()
{
	return 0;
}

size_t FootbotRelay::createProfileMessage(char *outptr)
{   
	long unsigned int initial_address =  (long unsigned int)&(*outptr);
  	printf("sending message to relay \n");
    //printf("beginning %lu\n", initial_address);

    char message_type = 'r';
    memcpy(outptr, &message_type, sizeof(message_type));
    outptr = outptr + sizeof(message_type);
    //printf("%c\n", message_type);

    memcpy(outptr, &relay_message.message_size, sizeof(relay_message.message_size));
    outptr = outptr + sizeof(relay_message.message_size); 
    
   /// Id of relay
   /// Id - uint8_t
	relay_message.relay_id = (uint8_t)m_myID;
    memcpy(outptr, &relay_message.relay_id, sizeof(relay_message.relay_id));
	outptr = outptr + sizeof(relay_message.relay_id);
	//printf("Relay Id %d\n", relay_message.relay_id);

	/// Relay Position - (x,y in double)
	relay_message.relay_current_x = (double) m_navClient->currentPosition().GetX();
	relay_message.relay_current_y = (double) m_navClient->currentPosition().GetY();  
       
	memcpy(outptr, &relay_message.relay_current_x, sizeof(relay_message.relay_current_x));
	outptr = outptr + sizeof(relay_message.relay_current_x);
	
  	memcpy(outptr, &relay_message.relay_current_y, sizeof(relay_message.relay_current_y));
	outptr = outptr + sizeof(relay_message.relay_current_y);
	
	//printf("Relay pos x %f\n", relay_message.relay_current_x);
	//printf("Relay pos y %f\n", relay_message.relay_current_y);

	/// Current time - uint64
	relay_message.time_message_sent = (uint64_t)getTime();
	memcpy(outptr, &relay_message.time_message_sent, sizeof(relay_message.time_message_sent));
	outptr = outptr + sizeof(relay_message.time_message_sent);
	//printf("time mes sent %u\n", relay_message.time_message_sent);

	
	// neighbours- uint8_t
	relay_message.number_neighbors = (uint8_t)neighbour_count;
	memcpy(outptr, &relay_message.number_neighbors, sizeof(relay_message.number_neighbors));
	outptr = outptr + sizeof(relay_message.number_neighbors);
    //printf("Neighbour %d\n", relay_message.number_neighbors);

	memcpy(outptr, &relay_message.target_basestation, sizeof(relay_message.target_basestation));
	outptr = outptr + sizeof(relay_message.target_basestation);
	//printf("target BS %d\n", relay_message.target_basestation);

	memcpy(outptr, &relay_message.last_served_time, sizeof(relay_message.last_served_time));
	outptr = outptr + sizeof(relay_message.last_served_time);
	//printf("Last srerved time %u\n", relay_message.last_served_time);

    memcpy(outptr, &relay_message.last_served_basestation, sizeof(relay_message.last_served_basestation));
	outptr = outptr + sizeof(relay_message.last_served_basestation);
	//printf("last served BS %d\n", relay_message.last_served_basestation);
	

	long unsigned int final_address = (long unsigned int)&(*outptr);

	relay_message.message_size = final_address-initial_address;
	//printf("Size of message sent: %d\n",relay_message.message_size);
	return size_t(relay_message.message_size);
} 

size_t 
FootbotRelay::createMessageToMissionAgents(char* out_to_agent)
{ 
  long unsigned int initial_address =  (long unsigned int)&(*out_to_agent);
  
  /// identifier - 10 (relay) 0(agent) to identify relay message 
  char identifier = 'c'; 
  DEBUGCOMM("Size of character %d\n", sizeof(identifier));
  memcpy(out_to_agent,&identifier,sizeof(identifier));
  out_to_agent = out_to_agent + sizeof(identifier);
  
  uint8_t id = (uint8_t)m_myID;
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
  
  char mes_type = (char)agent_mes_ptr[0];
  agent_mes_ptr = agent_mes_ptr + sizeof(mes_type);

  if(mes_type == 'a')
  {  
    neighbour_count = neighbour_count + 1;
    

  	//DEBUGCOMM("profile message from agent\n");

  	memcpy(&agent_message.message_size, agent_mes_ptr, sizeof(agent_message.message_size));
    agent_mes_ptr = agent_mes_ptr+sizeof(agent_message.message_size);

  	// Agent id
  	agent_message.agent_id = agent_mes_ptr[0];
  	agent_mes_ptr = agent_mes_ptr + sizeof(agent_message.agent_id);
  	
  	//DEBUGCOMM("agent id %d\n",agent_message.agent_id);

  	if(check_set.find(agent_message.agent_id) == check_set.end())
  	{
  		check_set.insert(agent_message.agent_id);
  		agent_ids.push(agent_message.agent_id);
  	}
  	
    
    // Time when message is sent
   
    memcpy(&agent_message.time_message_sent, agent_mes_ptr, sizeof(agent_message.time_message_sent));
    //DEBUGCOMM("time sent %u\n",agent_message.time_message_sent);
    agent_mes_ptr+=sizeof(agent_message.time_message_sent);

    // Agent pos
    memcpy(&agent_message.agent_current_x, agent_mes_ptr, sizeof(agent_message.agent_current_x));
    agent_mes_ptr+= sizeof(agent_message.agent_current_x);

    memcpy(&agent_message.agent_current_y, agent_mes_ptr, sizeof(agent_message.agent_current_y));
    agent_mes_ptr+= sizeof(agent_message.agent_current_y);
    
    //printf("agent pos x %f\n", agent_message.agent_current_x);
	//printf("AGENT pos y %f\n", agent_message.agent_current_y);


    // last time when hte data is transmitted
    memcpy(&agent_message.time_last_data_transmitted,agent_mes_ptr,sizeof(agent_message.time_last_data_transmitted));
    agent_mes_ptr+= sizeof(agent_message.time_last_data_transmitted);
    //DEBUGCOMM("LAST DATA transmitted %u\n",agent_message.time_last_data_transmitted);

    // number of neighbours
    agent_message.number_neighbors = (uint8_t)agent_mes_ptr[0];
    agent_mes_ptr = agent_mes_ptr+ sizeof(agent_message.number_neighbors);
    //DEBUGCOMM("number of neighbours for agent %d\n",agent_message.number_neighbors);

    //timestep
    memcpy(&agent_message.timestep, agent_mes_ptr, sizeof(agent_message.timestep));
    agent_mes_ptr = agent_mes_ptr + sizeof(agent_message.timestep);
    //DEBUGCOMM("timestep %d\n", agent_message.timestep);

    // future position
    memcpy(&agent_message.target_pos_x, agent_mes_ptr, sizeof(agent_message.target_pos_x));
    agent_mes_ptr = agent_mes_ptr + sizeof(agent_message.target_pos_x);
    //DEBUGCOMM("target X %d\n", agent_message.target_pos_x);

    memcpy(&agent_message.target_pos_y, agent_mes_ptr, sizeof(agent_message.target_pos_y));
    agent_mes_ptr = agent_mes_ptr + sizeof(agent_message.target_pos_y);
    //DEBUGCOMM("target Y %d\n", agent_message.target_pos_y);
    
    // amount of data available from the agent
    memcpy(&agent_message.data_available, agent_mes_ptr, sizeof(agent_message.data_available));
    agent_mes_ptr = agent_mes_ptr + sizeof(agent_message.data_available);
    DEBUGCOMM("Amount of data available %lu\n", agent_message.data_available);
    
    DEBUGCOMM("done parsing agent message\n");
	
  }
  else if(mes_type =='b')
  { 
    uint8_t temp_id;
    memcpy(&temp_id,agent_mes_ptr,sizeof(temp_id));
    agent_mes_ptr = agent_mes_ptr + sizeof(temp_id);

    memcpy(&agent_data.time_data_sent, agent_mes_ptr, sizeof(agent_data.time_data_sent));
    agent_mes_ptr = agent_mes_ptr + sizeof(agent_data.time_data_sent);

	neighbour_count = neighbour_count + 1;
  	DEBUGCOMM("Received data from agents\n");
      /// Receiving and storing data
  	 
  	memcpy(&agent_data.data_size,agent_mes_ptr,sizeof(agent_data.data_size));
  	agent_mes_ptr = agent_mes_ptr + sizeof(agent_data.data_size);
  	
  	DEBUGCOMM("Relay knows the size of data: %d sent \n",(agent_data.data_size));
  	/// -5 is 4 - uint32 message size and 1 - uint8 for message type identifier
  	//char data_collected[(agent_data.data_size)];
  	//memcpy(&data_collected, agent_mes_ptr, sizeof(data_collected));
  	
  	//unsigned arr_size = sizeof(data_collected)/sizeof(char);
  	 
  	//agent_data.fake_data.insert(agent_data.fake_data.end(), &data_collected[0], &data_collected[arr_size]);
    
    map<uint8_t, vector<Agent_data_info> >::iterator itr = data_from_agents.find(temp_id);
	
	if(itr != data_from_agents.end())
	{  
   		data_from_agents[temp_id].push_back(agent_data);		
	}
	else
	{ 
  		vector<Agent_data_info> data_info;
    	data_info.push_back(agent_data);
  		data_from_agents.insert(std::pair <uint8_t, std::vector<Agent_data_info> >(temp_id, data_info));
	}
  	

  }

  else if(mes_type == 'm')
  {
  	neighbour_count = neighbour_count + 1;
  }
}

void
FootbotRelay::parse_relay_message(std::vector<char> &incomingMsg)
{   

	char *cntptr = (char*)&incomingMsg[0];
    
	char mes_type = (char)cntptr[0];
	cntptr = cntptr + sizeof(mes_type);
    
    if(mes_type == 'r')
    {
	/// read size
		memcpy(&received_relay_message.message_size, cntptr, sizeof(received_relay_message.message_size));
		cntptr += sizeof(received_relay_message.message_size);
		printf("Received message size %d\n", received_relay_message.message_size);
		
		/// read id (1)
		/// #1:  id  - uint8_t
		received_relay_message.relay_id = uint8_t(cntptr[0]);
		cntptr += sizeof(received_relay_message.relay_id);
		//DEBUGCOMM("Read id %d from msg\n", received_relay_message.relay_id);
				
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
}

size_t
FootbotRelay::get_data_from_missionagents(char *msg_ptr)
{
    DEBUGCOMM("Creating message to request data from agent\n");
    long unsigned int initial_address =  (long unsigned int)&(*msg_ptr);

	char identifier = 'd';
	memcpy(msg_ptr,&identifier,sizeof(identifier));
	msg_ptr += sizeof(identifier);

	uint8_t id_relay = m_myID;
	memcpy(msg_ptr,&id_relay,sizeof(id_relay));
	msg_ptr+= sizeof(id_relay);
    
	long unsigned int final_address =  (long unsigned int)&(*msg_ptr);

    return size_t(final_address-initial_address);
	
}

size_t
FootbotRelay::send_collected_data(char* data_to_basestation_ptr)
{

long unsigned int initial_address =  (long unsigned int)&(*data_to_basestation_ptr);

char identifier = 'x';
memcpy(data_to_basestation_ptr, &identifier, sizeof(identifier));
data_to_basestation_ptr = data_to_basestation_ptr + sizeof(identifier);
DEBUGCOMM("Identifier data to base station %c\n", identifier);

memcpy(data_to_basestation_ptr, &data_to_BS_size, sizeof(data_to_BS_size));
data_to_basestation_ptr = data_to_basestation_ptr + sizeof(data_to_BS_size);

uint8_t relay_id = m_myID;
memcpy(data_to_basestation_ptr, &relay_id, sizeof(relay_id));
data_to_basestation_ptr = data_to_basestation_ptr + sizeof(relay_id);
DEBUGCOMM("Relay Id %d\n", relay_id);

uint8_t map_size = data_from_agents.size();
memcpy(data_to_basestation_ptr, &map_size, sizeof(map_size));
data_to_basestation_ptr = data_to_basestation_ptr + sizeof(map_size);
DEBUGCOMM("Number of agents %d\n", map_size);

for (std::map<uint8_t,vector<Agent_data_info> >::iterator it=data_from_agents.begin(); it!=data_from_agents.end(); ++it)
{
	uint8_t id = it->first;
	memcpy(data_to_basestation_ptr, &id, sizeof(id));
	data_to_basestation_ptr = data_to_basestation_ptr + sizeof(id);
	DEBUGCOMM("Agent Id %d\n", id);

	vector<Agent_data_info> &temp_info = it->second;
    
    uint8_t vec_size = temp_info.size();
	memcpy(data_to_basestation_ptr, &vec_size, sizeof(vec_size));
	data_to_basestation_ptr = data_to_basestation_ptr + sizeof(vec_size);
	DEBUGCOMM("Number of times data exchanged with the same agent %d\n", vec_size);

	for(int i=0; i < vec_size; i++)
	{
		memcpy(data_to_basestation_ptr, &temp_info[i].time_data_sent, sizeof(temp_info[i].time_data_sent));
		data_to_basestation_ptr = data_to_basestation_ptr + sizeof(temp_info[i].time_data_sent);
		DEBUGCOMM("Time step data exchanged %d\n", temp_info[i].time_data_sent);

		memcpy(data_to_basestation_ptr, &temp_info[i].data_size, sizeof(temp_info[i].data_size));
		data_to_basestation_ptr = data_to_basestation_ptr + sizeof(temp_info[i].data_size);
		DEBUGCOMM("size of data exchanged %d\n", temp_info[i].data_size);

		//memcpy(data_to_basestation_ptr, temp_info[i].fake_data.data(), sizeof(uint8_t)*(temp_info[i].fake_data.size()));
		//data_to_basestation_ptr = data_to_basestation_ptr + sizeof(uint8_t)*(temp_info[i].fake_data.size());
	}
}

long unsigned int final_address =  (long unsigned int)&(*data_to_basestation_ptr);
data_to_BS_size = uint32_t(final_address - initial_address);
cout << "data size " << data_to_BS_size << endl; 
DEBUGCOMM("total message size %d\n", data_to_BS_size);
return size_t(data_to_BS_size);
}

deque<double>
FootbotRelay::calculate_target(vector<double> pos, bool d)
{   
	double x = 0.0,y = 0.0;
    double curr_x = m_navClient->currentPosition().GetX();
    double curr_y = m_navClient->currentPosition().GetY();


	cout << "x " << curr_x << endl;
 	cout << "y " << curr_y << endl;
    
    deque<double> temp_pos;

    if(curr_y < 0)
		{
			y = breadth_size[1];
		}
		else
		{
			y = breadth_size[0];
		}
	
	if(d) // moving in positive direction
	{   
		x = curr_x + max_agent_range;
		
		cout << "x " << x << endl;
		cout << "y " << y << endl;

		temp_pos.push_back(x);
		temp_pos.push_back(y);

		while(x < length_size[1])
		{
			if(y == breadth_size[0])
 			{
  				y = breadth_size[1];
 			}
 			else 
 			{
    			y = breadth_size[0];
 			}
 			
 			double temp = x + max_agent_range;
  			if (temp < length_size[1])
  			{
    			x = temp;
  			}
  			else
  			{
    			x = length_size[1];
  			}
 			
 			cout << "x " << x << endl;
			cout << "y " << y << endl;
  			
  			temp_pos.push_back(x);
  			temp_pos.push_back(y);

 		}
	}
	else
	{
		x = curr_x - max_agent_range;

		cout << "x " << x << endl;
		cout << "y " << y << endl;
		
		temp_pos.push_back(x);
		temp_pos.push_back(y);

		while(x > length_size[0])
		{
			if(y == breadth_size[0])
 			{
  				y = breadth_size[1];
 			}
 			else 
 			{
    			y = breadth_size[0];
 			}
 			
 			double temp = x - max_agent_range;
  			if (temp > length_size[0])
  			{
    			x = temp;
  			}
  			else
  			{
    			x = length_size[0];
  			}

  			cout << "x " << x << endl;
			cout << "y " << y << endl;
  			
  			temp_pos.push_back(x);
  			temp_pos.push_back(y);

 		}
	}
	cout << "x " << pos[0] << endl;
	cout << "y " << pos[1] << endl;
	
	temp_pos.push_back(pos[0]);
	temp_pos.push_back(pos[1]);
    
    cout << "Size of pos " << temp_pos.size() << endl;

	return temp_pos;
}

void 
FootbotRelay::ControlStep() 
{   
	bool send_message_to_basestation = false;
	send_message_to_relay = false;
    if(m_Steps % 5 == 0 && m_Steps > 2)
		/*** Saving the waypoints ***/
		data_file << m_navClient->currentPosition().GetX() << "," << m_navClient->currentPosition().GetY() << "\n";
    
    vector<double> tempBasePos;
    bool pass=true;
	neighbour_count = 0;

	/***** Assigning target Position  *****/
   if(calculate_pos && m_Steps >= 5 )
   	{ 		
   		/*while(pass)
		{   
		//cout << "position " << m_navClient->currentPosition().GetX() << "," << m_navClient->currentPosition().GetY() << endl;
		int min = 0, max = 1;
		int random_integer = min + (rand() % (int)(max - min + 1));
		//cout << "time " << m_Steps << endl; 
		if(counter!=random_integer)
			{
				counter = random_integer;
				pass = false;
	    	}
		
		}*/

		if (m_myID%2 == 0)
		{   
			if(counter < 0)
			{
				counter = 1;
			}
			tempBasePos = baseStationPosition[target_even[counter]];
			relay_message.target_basestation = uint8_t(target_even[counter]);
			relay_message.last_served_basestation = uint8_t(target_even[counter]);
			cout << "target number" << target_even[counter] << endl;
			counter = counter - 1;
			
			cout <<"MyId: " << m_myID << "target: " << relay_message.target_basestation << endl;
		}
        else
        {	
        	if(counter < 0 )
			{
				counter = 1;
			}
        	tempBasePos = baseStationPosition[target_odd[counter]];
        	relay_message.target_basestation = uint8_t(target_odd[counter]);
        	relay_message.last_served_basestation = uint8_t(target_odd[counter]);
        	cout << "target number" << target_odd[counter] << endl;
        	counter = counter - 1;
        	
        	cout <<"MyId: " << m_myID << "target: " << relay_message.target_basestation << endl;
        }
        	if(tempBasePos[0] < 0)
			{
				direction = false;
			}
			else
			{
				direction = true;
			}
			relay_target_positions = calculate_target(tempBasePos,direction);
			calculate_pos = false;
   	} 
   
	else 
	{
		if(changePos)
		{   

			tempBasePos.clear();
			tempBasePos.push_back(relay_target_positions[0]);
			relay_target_positions.pop_front();
			tempBasePos.push_back(relay_target_positions[0]);
			relay_target_positions.pop_front();

			meeting_data_file << m_Steps << "," << m_navClient->currentPosition().GetX() << " " << m_navClient->currentPosition().GetY() << "," << tempBasePos[0] << " "<<tempBasePos[1] << ",";
			CVector3 targetPos(tempBasePos[0], tempBasePos[1], 0);
			m_navClient->setTargetPosition(targetPos);
			changePos = false;
	 	}
 	
		else if(m_navClient->state() == target_state)
		{   
			changePos = true;
			
			if(relay_target_positions.empty())
			{
				//wait_time = 5;
				send_message_to_basestation = true;
				//cout << "Reached BaseStation " << targetPos[0] << " " << targetPos[1] << endl;
				calculate_pos = true;
				relay_message.last_served_time = (uint64_t)getTime();
			}
			else
			{
				cout << "[" << m_myID << "]" << "Size of the target queue " << relay_target_positions.size() << endl; 
			}
		}

	}
    

    if(send_message_to_basestation)
    {   
    	//cout << "[" << m_myID << "]" << "Before sending data to base station " << endl; 
    	send_message_to_relay = true;
		meeting_data_file <<"," << m_Steps << "\n";
		//agents.clear();
		//data_exchange_agents.clear();
		changePos = true;
		visited_agents.clear();
        
        if(data_from_agents.size() > 0)
        {
			// sending collected data to Base Station
			char base_socket_msg[MAX_UDP_SOCKET_BUFFER_SIZE*2];
			size_t collected_data_size = send_collected_data(base_socket_msg); 
			
			std::ostringstream str_tmp(ostringstream::out);
	  		str_tmp << "fb_" << relay_message.target_basestation;
	  		string str_Dest = str_tmp.str();

	  		cout << "Base Station Target " << str_Dest << endl;
			m_pcWifiActuator->SendBinaryMessageTo_Extern(str_Dest.c_str(),base_socket_msg,collected_data_size);
			data_from_agents.clear();

			//char *p = base_socket_msg;
			/*for(int i=0;i < collected_data_size; i++)
			{
				printf("Sending message to Base Station %x\n",*p++);
			}*/

		}
		send_message_to_basestation = false;
    }
	

    /********  Send and Receive messages to and from mission agents ********/
	


	if(m_Steps % 10 == 0)
	{   
		char agent_socket_msg[20];
		size_t mes_size = createMessageToMissionAgents(agent_socket_msg);
		m_pcWifiActuator->SendBinaryMessageTo_Extern("-1",agent_socket_msg,mes_size);

		/*char *p = agent_socket_msg;
		for(int i=0;i < 2; i++)
		{
			printf("Sending message to agent %x\n",*p++);
		}*/
	}	

   
	TMessageList message_from_agents;
	m_pcWifiSensor->GetReceivedMessages_Extern(message_from_agents);
	for(TMessageList::iterator it = message_from_agents.begin(); it!=message_from_agents.end();it++)
	{   
		send_message_to_relay = true;
		DEBUGCOMM("Received %lu bytes to incoming buffer\n", it->Payload.size());

		vector<char> check_message = it->Payload;
		DEBUGCOMM("Identifier of received message [extern] %c\n",char(check_message[0]));
		parse_agent_message(it->Payload);
		string sender = it->Sender;
		if((char)check_message[0] == 'a')
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
		}
	} 

	
	
		for(size_t i=0;i<agent_ids.size();i++)
	{   
		uint8_t agent_id = agent_ids.front();
		
		if (std::find(visited_agents.begin(), visited_agents.end(),agent_id)==visited_agents.end())
		    {
		    	visited_agents.push_back(agent_id);
				char data_request_message[MAX_UDP_SOCKET_BUFFER_SIZE];
				size_t mes_size = get_data_from_missionagents(data_request_message);
		        
		        std::ostringstream str_tmp(ostringstream::out);
		  		str_tmp << "fb_" << agent_id;
		  		string str_Dest = str_tmp.str();

		      	m_pcWifiActuator->SendBinaryMessageTo_Extern(str_Dest.c_str(),data_request_message,mes_size);
			}
		agent_ids.pop();
		check_set.erase(agent_id);
	}


     /// Sending messages to other Relays
	if(send_message_to_relay)
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
		
		send_message_to_relay = false;
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
		vector<char> check_message = it->Payload;
		DEBUGCOMM("Identifier of received message [local] %c\n",char(check_message[0]));
		if((char)check_message[0] == 'r')
			parse_relay_message(it->Payload);
		
	} 



	m_pcLEDs->SetAllColors(CColor::MAGENTA);
	m_navClient->setTime(getTime());
	m_navClient->update();
	//cout <<"MyId: " << m_myID << "target: " << counter << endl;
	m_Steps+=1;
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

