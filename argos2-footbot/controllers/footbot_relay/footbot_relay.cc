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
  uint32_t messageSize = 0;

  /// Identifier message source
  uint8_t srcIdentifier = (uint8_t)1;
  memcpy(outptr, &srcIdentifier, sizeof(srcIdentifier));
  outptr = outptr + sizeof(srcIdentifier);
  
  messageSize = messageSize + sizeof(srcIdentifier);
  printf("Sending message\n");

	/// Id of relay
	/// Id - uint8_t
	uint8_t relay_id = (uint8_t)m_myID;
    printf("%d\n",relay_id);
	memcpy(outptr, &relay_id, sizeof(relay_id));
	outptr = outptr + sizeof(relay_id);
	messageSize = messageSize + sizeof(relay_id);

	/// Current time - uint64
	uint64_t currTimestamp = (uint64_t)getTime();
	printf("Size Time Stamp %d\n",sizeof(currTimestamp));
	memcpy(outptr, &currTimestamp, sizeof(currTimestamp));
	outptr = outptr + sizeof(currTimestamp);
	messageSize = messageSize + sizeof(currTimestamp);

	/// Relay Position - (x,y in double)
	double relay_x = (double) m_navClient->currentPosition().GetX();
	double relay_y = (double) m_navClient->currentPosition().GetY();  
       
	memcpy(outptr, &relay_x, sizeof(relay_x));
	outptr = outptr + sizeof(relay_x);
	messageSize = messageSize + sizeof(relay_x);
  
	memcpy(outptr, &relay_y, sizeof(relay_y));
	outptr = outptr + sizeof(relay_y);
	messageSize = messageSize + sizeof(relay_y);
	return messageSize;
	/// Know Neighbour Info
	/// Later -- > Map(NeighId, NeighPos)
	//uint8_t neighbourId = getNeighbourInfo();
	/*memcpy(outptr, &neighbourRelay, sizeof(neighbourRelay));
	outptr = outptr + sizeof(neighbourRelay);
	messageSize = messageSize + sizeof(neighbourRelay); 
	printf("%d\n",sizeof(neighbourRelay));

	///current target base station - uint8
	targetBaseStation = (uint8_t)counter;
	memcpy(outptr, &targetBaseStation, sizeof(targetBaseStation));
	outptr = outptr + sizeof(targetBaseStation);
	messageSize = messageSize + sizeof(targetBaseStation);
	
	/// last service information
	//lastserviceInfo.lastserved_station_id = 2;
	/// get the time when the relay meets the base station
	//lastserviceInfo.previousTxTime = 0;
	/// position where it starts for the next target i.e the positon where it is considered to meet the last BS 
	//lastserviceInfo.initialX = 0.0;
	//lastserviceInfo.initialY = 0.0;
	memcpy(outptr, &lastserviceInfo, sizeof(lastserviceInfo));
	outptr = outptr + sizeof(lastserviceInfo);
	messageSize = messageSize + sizeof(lastserviceInfo); 
    printf("%s\n", outptr);*/
	*outptr = '\0';
	 outptr = outptr + 1;
	 messageSize = messageSize + 1;
	 return size_t(messageSize);
} 

void
FootbotRelay::parseMessage(std::vector<char> &m_incomingMsg)
{   

	uint32_t bcnt = 0;
	char *cntptr = (char*)&m_incomingMsg[0];
	constants::MessageSource msgSource;
    char *p = cntptr;
		for(int i=0;i < 26; i++)
		{
			printf("Received %x\n",*p++);
		}
	/// read identifier
	uint8_t srcIdentifier = (uint8_t)m_incomingMsg[0];
	//memcpy(&srcIdentifier, cntptr, sizeof(srcIdentifier));
	msgSource = static_cast<constants::MessageSource>(srcIdentifier);
	cntptr += sizeof(srcIdentifier);
	//bcnt += sizeof(srcIdentifier);
	printf("in parsing\n");
	cout << msgSource << endl;
	switch(msgSource)
	{
		
		case MessageSource::MISSION_AGENTS:
		{
			/// read id (1)
			/// #1:  id  - uint8_t
			uint8_t received_robot_id;
			memcpy(&received_robot_id, cntptr, sizeof(received_robot_id));
			cntptr += sizeof(received_robot_id);
			bcnt += sizeof(received_robot_id);
			DEBUGCOMM("Read id %d from msg\n", received_robot_id);
			
			///read timestamp (2)
			/// #2: curr_time - uint64_t
			uint64_t curr_time;
			memcpy(&curr_time, cntptr, sizeof(curr_time));
			cntptr +=sizeof(curr_time);
			bcnt += sizeof(curr_time);
			DEBUGCOMM("timestamp: %lud\n",curr_time);

			///read position x,y (3)
			/// x,y - double
			double x,y;
			memcpy(&x, cntptr, sizeof(x));
			cntptr +=sizeof(x);
			bcnt = bcnt + sizeof(x);
			memcpy(&y, cntptr, sizeof(y));
			cntptr = cntptr + sizeof(y);
			bcnt = bcnt + sizeof(y);
      		vector<double> temp_robot_position {x,y};
      
      		/// check if neighbour id is already present
      		/// update if it is present else insert
      		NeighbourMap::iterator itr = neighbourRobot.find(received_robot_id);
      		if ( itr == neighbourRobot.end() ) 
      		{
      			neighbourRobot.insert(pair<uint8_t, vector<double> >(received_robot_id,temp_robot_position));
  			} 
			else 
			{
  				itr->second = temp_robot_position;
			}

      
			///read last transmitted time (4)
			/// lasttr_time - uint64_t
			uint64_t lasttr_time;
			memcpy(&lasttr_time, cntptr, sizeof(lasttr_time));
			cntptr = cntptr + sizeof(lasttr_time);
			bcnt = bcnt + sizeof(lasttr_time);

			///read numberofNeighbours (5)
			/// numberofNeighbours  - uint8_t
			uint8_t numberofNeighbours;
			memcpy(&numberofNeighbours, cntptr, sizeof(numberofNeighbours));
			cntptr = cntptr + sizeof(numberofNeighbours);
			bcnt = bcnt + sizeof(numberofNeighbours);

			DEBUGCOMM("Received MSG of size %d\n", bcnt);
			break;
		}

	case MessageSource::RELAY:
	{       
		    
			/// read id (1)
			/// #1:  id  - uint8_t
			uint8_t received_relay_id = uint8_t(cntptr[0]);
			//memcpy(&received_relay_id, cntptr, sizeof(received_relay_id));
			cntptr += sizeof(received_relay_id);
			//bcnt += sizeof(received_relay_id);
			DEBUGCOMM("Read id %d from msg\n", received_relay_id);
			
			///read timestamp (2)
			/// #2: curr_time - uint64_t
			uint64_t r_curr_time;
			memcpy(&r_curr_time, cntptr, sizeof(r_curr_time));
			cntptr +=sizeof(r_curr_time);
			//bcnt += sizeof(r_curr_time);
			DEBUGCOMM("timestamp: %lud\n",r_curr_time);

			///read position x,y (3)
			/// x,y - double
			double r_x,r_y;
			memcpy(&r_x, cntptr, sizeof(r_x));
			cntptr +=sizeof(r_x);
			//bcnt = bcnt + sizeof(r_x);
			DEBUGCOMM("received_position_x: %f\n",r_x);
			memcpy(&r_y, cntptr, sizeof(r_y));
			cntptr = cntptr + sizeof(r_y);
			//bcnt = bcnt + sizeof(r_y);
	  		DEBUGCOMM("received_position_y: %f\n",r_x);

	  	/*	vector<double> temp_robot_position {r_x,r_y};
	  		/// check if neighbour id is already present
	  		/// update if it is present else insert
	  		NeighbourMap::iterator itr = neighbourRelay.find(received_relay_id);
	  		if ( itr == neighbourRelay.end() ) 
	  		{
	  			neighbourRelay.insert(pair<uint8_t, vector<double> >(received_relay_id,temp_robot_position));
			} 
			else 
			{
  				itr->second = temp_robot_position;
			}
			//DEBUGCOMM("neighbour_relay: %d\n",sizeof(neighbourRelay));
			///read neighbour info (4)
			
			//uint64_t r_neighbour_info; */
			/*memcpy(&received_neighbour_info, cntptr, sizeof(received_neighbour_info));
			cntptr = cntptr + sizeof(received_neighbour_info);
			bcnt = bcnt + sizeof(received_neighbour_info);
			DEBUGCOMM("neighbour_received: %lud\n",sizeof(received_neighbour_info));
			
			uint8_t received_target_basestation;
			memcpy(&received_target_basestation, cntptr, sizeof(received_target_basestation));
			cntptr = cntptr + sizeof(received_target_basestation);
			bcnt = bcnt + sizeof(received_target_basestation);
			DEBUGCOMM("target base station: %d\n",sizeof(received_target_basestation));


			///read last service info (5)
			memcpy(&received_lastserviceInfo, cntptr, sizeof(received_lastserviceInfo));
			cntptr = cntptr + sizeof(received_lastserviceInfo);
			bcnt = bcnt + sizeof(received_lastserviceInfo);*/

			//DEBUGCOMM("Received MSG of size %d\n", bcnt);
			break;
    }
			default:
		{
			cout << "Not a valid message source" << endl;
		}
	 
	}
}

	void 
FootbotRelay::ControlStep() 
{ 

	bool pass =true;
	
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
		lastserviceInfo.lastserved_station_id = counter;
		lastserviceInfo.previousTxTime = getTime();
	}

	m_Steps+=1;
 
	if(m_Steps % 20 == 0)
	{
	  /// This profile message is sent to other relay robots using long communication range 
		char m_relaySocketMsg[MAX_UDP_SOCKET_BUFFER_SIZE];
		size_t psize = createProfileMessage(m_relaySocketMsg);
		DEBUGCOMM("Sending MSG of size %lu\n",psize); 
		
		char *p = m_relaySocketMsg;
		for(int i=0;i < 26; i++)
		{
			printf("Sending %x\n",*p++);
		}
		
		m_pcWifiActuator->SendBinaryMessageTo("-1",m_relaySocketMsg,27); 

		//char data[9]={2,3,4,5,6,7,8,0,2};
		//m_pcWifiActuator->SendBinaryMessageTo("-1",data,9); 
	}
	
	TMessageList t_incomingMsgs;

	m_pcWifiSensor->GetReceivedMessages(t_incomingMsgs);
	for(TMessageList::iterator it = t_incomingMsgs.begin(); it!=t_incomingMsgs.end();it++)
		{
			/// parse msg
			//std::copy(it->Payload.begin(),it->Payload.end() , m_incomingMsg);
			DEBUGCOMM("Received %lu bytes to incoming buffer\n", it->Payload.size());
			parseMessage(it->Payload);
			//parseMessage(it->Payload.size());
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

