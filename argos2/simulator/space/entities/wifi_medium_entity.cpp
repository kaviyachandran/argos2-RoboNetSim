/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#include "wifi_medium_entity.h"
//#include <argos2/simulator/space/entities/composable_entity.h>
//#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/space.h>
#include <argos2/simulator/space/entities/wifi_equipped_entity.h>
#include <argos2/simulator/physics_engines/physics_engine.h>

namespace argos {

  /****************************************/
  /****************************************/

  void error(const char *msg)
  {
    perror(msg);
    exit(-1);
  }

  /************************************************/
  /************************************************/

  SInt32 OpenBlockingUDPSocketServerSide(TSocket &t_new_socket, const UInt32 un_port)
  {
    if( (t_new_socket.sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
      error("Opening socket");

    t_new_socket.addr_len = sizeof(struct sockaddr_in);
    bzero(&t_new_socket.server, t_new_socket.addr_len);

    t_new_socket.server.sin_family      = AF_INET;
    t_new_socket.server.sin_addr.s_addr = INADDR_ANY;
    t_new_socket.server.sin_port        = htons(un_port);

    if ( bind(t_new_socket.sock,(struct sockaddr *)&t_new_socket.server, t_new_socket.addr_len) < 0 )
      error("Binding");

    return t_new_socket.sock;
  }

  /************************************************/
  /************************************************/

  SInt32 OpenBlockingUDPSocketClientSide(TSocket &t_new_socket, const UInt32 un_port, const char *p_server_host)
  {
    if( (t_new_socket.sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
      error("Opening socket");

    t_new_socket.server.sin_family = AF_INET;

    if ( (t_new_socket.hp = gethostbyname(p_server_host)) == 0 )
      error("Unknown host");

    bcopy((char *)t_new_socket.hp->h_addr, (char *)&t_new_socket.server.sin_addr, t_new_socket.hp->h_length);
    t_new_socket.server.sin_port = htons(un_port);

    t_new_socket.addr_len = sizeof(struct sockaddr_in);

    return t_new_socket.sock;
  }

  /************************************************/
  /************************************************/

  SInt32 ServerSendDataToSocket(TSocket &t_socket, const char *p_msg, const SInt32 n_msg_size)
  {
    //fprintf(stderr, "Server sending %d bytes to socket %d\n", msg_size, socket.sock);
    SInt32 unR = sendto(t_socket.sock, p_msg, n_msg_size, 0, (struct sockaddr *)&t_socket.client, t_socket.addr_len);
    if( unR != n_msg_size )
      error("Server send");
    return unR;
  }

  /************************************************/
  /************************************************/

  SInt32 ClientSendDataToSocket(TSocket &t_socket, const char *p_msg, const SInt32 n_msg_size)
  {
    //fprintf(stderr, "Client sending %d bytes to socket %d\n", msg_size, socket.sock);
    SInt32 nR = sendto(t_socket.sock, p_msg, n_msg_size, 0, (struct sockaddr *)&t_socket.server, t_socket.addr_len);
    if( nR != n_msg_size )
      error("Client send");
    return nR;
  }

  /************************************************/
  /************************************************/

  SInt32 ServerReceiveDataFromSocketNonBlocking(TSocket &t_socket, char **p_msg, SInt32 &n_msg_size, const SInt32 n_max_data)
  {
    //fprintf(stderr, "Server On receive max %d bytes from socket %d\n", max_data, socket.sock);

    n_msg_size = recvfrom(t_socket.sock, *p_msg, n_max_data, MSG_DONTWAIT, (struct sockaddr *)&t_socket.client, (socklen_t *)&t_socket.addr_len);

    if (n_msg_size <= 0)
      n_msg_size = 0;
    else
      {
	//fprintf(stderr, "Received %d bytes from socket %d\n", msg_size, socket.sock);
      }

    return n_msg_size;
  }

  /************************************************/
  /************************************************/

  SInt32 ServerReceiveDataFromSocketNonBlocking(TSocket &socket, char *msg, SInt32 &msg_size, const SInt32 max_data)
  {
    //fprintf(stderr, "Server On receive max %d bytes from socket %d\n", max_data, socket.sock);

    msg_size = recvfrom(socket.sock, msg, max_data, MSG_DONTWAIT, (struct sockaddr *)&socket.client, (socklen_t *)&socket.addr_len);

    if (msg_size <= 0)
      msg_size = 0;
    else
      {
	//fprintf(stderr, "Received %d bytes from socket %d\n", msg_size, socket.sock);
      }

    return msg_size;
  }

  /************************************************/
  /************************************************/

  SInt32 ServerPeekDataFromSocketBlocking(TSocket &socket, char *msg, SInt32 &msg_size, const SInt32 max_data)
  {
    //fprintf(stderr, "Server peek max %d bytes from socket %d\n", max_data, socket.sock);

    msg_size = recvfrom(socket.sock, msg, max_data, MSG_PEEK, (struct sockaddr *)&socket.client, (socklen_t *)&socket.addr_len);

    if (msg_size < 0)
      error("Server Recvfrom peek");

    //fprintf(stderr, "[Peek] Received %d bytes from socket %d\n", msg_size, socket.sock);

    return msg_size;
  }

  /************************************************/
  /************************************************/

  SInt32 ServerPeekDataFromSocketNonBlocking(TSocket &socket, char *msg, SInt32 &msg_size, const SInt32 max_data)
  {
    //fprintf(stderr, "Server peek max %d bytes from socket %d\n", max_data, socket.sock);

    msg_size = recvfrom(socket.sock, msg, max_data, MSG_PEEK | MSG_DONTWAIT, (struct sockaddr *)&socket.client, (socklen_t *)&socket.addr_len);

    if (msg_size <= 0)
      msg_size = 0;
    else
      {
	//fprintf(stderr, "[Peek] Received %d bytes from socket %d\n", msg_size, socket.sock);
      }

    return msg_size;
  }

  /************************************************/
  /************************************************/

  SInt32 ServerReceiveFixedDataLenFromSocketBlocking(TSocket &socket, char *msg, SInt32 &msg_size, const SInt32 data_len)
  {
    //fprintf(stderr, "Server On receive %d bytes from socket %d\n", data_len, socket.sock);

    msg_size = recvfrom(socket.sock, msg, data_len, MSG_WAITALL, (struct sockaddr *)&socket.client, (socklen_t *)&socket.addr_len);

    if (msg_size < 0)
      error("Server Recvfrom fixed");

    //fprintf(stderr, "Received %d bytes from socket %d\n", msg_size, socket.sock);

    return msg_size;
  }

  /************************************************/
  /************************************************/

  SInt32 ServerReceiveDataFromSocketBlocking(TSocket &socket, char **msg, SInt32 &msg_size, const SInt32 max_data)
  {
    //fprintf(stderr, "Server On receive max %d bytes from socket %d\n", max_data, socket.sock);

    msg_size = recvfrom(socket.sock, *msg, max_data, 0, (struct sockaddr *)&socket.client, (socklen_t *)&socket.addr_len);

    if (msg_size < 0)
      error("Server Recvfrom");

    //fprintf(stderr, "Received %d bytes from socket %d\n", msg_size, socket.sock);
    return msg_size;
  }

  /************************************************/
  /************************************************/

  SInt32 ServerReceiveDataFromSocketBlocking(TSocket &socket, char *msg, SInt32 &msg_size, const SInt32 max_data)
  {

    //fprintf(stderr, "On receive max %d bytes from socket %d\n", max_data, socket.sock);


    msg_size = recvfrom(socket.sock, msg, max_data, 0, (struct sockaddr *)&socket.client, (socklen_t *)&socket.addr_len);

    if (msg_size < 0)
      error("Server Recvfrom");

    //fprintf(stderr, "Received %d bytes from socket %d\n", msg_size, socket.sock);
    return msg_size;
  }

  /************************************************/
  /************************************************/

  SInt32 ClientReceiveDataFromSocketNonBlocking(TSocket &socket, 
						char **msg, 
						SInt32 &msg_size, 
						const SInt32 max_data)
  {

    //fprintf(stderr, "Client On receive max %d bytes from socket %d\n", max_data, socket.sock);

    struct sockaddr from;

    msg_size = recvfrom(socket.sock, *msg, max_data, MSG_DONTWAIT, (struct sockaddr *)&from, (socklen_t *)&socket.addr_len);

    if (msg_size <= 0)
      msg_size = 0;
    else
      {
	//fprintf(stderr, "Received %d bytes from socket %d\n", msg_size, socket.sock);
      }

    return msg_size;
  }

  /************************************************/
  /************************************************/

  SInt32 ClientReceiveDataFromSocketNonBlocking(TSocket &socket, 
						char *msg, 
						SInt32 &msg_size, 
						const SInt32 max_data)
  {

    //fprintf(stderr, "Client On receive max %d bytes from socket %d\n", max_data, socket.sock);

    struct sockaddr from;

    msg_size = recvfrom(socket.sock, msg, max_data, MSG_DONTWAIT, (struct sockaddr *)&from, (socklen_t *)&socket.addr_len);

    if (msg_size <= 0)
      msg_size = 0;
    else
      {
	//fprintf(stderr, "Received %d bytes from socket %d\n", msg_size, socket.sock);
      }

    return msg_size;
  }

  /************************************************/
  /************************************************/

  SInt32 ClientReceiveFixedDataLenFromSocketBlocking(TSocket &socket, char *msg, SInt32 &msg_size, const SInt32 data_len)
  {
    //fprintf(stderr, "Client On receive %d bytes from socket %d\n", data_len, socket.sock);

    struct sockaddr from;

    msg_size = recvfrom(socket.sock, msg, data_len, MSG_WAITALL, (struct sockaddr *)&from, (socklen_t *)&socket.addr_len);

    if (msg_size < 0)
      error("Client Recvfrom fixed");

    //fprintf(stderr, "Received %d bytes from socket %d\n", msg_size, socket.sock);

    return msg_size;
  }

  /************************************************/
  /************************************************/

  SInt32 ClientReceiveDataFromSocketBlocking(TSocket &socket, char **msg, SInt32 &msg_size, const SInt32 max_data)
  {
    //fprintf(stderr, "Client On receive max %d bytes from socket %d\n", max_data, socket.sock);

    struct sockaddr from;

    msg_size = recvfrom(socket.sock, *msg, max_data, 0, (struct sockaddr *)&from, (socklen_t *)&socket.addr_len);

    if (msg_size < 0)
      error("Client Recvfrom");

    //fprintf(stderr, "Received %d bytes from socket %d\n", msg_size, socket.sock);
    return msg_size;
  }

  /************************************************/
  /************************************************/

  SInt32 ClientReceiveDataFromSocketBlocking(TSocket &socket, char *msg, SInt32 &msg_size, const SInt32 max_data)
  {

    //fprintf(stderr, "Client On receive max %d bytes from socket %d\n", max_data, socket.sock);

    struct sockaddr from;

    msg_size = recvfrom(socket.sock, msg, max_data, 0, (struct sockaddr *)&from, (socklen_t *)&socket.addr_len);

    if (msg_size < 0)
      error("Client Recvfrom");

    //fprintf(stderr, "Received %d bytes from socket %d\n", msg_size, socket.sock);
    return msg_size;
  }
  /************************************************/
  /************************************************/

  SInt32 ClientPeekDataFromSocketBlocking(TSocket &socket, char *msg, SInt32 &msg_size, const SInt32 max_data)
  {

    //fprintf(stderr, "Client peek max %d bytes from socket %d\n", max_data, socket.sock);

    struct sockaddr from;

    msg_size = recvfrom(socket.sock, msg, max_data, MSG_PEEK, (struct sockaddr *)&from, (socklen_t *)&socket.addr_len);

    if (msg_size < 0)
      error("Client Recvfrom");

    //fprintf(stderr, "[Peek] Received %d bytes from socket %d\n", msg_size, socket.sock);
    return msg_size;
  }

  /************************************************/
  /************************************************/


  void BuildSimulationInformationPacket(char **msg, SInt32 &msg_size, const SInt32 robot_num,
					const ns3Real simulation_step, const ns3Real simulation_time,
					const char *sim_name, const ns3Real stop_time,
					const ns3Real f_arena_x, const ns3Real f_arena_y, const ns3Real f_arena_z)
  {
    TSimulationInformationPacket info;

    info.type            = SIMULATION_INFORMATION;
    info.robot_num       = robot_num;
    info.simulation_step = simulation_step;
    info.start_time      = simulation_time;
    info.stop_time       = stop_time;
    info.arena_x         = f_arena_x;
    info.arena_y         = f_arena_y;
    info.arena_z         = f_arena_z;
    strcpy(info.simulator_name, sim_name);

    memcpy(*msg, (void *)&info, sizeof(TSimulationInformationPacket));

    msg_size = sizeof(TSimulationInformationPacket);
  }

  /************************************************/
  /************************************************/

  void BuildSimulationInformationPacket(char *msg, SInt32 &msg_size, const SInt32 robot_num,
					const ns3Real simulation_step, const ns3Real simulation_time,
					const char *sim_name, const ns3Real stop_time,
					const ns3Real f_arena_x, const ns3Real f_arena_y, const ns3Real f_arena_z)
  {
    TSimulationInformationPacket info;

    info.type            = SIMULATION_INFORMATION;
    info.robot_num       = robot_num;
    info.simulation_step = simulation_step;
    info.start_time      = simulation_time;
    info.stop_time       = stop_time;
    info.arena_x         = f_arena_x;
    info.arena_y         = f_arena_y;
    info.arena_z         = f_arena_z;
    strcpy(info.simulator_name, sim_name);

    memcpy(msg, (void *)&info, sizeof(TSimulationInformationPacket));

    msg_size = sizeof(TSimulationInformationPacket);
  }

  /************************************************/
  /************************************************/

  void BuildRobotStartPacket(char **msg, SInt32 &msg_size, const SInt32 id, const char *robot_type)
  {
    TRobotStartPacket start;

    start.type  = START_FROM_ROBOT_PROCESS;
    start.id    = id;
    strcpy(start.robot_type, robot_type);

    memcpy(*msg, (void *)&start, sizeof(TRobotStartPacket));

    msg_size = sizeof(TRobotStartPacket);
  }

  /************************************************/
  /************************************************/

  void BuildRobotStartPacket(char *msg, SInt32 &msg_size, const SInt32 id, const char *robot_type)
  {
    TRobotStartPacket start;

    start.type  = START_FROM_ROBOT_PROCESS;
    start.id    = id;
    strcpy(start.robot_type, robot_type);

    memcpy(msg, (void *)&start, sizeof(TRobotStartPacket));

    msg_size = sizeof(TRobotStartPacket);
  }

  /************************************************/
  /************************************************/

  void BuildRobotPositionsPacket(char **msg, SInt32 &msg_size, const SInt32 robot_num, const TPosition *positions, const ns3Real timestamp)
  {
    TNodePositionsPacket pos;

    pos.type = NODE_POSITIONS;
    pos.timestamp = timestamp;
    pos.robot_num = robot_num;
    pos.size = robot_num * sizeof(TPosition);

    memcpy(*msg, (void *)&pos, sizeof(TNodePositionsPacket));
    memcpy((*msg) + sizeof(TNodePositionsPacket), (void *)positions, pos.size);

    msg_size = sizeof(TNodePositionsPacket) + pos.size;
  }

  /************************************************/
  /************************************************/

  void BuildRobotPositionsPacket(char *msg, SInt32 &msg_size, const SInt32 robot_num, const TPosition *positions, const ns3Real timestamp)
  {
    TNodePositionsPacket pos;

    pos.type = NODE_POSITIONS;
    pos.timestamp = timestamp;
    pos.robot_num = robot_num;
    pos.size = robot_num * sizeof(TPosition);

    memcpy(msg, (void *)&pos, sizeof(TNodePositionsPacket));
    memcpy((msg) + sizeof(TNodePositionsPacket), (void *)positions, pos.size);

    msg_size = sizeof(TNodePositionsPacket) + pos.size;
  }

  /************************************************/
  /************************************************/
  void GenerateMsg(char *p_msg, SInt32 &n_msg_size, 
		   const SInt32 n_from, const SInt32 n_to, 
		   const char *content, 
		   size_t len, const ns3Real f_timestamp,
		   SInt32 delay)
  {
    TApplicationPacket appl;
    appl.type = APPLICATION_DATA;
    appl.from = n_from;
    appl.to   = n_to;
    appl.timestamp = f_timestamp;
    appl.size = len+1;
    appl.delay = delay;

    memcpy(p_msg, (void *)&appl, sizeof(TApplicationPacket));
    memcpy((p_msg) + sizeof(TApplicationPacket), (void *)content, appl.size);

    n_msg_size = sizeof(TApplicationPacket) + appl.size;
  }
  /************************************************/
  /* Start of the class member definitions        */
  /************************************************/


  void CWiFiMediumEntity::Init(TConfigurationNode& t_tree){
    CEntity::Init(t_tree);

    m_pnMsg = new char[MAX_MSG_SIZE];
    this->m_unServerHost = (const UInt8 *)"127.0.0.1"; //In future readed from the xml file
    this->m_nPortBase = 9999; //In future readed from the xml file

    /*Opening the socket*/
    OpenBlockingUDPSocketClientSide(m_tSocketScheduler, m_nPortBase, (const char*)m_unServerHost);

    /*Sending the needed informations to the network simulator*/
    CSpace::TAnyEntityMap& tEntityMap = m_cSpace.GetEntitiesByType("wifi_equipped_entity");

    //m_nRobotNumber = 0;
    //for( CSpace::TAnyEntityMap::iterator it = tEntityMap.begin(); it != tEntityMap.end(); ++it ) 
    //{
      //CWiFiEquippedEntity& cWifiEntity = *(any_cast<CWiFiEquippedEntity*>(it->second));
      //if( cWifiEntity.GetMode() == argos::EXTERN )
	//m_nRobotNumber++;
    //}
    m_nRobotNumber = tEntityMap.size(); /*How many robots are wifi enabled?*/
    //UInt32 robotNumber = tEntityMap.size();
    if( NodeAttributeExists(t_tree, "number_of_nodes") )
      GetNodeAttribute<SInt32>(t_tree, "number_of_nodes",m_nRobotNumber);
    ns3Real fSimulationStep = CPhysicsEngine::GetSimulationClockTick();
    m_fCurrentTimeSlice =  m_cSpace.GetSimulationClock() / CPhysicsEngine::GetInverseSimulationClockTick();
    const char* strSimulatorName = "ARGoS2";
    ns3Real fTotalSimulationTime = CSimulator::GetInstance().GetMaxSimulationClock() / CPhysicsEngine::GetInverseSimulationClockTick();
    std::cerr << "There are " << m_nRobotNumber << " robots with wifi-equipment in the simulation\n";

    /*Acquiring the information about the arena*/
    CVector3 cArenaSize = m_cSpace.GetArenaSize();
    std::cerr << "Arena Size: " << cArenaSize << std::endl;

    BuildSimulationInformationPacket(&m_pnMsg, m_nMsgSize, m_nRobotNumber, fSimulationStep,
				     m_fCurrentTimeSlice, strSimulatorName, fTotalSimulationTime,
				     cArenaSize[0], cArenaSize[1], cArenaSize[2]);
    std::cerr << "Sending the info to the other simulator, size of the packet: " << m_nMsgSize << std::endl;
    printf("Sending to socket:\n");
    char ackMsg[100];
    char *pAckMsg = &ackMsg[0];
    ClientSendDataToSocket(m_tSocketScheduler, m_pnMsg, m_nMsgSize);
    std::cerr << "Data Sended, waiting for ACK\n";
    printf("done, waiting for ACK --\n");
    SInt32 rcv_size;
    sleep(2);
    while( ClientReceiveDataFromSocketNonBlocking(m_tSocketScheduler, 
						  &pAckMsg, rcv_size, 
						  MAX_MSG_SIZE) <= 0) 
    {
      printf("waiting...\n");
      printf("Sending AGAIN to socket:\n");
      ClientSendDataToSocket(m_tSocketScheduler, m_pnMsg, m_nMsgSize);
      sleep(2);
      std::cerr << "Data Sended, waiting for ACK\n";
      printf("done, waiting for ACK --\n");
    }
    printf("received %d (%d)\n", 
	   rcv_size, m_nMsgSize);
    /*Once the network simulator sended back the ack the procedure can resume*/
    /*Check if the network simulator sended the expected data*/
    if(m_nMsgSize > 0){
      TNetworkSimInfoPacket *info = (TNetworkSimInfoPacket *)pAckMsg;
      if(info->type != NETWORK_SIM_INFO)
      {
	std::cerr << "[NETWORK_SIM_INFO] WRONG PACKET TYPE: " 
	  << info->type << std::endl;
	exit(-1);
      }
      else
      {
	std::cerr << "Received Start info packet from network simulator " 
	  << info->network_sim << std::endl;
      }
    }
    //this->Update(); //Not valid, too early, deadlock
    m_ptPositions = new TPosition[m_nRobotNumber];
    //this->Update();//Not valid, too early, deadlock
  }

  /************************************************/
  /************************************************/

  void CWiFiMediumEntity::Update(){

    m_fCurrentTimeSlice =  m_cSpace.GetSimulationClock() / CPhysicsEngine::GetInverseSimulationClockTick();
    CVector3 cVectorToMessage;
    UInt8 un_i = 0;
    /*Waiting the ack from the network simulator*/
    ClientReceiveDataFromSocketBlocking(m_tSocketScheduler, &m_pnMsg, m_nMsgSize, MAX_MSG_SIZE);

    /*Update the positions of the robot*/
    CSpace::TAnyEntityMap& tEntityMap = m_cSpace.GetEntitiesByType("wifi_equipped_entity");
    /*Iterate over all robots with wifi and ask their position*/
    for(CSpace::TAnyEntityMap::iterator it = tEntityMap.begin();it != tEntityMap.end();++it) {
      CWiFiEquippedEntity& cWiFiEntity = *(any_cast<CWiFiEquippedEntity*>(it->second));
      cVectorToMessage = cWiFiEntity.GetPosition();
      un_i = cWiFiEntity.GetNumericId(cWiFiEntity.GetId());
      m_ptPositions[un_i].id = un_i;
      m_ptPositions[un_i].x = cVectorToMessage[0];
      m_ptPositions[un_i].y = cVectorToMessage[1];
      m_ptPositions[un_i].z = cVectorToMessage[2];

      //std::cout << m_ptPositions[un_i].id << " : " <<  m_ptPositions[un_i].x << " " <<m_ptPositions[un_i].y << " "<< m_ptPositions[un_i].z<< std::endl;

    }
    BuildRobotPositionsPacket(&m_pnMsg, m_nMsgSize, m_nRobotNumber, m_ptPositions, m_fCurrentTimeSlice);
    /*send the position of the robot*/
    ClientSendDataToSocket(m_tSocketScheduler, m_pnMsg, m_nMsgSize);
    /*Everything is done*/


  }
 /************************************************/
  /************************************************/

  void CWiFiMediumEntity::Destroy()
  {
    TStopSimulation tStop;
    tStop.type = STOP_SIMULATION;
    memcpy(m_pnMsg, (void *)&tStop, sizeof(TStopSimulation));
    ClientSendDataToSocket(m_tSocketScheduler, m_pnMsg, m_nMsgSize);
  }
  /************************************************/
  /************************************************/

  REGISTER_ENTITY(CWiFiMediumEntity,
                   "wifi-medium",
                   "A wifi medium",
                   "Marco Cinus [marco@idsia.ch]",
                   "TODO",
                   "UNDER DEVELOPMENT"
		  );
}
