/* -*- Mode: C++ -*-
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

/**
 * @file <argos2/simulator/space/entities/wifi_medium_entity.h>
 *
 * @author Carlo Pinciroli - <cpinciroli@ulb.ac.be>
 */

#ifndef WIFI_MEDIUM_ENTITY_H
#define WIFI_MEDIUM_ENTITY_H


#define MAX_MSG_SIZE 8192


namespace argos {
  class CWiFiMediumEntity;
}

#include <argos2/simulator/space/entities/medium_entity.h>
#include <argos2/simulator/simulator.h>

#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

/*Added the typedef ns3Real double because the simulator on the other side needs to know which
  datatype has been used (check of the header size and so on*/

typedef double ns3Real;


namespace argos {
  /*Added by marco*/
  typedef struct{
    SInt32 sock;
    SInt32 addr_len;
    struct sockaddr_in server;
    struct sockaddr_in client;
    struct hostent *hp;
  } TSocket;

  typedef struct{
    SInt32 id;
    ns3Real x;
    ns3Real y;
    ns3Real z;
  } TPosition;

  typedef enum{
    SIMULATION_INFORMATION,
    APPLICATION_DATA,
    RECEIVED_DATA,
    NODE_POSITIONS,
    START_FROM_ROBOT_PROCESS,
    NETWORK_SIM_INFO,
    MSG_TYPE_NUM,
    STOP_SIMULATION
  }TMsgType;

  typedef struct{
    TMsgType type;
    SInt32 robot_num;
    ns3Real simulation_step;
    ns3Real start_time;
    ns3Real stop_time;
    ns3Real arena_x;
    ns3Real arena_y;
    ns3Real arena_z;
    char simulator_name[30];
  } TSimulationInformationPacket;

  typedef struct{
    TMsgType type;
    ns3Real timestamp;
    SInt32 robot_num;
    SInt32 size;
  } TNodePositionsPacket;

  typedef struct{
    TMsgType type;
    ns3Real   timestamp;
    SInt32     from;
    SInt32     to;
    SInt32     size;
    UInt32     delay;

  } TApplicationPacket;


  typedef struct{
    TMsgType  type;
    SInt32      id;
    char     robot_type[50];

  } TRobotStartPacket;


  typedef struct{
    TMsgType  type;
    char     network_sim[50];

  } TNetworkSimInfoPacket;

  typedef struct{
    TMsgType type;
  } TStopSimulation;

  /************************************************************/
  /*    Common functions for socket comunication              */
  /************************************************************/
  void error(const char *msg);

  SInt32 OpenBlockingUDPSocketServerSide(TSocket &t_new_socket, const SInt32 n_port);

  SInt32 OpenBlockingUDPSocketClientSide(TSocket &t_new_socket, const UInt32 un_port, const char *p_server_host);

  SInt32 ServerSendDataToSocket(TSocket &t_socket, const char *p_msg, const SInt32 n_msg_size);

  SInt32 ClientSendDataToSocket(TSocket &t_socket, const char *p_msg, const SInt32 n_msg_size);

  SInt32 ServerReceiveDataFromSocketNonBlocking(TSocket &t_socket, char **p_msg, SInt32 &n_msg_size, const SInt32 n_max_data);
  SInt32 ServerReceiveDataFromSocketNonBlocking(TSocket &t_socket, char *p_msg, SInt32 &n_msg_size, const SInt32 n_max_data);

  SInt32 ServerReceiveDataFromSocketBlocking(TSocket &t_socket, char **p_msg, SInt32 &n_msg_size, const SInt32 n_max_data);
  SInt32 ServerReceiveDataFromSocketBlocking(TSocket &t_socket, char *p_msg, SInt32 &n_msg_size, const SInt32 n_max_data);

  SInt32 ServerPeekDataFromSocketBlocking(TSocket &t_socket, char *p_msg, SInt32 &n_msg_size, const SInt32 n_max_data);
  SInt32 ServerPeekDataFromSocketNonBlocking(TSocket &t_socket, char *p_msg, SInt32 &n_msg_size, const SInt32 n_max_data);

  SInt32 ServerReceiveFixedDataLenFromSocketBlocking(TSocket &t_socket, char *p_msg, SInt32 &n_msg_size, const SInt32 n_data_len);

  SInt32 ClientReceiveDataFromSocketNonBlocking(TSocket &t_socket, char **p_msg, SInt32 &n_msg_size, const SInt32 n_max_data);
  SInt32 ClientReceiveDataFromSocketNonBlocking(TSocket &t_socket, char *p_msg, SInt32 &n_msg_size, const SInt32 n_max_data);

  SInt32 ClientReceiveDataFromSocketBlocking(TSocket &t_socket, char **p_msg, SInt32 &n_msg_size, const SInt32 n_max_data);
  SInt32 ClientReceiveDataFromSocketBlocking(TSocket &t_socket, char *p_msg, SInt32 &n_msg_size, const SInt32 n_max_data);

  SInt32 ClientPeekDataFromSocketBlocking(TSocket &t_socket, char *p_msg, SInt32 &n_msg_size, const SInt32 n_max_data);

  SInt32 ClientReceiveFixedDataLenFromSocketBlocking(TSocket &t_socket, char *p_msg, SInt32 &n_msg_size, const SInt32 n_data_len);

  /************************************************************/
  /*    Socket Comunication for the Robot Side                */
  /************************************************************/
  void BuildSimulationInformationPacket(char **p_msg, SInt32 &n_msg_size, const SInt32 n_robot_num,
					const ns3Real f_simulation_step, const ns3Real f_simulation_time,
					const char *p_sim_name, const ns3Real f_stop_time,
					const ns3Real f_arena_x, const ns3Real f_arena_y, const ns3Real f_arena_z);
  void BuildSimulationInformationPacket(char *p_msg, SInt32 &n_msg_size, const SInt32 n_robot_num,
					const ns3Real f_simulation_step, const ns3Real f_simulation_time,
					const char *p_sim_name, const ns3Real f_stop_time,
					const ns3Real f_arena_x, const ns3Real f_arena_y, const ns3Real f_arena_z);

  void BuildRobotStartPacket(char **p_msg, SInt32 &n_msg_size, const SInt32 n_id, const char *p_robot_type);
  void BuildRobotStartPacket(char *p_msg, SInt32 &n_msg_size, const SInt32 n_id, const char *p_robot_type);

  void BuildRobotPositionsPacket(char **p_msg, SInt32 &n_msg_size, const SInt32 n_robot_num, const TPosition *t_positions, const ns3Real f_timestamp);
  void BuildRobotPositionsPacket(char *p_msg, SInt32 &n_msg_size, const SInt32 n_robot_num, const TPosition *t_positions, const ns3Real f_timestamp);
  void GenerateMsg(char *p_msg, SInt32 &n_msg_size, 
		   const SInt32 n_from, const SInt32 n_to, 
		   const char*content, size_t len, const ns3Real f_timestamp, SInt32 delay);
  /*End added by Marco*/


  class CWiFiMediumEntity : public CMediumEntity {

  public:
    CWiFiMediumEntity() :
      CMediumEntity(NULL),m_cSpace(CSimulator::GetInstance().GetSpace()) {}
    virtual ~CWiFiMediumEntity() {}

    virtual void Update();// {}
    virtual void Init(TConfigurationNode& t_tree);
    virtual void Destroy();
    inline virtual void Accept(CEntityVisitor& visitor) {
      visitor.Visit(*this);
    }

    inline virtual std::string GetTypeDescription() const {
      return "wifi_medium_entity";
    }

    inline SInt32 GetPortBase(void){
      return m_nPortBase;
    }
  private:
    UInt8 const *m_unServerHost;
    SInt32 m_nPortBase;
    TPosition* m_ptPositions;
    TSocket m_tSocketScheduler;
    CSpace& m_cSpace;
    SInt32 m_nMsgSize;
    char *m_pnMsg;
    SInt32 m_nRobotNumber;
    ns3Real m_fCurrentTimeSlice;

  };
}

#endif
