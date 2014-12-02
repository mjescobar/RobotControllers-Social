#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <semaphore.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <cstdio>
#include <vector>

#define pi 3.141593f

#define HEADER_LENGTH 6 // byte1=id1, byte2=id2, byte3+byte4=packetSize, byte5+byte6=packetsLeftToRead
#define HEADER_ID1 59
#define HEADER_ID2 57
#define MAX_PACKETSIZE 250

#define INVALID_SOCKET        -1
#define Sleep(x) (usleep(x*1000))

#define PORTNUMBER  10220
#define MAXCONNECTIONS 50

typedef struct thread_args{
	int *identifier;
	int *inc_connection;	// single incoming connection
} thread_args;


typedef int                 _SOCKET;
typedef struct timeval      _timeval;
typedef socklen_t           _socklen;

typedef unsigned short      WORD;
typedef unsigned long       DWORD;
typedef unsigned short      u_short;

DWORD getTimeInMs(void);
DWORD getTimeDiffInMs(DWORD lastTime);

void reusePort(int s);
int _receiveSimplePacket(std::vector<char>& packet, int acc_conn);
bool _sendSimplePacket(char* packet,int packetLength,WORD packetsLeft,int comm_addr);
bool replyToReceivedData(char* data,int dataSize, int comm_addr);
char* receiveData(int& dataSize,int acc_conn);
void * client_connection(void *arguments);
int simpleReuseSocket(int port);
int simpleAcceptConn(int s);



#endif // UTILS_H_INCLUDED
