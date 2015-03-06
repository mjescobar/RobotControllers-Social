#ifndef _SOCKET_FUNC_H_
#define _SOCKET_FUNC_H_

#include <stdio.h>
#include <sys/time.h>

#include <sys/types.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <sys/file.h>

#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip_icmp.h>
#include <netdb.h>
#include <arpa/inet.h>

#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#define HEADER_LENGTH 6 // byte1=id1, byte2=id2, byte3+byte4=packetSize, byte5+byte6=packetsLeftToRead
#define HEADER_ID1 59
#define HEADER_ID2 57
#define MAX_PACKETSIZE 250

#define INVALID_SOCKET        -1
#define Sleep(x) (usleep(x*1000))

#define PORTNUMBER  10220
#define MAXCONNECTIONS 50

//void start_server(char *hostname_in, int port_in);
int start_server();

void send_msg(int sock, char *msg);

int stop_server(int s);

#endif
