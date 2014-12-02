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

//void start_server(char *hostname_in, int port_in);
void start_server();

void send_msg(char *msg);

int stop_server();

#endif
