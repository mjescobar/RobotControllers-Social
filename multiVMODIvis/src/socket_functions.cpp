#include "socket_functions.h"

int port, n;
struct sockaddr_in serveraddr;
struct hostent *server;
char *hostname;

void reusePort(int s)
{
  int one=1;

  if ( setsockopt(s,SOL_SOCKET,SO_REUSEADDR,(char *) &one,sizeof(one)) == -1 )
  {
    printf("error in setsockopt,SO_REUSEPORT \n");
    exit(-1);
  }
}

//void start_server(char *hostname_in, int port_in) {
int start_server() {
    // Creacion y configuracion del socket para comunicarse con el servidor

    hostname = "127.0.0.1";
    port = PORTNUMBER;
    printf(" starting...");
    fflush(stdin);
    int s = socket(AF_INET, SOCK_STREAM, 0);
    //reusePort(s);
    if ( s < 0 )
        perror("ERROR opening socket");

    server = gethostbyname(hostname);
    if ( server == NULL ) {
        perror("ERROR, no such host");
        exit(0);
    }

    bcopy( server->h_addr, &(serveraddr.sin_addr), server->h_length);
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(port);

    // Estableciendo conexion con el servidor
    if ( connect(s, (struct sockaddr *) &serveraddr, sizeof(serveraddr)) < 0 )
        perror("ERROR connecting");
    return s;
}

void send_msg(int sock, char *msg) {
    // Envio de la opcion ingresada por el usuario
    n = send(sock, msg, sizeof(msg), 0);
    if ( n < 0 )
        perror("ERROR writing to socket");
}

int stop_server(int s) {
    // Cerrar conexion
    close(s);
    return 0;
}
