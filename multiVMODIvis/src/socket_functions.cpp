#include "socket_functions.h"

int s, port, n;
struct sockaddr_in serveraddr;
struct hostent *server;
char *hostname;


//void start_server(char *hostname_in, int port_in) {
void start_server() {
    // Creacion y configuracion del socket para comunicarse con el servidor

    hostname = "127.0.0.1";
    port = 13450;
    printf(" starting...");
    fflush(stdin);
    s = socket(AF_INET, SOCK_STREAM, 0);
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

}

void send_msg(char *msg) {
    // Envio de la opcion ingresada por el usuario
    n = send(s, msg, sizeof(msg), 0);
    if ( n < 0 )
        perror("ERROR writing to socket");

}

int stop_server() {
    // Cerrar conexion
    close(s);
    return 0;
}
