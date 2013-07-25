#include	<errno.h>
#include	<stdio.h>
#include	<string.h>    	//strlen
#include 	<sys/types.h>	//may not be needed, added for legacy purposes for listen call
#include	<sys/socket.h>
#include	<arpa/inet.h> 	//inet_addr
#include	<unistd.h>
#include	"cvNAV_receivedata.h"

#define debugging
//CVNS is Server, SNAPS is Client

/* Notes from Bob
In the satellite, the navigation process should open a standard (client) "socket", connect,
and send command to CVNS requesting nav data or disconnect.
Each time new data is received, it should converted from ascii,
and update the navigation state in satellite shared memory.
similar to what it did last semester, except its reading from the socket instead of magnetometers.

The steps involved in establishing a socket on the client side are as follows:

Create a socket with the socket() system call - CLIENT
	Connect the socket to the address of the server using the connect() system call
	Send and receive data. simplest way is to use the read() and write() system calls.

*/
//#define SATELLITE_IP_ADDR  "192.168.2.2" needs to be ip of CVNS
#define SATELLITE_IP_ADDR "127.0.0.23" //loopback IP for debugging
#define SERVER_IP_ADDR "127.0.0.31"
#define SERVER_PORT 2101

static struct sockaddr_in server;

int main(int argc, char *argv[]){
	int fd;
	fd = receivedata_open();
	for (int i = 0; i < 10; i++)
	{
		receivedata_action(fd, 0) ;
	}
	receivedata_action(fd, 1);
	receivedata_close(fd);
	return 0;
}

int receivedata_open() {
	int rc;
	int socket_desc;

#ifdef debugging
	printf("creating socket... \n");
#endif
	//create socket
	socket_desc = socket(AF_INET , SOCK_STREAM , 0);
	if (socket_desc == -1) {
		fprintf(stderr, "*ERROR* receivedata_open: Could not create socket.\n");
		return socket_desc;
	}

	server.sin_addr.s_addr = inet_addr(SERVER_IP_ADDR);
	server.sin_family 	= AF_INET;
	server.sin_port 	= htons(SERVER_PORT);

#ifdef debugging
	printf("connecting to CVNS... \n");
#endif
	//Connect to remote server
	rc = connect(socket_desc, (struct sockaddr *)&server , sizeof(server));
	if (rc < 0) {
		fprintf(stderr, "*ERROR* receivedata_open: Connection error. %s\n", strerror(errno));
		return rc;
	}

	return socket_desc;
} // end function receivedata_open

void receivedata_close(int fd) {

	if (fd >= 0) close (fd);
} // end function receivedata_close

int receivedata_action(int fd, int disconnect) {
	char navstate_buff[1024];
	int cvNAV_message_len;
	int rc = 0;
	int len;

	if (fd < 0) {
		fprintf(stderr, "*ERROR* receivedata_action: Invalid file descriptor.\n");
		return -3;
	}
	if (disconnect)
	{
		strcpy(navstate_buff, "2, D\0");
	}
	else
	{
		strcpy(navstate_buff, "2, N\0");
	}
	cvNAV_message_len = strlen(navstate_buff);
//send command to CVNS
	printf("sending command... %d \"%s\" \n", cvNAV_message_len, navstate_buff);
	len = send(fd, navstate_buff, cvNAV_message_len, 0);
	printf("Sent %d characters  \"%s\" \n", len, navstate_buff);
	printf("receiving navigation state... \n");
	cvNAV_message_len = recv(fd, navstate_buff, cvNAV_message_len, 0);
	navstate_buff[cvNAV_message_len++] = 0;
	printf("Navigation state is [%d]: \"%s\" \n", cvNAV_message_len, navstate_buff);

	return rc;
} // end function receivedata_action


