#include	<errno.h>
#include	<stdio.h>
#include	<string.h>    	//strlen
#include 	<sys/types.h>	//may not be needed, added for legacy purposes for listen call
#include	<sys/socket.h>
#include	<arpa/inet.h> 	//inet_addr
#include	<unistd.h>
#include 	<errno.h>
#include	"cvNAV_senddata.h"
#include	"get_nav_state.h"

#define debugging

//CVNS is Server, SNAPS is Client
/* Notes from Bob
Program should establish a "server socket".
It should then "listen" for someone to open a connection.
Once a connection is open, every time the nav_state is updated,
then the nav_state should be converted to a comma delimited ascii string and transmitted down each open connection.
The navigation data should also be written to a flight log file.

The interface between the two threads should be a block of shared memory,
very similar to the navigation task and the G&C task.

In the satellite, the navigation process should open a standard (client) "socket", connect,
and wait for navigation data transmissions.
Each time new data is received, it should converted from ascii,
and update the navigation state in satellite shared memory.
just like last semester, except its reading from the socket instead of magnetometers.

*/
//#define SATELLITE_IP_ADDR  "192.168.2.2"
#define SATELLITE_IP_ADDR  "127.0.0.31" //loopback IP for testing
#define SATELLITE_IP_PORT  2101


extern struct snaps_nav_state_struct my_nav_state;

static int client_fd, socket_desc;
static struct sockaddr_in server;
static struct sockaddr_in client;
//Create a socket with the socket() system call - SERVER
	//Listen for connections with the listen() system call
	//Accept a connection with the accept() system call. This call blocks until client connects w/ server.
	//Send and receive data
int senddata_open() {
	int rc, sockaddr_size;

#ifdef debugging
	printf("creating socket... \n");
#endif
	socket_desc 	= socket(AF_INET , SOCK_STREAM , 0);
	if (socket_desc == -1) {
		fprintf(stderr, "*ERROR* senddata_open: Could not create socket.\n");
		return socket_desc;
	}
	puts("Success: senddata_open: Socket was. \n");
	memset(&server, 0, sizeof(server));
	server.sin_addr.s_addr 	= inet_addr(SATELLITE_IP_ADDR);
	server.sin_family 		= AF_INET;
	server.sin_port 		= htons(SATELLITE_IP_PORT);

#ifdef debugging
	printf("binding socket... \n");
#endif
	//Bind the socket to an address using the bind() system call.
	rc = bind(socket_desc, (struct sockaddr*)&server, sizeof(server));
	if (rc == -1){
		fprintf(stderr,"*ERROR* senddata_open: binding failure. %d, %s \n", errno, strerror(errno));
		close (socket_desc);
		return rc;
	}
	puts("Success: senddata_open: Socket bind complete. \n");

#ifdef debugging
	printf("listening for someone to connect... \n");
#endif
	sockaddr_size = sizeof(struct sockaddr_in);
	//listen for someone to connect
	rc 	= listen (socket_desc, 2);
	if (rc == -1){
		fprintf(stderr,"*ERROR* senddata_open: listen failure. \n");
		close (socket_desc);
		return rc;
	}
#ifdef debugging
	printf("accepting connection... \n");
#endif
	//accept connection with client
	client_fd 		= accept (socket_desc, (struct sockaddr *)&client, (socklen_t*)&sockaddr_size);//sizeof(server) );
	if (client_fd == -1){
		fprintf(stderr,"*ERROR* senddata_open: could not accept connection. \n");
		close (client_fd);
		return client_fd;
	}
	else {
		printf("socket established is %d \n", client_fd);
	}
	return client_fd;
} // end function senddata_open

void senddata_close(int fd) {
#ifdef debugging
	printf("closing connection... \n");
#endif
	if (fd >= 0) close (fd);
} // end function senddata_close

int senddata_action(int client_fd) {
	// variable for command from  cvNAV_receivedata
	char cvNAV_command;
	// length of variable for command from cvNAV_receivedata
	int cvNAV_message_len;
	//buffer for i/o
	char navstate_buff[1024];
	int rc = 0;
	int len;

	if (client_fd < 0) {
		fprintf(stderr, "*ERROR* senddata_action: Invalid file descriptor.\n");
		return -3;
	}

	cvNAV_message_len = recv(client_fd, navstate_buff, 1024, 0);
	navstate_buff[cvNAV_message_len++] = 0;
	printf("cvNAV_message_len is: [%d] \n", cvNAV_message_len);
	printf("Satellite message is: \"%s\" \n", navstate_buff);
	cvNAV_command = navstate_buff[3];

	switch (cvNAV_command) {

	//get nav state
		case 'N':
			//send most current nav_state(ascii string) & timestamp
			printf("%d, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d \r \n",
					my_nav_state.version, my_nav_state.revision,
					my_nav_state.attitude_dcm[0][0], my_nav_state.attitude_dcm[0][1], my_nav_state.attitude_dcm[0][2],
					my_nav_state.attitude_dcm[1][0], my_nav_state.attitude_dcm[1][1], my_nav_state.attitude_dcm[1][2],
					my_nav_state.attitude_dcm[2][0], my_nav_state.attitude_dcm[2][1], my_nav_state.attitude_dcm[2][2],
					my_nav_state.status);

			cvNAV_message_len =	snprintf(navstate_buff, sizeof(navstate_buff), "%d, %d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d",
					my_nav_state.version, my_nav_state.revision,
					my_nav_state.attitude_dcm[0][0], my_nav_state.attitude_dcm[0][1], my_nav_state.attitude_dcm[0][2],
					my_nav_state.attitude_dcm[1][0], my_nav_state.attitude_dcm[1][1], my_nav_state.attitude_dcm[1][2],
					my_nav_state.attitude_dcm[2][0], my_nav_state.attitude_dcm[2][1], my_nav_state.attitude_dcm[2][2],
					my_nav_state.status);
			//cvNAV_message = snprintf (navstate_buff, cvNAV_message_len, "The navigation state is %s", name, nav_state);

			if ((len = send(client_fd , navstate_buff, cvNAV_message_len, 0)) < 0){
				fprintf(stderr, "*ERROR* senddata_action: send error. %s\n", strerror(errno));
				rc = -1;
			}
			else
				printf("send data complete... %d \n", len);
			break;

	//disconnect
		case 'D':
			//exit loop
			senddata_close(client_fd);
			break;

		default:
			printf("illegal command... \n");
			break;

	} // switch command

	return rc;
} // end function senddata_action
