#define _GNU_SOURCE
#include <stdio.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdlib.h>
#include <limits.h>
#include <stdint.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <string.h>
#include <getopt.h>
#include <netinet/in.h>
#include <netdb.h> 

extern char *optarg;
extern int optind, opterr, optopt;

int outputDebug=0;
int milliseconds_timeout=500;
int alarmSeconds=5;

#define DATA_BLOCK_N 32 /* maximum number of NMEA sentences */

typedef struct {
	uint64_t microtime_start;	/* time when STX was received*/
	uint8_t sentence[128];		/* full sentence ('$' to second character of checksum), null terminated */
} struct_data_block;
#define DUMP_CURRENT_JSON_SUGGESTED_SIZE (DATA_BLOCK_N * 2 * sizeof(data_block[0].sentence))

/* array of data_blocks to put Magnum network data into */
struct_data_block data_block[DATA_BLOCK_N];


uint64_t microtime() {
	struct timeval time;
	gettimeofday(&time, NULL); 
	return ((uint64_t)time.tv_sec * 1000000) + time.tv_usec;
}


void dump_current_json(char *buff) {
	int i;
	int printed;
	char buff2[256];

/*
{
	"GPGGA": {
		"age": 1234,
		"sentence": "$GPGGA,121,0943,z,*45"
	},
	"GPRMC": {
		"age": 97834,
		"sentence": "$GPRMC,some sentence,,121,0943,z,*DA"
	}
}
*/

	sprintf(buff,"{\n");

	printed=0;

	for ( i=0 ; i < DATA_BLOCK_N ; i++ ) {
		if ( 0 == data_block[i].microtime_start ) {
			continue;
		}

		if ( 0 != printed ) {
			sprintf(buff2,",\n");
			strcat(buff,buff2);
		}

		printed++;

		sprintf(buff2,"\t\"%.*s\": {\n",5,data_block[i].sentence+1);
		strcat(buff,buff2);

		sprintf(buff2,"\t\t\"age\": %d,\n",(int) (microtime() - data_block[i].microtime_start) );
		strcat(buff,buff2);

		sprintf(buff2,"\t\t\"sentence\": \"%s\"\n",data_block[i].sentence);
		strcat(buff,buff2);

		sprintf(buff2,"\t}");
		strcat(buff,buff2);
	}

	sprintf(buff2,"\n}\n");
	strcat(buff,buff2);
}

void signal_handler(int signum) {
	int i, j;
	char buff[DUMP_CURRENT_JSON_SUGGESTED_SIZE];


	if ( SIGALRM == signum ) {
		fprintf(stderr,"\n# Timeout while waiting for NMEA data.\n");
		fprintf(stderr,"# Terminating.\n");
		exit(100);
	} else if ( SIGPIPE == signum ) {
		fprintf(stderr,"\n# Broken pipe.\n");
		fprintf(stderr,"# Terminating.\n");
		exit(101);
	} else if ( SIGUSR1 == signum ) {
		/* clear signal */
		signal(SIGUSR1, SIG_IGN);

		fprintf(stderr,"# SIGUSR1 triggered data_block dump:\n");
		
		dump_current_json(buff);
		printf("%s\n",buff);

		/* re-install alarm handler */
		signal(SIGUSR1, signal_handler);
	} else {
		fprintf(stderr,"\n# Caught unexpected signal %d.\n",signum);
		fprintf(stderr,"# Terminating.\n");
		exit(102);
	}

}

int set_interface_attribs (int fd, int speed, int parity) {
	struct termios tty;

	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0) {
//		error_message ("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;	// disable break processing
	tty.c_lflag = 0;	// no signaling chars, no echo,
				// no canonical processing
	tty.c_oflag = 0;	// no remapping, no delays
	tty.c_cc[VMIN]  = 0;	// read doesn't block
	tty.c_cc[VTIME] = 5;	// 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
					// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0) {
//		error_message ("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int vmin, int vtime) {
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0) {
		fprintf(stderr,"# set_blocking unable to load tty attributes\n");
		return;
	}

	tty.c_cc[VMIN]  = vmin;	// minimum  number of characters for noncanonical read
	tty.c_cc[VTIME] = vtime; // timeout in deciseconds for noncanonical read

	if ( outputDebug ) {
		fprintf(stderr,"# set_blocking tty.c_cc[VMIN]=%d\n",tty.c_cc[VMIN]);
		fprintf(stderr,"# set_blocking tty.c_cc[VTIME]=%d\n",tty.c_cc[VTIME]);
	}

	if (tcsetattr (fd, TCSANOW, &tty) != 0) {
		printf("error %d setting term attributes", errno);
	}

}


void packet_processor(char *packet, int length, uint64_t microtime_start) {
	int i,data_pos;
	int lChecksum,rChecksum;
	int oldest_pos;

	/* quick sanity check */
	if ( length < 9 )
		return;

	/* null terminate so we can use string functions going forwards */
	packet[length]='\0';

	

	/* calculate local checksum */
	lChecksum=0;
	for ( i=1 ; i<length-3 ; i++ ) {
		lChecksum = lChecksum ^ packet[i];
	}

	/* read remote checksum */
	if ( 1 != sscanf(packet+length-2,"%x",&rChecksum) ) {
		if ( outputDebug ) {
			printf("(error scanning remote checksum hex)\n");
		}
		return;
	}

	/* compare local and remote checksums */
	if ( lChecksum != rChecksum ) {
		if ( outputDebug ) {
			printf("(remote and local checksum do not match!)\n");
		}
		return;
	}

	/* at this point we have a valid NMEA sentence */

	/* scan through data blocks and replace existing sentence with same talker
	or put in first available empty or put in place of oldest */
	for ( i=0, data_pos=-1,oldest_pos=0 ; i<DATA_BLOCK_N ; i++ ) {
		/* empty position */
		if ( -1 ==data_pos && '\0' == data_block[i].sentence[0] ) {
			data_pos=i;
		}

		/* match on first 6 characters */
		if ( 0 ==strncmp(data_block[i].sentence,packet,6) ) {
			data_pos=i;
			break;
		}

		/* update oldest_pos if we are older */
		if ( data_block[i].microtime_start < data_block[oldest_pos].microtime_start ) {
			oldest_pos=i;
		}
	}

	/* copy data to appropriate position */
	if ( -1 != data_pos ) {
		data_block[data_pos].microtime_start=microtime_start;
		strcpy(data_block[data_pos].sentence,packet);
	} else { 
		if ( outputDebug ) {
			printf("(replacing data block %d (oldest) with new data since all blocks are full)\n",oldest_pos);
		}

		data_block[oldest_pos].microtime_start=microtime_start;
		strcpy(data_block[oldest_pos].sentence,packet);
	}


}


void init() {
	int i;

	/* initialize the data strutures */
	for ( i=0 ; i < DATA_BLOCK_N ; i++ ) {
//		fprintf(stderr,"# initializing data_block[%d]\n",i);
		data_block[i].sentence[0]='\0';
		data_block[i].microtime_start=0;
	}

}

int json_to_client(int filedes) {
	char buff[DUMP_CURRENT_JSON_SUGGESTED_SIZE];

	dump_current_json(buff);
	write(filedes,buff,strlen(buff));
	return -1;
}


#define STATE_LOOKING_FOR_STX 0
#define STATE_IN_PACKET       1

void serial_process(int serialfd) {
	int i,n;
	uint64_t microtime_now;
	int milliseconds_since_stx;
	char buff[1];


	static char packet[128];
	static int packet_pos=0;
	static uint64_t microtime_start=0;
	static int state=STATE_LOOKING_FOR_STX;

	n = read (serialfd, buff, sizeof(buff));  // read next character if ready
	microtime_now=microtime();

//`	printf("# read buff[0]=%c\n",buff[0]);

	/* non-blocking, so we will get here if there was no data available */
	/* read timeout */
	if ( 0 == n ) {
		if ( outputDebug ) {
			printf("(read returned 0 bytes)\n",n);
		}
		return;
	}

	/* cancel pending alarm */
	alarm(0);
	/* set an alarm to send a SIGALARM if data not received within alarmSeconds */
	alarm(alarmSeconds);

	milliseconds_since_stx=(int) ((microtime_now-microtime_start)/1000.0);


	/* NMEA packets:
		start with '$'
		end with '\r' or '\n'
		get aborted on timeout
	*/

	/* copy byte to packet */
	for ( i=0 ; i<n ; i++ ) {
		/* look for start character */
		if ( STATE_LOOKING_FOR_STX == state && '$' ==  buff[i] ) {
			packet_pos=0;
			microtime_start=microtime_now;
			state=STATE_IN_PACKET;
			packet[0]='$';
		}

		if ( STATE_IN_PACKET == state ) {
//			printf("---> milliseconds_since_stx = %d\n",milliseconds_since_stx);
			if ( milliseconds_since_stx > milliseconds_timeout ) {
				packet_pos=0;
				state=STATE_LOOKING_FOR_STX;

				if ( outputDebug ) {
					printf("(timeout while reading NMEA sentence)\n");
				}
				continue;
			}
	
			if ( '\r' == buff[i] || '\n' == buff[i] ) {
				state=STATE_LOOKING_FOR_STX;

				/* process packet */
				packet_processor(packet,packet_pos,microtime_start);
			}

			if ( packet_pos < sizeof(packet)-1 ) {
				packet[packet_pos]=buff[i];
				packet_pos++;
			} else {
				if ( outputDebug ) {
					printf("(packet length exceeded length of buffer!)\n");
				}
			}

		}

	}

}

int main(int argc, char **argv) {
	char *portname = "/dev/ttyAMA0";
	int serialfd;

	int i,n;
	int tcpPort=2626;


	/* server socket */
	int sock;
	struct sockaddr_in name;
	fd_set active_fd_set, read_fd_set;
	struct sockaddr_in clientname;
	size_t size;


	/* command line arguments */
	while ((n = getopt (argc, argv, "a:hi:p:s:vt:")) != -1) {
		switch (n) {
			case 't':
				milliseconds_timeout=atoi(optarg);
				fprintf(stdout,"# timeout packet after %d milliseconds since start\n",milliseconds_timeout);
				break;
			case 'a':
				alarmSeconds=atoi(optarg);
				fprintf(stdout,"# terminate program after %d seconds without receiving data\n",alarmSeconds);
				break;
			case 'p':
				tcpPort=atoi(optarg);
				fprintf(stdout,"# TCP server port = %d\n",tcpPort);
				break;
			case 's':
				n=atoi(optarg);
				fprintf(stdout,"# Delaying startup for %d seconds ",n);
				fflush(stdout);
				for ( i=0 ; i<n ; i++ ) {
					sleep(1);
					fputc('.',stdout);
					fflush(stdout);
				}
				fprintf(stdout," done\n");
				fflush(stdout);
				break;
			case 'i':
				strncpy(portname,optarg,sizeof(portname));
				portname[sizeof(portname)-1]='\0';
				fprintf(stderr,"# serial port = %s\n",portname);
				break;
			case 'v':
				outputDebug=1;
				fprintf(stderr,"# verbose (debugging) output to stderr enabled\n");
				break;
			case 'h':
				fprintf(stdout,"# -a seconds\tTerminate after seconds without data\n");
				fprintf(stdout,"# -t milliseconds\tTimeout packet after milliseconds since start\n");
				fprintf(stdout,"# -s seconds\tstartup delay\n");
				fprintf(stdout,"# -p tcpPort\tTCP server port number\n");
				fprintf(stdout,"# -v\t\tOutput verbose / debugging to stderr\n");
				fprintf(stdout,"# -i\t\tserial device to use (default: /dev/ttyAMA0)\n");
				fprintf(stdout,"#\n");
				fprintf(stdout,"# -h\t\tThis help message then exit\n");
				fprintf(stdout,"#\n");
				exit(0);
		}
	}

	/* initialize data structures */
	init();

	/* create socket */
	sock = socket(PF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		fprintf(stderr,"# error creating socket. Aborting.\n");
		exit(1);
	}

	/* Give the socket a name. */
	name.sin_family = AF_INET;
	name.sin_port = htons(tcpPort);
	name.sin_addr.s_addr = htonl(INADDR_ANY);
	if ( bind( sock, (struct sockaddr *) &name, sizeof(name) ) < 0 ) {
		fprintf(stderr,"# error binding socket. Aborting.\n");
		exit(1);
	}
	/* setup socket to accept connections */
	if ( listen(sock, 1) < 0 ) {
		fprintf(stderr,"# error listening on socket. Aborting.\n");
		exit(1);
	}

	/* Initialize the set of active sockets. */
	FD_ZERO(&active_fd_set);
	/* add socket fd to fd_set for select() */
	FD_SET(sock, &active_fd_set);


	/* install signal handler */
	signal(SIGALRM, signal_handler); /* timeout */
	signal(SIGUSR1, signal_handler); /* user signal to do data block debug dump */
	signal(SIGPIPE, signal_handler); /* broken TCP connection */

	/* setup serial port for NMEA */
	serialfd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	
	if (serialfd < 0) {
		fprintf(stderr,"# error opening serial port. Aborting.\n");
		exit(1);
	}	

	/* NMEA runs at 4800 baud */
	set_interface_attribs (serialfd, B4800, 0);  // set speed to 4800 bps, 8n1 (no parity)
//	set_blocking (serialfd, 0, 100);		// blocking with 10 second timeout
	set_blocking (serialfd, 0, 0);		// blocking with 10 second timeout

	/* add serial fd to fd_set for select() */
	FD_SET(serialfd, &active_fd_set);

	/* set an alarm to send a SIGALARM if data not received within alarmSeconds */
	alarm(alarmSeconds);


	for ( ; ; ) {
		/* Block until input arrives on one or more active sockets. */
		read_fd_set = active_fd_set;

		if ( select(FD_SETSIZE, &read_fd_set, NULL, NULL, NULL) < 0 ) {
			fprintf(stderr,"# select() error. Aborting.\n");
			exit(1);
		}

		/* Service all the sockets with input pending. */
		for ( i=0 ; i < FD_SETSIZE ; ++i ) {
			if ( FD_ISSET(i, &read_fd_set) ) {
				if ( i == sock) {
					/* Connection request on original socket. */
					int new;
					size = sizeof(clientname);
					new = accept(sock, (struct sockaddr *) &clientname, &size);

					if ( new < 0 ) {
						fprintf(stderr,"# accept() error. Aborting.\n");
						exit(1);
					}

					fprintf(stderr, "Server: connect from host %s, port %d.\n", inet_ntoa (clientname.sin_addr), ntohs (clientname.sin_port));

					FD_SET(new, &active_fd_set);


					/* dump current data */
					json_to_client(new);

					/* disconnect */
					close(new);
					FD_CLR(new, &active_fd_set);
				} else if ( serialfd  == i ) {
					/* serial port has something to do */
					serial_process(serialfd);


				} else {
					printf("(fd=%d has something going on)\n");
				}
			}
		}
	}

	exit(0);
}
