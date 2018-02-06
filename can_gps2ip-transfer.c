/*
 * can_gps2ip-transfer.c
 *
 * reModeled by Ohira.
 * This program is transfer can packets infomation and GPS infomation from a can socket to from other ip network. 
 * To combine filtering program with Machine learning.
 *
 * usage:
 * 1)  ./alphard2ip-transfer			 : Analaze and Send Honda Alphard can packet(s) info
 * 2)  ./carrolla2ip-transfer			 : Analaze and Send Carrolla Fielder can packet(s) info
 * 3)  ./aqua2ip-transfer				 : Analaze and Send Toyota Aqua can packet(s) info
 * 4)  ./can2ip-transfer.dbg			 : To debug Analaze and Send Carrolla Fielder can packet(s) info
 *
 * how to compile:
 * ex) chmod a+x build.sh
 * ex) ./build.sh <alphard | carrolla | aqua | debug>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>
#include <netinet/in.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <linux/terminal.h>
#include <linux/lib.h>
#include <linux/lib.c>

#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <math.h>

#include "gps_module.h"
extern gps_t gps;
extern int gps_lock_flag;
void GPS_thread_main(void);

#define MAXSOCK 16    /* max. number of CAN interfaces given on the cmdline */
#define MAXIFNAMES 30 /* size of receive name index to omit ioctls */
#define MAXCOL 6      /* number of different colors for colorized output */
#define ANYDEV "any"  /* name of interface to receive from any CAN interface */
#define ANL "\r\n"    /* newline in ASC mode */

#define SILENT_INI 42 /* detect user setting on commandline */
#define SILENT_OFF 0  /* no silent mode */
#define SILENT_ANI 1  /* silent mode with animation */
#define SILENT_ON  2  /* silent mode (completely silent) */

#define BOLD    ATTBOLD
#define RED     ATTBOLD FGRED
#define GREEN   ATTBOLD FGGREEN
#define YELLOW  ATTBOLD FGYELLOW
#define BLUE    ATTBOLD FGBLUE
#define MAGENTA ATTBOLD FGMAGENTA
#define CYAN    ATTBOLD FGCYAN

const char col_on [MAXCOL][19] = {BLUE, RED, GREEN, BOLD, MAGENTA, CYAN};
const char col_off [] = ATTRESET;

static char *cmdlinename[MAXSOCK];
static __u32 dropcnt[MAXSOCK];
static __u32 last_dropcnt[MAXSOCK];
static char devname[MAXIFNAMES][IFNAMSIZ+1];
static int  dindex[MAXIFNAMES];
static int  max_devname_len; /* to prevent frazzled device name output */ 

#define MAXANI 4
extern int optind, opterr, optopt;

static volatile int running = 1;

void sigterm(int signo)
{
	running = 0;
}

#ifdef HA
char *vehicle_id = "Alphard";
//Honda Alphard Vehicle Speed
#define ANALAZE_CANID1 "158"
#define ANALAZE_PAY1_S 1
#define ANALAZE_PAY1_E 2
#define ANALAZE_PAY1_B 1.0

//Honda Alphard Engine Speed
#define ANALAZE_CANID2 "1DC"
#define ANALAZE_PAY2_S 4
#define ANALAZE_PAY2_E 4
#define ANALAZE_PAY2_B 0.1
#endif

#ifdef CF
char *vehicle_id = "Carrolla_Fielder";
//Carrolla Vehicle Speed
#define ANALAZE_CANID1 "0B4"
#define ANALAZE_PAY1_S 10
#define ANALAZE_PAY1_E 4
#define ANALAZE_PAY1_B 0.01

//Carrolla Engine Speed
#define ANALAZE_CANID2 "1C4"
#define ANALAZE_PAY2_S 0
#define ANALAZE_PAY2_E 4
#define ANALAZE_PAY2_B 1.0
#endif

#ifdef TA
char *vehicle_id = "Toyota_Aqua";
//Aqua Vehicle Speed
#define ANALAZE_CANID1 "0B4"
#define ANALAZE_PAY1_S 10
#define ANALAZE_PAY1_E 4
#define ANALAZE_PAY1_B 0.01

//Aqua Engine Speed
#define ANALAZE_CANID2 "1C4"
#define ANALAZE_PAY2_S 0
#define ANALAZE_PAY2_E 4
#define ANALAZE_PAY2_B 1.0

//Aqua Steering Angle
#define ANALAZE_CANID3 "025"
#define ANALAZE_PAY3_S 1
#define ANALAZE_PAY3_E 3
#define ANALAZE_PAY3_B 1.406

//Aqua Brake Info
#define ANALAZE_CANID4 "224"
#define ANALAZE_PAY4_S 9
#define ANALAZE_PAY4_E 3
#define ANALAZE_PAY4_B 1.0
#endif

//CAN width for Band Width
#define CAN_WIDTH 500000

#define DEG_MAX 5757.569824
#define DEG_MAX2 2899.172119
float global_speed;
float global_rpm;
int global_BrakeState;
float global_BrakePressRate;
struct timespec global_LastBrakeTime;
struct timespec global_LastStopTime;
struct timespec global_LastStr40TimeR;
struct timespec global_LastStr40TimeL;
struct timespec global_LastStrCenterTime;
float  global_StrAngle;
double global_latitude;
double global_longitude;

//Compose JSON Format
char *Composejson (float speed, float rpm, float bandwidth, double latitude, double longitude, char *jsonbuffer) {
	jsonbuffer = (char *)malloc(sizeof(char)*1024);
	sprintf(jsonbuffer, "{\"vehicleId\":\"%s\",\"status\":\"IGOFF\",\"speed\":%1.0f,\"rpm\":%1.0f,\"bandwidth\":%1.3f,\"GPS\":{\"lat\":%lf,\"lon\":%lf},\"LastBrakeTime\":%ld.%09ld,\"LastStopTime\":%ld.%09ld,\"LastStr40TimeR\":%ld.%09ld,\"LastStr40TimeL\":%ld.%09ld,\"LastStrCenterTime\":%ld.%09ld,\"StrAngle\":%1.3f}", \
			vehicle_id, speed, rpm, bandwidth, latitude, longitude, global_LastBrakeTime.tv_sec, global_LastBrakeTime.tv_nsec, global_LastStopTime.tv_sec, global_LastStopTime.tv_nsec, global_LastStr40TimeR.tv_sec, global_LastStr40TimeR.tv_nsec, global_LastStr40TimeL.tv_sec, global_LastStr40TimeL.tv_nsec, global_LastStrCenterTime.tv_sec, global_LastStrCenterTime.tv_nsec, global_StrAngle);
	return jsonbuffer;
}

/* Payload of 1C4 0B4...etc */
char *Createjsondata (float bandwidth, char *ret_json) {
	char jsonbuffer[500];
	if (gps_lock_flag == 1) {
		global_latitude = gps.latitude;
		global_longitude = gps.longitude;
#ifdef DEBAG
		printf("[main]	:[lati]%lf, [long]%lf\n", global_latitude, global_longitude);
#endif
	}
	ret_json = Composejson(global_speed, global_rpm, bandwidth, global_latitude, global_longitude, jsonbuffer);
	return ret_json;
}

void Global_speed_update(char *can_payload){
	char buf[500], can_ExtractCharData[20];
	int temp;
	strncpy(buf, can_payload + ANALAZE_PAY1_S, ANALAZE_PAY1_E);
	sscanf(buf, "%x", &temp);
	sprintf(can_ExtractCharData, "%d", temp);
	global_speed = atof(can_ExtractCharData) * ANALAZE_PAY1_B;
	if ((int)global_speed==0) {
		clock_gettime(CLOCK_REALTIME, &global_LastStopTime);
	}
#ifdef DEBAG
	//printf("[main]  :update speed:%lf\n", global_speed);
#endif
}

void Global_rpm_update(char *can_payload){
	char buf[500], can_ExtractCharData[20];
	int temp;
	strncpy(buf, can_payload + ANALAZE_PAY2_S, ANALAZE_PAY2_E);
	sscanf(buf, "%x", &temp);
	sprintf(can_ExtractCharData, "%d", temp);
	global_rpm = atof(can_ExtractCharData) * ANALAZE_PAY2_B;
#ifdef DEBAG
	//printf("[main]  :update rpm:%lf\n", global_rpm);
#endif
}

void Global_LastStrTime_update(char *can_payload){
	char buf[500], can_ExtractCharData[20];
	int temp;
	strncpy(buf, can_payload + ANALAZE_PAY3_S, ANALAZE_PAY3_E);
	sscanf(buf, "%x", &temp);
	temp /= 16;
	sprintf(can_ExtractCharData, "%d", temp);
	global_StrAngle = atof(can_ExtractCharData) * ANALAZE_PAY3_B;
	if (fabs(DEG_MAX-global_StrAngle) < 360) {
		global_StrAngle = global_StrAngle - DEG_MAX;
	} else if (fabs(DEG_MAX2-global_StrAngle < 360)) {
		global_StrAngle = global_StrAngle - DEG_MAX2;
	}
		//printf("[Angle]		%lf\n", global_StrAngle);
	if (global_StrAngle >= 40) {
		clock_gettime(CLOCK_REALTIME, &global_LastStr40TimeL);
	} else if (global_StrAngle <= -40) {
		clock_gettime(CLOCK_REALTIME, &global_LastStr40TimeR);
	} else {
		clock_gettime(CLOCK_REALTIME, &global_LastStrCenterTime);
	}
#ifdef DEBAG
	//printf("[main]  :update Steering Angle:%lf[degree] RAW:%s\n", global_LastStrTime, buf);
#endif
}

void Global_brake_update(char *can_payload){
	char buf[500], can_ExtractCharData[20];
	int temp;
	strncpy(buf, can_payload + ANALAZE_PAY4_S, ANALAZE_PAY4_E);
	sscanf(buf, "%x", &temp);
	temp /= 16;
	sprintf(can_ExtractCharData, "%d", temp);
	global_BrakePressRate = atof(can_ExtractCharData) * ANALAZE_PAY4_B;
	if (can_payload[0]=='2') {
		global_BrakeState = 1;
		clock_gettime(CLOCK_REALTIME, &global_LastBrakeTime);
	} else {
		global_BrakeState = 0;
	}
#ifdef DEBAG
	//printf("[main]  :update BrakeState:%d[degree] PressRate:%lf[%] RAW:%s\n", global_BrakeState, global_BrakePressRate, buf);
#endif
}

int main(void)
{
	fd_set rdfs;
	int s[MAXSOCK];
	int s2;
	int bridge = 0;
	useconds_t bridge_delay = 0;
	unsigned char timestamp = 0;
	unsigned char dropmonitor = 0;
	unsigned char silent = SILENT_INI;
	unsigned char silentani = 0;
	unsigned char color = 0;
	unsigned char view = 0;
	unsigned char log = 0;
	unsigned char logfrmt = 0;
	int count = 0;
	int rcvbuf_size = 0;
	int opt, ret;
	int currmax, numfilter;
	char *ptr, *nptr;
	struct sockaddr_can addr,addr2;
	int sock_ppp,sock_can;
 	struct sockaddr_in dest,addr1;
	char buf[1024],bufframe[1024];
	int maxfd;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
	struct iovec iov;
	struct msghdr msg;
	struct cmsghdr *cmsg;
	struct can_filter *rfilter;
	can_err_mask_t err_mask;
	struct can_frame frame;
	int nbytes, i;
	struct ifreq ifr;
	struct timespec ts, last_ts;
	FILE *logfile = NULL;
	char *jsondata;
	char *can_id;
	char *can_payload;
	int Accum_CANpacket = 0;
	int Accum_CANpacketcount = 0;
	float bandwidth = 0.0;
	int ret_thread;
	pthread_t gps_thread;
	int sendwait_count = 0;

	signal(SIGTERM, sigterm);
	signal(SIGHUP, sigterm);
	signal(SIGINT, sigterm);

	clock_gettime(CLOCK_REALTIME, &ts);

	//can=>soracom socket create
	sock_ppp = socket(AF_INET, SOCK_DGRAM, 0);
	dest.sin_family = AF_INET;
#ifdef DEBAG
	dest.sin_addr.s_addr = inet_addr("192.168.11.14"); //can=>ip
#else
	dest.sin_addr.s_addr = inet_addr("100.127.65.43"); //can=>soracom
#endif
	dest.sin_port = htons(23080);

	sock_can = socket(AF_INET, SOCK_DGRAM, 0);
	currmax = 1; /* find real number of CAN devices */

	for (i=0; i < currmax; i++) {

		ptr = "can0";
		nptr = strchr(ptr, ',');

		s[i] = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (s[i] < 0) {
			perror("socket");
			return 1;
		}

		cmdlinename[i] = ptr; /* save pointer to cmdline name of this socket */

		if (nptr)
			nbytes = nptr - ptr;  /* interface name is up the first ',' */
		else
			nbytes = strlen(ptr); /* no ',' found => no filter definitions */

		if (nbytes >= IFNAMSIZ) {
			fprintf(stderr, "name of CAN device '%s' is too long!\n", ptr);
			return 1;
		}

		if (nbytes > max_devname_len)
			max_devname_len = nbytes; /* for nice printing */

		addr.can_family = AF_CAN;

		memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
		strncpy(ifr.ifr_name, ptr, nbytes);

		if (strcmp(ANYDEV, ifr.ifr_name)) {
			if (ioctl(s[i], SIOCGIFINDEX, &ifr) < 0) {
				perror("SIOCGIFINDEX");
				exit(1);
			}
			addr.can_ifindex = ifr.ifr_ifindex;
		} else
			addr.can_ifindex = 0; /* any can interface */



		if (bind(s[i], (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			perror("bind");
			return 1;
		}
	}


	/* these settings are static and can be held out of the hot path */
	iov.iov_base = &frame;
	msg.msg_name = &addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;

	printf("start GPS thread\n");
	ret_thread = pthread_create(&gps_thread, NULL, (void *)GPS_thread_main, NULL);
	if (ret_thread != 0) {
		perror("GPS thread");
		return 1;
	}
	printf("start CAN dump\n");
	while (running) {

		FD_ZERO(&rdfs);
		for (i=0; i<currmax; i++)
			FD_SET(s[i], &rdfs);
		
		FD_SET(sock_can,&rdfs);
		
		if (s[0] > sock_can) {
		   maxfd = s[0];
		 } else {
		   maxfd = sock_can;
		 }

		if ((ret = select(maxfd+1, &rdfs, NULL, NULL, NULL)) < 0) {
			perror("select");
			running = 0;
			continue;
		}

		for (i=0; i<currmax; i++) {  /* check all CAN RAW sockets */

			if (FD_ISSET(s[i], &rdfs)) {

				int idx;

				/* these settings may be modified by recvmsg() */
				iov.iov_len = sizeof(frame);
				msg.msg_namelen = sizeof(addr);
				msg.msg_controllen = sizeof(ctrlmsg);  
				msg.msg_flags = 0;

				//CAN packet recv
				nbytes = recvmsg(s[i], &msg, 0);
#ifdef DEBAG
				//fprint_long_canframe(stdout, &frame, NULL, view);
				//printf("\n");
#endif
				sprint_canframe(bufframe, &frame, view);
				//CAN packet recv END

				//bandwidth calculation
				Accum_CANpacket += strlen(bufframe)-4;
				Accum_CANpacketcount += 1;
				clock_gettime(CLOCK_REALTIME, &last_ts);

				if ((last_ts.tv_sec - ts.tv_sec) >= 1) {
					bandwidth = (((float)Accum_CANpacket/2)*8.0 + (float)Accum_CANpacketcount*47)/CAN_WIDTH;
					Accum_CANpacket = 0;
					Accum_CANpacketcount = 0;
					clock_gettime(CLOCK_REALTIME, &ts);
					sendwait_count++;
				}//bandwidth calculation END

				//CAN packet Analaze
				can_id = strtok(bufframe, "#");
				can_payload = strtok(NULL, "#");
				if (!strcmp(can_id, ANALAZE_CANID1)) {
					Global_speed_update(can_payload);
				} else if (!strcmp(can_id, ANALAZE_CANID2)) {
					Global_rpm_update(can_payload);
				} else if (!strcmp(can_id, ANALAZE_CANID3)) {
					Global_LastStrTime_update(can_payload);
				} else if (!strcmp(can_id, ANALAZE_CANID4)) {
					Global_brake_update(can_payload);
				}//CAN packet Analaze END

				//JSON send
				if (sendwait_count >= 3) {
					jsondata = Createjsondata(bandwidth, bufframe);
					printf("[main]  :%s\n", jsondata);
					sendto(sock_ppp, jsondata, strlen(jsondata), 0, (struct sockaddr *)&dest, sizeof(dest));
					sendwait_count = 0;
					free(jsondata);
				}//JSON send END
			}
			out_fflush:
			fflush(stdout);
		}
	}

	for (i=0; i<currmax; i++)
		close(s[i]);
	
	close(sock_ppp);

	return 0;
}
