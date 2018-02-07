#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "gps_module.h"
gps_t gps;
int gps_lock_flag = 0;

#define BUF_SIZE 4096
int parse_lati_long(char *buf) {
	char *c_lati, *c_long;
	if ((c_lati=strtok(buf, ":")) == NULL ) {
		perror("strtok");
		return 1;
	}
	if ((gps.latitude = atof(c_lati)) == 0 ) {
		perror("parse");
		return 1;
	}
	if ((c_long=strtok(NULL, ":")) == NULL ) {
		perror("strtok");
		return 1;
	}
	if ((gps.longitude = atof(c_long)) == 0 ) {
		perror("parse");
		return 1;
	}
	return 0;
}

void LOCK_gps_flag(void) {
	gps_lock_flag = 0;
}
	
void UNLOCK_gps_flag(void) {
	gps_lock_flag = 1;
}

void GPS_thread_main(void) {
	int sock_gps;
	char recv_buf[BUF_SIZE];
	socklen_t sin_size;
	struct sockaddr_in addr_gps;

	sock_gps = socket(AF_INET, SOCK_DGRAM, 0);
	addr_gps.sin_family = AF_INET;
	addr_gps.sin_addr.s_addr = INADDR_ANY;
	addr_gps.sin_port = htons(4989);

	if (bind(sock_gps, (struct sockaddr *)&addr_gps, sizeof(addr_gps)) < 0) {
		perror("bind");
	}

#ifdef DEBAG
	printf("[gps_mod]  :start GPS main loop...\n");
#endif
	while(1) {
		memset(recv_buf, 0, sizeof(recv_buf));
		if (recvfrom(sock_gps, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_gps, &sin_size) < 0){
			perror("recvfrom");
		}
#ifdef DEBAG
 	   	printf("[gps_mod]  :%s", recv_buf);
#endif
		//$GPGGA parse
		LOCK_gps_flag();
		if (parse_lati_long(recv_buf)!=1) {
#ifdef DEBAG
			printf("[gps_mod]  :[lati] %lf, [long] %lf\n", gps.latitude, gps.longitude);
#endif
		}
		UNLOCK_gps_flag();
	}
}
