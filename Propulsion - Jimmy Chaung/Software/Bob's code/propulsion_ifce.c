/*
 * propulsion_ifce.c
 *
 *  Created on: Aug 7, 2012
 *      Author: Bob
 */
#include <ctype.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#define  u32 uint32_t

#include "SNAPSv2-propulsion-ifce.h"

#define PGMNAME "propulsion_ifce"

// GPIO to Fire Thrusters
#define FIRE_GPIO "/sys/class/gpio/gpio10/value"
// i2c device address to which the Quad-Thruster Pod will answer.
#define I2C_DEV_ADDR 0x01

static char pnp_reference[] = {
		'V', '1', '.', '0', 0, 0, 0, 0,
		'S', 'N', 'A', 'P', 'S', 0, 0, 0,
		'T', 'P', 'O', 'D', '4', '0', '0', '1'};

static char filename[4][20];
static int	fd[4];
static int fd_fire = 0;
static int is_open	= 0;
struct SNAPS_PNP {
	char	pnp_string[24];
	int		ticks_per_sec;
	short 	config;
	short 	pad1;
	int		t1_cal;
	int		t2_cal;
	int		t3_cal;
	int		t4_cal;
};
static struct SNAPS_PNP snaps_pnp[4];

static struct FIRE_MSG {
	char	memadr;
	char	msglen;
	char	t1_l;
	char	t1_h;
	char	t2_l;
	char	t2_h;
	char	t3_l;
	char	t3_h;
	char	t4_l;
	char	t4_h;
} fire_msg;

static struct RDPNP_MSG {
	char	memadr;
	char	msglen;
} rdpnp_msg;

static struct i2c_msg rw_msg[2];

#define	METHODNAME PGMNAME ".open: "
extern int	propulsion_ifce_open() {
	int ret = 0;
	int rc = 0;
	int i;
	struct i2c_rdwr_ioctl_data  rdwr_struct;

	fd_fire = open(FIRE_GPIO, O_WRONLY);
	if (0>fd_fire) {
		fprintf(stderr, METHODNAME
				"Error - Open gpio10 failed.  rc= %d, errno=%d\n", fd_fire, errno);
		ret |= THRUSTER_NOGPIO | THRUSTER_ERROR;
		return ret;
	}
	// reset the firing control
	write(fd_fire, "0", 2);

	// initialize the read pnp message
	rw_msg[0].addr = I2C_DEV_ADDR;
	rw_msg[0].flags = 0;
	rw_msg[0].len = 2;
	rw_msg[0].buf = (char *) &rdpnp_msg;
	rw_msg[1].addr = I2C_DEV_ADDR;
	rw_msg[1].flags = I2C_M_RD;
	rw_msg[1].len = sizeof(snaps_pnp[0]);
	rdwr_struct.nmsgs = 2;
	rdwr_struct.msgs = rw_msg;

	for (i=0; i<4; i++) { // for each thruster port

		// open the i2c bus for the device
		snprintf(filename[i], sizeof(filename[0]), "/dev/i2c/%d", i+4);
		filename[i][sizeof(filename[0]) - 1] = '\0';
		fd[i] = open(filename[i], O_RDWR);

		if (fd[i] < 0 && (errno == ENOENT || errno == ENOTDIR)) {
			sprintf(filename[i], "/dev/i2c-%d", i+4);
			fd[i] = open(filename[i], O_RDWR);
		}

		if (fd[i] < 0) {
			fprintf(stderr, METHODNAME
					"Error - Open Port%d failed - no such bus.  rc= %d, errno=%d\n", i+1, fd[i], errno);
			ret |= THRUSTER_NOBUS<<(i*4) | THRUSTER_ERROR;
			continue;
		}

		// read the pnp data
		rw_msg[1].buf = (char *) &snaps_pnp[i];
		rc = ioctl(fd[i], I2C_RDWR, &rdwr_struct);
		if (rc < 0) {
			fprintf(stderr, METHODNAME
					"Error - Communication to Port%d failed - no device.  rc= %d, errno=%d\n", i+1, rc, errno);
			ret |= (THRUSTER_NODEV<<(i*4)) | THRUSTER_ERROR;
			continue;
		}

		// compare the PnP string to what is expected
		if (0!=strncmp((char *) &snaps_pnp[i].pnp_string[0], (char *) &pnp_reference[0], 8)
				|| 0!=strncmp((char *) &snaps_pnp[i].pnp_string[8], (char *) &pnp_reference[8], 8)
				|| 0!=strncmp((char *) &snaps_pnp[i].pnp_string[16], (char *) &pnp_reference[16], 8)
				) {
			fprintf(stderr, METHODNAME
					"Error - Device on Port%d type not supported.\n", i+1);
			fprintf(stderr, "  \"%-8.8s\"\n", (char *) &snaps_pnp[i].pnp_string[0]);
			fprintf(stderr, "  \"%-8.8s\"\n", (char *) &snaps_pnp[i].pnp_string[8]);
			fprintf(stderr, "  \"%-8.8s\"\n", (char *) &snaps_pnp[i].pnp_string[16]);
			ret |= THRUSTER_NOTSUP<<(i*4) | THRUSTER_ERROR;
			continue;
		}

	} // for each thruster port

	// if errors occured, close any open devices
	if (ret) {
		for (i=0; i<4; i++) {
			if (fd[i]!=0) {
				close(fd[i]);
				fd[i] = 0;
			}
		} // for
		close(fd_fire);
	} // if ret
	else { // ret == no errors
		is_open = 1;
		fire_msg.t3_l = 0;
		fire_msg.t3_h = 0;
		fire_msg.t4_l = 0;
		fire_msg.t4_h = 0;

	} // else ret == no errors

	return (ret);
} // method open

#undef METHODNAME
#define	METHODNAME PGMNAME ".close: "
extern void	propulsion_ifce_close() {
	int i;

	for (i=0; i<4; i++) {
		if (fd[i]!=0) {
			close(fd[i]);
			fd[i] = 0;
		}
		if (fd_fire) {
			// reset the firing control
			write(fd_fire, "0", 2);
			close(fd_fire);
			fd_fire = 0;
		}
	} // for
	is_open = 0;
} // method close

#undef METHODNAME
#define	METHODNAME PGMNAME ".fire: "
extern int	propulsion_ifce_fire(
	int x_thrust_yp, int x_thrust_yn, int y_thrust_xp, int y_thrust_xn) {
	struct i2c_rdwr_ioctl_data  rdwr_struct;
	int pod4[4];
	int pod5[4];
	int pod6[4];
	int pod7[4];
	int ret = 0;
	int rc = 0;
	int i;

	if (!is_open) {
		ret = propulsion_ifce_open();
		if (ret) return ret;
	}

	// reset the firing control
	write(fd_fire, "0", 2);

	// map virtual thruster parameters to thruster pod millisecond values
	if (x_thrust_yp>=0) {	pod5[0] = x_thrust_yp; 	pod6[0] = 0;}
	else {					pod5[0] = 0;			pod6[0] = -x_thrust_yp;}
	if (x_thrust_yn>=0) {	pod4[0] = x_thrust_yn; 	pod7[0] = 0;}
	else {					pod4[0] = 0;			pod7[0] = -x_thrust_yn;}
	if (y_thrust_xp>=0) {	pod7[1] = y_thrust_xp; 	pod6[1] = 0;}
	else {					pod7[1] = 0;			pod6[1] = -y_thrust_xp;}
	if (y_thrust_xn>=0) {	pod4[1] = y_thrust_xn; 	pod5[1] = 0;}
	else {					pod4[1] = 0;			pod5[1] = -y_thrust_xn;}

//	printf("   Pod1      Pod2      Pod3      Pod4\n");
//	for (i=0; i<2; i++) { // for each thruster in a pod (we only use 2 now)
//		// print values
//		printf("%8d, %8d, %8d, %8d\n", pod4[i], pod5[i], pod6[i], pod7[i]) ;
//	} // for each thruster in a pod

	for (i=0; i<2; i++) { // for each thruster in a pod (we only use 2 now)
		// convert time to ticks
		pod4[i] = (pod4[i] * snaps_pnp[0].ticks_per_sec) / 1000;
		pod5[i] = (pod5[i] * snaps_pnp[1].ticks_per_sec) / 1000;
		pod6[i] = (pod6[i] * snaps_pnp[2].ticks_per_sec) / 1000;
		pod7[i] = (pod7[i] * snaps_pnp[3].ticks_per_sec) / 1000;
	} // for each thruster in a pod

	// initialize the firing message
	rw_msg[0].addr = I2C_DEV_ADDR;
	rw_msg[0].flags = 0;
	rw_msg[0].len = sizeof(fire_msg);
	rw_msg[0].buf = (char *) &fire_msg;
	rdwr_struct.nmsgs = 1;
	rdwr_struct.msgs = rw_msg;
	fire_msg.memadr = 0x080;
	fire_msg.msglen = 11;

	// send message to the pod4
	fire_msg.t1_l = (char) pod4[0];
	fire_msg.t1_h = (char) (pod4[0]>>8);
	fire_msg.t2_l = (char) pod4[1];
	fire_msg.t2_h = (char) (pod4[1]>>8);
	rc = ioctl(fd[0], I2C_RDWR, &rdwr_struct);
	if (rc < 0) {
		fprintf(stderr, METHODNAME
				"Error - ioctl failed.  rc= %d, errno=%d\n", rc, errno);
		ret |= THRUSTER_NODEV<<(0*4) | THRUSTER_ERROR;
	}

	// send message to the pod5
	fire_msg.t1_l = (char) pod5[0];
	fire_msg.t1_h = (char) (pod5[0]>>8);
	fire_msg.t2_l = (char) pod5[1];
	fire_msg.t2_h = (char) (pod5[1]>>8);
	rc = ioctl(fd[1], I2C_RDWR, &rdwr_struct);
	if (rc < 0) {
		fprintf(stderr, METHODNAME
				"Error - ioctl failed.  rc= %d, errno=%d\n", ret, errno);
		ret |= THRUSTER_NODEV<<(1*4) | THRUSTER_ERROR;
	}

	// send message to the pod6
	fire_msg.t1_l = (char) pod6[0];
	fire_msg.t1_h = (char) (pod6[0]>>8);
	fire_msg.t2_l = (char) pod6[1];
	fire_msg.t2_h = (char) (pod6[1]>>8);
	rc = ioctl(fd[2], I2C_RDWR, &rdwr_struct);
	if (rc < 0) {
		fprintf(stderr, METHODNAME
				"Error - ioctl failed.  rc= %d, errno=%d\n", ret, errno);
		ret |= THRUSTER_NODEV<<(2*4) | THRUSTER_ERROR;
	}

	// send message to the pod7
	fire_msg.t1_l = (char) pod7[0];
	fire_msg.t1_h = (char) (pod7[0]>>8);
	fire_msg.t2_l = (char) pod7[1];
	fire_msg.t2_h = (char) (pod7[1]>>8);
	rc = ioctl(fd[3], I2C_RDWR, &rdwr_struct);
	if (rc < 0) {
		fprintf(stderr, METHODNAME
				"Error - ioctl failed.  rc= %d, errno=%d\n", ret, errno);
		ret |= THRUSTER_NODEV<<(3*4) | THRUSTER_ERROR;
	}

	// fire all the thrusters at once
	if (ret==0) {
		// fire the thrusters
		//printf("*** firing thrusters! ***\n");
		write(fd_fire, "1", 2);
	}

	//printf("fire returning: ret=%d\n", ret);
	return ret;
} // method propulsion_ifce_fire

#undef METHODNAME
#define	METHODNAME PGMNAME ".main: "
int main(int argc, const char *argv[]) {
	int fd;
	int rc = 0;
	//int ret = 0;
	int i;

	printf(PGMNAME ": opening thruster devices...\n");

	fd = propulsion_ifce_open();
	if (fd<0) {
		printf(METHODNAME "error while opening... %8X\n", fd);
		return 99;
	} //if fd

	for (i=0; i<15; i++) {
		// send command to the thrusters
		printf(METHODNAME "firing thrusters...800, 800, -500, -500\n");
		rc = propulsion_ifce_fire(800, 800, -500, -500);
		if (rc) {
			printf(METHODNAME "error firing thrusters.  %8.8X\n", rc);
			return rc;
		} //if rc
		sleep (1);
		printf(METHODNAME "firing thrusters...-800, -800, 100, 100\n");
		rc = propulsion_ifce_fire(-800, -800, 100, 100);
		if (rc) {
			printf(METHODNAME "error firing thrusters.  %8.8X\n", rc);
			return rc;
		} //if rc
		sleep (1);
	}

	// close the thrusters
	printf(METHODNAME "closing thruster device...\n");
	propulsion_ifce_close();

	//printf(PGMNAME "Exiting.\n");
	return rc;
} // main

