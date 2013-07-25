#ifndef CVNAV_SENDDATA_H_
#define CVNAV_SENDDATA_H_

/**
 * Opens the connection to SNAPS satellite.
 * Returns: The file descriptor (a value >= 0).
 * 	-1 = An error occurred opening the device.
 *	ERRNO is set with an error code.
 */
int senddata_open();

/**
 * Closes the connection to SNAPS satellite.
 * Parameters:
 * 	fd = file descriptor
 */
void senddata_close(int fd);

/**
 * Sends Navigation data to get_nav_state.c
 * Parameters:
 * 	fd = file descriptor
 * Returns: error codes for good, partial, bad
 *   0 = Action was successful.
 *  -1 = Error transmitting. ERRNO is set with more info.
 *  -2 = Error response received. ERRNO is set with more info.
 *  -3 = Invalid file descriptor.
 */
int senddata_action(int fd);

#endif /* CVNAV_SENDDATA_H_ */
