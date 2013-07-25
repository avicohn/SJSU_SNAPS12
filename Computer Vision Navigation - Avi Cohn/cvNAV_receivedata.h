#ifndef CVNAV_RECEIVEDATA_H_
#define CVNAV_RECEIVEDATA_H_

/**
 * Opens the connection to CV Navigation System.
 * Returns: The file descriptor (a value >= 0).
 * 	-1 = An error occurred opening the device.
 *	ERRNO is set with an error code.
 */
int receivedata_open();

/**
 * Closes the connection to CV Navigation System
 * Parameters:
 * 	fd = file descriptor
 */
void receivedata_close(int fd);

/**
 * asks for Navigation data to get_nav_state.c
 * Parameters:
 * 	fd = file descriptor
 * Returns: error codes for good, partial, bad
 *   0 = Action was successful.
 *  -1 = Error transmitting. ERRNO is set with more info.
 *  -2 = Error response received. ERRNO is set with more info.
 *  -3 = Invalid file descriptor.
 */
int receivedata_action(int fd, int disconnect);

#endif /* CVNAV_RECEIVEDATA_H_ */

