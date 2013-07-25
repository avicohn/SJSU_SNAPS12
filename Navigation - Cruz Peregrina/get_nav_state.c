#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "get_nav_state.h"

#define NAVVERSION 0            // Global Constants
#define NAVREVISION 0

// allocate memory for private local persistent copy of my state
struct snaps_nav_state_struct my_nav_state = {
    .version 	= NAVVERSION,
    .revision 	= NAVREVISION,
    .status 	= 0
};

float attitude_dcm[3][3];   // attitude direction cosine matrix
float bcs_acceleration[3];  // m/sec^2; BCS(x, y, z)
float bcs_velocity[3];      // m/sec; BCS(x, y, z)
float wcs_acceleration[3];  // m/sec^2; WCS(x, y, z)
float wcs_velocity[3];      // m/sec; WCS(x, y, z)
float wcs_position[3];      // m; WCS(x, y, z)


extern void get_nav_state(struct snaps_nav_state_struct *snaps_nav_state) {

    my_nav_state.attitude_dcm[0][0] 	= 1.0;
    my_nav_state.attitude_dcm[0][1] 	= 0;
    my_nav_state.attitude_dcm[0][2] 	= 0;
    my_nav_state.attitude_dcm[1][0] 	= 0;
    my_nav_state.attitude_dcm[1][1] 	= 1.0;
    my_nav_state.attitude_dcm[1][2] 	= 0;
    my_nav_state.attitude_dcm[2][0] 	= 0;
    my_nav_state.attitude_dcm[2][1] 	= 0;
    my_nav_state.attitude_dcm[2][2] 	= 1.0;

    my_nav_state.bcs_acceleration[0] 	= .01;
    my_nav_state.bcs_acceleration[1] 	= .02;
    my_nav_state.bcs_acceleration[2] 	= 0;

    my_nav_state.bcs_velocity[0] 		= .01;
    my_nav_state.bcs_velocity[1] 		= .02;
    my_nav_state.bcs_velocity[2] 		= 0;

    my_nav_state.wcs_acceleration[0] 	= .02;
    my_nav_state.wcs_acceleration[1] 	= .03;
    my_nav_state.wcs_acceleration[2] 	= 0;

    my_nav_state.wcs_velocity[0] 		= .01;
    my_nav_state.wcs_velocity[1] 		= .01;
    my_nav_state.wcs_velocity[2] 		= 0;

    my_nav_state.wcs_position[0] 		= 1;
    my_nav_state.wcs_position[1] 		= 2;
    my_nav_state.wcs_position[2] 		= 0;

    snaps_nav_state->attitude_dcm[0][0] = my_nav_state.attitude_dcm[0][0]; // Copy your state into the calling routines memory.
    snaps_nav_state->attitude_dcm[0][1] = my_nav_state.attitude_dcm[0][1];
    snaps_nav_state->attitude_dcm[0][2] = my_nav_state.attitude_dcm[0][2];
    snaps_nav_state->attitude_dcm[1][0] = my_nav_state.attitude_dcm[1][0];
    snaps_nav_state->attitude_dcm[1][1] = my_nav_state.attitude_dcm[1][1];
    snaps_nav_state->attitude_dcm[1][2] = my_nav_state.attitude_dcm[1][2];
    snaps_nav_state->attitude_dcm[2][0] = my_nav_state.attitude_dcm[2][0];
    snaps_nav_state->attitude_dcm[2][1] = my_nav_state.attitude_dcm[2][1];
    snaps_nav_state->attitude_dcm[2][2] = my_nav_state.attitude_dcm[2][2];

    snaps_nav_state->bcs_acceleration[0] = my_nav_state.bcs_acceleration[0];
    snaps_nav_state->bcs_acceleration[1] = my_nav_state.bcs_acceleration[1];
    snaps_nav_state->bcs_acceleration[2] = my_nav_state.bcs_acceleration[2];

    snaps_nav_state->bcs_velocity[0] 	= my_nav_state.bcs_velocity[0];
    snaps_nav_state->bcs_velocity[1] 	= my_nav_state.bcs_velocity[1];
    snaps_nav_state->bcs_velocity[2] 	= my_nav_state.bcs_velocity[2];

    snaps_nav_state->wcs_acceleration[0] = my_nav_state.wcs_acceleration[0];
    snaps_nav_state->wcs_acceleration[1] = my_nav_state.wcs_acceleration[1];
    snaps_nav_state->wcs_acceleration[2] = my_nav_state.wcs_acceleration[2];

    snaps_nav_state->wcs_velocity[0] 	= my_nav_state.wcs_velocity[0];
    snaps_nav_state->wcs_velocity[1] 	= my_nav_state.wcs_velocity[1];
    snaps_nav_state->wcs_velocity[2] 	= my_nav_state.wcs_velocity[2];

    snaps_nav_state->wcs_position[0] 	= my_nav_state.wcs_position[0];
    snaps_nav_state->wcs_position[1] 	= my_nav_state.wcs_position[1];
    snaps_nav_state->wcs_position[2] 	= my_nav_state.wcs_position[2];

    snaps_nav_state->version 			= my_nav_state.version;
    snaps_nav_state->revision			= my_nav_state.revision;

}
