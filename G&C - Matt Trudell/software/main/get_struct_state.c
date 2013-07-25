/*
 * get_struct_state.c
 *
 *  Created on: Nov 14, 2012
 *      Author: NhatNguyen
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "get_struct_state.h" // read in the template definition
#include "get_prop_state.h" // read in the template definition
#include <math.h>

//on jan 15, moment_tensor and center_of_mass_offset commmented out, because there is nothing after the define label so the compiler is changing each instance
//#define moment_tensor // define moment of inertia Ixx, Ixy, Ixz, Iyx, etc ...//
//#define center_of_mass_offset //distance CG offset from origin//
#define MASS_WITHOUT_FUEL 2.5
#define STRUCTVERSION 0
#define STRUCTREVISION 0

static int initialized = 0;
//get_struct_state (&struct_state);

// static variable declarations (these are remembered between calls)
static struct snaps_struct_state_struct my_struct_state ={ // allocate memory for private local persistant copy of my state
	.version = STRUCTVERSION,
	.revision = STRUCTREVISION,
	.status = 0
};



extern void get_struct_state(struct snaps_struct_state_struct *callers_struct_state) {
    struct snaps_prop_state_struct prop_state; // allocate memory for private temporary copy of prop_state

                                                // Obtain fuel_mass from the Propulsion subsystem
    get_prop_state(&prop_state); // ask Propulsion to compute their state and copy it to your variable.

    // Compute the rest of your new state and store it in your my_struct_state structure

    my_struct_state.mass = MASS_WITHOUT_FUEL + prop_state.fuel_mass; // kg; mass with fuel
    my_struct_state.moment_tensor[0][0]= INFINITY; 			// moment of inertia Ixx,
    my_struct_state.moment_tensor[0][1]= 0.0; 				// moment of inertia Ixy
    my_struct_state.moment_tensor[0][2]= 0.0; 				// moment of inertia Ixz
    my_struct_state.moment_tensor[1][0]= 0.0; 				// moment of inertia Iyx
    my_struct_state.moment_tensor[1][1]= INFINITY; 			// moment of inertia Iyy,
    my_struct_state.moment_tensor[1][2]= 0.0; 				// moment of inertia Iyz,
    my_struct_state.moment_tensor[2][0]= 0.0; 				// moment of inertia Izx,
    my_struct_state.moment_tensor[2][1]= 0.0; 				// moment of inertia Izy,
    my_struct_state.moment_tensor[2][2]= 0.0;     			// moment of inertia Izz,

    my_struct_state.center_of_mass_offset[0]= 0; 			// meters from body origin
    my_struct_state.center_of_mass_offset[1]= 0;
    my_struct_state.center_of_mass_offset[2]= 0;			// Copy your state into the calling routines memory.

    // move data from structure to G&C
    callers_struct_state->version = my_struct_state.version;
    callers_struct_state->revision = my_struct_state.revision;
    callers_struct_state->mass = my_struct_state.mass;

    callers_struct_state->moment_tensor[0][0] = my_struct_state.moment_tensor[0][0];
    callers_struct_state->moment_tensor[0][1] = my_struct_state.moment_tensor[0][1];
    callers_struct_state->moment_tensor[0][2] = my_struct_state.moment_tensor[0][2];
    callers_struct_state->moment_tensor[1][0] = my_struct_state.moment_tensor[1][0];
    callers_struct_state->moment_tensor[1][1] = my_struct_state.moment_tensor[1][1];
    callers_struct_state->moment_tensor[1][2] = my_struct_state.moment_tensor[1][2];
    callers_struct_state->moment_tensor[2][0] = my_struct_state.moment_tensor[2][0];
    callers_struct_state->moment_tensor[2][1] = my_struct_state.moment_tensor[2][1];
    callers_struct_state->moment_tensor[2][2] = my_struct_state.moment_tensor[2][2];

    callers_struct_state->center_of_mass_offset[0]= my_struct_state.center_of_mass_offset[0];
    callers_struct_state->center_of_mass_offset[1]= my_struct_state.center_of_mass_offset[1];
    callers_struct_state->center_of_mass_offset[2]= my_struct_state.center_of_mass_offset[2];
    // get_struc_state subroutine
    // callers_struct_state->mass = my_struct_state.mass;
    // Copy rest of your state to the caller�s memory.>>>>
}
