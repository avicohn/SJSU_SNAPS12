// Example of obtaining the state info from another subsystem, updating a private local persistant
// copy of your state, and copying your state into the memory provided by the program that called you.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "get_prop_state.h"						// read in the template definition

#define		PROPVERSION 0
#define		PROPREVISION 0
#define		STARTINGFUELMASS 1000f				//kg
#define		CAPACITYLEFT .1f					//kg
#define		CONSUMPTIONRATE .001f				//kg/s


// static variable declarations (these are remembered between calls)
static int initialized = 0;							// not initialized yet, changed to static on jan 15, 2013
static struct snaps_prop_state_struct my_state;
float mass_without_fuel = 2.2f;					// kg; mass with no fuel

extern void get_prop_state(struct snaps_prop_state_struct *snaps_prop_state) {
	if (!initialized) {
		my_state.version = PROPVERSION;
		my_state.revision = PROPREVISION;
		my_state.alarm_threshold = CAPACITYLEFT;
		my_state.status = 0;
	} //not initialized

	//my Program goes here
	my_state.fuel_mass = 0.5;			// kg 
	my_state.alarm_threshold= 0.1f;		// kg for acceptable pressure
	my_state.initial_fuel_mass= 1;		// kg	
	my_state.fuel_remaining = 10;		// percent of capacity (above threshold)

	/* produces errors
	// Obtain fuel_mass from the Propulsion subsystem
	get_prop_state(&snaps_prop_state);				// ask Propulsion to compute their state and copy it to your variable.
	my_state.fuel_mass = mass_without_fuel + snaps_prop_state.fuel_mass;
	//<<<<Compute the rest of your new state and store it in your my_state structure.>>>>
	*/
	
	// Copy your state into the calling routines memory.
	snaps_prop_state->version = my_state.version;
	snaps_prop_state->revision = my_state.revision;
	snaps_prop_state->alarm_threshold = my_state.alarm_threshold;
	snaps_prop_state->status = my_state.status;

} // get_prop_state subroutine
