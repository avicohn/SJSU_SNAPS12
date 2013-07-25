#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "get_prop_state.h"						// read in the template definition

#define		PROPVERSION 1
#define		PROPREVISION 1
#define		STARTINGFUELMASS 1000f				//kg
#define		CAPACITYLEFT .1f					//fraction left (Note it should be a function)
#define		CONSUMPTIONRATE .001f				//kg/s


// static variable declarations (these are remembered between calls)
static int initialized = 0;						// not initialized yet if zero
static struct snaps_prop_state_struct prop_state;
static float mass_without_fuel = 2.2f;			// kg; mass with no fuel
												// The “f” suffix tells “C” to format the 2.2 as float rather than double.
												// without the “f” the compiler will complain about “loss of precision”.
static float fuel_mass = 1.0f;                    // kg mass of fuel
extern void get_prop_state(struct snaps_prop_state_struct *snaps_prop_state) {
	if (!initialized) {
		prop_state.version = PROPVERSION;
		prop_state.revision = PROPREVISION;
		prop_state.alarm_threshold = CAPACITYLEFT;
		prop_state.status = 0;
	} //not initialized
	//my Program goes here
	prop_state.fuel_mass = 0.5;			// kg
	prop_state.alarm_threshold= 0.1f;	// kg for acceptable pressure
	prop_state.initial_fuel_mass= 1;	// kg
	prop_state.fuel_remaining = 10;		// percent of capacity (above threshold)

	// Obtain fuel_mass from the Propulsion subsystem
	// get_prop_state(&prop_state);				// ask Propulsion to compute their state and copy it to your variable.
	//my_state.mass = mass_without_fuel + prop_state.fuel_mass;
	//<<<<Compute the rest of your new state and store it in your my_state structure.>>>>

	// Copy your state into the calling routines memory.
	snaps_prop_state->version = prop_state.version;
	snaps_prop_state->revision = prop_state.revision;
	snaps_prop_state->alarm_threshold = prop_state.alarm_threshold;
	snaps_prop_state->status = prop_state.status;

} // get_prop_state subroutine

float mass(float fuel_percent, mass_without_fuel, mass_with_fuel)
{
    float snaps_mass;
    snaps_mass = mass_without_fuel + fuel_percent * mass_with_fuel;
    return snaps_mass;
}
float fuel_percent(float thruster_seconds_used, float flight_elapsed_time, float *fuel_percent_used)
{
    float max_thrust_endurance = 240;       //seconds
    float max_air_bearing_endurance = 720;  //seconds
    float thrust_fuel_percent_used;
    float air_bearing_fuel_percent_used;
    thrust_fuel_percent_used = (max_thrust_endurance - thruster_seconds_used)/max_thrust_endurance;
    air_bearing_fuel_percent_used = (max_air_bearing_endurance - air_bearing_fuel_percent_used)/max_air_bearing_endurance;
    *fuel_percent_used = thrust_fuel_percent_used + air_bearing_fuel_percent_used;
    return 0;
}

extern float thrust(
	float total_impulse,
	float *linear_thrust_x,
	float *linear_thrust_y)
	{
	    float max_thrust_per_nozzle = 0.17;
        float thrust_per_nozzle = 0.8 * max_thrust_per_nozzle; //Newton
        float magnitude = sqrt(linear_thrust_x^2 + linear_thrust_y^2);
        float x, y;
        //normalize the vector
        x = linear_thrust_x/magnitude;
        y = linear_thrust_y/magnitude;
        linear_thrust_x = 2 * x * thrust_per_nozzle;
        linear_thrust_y = 2 * y * thrust_per_nozzle;

        /* then call on bob's thrust fire code - the manual control sub-routine
        fire thruster
        return total impulse give in term of force times time
        */
	}

extern float thrust_inquiry(
	float *linear_thrust_x,
	float *linear_thrust_y)
	{
	    float max_thrust_per_nozzle = 0.17;
        float thrust_per_nozzle = 0.8 * max_thrust_per_nozzle; //Newton
        float magnitude = sqrt(linear_thrust_x^2 + linear_thrust_y^2);
        float x, y;
        //normalize the vector
        x = linear_thrust_x/magnitude;
        y = linear_thrust_y/magnitude;
        linear_thrust_x = 2 * x * thrust_per_nozzle;
        linear_thrust_y = 2 * y * thrust_per_nozzle;
	}

