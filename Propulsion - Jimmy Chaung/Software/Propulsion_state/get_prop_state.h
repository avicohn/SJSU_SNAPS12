
#ifndef		get_prop_state_h
#define		get_prop_state_h


/*	Propulsion state structure:
 * 	The structure if the propulsion constains basic structures such as version and revision
 * 	Propulsion contains the following includes: fuel mass, alarm threshold, initial fuel mass,
 * 	current thrust capability, fuel remaining, and overall status of the satellite.
 * */

struct snaps_prop_state_struct {
	short version;	// Incr when new version is not backward compatible
	short revision;	// Incr when fields are added to the end
	float fuel_mass;				// kg
	float alarm_threshold;			// % for acceptable pressure
	float initial_fuel_mass;		// kg
	float max_thrust_capability;    // Newton
	float fuel_remaining;			// percent of capacity (above threshold)
	int status;						// 0 = all is well
									// 1 = pressure below threshold
									// 2 = out of gas
};

static float thruster_seconds_used;     //Thruster on in seconds
static float flight_elapsed_time = 30;  //flight time in seconds

float fuel_percent(float thruster_seconds_used,
                   float flight_elapsed_time,
                   float *fuel_percent_used);   //converts seconds thrust & seconds airbearing time to mass

extern void get_prop_state (struct snaps_prop_state_struct *snaps_prop_state);

//	3DoF Movement Cmds & queries
//	Fire thrusters subroutine call
//	Returns estimate of actual thrust delivered.
extern float thrust(
	float total_impulse,
	float *linear_thrust_x,
	float *linear_thrust_y);


//	Thruster inquiry subroutine call
//	Returns estimate of maximum thrust that can be delivered.
extern float thrust_inquiry(
	float *linear_thrust_x,
	float *linear_thrust_y);



//testing
int main(int argc, char* argv[]) {

	float max_thrust;
	struct snaps_prop_state_struct test_state;
	get_prop_state(&test_state);

	printf ("version =  %d\n", test_state.version);
	printf ("revision = %d\n", test_state.revision);
	printf ("Alarm threshold = %.2f\n", test_state.alarm_threshold);
	printf ("status = %d\n", test_state.status);
	//while(1==1);


	return(0);
}
#endif
