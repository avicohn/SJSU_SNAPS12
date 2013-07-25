
#ifndef		get_prop_state_h
#define		get_prop_state_h


/*	Propulsion state structure */
struct snaps_prop_state_struct {
	short version;	// Incr when new version is not backward compatible
	short revision;	// Incr when fields are added to the end
	float fuel_mass;				// kg 
	float alarm_threshold;			// kg for acceptable pressure
	float initial_fuel_mass;		// kg	
	float fuel_remaining;			// percent of capacity (above threshold)
	int status;						// 0 = all is well
									// 1 = pressure below threshold
									// 2 = out of gas
};

extern void get_prop_state (struct snaps_prop_state_struct *snaps_prop_state); 


//	3DoF Movement Cmds & queries
//	Fire thrusters subroutine call
//	Returns estimate of actual thrust delivered.
extern float thrust(
	float total_impulse,			// N*sec; impulse to be generated this period
	float linear_thrust_heading,	// degrees from body +X axis
	float rotational_thrust_dir);	// 1 = clockwise (positive r)
									// 2 = counter-clockwise (negative r) 

//	Thruster inquiry subroutine call
//	Returns estimate of maximum thrust that can be delivered.
extern float thrust_inquiry(
	float linear_thrust_heading,	// degrees from body +X axis
	float rotational_thrust_dir);	// 1 = clockwise (positive r)
									// 2 = counter-clockwise (negative r)

#endif