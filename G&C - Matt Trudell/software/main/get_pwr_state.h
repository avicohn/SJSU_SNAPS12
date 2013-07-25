//get_pwr_state.h
//Power State

#ifndef get_pwr_state_h
#define get_pwr_state_h

/* Power state structure */
struct snaps_pwr_state_struct {
	short version; 				// Incr when new version is not backward compatible short revision;
	short revision;				// Incr when fields are added to the end
	float battery_voltage;  	// Volts
	float minutes_remaining; 	// to alarm point at current usage rate
	float alarm_threshold; 		// percent of battery capacity
	float battery_level; 		// percent charged
	int status;					// 0 = all is well
								// 1 = low battery; alarm
								// 2 = request emergency shutdown
};
extern void get_pwr_state(struct snaps_pwr_state_struct *snaps_pwr_state);

#endif
