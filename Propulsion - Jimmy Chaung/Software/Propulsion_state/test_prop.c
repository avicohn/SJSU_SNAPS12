#include <stdio.h>
#include <stdlib.h>
#include "get_prop_state.h"					// read in the template definition

/*int main(int argc, char* argv[]) {

	float max_thrust;
	struct snaps_prop_state_struct test_state;
	get_prop_state(&test_state);
	/*test_state.version = 1;
	test_state.revision = 2;
	test_state.alarm_threshold = 3;
	test_state.status = 4;
	*/
	printf ("version =  %d\n", test_state.version);
	printf ("revision = %d\n", test_state.revision);
	printf ("Alarm threshold = %.2f\n", test_state.alarm_threshold);
	printf ("status = %d\n", test_state.status);
	//while(1==1);

 /*max_thrust = thrust(
	float total_impulse,			// N*sec; impulse to be generated this period
	float linear_thrust_heading,	// degrees from body +X axis
	float rotational_thrust_dir);

	return(0);
}
*/


