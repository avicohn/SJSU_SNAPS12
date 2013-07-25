//get_pwr_state.c
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
//#include <sys/ioctl.h>
#include <time.h>
#include <sys/time.h>
#include <getopt.h>
#include <errno.h>
#include <math.h>

#include "get_pwr_state.h"				//template defination
//#include "twl4030-madc.h"

#define ADC7 7

#define PWRVERSION 0
#define PWRREVISION 0
#define ALARMTHRESHOLD 0.2f							//set alarm limit 20% battery left
#define PWRCONSUMPTION 0.1f							//100mA
#define BATTERYCAP 60 * (2.4f / PWRCONSUMPTION)		//minutes

//for analog read
//commented out on jan 15, 2013, replaced with unsigned chars
//typedef uint8_t u8;
//typedef uint16_t u16;
#define uint8_t unsigned char;
#define uint16_t unsigned short;

// allocate memory for private local persistent copy of my state
static struct snaps_pwr_state_struct my_state = {
	.version = PWRVERSION,
	.revision = PWRREVISION,
	.alarm_threshold = ALARMTHRESHOLD,
	.status = 0
};

int ADC7read;							//for storing direct data from analog in read
int battery_voltage_MAX = 8.4106;		//full battery
int ADCtimeout = 100;					//timeout arbitrarily decided

//pointer to where state is in G&C caller's routine
extern void get_pwr_state(struct snaps_pwr_state_struct *snaps_pwr_state)  {

		//i2c bus 8 to device address 001, standard for Lego for FTPSC
		//get voltage of battery from reading ADC7 and multiplying by (7.1/2)

	/* NOT WORKING AS OF FEB 7, 2013
        if (( fp = open("/sys/class/hwmon/hwmon1/device/in7_input", "r")) == NULL)
        {
            printf("Can not open hwmon1 \n");
            exit(1);
        }
*/


		//store voltage of battery into float battery_voltage
        my_state.battery_voltage = ADC7read / 1000.0f;		//convert mV to V

		//set minutes_remaining look at battery discharge chart in slides
		my_state.minutes_remaining = ((-7E-09) * pow(my_state.battery_voltage,3) +
		(8E-06) * pow(my_state.battery_voltage,2) - (0.0045 * my_state.battery_voltage) + 8.4106);

		//calculate % of battery life left
		//store % battery left into battery_level
		my_state.battery_level = (my_state.minutes_remaining/BATTERYCAP);

		//compare current battery % to alarm and set status
        if (my_state.battery_level > my_state.alarm_threshold)
        {
            my_state.status = 0;
        }
        else if (my_state.battery_level > 0)
        {
            my_state.status = 1;
        }
		else
		{
			my_state.status = 2;
		}

		//move data from power struct to G&C struct
		snaps_pwr_state->version = my_state.version;
		snaps_pwr_state->revision = my_state.revision;
        snaps_pwr_state->battery_voltage = my_state.battery_voltage;
		snaps_pwr_state->minutes_remaining = my_state.minutes_remaining;
		snaps_pwr_state->battery_level = my_state.battery_level;
		snaps_pwr_state->status = my_state.status;
}
//verify that ADC7 has minimal noise
