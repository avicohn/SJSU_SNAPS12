#include "get_gnc_state.h"  // read in the template definition
#include "get_pwr_state.h"
#include "get_nav_state.h"
#include "get_struct_state.h"
#include "get_prop_state.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define GNCVERSION 0
#define GNCREVISION 0
#define PWRVERSION 0
#define PWRREVISION 0
#define NAVVERSION 0
#define NAVREVISION 0
#define STRUCTVERSION 0
#define STRUCTREVISION 0
#define PROPVERSION 0
#define PROPREVISION 0

static int initialized = 0;        											//MS c compiler friendly, changed to static on jan 15, 2013

// allocate memory for private local persistent copy of my/each state
static struct snaps_gnc_state_struct my_state = {   						//static variable definitions, struct=group variables into a single record
    .version = GNCVERSION,
    .revision = GNCREVISION,
    .status = 0
};

static struct snaps_pwr_state_struct my_pwr_state;
static struct snaps_nav_state_struct my_nav_state;
static struct snaps_struct_state_struct my_struct_state;
static struct snaps_prop_state_struct my_prop_state;

int control_mode = 0;
char program_name[32] = "gnc_state";
int program_name_length = 0;
int program_line_being_executed = 0;
int program_elapsed_time = 0;
int status = 0;

extern void get_gnc_state(struct snaps_gnc_state_struct *callers_gnc_state)		//extern=identifier is defined elsewhere, void=fnc doesn't return a value
{
//obtain states from all subsystems
	get_pwr_state(&my_pwr_state);
	get_nav_state(&my_nav_state);
	get_struct_state(&my_struct_state);
	get_prop_state(&my_prop_state); 											// ask Propulsion to compute their state and copy it to your variable.
	callers_gnc_state->control_mode=control_mode;
	callers_gnc_state->version=my_state.version;								//"."accesses elements in a structure
	callers_gnc_state->revision=my_state.revision;
	callers_gnc_state->pwr_state_ptr=my_state.pwr_state_ptr;
}
