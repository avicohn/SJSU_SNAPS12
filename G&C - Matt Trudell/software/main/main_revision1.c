/*main_revision1.c
 *
 * Author: Matthew Trudell
 * Created: 4-21-13
 * Description: Main G&C program.
 * Notes: See 2012-2013 G&C report for reference.
 */

#include "row_template.h"												//format of template_matrix defined here
#include "get_gnc_state.h"
#include <math.h>
#include <stdio.h>

#define THRUST_DURATION 0.8												//define fraction of 1sec interval that the thruster fires. In this case, .8 sec.
#define THRUST_SCALAR 0.8												//this is to reduce accel to allow for extra accel that may be required for speed corrections
#define V_PARA_PRC_ERR 10												//acceptable percentage of variance from expected velocity
#define V_PERP_PRC_ERR 10												//ASK USER TO DEFINE THESE, OR DEFINE HERE
#define MASS 7															//**HOW TO GET THIS FROM STRUCTURE? in kg

//function prototypes
void distance_and_velocity(int, float, float, float, float, float *, float *);								//returns expected d and expected v, use pointers?
void velocity_error_range(int, float, float, float, float, float, float, float *, float *, float *);		//returns 3 range values, use pointers?
void thrust_heading(float, float, float *);																	//returns heading
void impulse_magnitude(float, float, float *);																//returns impulse magnitude

//executepath main program
void execute_path ()
{

	//create structs for TM data
	struct template_matrix
	{
		int last_entry;
		struct row_template_matrix row[10000];							//declare struct to store TM parameters supplied by functions
	} data_structure;													//assign memory for struct

	//ask for/get destination coordinates and calculate distance
	float d, target_coord_wcs_x, target_coord_wcs_y;					//d=distance to target in METERS, from calculation below
	printf("Enter X Coordinate \n");
	scanf("%f", &target_coord_wcs_x);
	printf("Enter Y Coordinate \n");
	scanf("%f", &target_coord_wcs_y);

	//calculate dist to target
	d = sqrt(pow(target_coord_wcs_x, 2) + pow(target_coord_wcs_y, 2));

	float a, a_max, v_ini_1, v_ini_2, t1, t2;							//a=accel used for calculations, v_ini_1=initial velocity, only nonzero if TM is recalculated during flight
																		//d1_final=distance from start point after accel phase, v_ini_2=initial velocity at start of 2nd phase
																		//t1=total duration of phase 1, t2=total duration of phase 2
//TODO: ggg
	//get heading to target before asking for a_max
	//thrust_heading(target_coord_wcs_x, target_coord_wcs_y, thrust_heading_des);		//this should be commented out if not communicating with prop.

	int row = 0;
	v_ini_1 = 0; 														//current velocity magnitude from nav. this is set to zero for initial testing
	a_max 	= 0.045; //max accel in desired direction;						//**HOW TO GET THIS FROM PROP? Query prop how to get max accel in desired direction
	a 		= THRUST_DURATION * THRUST_SCALAR * a_max;
	t1 		= sqrt(pow(v_ini_1, 2) + a*d)/a;							//define total duration of acceleration phase of flight, t1
	v_ini_2 = v_ini_1 + a*t1;     										//define v_ini_2 for phase 2 calcs. t = sqrt(v_ini_1^2 + a*d)/a
	t2 		= v_ini_2/a;												//duration of deceleration phase, t2
	int last_row = t1+t2;

	//run through TM functions and store results in TM struct
	int i = 0;
	struct row_template_matrix *ptr;
	for(i=0; i<=t1; i++)
	{
		//run through functions and store data in struct
		ptr = &data_structure.row[i];																//ptr to each row of struct
		ptr->t = i;
		distance_and_velocity(i, v_ini_1, a, v_ini_2, t1, &(ptr->v_exp), &(ptr->d_exp));			//**RETURN d AND v.
		velocity_error_range(i, t1, v_ini_1, a, V_PARA_PRC_ERR, V_PERP_PRC_ERR, v_ini_2, &(ptr->v_para_range_low), &(ptr->v_para_range_high), &(ptr->v_perp_max));	//**RETURN v_para_range_low, v_para_range_high, v_perp_max. HOW?
		thrust_heading(target_coord_wcs_x, target_coord_wcs_y, &(ptr->thrust_heading_des));		    //**RETURN thrust_heading
		impulse_magnitude(a, MASS, &(ptr->impulse_des));											//**RETURN impulse_magnitude
	}
	for(i=t1+1; i<t1+t2; i++)
	{
		ptr = &data_structure.row[i];																//ptr to each row of struct
		ptr->t = i;
		distance_and_velocity(i, v_ini_1, a, v_ini_2, t1, &(ptr->v_exp), &(ptr->d_exp));			//**RETURN d AND v.
		velocity_error_range(i, t1, v_ini_1, a, V_PARA_PRC_ERR, V_PERP_PRC_ERR, v_ini_2, &(ptr->v_para_range_low), &(ptr->v_para_range_high), &(ptr->v_perp_max));	//**RETURN v_para_range_low, v_para_range_high, v_perp_max. HOW?
		thrust_heading(target_coord_wcs_x, target_coord_wcs_y, &(ptr->thrust_heading_des));			//**RETURN thrust_heading
		impulse_magnitude(a, MASS, &(ptr->impulse_des));											//**RETURN impulse_magnitude
	}

//FOR TEST: this will print the contents of the TM struct

	 for (row=0; row<last_row; row++)
	{

	    printf("[%3.3d]: d_exp=(%f), v_exp=(%f), min acceptable speed=(%f), max acceptable speed=(%f), max acceptable vel vector variance=(%f),"
	    		"desired thrust heading=(%f), desired impulse=(%f) \n", row, data_structure.row[row].d_exp, data_structure.row[row].v_exp,
	    		data_structure.row[row].v_para_range_low, data_structure.row[row].v_para_range_high, data_structure.row[row].v_perp_max,
	    		data_structure.row[row].thrust_heading_des, data_structure.row[row].impulse_des);
	}

	/* example for above
	for (row=0; row<last_row; row++)
	{
	    printf("[%3.3d]: d_exp=(%f), \n", row, data_structure.row[row].d_exp);
	}
	*/

//begin control loop. for each second:
//TODO:
/*	for(t=0, t<=??, t++);					//use do while instead?
	{
		//check subsystem status. take action if off nominal.
		//issue first thrust command. "impulse magnitude at heading xx"
		//check for velocity magnitude and vector deviations. take action if necessary.
	}
*/
}

//function definitions

//dist and vel
void distance_and_velocity(int t, float v_ini_1, float a, float v_ini_2, float t1, float *dptr, float *vptr)
{
	float d1_final;

	//compute expected distance and velocity for each second
	if(t<=t1)																				//if in first phase (accel phase) of flight
	{
		*dptr 		= v_ini_1*t + (1/2)*a*pow(t,2);											//expected distance from origin during first phase
		*vptr 		= v_ini_1 + a*t;														//expected velocity during first phase
	}
	else
	{
		d1_final 	= v_ini_1*t + (1/2)*a*pow(t,2);											// = d1_exp = distance at end of phase 1
		*dptr 		= d1_final + (v_ini_2*(t-t1) - (1/2)*a*pow((t-t1),2));					//expected distance from origin during 2nd phase
		*vptr 		= v_ini_2 - a*(t-t1);													//expected velocity during 2nd phase
	}
}

//vel error range
void velocity_error_range(int t, float t1, float v_ini_1, float a, float v_para_prc_err, float v_perp_prc_err, float v_ini_2, float *v_para_range_low_ptr, float *v_para_range_high_ptr, float *v_perp_max_ptr)
{
	float v_exp;

	if(t<=t1)
	{
		v_exp 					= v_ini_1 + a*t;							// = v1_exp = expected velocity during the 1st of flight
		*v_para_range_low_ptr 	= (1 - v_para_prc_err/100) * v_exp;			//minimum acceptable value of vel. vector component parallel to current actual velocity vector
		*v_para_range_high_ptr 	= (1 + v_para_prc_err/100) * v_exp;			//maximum "...", aka speed feedback.
		*v_perp_max_ptr			= (v_perp_prc_err/100) * v_exp;				//maximum acceptable value of vel. vector component perpendicular to current actual velocity vector
	}
	else
	{
		v_exp 					= v_ini_2 - a*(t-t1);						// = v2_exp = expected velocity during 2nd phase of flight
		*v_para_range_low_ptr 	= (1 - v_para_prc_err/100) * v_exp;
		*v_para_range_high_ptr 	= (1 + v_para_prc_err/100) * v_exp;
		*v_perp_max_ptr 		= (v_perp_prc_err/100) * v_exp;
	}
}

//thrust heading
void thrust_heading(float target_coord_wcs_x, float target_coord_wcs_y, float *thrust_heading_des)
{
	float sat_coord_wcs_x, sat_coord_wcs_y;													//thrust vector heading to target, sat coord, sat heading (in form of "(x)i+(y)j")
	float P_sat_to_origin_x, P_sat_to_origin_y, P_sat_to_target_x, P_sat_to_target_y; 		//position vector from sat to origin, "..." sat to target, etc...
	float P_origin_to_target_x, P_origin_to_target_y, P_unit_sat_to_target_x, P_unit_sat_to_target_y;
	//float P_x_sat_to_target, P_y_sat_to_target;											//x-component of sat to target position vector, y-component "..."
	float i_unit_x = 1;																		//x-axis unit vector, ie "i hat")

	//compute P_sat_to_target
	P_sat_to_origin_x 		= (-1) * sat_coord_wcs_x;										//**HOW TO GET THIS FROM NAV? in "(x)i+(y)j"
	P_sat_to_origin_y 		= (-1) * sat_coord_wcs_y;
	P_origin_to_target_x 	= target_coord_wcs_x;											//target coord provided by user. in "(x)i+(y)j"
	P_origin_to_target_y 	= target_coord_wcs_y;
	P_sat_to_target_x 		= P_sat_to_origin_x + P_origin_to_target_x;
	P_sat_to_target_y 		= P_sat_to_origin_y + P_origin_to_target_y;

	//compute P_unit_sat_to_target
	P_unit_sat_to_target_x 	= (P_sat_to_target_x/sqrt(pow(P_sat_to_target_x, 2) + pow(P_sat_to_target_y, 2)));
	P_unit_sat_to_target_y 	= (P_sat_to_target_y/sqrt(pow(P_sat_to_target_x, 2) + pow(P_sat_to_target_y, 2)));		//this is not used at this time, but may be in the future
	*thrust_heading_des = acos(P_unit_sat_to_target_x * i_unit_x);

//WORK IN PROGRESS!
}

//impulse magnitude
void impulse_magnitude(float a, float mass, float *impulse)
{
	*impulse = (mass*a);
}


int main(){
	printf("Execute_Path begin... \n");
	execute_path();
	return 0;
}

