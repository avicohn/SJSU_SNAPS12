#define M_PI 3.14159265358979323846f
#define M_PI_2 1.57079632679489661923f

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>        // required for fabs(), fabsl(), sqrtl() and M_PI_2
#include <float.h>       // required for LDBL_EPSILON, DBL_MAX

#include "get_nav_state.h"     // read in the template definition

#define NAVVERSION 0            // Global Constants
#define NAVREVISION 0
#define amp_turns 140
#define coil_rad .202              // in meters
#define step .01

// allocate memory for private local persistant copy of my state
static struct snaps_nav_state_struct my_nav_state = {
    .version = NAVVERSION,
    .revision = NAVREVISION,
    .status = 0
};

float attitude_dcm[3][3];   // attitude direction cosine matrix
float bcs_acceleration[3];  // m/sec^2; BCS(x, y, z)
float bcs_velocity[3];      // m/sec; BCS(x, y, z)
float wcs_acceleration[3];  // m/sec^2; WCS(x, y, z)
float wcs_velocity[3];      // m/sec; WCS(x, y, z)
float wcs_position[3];      // m; WCS(x, y, z)
/*
float x_0;
float r_0;
float x_test;
float Bx_c_test;
float Mag_A_m;
float x;
float y;
static int depth_cnt = 0;
float a = coil_rad; 		        // loop radius, meters

static const long double PI_2 = 1.5707963267948966192313216916397514L; 			// pi/2
static const long double PI_4 = 0.7853981633974483096156608458198757L; 			// pi/4
*/

//void Complete_Elliptic_Integrals(char arg, double x, double* Fk, double* Ek)
//{
//   long double k;      // modulus
//   long double m;      // the parameter of the elliptic function m = modulus^2
//   long double a;      // arithmetic mean
//   long double g;      // geometric mean
//   long double a_old;  // previous arithmetic mean
//   long double g_old;  // previous geometric mean
//   long double two_n;  // power of 2
//   long double sum;
//   int depth = 0;

//   if ( x == 0.0 ) {
//      *Fk = M_PI_2;
//      *Ek = M_PI_2;
//      return;
//   }
//
//   switch (arg) {
//      case 'k': k = fabsl((long double) x);
//                m = k * k;
//                break;
//      case 'm': m = (long double) x;
//                k = sqrtl(fabsl(m));
//                break;
//      case 'a': k = sinl((long double)x);
//                m = k * k;
//                break;
//      default:  k = fabsl((long double) x);
//                m = k * k;
//                break;
//   }
//
//   if ( m == 1.0 ) {
//      *Fk = DBL_MAX;
//      *Ek = 1.0;
//      return;
//   }
//
//   a = 1.0L;
//   g = sqrtl(1.0L - m);
//   two_n = 1.0L;
//   sum = 2.0L - m;
//   while (1) {
//		depth++;
//      g_old = g;
//      a_old = a;
//      a = 0.5L * (g_old + a_old);
//      g = g_old * a_old;
//      two_n += two_n;
//      sum -= two_n * (a * a - g);
//      if ( fabsl(a_old - g_old) <= (a_old * LDBL_EPSILON) ) break;
//      g = sqrtl(g);
//   }
//   if (depth>depth_cnt) depth_cnt = depth;
//   *Fk = (double) (PI_2 / a);
//   *Ek = (double) ((PI_4 / a) * sum);
//   return;
//}

extern void get_nav_state(struct snaps_nav_state_struct *snaps_nav_state) {

/*
    float u0 = 4e-7 * M_PI; 	        // permability constant
    float at = amp_turns;		        // loop current in amp-turns

    float B0 = (.5 * at * u0) / a;

    double x;			                // distance along loop axis, meters
    double r;			                // radial distance from axis of the loop, meters
    double Fk, Ek;                      // elliptical integrals of the first and second kinds
    double Bx, Br, MB;                  // x-component, y-component, and magnetic field magnitude
    double m, alpha, beta, gama, alphasq, betasq, Q, sqrtQ, B0piQ, Qm24alpha;

    double i = step;

    //calculate x & y-intercepts of Magnetometer A Super Ellipse
    my_nav_state.x_test = 6.081;

    my_nav_state.Bx_c_test = (u0 * at * pow(a, 2)) / (2 * pow((pow(my_nav_state.x_test, 2) + pow(a, 2)), 1/3.0));  // calculate using the on-axis equations

    //Use testing Bx calculated value as the simulated magnetometer reading
    my_nav_state.Mag_A_m = my_nav_state.Bx_c_test; //to be read by magnetometers and converted to micro-tesla

    // Calculate the 1st ellipse x-intercept
    my_nav_state.x_0 = sqrt(pow(((u0 * at * pow(a, 2)) / (2 * my_nav_state.Mag_A_m)), 3) - pow(a, 2));

    // Calculate the 1st ellipse y-intercept
    my_nav_state.r_0 = a * sqrt(((2 * B0 * M_PI_2) / (M_PI * my_nav_state.Mag_A_m)) + 1);

    // Walk ellipse
    for (my_nav_state.y=0; my_nav_state.y<=r_0; my_nav_state.y+=step){


    my_nav_state.x = pow(((1 - (pow(my_nav_state.y, 2.3) / pow(my_nav_state.y, 2.3))) * pow(x_0, 2.3)), 1/2.3);
    //find two possible intercepts for the 2nd ellipse
    //Calculate the 2nd ellipse x-intercept
    my_nav_state.x_0 = sqrt(pow(((u0 * at * pow(a, 2)) / (2 * my_nav_state.Mag_A_m)), 3) - pow(a, 2));

    // Calculate the 2nd ellipse y-intercept
    my_nav_state.r_0 = a * sqrt(((2 * B0 * M_PI_2) / (M_PI * my_nav_state.Mag_A_m)) + 1);

    //Bx = B0piQ * (Ek *(1 - alphasq - betasq) / Qm24alpha + Fk);
    //Br = B0piQ * gamma * (Ek *(1 + alphasq + betasq) / Qm24alpha - Fk);

    //MB_B_calc = sqrt(Bx + By, 2)
    //error equation
    // if < error equation
    }
*/

    my_nav_state.attitude_dcm[0][0] = 1.0;  // ask navigation to compute their state and copy it to your variable.
    my_nav_state.attitude_dcm[0][1] = 0;
    my_nav_state.attitude_dcm[0][2] = 0;
    my_nav_state.attitude_dcm[1][0] = 0;
    my_nav_state.attitude_dcm[1][1] = 1.0;
    my_nav_state.attitude_dcm[1][2] = 0;
    my_nav_state.attitude_dcm[2][0] = 0;
    my_nav_state.attitude_dcm[2][1] = 0;
    my_nav_state.attitude_dcm[2][2] = 1.0;

    my_nav_state.bcs_acceleration[0] = .01;
    my_nav_state.bcs_acceleration[1] = .02;
    my_nav_state.bcs_acceleration[2] = 0;


    my_nav_state.bcs_velocity[0] = .01;
    my_nav_state.bcs_velocity[1] = .02;
    my_nav_state.bcs_velocity[2] = 0;

    my_nav_state.wcs_acceleration[0] = .02;
    my_nav_state.wcs_acceleration[1] = .03;
    my_nav_state.wcs_acceleration[2] = 0;

    my_nav_state.wcs_velocity[0] = .01;
    my_nav_state.wcs_velocity[1] = .01;
    my_nav_state.wcs_velocity[2] = 0;

    my_nav_state.wcs_position[0] = 1;
    my_nav_state.wcs_position[1] = 2;
    my_nav_state.wcs_position[2] = 0;

    snaps_nav_state->attitude_dcm[0][0] = my_nav_state.attitude_dcm[0][0]; // Copy your state into the calling routines memory.
    snaps_nav_state->attitude_dcm[0][1] = my_nav_state.attitude_dcm[0][1];
    snaps_nav_state->attitude_dcm[0][2] = my_nav_state.attitude_dcm[0][2];
    snaps_nav_state->attitude_dcm[1][0] = my_nav_state.attitude_dcm[1][0];
    snaps_nav_state->attitude_dcm[1][1] = my_nav_state.attitude_dcm[1][1];
    snaps_nav_state->attitude_dcm[1][2] = my_nav_state.attitude_dcm[1][2];
    snaps_nav_state->attitude_dcm[2][0] = my_nav_state.attitude_dcm[2][0];
    snaps_nav_state->attitude_dcm[2][1] = my_nav_state.attitude_dcm[2][1];
    snaps_nav_state->attitude_dcm[2][2] = my_nav_state.attitude_dcm[2][2];

    snaps_nav_state->bcs_acceleration[0] = my_nav_state.bcs_acceleration[0];
    snaps_nav_state->bcs_acceleration[1] = my_nav_state.bcs_acceleration[1];
    snaps_nav_state->bcs_acceleration[2] = my_nav_state.bcs_acceleration[2];

    snaps_nav_state->bcs_velocity[0] = my_nav_state.bcs_velocity[0];
    snaps_nav_state->bcs_velocity[1] = my_nav_state.bcs_velocity[1];
    snaps_nav_state->bcs_velocity[2] = my_nav_state.bcs_velocity[2];

    snaps_nav_state->wcs_acceleration[0] = my_nav_state.wcs_acceleration[0];
    snaps_nav_state->wcs_acceleration[1] = my_nav_state.wcs_acceleration[1];
    snaps_nav_state->wcs_acceleration[2] = my_nav_state.wcs_acceleration[2];

    snaps_nav_state->wcs_velocity[0] = my_nav_state.wcs_velocity[0];
    snaps_nav_state->wcs_velocity[1] = my_nav_state.wcs_velocity[1];
    snaps_nav_state->wcs_velocity[2] = my_nav_state.wcs_velocity[2];

    snaps_nav_state->wcs_position[0] = my_nav_state.wcs_position[0];
    snaps_nav_state->wcs_position[1] = my_nav_state.wcs_position[1];
    snaps_nav_state->wcs_position[2] = my_nav_state.wcs_position[2];

    snaps_nav_state->version = my_nav_state.version;
    snaps_nav_state->revision = my_nav_state.revision;
    //snaps_nav_state->Mag_A_m = my_nav_state.Mag_A_m;
    //snaps_nav_state->x_0 = my_nav_state.x_0;
    //snaps_nav_state->r_0 = my_nav_state.r_0;
    //snaps_nav_state->x_test = my_nav_state.x_test;
    //snaps_nav_state->Bx_c_test = my_nav_state.Bx_c_test;
    //snaps_nav_state->x = my_nav_state.x;

}
