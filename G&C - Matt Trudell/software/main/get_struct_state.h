/*
 * get_struct_state.h
 *
 *  Created on: Nov 14, 2012
 *      Author: NhatNguyen
 */

#ifndef get_struct_state_h
#define get_struct_state_h

/* struct_state structure */
struct snaps_struct_state_struct {
    short version; // Incr when new version is not backward compatible
    short revision; // Incr when fields are added to the end
    float moment_tensor[3][3]; // moment of inertia Ixx, Ixy,Ixz, Iyx, etc..
    float center_of_mass_offset[3]; // meters from body origin
    float mass; // kg
    int status; // 0 = all is well
                // 1 = hull breach
                // 2 = life support failure
    };
// The moment_tensor, center_of_mass_offset, and mass will change
// due to fuel consumption. Therefore, "Structure State" needs data from
// the "Propulsion State"
extern void get_struct_state(struct snaps_struct_state_struct *snaps_struct_state);

#endif
