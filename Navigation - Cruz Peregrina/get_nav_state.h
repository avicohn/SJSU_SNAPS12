#ifndef get_nav_state_h
#define get_nav_state_h

/* Navigation state structure */
struct snaps_nav_state_struct {
    short version;              // Incr when new version is not backward compatible
    short revision;             // Incr when fields are added to the end
    float attitude_dcm[3][3];   // attitude direction cosine matrix
    float bcs_acceleration[3];  // m/sec^2; BCS(x, y, z)
    float bcs_velocity[3];      // m/sec; BCS(x, y, z)
    float wcs_acceleration[3];  // m/sec^2; WCS(x, y, z)
    float wcs_velocity[3];      // m/sec; WCS(x, y, z)
    float wcs_position[3];      // m; WCS(x, y, z)
    int status;                 // 0 = all is well
                                // 1 = mag field is weak, poor confidence
                                // 2 = mag field not sufficient
};
extern void get_nav_state(struct snaps_nav_state_struct *snaps_nav_state);

#endif
