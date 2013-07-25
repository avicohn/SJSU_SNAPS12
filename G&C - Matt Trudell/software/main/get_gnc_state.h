//get_gnc_state.h
//gnc state

#ifndef GET_GNC_STATE_H_           //check if state has already been defined
#define GET_GNC_STATE_H_

/* gnc state structure */
    struct snaps_gnc_state_struct{
        short version; 		                //Incr when new version is not backward compatible
        short revision; 		            //Incr when fields are added to the end
        int control_mode; 		            // 0 = program mode
                                            // 1 = interactive mode
        char program_name[32]; 		        // file name of the pgm that�s running
        int program_name_length; 		    // number of characters in above name
        int program_line_being_executed; 	// record number in file
        int program_elapsed_time; 		    // seconds; updated after each instruction
        int status; 				        // 0 = waiting for new instruction
                                            // 1 = executing instructions
                                            // 2 = execution abort in progress
                                            // 3 = execution aborted
        struct snaps_pwr_state_struct *pwr_state_ptr;
};

extern void get_gnc_state(struct snaps_gnc_state_struct *snaps_gnc_state);  //extern=visible to others, void=no paramaters are created

#endif
