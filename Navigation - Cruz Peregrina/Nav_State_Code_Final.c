/*
 ============================================================================
 Name        : OcvTest_V5.c
 Author      : SNAPS, Cruz Peregrina
 Version     :
 Copyright   :
 Description :
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#include <time.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <unistd.h>
#include "get_nav_state.h"


// Targets slide bar preset values
int T_range_low = 157;
int T_range_high = 174;
int T_canny_low = 0;
int T_canny_high = 255;
int T_rad_min = 5;
int T_rad_max = 9;
int T_tol_min = 25;
int T_tol_max = 83;

// Directional Marker (DM) slide bar preset values
int DM_range_low = 9;
int DM_range_high = 18;
int DM_canny_low = 255;
int DM_canny_high = 255;
int DM_rad_min = 5;
int DM_rad_max = 9;
int DM_tol_min = 25;
int DM_tol_max = 83;

// Other Variables
int d;                   //Used to find the distance in between targets

// Allocate images
IplImage* img;
IplImage* Cropped;
IplImage* imgHSV;
IplImage* TargetsFilter;
IplImage* DMFilter;
IplImage* TimgCanny;
IplImage* DMimgCanny;
IplImage* TimgDrawn;

//Allocate Memory Storage
CvMemStorage* TcircStorage;
CvMemStorage* DMcircStorage;


CvCapture* capture;

void timestamp_frame(time_t *framestamptime);
//void write_timestamp();
int snaps_open(); //char* arg for still image
void snaps_close();
int snaps_nav(struct snaps_nav_state_struct* snaps_cvnav_state, time_t *framestamptime);


int main( int argc, char** argv ) {

	int rc = 0;
	struct snaps_nav_state_struct snaps_cvnav_state;
	time_t frametime;

	snaps_open(argv[1]);

	while ( !rc ) {
		rc = snaps_nav(&snaps_cvnav_state, &frametime);
		printf("The current time is: %s \n", ctime (&frametime));
		//sleep(1);
		if( cvWaitKey( 15 ) == 27 )
			break;
		} // end of while !rc

	snaps_close();

	return 0;
}

int snaps_open(){ //char* arg was parameter for still frame
	//Load the input Image           /*********old way of input image *****
	//img = cvLoadImage( arg, CV_LOAD_IMAGE_COLOR);


	capture = cvCaptureFromCAM(1);
			   if ( !capture ) {
				 fprintf( stderr, "ERROR: capture is NULL \n" );
				 getchar();
				 return -1;
			   }


	//cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );

	cvNamedWindow("Controls", CV_WINDOW_AUTOSIZE);													/*****Only for troubleshooting values**/

	cvNamedWindow("TargetsFilter", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("DMFilter", CV_WINDOW_AUTOSIZE);

	cvNamedWindow("TCannyImage", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("DMCannyImage", CV_WINDOW_AUTOSIZE);

	cvNamedWindow("TDrawnImage", CV_WINDOW_AUTOSIZE);

	// Create trackbars
	cvCreateTrackbar( "Targets Range Low", "Controls", &T_range_low, 255, NULL );
	cvCreateTrackbar( "Targets Range High", "Controls", &T_range_high, 255, NULL );

	cvCreateTrackbar( "Directional Marker Range Low", "Controls", &DM_range_low, 255, NULL );
	cvCreateTrackbar( "Directional Marker Range High", "Controls", &DM_range_high, 255, NULL );

	//cvCreateTrackbar( "Targets Canny Low", "Controls", &T_canny_low, 255, NULL );
	//cvCreateTrackbar( "Targets Canny High", "Controls", &T_canny_high, 255, NULL );

	//cvCreateTrackbar( "Directional Marker Canny Low", "Controls", &DM_canny_low, 255, NULL );
	//cvCreateTrackbar( "Directional Marker Canny High", "Controls", &DM_canny_high, 255, NULL );

	cvCreateTrackbar( "Targets Radius Min", "Controls", &T_rad_min, 20, NULL );
	cvCreateTrackbar( "Targets Radius Max", "Controls", &T_rad_max, 20, NULL );

	cvCreateTrackbar( "Targets Distance Min", "Controls", &T_tol_min, 50, NULL );
	cvCreateTrackbar( "Targets Distance Max", "Controls", &T_tol_max, 100, NULL );

	cvCreateTrackbar( "Directional Marker Radius Min", "Controls", &DM_rad_min, 20, NULL );
	cvCreateTrackbar( "Directional Marker Radius Max", "Controls", &DM_rad_max, 20, NULL );

	cvCreateTrackbar( "DM Distance Min", "Controls", &DM_tol_min, 50, NULL );
	cvCreateTrackbar( "DM Distance Max", "Controls", &DM_tol_max, 100, NULL );

	// Allocate images
	Cropped       = cvCreateImage(cvSize(540,380),IPL_DEPTH_8U, 3);
	imgHSV        = cvCreateImage(cvSize(540,380), IPL_DEPTH_8U, 3);
	TargetsFilter = cvCreateImage(cvSize(540,380),IPL_DEPTH_8U, 1);
	DMFilter      = cvCreateImage(cvSize(540,380),IPL_DEPTH_8U, 1);
	TimgCanny     = cvCreateImage(cvSize(540,380),IPL_DEPTH_8U, 1);
	DMimgCanny	  = cvCreateImage(cvSize(540,380),IPL_DEPTH_8U, 1);
	TimgDrawn     = cvCreateImage(cvSize(540,380),IPL_DEPTH_8U, 3);

	//Allocate memory for target and DM circle finder
	TcircStorage = cvCreateMemStorage(0);
	DMcircStorage = cvCreateMemStorage(0);
	return 0;
}

void snaps_close(){
	//release memory for target and DM circle finder
	cvReleaseMemStorage(&TcircStorage);
	cvReleaseMemStorage(&DMcircStorage);

	//	cvWaitKey(0);
	cvReleaseImage( &Cropped);
	cvReleaseImage( &imgHSV);
	cvReleaseImage( &TargetsFilter);
	cvReleaseImage( &DMFilter);
	cvReleaseImage( &TimgCanny);
	cvReleaseImage( &DMimgCanny);
	cvReleaseImage( &TimgDrawn);
	cvReleaseImage( &img);
	cvDestroyAllWindows();

	// Release the capture device housekeeping
	//cvReleaseCapture( &capture );
	cvDestroyWindow( "mywindow" );
}

int snaps_nav(struct snaps_nav_state_struct* nav_state, time_t *ptr_framestamptime){

		//Get frame from camera
	 for (int i=0; i<6; i++){
		img = cvQueryFrame( capture );
		if ( !img ) {
		   fprintf( stderr, "ERROR: frame is null...\n" );
		   getchar();
		   break;
		 }
	 }

	timestamp_frame(ptr_framestamptime);

//Crop Image code
	cvSetImageROI(img,cvRect(1,1,540,380));
	cvCopy(img, Cropped, NULL);

//Change the color format from BGR to HSV
	cvCvtColor(Cropped, imgHSV, CV_BGR2HSV);

//copy original img to be displayed in drawn Targets/DM img
	cvCopy(Cropped, TimgDrawn, NULL );

	cvInRangeS(imgHSV, cvScalar(T_range_low,0,0,0), cvScalar(T_range_high,255,255,0), TargetsFilter);
	cvInRangeS(imgHSV, cvScalar(DM_range_low,0,0,0), cvScalar(DM_range_high,255,255,0), DMFilter);

//Magenta Marker Image Processing
	cvErode(TargetsFilter, TargetsFilter, 0, 1);
	cvDilate(TargetsFilter, TargetsFilter, NULL, 1);						//Dilate image
	cvSmooth(TargetsFilter, TargetsFilter, CV_GAUSSIAN, 3, 0, 0.0, 0.0);  	//Smooth Target image*/

//Orange Target Image Processing
	cvErode(DMFilter, DMFilter, 0, 1);
	cvDilate(DMFilter, DMFilter, NULL, 1);									//Dilate image
	//cvSmooth(DMFilter, DMFilter, CV_GAUSSIAN, 3, 0, 0.0, 0.0);  			//Smooth DM image

//Show filtered Images
	cvShowImage("TargetsFilter", TargetsFilter);							//Show Targets filter image
	cvShowImage("DMFilter", DMFilter);   									//Show DM filter image										//Show Noise Filter

//Perform Canny on Images
	cvCanny(TargetsFilter, TimgCanny, T_canny_low, T_canny_high, 3);  			// Apply canny filter to the targets image
	cvCanny(DMFilter, DMimgCanny, DM_canny_low, DM_canny_high, 3); 				// Apply canny filter to the DM image

	cvShowImage("TCannyImage", TimgCanny);
	cvShowImage("DMCannyImage", DMimgCanny);


// Find and Draw circles for the Targets image
	CvPoint Tpt;

	CvSeq* TimgHCirc = cvHoughCircles(
			TimgCanny, TcircStorage, CV_HOUGH_GRADIENT,					// in, out, method,
			2, 															//precision of the accumulator (2x the input image)
			T_rad_max*4, 												//min dist between circles
			T_tol_max, T_tol_min,										//parm1, parm2
			T_rad_min, T_rad_max); 										//min radius, max radius

		for (int i = 0; i < TimgHCirc->total; i++) {
			float* p = (float*) cvGetSeqElem(TimgHCirc, i);

		// To get the circle coordinates
			CvPoint pt = cvPoint(cvRound(p[0]), cvRound(p[1]));

		// Draw center of circles in green
			cvCircle(TimgDrawn, pt, 1, CV_RGB(0,255,0), -1, 8, 0 );
			cvCircle(TimgDrawn, pt, cvRound(p[2]), CV_RGB(255,255,0), 2, 8, 0); 	// img, center, radius, color, thickness, line type, shift

				Tpt = cvPoint(cvRound(p[0]), cvRound(p[1]));
				if (i == 0){
				Tpt = cvPoint(cvRound(p[0]), cvRound(p[1]));

				printf("Magenta Marker (x,y) - (%d, %d) \n", Tpt.x, Tpt.y);
				}
				else {printf("TM - There is an extra point frame not good");
				}
		} // end of for

	// Find and Draw circles for the DM image
	CvPoint DMpt;

	CvSeq* DMimgHCirc = cvHoughCircles(
			DMimgCanny, DMcircStorage, CV_HOUGH_GRADIENT,				// in, out, method,
			2, 															//precision of the accumulator (2x the input image)
			DM_rad_max*4, 												//min dist between circles
			DM_tol_max, DM_tol_min,										//parm1, parm2
			DM_rad_min, DM_rad_max); 									//min radius, max radius

	for (int i=0; i<DMimgHCirc->total; i++) {
		float* p = (float*) cvGetSeqElem(DMimgHCirc, i);
		CvPoint pt = cvPoint(cvRound(p[0]), cvRound(p[1]));

		// Draw center of circles in green
		cvCircle(TimgDrawn, pt, 1, CV_RGB(255,0,0), -1, 8, 0 );
		cvCircle(TimgDrawn, pt, cvRound(p[2]), CV_RGB(255,127,0), 2, 8, 0); 	// img, center, radius, color, thickness, line type, shift

		if (i == 0){
			DMpt = cvPoint(cvRound(p[0]), cvRound(p[1]));
			printf("Red Marker(x,y) - (%d, %d)\n", DMpt.x, DMpt.y);
		}
		else {printf("DM - There is an extra point frame not good");
		}
	} // end of for

	//Draw line in between points
		cvLine(TimgDrawn, Tpt, DMpt, CV_RGB(0,255,0), 1, 8, 0);
		d = sqrt(pow(Tpt.x-DMpt.x, 2)+pow(Tpt.y-DMpt.y, 2));      //distance in between points
		printf("Distance in between tagets %d \n", d);

		//Magenta target coordinates
							int MT_pt_x = Tpt.x;
							int MT_pt_y = Tpt.y;
							//Orange target coordinates
							int OT_pt_x = DMpt.x;
							int OT_pt_y = DMpt.y;

							//Minimum of the two coordinates
							int x_min;
							int x_max;
							int y_min;
							int y_max;

							if (MT_pt_x > OT_pt_x){
								x_min = OT_pt_x;
								x_max = MT_pt_x;
							}
							else{
									x_min = MT_pt_x;
									x_max = OT_pt_x;
							}
																							//printf("x_min %d \n", x_min);
																							//printf("x_max %d \n", x_max);
							if (MT_pt_y > OT_pt_y){
								y_min = OT_pt_y;
								y_max = MT_pt_y;
							}
							else{
									y_min = MT_pt_y;
									y_max = OT_pt_y;
							}
																							//printf("y_min %d", y_min);
																							//printf("y_max %d", y_max);
							//Center of targets point (CT)
							int CT_pt_x = (((x_max - x_min)/2) + x_min);
							int CT_pt_y = (((y_max - y_min)/2) + y_min);
							printf("Center coordinate (x, y) - (%d, %d) \n", CT_pt_x, CT_pt_y);

							//Draw halfway targets point
							CvPoint CT_pt = cvPoint(cvRound(CT_pt_x), cvRound(CT_pt_y));
							cvCircle(img, CT_pt, 2, CV_RGB(255,0,0), -1, 8, 0);

							//Orientation
							int orientation_x = (OT_pt_x - CT_pt_x);
							int orientation_y = (CT_pt_y - OT_pt_y);

							double Theta = (((atan2(orientation_y, orientation_x )) * (180/3.14))+360);
							//if
							printf("Orientation %f Degrees \n", Theta);

	//cvResetImageROI(img);
	cvShowImage("TDrawnImage", TimgDrawn);
	//cvShowImage("DMDrawnImage", DMimgDrawn);

	//clear memory for target and DM circle finder
	//note: this may not be necessary
	cvClearMemStorage(TcircStorage);
	cvClearMemStorage(DMcircStorage);

	return 0;
}
void timestamp_frame(time_t *ptr_rawtime){

	time (ptr_rawtime);
	//printf("The current time is: %s \n", ctime (&rawtime));
}
//
//	code to write timestamp of frame to file
//

/*
void write_timestamp(){
	time_t rawtime;
	time (&rawtime);
	FILE *timestamp_pointer;
	timestamp_pointer=fopen("/opencv_test03/src/TimestampData.txt", "wb");
	//fwrite(	ctime (&rawtime), timestamp_pointer);
	int fclose(FILE *timestamp_pointer);
}
*/
