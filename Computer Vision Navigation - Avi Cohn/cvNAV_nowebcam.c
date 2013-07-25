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
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>
#include "get_nav_state.h"
#include "cvNAV_senddata.h"


void timestamp_frame(time_t *framestamptime);
void snaps_open(char* arg);
void snaps_close();
int snaps_nav(struct snaps_nav_state_struct* snaps_cvnav_state, time_t *framestamptime);

// Targets slide bar preset values
	int T_range_low = 150;
	int T_range_high = 177;
	int T_canny_low = 0;
	int T_canny_high = 255;
	int T_rad_min = 5;
	int T_rad_max = 8;
	int T_tol_min = 25;
	int T_tol_max = 83;

	// Directional Marker (DM) slide bar preset values
	int DM_range_low = 8;
	int DM_range_high = 10;
	int DM_canny_low = 255;
	int DM_canny_high = 255;
	int DM_rad_min = 4;
	int DM_rad_max = 8;
	int DM_tol_min = 25;
	int DM_tol_max = 83;

	// Allocate images
	IplImage* imgHSV;
	IplImage* TargetsFilter;
	IplImage* DMFilter;
	IplImage* TimgCanny;
	IplImage* DMimgCanny;
	IplImage* TimgDrawn;
	//IplImage* DMimgDrawn;

	CvMemStorage* TcircStorage;
	CvMemStorage* DMcircStorage;

	IplImage* img;



int main( int argc, char** argv ) {

	int rc = 0;
	int fd;
	struct snaps_nav_state_struct snaps_cvnav_state;
	time_t frametime;

	snaps_open(argv[1]);
	fd = senddata_open();
	if (fd == -1){
		return fd;
	}
	while ( !rc ) {
		rc = snaps_nav(&snaps_cvnav_state, &frametime);
		printf("The current time is: %s \n", ctime (&frametime));
		printf("calling senddata_action \n");
		senddata_action(fd);
		sleep(1);
		if( cvWaitKey( 15 ) == 27 )
			break;
		} // end of while !rc

	snaps_close();

	return 0;
}

void snaps_open(char* arg){

	//Load the input Image           /*********old way of input image *****
	img = cvLoadImage( arg, CV_LOAD_IMAGE_COLOR);

	/* //open webcam
	CvCapture* capture = cvCaptureFromCAM(1);
			   if ( !capture ) {
				 fprintf( stderr, "ERROR: capture is NULL \n" );
				 getchar();
				 return -1;
			   }
	*/

	//cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );

	cvNamedWindow("Controls", CV_WINDOW_AUTOSIZE);				/*****Only for troubleshooting values**/

	cvNamedWindow("TargetsFilter", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("DMFilter", CV_WINDOW_AUTOSIZE);

	//cvNamedWindow("TCannyImage", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("DMCannyImage", CV_WINDOW_AUTOSIZE);

	cvNamedWindow("TDrawnImage", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("DMDrawnImage", CV_WINDOW_AUTOSIZE);

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
	imgHSV        = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3);
	TargetsFilter = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U, 1);
	DMFilter      = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U, 1);
	TimgCanny     = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U, 1);
	DMimgCanny	  = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U, 1);
	TimgDrawn     = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U, 3);
	//DMimgDrawn   = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U, 3);

	//Allocate memory for target and DM circle finder
	TcircStorage = cvCreateMemStorage(0);
	DMcircStorage = cvCreateMemStorage(0);

}

void snaps_close(){
	//release memory for target and DM circle finder
	cvReleaseMemStorage(&TcircStorage);
	cvReleaseMemStorage(&DMcircStorage);

	//	cvWaitKey(0);
	cvReleaseImage( &imgHSV);
	cvReleaseImage( &TargetsFilter);
	cvReleaseImage( &DMFilter);
	cvReleaseImage( &TimgCanny);
	cvReleaseImage( &DMimgCanny);
	cvReleaseImage( &TimgDrawn);
	//cvReleaseImage( &DMimgDrawn);
	cvReleaseImage( &img);
	cvDestroyAllWindows();

	// Release the capture device housekeeping
	//cvReleaseCapture( &capture );
	cvDestroyWindow( "mywindow" );
}

int snaps_nav(struct snaps_nav_state_struct* nav_state, time_t *ptr_framestamptime){
/*
		//Get fresh frame from camera
	 for (int i=0; i<6; i++){
		img = cvQueryFrame( capture );
		if ( !img ) {
		   fprintf( stderr, "ERROR: frame is null...\n" );
		   getchar();
		   break;
		 }
	 }
*/
	timestamp_frame(ptr_framestamptime);
 //cvShowImage( "mywindow", img );

//Change the color format from BGR to HSV
	cvCvtColor(img, imgHSV, CV_BGR2HSV);

//copy original img to be displayed in drawn Targets/DM img
	cvCopy(img, TimgDrawn, NULL );
	//cvCopy(img, DMimgDrawn, NULL );

	cvInRangeS(imgHSV, cvScalar(T_range_low,0,0,0), cvScalar(T_range_high,255,255,0), TargetsFilter);
	cvInRangeS(imgHSV, cvScalar(DM_range_low,0,0,0), cvScalar(DM_range_high,255,255,0), DMFilter);

//Dilate image Filters
	cvDilate(TargetsFilter, TargetsFilter, NULL, 1);
	cvDilate(DMFilter, DMFilter, NULL, 2);

/*/Smooth Filters
	cvSmooth(TargetsFilter, TargetsFilter, CV_GAUSSIAN, 3, 0, 0.0, 0.0);
	cvSmooth(DMFilter, DMFilter, CV_GAUSSIAN, 3, 0, 0.0, 0.0);*/

//Show Filtered Images
	cvShowImage("TargetsFilter", TargetsFilter);
	cvShowImage("DMFilter", DMFilter);

// Apply Canny filter to the targets/DM image
	cvCanny(TargetsFilter, TimgCanny, T_canny_low, T_canny_high, 3);
	cvCanny(DMFilter, DMimgCanny, DM_canny_low, DM_canny_high, 3);

/*/Show images after Canny filter is applied
	cvShowImage("TCannyImage", TimgCanny);
	cvShowImage("DMCannyImage", DMimgCanny);*/


// Find and Draw circles for the Targets image

	CvSeq* TimgHCirc = cvHoughCircles(
			TimgCanny, TcircStorage, CV_HOUGH_GRADIENT,				// in, out, method,
			2, 														//precision of the accumulator (2x the input image)
			T_rad_max*4, 											//min dist between circles
			T_tol_max, T_tol_min,									//parm1, parm2
			T_rad_min, T_rad_max); 									//min radius, max radius

	CvPoint pt1;
	CvPoint pt2;

	//only do loop if there are 2 and only 2 circles
	if(2 == TimgHCirc->total){
		for (int i = 0; i < TimgHCirc->total; i++) {
			float* p = (float*) cvGetSeqElem(TimgHCirc, i);

		// To get the circle coordinates
			CvPoint pt = cvPoint(cvRound(p[0]), cvRound(p[1]));

		// Draw center of circles in green
			cvCircle(TimgDrawn, pt, 1, CV_RGB(0,255,0), -1, 8, 0 );
			cvCircle(TimgDrawn, pt, cvRound(p[2]), CV_RGB(255,255,0), 2, 8, 0); 	// img, center, radius, color, thickness, line type, shift
		// first iteration of loop we set point one,
			if (i == 0){
				pt1 = cvPoint(cvRound(p[0]), cvRound(p[1]));
			}
			else {
				pt2 = cvPoint(cvRound(p[0]), cvRound(p[1]));
			}

		} // end of for
		printf("Target pt1(x,y) - (%d, %d)\nTarget pt2(x,y) - (%d, %d) \n", pt1.x, pt1.y, pt2.x, pt2.y);
		cvLine(TimgDrawn, pt1, pt2, CV_RGB(0,255,0), 1, 8, 0);
	}//end of if loop
	else {
		printf("Error: Number of targets = %d \n", TimgHCirc->total);
	}


// Find and Draw circles for the DM image

	CvSeq* DMimgHCirc = cvHoughCircles(
			DMimgCanny, DMcircStorage, CV_HOUGH_GRADIENT,				// in, out, method,
			2, 															//precision of the accumulator (2x the input image)
			DM_rad_max*4, 												//min dist between circles
			DM_tol_max, DM_tol_min,										//parm1, parm2
			DM_rad_min, DM_rad_max); 									//min radius, max radius

	CvPoint DMpt;

	for (int i=0; i<DMimgHCirc->total; i++) {
		float* p = (float*) cvGetSeqElem(DMimgHCirc, i);
		CvPoint pt = cvPoint(cvRound(p[0]), cvRound(p[1]));

		// Draw center of circles in green
		//used to be DMimgDrawn, changed 4/5/13
		cvCircle(TimgDrawn, pt, 1, CV_RGB(255,0,0), -1, 8, 0 );
		cvCircle(TimgDrawn, pt, cvRound(p[2]), CV_RGB(255,127,0), 2, 8, 0); 	// img, center, radius, color, thickness, line type, shift

		if (i == 0){
			DMpt = cvPoint(cvRound(p[0]), cvRound(p[1]));
		}
		else {printf("DM - There is an extra point frame not good");
		}

		printf("Directional Marker(x,y) - (%d, %d)\n", DMpt.x, DMpt.y);

	} // end of for

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
