/*
  Miscellaneous utility functions.
  
  Copyright (C) 2006-2007  Rob Hess <hess@eecs.oregonstate.edu>

  @version 1.1.1-20070913
*/

#include "utils.h"

#ifdef _WIN32
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#else
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#endif

//#include <gdk/gdk.h>
//#include <gtk.h>

#include <errno.h>
#include <string.h>
#include <stdlib.h>


/*************************** Function Definitions ****************************/


/*
  Prints an error message and aborts the program.  The error message is
  of the form "Error: ...", where the ... is specified by the \a format
  argument
  
  @param format an error message format string (as with \c printf(3)).
*/
void fatal_error(char* format, ...)
{
  /*va_list ap;
  
  fprintf( stderr, "Error: ");

  va_start( ap, format );
  vfprintf( stderr, format, ap );
  va_end( ap );
  fprintf( stderr, "\n" );*/
  abort();
}

/*
  Doubles the size of an array with error checking
  
  @param array pointer to an array whose size is to be doubled
  @param n number of elements allocated for \a array
  @param size size in bytes of elements in \a array
  
  @return Returns the new number of elements allocated for \a array.  If no
    memory is available, returns 0.
*/
int array_double( void** array, int n, int size )
{
  void* tmp;

  tmp = realloc( *array, 2 * n * size );
  if( ! tmp )
    {
      fprintf( stderr, "Warning: unable to allocate memory in array_double(),"
	       " %s line %d\n", __FILE__, __LINE__ );
      if( *array )
	free( *array );
      *array = NULL;
      return 0;
    }
  *array = tmp;
  return n*2;
}



/*
  Calculates the squared distance between two points.

  @param p1 a point
  @param p2 another point
*/
float dist_sq_2D( CvPoint2D32f p1, CvPoint2D32f p2 )
{
  float x_diff = p1.x - p2.x;
  float y_diff = p1.y - p2.y;

  return x_diff * x_diff + y_diff * y_diff;
}



/*
  Draws an x on an image.
  
  @param img an image
  @param pt the center point of the x
  @param r the x's radius
  @param w the x's line weight
  @param color the color of the x
*/
void draw_x( IplImage* img, CvPoint pt, int r, int w, CvScalar color )
{
  cvLine( img, pt, cvPoint( pt.x + r, pt.y + r), color, w, 8, 0 );
  cvLine( img, pt, cvPoint( pt.x - r, pt.y + r), color, w, 8, 0 );
  cvLine( img, pt, cvPoint( pt.x + r, pt.y - r), color, w, 8, 0 );
  cvLine( img, pt, cvPoint( pt.x - r, pt.y - r), color, w, 8, 0 );
}



/*
  Combines two images by scacking one on top of the other
  
  @param img1 top image
  @param img2 bottom image
  
  @return Returns the image resulting from stacking \a img1 on top if \a img2
*/
extern IplImage* stack_imgs( IplImage* img1, IplImage* img2 )
{
  IplImage* stacked = cvCreateImage( cvSize( MAX(img1->width, img2->width),
					     img1->height + img2->height ),
				     IPL_DEPTH_8U, 3 );

  cvZero( stacked );
  cvSetImageROI( stacked, cvRect( 0, 0, img1->width, img1->height ) );
  cvAdd( img1, stacked, stacked, NULL );
  cvSetImageROI( stacked, cvRect(0, img1->height, img2->width, img2->height) );
  cvAdd( img2, stacked, stacked, NULL );
  cvResetImageROI( stacked );

  return stacked;
}

/*
  Checks if a HighGUI window is still open or not
  
  @param name the name of the window we're checking
  
  @return Returns 1 if the window named \a name has been closed or 0 otherwise
*/
int win_closed( char* win_name )
{
  if( ! cvGetWindowHandle(win_name) )
    return 1;
  return 0;
}
