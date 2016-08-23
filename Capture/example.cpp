//=============================================================================
// Copyright © 2000,2001 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research Inc.
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//
// Digiclops® is a registered trademark of Point Grey Research Inc.
//=============================================================================

//=============================================================================
//
// simplegrab.cpp
//
// This program gives a simple example for opening the Digiclops, grabbing
// different kinds of images, and closing the Digiclops again.
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <sys/timeb.h>
#include <time.h>

//=============================================================================
// PGR Includes
//=============================================================================
#include <digiclops.h>


#define _HANDLE_DIGICLOPS_ERROR( functionname, error ) \
   if ( error != DIGICLOPS_OK ) \
   { \
      char  pszMessage[ 512 ]; \
      sprintf( pszMessage, "%s() failed: \"%s\"\n", \
         functionname, \
         ::digiclopsErrorToString( error ) ); \
   \
      printf( pszMessage ); \
      assert( false ); \
      return -1; \
   } \
\



int
main( int argc, char** argv )
{
   DigiclopsContext  context;
   DigiclopsError    error;
   TriclopsInput     triclopsInput;
   double            dPercentRate = 0.0;

   switch( argc )
   {
      default :
         fprintf( stderr,
                  "Usage : %s [%% of bus bandwidth]\n",
                  argv[0] );
         return 1;
      case 2 :
         dPercentRate = atof( argv[1] );
         if( ( dPercentRate <= 0.0 ) || ( dPercentRate > 100.0 ) )
         {
            fprintf( stderr,
                     "%s : %% of bus bandwidth must be >0.0 and <=100.0 \n",
                     argv[0] );
            return 1;
         }
         fprintf( stderr,
                  "%s : Trying %lf %% of bus bandwidth\n",
                  argv[0],
                  dPercentRate );
         break;
      case 1 :
         break;
   }

   error  = digiclopsCreateContext( &context );
   _HANDLE_DIGICLOPS_ERROR( "digiclopsCreateContext", error );

   // initialize device 0 (the first device on the bus)
   error  = digiclopsInitialize( context, 0 );
   _HANDLE_DIGICLOPS_ERROR( "digiclopsInitialize", error );

   // start the Digiclops device streaming images 
   error = digiclopsStart( context );
   _HANDLE_DIGICLOPS_ERROR( "digiclopsStart", error );

   // tell the Digiclops to provide all image types
   error  = digiclopsSetImageTypes( context, ALL_IMAGES );
   _HANDLE_DIGICLOPS_ERROR( "digiclopsSetImageTypes", error );

   error = digiclopsSetImageResolution( context, DIGICLOPS_FULL );
   _HANDLE_DIGICLOPS_ERROR( "digiclopsSetImageResolution", error );

   // set the frame rate if asked
   DigiclopsFrameRate frameRate;
   if( dPercentRate > 0.0 )
   {
      if( dPercentRate <= 12.0 )
      {
         frameRate = DIGICLOPS_FRAMERATE_012;
      }
      else if( dPercentRate <= 25.0 )
      {
         frameRate = DIGICLOPS_FRAMERATE_025;
      }
      else if( dPercentRate <= 50.0 )
      {
         frameRate = DIGICLOPS_FRAMERATE_050;
      }
      else
      {
         frameRate = DIGICLOPS_FRAMERATE_100;
      }

      error = digiclopsSetFrameRate( context, frameRate );
      _HANDLE_DIGICLOPS_ERROR( "digiclopsSetFrameRate", error );
   }

   // retrieve the current frame rate
   error = digiclopsGetFrameRate( context, &frameRate );
   _HANDLE_DIGICLOPS_ERROR( "digiclopsGetFrameRate", error );

   switch( frameRate )
   {
      case DIGICLOPS_FRAMERATE_100 :
         dPercentRate = 100;
         break;
      case DIGICLOPS_FRAMERATE_050 :
         dPercentRate = 50;
         break;
      case DIGICLOPS_FRAMERATE_025 :
         dPercentRate = 25;
         break;
      default :  // case DIGICLOPS_FRAMERATE_012 :
         dPercentRate = 12;
         break;
   }
   printf( "Current bus rate : %lf %%\n", dPercentRate );

   // perform a simple profiling
   // grab images for approximately 10 seconds and determine the frame rate

   int  elapsedMillis;
#ifdef WIN32
   struct _timeb start, end;
   _ftime( &start );
#else
   struct timeb start, end;
   ftime( &start );
#endif

   int   nGrabs = 0;
   printf( "Beginning profiling...\n" );

   for( ;; )
   {
      printf( "Grabbing an image...\n" );

      error = digiclopsGrabImage( context );
      _HANDLE_DIGICLOPS_ERROR( "digiclopsGrabImage", error );
   
      error = digiclopsExtractTriclopsInput( 
         context, STEREO_IMAGE, &triclopsInput );
      _HANDLE_DIGICLOPS_ERROR( "digiclopsExtractTriclopsInput", error );

      nGrabs++;

#ifdef WIN32
      _ftime( &end );
#else
      ftime( &end );
#endif

      elapsedMillis = 
         end.time * 1000 + end.millitm - start.time * 1000 - start.millitm;
      if ( elapsedMillis > 10 * 1000 )
      {
         break;
      }

//#define PRINT_TIMESTAMP
#ifdef PRINT_TIMESTAMP
      fprintf( 
         stderr, 
         "(%d) elapsedMSec = %d, Time = %ld sec %ld usec\n",
         nGrabs,
         elapsedMillis,
         triclopsInput.timeStamp.sec,
         triclopsInput.timeStamp.u_sec );
#endif
   }

   printf( "%d frames grabbed in %d milliseconds\n", nGrabs, elapsedMillis );
   printf( 
      " => %f frames per second\n", 
      (float)nGrabs * 1000.0 / (float)elapsedMillis );

   printf( 
      "Rows: %d, Cols: %d, Row Increment: %d\n", 
      triclopsInput.nrows, 
      triclopsInput.ncols, 
      triclopsInput.rowinc );
   
   // okay, lets save some of the images to disk
   digiclopsWritePPM( context, TOP_IMAGE,    "top.ppm" );
   digiclopsWritePPM( context, RIGHT_IMAGE,  "right.ppm" );
   digiclopsWritePPM( context, LEFT_IMAGE,   "left.ppm" );
   digiclopsWritePPM( context, STEREO_IMAGE, "stereo.ppm" );

   // stop the Digiclops device streaming images 
   error  = digiclopsStop( context );
   _HANDLE_DIGICLOPS_ERROR( "digiclopsStop", error );

   error  = digiclopsDestroyContext( context );
   _HANDLE_DIGICLOPS_ERROR( "digiclopsDestroyContext", error );

   return 0;
}
