/**@file
   Functions and structures for dealing with image features.

   Copyright (C) 2006-2007  Rob Hess <hess@eecs.oregonstate.edu>

   @version 1.1.1-20070913
*/

#ifndef IMGFEATURES_H
#define IMGFEATURES_H

#ifdef _WIN32
#include <cxcore.h>
#else
#include <opencv/cxcore.h>
#endif

#include <iostream>
#include <sstream>


/** FEATURE_FWD_MATCH <BR> FEATURE_BCK_MATCH <BR> FEATURE_MDL_MATCH */
enum feature_match_type
  {
    FEATURE_FWD_MATCH,
    FEATURE_BCK_MATCH,
    FEATURE_MDL_MATCH,
  };


/* colors in which to display different feature types */
#define FEATURE_OXFD_COLOR CV_RGB(255,255,0)
#define FEATURE_LOWE_COLOR CV_RGB(255,0,255)

/** max feature descriptor length */
#define FEATURE_MAX_D 128


/**
   Structure to represent an affine invariant image feature.  The fields
   x, y, a, b, c represent the affine region around the feature:

   a(x-u)(x-u) + 2b(x-u)(y-v) + c(y-v)(y-v) = 1
*/
struct feature
{
  float x;                      /**< x coord */
  float y;                      /**< y coord */
  float scl;                    /**< scale of a Lowe-style feature */
  float ori;                    /**< orientation of a Lowe-style feature */
  int d;                         /**< descriptor length */
  float descr[FEATURE_MAX_D];   /**< descriptor */
  int category;                  /**< all-purpose feature category */
  CvPoint2D32f img_pt;           /**< location in image */
  CvPoint2D32f mdl_pt;           /**< location in model */
  void* feature_data;            /**< user-definable data */
};


/**
   Reads image features from file.  The file should be formatted either as
   from the code provided by the Visual Geometry Group at Oxford or from
   the code provided by David Lowe.


   @param filename location of a file containing image features
   @param type determines how features are input.  If \a type is FEATURE_OXFD,
     the input file is treated as if it is from the code provided by the VGG
     at Oxford: http://www.robots.ox.ac.uk:5000/~vgg/research/affine/index.html
     <BR><BR>
     If \a type is FEATURE_LOWE, the input file is treated as if it is from
     David Lowe's SIFT code: http://www.cs.ubc.ca/~lowe/keypoints
   @param feat pointer to an array in which to store imported features

   @return Returns the number of features imported from filename or -1 on error
*/
extern int import_features( char* filename, int type, struct feature** feat );


/**
   Exports a feature set to a file formatted depending on the type of
   features, as specified in the feature struct's type field.

   @param filename name of file to which to export features
   @param feat feature array
   @param n number of features

   @return Returns 0 on success or 1 on error
*/
extern int export_features( char* filename, struct feature* feat, int n );


/**
   Displays a set of features on an image

   @param img image on which to display features
   @param feat array of Oxford-type features
   @param n number of features
*/
extern void draw_features( IplImage* img, struct feature* feat, int n );


/**
   Calculates the squared Euclidian distance between two feature descriptors.

   @param f1 first feature
   @param f2 second feature

   @return Returns the squared Euclidian distance between the descriptors of
     \a f1 and \a f2.
*/
inline float descr_dist_sq(const struct feature* f1, const struct feature* f2, float max=FLT_MAX){
    float result = 0.0f;
    for(unsigned i = 0; i < FEATURE_MAX_D; i++){
        float d = f1->descr[i] - f2->descr[i];
        result += d*d;
    }
    return result;
}


extern int write_feature(std::ostream &output_stream, const struct feature &f);
extern int read_feature(std::istream &input_stream, struct feature &f);

extern int write_feature_string(std::stringstream &output_stream, const struct feature &f);
extern int read_feature_string(std::stringstream &input_stream, struct feature &f);

extern int write_feature_buffer(char *buffer, const struct feature &f);
extern int read_feature_buffer(char *buffer, struct feature &f);
extern size_t sizeof_feature(void);

extern bool are_equal(const struct feature &f1, const struct feature &f2);


#endif
