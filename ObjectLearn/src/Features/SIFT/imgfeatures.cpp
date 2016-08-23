/*
  Functions and structures for dealing with image features

  Copyright (C) 2006-2007  Rob Hess <hess@eecs.oregonstate.edu>

  @version 1.1.1-20070913
*/

#include "utils.h"
#include "imgfeatures.h"

#include <opencv/cxcore.h>
#include <iostream>


int import_lowe_features( char*, struct feature** );
int export_lowe_features( char*, struct feature*, int );
void draw_lowe_features( IplImage*, struct feature*, int );
void draw_lowe_feature( IplImage*, struct feature*, CvScalar );


/*
  Reads image features from file.  The file should be formatted as from
  the code provided by the Visual Geometry Group at Oxford:


  @param filename location of a file containing image features
  @param type determines how features are input.  If \a type is FEATURE_OXFD,
    the input file is treated as if it is from the code provided by the VGG
    at Oxford:

    http://www.robots.ox.ac.uk:5000/~vgg/research/affine/index.html

    If \a type is FEATURE_LOWE, the input file is treated as if it is from
    David Lowe's SIFT code:

    http://www.cs.ubc.ca/~lowe/keypoints
  @param features pointer to an array in which to store features

  @return Returns the number of features imported from filename or -1 on error
*/
int import_features( char* filename, int type, struct feature** feat )
{
  int n = import_lowe_features( filename, feat );

  if( n == -1 )
    fprintf( stderr, "Warning: unable to import features from %s,"	\
	     " %s, line %d\n", filename, __FILE__, __LINE__ );
  return n;
}



/*
  Exports a feature set to a file formatted depending on the type of
  features, as specified in the feature struct's type field.

  @param filename name of file to which to export features
  @param feat feature array
  @param n number of features

  @return Returns 0 on success or 1 on error
*/
int export_features( char* filename, struct feature* feat, int n )
{
  if( n <= 0  ||  ! feat ){
      fprintf( stderr, "Warning: no features to export, %s line %d\n",
	       __FILE__, __LINE__ );
      return 1;
  }


  int r = export_lowe_features( filename, feat, n );

  if( r ){
    fprintf( stderr, "Warning: unable to export features to %s,"	\
	     " %s, line %d\n", filename, __FILE__, __LINE__ );
  }

  return r;
}


/*
  Draws a set of features on an image

  @param img image on which to draw features
  @param feat array of Oxford-type features
  @param n number of features
*/
void draw_features( IplImage* img, struct feature* feat, int n )
{
  if( n <= 0  ||  ! feat )
    {
      fprintf( stderr, "Warning: no features to draw, %s line %d\n",
	       __FILE__, __LINE__ );
      return;
    }

    draw_lowe_features( img, feat, n );
}

// Currently assumes its a LOWE type feature
int write_feature(std::ostream &output_stream, const struct feature &f){
    output_stream.write((char*)&(f.x), sizeof(float));
    output_stream.write((char*)&(f.y), sizeof(float));
    output_stream.write((char*)&(f.scl), sizeof(float));
    output_stream.write((char*)&(f.ori), sizeof(float));
    output_stream.write((char*)&(f.d), sizeof(int));

    int i;
    for(i = 0; i < f.d; i++){
        output_stream.write((char*)&(f.descr[i]), sizeof(float));
    }
    return 0;
}

int read_feature(std::istream &input_stream, struct feature &f){
    input_stream.read((char*)&(f.x), sizeof(float));
    input_stream.read((char*)&(f.y), sizeof(float));
    input_stream.read((char*)&(f.scl), sizeof(float));
    input_stream.read((char*)&(f.ori), sizeof(float));
    input_stream.read((char*)&(f.d), sizeof(int));

    f.img_pt.x = f.x;
    f.img_pt.y = f.y;

    if(f.d > FEATURE_MAX_D){
        std::cerr << "Error, incorrect feature dimension: " << f.d << std::endl;
        return 1;
    }

    for(int i = 0; i < f.d; i++){
        input_stream.read((char*)&(f.descr[i]), sizeof(float));
    }

    f.category = 0;
    f.mdl_pt.x = f.mdl_pt.y = -1;
    f.feature_data = NULL;

    return 0;
}


int write_feature_string(std::stringstream &output_stream, const struct feature &f){
    output_stream << f.x << " " << f.y << " " << f.scl << " " << f.ori << " " << f.d;

    for(int i = 0; i < f.d; i++){
        output_stream << " " << f.descr[i];
    }
    output_stream << std::endl;

    return 0;
}

int read_feature_string(std::stringstream &input_stream, struct feature &f){
    input_stream >> f.x;
    input_stream >> f.y;
    input_stream >> f.scl;
    input_stream >> f.ori;
    input_stream >> f.d;

    f.img_pt.x = f.x;
    f.img_pt.y = f.y;

    if(f.d > FEATURE_MAX_D){
        std::cerr << "Error, incorrect feature dimension: " << f.d << std::endl;
        return 1;
    }

    for(int i = 0; i < f.d; i++){
        input_stream >> f.descr[i];
    }

    f.category = 0;
    f.mdl_pt.x = f.mdl_pt.y = -1;

    return 0;
}

int write_feature_buffer(char* buffer, const struct feature &f){
    *(float*)(buffer) = f.x;     buffer += sizeof(float);
    *(float*)(buffer) = f.y;     buffer += sizeof(float);
    *(float*)(buffer) = f.scl;   buffer += sizeof(float);
    *(float*)(buffer) = f.ori;   buffer += sizeof(float);
    *(int*)(buffer) = f.d;       buffer += sizeof(int);

    for(int i = 0; i < f.d; i++){
        *(float*)(buffer) = f.descr[i];   buffer += sizeof(float);
    }
    return 0;
}

int read_feature_buffer(char* buffer, struct feature &f){
    f.x = *(float*)(buffer);     buffer += sizeof(float);
    f.y = *(float*)(buffer);     buffer += sizeof(float);
    f.scl = *(float*)(buffer);   buffer += sizeof(float);
    f.ori = *(float*)(buffer);   buffer += sizeof(float);
    f.d = *(int*)(buffer);       buffer += sizeof(int);

    f.img_pt.x = f.x;
    f.img_pt.y = f.y;

    if(f.d > FEATURE_MAX_D){
        std::cerr << "Error, incorrect feature dimension: " << f.d << std::endl;
        return 1;
    }

    for(int i = 0; i < f.d; i++){
        f.descr[i] = *(float*)(buffer);   buffer += sizeof(float);
    }

    f.category = 0;
    f.mdl_pt.x = f.mdl_pt.y = -1;

    return 0;
}

size_t sizeof_feature(void){
    return 4*sizeof(float) + sizeof(int) + FEATURE_MAX_D*sizeof(float);
}

/***************************** Local Functions *******************************/

/*
  Reads image features from file.  The file should be formatted as from
  the code provided by David Lowe:

  http://www.cs.ubc.ca/~lowe/keypoints/

  @param filename location of a file containing image features
  @param features pointer to an array in which to store features

  @return Returns the number of features imported from filename or -1 on error
*/
int import_lowe_features( char* filename, struct feature** features )
{
  struct feature* f;
  int i, j, n, d;
  float x, y, s, o, dv;
  FILE* file;

  if( ! features )
    assert(false);
  if( ! ( file = fopen( filename, "r" ) ) )
    {
      fprintf( stderr, "Warning: error opening %s, %s, line %d\n",
	       filename, __FILE__, __LINE__ );
      return -1;
    }

  /* read number of features and dimension */
  if( fscanf( file, " %d %d ", &n, &d ) != 2 )
    {
      fprintf( stderr, "Warning: file read error, %s, line %d\n",
	       __FILE__, __LINE__ );
      return -1;
    }
  if( d > FEATURE_MAX_D )
    {
      fprintf( stderr, "Warning: descriptor too long, %s, line %d\n",
	       __FILE__, __LINE__ );
      return -1;
    }

  f = (feature *)calloc( n, sizeof(struct feature) );
  for( i = 0; i < n; i++ )
    {
      /* read affine region parameters */
      if( fscanf( file, " %f %f %f %f ", &y, &x, &s, &o ) != 4 )
	{
	  fprintf( stderr, "Warning: error reading feature #%d, %s, line %d\n",
		   i+1, __FILE__, __LINE__ );
	  free( f );
	  return -1;
	}
      f[i].img_pt.x = f[i].x = x;
      f[i].img_pt.y = f[i].y = y;
      f[i].scl = s;
      f[i].ori = o;
      f[i].d = d;

      /* read descriptor */
      for( j = 0; j < d; j++ )
	{
	  if( ! fscanf( file, " %f ", &dv ) )
	    {
	      fprintf( stderr, "Warning: error reading feature descriptor" \
		       " #%d, %s, line %d\n", i+1, __FILE__, __LINE__ );
	      free( f );
	      return -1;
	    }
	  f[i].descr[j] = dv;
	}

      f[i].category = 0;
      f[i].mdl_pt.x = f[i].mdl_pt.y = -1;
      f[i].feature_data = NULL;
    }

  if( fclose(file) )
    {
      fprintf( stderr, "Warning: file close error, %s, line %d\n",
	       __FILE__, __LINE__ );
      free( f );
      return -1;
    }

  *features = f;
  return n;
}



/*
  Exports a feature set to a file formatted as one from the code provided
  by David Lowe:

  http://www.cs.ubc.ca/~lowe/keypoints/

  @param filename name of file to which to export features
  @param feat feature array
  @param n number of features

  @return Returns 0 on success or 1 on error
*/
int export_lowe_features( char* filename, struct feature* feat, int n )
{
  FILE* file;
  int i, j, d;

  if( n <= 0 )
    {
      fprintf( stderr, "Warning: feature count %d, %s, line %d\n",
	       n, __FILE__, __LINE__ );
      return 1;
    }
  if( ! ( file = fopen( filename, "w" ) ) )
    {
      fprintf( stderr, "Warning: error opening %s, %s, line %d\n",
	       filename, __FILE__, __LINE__ );
      return 1;
    }

  d = feat[0].d;
  fprintf( file, "%d %d\n", n, d );
  for( i = 0; i < n; i++ )
    {
      fprintf( file, "%f %f %f %f", feat[i].y, feat[i].x,
	       feat[i].scl, feat[i].ori );
      for( j = 0; j < d; j++ )
	{
	  /* write 20 descriptor values per line */
	  if( j % 20 == 0 )
	    fprintf( file, "\n" );
	  fprintf( file, " %d", (int)(feat[i].descr[j]) );
	}
      fprintf( file, "\n" );
    }

  if( fclose(file) )
    {
      fprintf( stderr, "Warning: file close error, %s, line %d\n",
	       __FILE__, __LINE__ );
      return 1;
    }
  return 0;
}


/*
  Draws Lowe-type features

  @param img image on which to draw features
  @param feat array of Oxford-type features
  @param n number of features
*/
void draw_lowe_features( IplImage* img, struct feature* feat, int n )
{
  CvScalar color = CV_RGB( 255, 255, 255 );
  int i;

  if( img-> nChannels > 1 )
    color = FEATURE_LOWE_COLOR;
  for( i = 0; i < n; i++ )
    draw_lowe_feature( img, feat + i, color );
}



/*
  Draws a single Lowe-type feature

  @param img image on which to draw
  @param feat feature to be drawn
  @param color color in which to draw
*/
void draw_lowe_feature( IplImage* img, struct feature* feat, CvScalar color )
{
  int len, hlen, blen, start_x, start_y, end_x, end_y, h1_x, h1_y, h2_x, h2_y;
  float scl, ori;
  float scale = 5.0f;
  float hscale = 0.75f;
  CvPoint start, end, h1, h2;

  /* compute points for an arrow scaled and rotated by feat's scl and ori */
  start_x = cvRound( feat->x );
  start_y = cvRound( feat->y );
  scl = feat->scl;
  ori = feat->ori;
  len = cvRound( scl * scale );
  hlen = cvRound( scl * hscale );
  blen = len - hlen;
  end_x = cvRound( len *  cos( ori ) ) + start_x;
  end_y = cvRound( len * -sin( ori ) ) + start_y;
  h1_x = cvRound( blen *  cos( ori + CV_PI / 18.0f ) ) + start_x;
  h1_y = cvRound( blen * -sin( ori + CV_PI / 18.0f ) ) + start_y;
  h2_x = cvRound( blen *  cos( ori - CV_PI / 18.0 ) ) + start_x;
  h2_y = cvRound( blen * -sin( ori - CV_PI / 18.0 ) ) + start_y;
  start = cvPoint( start_x, start_y );
  end = cvPoint( end_x, end_y );
  h1 = cvPoint( h1_x, h1_y );
  h2 = cvPoint( h2_x, h2_y );

  //cvCircle(img, start, 1, color, 2);

  cvLine( img, start, end, color, 1, 8, 0 );
  cvLine( img, end, h1, color, 1, 8, 0 );
  cvLine( img, end, h2, color, 1, 8, 0 );

    //cvCircle(img, start, 1, CV_RGB(0, 0, 255), 2);
}

#define EPSILON 0.001f

bool are_equal(const struct feature &f1, const struct feature &f2){
    bool maybe_equal = fabs(f1.x-f2.x) < EPSILON &&
                       fabs(f1.y-f2.y) < EPSILON &&
                       fabs(f1.scl-f2.scl) < EPSILON &&
                       fabs(f1.ori-f2.ori) < EPSILON &&
                       fabs(f1.x-f2.x) < EPSILON &&
                       f1.d == f2.d;

    if(maybe_equal){
        for(int i = 0; i < f1.d; i++){
            if(fabs(f1.descr[i]-f2.descr[i]) > EPSILON){ return false; }
        }

        return true;
    }
    else{
        return false;
    }
}

