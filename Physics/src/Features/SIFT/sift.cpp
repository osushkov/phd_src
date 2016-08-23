/*
  Functions for detecting SIFT image features.

  For more information, refer to:

  Lowe, D.  Distinctive image features from scale-invariant keypoints.
  <EM>International Journal of Computer Vision, 60</EM>, 2 (2004),
  pp.91--110.

  Copyright (C) 2006-2007  Rob Hess <hess@eecs.oregonstate.edu>

  Note: The SIFT algorithm is patented in the United States and cannot be
  used in commercial products without a license from the University of
  British Columbia.  For more information, refer to the file LICENSE.ubc
  that accompanied this distribution.

  @version 1.1.1-20070913
*/

#include "sift.h"
#include "imgfeatures.h"
#include "utils.h"
#include "../../Util/ParallelServer.h"
#include "../../Util/Semaphore.h"
#include "../../Util/Common.h"

#include <iostream>

#ifdef _WIN32
#include <cxcore.h>
#include <cv.h>
#else
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#endif

using namespace SIFT;

struct detection_data {
  int r;
  int c;
  int octv;
  int intvl;
  float subintvl;
  float scl_octv;
};

#define feat_detection_data(f) ( (struct detection_data*)(f->feature_data) )

ParallelServer parallel_server(PSM_PARALLEL);


/************************* Local Function Prototypes *************************/

IplImage* createInitImg( const IplImage*, int, float);
IplImage*** buildGaussPyr( const IplImage*, int, int, float);
IplImage* downsample( const IplImage* );
IplImage*** buildDOGPyr( const IplImage***, int, int );
CvSeq* scaleSpaceExtrema( const IplImage***, int, int, float, int, CvMemStorage*);
int isExtremum( const IplImage***, int, int, int, int );
struct feature* interpExtremum( const IplImage***, int, int, int, int, int, float);
void interpStep( const IplImage***, int, int, int, int, float*, float*, float* );
CvMat* deriv3D( const IplImage***, int, int, int, int );
CvMat* hessian3D( const IplImage***, int, int, int, int );
float interpContr( const IplImage***, int, int, int, int, float, float, float );
struct feature* newFeature( void );
int isTooEdgeLike( const IplImage*, int, int, int );
void calcFeatureScales( CvSeq*, float, int );
void adjustForImgDbl( CvSeq* );
void calcFeatureOris( CvSeq*, const IplImage*** );
float* oriHist( const IplImage*, int, int, int, int, float );
int calcGradMagOri( const IplImage*, int, int, float*, float* );
void smoothOriHist( float*, int );
float dominantOri( float*, int );
void addGoodOriFeatures( CvSeq*, float*, int, float, struct feature* );
struct feature* cloneFeature( struct feature* );
void computeDescriptors( CvSeq*, const IplImage***, int, int );
float*** descrHist( const IplImage*, int, int, float, float, int, int );
void interpHistEntry( float***, float, float, float, float, int, int);
void histToDescr( const float***, int, int, struct feature* );
void normalizeDescr( struct feature* );
int featureCmp( const void*, const void*, const void* );
void releaseDescrHist( float****, int );
void releasePyr( IplImage****, int, int );
void smoothImage(const IplImage *src_img, IplImage* dst_img, float sd);

float secs = 0.0f;
int n = 0;
/*********************** Functions prototyped in sift.h **********************/


/**
   Finds SIFT features in an image using default parameter values.  All
   detected features are stored in the array pointed to by \a feat.

   @param img the image in which to detect features
   @param feat a pointer to an array in which to store detected features

   @return Returns the number of features stored in \a feat or -1 on failure
   @see _sift_features()
*/
int SIFT::sift_features( const IplImage* img, struct feature** feat )
{
  return SIFT::_sift_features( img, feat, SIFT_INTVLS, SIFT_SIGMA, SIFT_CONTR_THR,
			 SIFT_CURV_THR, SIFT_IMG_DBL, SIFT_DESCR_WIDTH,
			 SIFT_DESCR_HIST_BINS );
}



/**
   Finds SIFT features in an image using user-specified parameter values.  All
   detected features are stored in the array pointed to by \a feat.

   @param img the image in which to detect features
   @param fea a pointer to an array in which to store detected features
   @param intvls the number of intervals sampled per octave of scale space
   @param sigma the amount of Gaussian smoothing applied to each image level
     before building the scale space representation for an octave
   @param cont_thr a threshold on the value of the scale space function
     \f$\left|D(\hat{x})\right|\f$, where \f$\hat{x}\f$ is a vector specifying
     feature location and scale, used to reject unstable features;  assumes
     pixel values in the range [0, 1]
   @param curv_thr threshold on a feature's ratio of principle curvatures
     used to reject features that are too edge-like
   @param img_dbl should be 1 if image doubling prior to scale space
     construction is desired or 0 if not
   @param descr_width the width, \f$n\f$, of the \f$n \times n\f$ array of
     orientation histograms used to compute a feature's descriptor
   @param descrHist_bins the number of orientations in each of the
     histograms in the array used to compute a feature's descriptor

   @return Returns the number of keypoints stored in \a feat or -1 on failure
   @see sift_keypoints()
*/
int SIFT::_sift_features(const IplImage* img, struct feature** feat, int intvls,
		                 float sigma, float contr_thr, int curv_thr,
		                 int img_dbl, int descr_width, int descrHist_bins){

    n++;

    IplImage* init_img;
    IplImage*** gauss_pyr, *** dog_pyr;
    CvMemStorage* storage;
    CvSeq* features;
    int octvs, i, n = 0;

    // check arguments
    if(!img){ assert(false); }
    if(!feat ){ assert(false); }

    // build scale space pyramid; smallest dimension of top level is ~4 pixels
    init_img = createInitImg(img, img_dbl, sigma);
    octvs = (int)(log((double)MIN(init_img->width, init_img->height))/log(2.0) - 2);

    gauss_pyr = buildGaussPyr(init_img, octvs, intvls, sigma);
    dog_pyr = buildDOGPyr((const IplImage***)gauss_pyr, octvs, intvls);
    storage = cvCreateMemStorage(0);
    features = scaleSpaceExtrema((const IplImage***)dog_pyr, octvs, intvls, contr_thr, curv_thr, storage);

    calcFeatureScales(features, sigma, intvls);
    if(img_dbl){
        adjustForImgDbl(features);
    }

    calcFeatureOris(features, (const IplImage***)gauss_pyr);
    computeDescriptors(features, (const IplImage ***)gauss_pyr, descr_width, descrHist_bins);

    // sort features by decreasing scale and move from CvSeq to array
    cvSeqSort(features, (CvCmpFunc)featureCmp, NULL);
    n = features->total;
    *feat = (feature *)calloc(n, sizeof(struct feature));
    *feat = (feature *)cvCvtSeqToArray(features, *feat, CV_WHOLE_SEQ);

    for(i = 0; i < n; i++){
        free((*feat)[i].feature_data);
        (*feat)[i].feature_data = NULL;
    }

    cvReleaseMemStorage(&storage);
    cvReleaseImage(&init_img);
    releasePyr(&gauss_pyr, octvs, intvls + 3);
    releasePyr(&dog_pyr, octvs, intvls + 2);


    //printf("secs: %f\n", 1000.0f*secs);
    return n;
}


/************************ Functions prototyped here **************************/

/*
  Converts an image to 8-bit grayscale and Gaussian-smooths it.  The image is
  optionally doubled in size prior to smoothing.

  @param img input image
  @param img_dbl if true, image is doubled in size prior to smoothing
  @param sigma total std of Gaussian smoothing
*/
IplImage* createInitImg(const IplImage* img, int img_dbl, float sigma){
    IplImage* gray, *dbl;
    float sig_diff;

    gray = Common::convertToGray32( img );
    if(img_dbl){
        sig_diff = sqrtf( sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA * 4 );
        dbl = cvCreateImage(cvSize(img->width*2, img->height*2), IPL_DEPTH_32F, 1);
        cvResize( gray, dbl, CV_INTER_CUBIC );
        cvSmooth( dbl, dbl, CV_GAUSSIAN, 0, 0, sig_diff, sig_diff );
        cvReleaseImage( &gray );
        return dbl;
    }
    else {
        sig_diff = sqrtf(sigma * sigma - SIFT_INIT_SIGMA * SIFT_INIT_SIGMA);
        cvSmooth(gray, gray, CV_GAUSSIAN, 0, 0, sig_diff, sig_diff);
        return gray;
    }
}

/*
  Builds Gaussian scale space pyramid from an image

  @param base base image of the pyramid
  @param octvs number of octaves of scale space
  @param intvls number of intervals per octave
  @param sigma amount of Gaussian smoothing per octave

  @return Returns a Gaussian scale space pyramid as an octvs x (intvls + 3)
    array
*/
IplImage*** buildGaussPyr(const IplImage* base, int octvs,
			              int intvls, float sigma) {

    IplImage*** gauss_pyr;
    const int _intvls = intvls;
    const int s = _intvls+3;
    float sig_total, sig_prev, k;
    float *sig = (float*)malloc(s*sizeof(float));

    gauss_pyr = (IplImage***)calloc(octvs, sizeof(IplImage**));
    for(int i = 0; i < octvs; i++){
        gauss_pyr[i] = (IplImage**)calloc(intvls + 3, sizeof(IplImage *));
    }

    /*
      precompute Gaussian sigmas using the following formula:
      \sigma_{total}^2 = \sigma_{i}^2 + \sigma_{i-1}^2
    */
    sig[0] = sigma;
    k = powf(2.0f, 1.0f/intvls);
    for(int i = 1; i < intvls + 3; i++){
        sig_prev = powf(k, (float)(i - 1))*sigma;
        sig_total = sig_prev*k;
        sig[i] = sqrtf(sig_total*sig_total - sig_prev*sig_prev);
    }

    for(int o = 0; o < octvs; o++){
        for(int i = 0; i < intvls+3; i++){
	        if(o == 0 && i == 0){
	            gauss_pyr[o][i] = cvCloneImage(base);
            }
	        // base of new octvave is halved image from end of previous octave
	        else if(i == 0){
	            gauss_pyr[o][i] = downsample(gauss_pyr[o-1][intvls]); //FIXME should this be invtls+2?
	        }
	        // blur the current octave's last image to create the next one
	        else{
	            gauss_pyr[o][i] = cvCreateImage(cvGetSize(gauss_pyr[o][i-1]),
			                                    IPL_DEPTH_32F, 1);


                //smoothImage(gauss_pyr[o][i-1], gauss_pyr[o][i], sig[i]);
	            cvSmooth(gauss_pyr[o][i-1], gauss_pyr[o][i], CV_GAUSSIAN, 0, 0, sig[i], sig[i]);
	        }
        }
    }

    delete[] sig;
    return gauss_pyr;
}



/*
  Downsamples an image to a quarter of its size (half in each dimension)
  using nearest-neighbor interpolation

  @param img an image

  @return Returns an image whose dimensions are half those of img
*/
IplImage* downsample(const IplImage* img){
    IplImage* smaller = cvCreateImage(cvSize(img->width/2, img->height/2),
                                      img->depth, img->nChannels);
    cvResize(img, smaller, CV_INTER_NN);
    return smaller;
}



/*
  Builds a difference of Gaussians scale space pyramid by subtracting adjacent
  intervals of a Gaussian pyramid

  @param gauss_pyr Gaussian scale-space pyramid
  @param octvs number of octaves of scale space
  @param intvls number of intervals per octave

  @return Returns a difference of Gaussians scale space pyramid as an
    octvs x (intvls + 2) array
*/
IplImage*** buildDOGPyr(const IplImage*** gauss_pyr, int octvs, int intvls){
    IplImage*** dog_pyr;
    int i, o;

    dog_pyr = (IplImage***)calloc(octvs, sizeof( IplImage** ));
    for(i = 0; i < octvs; i++){
        dog_pyr[i] = (IplImage**)calloc(intvls + 2, sizeof(IplImage*));
    }

    for(o = 0; o < octvs; o++){
        for(i = 0; i < intvls + 2; i++){
	        dog_pyr[o][i] = cvCreateImage(cvGetSize(gauss_pyr[o][i]),
				                          IPL_DEPTH_32F, 1);
	        cvSub(gauss_pyr[o][i+1], gauss_pyr[o][i], dog_pyr[o][i], NULL);
        }
    }

    return dog_pyr;
}



/*
  Detects features at extrema in DoG scale space.  Bad features are discarded
  based on contrast and ratio of principal curvatures.

  @param dog_pyr DoG scale space pyramid
  @param octvs octaves of scale space represented by dog_pyr
  @param intvls intervals per octave
  @param contr_thr low threshold on feature contrast
  @param curv_thr high threshold on feature ratio of principal curvatures
  @param storage memory storage in which to store detected features

  @return Returns an array of detected features whose scales, orientations,
    and descriptors are yet to be determined.
*/
CvSeq* scaleSpaceExtrema(const IplImage*** dog_pyr, int octvs, int intvls,
                         float contr_thr, int curv_thr,
                         CvMemStorage* storage) {

    CvSeq* features;
    float prelim_contr_thr = 0.5f * contr_thr / intvls;
    struct feature* feat;
    struct detection_data* ddata;
    int o, i, r, c;

    features = cvCreateSeq(0, sizeof(CvSeq), sizeof(struct feature), storage);
    for(o = 0; o < octvs; o++){
        for(i = 1; i <= intvls; i++){
            for(r = SIFT_IMG_BORDER; r < dog_pyr[o][0]->height-SIFT_IMG_BORDER; r++){
	            for(c = SIFT_IMG_BORDER; c < dog_pyr[o][0]->width-SIFT_IMG_BORDER; c++){
	                // perform preliminary check on contrast
	                if(ABS(pixval32f(dog_pyr[o][i], r, c)) > prelim_contr_thr){
	                    if(isExtremum(dog_pyr, o, i, r, c)){
		                    feat = interpExtremum(dog_pyr, o, i, r, c, intvls, contr_thr);
		                    if(feat){
		                        ddata = feat_detection_data(feat);
		                        if(!isTooEdgeLike(dog_pyr[ddata->octv][ddata->intvl],
					                              ddata->r, ddata->c, curv_thr)){
			                        cvSeqPush( features, feat );
		                        }
		                        else{
		                            free(ddata);
                                }
		                        free(feat);
		                    }
	                    }
                    }
                }
            }
        }
    }

    return features;
}



/*
  Determines whether a pixel is a scale-space extremum by comparing it to it's
  3x3x3 pixel neighborhood.

  @param dog_pyr DoG scale space pyramid
  @param octv pixel's scale space octave
  @param intvl pixel's within-octave interval
  @param r pixel's image row
  @param c pixel's image col

  @return Returns 1 if the specified pixel is an extremum (max or min) among
    it's 3x3x3 pixel neighborhood.
*/
int isExtremum(const IplImage*** dog_pyr, int octv, int intvl, int r, int c){
    float val = pixval32f(dog_pyr[octv][intvl], r, c);
    int i, j, k;

    // check for maximum
    if(val > 0){
        for(i = -1; i <= 1; i++){
            for(j = -1; j <= 1; j++){
                for(k = -1; k <= 1; k++){
                    if(val < pixval32f(dog_pyr[octv][intvl+i], r + j, c + k)){
                        return 0;
                    }
                }
            }
        }
    }

    // check for minimum
    else{
        for(i = -1; i <= 1; i++){
            for(j = -1; j <= 1; j++){
                for(k = -1; k <= 1; k++){
                    if(val > pixval32f(dog_pyr[octv][intvl+i], r + j, c + k)){
                        return 0;
                    }
                }
            }
        }
    }

    return 1;
}



/*
  Interpolates a scale-space extremum's location and scale to subpixel
  accuracy to form an image feature.  Rejects features with low contrast.
  Based on Section 4 of Lowe's paper.

  @param dog_pyr DoG scale space pyramid
  @param octv feature's octave of scale space
  @param intvl feature's within-octave interval
  @param r feature's image row
  @param c feature's image column
  @param intvls total intervals per octave
  @param contr_thr threshold on feature contrast

  @return Returns the feature resulting from interpolation of the given
    parameters or NULL if the given location could not be interpolated or
    if contrast at the interpolated loation was too low.  If a feature is
    returned, its scale, orientation, and descriptor are yet to be determined.
*/
struct feature* interpExtremum(const IplImage*** dog_pyr, int octv, int intvl,
                               int r, int c, int intvls, float contr_thr) {

    struct feature* feat;
    struct detection_data* ddata;
    float xi, xr, xc, contr;
    int i = 0;

    while(i < SIFT_MAX_INTERP_STEPS){
        interpStep(dog_pyr, octv, intvl, r, c, &xi, &xr, &xc);
        if(ABS(xi) < 0.5f && ABS(xr) < 0.5f && ABS(xc) < 0.5f){
            break;
        }

        c += cvRound(xc);
        r += cvRound(xr);
        intvl += cvRound(xi);

        if(intvl < 1
        || intvl > intvls
        || c < SIFT_IMG_BORDER
        || r < SIFT_IMG_BORDER
        || c >= dog_pyr[octv][0]->width - SIFT_IMG_BORDER
        || r >= dog_pyr[octv][0]->height - SIFT_IMG_BORDER){
	        return NULL;
	    }

        i++;
    }

    // ensure convergence of interpolation
    if(i >= SIFT_MAX_INTERP_STEPS){
      return NULL;
    }

    contr = interpContr(dog_pyr, octv, intvl, r, c, xi, xr, xc);
    if(ABS(contr) < contr_thr/intvls){
      return NULL;
    }

    feat = newFeature();
    ddata = feat_detection_data( feat );
    feat->img_pt.x = feat->x = (c + xc) * powf(2.0f, (float)octv);
    feat->img_pt.y = feat->y = (r + xr) * powf(2.0f, (float)octv);
    ddata->r = r;
    ddata->c = c;
    ddata->octv = octv;
    ddata->intvl = intvl;
    ddata->subintvl = xi;

    return feat;
}



/*
  Performs one step of extremum interpolation.  Based on Eqn. (3) in Lowe's
  paper.

  @param dog_pyr difference of Gaussians scale space pyramid
  @param octv octave of scale space
  @param intvl interval being interpolated
  @param r row being interpolated
  @param c column being interpolated
  @param xi output as interpolated subpixel increment to interval
  @param xr output as interpolated subpixel increment to row
  @param xc output as interpolated subpixel increment to col
*/

void interpStep(const IplImage*** dog_pyr, int octv, int intvl, int r, int c,
                float* xi, float* xr, float* xc){

    CvMat* dD, *H, *H_inv, X;
    float x[3] = {0.0f};

    dD = deriv3D(dog_pyr, octv, intvl, r, c);
    H = hessian3D(dog_pyr, octv, intvl, r, c);
    H_inv = cvCreateMat(3, 3, CV_32FC1);
    cvInvert(H, H_inv, CV_SVD);
    cvInitMatHeader(&X, 3, 1, CV_32FC1, x, CV_AUTOSTEP);
    cvGEMM(H_inv, dD, -1, NULL, 0, &X, 0);

    cvReleaseMat(&dD);
    cvReleaseMat(&H);
    cvReleaseMat(&H_inv);

    *xi = x[2];
    *xr = x[1];
    *xc = x[0];
}



/*
  Computes the partial derivatives in x, y, and scale of a pixel in the DoG
  scale space pyramid.

  @param dog_pyr DoG scale space pyramid
  @param octv pixel's octave in dog_pyr
  @param intvl pixel's interval in octv
  @param r pixel's image row
  @param c pixel's image col

  @return Returns the vector of partial derivatives for pixel I
    { dI/dx, dI/dy, dI/ds }^T as a CvMat*
*/
CvMat* deriv3D(const IplImage*** dog_pyr, int octv, int intvl, int r, int c){
    CvMat* dI;
    float dx, dy, ds;

    dx = (pixval32f(dog_pyr[octv][intvl], r, c+1) -
          pixval32f(dog_pyr[octv][intvl], r, c-1))/2.0f;
    dy = (pixval32f(dog_pyr[octv][intvl], r+1, c) -
          pixval32f(dog_pyr[octv][intvl], r-1, c))/2.0f;
    ds = (pixval32f(dog_pyr[octv][intvl+1], r, c) -
          pixval32f(dog_pyr[octv][intvl-1], r, c))/2.0f;

    dI = cvCreateMat(3, 1, CV_32FC1);
    cvmSet(dI, 0, 0, dx);
    cvmSet(dI, 1, 0, dy);
    cvmSet(dI, 2, 0, ds);

    return dI;
}



/*
  Computes the 3D Hessian matrix for a pixel in the DoG scale space pyramid.

  @param dog_pyr DoG scale space pyramid
  @param octv pixel's octave in dog_pyr
  @param intvl pixel's interval in octv
  @param r pixel's image row
  @param c pixel's image col

  @return Returns the Hessian matrix (below) for pixel I as a CvMat*

  / Ixx  Ixy  Ixs \ <BR>
  | Ixy  Iyy  Iys | <BR>
  \ Ixs  Iys  Iss /
*/
CvMat* hessian3D(const IplImage*** dog_pyr, int octv, int intvl, int r, int c){
    CvMat* H;
    float v, dxx, dyy, dss, dxy, dxs, dys;

    v = pixval32f( dog_pyr[octv][intvl], r, c );

    dxx = (pixval32f(dog_pyr[octv][intvl], r, c+1) +
           pixval32f(dog_pyr[octv][intvl], r, c-1) - 2*v);

    dyy = (pixval32f(dog_pyr[octv][intvl], r+1, c) +
           pixval32f(dog_pyr[octv][intvl], r-1, c) - 2*v);

    dss = (pixval32f(dog_pyr[octv][intvl+1], r, c) +
           pixval32f(dog_pyr[octv][intvl-1], r, c) - 2*v);

    dxy = (pixval32f(dog_pyr[octv][intvl], r+1, c+1) -
           pixval32f(dog_pyr[octv][intvl], r+1, c-1) -
           pixval32f(dog_pyr[octv][intvl], r-1, c+1) +
           pixval32f(dog_pyr[octv][intvl], r-1, c-1))/4.0f;

    dxs = (pixval32f(dog_pyr[octv][intvl+1], r, c+1) -
           pixval32f(dog_pyr[octv][intvl+1], r, c-1) -
           pixval32f(dog_pyr[octv][intvl-1], r, c+1) +
           pixval32f(dog_pyr[octv][intvl-1], r, c-1))/4.0f;

    dys = (pixval32f(dog_pyr[octv][intvl+1], r+1, c) -
           pixval32f(dog_pyr[octv][intvl+1], r-1, c) -
           pixval32f(dog_pyr[octv][intvl-1], r+1, c) +
           pixval32f(dog_pyr[octv][intvl-1], r-1, c))/4.0f;

    H = cvCreateMat(3, 3, CV_32FC1);
    cvmSet(H, 0, 0, dxx);
    cvmSet(H, 0, 1, dxy);
    cvmSet(H, 0, 2, dxs);
    cvmSet(H, 1, 0, dxy);
    cvmSet(H, 1, 1, dyy);
    cvmSet(H, 1, 2, dys);
    cvmSet(H, 2, 0, dxs);
    cvmSet(H, 2, 1, dys);
    cvmSet(H, 2, 2, dss);

    return H;
}



/*
  Calculates interpolated pixel contrast.  Based on Eqn. (3) in Lowe's
  paper.

  @param dog_pyr difference of Gaussians scale space pyramid
  @param octv octave of scale space
  @param intvl within-octave interval
  @param r pixel row
  @param c pixel column
  @param xi interpolated subpixel increment to interval
  @param xr interpolated subpixel increment to row
  @param xc interpolated subpixel increment to col

  @param Returns interpolated contrast.
*/
float interpContr(const IplImage*** dog_pyr, int octv, int intvl, int r,
	              int c, float xi, float xr, float xc){
    CvMat* dD, X, T;
    float t[1], x[3] = { xc, xr, xi };

    cvInitMatHeader(&X, 3, 1, CV_32FC1, x, CV_AUTOSTEP);
    cvInitMatHeader(&T, 1, 1, CV_32FC1, t, CV_AUTOSTEP);
    dD = deriv3D(dog_pyr, octv, intvl, r, c);
    cvGEMM(dD, &X, 1, NULL, 0, &T,  CV_GEMM_A_T);
    cvReleaseMat(&dD);

    return pixval32f(dog_pyr[octv][intvl], r, c) + t[0] * 0.5f;
}



/*
  Allocates and initializes a new feature

  @return Returns a pointer to the new feature
*/
struct feature* newFeature(void){
    struct feature* feat;
    struct detection_data* ddata;

    feat = (feature *)malloc(sizeof(struct feature));
    memset(feat, 0, sizeof(struct feature));
    ddata = (detection_data *)malloc(sizeof(struct detection_data));
    memset(ddata, 0, sizeof(struct detection_data));
    feat->feature_data = ddata;

    return feat;
}



/*
  Determines whether a feature is too edge like to be stable by computing the
  ratio of principal curvatures at that feature.  Based on Section 4.1 of
  Lowe's paper.

  @param dog_img image from the DoG pyramid in which feature was detected
  @param r feature row
  @param c feature col
  @param curv_thr high threshold on ratio of principal curvatures

  @return Returns 0 if the feature at (r,c) in dog_img is sufficiently
    corner-like or 1 otherwise.
*/
int isTooEdgeLike(const IplImage* dog_img, int r, int c, int curv_thr){
    float d, dxx, dyy, dxy, tr, det;

    /* principal curvatures are computed using the trace and det of Hessian */
    d = pixval32f(dog_img, r, c);
    dxx = pixval32f(dog_img, r, c+1) + pixval32f(dog_img, r, c-1) - 2*d;
    dyy = pixval32f(dog_img, r+1, c) + pixval32f(dog_img, r-1, c) - 2*d;
    dxy = (pixval32f(dog_img, r+1, c+1) - pixval32f(dog_img, r+1, c-1) -
           pixval32f(dog_img, r-1, c+1) + pixval32f(dog_img, r-1, c-1))/4.0f;
    tr = dxx + dyy;
    det = dxx*dyy - dxy*dxy;

    /* negative determinant -> curvatures have different signs; reject feature */
    if(det <= 0.0f){
        return 1;
    }

    if(tr*tr/det < (curv_thr + 1.0f)*(curv_thr + 1.0f)/curv_thr){
        return 0;
    }

    return 1;
}



/*
  Calculates characteristic scale for each feature in an array.

  @param features array of features
  @param sigma amount of Gaussian smoothing per octave of scale space
  @param intvls intervals per octave of scale space
*/
void calcFeatureScales(CvSeq* features, float sigma, int intvls){
    struct feature* feat;
    struct detection_data* ddata;
    float intvl;
    int i, n;

    n = features->total;
    for(i = 0; i < n; i++){
        feat = CV_GET_SEQ_ELEM(struct feature, features, i);
        ddata = feat_detection_data(feat);
        intvl = ddata->intvl + ddata->subintvl;
        feat->scl = sigma * powf(2.0f, ddata->octv + intvl/intvls);
        ddata->scl_octv = sigma * powf(2.0f, intvl/intvls);
    }
}



/*
  Halves feature coordinates and scale in case the input image was doubled
  prior to scale space construction.

  @param features array of features
*/
void adjustForImgDbl(CvSeq* features){
    struct feature* feat;
    int i, n;

    n = features->total;
    for(i = 0; i < n; i++){
        feat = CV_GET_SEQ_ELEM(struct feature, features, i);
        feat->x /= 2.0f;
        feat->y /= 2.0f;
        feat->scl /= 2.0f;
        feat->img_pt.x /= 2.0f;
        feat->img_pt.y /= 2.0f;
    }
}

struct CalcFeatureOrisWorkerData {
    CalcFeatureOrisWorkerData(CvSeq* _features, std::vector<feature *> &_all_features,
                              const IplImage*** _gauss_pyr, Util::Semaphore &_features_sem):
        features(_features), all_features(_all_features), gauss_pyr(_gauss_pyr),
        features_sem(_features_sem) {}

    CvSeq* features;
    std::vector<feature*> &all_features;
    const IplImage*** gauss_pyr;
    Util::Semaphore &features_sem;
};

class CalcFeatureOrisWorker : public ParallelExecutor {
  public:
    CalcFeatureOrisWorker(){}
    ~CalcFeatureOrisWorker(){}

    void performTask(void *task_data, unsigned rank, unsigned size){
        CalcFeatureOrisWorkerData *data = (CalcFeatureOrisWorkerData *)task_data;
        CvSeq* features = data->features;
        const IplImage*** gauss_pyr = data->gauss_pyr;

        struct detection_data* ddata;
        float* hist;
        float omax;

        const unsigned start = (rank*data->all_features.size())/size;
        const unsigned end = ((rank+1)*data->all_features.size())/size;

        for(unsigned i = start; i < end; i++){
            struct feature *feat = data->all_features[i];

            ddata = feat_detection_data(feat);
            hist = oriHist(gauss_pyr[ddata->octv][ddata->intvl],
                           ddata->r, ddata->c, SIFT_ORI_HIST_BINS,
                           cvRound(SIFT_ORI_RADIUS * ddata->scl_octv),
                           SIFT_ORI_SIG_FCTR * ddata->scl_octv);

            for(int j = 0; j < SIFT_ORI_SMOOTH_PASSES; j++){
                smoothOriHist(hist, SIFT_ORI_HIST_BINS);
            }

            omax = dominantOri(hist, SIFT_ORI_HIST_BINS);

            data->features_sem.wait();
            addGoodOriFeatures(features, hist, SIFT_ORI_HIST_BINS,
			                   omax*SIFT_ORI_PEAK_RATIO, feat);
            data->features_sem.signal();

            free(ddata);
            free(feat);
            free(hist);
        }
    }
};




/*
  Computes a canonical orientation for each image feature in an array.  Based
  on Section 5 of Lowe's paper.  This function adds features to the array when
  there is more than one dominant orientation at a given feature location.

  @param features an array of image features
  @param gauss_pyr Gaussian scale space pyramid
*/
void calcFeatureOris(CvSeq* features, const IplImage*** gauss_pyr){
    struct feature* feat;
    const int n = features->total;
    Util::Semaphore task_sem, features_sem(1);

    CalcFeatureOrisWorker executor;
    std::vector<struct feature*> all_features;

    for(int i = 0; i < n; i++ ){
        feat = (feature *)malloc(sizeof(struct feature));
        cvSeqPopFront(features, feat);
        all_features.push_back(feat);
    }


    CalcFeatureOrisWorkerData *data =
        new CalcFeatureOrisWorkerData(features, all_features, gauss_pyr, features_sem);

    ParallelTask task(&executor, data, &task_sem);
    parallel_server.executeParallelTask(task);

    task_sem.wait();
    delete data;
}



/*
  Computes a gradient orientation histogram at a specified pixel.

  @param img image
  @param r pixel row
  @param c pixel col
  @param n number of histogram bins
  @param rad radius of region over which histogram is computed
  @param sigma std for Gaussian weighting of histogram entries

  @return Returns an n-element array containing an orientation histogram
    representing orientations between 0 and 2 PI.
*/
float* oriHist(const IplImage* img, int r, int c, int n, int rad, float sigma){
    float* hist;
    float mag, ori, w, exp_denom, PI2 = (float)CV_PI * 2.0f;
    int bin, i, j;

    hist = (float *)calloc(n, sizeof(float));
    exp_denom = 2.0f * sigma * sigma;

    for(i = -rad; i <= rad; i++){
        for(j = -rad; j <= rad; j++){
            if(calcGradMagOri(img, r + i, c + j, &mag, &ori)){
                w = expf(-(i*i + j*j)/exp_denom);
                bin = cvRound(n*(ori + CV_PI)/PI2 );
                bin = (bin < n) ? bin : 0;
                hist[bin] += w*mag;
	        }
        }
    }

    return hist;
}



/*
  Calculates the gradient magnitude and orientation at a given pixel.

  @param img image
  @param r pixel row
  @param c pixel col
  @param mag output as gradient magnitude at pixel (r,c)
  @param ori output as gradient orientation at pixel (r,c)

  @return Returns 1 if the specified pixel is a valid one and sets mag and
    ori accordingly; otherwise returns 0
*/
int calcGradMagOri(const IplImage* img, int r, int c, float* mag, float* ori){
    float dx, dy;

    if(r > 0 && r < img->height-1 && c > 0 && c < img->width-1){
        dx = pixval32f(img, r, c+1) - pixval32f(img, r, c-1);
        dy = pixval32f(img, r-1, c) - pixval32f(img, r+1, c);
        *mag = sqrtf(dx*dx + dy*dy);
        *ori = atan2f(dy, dx);
        return 1;
    }
    else{
        return 0;
    }
}



/*
  Gaussian smooths an orientation histogram.

  @param hist an orientation histogram
  @param n number of bins
*/
void smoothOriHist(float* hist, int n){
    float prev, tmp, h0 = hist[0];
    int i;

    prev = hist[n-1];
    for(i = 0; i < n; i++){
        tmp = hist[i];
        hist[i] = 0.25f*prev + 0.5f*hist[i] +
                  0.25f*((i+1 == n) ? h0 : hist[i+1]);
        prev = tmp;
    }
}



/*
  Finds the magnitude of the dominant orientation in a histogram

  @param hist an orientation histogram
  @param n number of bins

  @return Returns the value of the largest bin in hist
*/
float dominantOri(float* hist, int n){
    float omax;
    int maxbin, i;

    omax = hist[0];
    maxbin = 0;

    for(i = 1; i < n; i++){
        if(hist[i] > omax){
            omax = hist[i];
            maxbin = i;
        }
    }

    return omax;
}



/*
  Interpolates a histogram peak from left, center, and right values
*/
#define interp_hist_peak( l, c, r ) ( 0.5f * ((l)-(r)) / ((l) - 2.0f*(c) + (r)) )



/*
  Adds features to an array for every orientation in a histogram greater than
  a specified threshold.

  @param features new features are added to the end of this array
  @param hist orientation histogram
  @param n number of bins in hist
  @param mag_thr new features are added for entries in hist greater than this
  @param feat new features are clones of this with different orientations
*/
void addGoodOriFeatures(CvSeq* features, float* hist, int n,
			            float mag_thr, struct feature* feat){

    struct feature* new_feat;
    float bin, PI2 = (float)CV_PI * 2.0f;
    int l, r, i;

    for( i = 0; i < n; i++ ){
        l = ( i == 0 )? n - 1 : i-1;
        r = ( i + 1 ) % n;

        if(hist[i] > hist[l]  &&  hist[i] > hist[r]  &&  hist[i] >= mag_thr){

            bin = i + interp_hist_peak(hist[l], hist[i], hist[r]);
            bin = ( bin < 0 )? n + bin : ( bin >= n )? bin - n : bin;
            new_feat = cloneFeature(feat);
            new_feat->ori = (PI2*bin)/n - (float)CV_PI;
            cvSeqPush(features, new_feat);
            free(new_feat);

        }
    }
}



/*
  Makes a deep copy of a feature

  @param feat feature to be cloned

  @return Returns a deep copy of feat
*/
struct feature* cloneFeature(struct feature* feat){
    struct feature* new_feat;
    struct detection_data* ddata;

    new_feat = newFeature();
    ddata = feat_detection_data(new_feat);
    memcpy(new_feat, feat, sizeof(struct feature));
    memcpy(ddata, feat_detection_data(feat), sizeof(struct detection_data));
    new_feat->feature_data = ddata;

    return new_feat;
}



/*
  Computes feature descriptors for features in an array.  Based on Section 6
  of Lowe's paper.

  @param features array of features
  @param gauss_pyr Gaussian scale space pyramid
  @param d width of 2D array of orientation histograms
  @param n number of bins per orientation histogram
*/

struct ComputeFeatureDescriptorsWorkerData {
    ComputeFeatureDescriptorsWorkerData(std::vector<feature*> &_all_features, const IplImage*** _gauss_pyr,
                                        int _d, int _n):
        all_features(_all_features), gauss_pyr(_gauss_pyr), d(_d), n(_n){}


    std::vector<feature*> &all_features;
    const IplImage*** gauss_pyr;
    int d, n;
};

class ComputeFeatureDescriptorsWorker : public ParallelExecutor {
  public:
    ComputeFeatureDescriptorsWorker(){}
    ~ComputeFeatureDescriptorsWorker(){}

    void performTask(void *task_data, unsigned rank, unsigned size){
        ComputeFeatureDescriptorsWorkerData *data = (ComputeFeatureDescriptorsWorkerData *)task_data;

        std::vector<feature*> &all_features(data->all_features);
        const IplImage*** gauss_pyr = data->gauss_pyr;
        const int d = data->d;
        const int n = data->n;

        struct detection_data* ddata;
        float*** hist;

        const unsigned start = (rank*all_features.size())/size;
        const unsigned end = ((rank+1)*all_features.size())/size;

        for(unsigned i = start; i < end; i++){
            struct feature *feat = all_features[i];

            ddata = feat_detection_data(feat);
            hist = descrHist(gauss_pyr[ddata->octv][ddata->intvl], ddata->r,
                    	     ddata->c, feat->ori, ddata->scl_octv, d, n);
            histToDescr((const float***)hist, d, n, feat);
            releaseDescrHist(&hist, d);
        }
    }
};


void computeDescriptors(CvSeq* features, const IplImage*** gauss_pyr, int d, int n){
    const int k = features->total;

    Util::Semaphore task_sem;
    ComputeFeatureDescriptorsWorker executor;

    std::vector<feature*> all_features;

    for(int i = 0; i < k; i++){
        all_features.push_back(CV_GET_SEQ_ELEM(struct feature, features, i));
    }

    ComputeFeatureDescriptorsWorkerData *data =
        new ComputeFeatureDescriptorsWorkerData(all_features, gauss_pyr, d, n);

    ParallelTask task(&executor, data, &task_sem);
    parallel_server.executeParallelTask(task);

    task_sem.wait();
    delete data;
}



/*
  Computes the 2D array of orientation histograms that form the feature
  descriptor.  Based on Section 6.1 of Lowe's paper.

  @param img image used in descriptor computation
  @param r row coord of center of orientation histogram array
  @param c column coord of center of orientation histogram array
  @param ori canonical orientation of feature whose descr is being computed
  @param scl scale relative to img of feature whose descr is being computed
  @param d width of 2d array of orientation histograms
  @param n bins per orientation histogram

  @return Returns a d x d array of n-bin orientation histograms.
*/
float*** descrHist(const IplImage* img, int r, int c, float ori,
                    float scl, int d, int n){

    float*** hist;
    float cos_t, sin_t, hist_width, exp_denom, r_rot, c_rot, grad_mag,
           grad_ori, w, rbin, cbin, obin, bins_per_rad, PI2 = 2.0f * (float)CV_PI;
    int radius, i, j;

    hist = (float ***)calloc(d, sizeof(float**));
    for(i = 0; i < d; i++){
        hist[i] = (float **)calloc(d, sizeof(float*));
        for(j = 0; j < d; j++){
            hist[i][j] = (float *)calloc(n, sizeof(float));
        }
    }

    cos_t = cosf(ori);
    sin_t = sinf(ori);
    bins_per_rad = n/PI2;
    exp_denom = d * d * 0.5f;
    hist_width = SIFT_DESCR_SCL_FCTR * scl;
    radius = (int)(hist_width*sqrtf(2.0f)*(d+1.0f)*0.5f + 0.5f);

    for(i = -radius; i <= radius; i++){
         for(j = -radius; j <= radius; j++){

            /*
            Calculate sample's histogram array coords rotated relative to ori.
            Subtract 0.5 so samples that fall e.g. in the center of row 1 (i.e.
            r_rot = 1.5) have full weight placed in row 1 after interpolation.
            */

            c_rot = (j*cos_t - i*sin_t)/hist_width;
            r_rot = (j*sin_t + i*cos_t)/hist_width;
            rbin = r_rot + d/2 - 0.5f;
            cbin = c_rot + d/2 - 0.5f;

            if(rbin > -1.0f && rbin < d && cbin > -1.0f && cbin < d){
	            if(calcGradMagOri(img, r + i, c + j, &grad_mag, &grad_ori)){
                    grad_ori -= ori;

                    while(grad_ori < 0.0f){ grad_ori += PI2; }
                    while(grad_ori >= PI2){ grad_ori -= PI2; } // TODO CAN DO THIS BETTER

                    obin = grad_ori * bins_per_rad;
                    w = expf(-(c_rot*c_rot + r_rot*r_rot)/exp_denom);
                    interpHistEntry(hist, rbin, cbin, obin, grad_mag*w, d, n);
                }
            }
        }
    }

    return hist;
}



/*
  Interpolates an entry into the array of orientation histograms that form
  the feature descriptor.

  @param hist 2D array of orientation histograms
  @param rbin sub-bin row coordinate of entry
  @param cbin sub-bin column coordinate of entry
  @param obin sub-bin orientation coordinate of entry
  @param mag size of entry
  @param d width of 2D array of orientation histograms
  @param n number of bins per orientation histogram
*/
void interpHistEntry(float*** hist, float rbin, float cbin,
                     float obin, float mag, int d, int n){

    float d_r, d_c, d_o, v_r, v_c, v_o;
    float** row, * h;
    int r0, c0, o0, rb, cb, ob;

    r0 = cvFloor(rbin);
    c0 = cvFloor(cbin);
    o0 = cvFloor(obin);
    d_r = rbin - r0;
    d_c = cbin - c0;
    d_o = obin - o0;

    /*
      The entry is distributed into up to 8 bins.  Each entry into a bin
      is multiplied by a weight of 1 - d for each dimension, where d is the
      distance from the center value of the bin measured in bin units.
    */
    for(int r = 0; r <= 1; r++){
        rb = r0 + r;

        if(rb >= 0 && rb < d){
            v_r = mag * ((r == 0) ? 1.0f-d_r : d_r);
            row = hist[rb];

            for(int c = 0; c <= 1; c++){
	            cb = c0 + c;

                if(cb >= 0 && cb < d){
                    v_c = v_r * ((c == 0 ) ? 1.0f-d_c : d_c);
                    h = row[cb];

                    for(int o = 0; o <= 1; o++){
                        ob = (o0 + o) % n;
                        v_o = v_c*((o == 0) ? 1.0f-d_o : d_o);
                        h[ob] += v_o;
                    }
                }
            }
        }
    }
}



/*
  Converts the 2D array of orientation histograms into a feature's descriptor
  vector.

  @param hist 2D array of orientation histograms
  @param d width of hist
  @param n bins per histogram
  @param feat feature into which to store descriptor
*/
void histToDescr(const float*** hist, int d, int n, struct feature* feat){
    int int_val, k = 0;

    for(int r = 0; r < d; r++){
        for(int c = 0; c < d; c++){
            for(int o = 0; o < n; o++){
                feat->descr[k++] = hist[r][c][o];
            }
        }
    }

    feat->d = k;
    normalizeDescr(feat);
    for(int i = 0; i < k; i++){
        if(feat->descr[i] > SIFT_DESCR_MAG_THR){
            feat->descr[i] = SIFT_DESCR_MAG_THR;
        }
    }
    normalizeDescr(feat);

    // convert floating-point descriptor to integer valued descriptor
    for(int i = 0; i < k; i++){
        int_val = (int)(SIFT_INT_DESCR_FCTR * feat->descr[i]);
        feat->descr[i] = (float)MIN(255, int_val);
    }
}



/*
  Normalizes a feature's descriptor vector to unit length

  @param feat feature
*/

void normalizeDescr(struct feature* feat){
#if 0

    float len_sq = 0.0f;
    const int d = feat->d;

    float len_sq_arr[4] = {0.0f};
    __m128 *s1 = (__m128 *)feat->descr;
    __m128 d1, d2, df;

    for(int i = 0; i < d/8; i++){
        d1 = _mm_mul_ps(*s1, *s1);
        s1++;

        d2 = _mm_mul_ps(*s1, *s1);
        s1++;

        df = _mm_hadd_ps(d1, d2);
        _mm_store_ps(len_sq_arr,df);

        len_sq += (len_sq_arr[0] + len_sq_arr[1] + len_sq_arr[2] + len_sq_arr[3]);
    }


    __m128 a = _mm_set_ps1(1.0f/sqrtf(len_sq));
    __m128 *b = (__m128 *)feat->descr;

    for(int i = 0; i < d/4; i++){
        *b = _mm_mul_ps(a, *b);
        b++;
    }

#else

    float cur, len_sq = 0.0f;
    const int d = feat->d;


    for(int i = 0; i < d; i++){
        cur = feat->descr[i];
        len_sq += cur*cur;
    }


    const float len_inv = 1.0f/sqrtf(len_sq);
    for(int i = 0; i < d; i++){
        feat->descr[i] *= len_inv;
    }

#endif

}



/*
  Compares features for a decreasing-scale ordering.  Intended for use with
  CvSeqSort

  @param feat1 first feature
  @param feat2 second feature
  @param param unused

  @return Returns 1 if feat1's scale is greater than feat2's, -1 if vice versa,
    and 0 if their scales are equal
*/
int featureCmp(const void* feat1, const void* feat2, const void* param){
    struct feature* f1 = (struct feature*) feat1;
    struct feature* f2 = (struct feature*) feat2;

    if(f1->scl < f2->scl){
        return 1;
    }
    if(f1->scl > f2->scl){
        return -1;
    }

    return 0;
}



/*
  De-allocates memory held by a descriptor histogram

  @param hist pointer to a 2D array of orientation histograms
  @param d width of hist
*/
void releaseDescrHist(float**** hist, int d){
    for(int i = 0; i < d; i++){
        for(int j = 0; j < d; j++){
            free((*hist)[i][j]);
        }
        free( (*hist)[i] );
    }

    free(*hist);
    *hist = NULL;
}


/*
  De-allocates memory held by a scale space pyramid

  @param pyr scale space pyramid
  @param octvs number of octaves of scale space
  @param n number of images per octave
*/
void releasePyr(IplImage**** pyr, int octvs, int n){
    for(int i = 0; i < octvs; i++){
        for(int j = 0; j < n; j++){
            cvReleaseImage(&(*pyr)[i][j]);
        }
        free((*pyr)[i]);
    }

    free(*pyr);
    *pyr = NULL;
}


// TODO cache the kernel, shouldnt have to recompute all the time.
// ** assumes an odd width
float* initGaussianKernel(int width, float sd){
    float *kernel = (float*)malloc(width*sizeof(float));
    float dist2 = 0.0f, weight = 0.0f, sum = 1.0f;
    const int middle = width/2;

    kernel[middle] = 1.0f; // the middle element.
    for(int x = 0; x < middle; x++){
        dist2 = (float)((x-middle)*(x-middle));
        weight = expf(-dist2/(2.0f*sd*sd));
        assert(weight <= 1.0f);

        kernel[x] = kernel[width-1-x] = weight;

        sum += 2.0f*weight;
    }

    sum = 1.0f/sum; // normalise factor.
    for(int i = 0; i < width; i++){
        kernel[i] *= sum;
    }

    return kernel;
}

void smoothImage(const IplImage *src_img, IplImage* dst_img, float sd){
    int kernel_width = cvRound(sd*8 + 1)|1;
    int middlek = kernel_width/2;
    float *kernel = initGaussianKernel(kernel_width, sd);

    int step = src_img->widthStep/sizeof(float);
    assert(src_img->widthStep == dst_img->widthStep);
    assert(src_img->width == dst_img->width);
    assert(src_img->height == dst_img->height);

    IplImage* int_img = cvCreateImage(cvGetSize(dst_img), IPL_DEPTH_32F, 1);
    //IplImage *int_img_t = cvCreateImage(cvSize(dst_img->height, dst_img->width), IPL_DEPTH_32F, 1);
    //IplImage *dst_img_t = cvCreateImage(cvSize(dst_img->height, dst_img->width), IPL_DEPTH_32F, 1);
    //int tstep = int_img_t->widthStep/sizeof(float);

    const float *src_img_data = reinterpret_cast<const float*>(src_img->imageData);
    float *int_img_data = reinterpret_cast<float*>(int_img->imageData);
    float *dst_img_data = reinterpret_cast<float*>(dst_img->imageData);
    //float *int_img_t_data = reinterpret_cast<float*>(int_img_t->imageData);
    //float *dst_img_t_data = reinterpret_cast<float*>(dst_img_t->imageData);

    // Zero out the dst and intermediate image.
    for(int y = 0; y < dst_img->height; y++){
        for(int x = 0; x < dst_img->width; x++){
            dst_img_data[x] = int_img_data[x] = 0.0f;
        }
        dst_img_data += step;
        int_img_data += step;
    }
    dst_img_data = reinterpret_cast<float*>(dst_img->imageData);
    int_img_data = reinterpret_cast<float*>(int_img->imageData);



    float weight = 0.0f;
    int src_x = 0, src_y = 0, dst_x = 0, dst_y = 0;
    (void)src_x; (void)src_y; (void)dst_x; (void)dst_y;
    (void)weight;
    //(void)src_val;

    if(src_img->width < kernel_width){
        for(int y = 0; y < src_img->height; y++){
            for(int x = 0; x < src_img->width; x++){
                for(int i = 0; i < middlek; i++){
                    weight = src_img_data[x]*kernel[i];

                    if(x + (middlek-i) < src_img->width){
                        int_img_data[x + (middlek-i)] += weight;
                    }

                    if(x - (middlek-i) >= 0){
                        int_img_data[x - (middlek-i)] += weight;
                    }
                }
                weight = src_img_data[x]*kernel[middlek];
                int_img_data[x] += weight;
            }

            float suml = 0.0f, sumr = 0.0f;
            for(int i = 0; i < middlek; i++){
                suml += src_img_data[0] * kernel[i];
                sumr += src_img_data[src_img->width-1] * kernel[i];

                if(middlek-i-1 < int_img->width){
                    int_img_data[middlek-i-1] += suml;
                }

                if(src_img->width-1 - (middlek-i-1) > 0){
                    int_img_data[src_img->width-1 - (middlek-i-1)] += sumr;
                }
            }

            int_img_data += step;
            src_img_data += step;
        }
    }
    else{
        for(int y = 0; y < src_img->height; y++){

            for(int x = 0; x < middlek; x++){
                for(int i = 0; i < middlek; i++){
                    weight = src_img_data[x]*kernel[i];
                    int_img_data[x + (middlek-i)] += weight;

                    if(x - (middlek-i) >= 0){
                        int_img_data[x - (middlek-i)] += weight;
                    }
                }
                weight = src_img_data[x]*kernel[middlek];
                int_img_data[x] += weight;
            }

            float suml = 0.0f;
            for(int i = 0; i < middlek; i++){
                suml += src_img_data[0] * kernel[i];

                if(middlek-i-1 < int_img->width){
                    int_img_data[middlek-i-1] += suml;
                }
            }

            for(int x = middlek; x < src_img->width-middlek; x++){
                for(int i = 0; i < middlek; i++){
                    weight = src_img_data[x]*kernel[i];
                    int_img_data[x + (middlek-i)] += weight;
                    int_img_data[x - (middlek-i)] += weight;
                }
                weight = src_img_data[x]*kernel[middlek];
                int_img_data[x] += weight;
            }

            for(int x = src_img->width-middlek; x < src_img->width; x++){
                for(int i = 0; i < middlek; i++){
                    weight = src_img_data[x]*kernel[i];
                    int_img_data[x - (middlek-i)] += weight;

                    if((middlek-i)+x < src_img->width){
                        int_img_data[x + (middlek-i)] += weight;
                    }
                }
                weight = src_img_data[x]*kernel[middlek];
                int_img_data[x] += weight;
            }

            float sumr = 0.0f;
            for(int i = 0; i < middlek; i++){
                sumr += src_img_data[src_img->width-1] * kernel[i];

                if(src_img->width-1 - (middlek-i-1) > 0){
                    int_img_data[src_img->width-1 - (middlek-i-1)] += sumr;
                }
            }


            int_img_data += step;
            src_img_data += step;
        }
    }

    src_img_data = reinterpret_cast<float*>(src_img->imageData);
    int_img_data = reinterpret_cast<float*>(int_img->imageData);

   /*
    for(int y = 0; y < int_img->height; y++){

        for(int x = middlek; x < int_img->width-middlek; x++){
            for(int i = 0; i < kernel_width; i++){
                src_x = x + (i - middlek);
                int_img_data[x] += kernel[i]*src_img_data[src_x];
            }
        }

        for(int x = 0; x < middlek; x++){
            for(int i = 0; i < kernel_width; i++){
                src_x = x + (i - middlek);
                if(src_x < 0){ src_x = 0; }
                int_img_data[x] += kernel[i]*src_img_data[src_x];
            }
        }

        for(int x = int_img->width-middlek; x < int_img->width; x++){
            for(int i = 0; i < kernel_width; i++){
                src_x = x + (i - middlek);
                if(src_x > int_img->width-1){ src_x = int_img->width-1; }
                int_img_data[x] += kernel[i]*src_img_data[src_x];
            }
        }

        int_img_data += step;
        src_img_data += step;
    }

    src_img_data = reinterpret_cast<float*>(src_img->imageData);
    int_img_data = reinterpret_cast<float*>(int_img->imageData);
*/

    if(dst_img->height < kernel_width){
        for(int y = 0; y < dst_img->height; y++){
            for(int i = 0; i < kernel_width; i++){
                int offset_row = y + (i - middlek);
                if(offset_row < 0){ offset_row = 0; }
                if(offset_row > dst_img->height-1){ offset_row = dst_img->height-1; }

                for(int x = 0; x < dst_img->width; x++){
                    dst_img_data[x] += kernel[i]*int_img_data[x + offset_row*step];
                }

            }
            dst_img_data += step;
        }
    }
    else{
        for(int y = 0; y < middlek; y++){
            for(int i = 0; i < kernel_width; i++){
                int offset_row = y + (i - middlek);
                if(offset_row < 0){ offset_row = 0; }

                float *offset_row_data = &(int_img_data[offset_row*step]);
                for(int x = 0; x < dst_img->width; x++){
                    dst_img_data[x] += kernel[i]*offset_row_data[x];
                }
            }
            dst_img_data += step;
        }


        for(int y = middlek; y < dst_img->height-middlek; y++){
            for(int i = 0; i < kernel_width; i++){
                int offset_row = y + (i-middlek);

                float *offset_row_data = &(int_img_data[offset_row*step]);
                for(int x = 0; x < dst_img->width; x++){
                    dst_img_data[x] += kernel[i]*offset_row_data[x];
                }
            }
            dst_img_data += step;
        }

        for(int y = dst_img->height-middlek; y < dst_img->height; y++){
            for(int i = 0; i < kernel_width; i++){
                int offset_row = y + (i - middlek);
                if(offset_row > dst_img->height-1){ offset_row = dst_img->height-1; }

                float *offset_row_data = &(int_img_data[offset_row*step]);
                for(int x = 0; x < dst_img->width; x++){
                    dst_img_data[x] += kernel[i]*offset_row_data[x];
                }
            }
            dst_img_data += step;
        }
    }

/*
    if(dst_img->height < kernel_width){
        for(int y = 0; y < dst_img->height; y++){
            for(int i = 0; i < kernel_width; i++){
                int offset_row = y + (i - middlek);
                if(offset_row < 0){ offset_row = 0; }
                if(offset_row > dst_img->height-1){ offset_row = dst_img->height-1; }

                for(int x = 0; x < dst_img->width; x++){
                    dst_img_data[x] += kernel[i]*int_img_data[x + offset_row*step];
                }

            }
            dst_img_data += step;
        }
    }
    else{
        for(int y = middlek; y < int_img->height-middlek; y++){
            for(int i = 0; i < middlek; i++){
                float *top_row = &(dst_img_data[(y - (middlek - i))*step]);

                for(int x = 0; x < int_img->width; x++){
                    weight = int_img_data[x]*kernel[i];
                    top_row[x] += weight;
                }
            }

            for(int i = 0; i < middlek; i++){
                float *bottom_row = &(dst_img_data[(y + i + 1)*step]);

                for(int x = 0; x < int_img->width; x++){
                    weight = int_img_data[x]*kernel[i];
                    bottom_row[x] += weight;
                }
            }


            float *cur_row = &(dst_img_data[y*step]);
            for(int x = 0; x < int_img->width; x++){
                cur_row[x] +=  int_img_data[x]*kernel[middlek];
            }

            int_img_data += step;
        }

    }
*/

    cvReleaseImage(&int_img);
    free(kernel);
}


