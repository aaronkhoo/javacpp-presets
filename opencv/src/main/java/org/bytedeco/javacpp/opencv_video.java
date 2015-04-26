// Targeted by JavaCPP version 0.11

package org.bytedeco.javacpp;

import java.nio.*;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import static org.bytedeco.javacpp.opencv_core.*;
import static org.bytedeco.javacpp.opencv_imgproc.*;

public class opencv_video extends org.bytedeco.javacpp.helper.opencv_video {
    static { Loader.load(); }

// Parsed from <opencv2/video.hpp>

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

// #ifndef __OPENCV_VIDEO_HPP__
// #define __OPENCV_VIDEO_HPP__

// #include "opencv2/video/tracking.hpp"
// #include "opencv2/video/background_segm.hpp"

// #endif //__OPENCV_VIDEO_HPP__


// Parsed from <opencv2/video/tracking_c.h>

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

// #ifndef __OPENCV_TRACKING_C_H__
// #define __OPENCV_TRACKING_C_H__

// #include "opencv2/imgproc/types_c.h"

// #ifdef __cplusplus
// #endif

/****************************************************************************************\
*                                  Motion Analysis                                       *
\****************************************************************************************/

/************************************ optical flow ***************************************/

public static final int CV_LKFLOW_PYR_A_READY =       1;
public static final int CV_LKFLOW_PYR_B_READY =       2;
public static final int CV_LKFLOW_INITIAL_GUESSES =   4;
public static final int CV_LKFLOW_GET_MIN_EIGENVALS = 8;

/* It is Lucas & Kanade method, modified to use pyramids.
   Also it does several iterations to get optical flow for
   every point at every pyramid level.
   Calculates optical flow between two images for certain set of points (i.e.
   it is a "sparse" optical flow, which is opposite to the previous 3 methods) */
public static native void cvCalcOpticalFlowPyrLK( @Const CvArr prev, @Const CvArr curr,
                                     CvArr prev_pyr, CvArr curr_pyr,
                                     @Const CvPoint2D32f prev_features,
                                     CvPoint2D32f curr_features,
                                     int count,
                                     @ByVal CvSize win_size,
                                     int level,
                                     @Cast("char*") BytePointer status,
                                     FloatPointer track_error,
                                     @ByVal CvTermCriteria criteria,
                                     int flags );
public static native void cvCalcOpticalFlowPyrLK( @Const CvArr prev, @Const CvArr curr,
                                     CvArr prev_pyr, CvArr curr_pyr,
                                     @Cast("const CvPoint2D32f*") FloatBuffer prev_features,
                                     @Cast("CvPoint2D32f*") FloatBuffer curr_features,
                                     int count,
                                     @ByVal CvSize win_size,
                                     int level,
                                     @Cast("char*") ByteBuffer status,
                                     FloatBuffer track_error,
                                     @ByVal CvTermCriteria criteria,
                                     int flags );
public static native void cvCalcOpticalFlowPyrLK( @Const CvArr prev, @Const CvArr curr,
                                     CvArr prev_pyr, CvArr curr_pyr,
                                     @Cast("const CvPoint2D32f*") float[] prev_features,
                                     @Cast("CvPoint2D32f*") float[] curr_features,
                                     int count,
                                     @ByVal CvSize win_size,
                                     int level,
                                     @Cast("char*") byte[] status,
                                     float[] track_error,
                                     @ByVal CvTermCriteria criteria,
                                     int flags );


/* Modification of a previous sparse optical flow algorithm to calculate
   affine flow */
public static native void cvCalcAffineFlowPyrLK( @Const CvArr prev, @Const CvArr curr,
                                    CvArr prev_pyr, CvArr curr_pyr,
                                    @Const CvPoint2D32f prev_features,
                                    CvPoint2D32f curr_features,
                                    FloatPointer matrices, int count,
                                    @ByVal CvSize win_size, int level,
                                    @Cast("char*") BytePointer status, FloatPointer track_error,
                                    @ByVal CvTermCriteria criteria, int flags );
public static native void cvCalcAffineFlowPyrLK( @Const CvArr prev, @Const CvArr curr,
                                    CvArr prev_pyr, CvArr curr_pyr,
                                    @Cast("const CvPoint2D32f*") FloatBuffer prev_features,
                                    @Cast("CvPoint2D32f*") FloatBuffer curr_features,
                                    FloatBuffer matrices, int count,
                                    @ByVal CvSize win_size, int level,
                                    @Cast("char*") ByteBuffer status, FloatBuffer track_error,
                                    @ByVal CvTermCriteria criteria, int flags );
public static native void cvCalcAffineFlowPyrLK( @Const CvArr prev, @Const CvArr curr,
                                    CvArr prev_pyr, CvArr curr_pyr,
                                    @Cast("const CvPoint2D32f*") float[] prev_features,
                                    @Cast("CvPoint2D32f*") float[] curr_features,
                                    float[] matrices, int count,
                                    @ByVal CvSize win_size, int level,
                                    @Cast("char*") byte[] status, float[] track_error,
                                    @ByVal CvTermCriteria criteria, int flags );

/* Estimate rigid transformation between 2 images or 2 point sets */
public static native int cvEstimateRigidTransform( @Const CvArr A, @Const CvArr B,
                                      CvMat M, int full_affine );

/* Estimate optical flow for each pixel using the two-frame G. Farneback algorithm */
public static native void cvCalcOpticalFlowFarneback( @Const CvArr prev, @Const CvArr next,
                                        CvArr flow, double pyr_scale, int levels,
                                        int winsize, int iterations, int poly_n,
                                        double poly_sigma, int flags );

/********************************* motion templates *************************************/

/****************************************************************************************\
*        All the motion template functions work only with single channel images.         *
*        Silhouette image must have depth IPL_DEPTH_8U or IPL_DEPTH_8S                   *
*        Motion history image must have depth IPL_DEPTH_32F,                             *
*        Gradient mask - IPL_DEPTH_8U or IPL_DEPTH_8S,                                   *
*        Motion orientation image - IPL_DEPTH_32F                                        *
*        Segmentation mask - IPL_DEPTH_32F                                               *
*        All the angles are in degrees, all the times are in milliseconds                *
\****************************************************************************************/

/* Updates motion history image given motion silhouette */
public static native void cvUpdateMotionHistory( @Const CvArr silhouette, CvArr mhi,
                                      double timestamp, double duration );

/* Calculates gradient of the motion history image and fills
   a mask indicating where the gradient is valid */
public static native void cvCalcMotionGradient( @Const CvArr mhi, CvArr mask, CvArr orientation,
                                     double delta1, double delta2,
                                     int aperture_size/*=3*/);
public static native void cvCalcMotionGradient( @Const CvArr mhi, CvArr mask, CvArr orientation,
                                     double delta1, double delta2);

/* Calculates average motion direction within a selected motion region
   (region can be selected by setting ROIs and/or by composing a valid gradient mask
   with the region mask) */
public static native double cvCalcGlobalOrientation( @Const CvArr orientation, @Const CvArr mask,
                                        @Const CvArr mhi, double timestamp,
                                        double duration );

/* Splits a motion history image into a few parts corresponding to separate independent motions
   (e.g. left hand, right hand) */
public static native CvSeq cvSegmentMotion( @Const CvArr mhi, CvArr seg_mask,
                                CvMemStorage storage,
                                double timestamp, double seg_thresh );

/****************************************************************************************\
*                                       Tracking                                         *
\****************************************************************************************/

/* Implements CAMSHIFT algorithm - determines object position, size and orientation
   from the object histogram back project (extension of meanshift) */
public static native int cvCamShift( @Const CvArr prob_image, @ByVal CvRect window,
                        @ByVal CvTermCriteria criteria, CvConnectedComp comp,
                        CvBox2D box/*=NULL*/ );
public static native int cvCamShift( @Const CvArr prob_image, @ByVal CvRect window,
                        @ByVal CvTermCriteria criteria, CvConnectedComp comp );

/* Implements MeanShift algorithm - determines object position
   from the object histogram back project */
public static native int cvMeanShift( @Const CvArr prob_image, @ByVal CvRect window,
                         @ByVal CvTermCriteria criteria, CvConnectedComp comp );

/*
standard Kalman filter (in G. Welch' and G. Bishop's notation):

  x(k)=A*x(k-1)+B*u(k)+w(k)  p(w)~N(0,Q)
  z(k)=H*x(k)+v(k),   p(v)~N(0,R)
*/
public static class CvKalman extends AbstractCvKalman {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvKalman() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvKalman(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvKalman(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvKalman position(int position) {
        return (CvKalman)super.position(position);
    }

    public native int MP(); public native CvKalman MP(int MP);                     /* number of measurement vector dimensions */
    public native int DP(); public native CvKalman DP(int DP);                     /* number of state vector dimensions */
    public native int CP(); public native CvKalman CP(int CP);                     /* number of control vector dimensions */

    /* backward compatibility fields */
// #if 1
    public native FloatPointer PosterState(); public native CvKalman PosterState(FloatPointer PosterState);         /* =state_pre->data.fl */
    public native FloatPointer PriorState(); public native CvKalman PriorState(FloatPointer PriorState);          /* =state_post->data.fl */
    public native FloatPointer DynamMatr(); public native CvKalman DynamMatr(FloatPointer DynamMatr);           /* =transition_matrix->data.fl */
    public native FloatPointer MeasurementMatr(); public native CvKalman MeasurementMatr(FloatPointer MeasurementMatr);     /* =measurement_matrix->data.fl */
    public native FloatPointer MNCovariance(); public native CvKalman MNCovariance(FloatPointer MNCovariance);        /* =measurement_noise_cov->data.fl */
    public native FloatPointer PNCovariance(); public native CvKalman PNCovariance(FloatPointer PNCovariance);        /* =process_noise_cov->data.fl */
    public native FloatPointer KalmGainMatr(); public native CvKalman KalmGainMatr(FloatPointer KalmGainMatr);        /* =gain->data.fl */
    public native FloatPointer PriorErrorCovariance(); public native CvKalman PriorErrorCovariance(FloatPointer PriorErrorCovariance);/* =error_cov_pre->data.fl */
    public native FloatPointer PosterErrorCovariance(); public native CvKalman PosterErrorCovariance(FloatPointer PosterErrorCovariance);/* =error_cov_post->data.fl */
    public native FloatPointer Temp1(); public native CvKalman Temp1(FloatPointer Temp1);               /* temp1->data.fl */
    public native FloatPointer Temp2(); public native CvKalman Temp2(FloatPointer Temp2);               /* temp2->data.fl */
// #endif

    public native CvMat state_pre(); public native CvKalman state_pre(CvMat state_pre);           /* predicted state (x'(k)):
                                    x(k)=A*x(k-1)+B*u(k) */
    public native CvMat state_post(); public native CvKalman state_post(CvMat state_post);          /* corrected state (x(k)):
                                    x(k)=x'(k)+K(k)*(z(k)-H*x'(k)) */
    public native CvMat transition_matrix(); public native CvKalman transition_matrix(CvMat transition_matrix);   /* state transition matrix (A) */
    public native CvMat control_matrix(); public native CvKalman control_matrix(CvMat control_matrix);      /* control matrix (B)
                                   (it is not used if there is no control)*/
    public native CvMat measurement_matrix(); public native CvKalman measurement_matrix(CvMat measurement_matrix);  /* measurement matrix (H) */
    public native CvMat process_noise_cov(); public native CvKalman process_noise_cov(CvMat process_noise_cov);   /* process noise covariance matrix (Q) */
    public native CvMat measurement_noise_cov(); public native CvKalman measurement_noise_cov(CvMat measurement_noise_cov); /* measurement noise covariance matrix (R) */
    public native CvMat error_cov_pre(); public native CvKalman error_cov_pre(CvMat error_cov_pre);       /* priori error estimate covariance matrix (P'(k)):
                                    P'(k)=A*P(k-1)*At + Q)*/
    public native CvMat gain(); public native CvKalman gain(CvMat gain);                /* Kalman gain matrix (K(k)):
                                    K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)*/
    public native CvMat error_cov_post(); public native CvKalman error_cov_post(CvMat error_cov_post);      /* posteriori error estimate covariance matrix (P(k)):
                                    P(k)=(I-K(k)*H)*P'(k) */
    public native CvMat temp1(); public native CvKalman temp1(CvMat temp1);               /* temporary matrices */
    public native CvMat temp2(); public native CvKalman temp2(CvMat temp2);
    public native CvMat temp3(); public native CvKalman temp3(CvMat temp3);
    public native CvMat temp4(); public native CvKalman temp4(CvMat temp4);
    public native CvMat temp5(); public native CvKalman temp5(CvMat temp5);
}

/* Creates Kalman filter and sets A, B, Q, R and state to some initial values */
public static native CvKalman cvCreateKalman( int dynam_params, int measure_params,
                                 int control_params/*=0*/);
public static native CvKalman cvCreateKalman( int dynam_params, int measure_params);

/* Releases Kalman filter state */
public static native void cvReleaseKalman( @Cast("CvKalman**") PointerPointer kalman);
public static native void cvReleaseKalman( @ByPtrPtr CvKalman kalman);

/* Updates Kalman filter by time (predicts future state of the system) */
public static native @Const CvMat cvKalmanPredict( CvKalman kalman,
                                      @Const CvMat control/*=NULL*/);
public static native @Const CvMat cvKalmanPredict( CvKalman kalman);

/* Updates Kalman filter by measurement
   (corrects state of the system and internal matrices) */
public static native @Const CvMat cvKalmanCorrect( CvKalman kalman, @Const CvMat measurement );

public static native @Const CvMat cvKalmanUpdateByTime(CvKalman arg1, CvMat arg2);
public static native @Const CvMat cvKalmanUpdateByMeasurement(CvKalman arg1, CvMat arg2);


// #ifdef __cplusplus // extern "C"
// #endif


// #endif // __OPENCV_TRACKING_C_H__


// Parsed from <opencv2/video/tracking.hpp>

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

// #ifndef __OPENCV_TRACKING_HPP__
// #define __OPENCV_TRACKING_HPP__

// #include "opencv2/core.hpp"
// #include "opencv2/imgproc.hpp"

/** enum cv:: */
public static final int OPTFLOW_USE_INITIAL_FLOW     = 4,
       OPTFLOW_LK_GET_MIN_EIGENVALS = 8,
       OPTFLOW_FARNEBACK_GAUSSIAN   = 256;

/** updates the object tracking window using CAMSHIFT algorithm */
@Namespace("cv") public static native @ByVal RotatedRect CamShift( @ByVal Mat probImage, @ByRef Rect window,
                                   @ByVal TermCriteria criteria );

/** updates the object tracking window using meanshift algorithm */
@Namespace("cv") public static native int meanShift( @ByVal Mat probImage, @ByRef Rect window, @ByVal TermCriteria criteria );

/** constructs a pyramid which can be used as input for calcOpticalFlowPyrLK */
@Namespace("cv") public static native int buildOpticalFlowPyramid( @ByVal Mat img, @ByVal MatVector pyramid,
                                          @ByVal Size winSize, int maxLevel, @Cast("bool") boolean withDerivatives/*=true*/,
                                          int pyrBorder/*=BORDER_REFLECT_101*/,
                                          int derivBorder/*=BORDER_CONSTANT*/,
                                          @Cast("bool") boolean tryReuseInputImage/*=true*/ );
@Namespace("cv") public static native int buildOpticalFlowPyramid( @ByVal Mat img, @ByVal MatVector pyramid,
                                          @ByVal Size winSize, int maxLevel );

/** computes sparse optical flow using multi-scale Lucas-Kanade algorithm */
@Namespace("cv") public static native void calcOpticalFlowPyrLK( @ByVal Mat prevImg, @ByVal Mat nextImg,
                                        @ByVal Mat prevPts, @ByVal Mat nextPts,
                                        @ByVal Mat status, @ByVal Mat err,
                                        @ByVal Size winSize/*=Size(21,21)*/, int maxLevel/*=3*/,
                                        @ByVal TermCriteria criteria/*=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01)*/,
                                        int flags/*=0*/, double minEigThreshold/*=1e-4*/ );
@Namespace("cv") public static native void calcOpticalFlowPyrLK( @ByVal Mat prevImg, @ByVal Mat nextImg,
                                        @ByVal Mat prevPts, @ByVal Mat nextPts,
                                        @ByVal Mat status, @ByVal Mat err );

/** computes dense optical flow using Farneback algorithm */
@Namespace("cv") public static native void calcOpticalFlowFarneback( @ByVal Mat prev, @ByVal Mat next, @ByVal Mat flow,
                                            double pyr_scale, int levels, int winsize,
                                            int iterations, int poly_n, double poly_sigma,
                                            int flags );

/** estimates the best-fit Euqcidean, similarity, affine or perspective transformation */
// that maps one 2D point set to another or one image to another.
@Namespace("cv") public static native @ByVal Mat estimateRigidTransform( @ByVal Mat src, @ByVal Mat dst, @Cast("bool") boolean fullAffine );


/** enum cv:: */
public static final int
    MOTION_TRANSLATION = 0,
    MOTION_EUCLIDEAN   = 1,
    MOTION_AFFINE      = 2,
    MOTION_HOMOGRAPHY  = 3;

/** estimates the best-fit Translation, Euclidean, Affine or Perspective Transformation */
// with respect to Enhanced Correlation Coefficient criterion that maps one image to
// another (area-based alignment)
//
// see reference:
// Evangelidis, G. E., Psarakis, E.Z., Parametric Image Alignment using
// Enhanced Correlation Coefficient Maximization, PAMI, 30(8), 2008
@Namespace("cv") public static native double findTransformECC( @ByVal Mat templateImage, @ByVal Mat inputImage,
                                      @ByVal Mat warpMatrix, int motionType/*=MOTION_AFFINE*/,
                                      @ByVal TermCriteria criteria/*=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 50, 0.001)*/);
@Namespace("cv") public static native double findTransformECC( @ByVal Mat templateImage, @ByVal Mat inputImage,
                                      @ByVal Mat warpMatrix);

/**
 Kalman filter.

 The class implements standard Kalman filter http://en.wikipedia.org/wiki/Kalman_filter.
 However, you can modify KalmanFilter::transitionMatrix, KalmanFilter::controlMatrix and
 KalmanFilter::measurementMatrix to get the extended Kalman filter functionality.
*/
@Namespace("cv") @NoOffset public static class KalmanFilter extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public KalmanFilter(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public KalmanFilter(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public KalmanFilter position(int position) {
        return (KalmanFilter)super.position(position);
    }

    /** the default constructor */
    public KalmanFilter() { allocate(); }
    private native void allocate();
    /** the full constructor taking the dimensionality of the state, of the measurement and of the control vector */
    public KalmanFilter( int dynamParams, int measureParams, int controlParams/*=0*/, int type/*=CV_32F*/ ) { allocate(dynamParams, measureParams, controlParams, type); }
    private native void allocate( int dynamParams, int measureParams, int controlParams/*=0*/, int type/*=CV_32F*/ );
    public KalmanFilter( int dynamParams, int measureParams ) { allocate(dynamParams, measureParams); }
    private native void allocate( int dynamParams, int measureParams );
    /** re-initializes Kalman filter. The previous content is destroyed. */
    public native void init( int dynamParams, int measureParams, int controlParams/*=0*/, int type/*=CV_32F*/ );
    public native void init( int dynamParams, int measureParams );

    /** computes predicted state */
    public native @Const @ByRef Mat predict( @Const @ByRef Mat control/*=Mat()*/ );
    public native @Const @ByRef Mat predict( );
    /** updates the predicted state from the measurement */
    public native @Const @ByRef Mat correct( @Const @ByRef Mat measurement );

    /** predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k) */
    public native @ByRef Mat statePre(); public native KalmanFilter statePre(Mat statePre);
    /** corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k)) */
    public native @ByRef Mat statePost(); public native KalmanFilter statePost(Mat statePost);
    /** state transition matrix (A) */
    public native @ByRef Mat transitionMatrix(); public native KalmanFilter transitionMatrix(Mat transitionMatrix);
    /** control matrix (B) (not used if there is no control) */
    public native @ByRef Mat controlMatrix(); public native KalmanFilter controlMatrix(Mat controlMatrix);
    /** measurement matrix (H) */
    public native @ByRef Mat measurementMatrix(); public native KalmanFilter measurementMatrix(Mat measurementMatrix);
    /** process noise covariance matrix (Q) */
    public native @ByRef Mat processNoiseCov(); public native KalmanFilter processNoiseCov(Mat processNoiseCov);
    /** measurement noise covariance matrix (R) */
    public native @ByRef Mat measurementNoiseCov(); public native KalmanFilter measurementNoiseCov(Mat measurementNoiseCov);
    /** priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
    public native @ByRef Mat errorCovPre(); public native KalmanFilter errorCovPre(Mat errorCovPre);
    /** Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R) */
    public native @ByRef Mat gain(); public native KalmanFilter gain(Mat gain);
    /** posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k) */
    public native @ByRef Mat errorCovPost(); public native KalmanFilter errorCovPost(Mat errorCovPost);

    // temporary matrices
    public native @ByRef Mat temp1(); public native KalmanFilter temp1(Mat temp1);
    public native @ByRef Mat temp2(); public native KalmanFilter temp2(Mat temp2);
    public native @ByRef Mat temp3(); public native KalmanFilter temp3(Mat temp3);
    public native @ByRef Mat temp4(); public native KalmanFilter temp4(Mat temp4);
    public native @ByRef Mat temp5(); public native KalmanFilter temp5(Mat temp5);
}



@Namespace("cv") public static class DenseOpticalFlow extends Algorithm {
    static { Loader.load(); }
    /** Empty constructor. */
    public DenseOpticalFlow() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public DenseOpticalFlow(Pointer p) { super(p); }

    public native void calc( @ByVal Mat I0, @ByVal Mat I1, @ByVal Mat flow );
    public native void collectGarbage();
}

// Implementation of the Zach, Pock and Bischof Dual TV-L1 Optical Flow method
//
// see reference:
//   [1] C. Zach, T. Pock and H. Bischof, "A Duality Based Approach for Realtime TV-L1 Optical Flow".
//   [2] Javier Sanchez, Enric Meinhardt-Llopis and Gabriele Facciolo. "TV-L1 Optical Flow Estimation".
@Namespace("cv") public static native @Ptr DenseOpticalFlow createOptFlow_DualTVL1();

 // cv

// #endif


// Parsed from <opencv2/video/background_segm.hpp>

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

// #ifndef __OPENCV_BACKGROUND_SEGM_HPP__
// #define __OPENCV_BACKGROUND_SEGM_HPP__

// #include "opencv2/core.hpp"

/**
 The Base Class for Background/Foreground Segmentation

 The class is only used to define the common interface for
 the whole family of background/foreground segmentation algorithms.
*/
@Namespace("cv") public static class BackgroundSubtractor extends Algorithm {
    static { Loader.load(); }
    /** Empty constructor. */
    public BackgroundSubtractor() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public BackgroundSubtractor(Pointer p) { super(p); }

    /** the update operator that takes the next video frame and returns the current foreground mask as 8-bit binary image. */
    public native void apply(@ByVal Mat image, @ByVal Mat fgmask, double learningRate/*=-1*/);
    public native void apply(@ByVal Mat image, @ByVal Mat fgmask);

    /** computes a background image */
    public native void getBackgroundImage(@ByVal Mat backgroundImage);
}


/**
 The class implements the following algorithm:
 "Improved adaptive Gausian mixture model for background subtraction"
 Z.Zivkovic
 International Conference Pattern Recognition, UK, August, 2004.
 http://www.zoranz.net/Publications/zivkovic2004ICPR.pdf
 */
@Namespace("cv") public static class BackgroundSubtractorMOG2 extends BackgroundSubtractor {
    static { Loader.load(); }
    /** Empty constructor. */
    public BackgroundSubtractorMOG2() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public BackgroundSubtractorMOG2(Pointer p) { super(p); }

    public native int getHistory();
    public native void setHistory(int history);

    public native int getNMixtures();
    public native void setNMixtures(int nmixtures);//needs reinitialization!

    public native double getBackgroundRatio();
    public native void setBackgroundRatio(double ratio);

    public native double getVarThreshold();
    public native void setVarThreshold(double varThreshold);

    public native double getVarThresholdGen();
    public native void setVarThresholdGen(double varThresholdGen);

    public native double getVarInit();
    public native void setVarInit(double varInit);

    public native double getVarMin();
    public native void setVarMin(double varMin);

    public native double getVarMax();
    public native void setVarMax(double varMax);

    public native double getComplexityReductionThreshold();
    public native void setComplexityReductionThreshold(double ct);

    public native @Cast("bool") boolean getDetectShadows();
    public native void setDetectShadows(@Cast("bool") boolean detectShadows);

    public native int getShadowValue();
    public native void setShadowValue(int value);

    public native double getShadowThreshold();
    public native void setShadowThreshold(double threshold);
}

@Namespace("cv") public static native @Ptr BackgroundSubtractorMOG2 createBackgroundSubtractorMOG2(int history/*=500*/, double varThreshold/*=16*/,
                                   @Cast("bool") boolean detectShadows/*=true*/);
@Namespace("cv") public static native @Ptr BackgroundSubtractorMOG2 createBackgroundSubtractorMOG2();

/**
 The class implements the K nearest neigbours algorithm from:
 "Efficient Adaptive Density Estimation per Image Pixel for the Task of Background Subtraction"
 Z.Zivkovic, F. van der Heijden
 Pattern Recognition Letters, vol. 27, no. 7, pages 773-780, 2006
 http://www.zoranz.net/Publications/zivkovicPRL2006.pdf

 Fast for small foreground object. Results on the benchmark data is at http://www.changedetection.net.
*/

@Namespace("cv") public static class BackgroundSubtractorKNN extends BackgroundSubtractor {
    static { Loader.load(); }
    /** Empty constructor. */
    public BackgroundSubtractorKNN() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public BackgroundSubtractorKNN(Pointer p) { super(p); }

    public native int getHistory();
    public native void setHistory(int history);

    public native int getNSamples();
    public native void setNSamples(int _nN);//needs reinitialization!

    public native double getDist2Threshold();
    public native void setDist2Threshold(double _dist2Threshold);

    public native int getkNNSamples();
    public native void setkNNSamples(int _nkNN);

    public native @Cast("bool") boolean getDetectShadows();
    public native void setDetectShadows(@Cast("bool") boolean detectShadows);

    public native int getShadowValue();
    public native void setShadowValue(int value);

    public native double getShadowThreshold();
    public native void setShadowThreshold(double threshold);
}

@Namespace("cv") public static native @Ptr BackgroundSubtractorKNN createBackgroundSubtractorKNN(int history/*=500*/, double dist2Threshold/*=400.0*/,
                                   @Cast("bool") boolean detectShadows/*=true*/);
@Namespace("cv") public static native @Ptr BackgroundSubtractorKNN createBackgroundSubtractorKNN();

 // cv

// #endif


}
