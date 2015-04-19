// Targeted by JavaCPP version 0.11

package org.bytedeco.javacpp;

import java.nio.*;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import static org.bytedeco.javacpp.opencv_core.*;
import static org.bytedeco.javacpp.opencv_imgproc.*;
import static org.bytedeco.javacpp.opencv_imgcodecs.*;
import static org.bytedeco.javacpp.opencv_videoio.*;
import static org.bytedeco.javacpp.opencv_highgui.*;
import static org.bytedeco.javacpp.opencv_flann.*;
import static org.bytedeco.javacpp.opencv_ml.*;
import static org.bytedeco.javacpp.opencv_features2d.*;

public class opencv_calib3d extends org.bytedeco.javacpp.helper.opencv_calib3d {
    static { Loader.load(); }

// Parsed from <opencv2/calib3d/calib3d_c.h>

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

// #ifndef __OPENCV_CALIB3D_C_H__
// #define __OPENCV_CALIB3D_C_H__

// #include "opencv2/core/core_c.h"

// #ifdef __cplusplus
// #endif

/****************************************************************************************\
*                      Camera Calibration, Pose Estimation and Stereo                    *
\****************************************************************************************/

@Opaque public static class CvPOSITObject extends AbstractCvPOSITObject {
    /** Empty constructor. */
    public CvPOSITObject() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvPOSITObject(Pointer p) { super(p); }
}

/* Allocates and initializes CvPOSITObject structure before doing cvPOSIT */
public static native CvPOSITObject cvCreatePOSITObject( CvPoint3D32f points, int point_count );
public static native CvPOSITObject cvCreatePOSITObject( @Cast("CvPoint3D32f*") FloatBuffer points, int point_count );
public static native CvPOSITObject cvCreatePOSITObject( @Cast("CvPoint3D32f*") float[] points, int point_count );


/* Runs POSIT (POSe from ITeration) algorithm for determining 3d position of
   an object given its model and projection in a weak-perspective case */
public static native void cvPOSIT(  CvPOSITObject posit_object, CvPoint2D32f image_points,
                       double focal_length, @ByVal CvTermCriteria criteria,
                       FloatPointer rotation_matrix, FloatPointer translation_vector);
public static native void cvPOSIT(  CvPOSITObject posit_object, @Cast("CvPoint2D32f*") FloatBuffer image_points,
                       double focal_length, @ByVal CvTermCriteria criteria,
                       FloatBuffer rotation_matrix, FloatBuffer translation_vector);
public static native void cvPOSIT(  CvPOSITObject posit_object, @Cast("CvPoint2D32f*") float[] image_points,
                       double focal_length, @ByVal CvTermCriteria criteria,
                       float[] rotation_matrix, float[] translation_vector);

/* Releases CvPOSITObject structure */
public static native void cvReleasePOSITObject( @Cast("CvPOSITObject**") PointerPointer posit_object );
public static native void cvReleasePOSITObject( @ByPtrPtr CvPOSITObject posit_object );

/* updates the number of RANSAC iterations */
public static native int cvRANSACUpdateNumIters( double p, double err_prob,
                                   int model_points, int max_iters );

public static native void cvConvertPointsHomogeneous( @Const CvMat src, CvMat dst );

/* Calculates fundamental matrix given a set of corresponding points */
public static final int CV_FM_7POINT = 1;
public static final int CV_FM_8POINT = 2;

public static final int CV_LMEDS = 4;
public static final int CV_RANSAC = 8;

public static final int CV_FM_LMEDS_ONLY =  CV_LMEDS;
public static final int CV_FM_RANSAC_ONLY = CV_RANSAC;
public static final int CV_FM_LMEDS = CV_LMEDS;
public static final int CV_FM_RANSAC = CV_RANSAC;

/** enum  */
public static final int
    CV_ITERATIVE = 0,
    CV_EPNP = 1, // F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
    CV_P3P = 2, // X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
    CV_DLS = 3; // Joel A. Hesch and Stergios I. Roumeliotis. "A Direct Least-Squares (DLS) Method for PnP"

public static native int cvFindFundamentalMat( @Const CvMat points1, @Const CvMat points2,
                                 CvMat fundamental_matrix,
                                 int method/*=CV_FM_RANSAC*/,
                                 double param1/*=3.*/, double param2/*=0.99*/,
                                 CvMat status/*=NULL*/ );
public static native int cvFindFundamentalMat( @Const CvMat points1, @Const CvMat points2,
                                 CvMat fundamental_matrix );

/* For each input point on one of images
   computes parameters of the corresponding
   epipolar line on the other image */
public static native void cvComputeCorrespondEpilines( @Const CvMat points,
                                         int which_image,
                                         @Const CvMat fundamental_matrix,
                                         CvMat correspondent_lines );

/* Triangulation functions */

public static native void cvTriangulatePoints(CvMat projMatr1, CvMat projMatr2,
                                CvMat projPoints1, CvMat projPoints2,
                                CvMat points4D);

public static native void cvCorrectMatches(CvMat F, CvMat points1, CvMat points2,
                             CvMat new_points1, CvMat new_points2);


/* Computes the optimal new camera matrix according to the free scaling parameter alpha:
   alpha=0 - only valid pixels will be retained in the undistorted image
   alpha=1 - all the source image pixels will be retained in the undistorted image
*/
public static native void cvGetOptimalNewCameraMatrix( @Const CvMat camera_matrix,
                                         @Const CvMat dist_coeffs,
                                         @ByVal CvSize image_size, double alpha,
                                         CvMat new_camera_matrix,
                                         @ByVal CvSize new_imag_size/*=cvSize(0,0)*/,
                                         CvRect valid_pixel_ROI/*=0*/,
                                         int center_principal_point/*=0*/);
public static native void cvGetOptimalNewCameraMatrix( @Const CvMat camera_matrix,
                                         @Const CvMat dist_coeffs,
                                         @ByVal CvSize image_size, double alpha,
                                         CvMat new_camera_matrix);

/* Converts rotation vector to rotation matrix or vice versa */
public static native int cvRodrigues2( @Const CvMat src, CvMat dst,
                         CvMat jacobian/*=0*/ );
public static native int cvRodrigues2( @Const CvMat src, CvMat dst );

/* Finds perspective transformation between the object plane and image (view) plane */
public static native int cvFindHomography( @Const CvMat src_points,
                             @Const CvMat dst_points,
                             CvMat homography,
                             int method/*=0*/,
                             double ransacReprojThreshold/*=3*/,
                             CvMat mask/*=0*/,
                             int maxIters/*=2000*/,
                             double confidence/*=0.995*/);
public static native int cvFindHomography( @Const CvMat src_points,
                             @Const CvMat dst_points,
                             CvMat homography);

/* Computes RQ decomposition for 3x3 matrices */
public static native void cvRQDecomp3x3( @Const CvMat matrixM, CvMat matrixR, CvMat matrixQ,
                           CvMat matrixQx/*=NULL*/,
                           CvMat matrixQy/*=NULL*/,
                           CvMat matrixQz/*=NULL*/,
                           CvPoint3D64f eulerAngles/*=NULL*/);
public static native void cvRQDecomp3x3( @Const CvMat matrixM, CvMat matrixR, CvMat matrixQ);
public static native void cvRQDecomp3x3( @Const CvMat matrixM, CvMat matrixR, CvMat matrixQ,
                           CvMat matrixQx/*=NULL*/,
                           CvMat matrixQy/*=NULL*/,
                           CvMat matrixQz/*=NULL*/,
                           @Cast("CvPoint3D64f*") DoubleBuffer eulerAngles/*=NULL*/);
public static native void cvRQDecomp3x3( @Const CvMat matrixM, CvMat matrixR, CvMat matrixQ,
                           CvMat matrixQx/*=NULL*/,
                           CvMat matrixQy/*=NULL*/,
                           CvMat matrixQz/*=NULL*/,
                           @Cast("CvPoint3D64f*") double[] eulerAngles/*=NULL*/);

/* Computes projection matrix decomposition */
public static native void cvDecomposeProjectionMatrix( @Const CvMat projMatr, CvMat calibMatr,
                                         CvMat rotMatr, CvMat posVect,
                                         CvMat rotMatrX/*=NULL*/,
                                         CvMat rotMatrY/*=NULL*/,
                                         CvMat rotMatrZ/*=NULL*/,
                                         CvPoint3D64f eulerAngles/*=NULL*/);
public static native void cvDecomposeProjectionMatrix( @Const CvMat projMatr, CvMat calibMatr,
                                         CvMat rotMatr, CvMat posVect);
public static native void cvDecomposeProjectionMatrix( @Const CvMat projMatr, CvMat calibMatr,
                                         CvMat rotMatr, CvMat posVect,
                                         CvMat rotMatrX/*=NULL*/,
                                         CvMat rotMatrY/*=NULL*/,
                                         CvMat rotMatrZ/*=NULL*/,
                                         @Cast("CvPoint3D64f*") DoubleBuffer eulerAngles/*=NULL*/);
public static native void cvDecomposeProjectionMatrix( @Const CvMat projMatr, CvMat calibMatr,
                                         CvMat rotMatr, CvMat posVect,
                                         CvMat rotMatrX/*=NULL*/,
                                         CvMat rotMatrY/*=NULL*/,
                                         CvMat rotMatrZ/*=NULL*/,
                                         @Cast("CvPoint3D64f*") double[] eulerAngles/*=NULL*/);

/* Computes d(AB)/dA and d(AB)/dB */
public static native void cvCalcMatMulDeriv( @Const CvMat A, @Const CvMat B, CvMat dABdA, CvMat dABdB );

/* Computes r3 = rodrigues(rodrigues(r2)*rodrigues(r1)),
   t3 = rodrigues(r2)*t1 + t2 and the respective derivatives */
public static native void cvComposeRT( @Const CvMat _rvec1, @Const CvMat _tvec1,
                         @Const CvMat _rvec2, @Const CvMat _tvec2,
                         CvMat _rvec3, CvMat _tvec3,
                         CvMat dr3dr1/*=0*/, CvMat dr3dt1/*=0*/,
                         CvMat dr3dr2/*=0*/, CvMat dr3dt2/*=0*/,
                         CvMat dt3dr1/*=0*/, CvMat dt3dt1/*=0*/,
                         CvMat dt3dr2/*=0*/, CvMat dt3dt2/*=0*/ );
public static native void cvComposeRT( @Const CvMat _rvec1, @Const CvMat _tvec1,
                         @Const CvMat _rvec2, @Const CvMat _tvec2,
                         CvMat _rvec3, CvMat _tvec3 );

/* Projects object points to the view plane using
   the specified extrinsic and intrinsic camera parameters */
public static native void cvProjectPoints2( @Const CvMat object_points, @Const CvMat rotation_vector,
                              @Const CvMat translation_vector, @Const CvMat camera_matrix,
                              @Const CvMat distortion_coeffs, CvMat image_points,
                              CvMat dpdrot/*=NULL*/, CvMat dpdt/*=NULL*/,
                              CvMat dpdf/*=NULL*/, CvMat dpdc/*=NULL*/,
                              CvMat dpddist/*=NULL*/,
                              double aspect_ratio/*=0*/);
public static native void cvProjectPoints2( @Const CvMat object_points, @Const CvMat rotation_vector,
                              @Const CvMat translation_vector, @Const CvMat camera_matrix,
                              @Const CvMat distortion_coeffs, CvMat image_points);

/* Finds extrinsic camera parameters from
   a few known corresponding point pairs and intrinsic parameters */
public static native void cvFindExtrinsicCameraParams2( @Const CvMat object_points,
                                          @Const CvMat image_points,
                                          @Const CvMat camera_matrix,
                                          @Const CvMat distortion_coeffs,
                                          CvMat rotation_vector,
                                          CvMat translation_vector,
                                          int use_extrinsic_guess/*=0*/ );
public static native void cvFindExtrinsicCameraParams2( @Const CvMat object_points,
                                          @Const CvMat image_points,
                                          @Const CvMat camera_matrix,
                                          @Const CvMat distortion_coeffs,
                                          CvMat rotation_vector,
                                          CvMat translation_vector );

/* Computes initial estimate of the intrinsic camera parameters
   in case of planar calibration target (e.g. chessboard) */
public static native void cvInitIntrinsicParams2D( @Const CvMat object_points,
                                     @Const CvMat image_points,
                                     @Const CvMat npoints, @ByVal CvSize image_size,
                                     CvMat camera_matrix,
                                     double aspect_ratio/*=1.*/ );
public static native void cvInitIntrinsicParams2D( @Const CvMat object_points,
                                     @Const CvMat image_points,
                                     @Const CvMat npoints, @ByVal CvSize image_size,
                                     CvMat camera_matrix );

public static final int CV_CALIB_CB_ADAPTIVE_THRESH =  1;
public static final int CV_CALIB_CB_NORMALIZE_IMAGE =  2;
public static final int CV_CALIB_CB_FILTER_QUADS =     4;
public static final int CV_CALIB_CB_FAST_CHECK =       8;

// Performs a fast check if a chessboard is in the input image. This is a workaround to
// a problem of cvFindChessboardCorners being slow on images with no chessboard
// - src: input image
// - size: chessboard size
// Returns 1 if a chessboard can be in this image and findChessboardCorners should be called,
// 0 if there is no chessboard, -1 in case of error
public static native int cvCheckChessboard(IplImage src, @ByVal CvSize size);

    /* Detects corners on a chessboard calibration pattern */
public static native int cvFindChessboardCorners( @Const Pointer image, @ByVal CvSize pattern_size,
                                    CvPoint2D32f corners,
                                    IntPointer corner_count/*=NULL*/,
                                    int flags/*=CV_CALIB_CB_ADAPTIVE_THRESH+CV_CALIB_CB_NORMALIZE_IMAGE*/ );
public static native int cvFindChessboardCorners( @Const Pointer image, @ByVal CvSize pattern_size,
                                    CvPoint2D32f corners );
public static native int cvFindChessboardCorners( @Const Pointer image, @ByVal CvSize pattern_size,
                                    @Cast("CvPoint2D32f*") FloatBuffer corners,
                                    IntBuffer corner_count/*=NULL*/,
                                    int flags/*=CV_CALIB_CB_ADAPTIVE_THRESH+CV_CALIB_CB_NORMALIZE_IMAGE*/ );
public static native int cvFindChessboardCorners( @Const Pointer image, @ByVal CvSize pattern_size,
                                    @Cast("CvPoint2D32f*") FloatBuffer corners );
public static native int cvFindChessboardCorners( @Const Pointer image, @ByVal CvSize pattern_size,
                                    @Cast("CvPoint2D32f*") float[] corners,
                                    int[] corner_count/*=NULL*/,
                                    int flags/*=CV_CALIB_CB_ADAPTIVE_THRESH+CV_CALIB_CB_NORMALIZE_IMAGE*/ );
public static native int cvFindChessboardCorners( @Const Pointer image, @ByVal CvSize pattern_size,
                                    @Cast("CvPoint2D32f*") float[] corners );

/* Draws individual chessboard corners or the whole chessboard detected */
public static native void cvDrawChessboardCorners( CvArr image, @ByVal CvSize pattern_size,
                                     CvPoint2D32f corners,
                                     int count, int pattern_was_found );
public static native void cvDrawChessboardCorners( CvArr image, @ByVal CvSize pattern_size,
                                     @Cast("CvPoint2D32f*") FloatBuffer corners,
                                     int count, int pattern_was_found );
public static native void cvDrawChessboardCorners( CvArr image, @ByVal CvSize pattern_size,
                                     @Cast("CvPoint2D32f*") float[] corners,
                                     int count, int pattern_was_found );

public static final int CV_CALIB_USE_INTRINSIC_GUESS =  1;
public static final int CV_CALIB_FIX_ASPECT_RATIO =     2;
public static final int CV_CALIB_FIX_PRINCIPAL_POINT =  4;
public static final int CV_CALIB_ZERO_TANGENT_DIST =    8;
public static final int CV_CALIB_FIX_FOCAL_LENGTH = 16;
public static final int CV_CALIB_FIX_K1 =  32;
public static final int CV_CALIB_FIX_K2 =  64;
public static final int CV_CALIB_FIX_K3 =  128;
public static final int CV_CALIB_FIX_K4 =  2048;
public static final int CV_CALIB_FIX_K5 =  4096;
public static final int CV_CALIB_FIX_K6 =  8192;
public static final int CV_CALIB_RATIONAL_MODEL = 16384;
public static final int CV_CALIB_THIN_PRISM_MODEL = 32768;
public static final int CV_CALIB_FIX_S1_S2_S3_S4 =  65536;


/* Finds intrinsic and extrinsic camera parameters
   from a few views of known calibration pattern */
public static native double cvCalibrateCamera2( @Const CvMat object_points,
                                @Const CvMat image_points,
                                @Const CvMat point_counts,
                                @ByVal CvSize image_size,
                                CvMat camera_matrix,
                                CvMat distortion_coeffs,
                                CvMat rotation_vectors/*=NULL*/,
                                CvMat translation_vectors/*=NULL*/,
                                int flags/*=0*/,
                                @ByVal CvTermCriteria term_crit/*=cvTermCriteria(
                                    CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30,DBL_EPSILON)*/ );
public static native double cvCalibrateCamera2( @Const CvMat object_points,
                                @Const CvMat image_points,
                                @Const CvMat point_counts,
                                @ByVal CvSize image_size,
                                CvMat camera_matrix,
                                CvMat distortion_coeffs );

/* Computes various useful characteristics of the camera from the data computed by
   cvCalibrateCamera2 */
public static native void cvCalibrationMatrixValues( @Const CvMat camera_matrix,
                                @ByVal CvSize image_size,
                                double aperture_width/*=0*/,
                                double aperture_height/*=0*/,
                                DoublePointer fovx/*=NULL*/,
                                DoublePointer fovy/*=NULL*/,
                                DoublePointer focal_length/*=NULL*/,
                                CvPoint2D64f principal_point/*=NULL*/,
                                DoublePointer pixel_aspect_ratio/*=NULL*/);
public static native void cvCalibrationMatrixValues( @Const CvMat camera_matrix,
                                @ByVal CvSize image_size);
public static native void cvCalibrationMatrixValues( @Const CvMat camera_matrix,
                                @ByVal CvSize image_size,
                                double aperture_width/*=0*/,
                                double aperture_height/*=0*/,
                                DoubleBuffer fovx/*=NULL*/,
                                DoubleBuffer fovy/*=NULL*/,
                                DoubleBuffer focal_length/*=NULL*/,
                                @Cast("CvPoint2D64f*") DoubleBuffer principal_point/*=NULL*/,
                                DoubleBuffer pixel_aspect_ratio/*=NULL*/);
public static native void cvCalibrationMatrixValues( @Const CvMat camera_matrix,
                                @ByVal CvSize image_size,
                                double aperture_width/*=0*/,
                                double aperture_height/*=0*/,
                                double[] fovx/*=NULL*/,
                                double[] fovy/*=NULL*/,
                                double[] focal_length/*=NULL*/,
                                @Cast("CvPoint2D64f*") double[] principal_point/*=NULL*/,
                                double[] pixel_aspect_ratio/*=NULL*/);

public static final int CV_CALIB_FIX_INTRINSIC =  256;
public static final int CV_CALIB_SAME_FOCAL_LENGTH = 512;

/* Computes the transformation from one camera coordinate system to another one
   from a few correspondent views of the same calibration target. Optionally, calibrates
   both cameras */
public static native double cvStereoCalibrate( @Const CvMat object_points, @Const CvMat image_points1,
                               @Const CvMat image_points2, @Const CvMat npoints,
                               CvMat camera_matrix1, CvMat dist_coeffs1,
                               CvMat camera_matrix2, CvMat dist_coeffs2,
                               @ByVal CvSize image_size, CvMat R, CvMat T,
                               CvMat E/*=0*/, CvMat F/*=0*/,
                               int flags/*=CV_CALIB_FIX_INTRINSIC*/,
                               @ByVal CvTermCriteria term_crit/*=cvTermCriteria(
                                   CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30,1e-6)*/ );
public static native double cvStereoCalibrate( @Const CvMat object_points, @Const CvMat image_points1,
                               @Const CvMat image_points2, @Const CvMat npoints,
                               CvMat camera_matrix1, CvMat dist_coeffs1,
                               CvMat camera_matrix2, CvMat dist_coeffs2,
                               @ByVal CvSize image_size, CvMat R, CvMat T );

public static final int CV_CALIB_ZERO_DISPARITY = 1024;

/* Computes 3D rotations (+ optional shift) for each camera coordinate system to make both
   views parallel (=> to make all the epipolar lines horizontal or vertical) */
public static native void cvStereoRectify( @Const CvMat camera_matrix1, @Const CvMat camera_matrix2,
                             @Const CvMat dist_coeffs1, @Const CvMat dist_coeffs2,
                             @ByVal CvSize image_size, @Const CvMat R, @Const CvMat T,
                             CvMat R1, CvMat R2, CvMat P1, CvMat P2,
                             CvMat Q/*=0*/,
                             int flags/*=CV_CALIB_ZERO_DISPARITY*/,
                             double alpha/*=-1*/,
                             @ByVal CvSize new_image_size/*=cvSize(0,0)*/,
                             CvRect valid_pix_ROI1/*=0*/,
                             CvRect valid_pix_ROI2/*=0*/);
public static native void cvStereoRectify( @Const CvMat camera_matrix1, @Const CvMat camera_matrix2,
                             @Const CvMat dist_coeffs1, @Const CvMat dist_coeffs2,
                             @ByVal CvSize image_size, @Const CvMat R, @Const CvMat T,
                             CvMat R1, CvMat R2, CvMat P1, CvMat P2);

/* Computes rectification transformations for uncalibrated pair of images using a set
   of point correspondences */
public static native int cvStereoRectifyUncalibrated( @Const CvMat points1, @Const CvMat points2,
                                        @Const CvMat F, @ByVal CvSize img_size,
                                        CvMat H1, CvMat H2,
                                        double threshold/*=5*/);
public static native int cvStereoRectifyUncalibrated( @Const CvMat points1, @Const CvMat points2,
                                        @Const CvMat F, @ByVal CvSize img_size,
                                        CvMat H1, CvMat H2);



/* stereo correspondence parameters and functions */

public static final int CV_STEREO_BM_NORMALIZED_RESPONSE =  0;
public static final int CV_STEREO_BM_XSOBEL =               1;

/* Block matching algorithm structure */
public static class CvStereoBMState extends AbstractCvStereoBMState {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvStereoBMState() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvStereoBMState(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvStereoBMState(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvStereoBMState position(int position) {
        return (CvStereoBMState)super.position(position);
    }

    // pre-filtering (normalization of input images)
    public native int preFilterType(); public native CvStereoBMState preFilterType(int preFilterType); // =CV_STEREO_BM_NORMALIZED_RESPONSE now
    public native int preFilterSize(); public native CvStereoBMState preFilterSize(int preFilterSize); // averaging window size: ~5x5..21x21
    public native int preFilterCap(); public native CvStereoBMState preFilterCap(int preFilterCap); // the output of pre-filtering is clipped by [-preFilterCap,preFilterCap]

    // correspondence using Sum of Absolute Difference (SAD)
    public native int SADWindowSize(); public native CvStereoBMState SADWindowSize(int SADWindowSize); // ~5x5..21x21
    public native int minDisparity(); public native CvStereoBMState minDisparity(int minDisparity);  // minimum disparity (can be negative)
    public native int numberOfDisparities(); public native CvStereoBMState numberOfDisparities(int numberOfDisparities); // maximum disparity - minimum disparity (> 0)

    // post-filtering
    public native int textureThreshold(); public native CvStereoBMState textureThreshold(int textureThreshold);  // the disparity is only computed for pixels
                           // with textured enough neighborhood
    public native int uniquenessRatio(); public native CvStereoBMState uniquenessRatio(int uniquenessRatio);   // accept the computed disparity d* only if
                           // SAD(d) >= SAD(d*)*(1 + uniquenessRatio/100.)
                           // for any d != d*+/-1 within the search range.
    public native int speckleWindowSize(); public native CvStereoBMState speckleWindowSize(int speckleWindowSize); // disparity variation window
    public native int speckleRange(); public native CvStereoBMState speckleRange(int speckleRange); // acceptable range of variation in window

    public native int trySmallerWindows(); public native CvStereoBMState trySmallerWindows(int trySmallerWindows); // if 1, the results may be more accurate,
                           // at the expense of slower processing
    public native @ByRef CvRect roi1(); public native CvStereoBMState roi1(CvRect roi1);
    public native @ByRef CvRect roi2(); public native CvStereoBMState roi2(CvRect roi2);
    public native int disp12MaxDiff(); public native CvStereoBMState disp12MaxDiff(int disp12MaxDiff);

    // temporary buffers
    public native CvMat preFilteredImg0(); public native CvStereoBMState preFilteredImg0(CvMat preFilteredImg0);
    public native CvMat preFilteredImg1(); public native CvStereoBMState preFilteredImg1(CvMat preFilteredImg1);
    public native CvMat slidingSumBuf(); public native CvStereoBMState slidingSumBuf(CvMat slidingSumBuf);
    public native CvMat cost(); public native CvStereoBMState cost(CvMat cost);
    public native CvMat disp(); public native CvStereoBMState disp(CvMat disp);
}

public static final int CV_STEREO_BM_BASIC = 0;
public static final int CV_STEREO_BM_FISH_EYE = 1;
public static final int CV_STEREO_BM_NARROW = 2;

public static native CvStereoBMState cvCreateStereoBMState(int preset/*=CV_STEREO_BM_BASIC*/,
                                              int numberOfDisparities/*=0*/);
public static native CvStereoBMState cvCreateStereoBMState();

public static native void cvReleaseStereoBMState( @Cast("CvStereoBMState**") PointerPointer state );
public static native void cvReleaseStereoBMState( @ByPtrPtr CvStereoBMState state );

public static native void cvFindStereoCorrespondenceBM( @Const CvArr left, @Const CvArr right,
                                          CvArr disparity, CvStereoBMState state );

public static native @ByVal CvRect cvGetValidDisparityROI( @ByVal CvRect roi1, @ByVal CvRect roi2, int minDisparity,
                                      int numberOfDisparities, int SADWindowSize );

public static native void cvValidateDisparity( CvArr disparity, @Const CvArr cost,
                                 int minDisparity, int numberOfDisparities,
                                 int disp12MaxDiff/*=1*/ );
public static native void cvValidateDisparity( CvArr disparity, @Const CvArr cost,
                                 int minDisparity, int numberOfDisparities );

/* Reprojects the computed disparity image to the 3D space using the specified 4x4 matrix */
public static native void cvReprojectImageTo3D( @Const CvArr disparityImage,
                                   CvArr _3dImage, @Const CvMat Q,
                                   int handleMissingValues/*=0*/ );
public static native void cvReprojectImageTo3D( @Const CvArr disparityImage,
                                   CvArr _3dImage, @Const CvMat Q );

// #ifdef __cplusplus // extern "C"

//////////////////////////////////////////////////////////////////////////////////////////
@NoOffset public static class CvLevMarq extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvLevMarq(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvLevMarq(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public CvLevMarq position(int position) {
        return (CvLevMarq)super.position(position);
    }

    public CvLevMarq() { allocate(); }
    private native void allocate();
    public CvLevMarq( int nparams, int nerrs, @ByVal CvTermCriteria criteria/*=cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,DBL_EPSILON)*/,
                  @Cast("bool") boolean completeSymmFlag/*=false*/ ) { allocate(nparams, nerrs, criteria, completeSymmFlag); }
    private native void allocate( int nparams, int nerrs, @ByVal CvTermCriteria criteria/*=cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,DBL_EPSILON)*/,
                  @Cast("bool") boolean completeSymmFlag/*=false*/ );
    public CvLevMarq( int nparams, int nerrs ) { allocate(nparams, nerrs); }
    private native void allocate( int nparams, int nerrs );
    public native void init( int nparams, int nerrs, @ByVal CvTermCriteria criteria/*=cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,DBL_EPSILON)*/,
                  @Cast("bool") boolean completeSymmFlag/*=false*/ );
    public native void init( int nparams, int nerrs );
    public native @Cast("bool") boolean update( @Const @ByPtrRef CvMat param, @ByPtrRef CvMat J, @ByPtrRef CvMat err );
    public native @Cast("bool") boolean updateAlt( @Const @ByPtrRef CvMat param, @ByPtrRef CvMat JtJ, @ByPtrRef CvMat JtErr, @ByPtrRef DoublePointer errNorm );
    public native @Cast("bool") boolean updateAlt( @Const @ByPtrRef CvMat param, @ByPtrRef CvMat JtJ, @ByPtrRef CvMat JtErr, @ByPtrRef DoubleBuffer errNorm );
    public native @Cast("bool") boolean updateAlt( @Const @ByPtrRef CvMat param, @ByPtrRef CvMat JtJ, @ByPtrRef CvMat JtErr, @ByPtrRef double[] errNorm );

    public native void clear();
    public native void step();
    /** enum CvLevMarq:: */
    public static final int DONE= 0, STARTED= 1, CALC_J= 2, CHECK_ERR= 3;

    public native @Ptr CvMat mask(); public native CvLevMarq mask(CvMat mask);
    public native @Ptr CvMat prevParam(); public native CvLevMarq prevParam(CvMat prevParam);
    public native @Ptr CvMat param(); public native CvLevMarq param(CvMat param);
    public native @Ptr CvMat J(); public native CvLevMarq J(CvMat J);
    public native @Ptr CvMat err(); public native CvLevMarq err(CvMat err);
    public native @Ptr CvMat JtJ(); public native CvLevMarq JtJ(CvMat JtJ);
    public native @Ptr CvMat JtJN(); public native CvLevMarq JtJN(CvMat JtJN);
    public native @Ptr CvMat JtErr(); public native CvLevMarq JtErr(CvMat JtErr);
    public native @Ptr CvMat JtJV(); public native CvLevMarq JtJV(CvMat JtJV);
    public native @Ptr CvMat JtJW(); public native CvLevMarq JtJW(CvMat JtJW);
    public native double prevErrNorm(); public native CvLevMarq prevErrNorm(double prevErrNorm);
    public native double errNorm(); public native CvLevMarq errNorm(double errNorm);
    public native int lambdaLg10(); public native CvLevMarq lambdaLg10(int lambdaLg10);
    public native @ByRef CvTermCriteria criteria(); public native CvLevMarq criteria(CvTermCriteria criteria);
    public native int state(); public native CvLevMarq state(int state);
    public native int iters(); public native CvLevMarq iters(int iters);
    public native @Cast("bool") boolean completeSymmFlag(); public native CvLevMarq completeSymmFlag(boolean completeSymmFlag);
}

// #endif

// #endif /* __OPENCV_CALIB3D_C_H__ */


// Parsed from <opencv2/calib3d.hpp>

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

// #ifndef __OPENCV_CALIB3D_HPP__
// #define __OPENCV_CALIB3D_HPP__

// #include "opencv2/core.hpp"
// #include "opencv2/features2d.hpp"
// #include "opencv2/core/affine.hpp"

/** type of the robust estimation algorithm */
/** enum cv:: */
public static final int /** least-median algorithm */
 LMEDS  = 4,
       /** RANSAC algorithm */
       RANSAC = 8;

/** enum cv:: */
public static final int SOLVEPNP_ITERATIVE = 0,
       SOLVEPNP_EPNP      = 1, // F.Moreno-Noguer, V.Lepetit and P.Fua "EPnP: Efficient Perspective-n-Point Camera Pose Estimation"
       SOLVEPNP_P3P       = 2, // X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang; "Complete Solution Classification for the Perspective-Three-Point Problem"
       SOLVEPNP_DLS       = 3, // Joel A. Hesch and Stergios I. Roumeliotis. "A Direct Least-Squares (DLS) Method for PnP"
       SOLVEPNP_UPNP      = 4;  // A.Penate-Sanchez, J.Andrade-Cetto, F.Moreno-Noguer. "Exhaustive Linearization for Robust Camera Pose and Focal Length Estimation"

/** enum cv:: */
public static final int CALIB_CB_ADAPTIVE_THRESH = 1,
       CALIB_CB_NORMALIZE_IMAGE = 2,
       CALIB_CB_FILTER_QUADS    = 4,
       CALIB_CB_FAST_CHECK      = 8;

/** enum cv:: */
public static final int CALIB_CB_SYMMETRIC_GRID  = 1,
       CALIB_CB_ASYMMETRIC_GRID = 2,
       CALIB_CB_CLUSTERING      = 4;

/** enum cv:: */
public static final int CALIB_USE_INTRINSIC_GUESS =  0x00001,
       CALIB_FIX_ASPECT_RATIO    =  0x00002,
       CALIB_FIX_PRINCIPAL_POINT =  0x00004,
       CALIB_ZERO_TANGENT_DIST   =  0x00008,
       CALIB_FIX_FOCAL_LENGTH    =  0x00010,
       CALIB_FIX_K1              =  0x00020,
       CALIB_FIX_K2              =  0x00040,
       CALIB_FIX_K3              =  0x00080,
       CALIB_FIX_K4              =  0x00800,
       CALIB_FIX_K5              =  0x01000,
       CALIB_FIX_K6              =  0x02000,
       CALIB_RATIONAL_MODEL      =  0x04000,
       CALIB_THIN_PRISM_MODEL    =  0x08000,
       CALIB_FIX_S1_S2_S3_S4     =  0x10000,
       // only for stereo
       CALIB_FIX_INTRINSIC       =  0x00100,
       CALIB_SAME_FOCAL_LENGTH   =  0x00200,
       // for stereo rectification
       CALIB_ZERO_DISPARITY      =  0x00400;

/** the algorithm for finding fundamental matrix */
/** enum cv:: */
public static final int /** 7-point algorithm */
 FM_7POINT = 1,
       /** 8-point algorithm */
       FM_8POINT = 2,
       /** least-median algorithm */
       FM_LMEDS  = 4,
       /** RANSAC algorithm */
       FM_RANSAC = 8;



/** converts rotation vector to rotation matrix or vice versa using Rodrigues transformation */
@Namespace("cv") public static native void Rodrigues( @ByVal Mat src, @ByVal Mat dst, @ByVal Mat jacobian/*=noArray()*/ );
@Namespace("cv") public static native void Rodrigues( @ByVal Mat src, @ByVal Mat dst );

/** computes the best-fit perspective transformation mapping srcPoints to dstPoints. */
@Namespace("cv") public static native @ByVal Mat findHomography( @ByVal Mat srcPoints, @ByVal Mat dstPoints,
                                 int method/*=0*/, double ransacReprojThreshold/*=3*/,
                                 @ByVal Mat mask/*=noArray()*/, int maxIters/*=2000*/,
                                 double confidence/*=0.995*/);
@Namespace("cv") public static native @ByVal Mat findHomography( @ByVal Mat srcPoints, @ByVal Mat dstPoints);

/** variant of findHomography for backward compatibility */
@Namespace("cv") public static native @ByVal Mat findHomography( @ByVal Mat srcPoints, @ByVal Mat dstPoints,
                               @ByVal Mat mask, int method/*=0*/, double ransacReprojThreshold/*=3*/ );
@Namespace("cv") public static native @ByVal Mat findHomography( @ByVal Mat srcPoints, @ByVal Mat dstPoints,
                               @ByVal Mat mask );

/** Computes RQ decomposition of 3x3 matrix */
@Namespace("cv") public static native @ByVal Point3d RQDecomp3x3( @ByVal Mat src, @ByVal Mat mtxR, @ByVal Mat mtxQ,
                                @ByVal Mat Qx/*=noArray()*/,
                                @ByVal Mat Qy/*=noArray()*/,
                                @ByVal Mat Qz/*=noArray()*/);
@Namespace("cv") public static native @ByVal Point3d RQDecomp3x3( @ByVal Mat src, @ByVal Mat mtxR, @ByVal Mat mtxQ);

/** Decomposes the projection matrix into camera matrix and the rotation martix and the translation vector */
@Namespace("cv") public static native void decomposeProjectionMatrix( @ByVal Mat projMatrix, @ByVal Mat cameraMatrix,
                                             @ByVal Mat rotMatrix, @ByVal Mat transVect,
                                             @ByVal Mat rotMatrixX/*=noArray()*/,
                                             @ByVal Mat rotMatrixY/*=noArray()*/,
                                             @ByVal Mat rotMatrixZ/*=noArray()*/,
                                             @ByVal Mat eulerAngles/*=noArray()*/ );
@Namespace("cv") public static native void decomposeProjectionMatrix( @ByVal Mat projMatrix, @ByVal Mat cameraMatrix,
                                             @ByVal Mat rotMatrix, @ByVal Mat transVect );

/** computes derivatives of the matrix product w.r.t each of the multiplied matrix coefficients */
@Namespace("cv") public static native void matMulDeriv( @ByVal Mat A, @ByVal Mat B, @ByVal Mat dABdA, @ByVal Mat dABdB );

/** composes 2 [R|t] transformations together. Also computes the derivatives of the result w.r.t the arguments */
@Namespace("cv") public static native void composeRT( @ByVal Mat rvec1, @ByVal Mat tvec1,
                             @ByVal Mat rvec2, @ByVal Mat tvec2,
                             @ByVal Mat rvec3, @ByVal Mat tvec3,
                             @ByVal Mat dr3dr1/*=noArray()*/, @ByVal Mat dr3dt1/*=noArray()*/,
                             @ByVal Mat dr3dr2/*=noArray()*/, @ByVal Mat dr3dt2/*=noArray()*/,
                             @ByVal Mat dt3dr1/*=noArray()*/, @ByVal Mat dt3dt1/*=noArray()*/,
                             @ByVal Mat dt3dr2/*=noArray()*/, @ByVal Mat dt3dt2/*=noArray()*/ );
@Namespace("cv") public static native void composeRT( @ByVal Mat rvec1, @ByVal Mat tvec1,
                             @ByVal Mat rvec2, @ByVal Mat tvec2,
                             @ByVal Mat rvec3, @ByVal Mat tvec3 );

/** projects points from the model coordinate space to the image coordinates. Also computes derivatives of the image coordinates w.r.t the intrinsic and extrinsic camera parameters */
@Namespace("cv") public static native void projectPoints( @ByVal Mat objectPoints,
                                 @ByVal Mat rvec, @ByVal Mat tvec,
                                 @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
                                 @ByVal Mat imagePoints,
                                 @ByVal Mat jacobian/*=noArray()*/,
                                 double aspectRatio/*=0*/ );
@Namespace("cv") public static native void projectPoints( @ByVal Mat objectPoints,
                                 @ByVal Mat rvec, @ByVal Mat tvec,
                                 @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
                                 @ByVal Mat imagePoints );

/** computes the camera pose from a few 3D points and the corresponding projections. The outliers are not handled. */
@Namespace("cv") public static native @Cast("bool") boolean solvePnP( @ByVal Mat objectPoints, @ByVal Mat imagePoints,
                            @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
                            @ByVal Mat rvec, @ByVal Mat tvec,
                            @Cast("bool") boolean useExtrinsicGuess/*=false*/, int flags/*=SOLVEPNP_ITERATIVE*/ );
@Namespace("cv") public static native @Cast("bool") boolean solvePnP( @ByVal Mat objectPoints, @ByVal Mat imagePoints,
                            @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
                            @ByVal Mat rvec, @ByVal Mat tvec );

/** computes the camera pose from a few 3D points and the corresponding projections. The outliers are possible. */
@Namespace("cv") public static native @Cast("bool") boolean solvePnPRansac( @ByVal Mat objectPoints, @ByVal Mat imagePoints,
                                  @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
                                  @ByVal Mat rvec, @ByVal Mat tvec,
                                  @Cast("bool") boolean useExtrinsicGuess/*=false*/, int iterationsCount/*=100*/,
                                  float reprojectionError/*=8.0*/, double confidence/*=0.99*/,
                                  @ByVal Mat inliers/*=noArray()*/, int flags/*=SOLVEPNP_ITERATIVE*/ );
@Namespace("cv") public static native @Cast("bool") boolean solvePnPRansac( @ByVal Mat objectPoints, @ByVal Mat imagePoints,
                                  @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
                                  @ByVal Mat rvec, @ByVal Mat tvec );

/** initializes camera matrix from a few 3D points and the corresponding projections. */
@Namespace("cv") public static native @ByVal Mat initCameraMatrix2D( @ByVal MatVector objectPoints,
                                     @ByVal MatVector imagePoints,
                                     @ByVal Size imageSize, double aspectRatio/*=1.0*/ );
@Namespace("cv") public static native @ByVal Mat initCameraMatrix2D( @ByVal MatVector objectPoints,
                                     @ByVal MatVector imagePoints,
                                     @ByVal Size imageSize );

/** finds checkerboard pattern of the specified size in the image */
@Namespace("cv") public static native @Cast("bool") boolean findChessboardCorners( @ByVal Mat image, @ByVal Size patternSize, @ByVal Mat corners,
                                         int flags/*=CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE*/ );
@Namespace("cv") public static native @Cast("bool") boolean findChessboardCorners( @ByVal Mat image, @ByVal Size patternSize, @ByVal Mat corners );

/** finds subpixel-accurate positions of the chessboard corners */
@Namespace("cv") public static native @Cast("bool") boolean find4QuadCornerSubpix( @ByVal Mat img, @ByVal Mat corners, @ByVal Size region_size );

/** draws the checkerboard pattern (found or partly found) in the image */
@Namespace("cv") public static native void drawChessboardCorners( @ByVal Mat image, @ByVal Size patternSize,
                                         @ByVal Mat corners, @Cast("bool") boolean patternWasFound );

/** finds circles' grid pattern of the specified size in the image */
@Namespace("cv") public static native @Cast("bool") boolean findCirclesGrid( @ByVal Mat image, @ByVal Size patternSize,
                                   @ByVal Mat centers, int flags/*=CALIB_CB_SYMMETRIC_GRID*/,
                                   @Cast("cv::FeatureDetector*") @Ptr Feature2D blobDetector/*=SimpleBlobDetector::create()*/);
@Namespace("cv") public static native @Cast("bool") boolean findCirclesGrid( @ByVal Mat image, @ByVal Size patternSize,
                                   @ByVal Mat centers);

/** finds intrinsic and extrinsic camera parameters from several fews of a known calibration pattern. */
@Namespace("cv") public static native double calibrateCamera( @ByVal MatVector objectPoints,
                                     @ByVal MatVector imagePoints, @ByVal Size imageSize,
                                     @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
                                     @ByVal MatVector rvecs, @ByVal MatVector tvecs,
                                     int flags/*=0*/, @ByVal TermCriteria criteria/*=TermCriteria(
                                        TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON)*/ );
@Namespace("cv") public static native double calibrateCamera( @ByVal MatVector objectPoints,
                                     @ByVal MatVector imagePoints, @ByVal Size imageSize,
                                     @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
                                     @ByVal MatVector rvecs, @ByVal MatVector tvecs );

/** computes several useful camera characteristics from the camera matrix, camera frame resolution and the physical sensor size. */
@Namespace("cv") public static native void calibrationMatrixValues( @ByVal Mat cameraMatrix, @ByVal Size imageSize,
                                           double apertureWidth, double apertureHeight,
                                           @ByRef DoublePointer fovx, @ByRef DoublePointer fovy,
                                           @ByRef DoublePointer focalLength, @ByRef Point2d principalPoint,
                                           @ByRef DoublePointer aspectRatio );
@Namespace("cv") public static native void calibrationMatrixValues( @ByVal Mat cameraMatrix, @ByVal Size imageSize,
                                           double apertureWidth, double apertureHeight,
                                           @ByRef DoubleBuffer fovx, @ByRef DoubleBuffer fovy,
                                           @ByRef DoubleBuffer focalLength, @ByRef Point2d principalPoint,
                                           @ByRef DoubleBuffer aspectRatio );
@Namespace("cv") public static native void calibrationMatrixValues( @ByVal Mat cameraMatrix, @ByVal Size imageSize,
                                           double apertureWidth, double apertureHeight,
                                           @ByRef double[] fovx, @ByRef double[] fovy,
                                           @ByRef double[] focalLength, @ByRef Point2d principalPoint,
                                           @ByRef double[] aspectRatio );

/** finds intrinsic and extrinsic parameters of a stereo camera */
@Namespace("cv") public static native double stereoCalibrate( @ByVal MatVector objectPoints,
                                     @ByVal MatVector imagePoints1, @ByVal MatVector imagePoints2,
                                     @ByVal Mat cameraMatrix1, @ByVal Mat distCoeffs1,
                                     @ByVal Mat cameraMatrix2, @ByVal Mat distCoeffs2,
                                     @ByVal Size imageSize, @ByVal Mat R,@ByVal Mat T, @ByVal Mat E, @ByVal Mat F,
                                     int flags/*=CALIB_FIX_INTRINSIC*/,
                                     @ByVal TermCriteria criteria/*=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6)*/ );
@Namespace("cv") public static native double stereoCalibrate( @ByVal MatVector objectPoints,
                                     @ByVal MatVector imagePoints1, @ByVal MatVector imagePoints2,
                                     @ByVal Mat cameraMatrix1, @ByVal Mat distCoeffs1,
                                     @ByVal Mat cameraMatrix2, @ByVal Mat distCoeffs2,
                                     @ByVal Size imageSize, @ByVal Mat R,@ByVal Mat T, @ByVal Mat E, @ByVal Mat F );


/** computes the rectification transformation for a stereo camera from its intrinsic and extrinsic parameters */
@Namespace("cv") public static native void stereoRectify( @ByVal Mat cameraMatrix1, @ByVal Mat distCoeffs1,
                                 @ByVal Mat cameraMatrix2, @ByVal Mat distCoeffs2,
                                 @ByVal Size imageSize, @ByVal Mat R, @ByVal Mat T,
                                 @ByVal Mat R1, @ByVal Mat R2,
                                 @ByVal Mat P1, @ByVal Mat P2,
                                 @ByVal Mat Q, int flags/*=CALIB_ZERO_DISPARITY*/,
                                 double alpha/*=-1*/, @ByVal Size newImageSize/*=Size()*/,
                                 Rect validPixROI1/*=0*/, Rect validPixROI2/*=0*/ );
@Namespace("cv") public static native void stereoRectify( @ByVal Mat cameraMatrix1, @ByVal Mat distCoeffs1,
                                 @ByVal Mat cameraMatrix2, @ByVal Mat distCoeffs2,
                                 @ByVal Size imageSize, @ByVal Mat R, @ByVal Mat T,
                                 @ByVal Mat R1, @ByVal Mat R2,
                                 @ByVal Mat P1, @ByVal Mat P2,
                                 @ByVal Mat Q );

/** computes the rectification transformation for an uncalibrated stereo camera (zero distortion is assumed) */
@Namespace("cv") public static native @Cast("bool") boolean stereoRectifyUncalibrated( @ByVal Mat points1, @ByVal Mat points2,
                                             @ByVal Mat F, @ByVal Size imgSize,
                                             @ByVal Mat H1, @ByVal Mat H2,
                                             double threshold/*=5*/ );
@Namespace("cv") public static native @Cast("bool") boolean stereoRectifyUncalibrated( @ByVal Mat points1, @ByVal Mat points2,
                                             @ByVal Mat F, @ByVal Size imgSize,
                                             @ByVal Mat H1, @ByVal Mat H2 );

/** computes the rectification transformations for 3-head camera, where all the heads are on the same line. */
@Namespace("cv") public static native float rectify3Collinear( @ByVal Mat cameraMatrix1, @ByVal Mat distCoeffs1,
                                      @ByVal Mat cameraMatrix2, @ByVal Mat distCoeffs2,
                                      @ByVal Mat cameraMatrix3, @ByVal Mat distCoeffs3,
                                      @ByVal MatVector imgpt1, @ByVal MatVector imgpt3,
                                      @ByVal Size imageSize, @ByVal Mat R12, @ByVal Mat T12,
                                      @ByVal Mat R13, @ByVal Mat T13,
                                      @ByVal Mat R1, @ByVal Mat R2, @ByVal Mat R3,
                                      @ByVal Mat P1, @ByVal Mat P2, @ByVal Mat P3,
                                      @ByVal Mat Q, double alpha, @ByVal Size newImgSize,
                                      Rect roi1, Rect roi2, int flags );

/** returns the optimal new camera matrix */
@Namespace("cv") public static native @ByVal Mat getOptimalNewCameraMatrix( @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
                                            @ByVal Size imageSize, double alpha, @ByVal Size newImgSize/*=Size()*/,
                                            Rect validPixROI/*=0*/,
                                            @Cast("bool") boolean centerPrincipalPoint/*=false*/);
@Namespace("cv") public static native @ByVal Mat getOptimalNewCameraMatrix( @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
                                            @ByVal Size imageSize, double alpha);

/** converts point coordinates from normal pixel coordinates to homogeneous coordinates ((x,y)->(x,y,1)) */
@Namespace("cv") public static native void convertPointsToHomogeneous( @ByVal Mat src, @ByVal Mat dst );

/** converts point coordinates from homogeneous to normal pixel coordinates ((x,y,z)->(x/z, y/z)) */
@Namespace("cv") public static native void convertPointsFromHomogeneous( @ByVal Mat src, @ByVal Mat dst );

/** for backward compatibility */
@Namespace("cv") public static native void convertPointsHomogeneous( @ByVal Mat src, @ByVal Mat dst );

/** finds fundamental matrix from a set of corresponding 2D points */
@Namespace("cv") public static native @ByVal Mat findFundamentalMat( @ByVal Mat points1, @ByVal Mat points2,
                                     int method/*=FM_RANSAC*/,
                                     double param1/*=3.*/, double param2/*=0.99*/,
                                     @ByVal Mat mask/*=noArray()*/ );
@Namespace("cv") public static native @ByVal Mat findFundamentalMat( @ByVal Mat points1, @ByVal Mat points2 );

/** variant of findFundamentalMat for backward compatibility */
@Namespace("cv") public static native @ByVal Mat findFundamentalMat( @ByVal Mat points1, @ByVal Mat points2,
                                   @ByVal Mat mask, int method/*=FM_RANSAC*/,
                                   double param1/*=3.*/, double param2/*=0.99*/ );
@Namespace("cv") public static native @ByVal Mat findFundamentalMat( @ByVal Mat points1, @ByVal Mat points2,
                                   @ByVal Mat mask );

/** finds essential matrix from a set of corresponding 2D points using five-point algorithm */
@Namespace("cv") public static native @ByVal Mat findEssentialMat( @ByVal Mat points1, @ByVal Mat points2,
                                 double focal/*=1.0*/, @ByVal Point2d pp/*=Point2d(0, 0)*/,
                                 int method/*=RANSAC*/, double prob/*=0.999*/,
                                 double threshold/*=1.0*/, @ByVal Mat mask/*=noArray()*/ );
@Namespace("cv") public static native @ByVal Mat findEssentialMat( @ByVal Mat points1, @ByVal Mat points2 );

/** decompose essential matrix to possible rotation matrix and one translation vector */
@Namespace("cv") public static native void decomposeEssentialMat( @ByVal Mat E, @ByVal Mat R1, @ByVal Mat R2, @ByVal Mat t );

/** recover relative camera pose from a set of corresponding 2D points */
@Namespace("cv") public static native int recoverPose( @ByVal Mat E, @ByVal Mat points1, @ByVal Mat points2,
                            @ByVal Mat R, @ByVal Mat t,
                            double focal/*=1.0*/, @ByVal Point2d pp/*=Point2d(0, 0)*/,
                            @ByVal Mat mask/*=noArray()*/ );
@Namespace("cv") public static native int recoverPose( @ByVal Mat E, @ByVal Mat points1, @ByVal Mat points2,
                            @ByVal Mat R, @ByVal Mat t );


/** finds coordinates of epipolar lines corresponding the specified points */
@Namespace("cv") public static native void computeCorrespondEpilines( @ByVal Mat points, int whichImage,
                                             @ByVal Mat F, @ByVal Mat lines );

@Namespace("cv") public static native void triangulatePoints( @ByVal Mat projMatr1, @ByVal Mat projMatr2,
                                     @ByVal Mat projPoints1, @ByVal Mat projPoints2,
                                     @ByVal Mat points4D );

@Namespace("cv") public static native void correctMatches( @ByVal Mat F, @ByVal Mat points1, @ByVal Mat points2,
                                  @ByVal Mat newPoints1, @ByVal Mat newPoints2 );

/** filters off speckles (small regions of incorrectly computed disparity) */
@Namespace("cv") public static native void filterSpeckles( @ByVal Mat img, double newVal,
                                  int maxSpeckleSize, double maxDiff,
                                  @ByVal Mat buf/*=noArray()*/ );
@Namespace("cv") public static native void filterSpeckles( @ByVal Mat img, double newVal,
                                  int maxSpeckleSize, double maxDiff );

/** computes valid disparity ROI from the valid ROIs of the rectified images (that are returned by cv::stereoRectify()) */
@Namespace("cv") public static native @ByVal Rect getValidDisparityROI( @ByVal Rect roi1, @ByVal Rect roi2,
                                        int minDisparity, int numberOfDisparities,
                                        int SADWindowSize );

/** validates disparity using the left-right check. The matrix "cost" should be computed by the stereo correspondence algorithm */
@Namespace("cv") public static native void validateDisparity( @ByVal Mat disparity, @ByVal Mat cost,
                                     int minDisparity, int numberOfDisparities,
                                     int disp12MaxDisp/*=1*/ );
@Namespace("cv") public static native void validateDisparity( @ByVal Mat disparity, @ByVal Mat cost,
                                     int minDisparity, int numberOfDisparities );

/** reprojects disparity image to 3D: (x,y,d)->(X,Y,Z) using the matrix Q returned by cv::stereoRectify */
@Namespace("cv") public static native void reprojectImageTo3D( @ByVal Mat disparity,
                                      @ByVal Mat _3dImage, @ByVal Mat Q,
                                      @Cast("bool") boolean handleMissingValues/*=false*/,
                                      int ddepth/*=-1*/ );
@Namespace("cv") public static native void reprojectImageTo3D( @ByVal Mat disparity,
                                      @ByVal Mat _3dImage, @ByVal Mat Q );

@Namespace("cv") public static native int estimateAffine3D(@ByVal Mat src, @ByVal Mat dst,
                                   @ByVal Mat out, @ByVal Mat inliers,
                                   double ransacThreshold/*=3*/, double confidence/*=0.99*/);
@Namespace("cv") public static native int estimateAffine3D(@ByVal Mat src, @ByVal Mat dst,
                                   @ByVal Mat out, @ByVal Mat inliers);


@Namespace("cv") public static native int decomposeHomographyMat(@ByVal Mat H,
                                        @ByVal Mat K,
                                        @ByVal MatVector rotations,
                                        @ByVal MatVector translations,
                                        @ByVal MatVector normals);

@Namespace("cv") public static class StereoMatcher extends Algorithm {
    static { Loader.load(); }
    /** Empty constructor. */
    public StereoMatcher() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public StereoMatcher(Pointer p) { super(p); }

    /** enum cv::StereoMatcher:: */
    public static final int DISP_SHIFT = 4,
           DISP_SCALE =  (1 << DISP_SHIFT);

    public native void compute( @ByVal Mat left, @ByVal Mat right,
                                      @ByVal Mat disparity );

    public native int getMinDisparity();
    public native void setMinDisparity(int minDisparity);

    public native int getNumDisparities();
    public native void setNumDisparities(int numDisparities);

    public native int getBlockSize();
    public native void setBlockSize(int blockSize);

    public native int getSpeckleWindowSize();
    public native void setSpeckleWindowSize(int speckleWindowSize);

    public native int getSpeckleRange();
    public native void setSpeckleRange(int speckleRange);

    public native int getDisp12MaxDiff();
    public native void setDisp12MaxDiff(int disp12MaxDiff);
}



@Namespace("cv") public static class StereoBM extends StereoMatcher {
    static { Loader.load(); }
    /** Empty constructor. */
    public StereoBM() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public StereoBM(Pointer p) { super(p); }

    /** enum cv::StereoBM:: */
    public static final int PREFILTER_NORMALIZED_RESPONSE = 0,
           PREFILTER_XSOBEL              = 1;

    public native int getPreFilterType();
    public native void setPreFilterType(int preFilterType);

    public native int getPreFilterSize();
    public native void setPreFilterSize(int preFilterSize);

    public native int getPreFilterCap();
    public native void setPreFilterCap(int preFilterCap);

    public native int getTextureThreshold();
    public native void setTextureThreshold(int textureThreshold);

    public native int getUniquenessRatio();
    public native void setUniquenessRatio(int uniquenessRatio);

    public native int getSmallerBlockSize();
    public native void setSmallerBlockSize(int blockSize);

    public native @ByVal Rect getROI1();
    public native void setROI1(@ByVal Rect roi1);

    public native @ByVal Rect getROI2();
    public native void setROI2(@ByVal Rect roi2);

    public static native @Ptr StereoBM create(int numDisparities/*=0*/, int blockSize/*=21*/);
    public static native @Ptr StereoBM create();
}


@Namespace("cv") public static class StereoSGBM extends StereoMatcher {
    static { Loader.load(); }
    /** Empty constructor. */
    public StereoSGBM() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public StereoSGBM(Pointer p) { super(p); }

    /** enum cv::StereoSGBM:: */
    public static final int
        MODE_SGBM = 0,
        MODE_HH   = 1;

    public native int getPreFilterCap();
    public native void setPreFilterCap(int preFilterCap);

    public native int getUniquenessRatio();
    public native void setUniquenessRatio(int uniquenessRatio);

    public native int getP1();
    public native void setP1(int P1);

    public native int getP2();
    public native void setP2(int P2);

    public native int getMode();
    public native void setMode(int mode);

    public static native @Ptr StereoSGBM create(int minDisparity, int numDisparities, int blockSize,
                                              int P1/*=0*/, int P2/*=0*/, int disp12MaxDiff/*=0*/,
                                              int preFilterCap/*=0*/, int uniquenessRatio/*=0*/,
                                              int speckleWindowSize/*=0*/, int speckleRange/*=0*/,
                                              int mode/*=StereoSGBM::MODE_SGBM*/);
    public static native @Ptr StereoSGBM create(int minDisparity, int numDisparities, int blockSize);
}
    /** enum cv::fisheye:: */
    public static final int
        FISHEYE_CALIB_USE_INTRINSIC_GUESS   = 1,
        FISHEYE_CALIB_RECOMPUTE_EXTRINSIC   = 2,
        FISHEYE_CALIB_CHECK_COND            = 4,
        FISHEYE_CALIB_FIX_SKEW              = 8,
        FISHEYE_CALIB_FIX_K1                = 16,
        FISHEYE_CALIB_FIX_K2                = 32,
        FISHEYE_CALIB_FIX_K3                = 64,
        FISHEYE_CALIB_FIX_K4                = 128,
        FISHEYE_CALIB_FIX_INTRINSIC         = 256;

    /** projects 3D points using fisheye model */
    @Namespace("cv::fisheye") public static native void projectPoints(@ByVal Mat objectPoints, @ByVal Mat imagePoints, @Const @ByRef Mat affine,
            @ByVal Mat K, @ByVal Mat D, double alpha/*=0*/, @ByVal Mat jacobian/*=noArray()*/);
    @Namespace("cv::fisheye") public static native void projectPoints(@ByVal Mat objectPoints, @ByVal Mat imagePoints, @Const @ByRef Mat affine,
            @ByVal Mat K, @ByVal Mat D);

    /** projects points using fisheye model */
    @Namespace("cv::fisheye") public static native void projectPoints(@ByVal Mat objectPoints, @ByVal Mat imagePoints, @ByVal Mat rvec, @ByVal Mat tvec,
            @ByVal Mat K, @ByVal Mat D, double alpha/*=0*/, @ByVal Mat jacobian/*=noArray()*/);

    /** distorts 2D points using fisheye model */
    @Namespace("cv::fisheye") public static native void distortPoints(@ByVal Mat undistorted, @ByVal Mat distorted, @ByVal Mat K, @ByVal Mat D, double alpha/*=0*/);
    @Namespace("cv::fisheye") public static native void distortPoints(@ByVal Mat undistorted, @ByVal Mat distorted, @ByVal Mat K, @ByVal Mat D);

    /** undistorts 2D points using fisheye model */
    @Namespace("cv::fisheye") public static native void undistortPoints(@ByVal Mat distorted, @ByVal Mat undistorted,
            @ByVal Mat K, @ByVal Mat D, @ByVal Mat R/*=noArray()*/, @ByVal Mat P/*=noArray()*/);
    @Namespace("cv::fisheye") public static native void undistortPoints(@ByVal Mat distorted, @ByVal Mat undistorted,
            @ByVal Mat K, @ByVal Mat D);

    /** computing undistortion and rectification maps for image transform by cv::remap()
     *  If D is empty zero distortion is used, if R or P is empty identity matrixes are used */
    @Namespace("cv::fisheye") public static native void initUndistortRectifyMap(@ByVal Mat K, @ByVal Mat D, @ByVal Mat R, @ByVal Mat P,
            @Const @ByRef Size size, int m1type, @ByVal Mat map1, @ByVal Mat map2);

    /** undistorts image, optionally changes resolution and camera matrix. If Knew zero identity matrix is used */
    @Namespace("cv::fisheye") public static native void undistortImage(@ByVal Mat distorted, @ByVal Mat undistorted,
            @ByVal Mat K, @ByVal Mat D, @ByVal Mat Knew/*=cv::noArray()*/, @Const @ByRef Size new_size/*=Size()*/);
    @Namespace("cv::fisheye") public static native void undistortImage(@ByVal Mat distorted, @ByVal Mat undistorted,
            @ByVal Mat K, @ByVal Mat D);

    /** estimates new camera matrix for undistortion or rectification */
    @Namespace("cv::fisheye") public static native void estimateNewCameraMatrixForUndistortRectify(@ByVal Mat K, @ByVal Mat D, @Const @ByRef Size image_size, @ByVal Mat R,
            @ByVal Mat P, double balance/*=0.0*/, @Const @ByRef Size new_size/*=Size()*/, double fov_scale/*=1.0*/);
    @Namespace("cv::fisheye") public static native void estimateNewCameraMatrixForUndistortRectify(@ByVal Mat K, @ByVal Mat D, @Const @ByRef Size image_size, @ByVal Mat R,
            @ByVal Mat P);

    /** performs camera calibaration */
    @Namespace("cv::fisheye") public static native double calibrate(@ByVal MatVector objectPoints, @ByVal MatVector imagePoints, @Const @ByRef Size image_size,
            @ByVal Mat K, @ByVal Mat D, @ByVal MatVector rvecs, @ByVal MatVector tvecs, int flags/*=0*/,
                @ByVal TermCriteria criteria/*=TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON)*/);
    @Namespace("cv::fisheye") public static native double calibrate(@ByVal MatVector objectPoints, @ByVal MatVector imagePoints, @Const @ByRef Size image_size,
            @ByVal Mat K, @ByVal Mat D, @ByVal MatVector rvecs, @ByVal MatVector tvecs);

    /** stereo rectification estimation */
    @Namespace("cv::fisheye") public static native void stereoRectify(@ByVal Mat K1, @ByVal Mat D1, @ByVal Mat K2, @ByVal Mat D2, @Const @ByRef Size imageSize, @ByVal Mat R, @ByVal Mat tvec,
            @ByVal Mat R1, @ByVal Mat R2, @ByVal Mat P1, @ByVal Mat P2, @ByVal Mat Q, int flags, @Const @ByRef Size newImageSize/*=Size()*/,
            double balance/*=0.0*/, double fov_scale/*=1.0*/);
    @Namespace("cv::fisheye") public static native void stereoRectify(@ByVal Mat K1, @ByVal Mat D1, @ByVal Mat K2, @ByVal Mat D2, @Const @ByRef Size imageSize, @ByVal Mat R, @ByVal Mat tvec,
            @ByVal Mat R1, @ByVal Mat R2, @ByVal Mat P1, @ByVal Mat P2, @ByVal Mat Q, int flags);

    /** performs stereo calibaration */
    @Namespace("cv::fisheye") public static native double stereoCalibrate(@ByVal MatVector objectPoints, @ByVal MatVector imagePoints1, @ByVal MatVector imagePoints2,
                                      @ByVal Mat K1, @ByVal Mat D1, @ByVal Mat K2, @ByVal Mat D2, @ByVal Size imageSize,
                                      @ByVal Mat R, @ByVal Mat T, int flags/*=CALIB_FIX_INTRINSIC*/,
                                      @ByVal TermCriteria criteria/*=TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON)*/);
    @Namespace("cv::fisheye") public static native double stereoCalibrate(@ByVal MatVector objectPoints, @ByVal MatVector imagePoints1, @ByVal MatVector imagePoints2,
                                      @ByVal Mat K1, @ByVal Mat D1, @ByVal Mat K2, @ByVal Mat D2, @ByVal Size imageSize,
                                      @ByVal Mat R, @ByVal Mat T);



 // cv

// #endif


}
