// Targeted by JavaCPP version 0.11

package org.bytedeco.javacpp;

import java.nio.*;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import static org.bytedeco.javacpp.opencv_core.*;

public class opencv_imgproc extends org.bytedeco.javacpp.helper.opencv_imgproc {
    static { Loader.load(); }

// Parsed from <opencv2/imgproc/types_c.h>

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
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

// #ifndef __OPENCV_IMGPROC_TYPES_C_H__
// #define __OPENCV_IMGPROC_TYPES_C_H__

// #include "opencv2/core/core_c.h"

// #ifdef __cplusplus
// #endif

/* Connected component structure */
public static class CvConnectedComp extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvConnectedComp() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvConnectedComp(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvConnectedComp(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvConnectedComp position(int position) {
        return (CvConnectedComp)super.position(position);
    }

    public native double area(); public native CvConnectedComp area(double area);    /* area of the connected component  */
    public native @ByRef CvScalar value(); public native CvConnectedComp value(CvScalar value); /* average color of the connected component */
    public native @ByRef CvRect rect(); public native CvConnectedComp rect(CvRect rect);    /* ROI of the component  */
    public native CvSeq contour(); public native CvConnectedComp contour(CvSeq contour); /* optional component boundary
                      (the contour might have child contours corresponding to the holes)*/
}

/* Image smooth methods */
/** enum  */
public static final int
    CV_BLUR_NO_SCALE = 0,
    CV_BLUR  = 1,
    CV_GAUSSIAN  = 2,
    CV_MEDIAN = 3,
    CV_BILATERAL = 4;

/* Filters used in pyramid decomposition */
/** enum  */
public static final int
    CV_GAUSSIAN_5x5 = 7;

/* Special filters */
/** enum  */
public static final int
    CV_SCHARR = -1,
    CV_MAX_SOBEL_KSIZE = 7;

/* Constants for color conversion */
/** enum  */
public static final int
    CV_BGR2BGRA    = 0,
    CV_RGB2RGBA    = CV_BGR2BGRA,

    CV_BGRA2BGR    = 1,
    CV_RGBA2RGB    = CV_BGRA2BGR,

    CV_BGR2RGBA    = 2,
    CV_RGB2BGRA    = CV_BGR2RGBA,

    CV_RGBA2BGR    = 3,
    CV_BGRA2RGB    = CV_RGBA2BGR,

    CV_BGR2RGB     = 4,
    CV_RGB2BGR     = CV_BGR2RGB,

    CV_BGRA2RGBA   = 5,
    CV_RGBA2BGRA   = CV_BGRA2RGBA,

    CV_BGR2GRAY    = 6,
    CV_RGB2GRAY    = 7,
    CV_GRAY2BGR    = 8,
    CV_GRAY2RGB    = CV_GRAY2BGR,
    CV_GRAY2BGRA   = 9,
    CV_GRAY2RGBA   = CV_GRAY2BGRA,
    CV_BGRA2GRAY   = 10,
    CV_RGBA2GRAY   = 11,

    CV_BGR2BGR565  = 12,
    CV_RGB2BGR565  = 13,
    CV_BGR5652BGR  = 14,
    CV_BGR5652RGB  = 15,
    CV_BGRA2BGR565 = 16,
    CV_RGBA2BGR565 = 17,
    CV_BGR5652BGRA = 18,
    CV_BGR5652RGBA = 19,

    CV_GRAY2BGR565 = 20,
    CV_BGR5652GRAY = 21,

    CV_BGR2BGR555  = 22,
    CV_RGB2BGR555  = 23,
    CV_BGR5552BGR  = 24,
    CV_BGR5552RGB  = 25,
    CV_BGRA2BGR555 = 26,
    CV_RGBA2BGR555 = 27,
    CV_BGR5552BGRA = 28,
    CV_BGR5552RGBA = 29,

    CV_GRAY2BGR555 = 30,
    CV_BGR5552GRAY = 31,

    CV_BGR2XYZ     = 32,
    CV_RGB2XYZ     = 33,
    CV_XYZ2BGR     = 34,
    CV_XYZ2RGB     = 35,

    CV_BGR2YCrCb   = 36,
    CV_RGB2YCrCb   = 37,
    CV_YCrCb2BGR   = 38,
    CV_YCrCb2RGB   = 39,

    CV_BGR2HSV     = 40,
    CV_RGB2HSV     = 41,

    CV_BGR2Lab     = 44,
    CV_RGB2Lab     = 45,

    CV_BayerBG2BGR = 46,
    CV_BayerGB2BGR = 47,
    CV_BayerRG2BGR = 48,
    CV_BayerGR2BGR = 49,

    CV_BayerBG2RGB = CV_BayerRG2BGR,
    CV_BayerGB2RGB = CV_BayerGR2BGR,
    CV_BayerRG2RGB = CV_BayerBG2BGR,
    CV_BayerGR2RGB = CV_BayerGB2BGR,

    CV_BGR2Luv     = 50,
    CV_RGB2Luv     = 51,
    CV_BGR2HLS     = 52,
    CV_RGB2HLS     = 53,

    CV_HSV2BGR     = 54,
    CV_HSV2RGB     = 55,

    CV_Lab2BGR     = 56,
    CV_Lab2RGB     = 57,
    CV_Luv2BGR     = 58,
    CV_Luv2RGB     = 59,
    CV_HLS2BGR     = 60,
    CV_HLS2RGB     = 61,

    CV_BayerBG2BGR_VNG = 62,
    CV_BayerGB2BGR_VNG = 63,
    CV_BayerRG2BGR_VNG = 64,
    CV_BayerGR2BGR_VNG = 65,

    CV_BayerBG2RGB_VNG = CV_BayerRG2BGR_VNG,
    CV_BayerGB2RGB_VNG = CV_BayerGR2BGR_VNG,
    CV_BayerRG2RGB_VNG = CV_BayerBG2BGR_VNG,
    CV_BayerGR2RGB_VNG = CV_BayerGB2BGR_VNG,

    CV_BGR2HSV_FULL = 66,
    CV_RGB2HSV_FULL = 67,
    CV_BGR2HLS_FULL = 68,
    CV_RGB2HLS_FULL = 69,

    CV_HSV2BGR_FULL = 70,
    CV_HSV2RGB_FULL = 71,
    CV_HLS2BGR_FULL = 72,
    CV_HLS2RGB_FULL = 73,

    CV_LBGR2Lab     = 74,
    CV_LRGB2Lab     = 75,
    CV_LBGR2Luv     = 76,
    CV_LRGB2Luv     = 77,

    CV_Lab2LBGR     = 78,
    CV_Lab2LRGB     = 79,
    CV_Luv2LBGR     = 80,
    CV_Luv2LRGB     = 81,

    CV_BGR2YUV      = 82,
    CV_RGB2YUV      = 83,
    CV_YUV2BGR      = 84,
    CV_YUV2RGB      = 85,

    CV_BayerBG2GRAY = 86,
    CV_BayerGB2GRAY = 87,
    CV_BayerRG2GRAY = 88,
    CV_BayerGR2GRAY = 89,

    //YUV 4:2:0 formats family
    CV_YUV2RGB_NV12 = 90,
    CV_YUV2BGR_NV12 = 91,
    CV_YUV2RGB_NV21 = 92,
    CV_YUV2BGR_NV21 = 93,
    CV_YUV420sp2RGB =  CV_YUV2RGB_NV21,
    CV_YUV420sp2BGR =  CV_YUV2BGR_NV21,

    CV_YUV2RGBA_NV12 = 94,
    CV_YUV2BGRA_NV12 = 95,
    CV_YUV2RGBA_NV21 = 96,
    CV_YUV2BGRA_NV21 = 97,
    CV_YUV420sp2RGBA =  CV_YUV2RGBA_NV21,
    CV_YUV420sp2BGRA =  CV_YUV2BGRA_NV21,

    CV_YUV2RGB_YV12 = 98,
    CV_YUV2BGR_YV12 = 99,
    CV_YUV2RGB_IYUV = 100,
    CV_YUV2BGR_IYUV = 101,
    CV_YUV2RGB_I420 =  CV_YUV2RGB_IYUV,
    CV_YUV2BGR_I420 =  CV_YUV2BGR_IYUV,
    CV_YUV420p2RGB =  CV_YUV2RGB_YV12,
    CV_YUV420p2BGR =  CV_YUV2BGR_YV12,

    CV_YUV2RGBA_YV12 = 102,
    CV_YUV2BGRA_YV12 = 103,
    CV_YUV2RGBA_IYUV = 104,
    CV_YUV2BGRA_IYUV = 105,
    CV_YUV2RGBA_I420 =  CV_YUV2RGBA_IYUV,
    CV_YUV2BGRA_I420 =  CV_YUV2BGRA_IYUV,
    CV_YUV420p2RGBA =  CV_YUV2RGBA_YV12,
    CV_YUV420p2BGRA =  CV_YUV2BGRA_YV12,

    CV_YUV2GRAY_420 = 106,
    CV_YUV2GRAY_NV21 =  CV_YUV2GRAY_420,
    CV_YUV2GRAY_NV12 =  CV_YUV2GRAY_420,
    CV_YUV2GRAY_YV12 =  CV_YUV2GRAY_420,
    CV_YUV2GRAY_IYUV =  CV_YUV2GRAY_420,
    CV_YUV2GRAY_I420 =  CV_YUV2GRAY_420,
    CV_YUV420sp2GRAY =  CV_YUV2GRAY_420,
    CV_YUV420p2GRAY =  CV_YUV2GRAY_420,

    //YUV 4:2:2 formats family
    CV_YUV2RGB_UYVY = 107,
    CV_YUV2BGR_UYVY = 108,
    //CV_YUV2RGB_VYUY = 109,
    //CV_YUV2BGR_VYUY = 110,
    CV_YUV2RGB_Y422 =  CV_YUV2RGB_UYVY,
    CV_YUV2BGR_Y422 =  CV_YUV2BGR_UYVY,
    CV_YUV2RGB_UYNV =  CV_YUV2RGB_UYVY,
    CV_YUV2BGR_UYNV =  CV_YUV2BGR_UYVY,

    CV_YUV2RGBA_UYVY = 111,
    CV_YUV2BGRA_UYVY = 112,
    //CV_YUV2RGBA_VYUY = 113,
    //CV_YUV2BGRA_VYUY = 114,
    CV_YUV2RGBA_Y422 =  CV_YUV2RGBA_UYVY,
    CV_YUV2BGRA_Y422 =  CV_YUV2BGRA_UYVY,
    CV_YUV2RGBA_UYNV =  CV_YUV2RGBA_UYVY,
    CV_YUV2BGRA_UYNV =  CV_YUV2BGRA_UYVY,

    CV_YUV2RGB_YUY2 = 115,
    CV_YUV2BGR_YUY2 = 116,
    CV_YUV2RGB_YVYU = 117,
    CV_YUV2BGR_YVYU = 118,
    CV_YUV2RGB_YUYV =  CV_YUV2RGB_YUY2,
    CV_YUV2BGR_YUYV =  CV_YUV2BGR_YUY2,
    CV_YUV2RGB_YUNV =  CV_YUV2RGB_YUY2,
    CV_YUV2BGR_YUNV =  CV_YUV2BGR_YUY2,

    CV_YUV2RGBA_YUY2 = 119,
    CV_YUV2BGRA_YUY2 = 120,
    CV_YUV2RGBA_YVYU = 121,
    CV_YUV2BGRA_YVYU = 122,
    CV_YUV2RGBA_YUYV =  CV_YUV2RGBA_YUY2,
    CV_YUV2BGRA_YUYV =  CV_YUV2BGRA_YUY2,
    CV_YUV2RGBA_YUNV =  CV_YUV2RGBA_YUY2,
    CV_YUV2BGRA_YUNV =  CV_YUV2BGRA_YUY2,

    CV_YUV2GRAY_UYVY = 123,
    CV_YUV2GRAY_YUY2 = 124,
    //CV_YUV2GRAY_VYUY = CV_YUV2GRAY_UYVY,
    CV_YUV2GRAY_Y422 =  CV_YUV2GRAY_UYVY,
    CV_YUV2GRAY_UYNV =  CV_YUV2GRAY_UYVY,
    CV_YUV2GRAY_YVYU =  CV_YUV2GRAY_YUY2,
    CV_YUV2GRAY_YUYV =  CV_YUV2GRAY_YUY2,
    CV_YUV2GRAY_YUNV =  CV_YUV2GRAY_YUY2,

    // alpha premultiplication
    CV_RGBA2mRGBA = 125,
    CV_mRGBA2RGBA = 126,

    CV_RGB2YUV_I420 = 127,
    CV_BGR2YUV_I420 = 128,
    CV_RGB2YUV_IYUV =  CV_RGB2YUV_I420,
    CV_BGR2YUV_IYUV =  CV_BGR2YUV_I420,

    CV_RGBA2YUV_I420 = 129,
    CV_BGRA2YUV_I420 = 130,
    CV_RGBA2YUV_IYUV =  CV_RGBA2YUV_I420,
    CV_BGRA2YUV_IYUV =  CV_BGRA2YUV_I420,
    CV_RGB2YUV_YV12  = 131,
    CV_BGR2YUV_YV12  = 132,
    CV_RGBA2YUV_YV12 = 133,
    CV_BGRA2YUV_YV12 = 134,

    // Edge-Aware Demosaicing
    CV_BayerBG2BGR_EA = 135,
    CV_BayerGB2BGR_EA = 136,
    CV_BayerRG2BGR_EA = 137,
    CV_BayerGR2BGR_EA = 138,

    CV_BayerBG2RGB_EA =  CV_BayerRG2BGR_EA,
    CV_BayerGB2RGB_EA =  CV_BayerGR2BGR_EA,
    CV_BayerRG2RGB_EA =  CV_BayerBG2BGR_EA,
    CV_BayerGR2RGB_EA =  CV_BayerGB2BGR_EA,

    CV_COLORCVT_MAX  = 139;


/* Sub-pixel interpolation methods */
/** enum  */
public static final int
    CV_INTER_NN        = 0,
    CV_INTER_LINEAR    = 1,
    CV_INTER_CUBIC     = 2,
    CV_INTER_AREA      = 3,
    CV_INTER_LANCZOS4  = 4;

/* ... and other image warping flags */
/** enum  */
public static final int
    CV_WARP_FILL_OUTLIERS = 8,
    CV_WARP_INVERSE_MAP  = 16;

/* Shapes of a structuring element for morphological operations */
/** enum  */
public static final int
    CV_SHAPE_RECT      = 0,
    CV_SHAPE_CROSS     = 1,
    CV_SHAPE_ELLIPSE   = 2,
    CV_SHAPE_CUSTOM    = 100;

/* Morphological operations */
/** enum  */
public static final int
    CV_MOP_ERODE        = 0,
    CV_MOP_DILATE       = 1,
    CV_MOP_OPEN         = 2,
    CV_MOP_CLOSE        = 3,
    CV_MOP_GRADIENT     = 4,
    CV_MOP_TOPHAT       = 5,
    CV_MOP_BLACKHAT     = 6;

/* Spatial and central moments */
@NoOffset public static class CvMoments extends AbstractCvMoments {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvMoments(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvMoments(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public CvMoments position(int position) {
        return (CvMoments)super.position(position);
    }

    public native double m00(); public native CvMoments m00(double m00);
    public native double m10(); public native CvMoments m10(double m10);
    public native double m01(); public native CvMoments m01(double m01);
    public native double m20(); public native CvMoments m20(double m20);
    public native double m11(); public native CvMoments m11(double m11);
    public native double m02(); public native CvMoments m02(double m02);
    public native double m30(); public native CvMoments m30(double m30);
    public native double m21(); public native CvMoments m21(double m21);
    public native double m12(); public native CvMoments m12(double m12);
    public native double m03(); public native CvMoments m03(double m03); /* spatial moments */
    public native double mu20(); public native CvMoments mu20(double mu20);
    public native double mu11(); public native CvMoments mu11(double mu11);
    public native double mu02(); public native CvMoments mu02(double mu02);
    public native double mu30(); public native CvMoments mu30(double mu30);
    public native double mu21(); public native CvMoments mu21(double mu21);
    public native double mu12(); public native CvMoments mu12(double mu12);
    public native double mu03(); public native CvMoments mu03(double mu03); /* central moments */
    public native double inv_sqrt_m00(); public native CvMoments inv_sqrt_m00(double inv_sqrt_m00); /* m00 != 0 ? 1/sqrt(m00) : 0 */

// #ifdef __cplusplus
    public CvMoments() { allocate(); }
    private native void allocate();
    public CvMoments(@Const @ByRef Moments m) { allocate(m); }
    private native void allocate(@Const @ByRef Moments m);
    public native @ByVal @Name("operator cv::Moments") Moments asMoments();
// #endif
}

/* Hu invariants */
public static class CvHuMoments extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvHuMoments() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvHuMoments(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvHuMoments(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvHuMoments position(int position) {
        return (CvHuMoments)super.position(position);
    }

    public native double hu1(); public native CvHuMoments hu1(double hu1);
    public native double hu2(); public native CvHuMoments hu2(double hu2);
    public native double hu3(); public native CvHuMoments hu3(double hu3);
    public native double hu4(); public native CvHuMoments hu4(double hu4);
    public native double hu5(); public native CvHuMoments hu5(double hu5);
    public native double hu6(); public native CvHuMoments hu6(double hu6);
    public native double hu7(); public native CvHuMoments hu7(double hu7); /* Hu invariants */
}

/* Template matching methods */
/** enum  */
public static final int
    CV_TM_SQDIFF        = 0,
    CV_TM_SQDIFF_NORMED = 1,
    CV_TM_CCORR         = 2,
    CV_TM_CCORR_NORMED  = 3,
    CV_TM_CCOEFF        = 4,
    CV_TM_CCOEFF_NORMED = 5;

@Convention("CV_CDECL") public static class CvDistanceFunction extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    CvDistanceFunction(Pointer p) { super(p); }
    protected CvDistanceFunction() { allocate(); }
    private native void allocate();
    public native float call( @Const FloatPointer a, @Const FloatPointer b, Pointer user_param );
}

/* Contour retrieval modes */
/** enum  */
public static final int
    CV_RETR_EXTERNAL= 0,
    CV_RETR_LIST= 1,
    CV_RETR_CCOMP= 2,
    CV_RETR_TREE= 3,
    CV_RETR_FLOODFILL= 4;

/* Contour approximation methods */
/** enum  */
public static final int
    CV_CHAIN_CODE= 0,
    CV_CHAIN_APPROX_NONE= 1,
    CV_CHAIN_APPROX_SIMPLE= 2,
    CV_CHAIN_APPROX_TC89_L1= 3,
    CV_CHAIN_APPROX_TC89_KCOS= 4,
    CV_LINK_RUNS= 5;

/*
Internal structure that is used for sequental retrieving contours from the image.
It supports both hierarchical and plane variants of Suzuki algorithm.
*/
@Name("_CvContourScanner") @Opaque public static class CvContourScanner extends Pointer {
    /** Empty constructor. */
    public CvContourScanner() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvContourScanner(Pointer p) { super(p); }
}

/* Freeman chain reader state */
public static class CvChainPtReader extends CvSeqReader {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvChainPtReader() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvChainPtReader(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvChainPtReader(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvChainPtReader position(int position) {
        return (CvChainPtReader)super.position(position);
    }

    public native int header_size(); public native CvChainPtReader header_size(int header_size);
    public native CvSeq seq(); public native CvChainPtReader seq(CvSeq seq);        /* sequence, beign read */
    public native CvSeqBlock block(); public native CvChainPtReader block(CvSeqBlock block);      /* current block */
    public native @Cast("schar*") BytePointer ptr(); public native CvChainPtReader ptr(BytePointer ptr);        /* pointer to element be read next */
    public native @Cast("schar*") BytePointer block_min(); public native CvChainPtReader block_min(BytePointer block_min);  /* pointer to the beginning of block */
    public native @Cast("schar*") BytePointer block_max(); public native CvChainPtReader block_max(BytePointer block_max);  /* pointer to the end of block */
    public native int delta_index(); public native CvChainPtReader delta_index(int delta_index);/* = seq->first->start_index   */
    public native @Cast("schar*") BytePointer prev_elem(); public native CvChainPtReader prev_elem(BytePointer prev_elem);  /* pointer to previous element */
    public native @Cast("char") byte code(); public native CvChainPtReader code(byte code);
    public native @ByRef CvPoint pt(); public native CvChainPtReader pt(CvPoint pt);
    public native @Cast("schar") byte deltas(int i, int j); public native CvChainPtReader deltas(int i, int j, byte deltas);
    @MemberGetter public native @Cast("schar(*)[2]") BytePointer deltas();
}

/* initializes 8-element array for fast access to 3x3 neighborhood of a pixel */
// #define  CV_INIT_3X3_DELTAS( deltas, step, nch )
//     ((deltas)[0] =  (nch),  (deltas)[1] = -(step) + (nch),
//      (deltas)[2] = -(step), (deltas)[3] = -(step) - (nch),
//      (deltas)[4] = -(nch),  (deltas)[5] =  (step) - (nch),
//      (deltas)[6] =  (step), (deltas)[7] =  (step) + (nch))


/* Contour approximation algorithms */
/** enum  */
public static final int
    CV_POLY_APPROX_DP = 0;

/* Shape matching methods */
/** enum  */
public static final int
    CV_CONTOURS_MATCH_I1  = 1,
    CV_CONTOURS_MATCH_I2  = 2,
    CV_CONTOURS_MATCH_I3  = 3;

/* Shape orientation */
/** enum  */
public static final int
    CV_CLOCKWISE         = 1,
    CV_COUNTER_CLOCKWISE = 2;


/* Convexity defect */
public static class CvConvexityDefect extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvConvexityDefect() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvConvexityDefect(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvConvexityDefect(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvConvexityDefect position(int position) {
        return (CvConvexityDefect)super.position(position);
    }

    public native CvPoint start(); public native CvConvexityDefect start(CvPoint start); /* point of the contour where the defect begins */
    public native CvPoint end(); public native CvConvexityDefect end(CvPoint end); /* point of the contour where the defect ends */
    public native CvPoint depth_point(); public native CvConvexityDefect depth_point(CvPoint depth_point); /* the farthest from the convex hull point within the defect */
    public native float depth(); public native CvConvexityDefect depth(float depth); /* distance between the farthest point and the convex hull */
}


/* Histogram comparison methods */
/** enum  */
public static final int
    CV_COMP_CORREL        = 0,
    CV_COMP_CHISQR        = 1,
    CV_COMP_INTERSECT     = 2,
    CV_COMP_BHATTACHARYYA = 3,
    CV_COMP_HELLINGER     = CV_COMP_BHATTACHARYYA,
    CV_COMP_CHISQR_ALT    = 4,
    CV_COMP_KL_DIV        = 5;

/* Mask size for distance transform */
/** enum  */
public static final int
    CV_DIST_MASK_3   = 3,
    CV_DIST_MASK_5   = 5,
    CV_DIST_MASK_PRECISE = 0;

/* Content of output label array: connected components or pixels */
/** enum  */
public static final int
  CV_DIST_LABEL_CCOMP = 0,
  CV_DIST_LABEL_PIXEL = 1;

/* Distance types for Distance Transform and M-estimators */
/** enum  */
public static final int
    CV_DIST_USER    = -1,  /* User defined distance */
    CV_DIST_L1      = 1,   /* distance = |x1-x2| + |y1-y2| */
    CV_DIST_L2      = 2,   /* the simple euclidean distance */
    CV_DIST_C       = 3,   /* distance = max(|x1-x2|,|y1-y2|) */
    CV_DIST_L12     = 4,   /* L1-L2 metric: distance = 2(sqrt(1+x*x/2) - 1)) */
    CV_DIST_FAIR    = 5,   /* distance = c^2(|x|/c-log(1+|x|/c)), c = 1.3998 */
    CV_DIST_WELSCH  = 6,   /* distance = c^2/2(1-exp(-(x/c)^2)), c = 2.9846 */
    CV_DIST_HUBER   = 7;    /* distance = |x|<c ? x^2/2 : c(|x|-c/2), c=1.345 */


/* Threshold types */
/** enum  */
public static final int
    CV_THRESH_BINARY      = 0,  /* value = value > threshold ? max_value : 0       */
    CV_THRESH_BINARY_INV  = 1,  /* value = value > threshold ? 0 : max_value       */
    CV_THRESH_TRUNC       = 2,  /* value = value > threshold ? threshold : value   */
    CV_THRESH_TOZERO      = 3,  /* value = value > threshold ? value : 0           */
    CV_THRESH_TOZERO_INV  = 4,  /* value = value > threshold ? 0 : value           */
    CV_THRESH_MASK        = 7,
    CV_THRESH_OTSU        = 8, /* use Otsu algorithm to choose the optimal threshold value;
                                 combine the flag with one of the above CV_THRESH_* values */
    CV_THRESH_TRIANGLE    = 16;  /* use Triangle algorithm to choose the optimal threshold value;
                                 combine the flag with one of the above CV_THRESH_* values, but not
                                 with CV_THRESH_OTSU */

/* Adaptive threshold methods */
/** enum  */
public static final int
    CV_ADAPTIVE_THRESH_MEAN_C  = 0,
    CV_ADAPTIVE_THRESH_GAUSSIAN_C  = 1;

/* FloodFill flags */
/** enum  */
public static final int
    CV_FLOODFILL_FIXED_RANGE = (1 << 16),
    CV_FLOODFILL_MASK_ONLY   = (1 << 17);


/* Canny edge detector flags */
/** enum  */
public static final int
    CV_CANNY_L2_GRADIENT  = (1 << 31);

/* Variants of a Hough transform */
/** enum  */
public static final int
    CV_HOUGH_STANDARD = 0,
    CV_HOUGH_PROBABILISTIC = 1,
    CV_HOUGH_MULTI_SCALE = 2,
    CV_HOUGH_GRADIENT = 3;


/* Fast search data structures  */
@Opaque public static class CvFeatureTree extends Pointer {
    /** Empty constructor. */
    public CvFeatureTree() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvFeatureTree(Pointer p) { super(p); }
}
@Opaque public static class CvLSH extends Pointer {
    /** Empty constructor. */
    public CvLSH() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvLSH(Pointer p) { super(p); }
}
@Opaque public static class CvLSHOperations extends Pointer {
    /** Empty constructor. */
    public CvLSHOperations() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvLSHOperations(Pointer p) { super(p); }
}

// #ifdef __cplusplus
// #endif

// #endif


// Parsed from <opencv2/imgproc/imgproc_c.h>

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
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

// #ifndef __OPENCV_IMGPROC_IMGPROC_C_H__
// #define __OPENCV_IMGPROC_IMGPROC_C_H__

// #include "opencv2/imgproc/types_c.h"

// #ifdef __cplusplus
// #endif

/*********************** Background statistics accumulation *****************************/

/* Adds image to accumulator */
public static native void cvAcc( @Const CvArr image, CvArr sum,
                   @Const CvArr mask/*=NULL*/ );
public static native void cvAcc( @Const CvArr image, CvArr sum );

/* Adds squared image to accumulator */
public static native void cvSquareAcc( @Const CvArr image, CvArr sqsum,
                         @Const CvArr mask/*=NULL*/ );
public static native void cvSquareAcc( @Const CvArr image, CvArr sqsum );

/* Adds a product of two images to accumulator */
public static native void cvMultiplyAcc( @Const CvArr image1, @Const CvArr image2, CvArr acc,
                           @Const CvArr mask/*=NULL*/ );
public static native void cvMultiplyAcc( @Const CvArr image1, @Const CvArr image2, CvArr acc );

/* Adds image to accumulator with weights: acc = acc*(1-alpha) + image*alpha */
public static native void cvRunningAvg( @Const CvArr image, CvArr acc, double alpha,
                          @Const CvArr mask/*=NULL*/ );
public static native void cvRunningAvg( @Const CvArr image, CvArr acc, double alpha );

/****************************************************************************************\
*                                    Image Processing                                    *
\****************************************************************************************/

/* Copies source 2D array inside of the larger destination array and
   makes a border of the specified type (IPL_BORDER_*) around the copied area. */
public static native void cvCopyMakeBorder( @Const CvArr src, CvArr dst, @ByVal CvPoint offset,
                              int bordertype, @ByVal CvScalar value/*=cvScalarAll(0)*/);
public static native void cvCopyMakeBorder( @Const CvArr src, CvArr dst, @ByVal CvPoint offset,
                              int bordertype);
public static native void cvCopyMakeBorder( @Const CvArr src, CvArr dst, @ByVal @Cast("CvPoint*") IntBuffer offset,
                              int bordertype, @ByVal CvScalar value/*=cvScalarAll(0)*/);
public static native void cvCopyMakeBorder( @Const CvArr src, CvArr dst, @ByVal @Cast("CvPoint*") IntBuffer offset,
                              int bordertype);
public static native void cvCopyMakeBorder( @Const CvArr src, CvArr dst, @ByVal @Cast("CvPoint*") int[] offset,
                              int bordertype, @ByVal CvScalar value/*=cvScalarAll(0)*/);
public static native void cvCopyMakeBorder( @Const CvArr src, CvArr dst, @ByVal @Cast("CvPoint*") int[] offset,
                              int bordertype);

/* Smoothes array (removes noise) */
public static native void cvSmooth( @Const CvArr src, CvArr dst,
                      int smoothtype/*=CV_GAUSSIAN*/,
                      int size1/*=3*/,
                      int size2/*=0*/,
                      double sigma1/*=0*/,
                      double sigma2/*=0*/);
public static native void cvSmooth( @Const CvArr src, CvArr dst);

/* Convolves the image with the kernel */
public static native void cvFilter2D( @Const CvArr src, CvArr dst, @Const CvMat kernel,
                        @ByVal CvPoint anchor/*=cvPoint(-1,-1)*/);
public static native void cvFilter2D( @Const CvArr src, CvArr dst, @Const CvMat kernel);
public static native void cvFilter2D( @Const CvArr src, CvArr dst, @Const CvMat kernel,
                        @ByVal @Cast("CvPoint*") IntBuffer anchor/*=cvPoint(-1,-1)*/);
public static native void cvFilter2D( @Const CvArr src, CvArr dst, @Const CvMat kernel,
                        @ByVal @Cast("CvPoint*") int[] anchor/*=cvPoint(-1,-1)*/);

/* Finds integral image: SUM(X,Y) = sum(x<X,y<Y)I(x,y) */
public static native void cvIntegral( @Const CvArr image, CvArr sum,
                       CvArr sqsum/*=NULL*/,
                       CvArr tilted_sum/*=NULL*/);
public static native void cvIntegral( @Const CvArr image, CvArr sum);

/*
   Smoothes the input image with gaussian kernel and then down-samples it.
   dst_width = floor(src_width/2)[+1],
   dst_height = floor(src_height/2)[+1]
*/
public static native void cvPyrDown( @Const CvArr src, CvArr dst,
                        int filter/*=CV_GAUSSIAN_5x5*/ );
public static native void cvPyrDown( @Const CvArr src, CvArr dst );

/*
   Up-samples image and smoothes the result with gaussian kernel.
   dst_width = src_width*2,
   dst_height = src_height*2
*/
public static native void cvPyrUp( @Const CvArr src, CvArr dst,
                      int filter/*=CV_GAUSSIAN_5x5*/ );
public static native void cvPyrUp( @Const CvArr src, CvArr dst );

/* Builds pyramid for an image */
public static native @Cast("CvMat**") PointerPointer cvCreatePyramid( @Const CvArr img, int extra_layers, double rate,
                                @Const CvSize layer_sizes/*=0*/,
                                CvArr bufarr/*=0*/,
                                int calc/*=1*/,
                                int filter/*=CV_GAUSSIAN_5x5*/ );
public static native @ByPtrPtr CvMat cvCreatePyramid( @Const CvArr img, int extra_layers, double rate );

/* Releases pyramid */
public static native void cvReleasePyramid( @Cast("CvMat***") PointerPointer pyramid, int extra_layers );


/* Filters image using meanshift algorithm */
public static native void cvPyrMeanShiftFiltering( @Const CvArr src, CvArr dst,
    double sp, double sr, int max_level/*=1*/,
    @ByVal CvTermCriteria termcrit/*=cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,5,1)*/);
public static native void cvPyrMeanShiftFiltering( @Const CvArr src, CvArr dst,
    double sp, double sr);

/* Segments image using seed "markers" */
public static native void cvWatershed( @Const CvArr image, CvArr markers );

/* Calculates an image derivative using generalized Sobel
   (aperture_size = 1,3,5,7) or Scharr (aperture_size = -1) operator.
   Scharr can be used only for the first dx or dy derivative */
public static native void cvSobel( @Const CvArr src, CvArr dst,
                    int xorder, int yorder,
                    int aperture_size/*=3*/);
public static native void cvSobel( @Const CvArr src, CvArr dst,
                    int xorder, int yorder);

/* Calculates the image Laplacian: (d2/dx + d2/dy)I */
public static native void cvLaplace( @Const CvArr src, CvArr dst,
                      int aperture_size/*=3*/ );
public static native void cvLaplace( @Const CvArr src, CvArr dst );

/* Converts input array pixels from one color space to another */
public static native void cvCvtColor( @Const CvArr src, CvArr dst, int code );


/* Resizes image (input array is resized to fit the destination array) */
public static native void cvResize( @Const CvArr src, CvArr dst,
                       int interpolation/*=CV_INTER_LINEAR*/);
public static native void cvResize( @Const CvArr src, CvArr dst);

/* Warps image with affine transform */
public static native void cvWarpAffine( @Const CvArr src, CvArr dst, @Const CvMat map_matrix,
                           int flags/*=CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS*/,
                           @ByVal CvScalar fillval/*=cvScalarAll(0)*/ );
public static native void cvWarpAffine( @Const CvArr src, CvArr dst, @Const CvMat map_matrix );

/* Computes affine transform matrix for mapping src[i] to dst[i] (i=0,1,2) */
public static native CvMat cvGetAffineTransform( @Const CvPoint2D32f src,
                                    @Const CvPoint2D32f dst,
                                    CvMat map_matrix );
public static native CvMat cvGetAffineTransform( @Cast("const CvPoint2D32f*") FloatBuffer src,
                                    @Cast("const CvPoint2D32f*") FloatBuffer dst,
                                    CvMat map_matrix );
public static native CvMat cvGetAffineTransform( @Cast("const CvPoint2D32f*") float[] src,
                                    @Cast("const CvPoint2D32f*") float[] dst,
                                    CvMat map_matrix );

/* Computes rotation_matrix matrix */
public static native CvMat cv2DRotationMatrix( @ByVal CvPoint2D32f center, double angle,
                                   double scale, CvMat map_matrix );
public static native CvMat cv2DRotationMatrix( @ByVal @Cast("CvPoint2D32f*") FloatBuffer center, double angle,
                                   double scale, CvMat map_matrix );
public static native CvMat cv2DRotationMatrix( @ByVal @Cast("CvPoint2D32f*") float[] center, double angle,
                                   double scale, CvMat map_matrix );

/* Warps image with perspective (projective) transform */
public static native void cvWarpPerspective( @Const CvArr src, CvArr dst, @Const CvMat map_matrix,
                                int flags/*=CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS*/,
                                @ByVal CvScalar fillval/*=cvScalarAll(0)*/ );
public static native void cvWarpPerspective( @Const CvArr src, CvArr dst, @Const CvMat map_matrix );

/* Computes perspective transform matrix for mapping src[i] to dst[i] (i=0,1,2,3) */
public static native CvMat cvGetPerspectiveTransform( @Const CvPoint2D32f src,
                                         @Const CvPoint2D32f dst,
                                         CvMat map_matrix );
public static native CvMat cvGetPerspectiveTransform( @Cast("const CvPoint2D32f*") FloatBuffer src,
                                         @Cast("const CvPoint2D32f*") FloatBuffer dst,
                                         CvMat map_matrix );
public static native CvMat cvGetPerspectiveTransform( @Cast("const CvPoint2D32f*") float[] src,
                                         @Cast("const CvPoint2D32f*") float[] dst,
                                         CvMat map_matrix );

/* Performs generic geometric transformation using the specified coordinate maps */
public static native void cvRemap( @Const CvArr src, CvArr dst,
                      @Const CvArr mapx, @Const CvArr mapy,
                      int flags/*=CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS*/,
                      @ByVal CvScalar fillval/*=cvScalarAll(0)*/ );
public static native void cvRemap( @Const CvArr src, CvArr dst,
                      @Const CvArr mapx, @Const CvArr mapy );

/* Converts mapx & mapy from floating-point to integer formats for cvRemap */
public static native void cvConvertMaps( @Const CvArr mapx, @Const CvArr mapy,
                            CvArr mapxy, CvArr mapalpha );

/* Performs forward or inverse log-polar image transform */
public static native void cvLogPolar( @Const CvArr src, CvArr dst,
                         @ByVal CvPoint2D32f center, double M,
                         int flags/*=CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS*/);
public static native void cvLogPolar( @Const CvArr src, CvArr dst,
                         @ByVal CvPoint2D32f center, double M);
public static native void cvLogPolar( @Const CvArr src, CvArr dst,
                         @ByVal @Cast("CvPoint2D32f*") FloatBuffer center, double M,
                         int flags/*=CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS*/);
public static native void cvLogPolar( @Const CvArr src, CvArr dst,
                         @ByVal @Cast("CvPoint2D32f*") FloatBuffer center, double M);
public static native void cvLogPolar( @Const CvArr src, CvArr dst,
                         @ByVal @Cast("CvPoint2D32f*") float[] center, double M,
                         int flags/*=CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS*/);
public static native void cvLogPolar( @Const CvArr src, CvArr dst,
                         @ByVal @Cast("CvPoint2D32f*") float[] center, double M);

/* Performs forward or inverse linear-polar image transform */
public static native void cvLinearPolar( @Const CvArr src, CvArr dst,
                         @ByVal CvPoint2D32f center, double maxRadius,
                         int flags/*=CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS*/);
public static native void cvLinearPolar( @Const CvArr src, CvArr dst,
                         @ByVal CvPoint2D32f center, double maxRadius);
public static native void cvLinearPolar( @Const CvArr src, CvArr dst,
                         @ByVal @Cast("CvPoint2D32f*") FloatBuffer center, double maxRadius,
                         int flags/*=CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS*/);
public static native void cvLinearPolar( @Const CvArr src, CvArr dst,
                         @ByVal @Cast("CvPoint2D32f*") FloatBuffer center, double maxRadius);
public static native void cvLinearPolar( @Const CvArr src, CvArr dst,
                         @ByVal @Cast("CvPoint2D32f*") float[] center, double maxRadius,
                         int flags/*=CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS*/);
public static native void cvLinearPolar( @Const CvArr src, CvArr dst,
                         @ByVal @Cast("CvPoint2D32f*") float[] center, double maxRadius);

/* Transforms the input image to compensate lens distortion */
public static native void cvUndistort2( @Const CvArr src, CvArr dst,
                          @Const CvMat camera_matrix,
                          @Const CvMat distortion_coeffs,
                          @Const CvMat new_camera_matrix/*=0*/ );
public static native void cvUndistort2( @Const CvArr src, CvArr dst,
                          @Const CvMat camera_matrix,
                          @Const CvMat distortion_coeffs );

/* Computes transformation map from intrinsic camera parameters
   that can used by cvRemap */
public static native void cvInitUndistortMap( @Const CvMat camera_matrix,
                                @Const CvMat distortion_coeffs,
                                CvArr mapx, CvArr mapy );

/* Computes undistortion+rectification map for a head of stereo camera */
public static native void cvInitUndistortRectifyMap( @Const CvMat camera_matrix,
                                       @Const CvMat dist_coeffs,
                                       @Const CvMat R, @Const CvMat new_camera_matrix,
                                       CvArr mapx, CvArr mapy );

/* Computes the original (undistorted) feature coordinates
   from the observed (distorted) coordinates */
public static native void cvUndistortPoints( @Const CvMat src, CvMat dst,
                               @Const CvMat camera_matrix,
                               @Const CvMat dist_coeffs,
                               @Const CvMat R/*=0*/,
                               @Const CvMat P/*=0*/);
public static native void cvUndistortPoints( @Const CvMat src, CvMat dst,
                               @Const CvMat camera_matrix,
                               @Const CvMat dist_coeffs);

/* creates structuring element used for morphological operations */
public static native IplConvKernel cvCreateStructuringElementEx(
            int cols, int rows, int anchor_x, int anchor_y,
            int shape, IntPointer values/*=NULL*/ );
public static native IplConvKernel cvCreateStructuringElementEx(
            int cols, int rows, int anchor_x, int anchor_y,
            int shape );
public static native IplConvKernel cvCreateStructuringElementEx(
            int cols, int rows, int anchor_x, int anchor_y,
            int shape, IntBuffer values/*=NULL*/ );
public static native IplConvKernel cvCreateStructuringElementEx(
            int cols, int rows, int anchor_x, int anchor_y,
            int shape, int[] values/*=NULL*/ );

/* releases structuring element */
public static native void cvReleaseStructuringElement( @Cast("IplConvKernel**") PointerPointer element );
public static native void cvReleaseStructuringElement( @ByPtrPtr IplConvKernel element );

/* erodes input image (applies minimum filter) one or more times.
   If element pointer is NULL, 3x3 rectangular element is used */
public static native void cvErode( @Const CvArr src, CvArr dst,
                      IplConvKernel element/*=NULL*/,
                      int iterations/*=1*/ );
public static native void cvErode( @Const CvArr src, CvArr dst );

/* dilates input image (applies maximum filter) one or more times.
   If element pointer is NULL, 3x3 rectangular element is used */
public static native void cvDilate( @Const CvArr src, CvArr dst,
                       IplConvKernel element/*=NULL*/,
                       int iterations/*=1*/ );
public static native void cvDilate( @Const CvArr src, CvArr dst );

/* Performs complex morphological transformation */
public static native void cvMorphologyEx( @Const CvArr src, CvArr dst,
                             CvArr temp, IplConvKernel element,
                             int operation, int iterations/*=1*/ );
public static native void cvMorphologyEx( @Const CvArr src, CvArr dst,
                             CvArr temp, IplConvKernel element,
                             int operation );

/* Calculates all spatial and central moments up to the 3rd order */
public static native void cvMoments( @Const CvArr arr, CvMoments moments, int binary/*=0*/);
public static native void cvMoments( @Const CvArr arr, CvMoments moments);

/* Retrieve particular spatial, central or normalized central moments */
public static native double cvGetSpatialMoment( CvMoments moments, int x_order, int y_order );
public static native double cvGetCentralMoment( CvMoments moments, int x_order, int y_order );
public static native double cvGetNormalizedCentralMoment( CvMoments moments,
                                             int x_order, int y_order );

/* Calculates 7 Hu's invariants from precalculated spatial and central moments */
public static native void cvGetHuMoments( CvMoments moments, CvHuMoments hu_moments );

/*********************************** data sampling **************************************/

/* Fetches pixels that belong to the specified line segment and stores them to the buffer.
   Returns the number of retrieved points. */
public static native int cvSampleLine( @Const CvArr image, @ByVal CvPoint pt1, @ByVal CvPoint pt2, Pointer buffer,
                          int connectivity/*=8*/);
public static native int cvSampleLine( @Const CvArr image, @ByVal CvPoint pt1, @ByVal CvPoint pt2, Pointer buffer);
public static native int cvSampleLine( @Const CvArr image, @ByVal @Cast("CvPoint*") IntBuffer pt1, @ByVal @Cast("CvPoint*") IntBuffer pt2, Pointer buffer,
                          int connectivity/*=8*/);
public static native int cvSampleLine( @Const CvArr image, @ByVal @Cast("CvPoint*") IntBuffer pt1, @ByVal @Cast("CvPoint*") IntBuffer pt2, Pointer buffer);
public static native int cvSampleLine( @Const CvArr image, @ByVal @Cast("CvPoint*") int[] pt1, @ByVal @Cast("CvPoint*") int[] pt2, Pointer buffer,
                          int connectivity/*=8*/);
public static native int cvSampleLine( @Const CvArr image, @ByVal @Cast("CvPoint*") int[] pt1, @ByVal @Cast("CvPoint*") int[] pt2, Pointer buffer);

/* Retrieves the rectangular image region with specified center from the input array.
 dst(x,y) <- src(x + center.x - dst_width/2, y + center.y - dst_height/2).
 Values of pixels with fractional coordinates are retrieved using bilinear interpolation*/
public static native void cvGetRectSubPix( @Const CvArr src, CvArr dst, @ByVal CvPoint2D32f center );
public static native void cvGetRectSubPix( @Const CvArr src, CvArr dst, @ByVal @Cast("CvPoint2D32f*") FloatBuffer center );
public static native void cvGetRectSubPix( @Const CvArr src, CvArr dst, @ByVal @Cast("CvPoint2D32f*") float[] center );


/* Retrieves quadrangle from the input array.
    matrixarr = ( a11  a12 | b1 )   dst(x,y) <- src(A[x y]' + b)
                ( a21  a22 | b2 )   (bilinear interpolation is used to retrieve pixels
                                     with fractional coordinates)
*/
public static native void cvGetQuadrangleSubPix( @Const CvArr src, CvArr dst,
                                    @Const CvMat map_matrix );

/* Measures similarity between template and overlapped windows in the source image
   and fills the resultant image with the measurements */
public static native void cvMatchTemplate( @Const CvArr image, @Const CvArr templ,
                              CvArr result, int method );

/* Computes earth mover distance between
   two weighted point sets (called signatures) */
public static native float cvCalcEMD2( @Const CvArr signature1,
                          @Const CvArr signature2,
                          int distance_type,
                          CvDistanceFunction distance_func/*=NULL*/,
                          @Const CvArr cost_matrix/*=NULL*/,
                          CvArr flow/*=NULL*/,
                          FloatPointer lower_bound/*=NULL*/,
                          Pointer userdata/*=NULL*/);
public static native float cvCalcEMD2( @Const CvArr signature1,
                          @Const CvArr signature2,
                          int distance_type);
public static native float cvCalcEMD2( @Const CvArr signature1,
                          @Const CvArr signature2,
                          int distance_type,
                          CvDistanceFunction distance_func/*=NULL*/,
                          @Const CvArr cost_matrix/*=NULL*/,
                          CvArr flow/*=NULL*/,
                          FloatBuffer lower_bound/*=NULL*/,
                          Pointer userdata/*=NULL*/);
public static native float cvCalcEMD2( @Const CvArr signature1,
                          @Const CvArr signature2,
                          int distance_type,
                          CvDistanceFunction distance_func/*=NULL*/,
                          @Const CvArr cost_matrix/*=NULL*/,
                          CvArr flow/*=NULL*/,
                          float[] lower_bound/*=NULL*/,
                          Pointer userdata/*=NULL*/);

/****************************************************************************************\
*                              Contours retrieving                                       *
\****************************************************************************************/

/* Retrieves outer and optionally inner boundaries of white (non-zero) connected
   components in the black (zero) background */
public static native int cvFindContours( CvArr image, CvMemStorage storage, @Cast("CvSeq**") PointerPointer first_contour,
                            int header_size/*=sizeof(CvContour)*/,
                            int mode/*=CV_RETR_LIST*/,
                            int method/*=CV_CHAIN_APPROX_SIMPLE*/,
                            @ByVal CvPoint offset/*=cvPoint(0,0)*/);
public static native int cvFindContours( CvArr image, CvMemStorage storage, @ByPtrPtr CvSeq first_contour);
public static native int cvFindContours( CvArr image, CvMemStorage storage, @ByPtrPtr CvSeq first_contour,
                            int header_size/*=sizeof(CvContour)*/,
                            int mode/*=CV_RETR_LIST*/,
                            int method/*=CV_CHAIN_APPROX_SIMPLE*/,
                            @ByVal CvPoint offset/*=cvPoint(0,0)*/);
public static native int cvFindContours( CvArr image, CvMemStorage storage, @ByPtrPtr CvSeq first_contour,
                            int header_size/*=sizeof(CvContour)*/,
                            int mode/*=CV_RETR_LIST*/,
                            int method/*=CV_CHAIN_APPROX_SIMPLE*/,
                            @ByVal @Cast("CvPoint*") IntBuffer offset/*=cvPoint(0,0)*/);
public static native int cvFindContours( CvArr image, CvMemStorage storage, @ByPtrPtr CvSeq first_contour,
                            int header_size/*=sizeof(CvContour)*/,
                            int mode/*=CV_RETR_LIST*/,
                            int method/*=CV_CHAIN_APPROX_SIMPLE*/,
                            @ByVal @Cast("CvPoint*") int[] offset/*=cvPoint(0,0)*/);

/* Initializes contour retrieving process.
   Calls cvStartFindContours.
   Calls cvFindNextContour until null pointer is returned
   or some other condition becomes true.
   Calls cvEndFindContours at the end. */
public static native CvContourScanner cvStartFindContours( CvArr image, CvMemStorage storage,
                            int header_size/*=sizeof(CvContour)*/,
                            int mode/*=CV_RETR_LIST*/,
                            int method/*=CV_CHAIN_APPROX_SIMPLE*/,
                            @ByVal CvPoint offset/*=cvPoint(0,0)*/);
public static native CvContourScanner cvStartFindContours( CvArr image, CvMemStorage storage);
public static native CvContourScanner cvStartFindContours( CvArr image, CvMemStorage storage,
                            int header_size/*=sizeof(CvContour)*/,
                            int mode/*=CV_RETR_LIST*/,
                            int method/*=CV_CHAIN_APPROX_SIMPLE*/,
                            @ByVal @Cast("CvPoint*") IntBuffer offset/*=cvPoint(0,0)*/);
public static native CvContourScanner cvStartFindContours( CvArr image, CvMemStorage storage,
                            int header_size/*=sizeof(CvContour)*/,
                            int mode/*=CV_RETR_LIST*/,
                            int method/*=CV_CHAIN_APPROX_SIMPLE*/,
                            @ByVal @Cast("CvPoint*") int[] offset/*=cvPoint(0,0)*/);

/* Retrieves next contour */
public static native CvSeq cvFindNextContour( CvContourScanner scanner );


/* Substitutes the last retrieved contour with the new one
   (if the substitutor is null, the last retrieved contour is removed from the tree) */
public static native void cvSubstituteContour( CvContourScanner scanner, CvSeq new_contour );


/* Releases contour scanner and returns pointer to the first outer contour */
public static native CvSeq cvEndFindContours( @ByPtrPtr CvContourScanner scanner );

/* Approximates a single Freeman chain or a tree of chains to polygonal curves */
public static native CvSeq cvApproxChains( CvSeq src_seq, CvMemStorage storage,
                            int method/*=CV_CHAIN_APPROX_SIMPLE*/,
                            double parameter/*=0*/,
                            int minimal_perimeter/*=0*/,
                            int recursive/*=0*/);
public static native CvSeq cvApproxChains( CvSeq src_seq, CvMemStorage storage);

/* Initializes Freeman chain reader.
   The reader is used to iteratively get coordinates of all the chain points.
   If the Freeman codes should be read as is, a simple sequence reader should be used */
public static native void cvStartReadChainPoints( CvChain chain, CvChainPtReader reader );

/* Retrieves the next chain point */
public static native @ByVal CvPoint cvReadChainPoint( CvChainPtReader reader );


/****************************************************************************************\
*                            Contour Processing and Shape Analysis                       *
\****************************************************************************************/

/* Approximates a single polygonal curve (contour) or
   a tree of polygonal curves (contours) */
public static native CvSeq cvApproxPoly( @Const Pointer src_seq,
                             int header_size, CvMemStorage storage,
                             int method, double eps,
                             int recursive/*=0*/);
public static native CvSeq cvApproxPoly( @Const Pointer src_seq,
                             int header_size, CvMemStorage storage,
                             int method, double eps);

/* Calculates perimeter of a contour or length of a part of contour */
public static native double cvArcLength( @Const Pointer curve,
                            @ByVal CvSlice slice/*=CV_WHOLE_SEQ*/,
                            int is_closed/*=-1*/);
public static native double cvArcLength( @Const Pointer curve);

public static native double cvContourPerimeter( @Const Pointer contour );


/* Calculates contour bounding rectangle (update=1) or
   just retrieves pre-calculated rectangle (update=0) */
public static native @ByVal CvRect cvBoundingRect( CvArr points, int update/*=0*/ );
public static native @ByVal CvRect cvBoundingRect( CvArr points );

/* Calculates area of a contour or contour segment */
public static native double cvContourArea( @Const CvArr contour,
                              @ByVal CvSlice slice/*=CV_WHOLE_SEQ*/,
                              int oriented/*=0*/);
public static native double cvContourArea( @Const CvArr contour);

/* Finds minimum area rotated rectangle bounding a set of points */
public static native @ByVal CvBox2D cvMinAreaRect2( @Const CvArr points,
                                CvMemStorage storage/*=NULL*/);
public static native @ByVal CvBox2D cvMinAreaRect2( @Const CvArr points);

/* Finds minimum enclosing circle for a set of points */
public static native int cvMinEnclosingCircle( @Const CvArr points,
                                  CvPoint2D32f center, FloatPointer radius );
public static native int cvMinEnclosingCircle( @Const CvArr points,
                                  @Cast("CvPoint2D32f*") FloatBuffer center, FloatBuffer radius );
public static native int cvMinEnclosingCircle( @Const CvArr points,
                                  @Cast("CvPoint2D32f*") float[] center, float[] radius );

/* Compares two contours by matching their moments */
public static native double cvMatchShapes( @Const Pointer object1, @Const Pointer object2,
                              int method, double parameter/*=0*/);
public static native double cvMatchShapes( @Const Pointer object1, @Const Pointer object2,
                              int method);

/* Calculates exact convex hull of 2d point set */
public static native CvSeq cvConvexHull2( @Const CvArr input,
                             Pointer hull_storage/*=NULL*/,
                             int orientation/*=CV_CLOCKWISE*/,
                             int return_points/*=0*/);
public static native CvSeq cvConvexHull2( @Const CvArr input);

/* Checks whether the contour is convex or not (returns 1 if convex, 0 if not) */
public static native int cvCheckContourConvexity( @Const CvArr contour );


/* Finds convexity defects for the contour */
public static native CvSeq cvConvexityDefects( @Const CvArr contour, @Const CvArr convexhull,
                                   CvMemStorage storage/*=NULL*/);
public static native CvSeq cvConvexityDefects( @Const CvArr contour, @Const CvArr convexhull);

/* Fits ellipse into a set of 2d points */
public static native @ByVal CvBox2D cvFitEllipse2( @Const CvArr points );

/* Finds minimum rectangle containing two given rectangles */
public static native @ByVal CvRect cvMaxRect( @Const CvRect rect1, @Const CvRect rect2 );

/* Finds coordinates of the box vertices */
public static native void cvBoxPoints( @ByVal CvBox2D box, CvPoint2D32f pt );
public static native void cvBoxPoints( @ByVal CvBox2D box, @Cast("CvPoint2D32f*") FloatBuffer pt );
public static native void cvBoxPoints( @ByVal CvBox2D box, @Cast("CvPoint2D32f*") float[] pt );

/* Initializes sequence header for a matrix (column or row vector) of points -
   a wrapper for cvMakeSeqHeaderForArray (it does not initialize bounding rectangle!!!) */
public static native CvSeq cvPointSeqFromMat( int seq_kind, @Const CvArr mat,
                                 CvContour contour_header,
                                 CvSeqBlock block );

/* Checks whether the point is inside polygon, outside, on an edge (at a vertex).
   Returns positive, negative or zero value, correspondingly.
   Optionally, measures a signed distance between
   the point and the nearest polygon edge (measure_dist=1) */
public static native double cvPointPolygonTest( @Const CvArr contour,
                                  @ByVal CvPoint2D32f pt, int measure_dist );
public static native double cvPointPolygonTest( @Const CvArr contour,
                                  @ByVal @Cast("CvPoint2D32f*") FloatBuffer pt, int measure_dist );
public static native double cvPointPolygonTest( @Const CvArr contour,
                                  @ByVal @Cast("CvPoint2D32f*") float[] pt, int measure_dist );

/****************************************************************************************\
*                                  Histogram functions                                   *
\****************************************************************************************/

/* Creates new histogram */
public static native CvHistogram cvCreateHist( int dims, IntPointer sizes, int type,
                                   @Cast("float**") PointerPointer ranges/*=NULL*/,
                                   int uniform/*=1*/);
public static native CvHistogram cvCreateHist( int dims, IntPointer sizes, int type);
public static native CvHistogram cvCreateHist( int dims, IntPointer sizes, int type,
                                   @ByPtrPtr FloatPointer ranges/*=NULL*/,
                                   int uniform/*=1*/);
public static native CvHistogram cvCreateHist( int dims, IntBuffer sizes, int type,
                                   @ByPtrPtr FloatBuffer ranges/*=NULL*/,
                                   int uniform/*=1*/);
public static native CvHistogram cvCreateHist( int dims, IntBuffer sizes, int type);
public static native CvHistogram cvCreateHist( int dims, int[] sizes, int type,
                                   @ByPtrPtr float[] ranges/*=NULL*/,
                                   int uniform/*=1*/);
public static native CvHistogram cvCreateHist( int dims, int[] sizes, int type);

/* Assignes histogram bin ranges */
public static native void cvSetHistBinRanges( CvHistogram hist, @Cast("float**") PointerPointer ranges,
                                int uniform/*=1*/);
public static native void cvSetHistBinRanges( CvHistogram hist, @ByPtrPtr FloatPointer ranges);
public static native void cvSetHistBinRanges( CvHistogram hist, @ByPtrPtr FloatPointer ranges,
                                int uniform/*=1*/);
public static native void cvSetHistBinRanges( CvHistogram hist, @ByPtrPtr FloatBuffer ranges,
                                int uniform/*=1*/);
public static native void cvSetHistBinRanges( CvHistogram hist, @ByPtrPtr FloatBuffer ranges);
public static native void cvSetHistBinRanges( CvHistogram hist, @ByPtrPtr float[] ranges,
                                int uniform/*=1*/);
public static native void cvSetHistBinRanges( CvHistogram hist, @ByPtrPtr float[] ranges);

/* Creates histogram header for array */
public static native CvHistogram cvMakeHistHeaderForArray(
                            int dims, IntPointer sizes, CvHistogram hist,
                            FloatPointer data, @Cast("float**") PointerPointer ranges/*=NULL*/,
                            int uniform/*=1*/);
public static native CvHistogram cvMakeHistHeaderForArray(
                            int dims, IntPointer sizes, CvHistogram hist,
                            FloatPointer data);
public static native CvHistogram cvMakeHistHeaderForArray(
                            int dims, IntPointer sizes, CvHistogram hist,
                            FloatPointer data, @ByPtrPtr FloatPointer ranges/*=NULL*/,
                            int uniform/*=1*/);
public static native CvHistogram cvMakeHistHeaderForArray(
                            int dims, IntBuffer sizes, CvHistogram hist,
                            FloatBuffer data, @ByPtrPtr FloatBuffer ranges/*=NULL*/,
                            int uniform/*=1*/);
public static native CvHistogram cvMakeHistHeaderForArray(
                            int dims, IntBuffer sizes, CvHistogram hist,
                            FloatBuffer data);
public static native CvHistogram cvMakeHistHeaderForArray(
                            int dims, int[] sizes, CvHistogram hist,
                            float[] data, @ByPtrPtr float[] ranges/*=NULL*/,
                            int uniform/*=1*/);
public static native CvHistogram cvMakeHistHeaderForArray(
                            int dims, int[] sizes, CvHistogram hist,
                            float[] data);

/* Releases histogram */
public static native void cvReleaseHist( @Cast("CvHistogram**") PointerPointer hist );
public static native void cvReleaseHist( @ByPtrPtr CvHistogram hist );

/* Clears all the histogram bins */
public static native void cvClearHist( CvHistogram hist );

/* Finds indices and values of minimum and maximum histogram bins */
public static native void cvGetMinMaxHistValue( @Const CvHistogram hist,
                                   FloatPointer min_value, FloatPointer max_value,
                                   IntPointer min_idx/*=NULL*/,
                                   IntPointer max_idx/*=NULL*/);
public static native void cvGetMinMaxHistValue( @Const CvHistogram hist,
                                   FloatPointer min_value, FloatPointer max_value);
public static native void cvGetMinMaxHistValue( @Const CvHistogram hist,
                                   FloatBuffer min_value, FloatBuffer max_value,
                                   IntBuffer min_idx/*=NULL*/,
                                   IntBuffer max_idx/*=NULL*/);
public static native void cvGetMinMaxHistValue( @Const CvHistogram hist,
                                   FloatBuffer min_value, FloatBuffer max_value);
public static native void cvGetMinMaxHistValue( @Const CvHistogram hist,
                                   float[] min_value, float[] max_value,
                                   int[] min_idx/*=NULL*/,
                                   int[] max_idx/*=NULL*/);
public static native void cvGetMinMaxHistValue( @Const CvHistogram hist,
                                   float[] min_value, float[] max_value);


/* Normalizes histogram by dividing all bins by sum of the bins, multiplied by <factor>.
   After that sum of histogram bins is equal to <factor> */
public static native void cvNormalizeHist( CvHistogram hist, double factor );


/* Clear all histogram bins that are below the threshold */
public static native void cvThreshHist( CvHistogram hist, double threshold );


/* Compares two histogram */
public static native double cvCompareHist( @Const CvHistogram hist1,
                              @Const CvHistogram hist2,
                              int method);

/* Copies one histogram to another. Destination histogram is created if
   the destination pointer is NULL */
public static native void cvCopyHist( @Const CvHistogram src, @Cast("CvHistogram**") PointerPointer dst );
public static native void cvCopyHist( @Const CvHistogram src, @ByPtrPtr CvHistogram dst );


/* Calculates bayesian probabilistic histograms
   (each or src and dst is an array of <number> histograms */
public static native void cvCalcBayesianProb( @Cast("CvHistogram**") PointerPointer src, int number,
                                @Cast("CvHistogram**") PointerPointer dst);
public static native void cvCalcBayesianProb( @ByPtrPtr CvHistogram src, int number,
                                @ByPtrPtr CvHistogram dst);

/* Calculates array histogram */
public static native void cvCalcArrHist( @Cast("CvArr**") PointerPointer arr, CvHistogram hist,
                            int accumulate/*=0*/,
                            @Const CvArr mask/*=NULL*/ );
public static native void cvCalcArrHist( @ByPtrPtr CvArr arr, CvHistogram hist );
public static native void cvCalcArrHist( @ByPtrPtr CvArr arr, CvHistogram hist,
                            int accumulate/*=0*/,
                            @Const CvArr mask/*=NULL*/ );

public static native void cvCalcHist( @Cast("IplImage**") PointerPointer image, CvHistogram hist,
                             int accumulate/*=0*/,
                             @Const CvArr mask/*=NULL*/ );
public static native void cvCalcHist( @ByPtrPtr IplImage image, CvHistogram hist );
public static native void cvCalcHist( @ByPtrPtr IplImage image, CvHistogram hist,
                             int accumulate/*=0*/,
                             @Const CvArr mask/*=NULL*/ );

/* Calculates back project */
public static native void cvCalcArrBackProject( @Cast("CvArr**") PointerPointer image, CvArr dst,
                                   @Const CvHistogram hist );
public static native void cvCalcArrBackProject( @ByPtrPtr CvArr image, CvArr dst,
                                   @Const CvHistogram hist );
public static native void cvCalcBackProject(@Cast("IplImage**") PointerPointer image, CvArr dst, CvHistogram hist);
public static native void cvCalcBackProject(@ByPtrPtr IplImage image, CvArr dst, CvHistogram hist);


/* Does some sort of template matching but compares histograms of
   template and each window location */
public static native void cvCalcArrBackProjectPatch( @Cast("CvArr**") PointerPointer image, CvArr dst, @ByVal CvSize range,
                                        CvHistogram hist, int method,
                                        double factor );
public static native void cvCalcArrBackProjectPatch( @ByPtrPtr CvArr image, CvArr dst, @ByVal CvSize range,
                                        CvHistogram hist, int method,
                                        double factor );
public static native void cvCalcBackProjectPatch(@Cast("IplImage**") PointerPointer image, CvArr dst, @ByVal CvSize range, CvHistogram hist, int method, double factor);
public static native void cvCalcBackProjectPatch(@ByPtrPtr IplImage image, CvArr dst, @ByVal CvSize range, CvHistogram hist, int method, double factor);


/* calculates probabilistic density (divides one histogram by another) */
public static native void cvCalcProbDensity( @Const CvHistogram hist1, @Const CvHistogram hist2,
                                CvHistogram dst_hist, double scale/*=255*/ );
public static native void cvCalcProbDensity( @Const CvHistogram hist1, @Const CvHistogram hist2,
                                CvHistogram dst_hist );

/* equalizes histogram of 8-bit single-channel image */
public static native void cvEqualizeHist( @Const CvArr src, CvArr dst );


/* Applies distance transform to binary image */
public static native void cvDistTransform( @Const CvArr src, CvArr dst,
                              int distance_type/*=CV_DIST_L2*/,
                              int mask_size/*=3*/,
                              @Const FloatPointer mask/*=NULL*/,
                              CvArr labels/*=NULL*/,
                              int labelType/*=CV_DIST_LABEL_CCOMP*/);
public static native void cvDistTransform( @Const CvArr src, CvArr dst);
public static native void cvDistTransform( @Const CvArr src, CvArr dst,
                              int distance_type/*=CV_DIST_L2*/,
                              int mask_size/*=3*/,
                              @Const FloatBuffer mask/*=NULL*/,
                              CvArr labels/*=NULL*/,
                              int labelType/*=CV_DIST_LABEL_CCOMP*/);
public static native void cvDistTransform( @Const CvArr src, CvArr dst,
                              int distance_type/*=CV_DIST_L2*/,
                              int mask_size/*=3*/,
                              @Const float[] mask/*=NULL*/,
                              CvArr labels/*=NULL*/,
                              int labelType/*=CV_DIST_LABEL_CCOMP*/);


/* Applies fixed-level threshold to grayscale image.
   This is a basic operation applied before retrieving contours */
public static native double cvThreshold( @Const CvArr src, CvArr dst,
                            double threshold, double max_value,
                            int threshold_type );

/* Applies adaptive threshold to grayscale image.
   The two parameters for methods CV_ADAPTIVE_THRESH_MEAN_C and
   CV_ADAPTIVE_THRESH_GAUSSIAN_C are:
   neighborhood size (3, 5, 7 etc.),
   and a constant subtracted from mean (...,-3,-2,-1,0,1,2,3,...) */
public static native void cvAdaptiveThreshold( @Const CvArr src, CvArr dst, double max_value,
                                  int adaptive_method/*=CV_ADAPTIVE_THRESH_MEAN_C*/,
                                  int threshold_type/*=CV_THRESH_BINARY*/,
                                  int block_size/*=3*/,
                                  double param1/*=5*/);
public static native void cvAdaptiveThreshold( @Const CvArr src, CvArr dst, double max_value);

/* Fills the connected component until the color difference gets large enough */
public static native void cvFloodFill( CvArr image, @ByVal CvPoint seed_point,
                          @ByVal CvScalar new_val, @ByVal CvScalar lo_diff/*=cvScalarAll(0)*/,
                          @ByVal CvScalar up_diff/*=cvScalarAll(0)*/,
                          CvConnectedComp comp/*=NULL*/,
                          int flags/*=4*/,
                          CvArr mask/*=NULL*/);
public static native void cvFloodFill( CvArr image, @ByVal CvPoint seed_point,
                          @ByVal CvScalar new_val);
public static native void cvFloodFill( CvArr image, @ByVal @Cast("CvPoint*") IntBuffer seed_point,
                          @ByVal CvScalar new_val, @ByVal CvScalar lo_diff/*=cvScalarAll(0)*/,
                          @ByVal CvScalar up_diff/*=cvScalarAll(0)*/,
                          CvConnectedComp comp/*=NULL*/,
                          int flags/*=4*/,
                          CvArr mask/*=NULL*/);
public static native void cvFloodFill( CvArr image, @ByVal @Cast("CvPoint*") IntBuffer seed_point,
                          @ByVal CvScalar new_val);
public static native void cvFloodFill( CvArr image, @ByVal @Cast("CvPoint*") int[] seed_point,
                          @ByVal CvScalar new_val, @ByVal CvScalar lo_diff/*=cvScalarAll(0)*/,
                          @ByVal CvScalar up_diff/*=cvScalarAll(0)*/,
                          CvConnectedComp comp/*=NULL*/,
                          int flags/*=4*/,
                          CvArr mask/*=NULL*/);
public static native void cvFloodFill( CvArr image, @ByVal @Cast("CvPoint*") int[] seed_point,
                          @ByVal CvScalar new_val);

/****************************************************************************************\
*                                  Feature detection                                     *
\****************************************************************************************/

/* Runs canny edge detector */
public static native void cvCanny( @Const CvArr image, CvArr edges, double threshold1,
                      double threshold2, int aperture_size/*=3*/ );
public static native void cvCanny( @Const CvArr image, CvArr edges, double threshold1,
                      double threshold2 );

/* Calculates constraint image for corner detection
   Dx^2 * Dyy + Dxx * Dy^2 - 2 * Dx * Dy * Dxy.
   Applying threshold to the result gives coordinates of corners */
public static native void cvPreCornerDetect( @Const CvArr image, CvArr corners,
                               int aperture_size/*=3*/ );
public static native void cvPreCornerDetect( @Const CvArr image, CvArr corners );

/* Calculates eigen values and vectors of 2x2
   gradient covariation matrix at every image pixel */
public static native void cvCornerEigenValsAndVecs( @Const CvArr image, CvArr eigenvv,
                                       int block_size, int aperture_size/*=3*/ );
public static native void cvCornerEigenValsAndVecs( @Const CvArr image, CvArr eigenvv,
                                       int block_size );

/* Calculates minimal eigenvalue for 2x2 gradient covariation matrix at
   every image pixel */
public static native void cvCornerMinEigenVal( @Const CvArr image, CvArr eigenval,
                                  int block_size, int aperture_size/*=3*/ );
public static native void cvCornerMinEigenVal( @Const CvArr image, CvArr eigenval,
                                  int block_size );

/* Harris corner detector:
   Calculates det(M) - k*(trace(M)^2), where M is 2x2 gradient covariation matrix for each pixel */
public static native void cvCornerHarris( @Const CvArr image, CvArr harris_response,
                             int block_size, int aperture_size/*=3*/,
                             double k/*=0.04*/ );
public static native void cvCornerHarris( @Const CvArr image, CvArr harris_response,
                             int block_size );

/* Adjust corner position using some sort of gradient search */
public static native void cvFindCornerSubPix( @Const CvArr image, CvPoint2D32f corners,
                                 int count, @ByVal CvSize win, @ByVal CvSize zero_zone,
                                 @ByVal CvTermCriteria criteria );
public static native void cvFindCornerSubPix( @Const CvArr image, @Cast("CvPoint2D32f*") FloatBuffer corners,
                                 int count, @ByVal CvSize win, @ByVal CvSize zero_zone,
                                 @ByVal CvTermCriteria criteria );
public static native void cvFindCornerSubPix( @Const CvArr image, @Cast("CvPoint2D32f*") float[] corners,
                                 int count, @ByVal CvSize win, @ByVal CvSize zero_zone,
                                 @ByVal CvTermCriteria criteria );

/* Finds a sparse set of points within the selected region
   that seem to be easy to track */
public static native void cvGoodFeaturesToTrack( @Const CvArr image, CvArr eig_image,
                                    CvArr temp_image, CvPoint2D32f corners,
                                    IntPointer corner_count, double quality_level,
                                    double min_distance,
                                    @Const CvArr mask/*=NULL*/,
                                    int block_size/*=3*/,
                                    int use_harris/*=0*/,
                                    double k/*=0.04*/ );
public static native void cvGoodFeaturesToTrack( @Const CvArr image, CvArr eig_image,
                                    CvArr temp_image, CvPoint2D32f corners,
                                    IntPointer corner_count, double quality_level,
                                    double min_distance );
public static native void cvGoodFeaturesToTrack( @Const CvArr image, CvArr eig_image,
                                    CvArr temp_image, @Cast("CvPoint2D32f*") FloatBuffer corners,
                                    IntBuffer corner_count, double quality_level,
                                    double min_distance,
                                    @Const CvArr mask/*=NULL*/,
                                    int block_size/*=3*/,
                                    int use_harris/*=0*/,
                                    double k/*=0.04*/ );
public static native void cvGoodFeaturesToTrack( @Const CvArr image, CvArr eig_image,
                                    CvArr temp_image, @Cast("CvPoint2D32f*") FloatBuffer corners,
                                    IntBuffer corner_count, double quality_level,
                                    double min_distance );
public static native void cvGoodFeaturesToTrack( @Const CvArr image, CvArr eig_image,
                                    CvArr temp_image, @Cast("CvPoint2D32f*") float[] corners,
                                    int[] corner_count, double quality_level,
                                    double min_distance,
                                    @Const CvArr mask/*=NULL*/,
                                    int block_size/*=3*/,
                                    int use_harris/*=0*/,
                                    double k/*=0.04*/ );
public static native void cvGoodFeaturesToTrack( @Const CvArr image, CvArr eig_image,
                                    CvArr temp_image, @Cast("CvPoint2D32f*") float[] corners,
                                    int[] corner_count, double quality_level,
                                    double min_distance );

/* Finds lines on binary image using one of several methods.
   line_storage is either memory storage or 1 x <max number of lines> CvMat, its
   number of columns is changed by the function.
   method is one of CV_HOUGH_*;
   rho, theta and threshold are used for each of those methods;
   param1 ~ line length, param2 ~ line gap - for probabilistic,
   param1 ~ srn, param2 ~ stn - for multi-scale */
public static native CvSeq cvHoughLines2( CvArr image, Pointer line_storage, int method,
                              double rho, double theta, int threshold,
                              double param1/*=0*/, double param2/*=0*/,
                              double min_theta/*=0*/, double max_theta/*=CV_PI*/);
public static native CvSeq cvHoughLines2( CvArr image, Pointer line_storage, int method,
                              double rho, double theta, int threshold);

/* Finds circles in the image */
public static native CvSeq cvHoughCircles( CvArr image, Pointer circle_storage,
                              int method, double dp, double min_dist,
                              double param1/*=100*/,
                              double param2/*=100*/,
                              int min_radius/*=0*/,
                              int max_radius/*=0*/);
public static native CvSeq cvHoughCircles( CvArr image, Pointer circle_storage,
                              int method, double dp, double min_dist);

/* Fits a line into set of 2d or 3d points in a robust way (M-estimator technique) */
public static native void cvFitLine( @Const CvArr points, int dist_type, double param,
                        double reps, double aeps, FloatPointer line );
public static native void cvFitLine( @Const CvArr points, int dist_type, double param,
                        double reps, double aeps, FloatBuffer line );
public static native void cvFitLine( @Const CvArr points, int dist_type, double param,
                        double reps, double aeps, float[] line );

/****************************************************************************************\
*                                     Drawing                                            *
\****************************************************************************************/

/****************************************************************************************\
*       Drawing functions work with images/matrices of arbitrary type.                   *
*       For color images the channel order is BGR[A]                                     *
*       Antialiasing is supported only for 8-bit image now.                              *
*       All the functions include parameter color that means rgb value (that may be      *
*       constructed with CV_RGB macro) for color images and brightness                   *
*       for grayscale images.                                                            *
*       If a drawn figure is partially or completely outside of the image, it is clipped.*
\****************************************************************************************/

// #define CV_RGB( r, g, b )  cvScalar( (b), (g), (r), 0 )
public static final int CV_FILLED = -1;

public static final int CV_AA = 16;

/* Draws 4-connected, 8-connected or antialiased line segment connecting two points */
public static native void cvLine( CvArr img, @ByVal CvPoint pt1, @ByVal CvPoint pt2,
                     @ByVal CvScalar color, int thickness/*=1*/,
                     int line_type/*=8*/, int shift/*=0*/ );
public static native void cvLine( CvArr img, @ByVal CvPoint pt1, @ByVal CvPoint pt2,
                     @ByVal CvScalar color );
public static native void cvLine( CvArr img, @ByVal @Cast("CvPoint*") IntBuffer pt1, @ByVal @Cast("CvPoint*") IntBuffer pt2,
                     @ByVal CvScalar color, int thickness/*=1*/,
                     int line_type/*=8*/, int shift/*=0*/ );
public static native void cvLine( CvArr img, @ByVal @Cast("CvPoint*") IntBuffer pt1, @ByVal @Cast("CvPoint*") IntBuffer pt2,
                     @ByVal CvScalar color );
public static native void cvLine( CvArr img, @ByVal @Cast("CvPoint*") int[] pt1, @ByVal @Cast("CvPoint*") int[] pt2,
                     @ByVal CvScalar color, int thickness/*=1*/,
                     int line_type/*=8*/, int shift/*=0*/ );
public static native void cvLine( CvArr img, @ByVal @Cast("CvPoint*") int[] pt1, @ByVal @Cast("CvPoint*") int[] pt2,
                     @ByVal CvScalar color );

/* Draws a rectangle given two opposite corners of the rectangle (pt1 & pt2),
   if thickness<0 (e.g. thickness == CV_FILLED), the filled box is drawn */
public static native void cvRectangle( CvArr img, @ByVal CvPoint pt1, @ByVal CvPoint pt2,
                          @ByVal CvScalar color, int thickness/*=1*/,
                          int line_type/*=8*/,
                          int shift/*=0*/);
public static native void cvRectangle( CvArr img, @ByVal CvPoint pt1, @ByVal CvPoint pt2,
                          @ByVal CvScalar color);
public static native void cvRectangle( CvArr img, @ByVal @Cast("CvPoint*") IntBuffer pt1, @ByVal @Cast("CvPoint*") IntBuffer pt2,
                          @ByVal CvScalar color, int thickness/*=1*/,
                          int line_type/*=8*/,
                          int shift/*=0*/);
public static native void cvRectangle( CvArr img, @ByVal @Cast("CvPoint*") IntBuffer pt1, @ByVal @Cast("CvPoint*") IntBuffer pt2,
                          @ByVal CvScalar color);
public static native void cvRectangle( CvArr img, @ByVal @Cast("CvPoint*") int[] pt1, @ByVal @Cast("CvPoint*") int[] pt2,
                          @ByVal CvScalar color, int thickness/*=1*/,
                          int line_type/*=8*/,
                          int shift/*=0*/);
public static native void cvRectangle( CvArr img, @ByVal @Cast("CvPoint*") int[] pt1, @ByVal @Cast("CvPoint*") int[] pt2,
                          @ByVal CvScalar color);

/* Draws a rectangle specified by a CvRect structure */
public static native void cvRectangleR( CvArr img, @ByVal CvRect r,
                           @ByVal CvScalar color, int thickness/*=1*/,
                           int line_type/*=8*/,
                           int shift/*=0*/);
public static native void cvRectangleR( CvArr img, @ByVal CvRect r,
                           @ByVal CvScalar color);


/* Draws a circle with specified center and radius.
   Thickness works in the same way as with cvRectangle */
public static native void cvCircle( CvArr img, @ByVal CvPoint center, int radius,
                       @ByVal CvScalar color, int thickness/*=1*/,
                       int line_type/*=8*/, int shift/*=0*/);
public static native void cvCircle( CvArr img, @ByVal CvPoint center, int radius,
                       @ByVal CvScalar color);
public static native void cvCircle( CvArr img, @ByVal @Cast("CvPoint*") IntBuffer center, int radius,
                       @ByVal CvScalar color, int thickness/*=1*/,
                       int line_type/*=8*/, int shift/*=0*/);
public static native void cvCircle( CvArr img, @ByVal @Cast("CvPoint*") IntBuffer center, int radius,
                       @ByVal CvScalar color);
public static native void cvCircle( CvArr img, @ByVal @Cast("CvPoint*") int[] center, int radius,
                       @ByVal CvScalar color, int thickness/*=1*/,
                       int line_type/*=8*/, int shift/*=0*/);
public static native void cvCircle( CvArr img, @ByVal @Cast("CvPoint*") int[] center, int radius,
                       @ByVal CvScalar color);

/* Draws ellipse outline, filled ellipse, elliptic arc or filled elliptic sector,
   depending on <thickness>, <start_angle> and <end_angle> parameters. The resultant figure
   is rotated by <angle>. All the angles are in degrees */
public static native void cvEllipse( CvArr img, @ByVal CvPoint center, @ByVal CvSize axes,
                        double angle, double start_angle, double end_angle,
                        @ByVal CvScalar color, int thickness/*=1*/,
                        int line_type/*=8*/, int shift/*=0*/);
public static native void cvEllipse( CvArr img, @ByVal CvPoint center, @ByVal CvSize axes,
                        double angle, double start_angle, double end_angle,
                        @ByVal CvScalar color);
public static native void cvEllipse( CvArr img, @ByVal @Cast("CvPoint*") IntBuffer center, @ByVal CvSize axes,
                        double angle, double start_angle, double end_angle,
                        @ByVal CvScalar color, int thickness/*=1*/,
                        int line_type/*=8*/, int shift/*=0*/);
public static native void cvEllipse( CvArr img, @ByVal @Cast("CvPoint*") IntBuffer center, @ByVal CvSize axes,
                        double angle, double start_angle, double end_angle,
                        @ByVal CvScalar color);
public static native void cvEllipse( CvArr img, @ByVal @Cast("CvPoint*") int[] center, @ByVal CvSize axes,
                        double angle, double start_angle, double end_angle,
                        @ByVal CvScalar color, int thickness/*=1*/,
                        int line_type/*=8*/, int shift/*=0*/);
public static native void cvEllipse( CvArr img, @ByVal @Cast("CvPoint*") int[] center, @ByVal CvSize axes,
                        double angle, double start_angle, double end_angle,
                        @ByVal CvScalar color);

public static native void cvEllipseBox( CvArr img, @ByVal CvBox2D box, @ByVal CvScalar color,
                               int thickness/*=1*/,
                               int line_type/*=8*/, int shift/*=0*/ );
public static native void cvEllipseBox( CvArr img, @ByVal CvBox2D box, @ByVal CvScalar color );

/* Fills convex or monotonous polygon. */
public static native void cvFillConvexPoly( CvArr img, @Const CvPoint pts, int npts, @ByVal CvScalar color,
                               int line_type/*=8*/, int shift/*=0*/);
public static native void cvFillConvexPoly( CvArr img, @Const CvPoint pts, int npts, @ByVal CvScalar color);
public static native void cvFillConvexPoly( CvArr img, @Cast("const CvPoint*") IntBuffer pts, int npts, @ByVal CvScalar color,
                               int line_type/*=8*/, int shift/*=0*/);
public static native void cvFillConvexPoly( CvArr img, @Cast("const CvPoint*") IntBuffer pts, int npts, @ByVal CvScalar color);
public static native void cvFillConvexPoly( CvArr img, @Cast("const CvPoint*") int[] pts, int npts, @ByVal CvScalar color,
                               int line_type/*=8*/, int shift/*=0*/);
public static native void cvFillConvexPoly( CvArr img, @Cast("const CvPoint*") int[] pts, int npts, @ByVal CvScalar color);

/* Fills an area bounded by one or more arbitrary polygons */
public static native void cvFillPoly( CvArr img, @Cast("CvPoint**") PointerPointer pts, @Const IntPointer npts,
                         int contours, @ByVal CvScalar color,
                         int line_type/*=8*/, int shift/*=0*/ );
public static native void cvFillPoly( CvArr img, @ByPtrPtr CvPoint pts, @Const IntPointer npts,
                         int contours, @ByVal CvScalar color );
public static native void cvFillPoly( CvArr img, @ByPtrPtr CvPoint pts, @Const IntPointer npts,
                         int contours, @ByVal CvScalar color,
                         int line_type/*=8*/, int shift/*=0*/ );
public static native void cvFillPoly( CvArr img, @Cast("CvPoint**") @ByPtrPtr IntBuffer pts, @Const IntBuffer npts,
                         int contours, @ByVal CvScalar color,
                         int line_type/*=8*/, int shift/*=0*/ );
public static native void cvFillPoly( CvArr img, @Cast("CvPoint**") @ByPtrPtr IntBuffer pts, @Const IntBuffer npts,
                         int contours, @ByVal CvScalar color );
public static native void cvFillPoly( CvArr img, @Cast("CvPoint**") @ByPtrPtr int[] pts, @Const int[] npts,
                         int contours, @ByVal CvScalar color,
                         int line_type/*=8*/, int shift/*=0*/ );
public static native void cvFillPoly( CvArr img, @Cast("CvPoint**") @ByPtrPtr int[] pts, @Const int[] npts,
                         int contours, @ByVal CvScalar color );

/* Draws one or more polygonal curves */
public static native void cvPolyLine( CvArr img, @Cast("CvPoint**") PointerPointer pts, @Const IntPointer npts, int contours,
                         int is_closed, @ByVal CvScalar color, int thickness/*=1*/,
                         int line_type/*=8*/, int shift/*=0*/ );
public static native void cvPolyLine( CvArr img, @ByPtrPtr CvPoint pts, @Const IntPointer npts, int contours,
                         int is_closed, @ByVal CvScalar color );
public static native void cvPolyLine( CvArr img, @ByPtrPtr CvPoint pts, @Const IntPointer npts, int contours,
                         int is_closed, @ByVal CvScalar color, int thickness/*=1*/,
                         int line_type/*=8*/, int shift/*=0*/ );
public static native void cvPolyLine( CvArr img, @Cast("CvPoint**") @ByPtrPtr IntBuffer pts, @Const IntBuffer npts, int contours,
                         int is_closed, @ByVal CvScalar color, int thickness/*=1*/,
                         int line_type/*=8*/, int shift/*=0*/ );
public static native void cvPolyLine( CvArr img, @Cast("CvPoint**") @ByPtrPtr IntBuffer pts, @Const IntBuffer npts, int contours,
                         int is_closed, @ByVal CvScalar color );
public static native void cvPolyLine( CvArr img, @Cast("CvPoint**") @ByPtrPtr int[] pts, @Const int[] npts, int contours,
                         int is_closed, @ByVal CvScalar color, int thickness/*=1*/,
                         int line_type/*=8*/, int shift/*=0*/ );
public static native void cvPolyLine( CvArr img, @Cast("CvPoint**") @ByPtrPtr int[] pts, @Const int[] npts, int contours,
                         int is_closed, @ByVal CvScalar color );

public static native void cvDrawRect(CvArr arg1, @ByVal CvPoint arg2, @ByVal CvPoint arg3, @ByVal CvScalar arg4, int arg5, int arg6, int arg7);
public static native void cvDrawRect(CvArr arg1, @ByVal @Cast("CvPoint*") IntBuffer arg2, @ByVal @Cast("CvPoint*") IntBuffer arg3, @ByVal CvScalar arg4, int arg5, int arg6, int arg7);
public static native void cvDrawRect(CvArr arg1, @ByVal @Cast("CvPoint*") int[] arg2, @ByVal @Cast("CvPoint*") int[] arg3, @ByVal CvScalar arg4, int arg5, int arg6, int arg7);
public static native void cvDrawLine(CvArr arg1, @ByVal CvPoint arg2, @ByVal CvPoint arg3, @ByVal CvScalar arg4, int arg5, int arg6, int arg7);
public static native void cvDrawLine(CvArr arg1, @ByVal @Cast("CvPoint*") IntBuffer arg2, @ByVal @Cast("CvPoint*") IntBuffer arg3, @ByVal CvScalar arg4, int arg5, int arg6, int arg7);
public static native void cvDrawLine(CvArr arg1, @ByVal @Cast("CvPoint*") int[] arg2, @ByVal @Cast("CvPoint*") int[] arg3, @ByVal CvScalar arg4, int arg5, int arg6, int arg7);
public static native void cvDrawCircle(CvArr arg1, @ByVal CvPoint arg2, int arg3, @ByVal CvScalar arg4, int arg5, int arg6, int arg7);
public static native void cvDrawCircle(CvArr arg1, @ByVal @Cast("CvPoint*") IntBuffer arg2, int arg3, @ByVal CvScalar arg4, int arg5, int arg6, int arg7);
public static native void cvDrawCircle(CvArr arg1, @ByVal @Cast("CvPoint*") int[] arg2, int arg3, @ByVal CvScalar arg4, int arg5, int arg6, int arg7);
public static native void cvDrawEllipse(CvArr arg1, @ByVal CvPoint arg2, @ByVal CvSize arg3, double arg4, double arg5, double arg6, @ByVal CvScalar arg7, int arg8, int arg9, int arg10);
public static native void cvDrawEllipse(CvArr arg1, @ByVal @Cast("CvPoint*") IntBuffer arg2, @ByVal CvSize arg3, double arg4, double arg5, double arg6, @ByVal CvScalar arg7, int arg8, int arg9, int arg10);
public static native void cvDrawEllipse(CvArr arg1, @ByVal @Cast("CvPoint*") int[] arg2, @ByVal CvSize arg3, double arg4, double arg5, double arg6, @ByVal CvScalar arg7, int arg8, int arg9, int arg10);
public static native void cvDrawPolyLine(CvArr arg1, @Cast("CvPoint**") PointerPointer arg2, IntPointer arg3, int arg4, int arg5, @ByVal CvScalar arg6, int arg7, int arg8, int arg9);
public static native void cvDrawPolyLine(CvArr arg1, @ByPtrPtr CvPoint arg2, IntPointer arg3, int arg4, int arg5, @ByVal CvScalar arg6, int arg7, int arg8, int arg9);
public static native void cvDrawPolyLine(CvArr arg1, @Cast("CvPoint**") @ByPtrPtr IntBuffer arg2, IntBuffer arg3, int arg4, int arg5, @ByVal CvScalar arg6, int arg7, int arg8, int arg9);
public static native void cvDrawPolyLine(CvArr arg1, @Cast("CvPoint**") @ByPtrPtr int[] arg2, int[] arg3, int arg4, int arg5, @ByVal CvScalar arg6, int arg7, int arg8, int arg9);

/* Clips the line segment connecting *pt1 and *pt2
   by the rectangular window
   (0<=x<img_size.width, 0<=y<img_size.height). */
public static native int cvClipLine( @ByVal CvSize img_size, CvPoint pt1, CvPoint pt2 );
public static native int cvClipLine( @ByVal CvSize img_size, @Cast("CvPoint*") IntBuffer pt1, @Cast("CvPoint*") IntBuffer pt2 );
public static native int cvClipLine( @ByVal CvSize img_size, @Cast("CvPoint*") int[] pt1, @Cast("CvPoint*") int[] pt2 );

/* Initializes line iterator. Initially, line_iterator->ptr will point
   to pt1 (or pt2, see left_to_right description) location in the image.
   Returns the number of pixels on the line between the ending points. */
public static native int cvInitLineIterator( @Const CvArr image, @ByVal CvPoint pt1, @ByVal CvPoint pt2,
                                CvLineIterator line_iterator,
                                int connectivity/*=8*/,
                                int left_to_right/*=0*/);
public static native int cvInitLineIterator( @Const CvArr image, @ByVal CvPoint pt1, @ByVal CvPoint pt2,
                                CvLineIterator line_iterator);
public static native int cvInitLineIterator( @Const CvArr image, @ByVal @Cast("CvPoint*") IntBuffer pt1, @ByVal @Cast("CvPoint*") IntBuffer pt2,
                                CvLineIterator line_iterator,
                                int connectivity/*=8*/,
                                int left_to_right/*=0*/);
public static native int cvInitLineIterator( @Const CvArr image, @ByVal @Cast("CvPoint*") IntBuffer pt1, @ByVal @Cast("CvPoint*") IntBuffer pt2,
                                CvLineIterator line_iterator);
public static native int cvInitLineIterator( @Const CvArr image, @ByVal @Cast("CvPoint*") int[] pt1, @ByVal @Cast("CvPoint*") int[] pt2,
                                CvLineIterator line_iterator,
                                int connectivity/*=8*/,
                                int left_to_right/*=0*/);
public static native int cvInitLineIterator( @Const CvArr image, @ByVal @Cast("CvPoint*") int[] pt1, @ByVal @Cast("CvPoint*") int[] pt2,
                                CvLineIterator line_iterator);

/* Moves iterator to the next line point */
// #define CV_NEXT_LINE_POINT( line_iterator )
// {
//     int _line_iterator_mask = (line_iterator).err < 0 ? -1 : 0;
//     (line_iterator).err += (line_iterator).minus_delta +
//         ((line_iterator).plus_delta & _line_iterator_mask);
//     (line_iterator).ptr += (line_iterator).minus_step +
//         ((line_iterator).plus_step & _line_iterator_mask);
// }


/* basic font types */
public static final int CV_FONT_HERSHEY_SIMPLEX =         0;
public static final int CV_FONT_HERSHEY_PLAIN =           1;
public static final int CV_FONT_HERSHEY_DUPLEX =          2;
public static final int CV_FONT_HERSHEY_COMPLEX =         3;
public static final int CV_FONT_HERSHEY_TRIPLEX =         4;
public static final int CV_FONT_HERSHEY_COMPLEX_SMALL =   5;
public static final int CV_FONT_HERSHEY_SCRIPT_SIMPLEX =  6;
public static final int CV_FONT_HERSHEY_SCRIPT_COMPLEX =  7;

/* font flags */
public static final int CV_FONT_ITALIC =                 16;

public static final int CV_FONT_VECTOR0 =    CV_FONT_HERSHEY_SIMPLEX;


/* Font structure */
public static class CvFont extends AbstractCvFont {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvFont() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvFont(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvFont(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvFont position(int position) {
        return (CvFont)super.position(position);
    }

  @MemberGetter public native @Cast("const char*") BytePointer nameFont();   //Qt:nameFont
  public native @ByRef CvScalar color(); public native CvFont color(CvScalar color);       //Qt:ColorFont -> cvScalar(blue_component, green_component, red\_component[, alpha_component])
    public native int font_face(); public native CvFont font_face(int font_face);    //Qt: bool italic         /* =CV_FONT_* */
    @MemberGetter public native @Const IntPointer ascii();      /* font data and metrics */
    @MemberGetter public native @Const IntPointer greek();
    @MemberGetter public native @Const IntPointer cyrillic();
    public native float hscale(); public native CvFont hscale(float hscale);
    public native float vscale(); public native CvFont vscale(float vscale);
    public native float shear(); public native CvFont shear(float shear);      /* slope coefficient: 0 - normal, >0 - italic */
    public native int thickness(); public native CvFont thickness(int thickness);    //Qt: weight               /* letters thickness */
    public native float dx(); public native CvFont dx(float dx);       /* horizontal interval between letters */
    public native int line_type(); public native CvFont line_type(int line_type);    //Qt: PointSize
}

/* Initializes font structure used further in cvPutText */
public static native void cvInitFont( CvFont font, int font_face,
                         double hscale, double vscale,
                         double shear/*=0*/,
                         int thickness/*=1*/,
                         int line_type/*=8*/);
public static native void cvInitFont( CvFont font, int font_face,
                         double hscale, double vscale);

public static native @ByVal CvFont cvFont( double scale, int thickness/*=1*/ );
public static native @ByVal CvFont cvFont( double scale );

/* Renders text stroke with specified font and color at specified location.
   CvFont should be initialized with cvInitFont */
public static native void cvPutText( CvArr img, @Cast("const char*") BytePointer text, @ByVal CvPoint org,
                        @Const CvFont font, @ByVal CvScalar color );
public static native void cvPutText( CvArr img, String text, @ByVal @Cast("CvPoint*") IntBuffer org,
                        @Const CvFont font, @ByVal CvScalar color );
public static native void cvPutText( CvArr img, @Cast("const char*") BytePointer text, @ByVal @Cast("CvPoint*") int[] org,
                        @Const CvFont font, @ByVal CvScalar color );
public static native void cvPutText( CvArr img, String text, @ByVal CvPoint org,
                        @Const CvFont font, @ByVal CvScalar color );
public static native void cvPutText( CvArr img, @Cast("const char*") BytePointer text, @ByVal @Cast("CvPoint*") IntBuffer org,
                        @Const CvFont font, @ByVal CvScalar color );
public static native void cvPutText( CvArr img, String text, @ByVal @Cast("CvPoint*") int[] org,
                        @Const CvFont font, @ByVal CvScalar color );

/* Calculates bounding box of text stroke (useful for alignment) */
public static native void cvGetTextSize( @Cast("const char*") BytePointer text_string, @Const CvFont font,
                            CvSize text_size, IntPointer baseline );
public static native void cvGetTextSize( String text_string, @Const CvFont font,
                            CvSize text_size, IntBuffer baseline );
public static native void cvGetTextSize( @Cast("const char*") BytePointer text_string, @Const CvFont font,
                            CvSize text_size, int[] baseline );
public static native void cvGetTextSize( String text_string, @Const CvFont font,
                            CvSize text_size, IntPointer baseline );
public static native void cvGetTextSize( @Cast("const char*") BytePointer text_string, @Const CvFont font,
                            CvSize text_size, IntBuffer baseline );
public static native void cvGetTextSize( String text_string, @Const CvFont font,
                            CvSize text_size, int[] baseline );

/* Unpacks color value, if arrtype is CV_8UC?, <color> is treated as
   packed color value, otherwise the first channels (depending on arrtype)
   of destination scalar are set to the same value = <color> */
public static native @ByVal CvScalar cvColorToScalar( double packed_color, int arrtype );

/* Returns the polygon points which make up the given ellipse.  The ellipse is define by
   the box of size 'axes' rotated 'angle' around the 'center'.  A partial sweep
   of the ellipse arc can be done by spcifying arc_start and arc_end to be something
   other than 0 and 360, respectively.  The input array 'pts' must be large enough to
   hold the result.  The total number of points stored into 'pts' is returned by this
   function. */
public static native int cvEllipse2Poly( @ByVal CvPoint center, @ByVal CvSize axes,
                 int angle, int arc_start, int arc_end, CvPoint pts, int delta );
public static native int cvEllipse2Poly( @ByVal @Cast("CvPoint*") IntBuffer center, @ByVal CvSize axes,
                 int angle, int arc_start, int arc_end, @Cast("CvPoint*") IntBuffer pts, int delta );
public static native int cvEllipse2Poly( @ByVal @Cast("CvPoint*") int[] center, @ByVal CvSize axes,
                 int angle, int arc_start, int arc_end, @Cast("CvPoint*") int[] pts, int delta );

/* Draws contour outlines or filled interiors on the image */
public static native void cvDrawContours( CvArr img, CvSeq contour,
                             @ByVal CvScalar external_color, @ByVal CvScalar hole_color,
                             int max_level, int thickness/*=1*/,
                             int line_type/*=8*/,
                             @ByVal CvPoint offset/*=cvPoint(0,0)*/);
public static native void cvDrawContours( CvArr img, CvSeq contour,
                             @ByVal CvScalar external_color, @ByVal CvScalar hole_color,
                             int max_level);
public static native void cvDrawContours( CvArr img, CvSeq contour,
                             @ByVal CvScalar external_color, @ByVal CvScalar hole_color,
                             int max_level, int thickness/*=1*/,
                             int line_type/*=8*/,
                             @ByVal @Cast("CvPoint*") IntBuffer offset/*=cvPoint(0,0)*/);
public static native void cvDrawContours( CvArr img, CvSeq contour,
                             @ByVal CvScalar external_color, @ByVal CvScalar hole_color,
                             int max_level, int thickness/*=1*/,
                             int line_type/*=8*/,
                             @ByVal @Cast("CvPoint*") int[] offset/*=cvPoint(0,0)*/);

// #ifdef __cplusplus
// #endif

// #endif


// Parsed from <opencv2/imgproc.hpp>

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
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

// #ifndef __OPENCV_IMGPROC_HPP__
// #define __OPENCV_IMGPROC_HPP__

// #include "opencv2/core.hpp"

/** \namespace cv
 Namespace where all the C++ OpenCV functionality resides
 */

/** type of morphological operation */
/** enum cv:: */
public static final int MORPH_ERODE    = 0,
       MORPH_DILATE   = 1,
       MORPH_OPEN     = 2,
       MORPH_CLOSE    = 3,
       MORPH_GRADIENT = 4,
       MORPH_TOPHAT   = 5,
       MORPH_BLACKHAT = 6;

/** shape of the structuring element */
/** enum cv:: */
public static final int MORPH_RECT    = 0,
       MORPH_CROSS   = 1,
       MORPH_ELLIPSE = 2;

/** interpolation algorithm */
/** enum cv:: */
public static final int /** nearest neighbor interpolation */
 INTER_NEAREST        = 0,
       /** bilinear interpolation */
       INTER_LINEAR         = 1,
       /** bicubic interpolation */
       INTER_CUBIC          = 2,
       /** area-based (or super) interpolation */
       INTER_AREA           = 3,
       /** Lanczos interpolation over 8x8 neighborhood */
       INTER_LANCZOS4       = 4,

       /** mask for interpolation codes */
       INTER_MAX            = 7,
       WARP_FILL_OUTLIERS   = 8,
       WARP_INVERSE_MAP     = 16;

/** enum cv:: */
public static final int INTER_BITS      = 5,
       INTER_BITS2     =  INTER_BITS * 2,
       INTER_TAB_SIZE  =  1 << INTER_BITS,
       INTER_TAB_SIZE2 =  INTER_TAB_SIZE * INTER_TAB_SIZE;

/** Distance types for Distance Transform and M-estimators */
/** enum cv:: */
public static final int DIST_USER    = -1,  // User defined distance
       DIST_L1      = 1,   // distance = |x1-x2| + |y1-y2|
       DIST_L2      = 2,   // the simple euclidean distance
       DIST_C       = 3,   // distance = max(|x1-x2|,|y1-y2|)
       DIST_L12     = 4,   // L1-L2 metric: distance = 2(sqrt(1+x*x/2) - 1))
       DIST_FAIR    = 5,   // distance = c^2(|x|/c-log(1+|x|/c)), c = 1.3998
       DIST_WELSCH  = 6,   // distance = c^2/2(1-exp(-(x/c)^2)), c = 2.9846
       DIST_HUBER   = 7;    // distance = |x|<c ? x^2/2 : c(|x|-c/2), c=1.345

/** Mask size for distance transform */
/** enum cv:: */
public static final int DIST_MASK_3       = 3,
       DIST_MASK_5       = 5,
       DIST_MASK_PRECISE = 0;

/** type of the threshold operation */
/** enum cv:: */
public static final int THRESH_BINARY     = 0, // value = value > threshold ? max_value : 0
       THRESH_BINARY_INV = 1, // value = value > threshold ? 0 : max_value
       THRESH_TRUNC      = 2, // value = value > threshold ? threshold : value
       THRESH_TOZERO     = 3, // value = value > threshold ? value : 0
       THRESH_TOZERO_INV = 4, // value = value > threshold ? 0 : value
       THRESH_MASK       = 7,
       THRESH_OTSU       = 8, // use Otsu algorithm to choose the optimal threshold value
       THRESH_TRIANGLE   = 16;  // use Triangle algorithm to choose the optimal threshold value

/** adaptive threshold algorithm */
/** enum cv:: */
public static final int ADAPTIVE_THRESH_MEAN_C     = 0,
       ADAPTIVE_THRESH_GAUSSIAN_C = 1;

/** enum cv:: */
public static final int PROJ_SPHERICAL_ORTHO  = 0,
       PROJ_SPHERICAL_EQRECT = 1;

/** class of the pixel in GrabCut algorithm */
/** enum cv:: */
public static final int /** background */
 GC_BGD    = 0,
       /** foreground */
       GC_FGD    = 1,
       /** most probably background */
       GC_PR_BGD = 2,
       /** most probably foreground */
       GC_PR_FGD = 3;

/** GrabCut algorithm flags */
/** enum cv:: */
public static final int GC_INIT_WITH_RECT  = 0,
       GC_INIT_WITH_MASK  = 1,
       GC_EVAL            = 2;

/** distanceTransform algorithm flags */
/** enum cv:: */
public static final int DIST_LABEL_CCOMP = 0,
       DIST_LABEL_PIXEL = 1;

/** floodfill algorithm flags */
/** enum cv:: */
public static final int FLOODFILL_FIXED_RANGE =  1 << 16,
       FLOODFILL_MASK_ONLY   =  1 << 17;

/** type of the template matching operation */
/** enum cv:: */
public static final int TM_SQDIFF        = 0,
       TM_SQDIFF_NORMED = 1,
       TM_CCORR         = 2,
       TM_CCORR_NORMED  = 3,
       TM_CCOEFF        = 4,
       TM_CCOEFF_NORMED = 5;

/** connected components algorithm output formats */
/** enum cv:: */
public static final int CC_STAT_LEFT   = 0,
       CC_STAT_TOP    = 1,
       CC_STAT_WIDTH  = 2,
       CC_STAT_HEIGHT = 3,
       CC_STAT_AREA   = 4,
       CC_STAT_MAX    = 5;

/** mode of the contour retrieval algorithm */
/** enum cv:: */
public static final int /** retrieve only the most external (top-level) contours */
 RETR_EXTERNAL  = 0,
       /** retrieve all the contours without any hierarchical information */
       RETR_LIST      = 1,
       /** retrieve the connected components (that can possibly be nested) */
       RETR_CCOMP     = 2,
       /** retrieve all the contours and the whole hierarchy */
       RETR_TREE      = 3,
       RETR_FLOODFILL = 4;

/** the contour approximation algorithm */
/** enum cv:: */
public static final int CHAIN_APPROX_NONE      = 1,
       CHAIN_APPROX_SIMPLE    = 2,
       CHAIN_APPROX_TC89_L1   = 3,
       CHAIN_APPROX_TC89_KCOS = 4;

/** Variants of a Hough transform */
/** enum cv:: */
public static final int HOUGH_STANDARD      = 0,
       HOUGH_PROBABILISTIC = 1,
       HOUGH_MULTI_SCALE   = 2,
       HOUGH_GRADIENT      = 3;

/** Variants of Line Segment Detector */
/** enum cv:: */
public static final int LSD_REFINE_NONE = 0,
       LSD_REFINE_STD  = 1,
       LSD_REFINE_ADV  = 2;

/** Histogram comparison methods */
/** enum cv:: */
public static final int HISTCMP_CORREL        = 0,
       HISTCMP_CHISQR        = 1,
       HISTCMP_INTERSECT     = 2,
       HISTCMP_BHATTACHARYYA = 3,
       HISTCMP_HELLINGER     =  HISTCMP_BHATTACHARYYA,
       HISTCMP_CHISQR_ALT    = 4,
       HISTCMP_KL_DIV        = 5;

/** the color conversion code */
/** enum cv:: */
public static final int COLOR_BGR2BGRA     = 0,
       COLOR_RGB2RGBA     =  COLOR_BGR2BGRA,

       COLOR_BGRA2BGR     = 1,
       COLOR_RGBA2RGB     =  COLOR_BGRA2BGR,

       COLOR_BGR2RGBA     = 2,
       COLOR_RGB2BGRA     =  COLOR_BGR2RGBA,

       COLOR_RGBA2BGR     = 3,
       COLOR_BGRA2RGB     =  COLOR_RGBA2BGR,

       COLOR_BGR2RGB      = 4,
       COLOR_RGB2BGR      =  COLOR_BGR2RGB,

       COLOR_BGRA2RGBA    = 5,
       COLOR_RGBA2BGRA    =  COLOR_BGRA2RGBA,

       COLOR_BGR2GRAY     = 6,
       COLOR_RGB2GRAY     = 7,
       COLOR_GRAY2BGR     = 8,
       COLOR_GRAY2RGB     =  COLOR_GRAY2BGR,
       COLOR_GRAY2BGRA    = 9,
       COLOR_GRAY2RGBA    =  COLOR_GRAY2BGRA,
       COLOR_BGRA2GRAY    = 10,
       COLOR_RGBA2GRAY    = 11,

       COLOR_BGR2BGR565   = 12,
       COLOR_RGB2BGR565   = 13,
       COLOR_BGR5652BGR   = 14,
       COLOR_BGR5652RGB   = 15,
       COLOR_BGRA2BGR565  = 16,
       COLOR_RGBA2BGR565  = 17,
       COLOR_BGR5652BGRA  = 18,
       COLOR_BGR5652RGBA  = 19,

       COLOR_GRAY2BGR565  = 20,
       COLOR_BGR5652GRAY  = 21,

       COLOR_BGR2BGR555   = 22,
       COLOR_RGB2BGR555   = 23,
       COLOR_BGR5552BGR   = 24,
       COLOR_BGR5552RGB   = 25,
       COLOR_BGRA2BGR555  = 26,
       COLOR_RGBA2BGR555  = 27,
       COLOR_BGR5552BGRA  = 28,
       COLOR_BGR5552RGBA  = 29,

       COLOR_GRAY2BGR555  = 30,
       COLOR_BGR5552GRAY  = 31,

       COLOR_BGR2XYZ      = 32,
       COLOR_RGB2XYZ      = 33,
       COLOR_XYZ2BGR      = 34,
       COLOR_XYZ2RGB      = 35,

       COLOR_BGR2YCrCb    = 36,
       COLOR_RGB2YCrCb    = 37,
       COLOR_YCrCb2BGR    = 38,
       COLOR_YCrCb2RGB    = 39,

       COLOR_BGR2HSV      = 40,
       COLOR_RGB2HSV      = 41,

       COLOR_BGR2Lab      = 44,
       COLOR_RGB2Lab      = 45,

       COLOR_BGR2Luv      = 50,
       COLOR_RGB2Luv      = 51,
       COLOR_BGR2HLS      = 52,
       COLOR_RGB2HLS      = 53,

       COLOR_HSV2BGR      = 54,
       COLOR_HSV2RGB      = 55,

       COLOR_Lab2BGR      = 56,
       COLOR_Lab2RGB      = 57,
       COLOR_Luv2BGR      = 58,
       COLOR_Luv2RGB      = 59,
       COLOR_HLS2BGR      = 60,
       COLOR_HLS2RGB      = 61,

       COLOR_BGR2HSV_FULL = 66,
       COLOR_RGB2HSV_FULL = 67,
       COLOR_BGR2HLS_FULL = 68,
       COLOR_RGB2HLS_FULL = 69,

       COLOR_HSV2BGR_FULL = 70,
       COLOR_HSV2RGB_FULL = 71,
       COLOR_HLS2BGR_FULL = 72,
       COLOR_HLS2RGB_FULL = 73,

       COLOR_LBGR2Lab     = 74,
       COLOR_LRGB2Lab     = 75,
       COLOR_LBGR2Luv     = 76,
       COLOR_LRGB2Luv     = 77,

       COLOR_Lab2LBGR     = 78,
       COLOR_Lab2LRGB     = 79,
       COLOR_Luv2LBGR     = 80,
       COLOR_Luv2LRGB     = 81,

       COLOR_BGR2YUV      = 82,
       COLOR_RGB2YUV      = 83,
       COLOR_YUV2BGR      = 84,
       COLOR_YUV2RGB      = 85,

       // YUV 4:2:0 family to RGB
       COLOR_YUV2RGB_NV12  = 90,
       COLOR_YUV2BGR_NV12  = 91,
       COLOR_YUV2RGB_NV21  = 92,
       COLOR_YUV2BGR_NV21  = 93,
       COLOR_YUV420sp2RGB  =  COLOR_YUV2RGB_NV21,
       COLOR_YUV420sp2BGR  =  COLOR_YUV2BGR_NV21,

       COLOR_YUV2RGBA_NV12 = 94,
       COLOR_YUV2BGRA_NV12 = 95,
       COLOR_YUV2RGBA_NV21 = 96,
       COLOR_YUV2BGRA_NV21 = 97,
       COLOR_YUV420sp2RGBA =  COLOR_YUV2RGBA_NV21,
       COLOR_YUV420sp2BGRA =  COLOR_YUV2BGRA_NV21,

       COLOR_YUV2RGB_YV12  = 98,
       COLOR_YUV2BGR_YV12  = 99,
       COLOR_YUV2RGB_IYUV  = 100,
       COLOR_YUV2BGR_IYUV  = 101,
       COLOR_YUV2RGB_I420  =  COLOR_YUV2RGB_IYUV,
       COLOR_YUV2BGR_I420  =  COLOR_YUV2BGR_IYUV,
       COLOR_YUV420p2RGB   =  COLOR_YUV2RGB_YV12,
       COLOR_YUV420p2BGR   =  COLOR_YUV2BGR_YV12,

       COLOR_YUV2RGBA_YV12 = 102,
       COLOR_YUV2BGRA_YV12 = 103,
       COLOR_YUV2RGBA_IYUV = 104,
       COLOR_YUV2BGRA_IYUV = 105,
       COLOR_YUV2RGBA_I420 =  COLOR_YUV2RGBA_IYUV,
       COLOR_YUV2BGRA_I420 =  COLOR_YUV2BGRA_IYUV,
       COLOR_YUV420p2RGBA  =  COLOR_YUV2RGBA_YV12,
       COLOR_YUV420p2BGRA  =  COLOR_YUV2BGRA_YV12,

       COLOR_YUV2GRAY_420  = 106,
       COLOR_YUV2GRAY_NV21 =  COLOR_YUV2GRAY_420,
       COLOR_YUV2GRAY_NV12 =  COLOR_YUV2GRAY_420,
       COLOR_YUV2GRAY_YV12 =  COLOR_YUV2GRAY_420,
       COLOR_YUV2GRAY_IYUV =  COLOR_YUV2GRAY_420,
       COLOR_YUV2GRAY_I420 =  COLOR_YUV2GRAY_420,
       COLOR_YUV420sp2GRAY =  COLOR_YUV2GRAY_420,
       COLOR_YUV420p2GRAY  =  COLOR_YUV2GRAY_420,

       // YUV 4:2:2 family to RGB
       COLOR_YUV2RGB_UYVY = 107,
       COLOR_YUV2BGR_UYVY = 108,
     //COLOR_YUV2RGB_VYUY = 109,
     //COLOR_YUV2BGR_VYUY = 110,
       COLOR_YUV2RGB_Y422 =  COLOR_YUV2RGB_UYVY,
       COLOR_YUV2BGR_Y422 =  COLOR_YUV2BGR_UYVY,
       COLOR_YUV2RGB_UYNV =  COLOR_YUV2RGB_UYVY,
       COLOR_YUV2BGR_UYNV =  COLOR_YUV2BGR_UYVY,

       COLOR_YUV2RGBA_UYVY = 111,
       COLOR_YUV2BGRA_UYVY = 112,
     //COLOR_YUV2RGBA_VYUY = 113,
     //COLOR_YUV2BGRA_VYUY = 114,
       COLOR_YUV2RGBA_Y422 =  COLOR_YUV2RGBA_UYVY,
       COLOR_YUV2BGRA_Y422 =  COLOR_YUV2BGRA_UYVY,
       COLOR_YUV2RGBA_UYNV =  COLOR_YUV2RGBA_UYVY,
       COLOR_YUV2BGRA_UYNV =  COLOR_YUV2BGRA_UYVY,

       COLOR_YUV2RGB_YUY2 = 115,
       COLOR_YUV2BGR_YUY2 = 116,
       COLOR_YUV2RGB_YVYU = 117,
       COLOR_YUV2BGR_YVYU = 118,
       COLOR_YUV2RGB_YUYV =  COLOR_YUV2RGB_YUY2,
       COLOR_YUV2BGR_YUYV =  COLOR_YUV2BGR_YUY2,
       COLOR_YUV2RGB_YUNV =  COLOR_YUV2RGB_YUY2,
       COLOR_YUV2BGR_YUNV =  COLOR_YUV2BGR_YUY2,

       COLOR_YUV2RGBA_YUY2 = 119,
       COLOR_YUV2BGRA_YUY2 = 120,
       COLOR_YUV2RGBA_YVYU = 121,
       COLOR_YUV2BGRA_YVYU = 122,
       COLOR_YUV2RGBA_YUYV =  COLOR_YUV2RGBA_YUY2,
       COLOR_YUV2BGRA_YUYV =  COLOR_YUV2BGRA_YUY2,
       COLOR_YUV2RGBA_YUNV =  COLOR_YUV2RGBA_YUY2,
       COLOR_YUV2BGRA_YUNV =  COLOR_YUV2BGRA_YUY2,

       COLOR_YUV2GRAY_UYVY = 123,
       COLOR_YUV2GRAY_YUY2 = 124,
     //CV_YUV2GRAY_VYUY    = CV_YUV2GRAY_UYVY,
       COLOR_YUV2GRAY_Y422 =  COLOR_YUV2GRAY_UYVY,
       COLOR_YUV2GRAY_UYNV =  COLOR_YUV2GRAY_UYVY,
       COLOR_YUV2GRAY_YVYU =  COLOR_YUV2GRAY_YUY2,
       COLOR_YUV2GRAY_YUYV =  COLOR_YUV2GRAY_YUY2,
       COLOR_YUV2GRAY_YUNV =  COLOR_YUV2GRAY_YUY2,

       // alpha premultiplication
       COLOR_RGBA2mRGBA    = 125,
       COLOR_mRGBA2RGBA    = 126,

       // RGB to YUV 4:2:0 family
       COLOR_RGB2YUV_I420  = 127,
       COLOR_BGR2YUV_I420  = 128,
       COLOR_RGB2YUV_IYUV  =  COLOR_RGB2YUV_I420,
       COLOR_BGR2YUV_IYUV  =  COLOR_BGR2YUV_I420,

       COLOR_RGBA2YUV_I420 = 129,
       COLOR_BGRA2YUV_I420 = 130,
       COLOR_RGBA2YUV_IYUV =  COLOR_RGBA2YUV_I420,
       COLOR_BGRA2YUV_IYUV =  COLOR_BGRA2YUV_I420,
       COLOR_RGB2YUV_YV12  = 131,
       COLOR_BGR2YUV_YV12  = 132,
       COLOR_RGBA2YUV_YV12 = 133,
       COLOR_BGRA2YUV_YV12 = 134,

       // Demosaicing
       COLOR_BayerBG2BGR = 46,
       COLOR_BayerGB2BGR = 47,
       COLOR_BayerRG2BGR = 48,
       COLOR_BayerGR2BGR = 49,

       COLOR_BayerBG2RGB =  COLOR_BayerRG2BGR,
       COLOR_BayerGB2RGB =  COLOR_BayerGR2BGR,
       COLOR_BayerRG2RGB =  COLOR_BayerBG2BGR,
       COLOR_BayerGR2RGB =  COLOR_BayerGB2BGR,

       COLOR_BayerBG2GRAY = 86,
       COLOR_BayerGB2GRAY = 87,
       COLOR_BayerRG2GRAY = 88,
       COLOR_BayerGR2GRAY = 89,

       // Demosaicing using Variable Number of Gradients
       COLOR_BayerBG2BGR_VNG = 62,
       COLOR_BayerGB2BGR_VNG = 63,
       COLOR_BayerRG2BGR_VNG = 64,
       COLOR_BayerGR2BGR_VNG = 65,

       COLOR_BayerBG2RGB_VNG =  COLOR_BayerRG2BGR_VNG,
       COLOR_BayerGB2RGB_VNG =  COLOR_BayerGR2BGR_VNG,
       COLOR_BayerRG2RGB_VNG =  COLOR_BayerBG2BGR_VNG,
       COLOR_BayerGR2RGB_VNG =  COLOR_BayerGB2BGR_VNG,

       // Edge-Aware Demosaicing
       COLOR_BayerBG2BGR_EA  = 135,
       COLOR_BayerGB2BGR_EA  = 136,
       COLOR_BayerRG2BGR_EA  = 137,
       COLOR_BayerGR2BGR_EA  = 138,

       COLOR_BayerBG2RGB_EA  =  COLOR_BayerRG2BGR_EA,
       COLOR_BayerGB2RGB_EA  =  COLOR_BayerGR2BGR_EA,
       COLOR_BayerRG2RGB_EA  =  COLOR_BayerBG2BGR_EA,
       COLOR_BayerGR2RGB_EA  =  COLOR_BayerGB2BGR_EA,


       COLOR_COLORCVT_MAX  = 139;

/** types of intersection between rectangles */
/** enum cv:: */
public static final int INTERSECT_NONE = 0,
       INTERSECT_PARTIAL  = 1,
       INTERSECT_FULL  = 2;

/** finds arbitrary template in the grayscale image using Generalized Hough Transform */
@Namespace("cv") public static class GeneralizedHough extends Algorithm {
    static { Loader.load(); }
    /** Empty constructor. */
    public GeneralizedHough() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public GeneralizedHough(Pointer p) { super(p); }

    /** set template to search */
    public native void setTemplate(@ByVal Mat templ, @ByVal Point templCenter/*=Point(-1, -1)*/);
    public native void setTemplate(@ByVal Mat templ);
    public native void setTemplate(@ByVal Mat edges, @ByVal Mat dx, @ByVal Mat dy, @ByVal Point templCenter/*=Point(-1, -1)*/);
    public native void setTemplate(@ByVal Mat edges, @ByVal Mat dx, @ByVal Mat dy);

    /** find template on image */
    public native void detect(@ByVal Mat image, @ByVal Mat positions, @ByVal Mat votes/*=noArray()*/);
    public native void detect(@ByVal Mat image, @ByVal Mat positions);
    public native void detect(@ByVal Mat edges, @ByVal Mat dx, @ByVal Mat dy, @ByVal Mat positions, @ByVal Mat votes/*=noArray()*/);
    public native void detect(@ByVal Mat edges, @ByVal Mat dx, @ByVal Mat dy, @ByVal Mat positions);

    /** Canny low threshold. */
    public native void setCannyLowThresh(int cannyLowThresh);
    public native int getCannyLowThresh();

    /** Canny high threshold. */
    public native void setCannyHighThresh(int cannyHighThresh);
    public native int getCannyHighThresh();

    /** Minimum distance between the centers of the detected objects. */
    public native void setMinDist(double minDist);
    public native double getMinDist();

    /** Inverse ratio of the accumulator resolution to the image resolution. */
    public native void setDp(double dp);
    public native double getDp();

    /** Maximal size of inner buffers. */
    public native void setMaxBufferSize(int maxBufferSize);
    public native int getMaxBufferSize();
}

/** Ballard, D.H. (1981). Generalizing the Hough transform to detect arbitrary shapes. Pattern Recognition 13 (2): 111-122.
 *  Detects position only without traslation and rotation */
@Namespace("cv") public static class GeneralizedHoughBallard extends GeneralizedHough {
    static { Loader.load(); }
    /** Empty constructor. */
    public GeneralizedHoughBallard() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public GeneralizedHoughBallard(Pointer p) { super(p); }

    /** R-Table levels. */
    public native void setLevels(int levels);
    public native int getLevels();

    /** The accumulator threshold for the template centers at the detection stage. The smaller it is, the more false positions may be detected. */
    public native void setVotesThreshold(int votesThreshold);
    public native int getVotesThreshold();
}

/** Guil, N., González-Linares, J.M. and Zapata, E.L. (1999). Bidimensional shape detection using an invariant approach. Pattern Recognition 32 (6): 1025-1038.
 *  Detects position, traslation and rotation */
@Namespace("cv") public static class GeneralizedHoughGuil extends GeneralizedHough {
    static { Loader.load(); }
    /** Empty constructor. */
    public GeneralizedHoughGuil() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public GeneralizedHoughGuil(Pointer p) { super(p); }

    /** Angle difference in degrees between two points in feature. */
    public native void setXi(double xi);
    public native double getXi();

    /** Feature table levels. */
    public native void setLevels(int levels);
    public native int getLevels();

    /** Maximal difference between angles that treated as equal. */
    public native void setAngleEpsilon(double angleEpsilon);
    public native double getAngleEpsilon();

    /** Minimal rotation angle to detect in degrees. */
    public native void setMinAngle(double minAngle);
    public native double getMinAngle();

    /** Maximal rotation angle to detect in degrees. */
    public native void setMaxAngle(double maxAngle);
    public native double getMaxAngle();

    /** Angle step in degrees. */
    public native void setAngleStep(double angleStep);
    public native double getAngleStep();

    /** Angle votes threshold. */
    public native void setAngleThresh(int angleThresh);
    public native int getAngleThresh();

    /** Minimal scale to detect. */
    public native void setMinScale(double minScale);
    public native double getMinScale();

    /** Maximal scale to detect. */
    public native void setMaxScale(double maxScale);
    public native double getMaxScale();

    /** Scale step. */
    public native void setScaleStep(double scaleStep);
    public native double getScaleStep();

    /** Scale votes threshold. */
    public native void setScaleThresh(int scaleThresh);
    public native int getScaleThresh();

    /** Position votes threshold. */
    public native void setPosThresh(int posThresh);
    public native int getPosThresh();
}


@Namespace("cv") public static class CLAHE extends Algorithm {
    static { Loader.load(); }
    /** Empty constructor. */
    public CLAHE() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CLAHE(Pointer p) { super(p); }

    public native void apply(@ByVal Mat src, @ByVal Mat dst);

    public native void setClipLimit(double clipLimit);
    public native double getClipLimit();

    public native void setTilesGridSize(@ByVal Size tileGridSize);
    public native @ByVal Size getTilesGridSize();

    public native void collectGarbage();
}


@Namespace("cv") @NoOffset public static class Subdiv2D extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Subdiv2D(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Subdiv2D(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Subdiv2D position(int position) {
        return (Subdiv2D)super.position(position);
    }

    /** enum cv::Subdiv2D:: */
    public static final int PTLOC_ERROR        = -2,
           PTLOC_OUTSIDE_RECT = -1,
           PTLOC_INSIDE       = 0,
           PTLOC_VERTEX       = 1,
           PTLOC_ON_EDGE      = 2;

    /** enum cv::Subdiv2D:: */
    public static final int NEXT_AROUND_ORG   =  0x00,
           NEXT_AROUND_DST   =  0x22,
           PREV_AROUND_ORG   =  0x11,
           PREV_AROUND_DST   =  0x33,
           NEXT_AROUND_LEFT  =  0x13,
           NEXT_AROUND_RIGHT =  0x31,
           PREV_AROUND_LEFT  =  0x20,
           PREV_AROUND_RIGHT =  0x02;

    public Subdiv2D() { allocate(); }
    private native void allocate();
    public Subdiv2D(@ByVal Rect rect) { allocate(rect); }
    private native void allocate(@ByVal Rect rect);
    public native void initDelaunay(@ByVal Rect rect);

    public native int insert(@ByVal Point2f pt);
    public native int locate(@ByVal Point2f pt, @ByRef IntPointer edge, @ByRef IntPointer vertex);
    public native int locate(@ByVal Point2f pt, @ByRef IntBuffer edge, @ByRef IntBuffer vertex);
    public native int locate(@ByVal Point2f pt, @ByRef int[] edge, @ByRef int[] vertex);

    public native int findNearest(@ByVal Point2f pt, Point2f nearestPt/*=0*/);
    public native int findNearest(@ByVal Point2f pt);
    public native void getEdgeList(@Cast("cv::Vec4f*") @StdVector FloatPointer edgeList);
    public native void getTriangleList(@Cast("cv::Vec6f*") @StdVector FloatPointer triangleList);
    public native void getVoronoiFacetList(@StdVector IntPointer idx, @ByRef Point2fVectorVector facetList,
                                         @StdVector Point2f facetCenters);
    public native void getVoronoiFacetList(@StdVector IntBuffer idx, @ByRef Point2fVectorVector facetList,
                                         @StdVector Point2f facetCenters);
    public native void getVoronoiFacetList(@StdVector int[] idx, @ByRef Point2fVectorVector facetList,
                                         @StdVector Point2f facetCenters);

    public native @ByVal Point2f getVertex(int vertex, IntPointer firstEdge/*=0*/);
    public native @ByVal Point2f getVertex(int vertex);
    public native @ByVal Point2f getVertex(int vertex, IntBuffer firstEdge/*=0*/);
    public native @ByVal Point2f getVertex(int vertex, int[] firstEdge/*=0*/);

    public native int getEdge( int edge, int nextEdgeType );
    public native int nextEdge(int edge);
    public native int rotateEdge(int edge, int rotate);
    public native int symEdge(int edge);
    public native int edgeOrg(int edge, Point2f orgpt/*=0*/);
    public native int edgeOrg(int edge);
    public native int edgeDst(int edge, Point2f dstpt/*=0*/);
    public native int edgeDst(int edge);
}

@Namespace("cv") public static class LineSegmentDetector extends Algorithm {
    static { Loader.load(); }
    /** Empty constructor. */
    public LineSegmentDetector() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public LineSegmentDetector(Pointer p) { super(p); }

/**
 * Detect lines in the input image.
 *
 * @param _image    A grayscale(CV_8UC1) input image.
 *                  If only a roi needs to be selected, use
 *                  lsd_ptr->detect(image(roi), ..., lines);
 *                  lines += Scalar(roi.x, roi.y, roi.x, roi.y);
 * @param _lines    Return: A vector of Vec4i elements specifying the beginning and ending point of a line.
 *                          Where Vec4i is (x1, y1, x2, y2), point 1 is the start, point 2 - end.
 *                          Returned lines are strictly oriented depending on the gradient.
 * @param width     Return: Vector of widths of the regions, where the lines are found. E.g. Width of line.
 * @param prec      Return: Vector of precisions with which the lines are found.
 * @param nfa       Return: Vector containing number of false alarms in the line region, with precision of 10%.
 *                          The bigger the value, logarithmically better the detection.
 *                              * -1 corresponds to 10 mean false alarms
 *                              * 0 corresponds to 1 mean false alarm
 *                              * 1 corresponds to 0.1 mean false alarms
 *                          This vector will be calculated _only_ when the objects type is REFINE_ADV
 */
    public native void detect(@ByVal Mat _image, @ByVal Mat _lines,
                            @ByVal Mat width/*=noArray()*/, @ByVal Mat prec/*=noArray()*/,
                            @ByVal Mat nfa/*=noArray()*/);
    public native void detect(@ByVal Mat _image, @ByVal Mat _lines);

/**
 * Draw lines on the given canvas.
 * @param _image    The image, where lines will be drawn.
 *                  Should have the size of the image, where the lines were found
 * @param lines     The lines that need to be drawn
 */
    public native void drawSegments(@ByVal Mat _image, @ByVal Mat lines);

/**
 * Draw both vectors on the image canvas. Uses blue for lines 1 and red for lines 2.
 * @param size      The size of the image, where lines were found.
 * @param lines1    The first lines that need to be drawn. Color - Blue.
 * @param lines2    The second lines that need to be drawn. Color - Red.
 * @param _image    Optional image, where lines will be drawn.
 *                  Should have the size of the image, where the lines were found
 * @return          The number of mismatching pixels between lines1 and lines2.
 */
    public native int compareSegments(@Const @ByRef Size size, @ByVal Mat lines1, @ByVal Mat lines2, @ByVal Mat _image/*=noArray()*/);
    public native int compareSegments(@Const @ByRef Size size, @ByVal Mat lines1, @ByVal Mat lines2);
}

/** Returns a pointer to a LineSegmentDetector class. */
@Namespace("cv") public static native @Ptr LineSegmentDetector createLineSegmentDetector(
    int _refine/*=LSD_REFINE_STD*/, double _scale/*=0.8*/,
    double _sigma_scale/*=0.6*/, double _quant/*=2.0*/, double _ang_th/*=22.5*/,
    double _log_eps/*=0*/, double _density_th/*=0.7*/, int _n_bins/*=1024*/);
@Namespace("cv") public static native @Ptr LineSegmentDetector createLineSegmentDetector();

/** returns the Gaussian kernel with the specified parameters */
@Namespace("cv") public static native @ByVal Mat getGaussianKernel( int ksize, double sigma, int ktype/*=CV_64F*/ );
@Namespace("cv") public static native @ByVal Mat getGaussianKernel( int ksize, double sigma );

/** initializes kernels of the generalized Sobel operator */
@Namespace("cv") public static native void getDerivKernels( @ByVal Mat kx, @ByVal Mat ky,
                                   int dx, int dy, int ksize,
                                   @Cast("bool") boolean normalize/*=false*/, int ktype/*=CV_32F*/ );
@Namespace("cv") public static native void getDerivKernels( @ByVal Mat kx, @ByVal Mat ky,
                                   int dx, int dy, int ksize );

/** returns the Gabor kernel with the specified parameters */
@Namespace("cv") public static native @ByVal Mat getGaborKernel( @ByVal Size ksize, double sigma, double theta, double lambd,
                                 double gamma, double psi/*=CV_PI*0.5*/, int ktype/*=CV_64F*/ );
@Namespace("cv") public static native @ByVal Mat getGaborKernel( @ByVal Size ksize, double sigma, double theta, double lambd,
                                 double gamma );

/** returns "magic" border value for erosion and dilation. It is automatically transformed to Scalar::all(-DBL_MAX) for dilation. */
@Namespace("cv") public static native @ByVal Scalar morphologyDefaultBorderValue();

/** returns structuring element of the specified shape and size */
@Namespace("cv") public static native @ByVal Mat getStructuringElement(int shape, @ByVal Size ksize, @ByVal Point anchor/*=Point(-1,-1)*/);
@Namespace("cv") public static native @ByVal Mat getStructuringElement(int shape, @ByVal Size ksize);

/** smooths the image using median filter. */
@Namespace("cv") public static native void medianBlur( @ByVal Mat src, @ByVal Mat dst, int ksize );

/** smooths the image using Gaussian filter. */
@Namespace("cv") public static native void GaussianBlur( @ByVal Mat src, @ByVal Mat dst, @ByVal Size ksize,
                                double sigmaX, double sigmaY/*=0*/,
                                int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void GaussianBlur( @ByVal Mat src, @ByVal Mat dst, @ByVal Size ksize,
                                double sigmaX );

/** smooths the image using bilateral filter */
@Namespace("cv") public static native void bilateralFilter( @ByVal Mat src, @ByVal Mat dst, int d,
                                   double sigmaColor, double sigmaSpace,
                                   int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void bilateralFilter( @ByVal Mat src, @ByVal Mat dst, int d,
                                   double sigmaColor, double sigmaSpace );

/** smooths the image using the box filter. Each pixel is processed in O(1) time */
@Namespace("cv") public static native void boxFilter( @ByVal Mat src, @ByVal Mat dst, int ddepth,
                             @ByVal Size ksize, @ByVal Point anchor/*=Point(-1,-1)*/,
                             @Cast("bool") boolean normalize/*=true*/,
                             int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void boxFilter( @ByVal Mat src, @ByVal Mat dst, int ddepth,
                             @ByVal Size ksize );

@Namespace("cv") public static native void sqrBoxFilter( @ByVal Mat _src, @ByVal Mat _dst, int ddepth,
                                @ByVal Size ksize, @ByVal Point anchor/*=Point(-1, -1)*/,
                                @Cast("bool") boolean normalize/*=true*/,
                                int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void sqrBoxFilter( @ByVal Mat _src, @ByVal Mat _dst, int ddepth,
                                @ByVal Size ksize );

/** a synonym for normalized box filter */
@Namespace("cv") public static native void blur( @ByVal Mat src, @ByVal Mat dst,
                        @ByVal Size ksize, @ByVal Point anchor/*=Point(-1,-1)*/,
                        int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void blur( @ByVal Mat src, @ByVal Mat dst,
                        @ByVal Size ksize );

/** applies non-separable 2D linear filter to the image */
@Namespace("cv") public static native void filter2D( @ByVal Mat src, @ByVal Mat dst, int ddepth,
                            @ByVal Mat kernel, @ByVal Point anchor/*=Point(-1,-1)*/,
                            double delta/*=0*/, int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void filter2D( @ByVal Mat src, @ByVal Mat dst, int ddepth,
                            @ByVal Mat kernel );

/** applies separable 2D linear filter to the image */
@Namespace("cv") public static native void sepFilter2D( @ByVal Mat src, @ByVal Mat dst, int ddepth,
                               @ByVal Mat kernelX, @ByVal Mat kernelY,
                               @ByVal Point anchor/*=Point(-1,-1)*/,
                               double delta/*=0*/, int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void sepFilter2D( @ByVal Mat src, @ByVal Mat dst, int ddepth,
                               @ByVal Mat kernelX, @ByVal Mat kernelY );

/** applies generalized Sobel operator to the image */
@Namespace("cv") public static native void Sobel( @ByVal Mat src, @ByVal Mat dst, int ddepth,
                         int dx, int dy, int ksize/*=3*/,
                         double scale/*=1*/, double delta/*=0*/,
                         int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void Sobel( @ByVal Mat src, @ByVal Mat dst, int ddepth,
                         int dx, int dy );

/** applies the vertical or horizontal Scharr operator to the image */
@Namespace("cv") public static native void Scharr( @ByVal Mat src, @ByVal Mat dst, int ddepth,
                          int dx, int dy, double scale/*=1*/, double delta/*=0*/,
                          int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void Scharr( @ByVal Mat src, @ByVal Mat dst, int ddepth,
                          int dx, int dy );

/** applies Laplacian operator to the image */
@Namespace("cv") public static native void Laplacian( @ByVal Mat src, @ByVal Mat dst, int ddepth,
                             int ksize/*=1*/, double scale/*=1*/, double delta/*=0*/,
                             int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void Laplacian( @ByVal Mat src, @ByVal Mat dst, int ddepth );

/** applies Canny edge detector and produces the edge map. */
@Namespace("cv") public static native void Canny( @ByVal Mat image, @ByVal Mat edges,
                         double threshold1, double threshold2,
                         int apertureSize/*=3*/, @Cast("bool") boolean L2gradient/*=false*/ );
@Namespace("cv") public static native void Canny( @ByVal Mat image, @ByVal Mat edges,
                         double threshold1, double threshold2 );

/** computes minimum eigen value of 2x2 derivative covariation matrix at each pixel - the cornerness criteria */
@Namespace("cv") public static native void cornerMinEigenVal( @ByVal Mat src, @ByVal Mat dst,
                                     int blockSize, int ksize/*=3*/,
                                     int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void cornerMinEigenVal( @ByVal Mat src, @ByVal Mat dst,
                                     int blockSize );

/** computes Harris cornerness criteria at each image pixel */
@Namespace("cv") public static native void cornerHarris( @ByVal Mat src, @ByVal Mat dst, int blockSize,
                                int ksize, double k,
                                int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void cornerHarris( @ByVal Mat src, @ByVal Mat dst, int blockSize,
                                int ksize, double k );

/** computes both eigenvalues and the eigenvectors of 2x2 derivative covariation matrix  at each pixel. The output is stored as 6-channel matrix. */
@Namespace("cv") public static native void cornerEigenValsAndVecs( @ByVal Mat src, @ByVal Mat dst,
                                          int blockSize, int ksize,
                                          int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void cornerEigenValsAndVecs( @ByVal Mat src, @ByVal Mat dst,
                                          int blockSize, int ksize );

/** computes another complex cornerness criteria at each pixel */
@Namespace("cv") public static native void preCornerDetect( @ByVal Mat src, @ByVal Mat dst, int ksize,
                                   int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void preCornerDetect( @ByVal Mat src, @ByVal Mat dst, int ksize );

/** adjusts the corner locations with sub-pixel accuracy to maximize the certain cornerness criteria */
@Namespace("cv") public static native void cornerSubPix( @ByVal Mat image, @ByVal Mat corners,
                                @ByVal Size winSize, @ByVal Size zeroZone,
                                @ByVal TermCriteria criteria );

/** finds the strong enough corners where the cornerMinEigenVal() or cornerHarris() report the local maxima */
@Namespace("cv") public static native void goodFeaturesToTrack( @ByVal Mat image, @ByVal Mat corners,
                                     int maxCorners, double qualityLevel, double minDistance,
                                     @ByVal Mat mask/*=noArray()*/, int blockSize/*=3*/,
                                     @Cast("bool") boolean useHarrisDetector/*=false*/, double k/*=0.04*/ );
@Namespace("cv") public static native void goodFeaturesToTrack( @ByVal Mat image, @ByVal Mat corners,
                                     int maxCorners, double qualityLevel, double minDistance );

/** finds lines in the black-n-white image using the standard or pyramid Hough transform */
@Namespace("cv") public static native void HoughLines( @ByVal Mat image, @ByVal Mat lines,
                              double rho, double theta, int threshold,
                              double srn/*=0*/, double stn/*=0*/,
                              double min_theta/*=0*/, double max_theta/*=CV_PI*/ );
@Namespace("cv") public static native void HoughLines( @ByVal Mat image, @ByVal Mat lines,
                              double rho, double theta, int threshold );

/** finds line segments in the black-n-white image using probabilistic Hough transform */
@Namespace("cv") public static native void HoughLinesP( @ByVal Mat image, @ByVal Mat lines,
                               double rho, double theta, int threshold,
                               double minLineLength/*=0*/, double maxLineGap/*=0*/ );
@Namespace("cv") public static native void HoughLinesP( @ByVal Mat image, @ByVal Mat lines,
                               double rho, double theta, int threshold );

/** finds circles in the grayscale image using 2+1 gradient Hough transform */
@Namespace("cv") public static native void HoughCircles( @ByVal Mat image, @ByVal Mat circles,
                               int method, double dp, double minDist,
                               double param1/*=100*/, double param2/*=100*/,
                               int minRadius/*=0*/, int maxRadius/*=0*/ );
@Namespace("cv") public static native void HoughCircles( @ByVal Mat image, @ByVal Mat circles,
                               int method, double dp, double minDist );

/** erodes the image (applies the local minimum operator) */
@Namespace("cv") public static native void erode( @ByVal Mat src, @ByVal Mat dst, @ByVal Mat kernel,
                         @ByVal Point anchor/*=Point(-1,-1)*/, int iterations/*=1*/,
                         int borderType/*=BORDER_CONSTANT*/,
                         @Const @ByRef Scalar borderValue/*=morphologyDefaultBorderValue()*/ );
@Namespace("cv") public static native void erode( @ByVal Mat src, @ByVal Mat dst, @ByVal Mat kernel );

/** dilates the image (applies the local maximum operator) */
@Namespace("cv") public static native void dilate( @ByVal Mat src, @ByVal Mat dst, @ByVal Mat kernel,
                          @ByVal Point anchor/*=Point(-1,-1)*/, int iterations/*=1*/,
                          int borderType/*=BORDER_CONSTANT*/,
                          @Const @ByRef Scalar borderValue/*=morphologyDefaultBorderValue()*/ );
@Namespace("cv") public static native void dilate( @ByVal Mat src, @ByVal Mat dst, @ByVal Mat kernel );

/** applies an advanced morphological operation to the image */
@Namespace("cv") public static native void morphologyEx( @ByVal Mat src, @ByVal Mat dst,
                                int op, @ByVal Mat kernel,
                                @ByVal Point anchor/*=Point(-1,-1)*/, int iterations/*=1*/,
                                int borderType/*=BORDER_CONSTANT*/,
                                @Const @ByRef Scalar borderValue/*=morphologyDefaultBorderValue()*/ );
@Namespace("cv") public static native void morphologyEx( @ByVal Mat src, @ByVal Mat dst,
                                int op, @ByVal Mat kernel );

/** resizes the image */
@Namespace("cv") public static native void resize( @ByVal Mat src, @ByVal Mat dst,
                          @ByVal Size dsize, double fx/*=0*/, double fy/*=0*/,
                          int interpolation/*=INTER_LINEAR*/ );
@Namespace("cv") public static native void resize( @ByVal Mat src, @ByVal Mat dst,
                          @ByVal Size dsize );

/** warps the image using affine transformation */
@Namespace("cv") public static native void warpAffine( @ByVal Mat src, @ByVal Mat dst,
                              @ByVal Mat M, @ByVal Size dsize,
                              int flags/*=INTER_LINEAR*/,
                              int borderMode/*=BORDER_CONSTANT*/,
                              @Const @ByRef Scalar borderValue/*=Scalar()*/);
@Namespace("cv") public static native void warpAffine( @ByVal Mat src, @ByVal Mat dst,
                              @ByVal Mat M, @ByVal Size dsize);

/** warps the image using perspective transformation */
@Namespace("cv") public static native void warpPerspective( @ByVal Mat src, @ByVal Mat dst,
                                   @ByVal Mat M, @ByVal Size dsize,
                                   int flags/*=INTER_LINEAR*/,
                                   int borderMode/*=BORDER_CONSTANT*/,
                                   @Const @ByRef Scalar borderValue/*=Scalar()*/);
@Namespace("cv") public static native void warpPerspective( @ByVal Mat src, @ByVal Mat dst,
                                   @ByVal Mat M, @ByVal Size dsize);

/** warps the image using the precomputed maps. The maps are stored in either floating-point or integer fixed-point format */
@Namespace("cv") public static native void remap( @ByVal Mat src, @ByVal Mat dst,
                         @ByVal Mat map1, @ByVal Mat map2,
                         int interpolation, int borderMode/*=BORDER_CONSTANT*/,
                         @Const @ByRef Scalar borderValue/*=Scalar()*/);
@Namespace("cv") public static native void remap( @ByVal Mat src, @ByVal Mat dst,
                         @ByVal Mat map1, @ByVal Mat map2,
                         int interpolation);

/** converts maps for remap from floating-point to fixed-point format or backwards */
@Namespace("cv") public static native void convertMaps( @ByVal Mat map1, @ByVal Mat map2,
                               @ByVal Mat dstmap1, @ByVal Mat dstmap2,
                               int dstmap1type, @Cast("bool") boolean nninterpolation/*=false*/ );
@Namespace("cv") public static native void convertMaps( @ByVal Mat map1, @ByVal Mat map2,
                               @ByVal Mat dstmap1, @ByVal Mat dstmap2,
                               int dstmap1type );

/** returns 2x3 affine transformation matrix for the planar rotation. */
@Namespace("cv") public static native @ByVal Mat getRotationMatrix2D( @ByVal Point2f center, double angle, double scale );

/** returns 3x3 perspective transformation for the corresponding 4 point pairs. */
@Namespace("cv") public static native @ByVal Mat getPerspectiveTransform( @Const Point2f src, @Const Point2f dst );

/** returns 2x3 affine transformation for the corresponding 3 point pairs. */
@Namespace("cv") public static native @ByVal Mat getAffineTransform( @Const Point2f src, @Const Point2f dst );

/** computes 2x3 affine transformation matrix that is inverse to the specified 2x3 affine transformation. */
@Namespace("cv") public static native void invertAffineTransform( @ByVal Mat M, @ByVal Mat iM );

@Namespace("cv") public static native @ByVal Mat getPerspectiveTransform( @ByVal Mat src, @ByVal Mat dst );

@Namespace("cv") public static native @ByVal Mat getAffineTransform( @ByVal Mat src, @ByVal Mat dst );

/** extracts rectangle from the image at sub-pixel location */
@Namespace("cv") public static native void getRectSubPix( @ByVal Mat image, @ByVal Size patchSize,
                                 @ByVal Point2f center, @ByVal Mat patch, int patchType/*=-1*/ );
@Namespace("cv") public static native void getRectSubPix( @ByVal Mat image, @ByVal Size patchSize,
                                 @ByVal Point2f center, @ByVal Mat patch );

/** computes the log polar transform */
@Namespace("cv") public static native void logPolar( @ByVal Mat src, @ByVal Mat dst,
                            @ByVal Point2f center, double M, int flags );

/** computes the linear polar transform */
@Namespace("cv") public static native void linearPolar( @ByVal Mat src, @ByVal Mat dst,
                               @ByVal Point2f center, double maxRadius, int flags );

/** computes the integral image */
@Namespace("cv") public static native void integral( @ByVal Mat src, @ByVal Mat sum, int sdepth/*=-1*/ );
@Namespace("cv") public static native void integral( @ByVal Mat src, @ByVal Mat sum );

/** computes the integral image and integral for the squared image */
@Namespace("cv") public static native @Name("integral") void integral2( @ByVal Mat src, @ByVal Mat sum,
                                        @ByVal Mat sqsum, int sdepth/*=-1*/, int sqdepth/*=-1*/ );
@Namespace("cv") public static native @Name("integral") void integral2( @ByVal Mat src, @ByVal Mat sum,
                                        @ByVal Mat sqsum );

/** computes the integral image, integral for the squared image and the tilted integral image */
@Namespace("cv") public static native @Name("integral") void integral3( @ByVal Mat src, @ByVal Mat sum,
                                        @ByVal Mat sqsum, @ByVal Mat tilted,
                                        int sdepth/*=-1*/, int sqdepth/*=-1*/ );
@Namespace("cv") public static native @Name("integral") void integral3( @ByVal Mat src, @ByVal Mat sum,
                                        @ByVal Mat sqsum, @ByVal Mat tilted );

/** adds image to the accumulator (dst += src). Unlike cv::add, dst and src can have different types. */
@Namespace("cv") public static native void accumulate( @ByVal Mat src, @ByVal Mat dst,
                              @ByVal Mat mask/*=noArray()*/ );
@Namespace("cv") public static native void accumulate( @ByVal Mat src, @ByVal Mat dst );

/** adds squared src image to the accumulator (dst += src*src). */
@Namespace("cv") public static native void accumulateSquare( @ByVal Mat src, @ByVal Mat dst,
                                    @ByVal Mat mask/*=noArray()*/ );
@Namespace("cv") public static native void accumulateSquare( @ByVal Mat src, @ByVal Mat dst );
/** adds product of the 2 images to the accumulator (dst += src1*src2). */
@Namespace("cv") public static native void accumulateProduct( @ByVal Mat src1, @ByVal Mat src2,
                                     @ByVal Mat dst, @ByVal Mat mask/*=noArray()*/ );
@Namespace("cv") public static native void accumulateProduct( @ByVal Mat src1, @ByVal Mat src2,
                                     @ByVal Mat dst );

/** updates the running average (dst = dst*(1-alpha) + src*alpha) */
@Namespace("cv") public static native void accumulateWeighted( @ByVal Mat src, @ByVal Mat dst,
                                      double alpha, @ByVal Mat mask/*=noArray()*/ );
@Namespace("cv") public static native void accumulateWeighted( @ByVal Mat src, @ByVal Mat dst,
                                      double alpha );

@Namespace("cv") public static native @ByVal Point2d phaseCorrelate(@ByVal Mat src1, @ByVal Mat src2,
                                    @ByVal Mat window/*=noArray()*/, DoublePointer response/*=0*/);
@Namespace("cv") public static native @ByVal Point2d phaseCorrelate(@ByVal Mat src1, @ByVal Mat src2);
@Namespace("cv") public static native @ByVal Point2d phaseCorrelate(@ByVal Mat src1, @ByVal Mat src2,
                                    @ByVal Mat window/*=noArray()*/, DoubleBuffer response/*=0*/);
@Namespace("cv") public static native @ByVal Point2d phaseCorrelate(@ByVal Mat src1, @ByVal Mat src2,
                                    @ByVal Mat window/*=noArray()*/, double[] response/*=0*/);

@Namespace("cv") public static native void createHanningWindow(@ByVal Mat dst, @ByVal Size winSize, int type);

/** applies fixed threshold to the image */
@Namespace("cv") public static native double threshold( @ByVal Mat src, @ByVal Mat dst,
                               double thresh, double maxval, int type );


/** applies variable (adaptive) threshold to the image */
@Namespace("cv") public static native void adaptiveThreshold( @ByVal Mat src, @ByVal Mat dst,
                                     double maxValue, int adaptiveMethod,
                                     int thresholdType, int blockSize, double C );

/** smooths and downsamples the image */
@Namespace("cv") public static native void pyrDown( @ByVal Mat src, @ByVal Mat dst,
                           @Const @ByRef Size dstsize/*=Size()*/, int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void pyrDown( @ByVal Mat src, @ByVal Mat dst );

/** upsamples and smoothes the image */
@Namespace("cv") public static native void pyrUp( @ByVal Mat src, @ByVal Mat dst,
                         @Const @ByRef Size dstsize/*=Size()*/, int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void pyrUp( @ByVal Mat src, @ByVal Mat dst );

/** builds the gaussian pyramid using pyrDown() as a basic operation */
@Namespace("cv") public static native void buildPyramid( @ByVal Mat src, @ByVal MatVector dst,
                              int maxlevel, int borderType/*=BORDER_DEFAULT*/ );
@Namespace("cv") public static native void buildPyramid( @ByVal Mat src, @ByVal MatVector dst,
                              int maxlevel );

/** corrects lens distortion for the given camera matrix and distortion coefficients */
@Namespace("cv") public static native void undistort( @ByVal Mat src, @ByVal Mat dst,
                             @ByVal Mat cameraMatrix,
                             @ByVal Mat distCoeffs,
                             @ByVal Mat newCameraMatrix/*=noArray()*/ );
@Namespace("cv") public static native void undistort( @ByVal Mat src, @ByVal Mat dst,
                             @ByVal Mat cameraMatrix,
                             @ByVal Mat distCoeffs );

/** initializes maps for cv::remap() to correct lens distortion and optionally rectify the image */
@Namespace("cv") public static native void initUndistortRectifyMap( @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
                           @ByVal Mat R, @ByVal Mat newCameraMatrix,
                           @ByVal Size size, int m1type, @ByVal Mat map1, @ByVal Mat map2 );

/** initializes maps for cv::remap() for wide-angle */
@Namespace("cv") public static native float initWideAngleProjMap( @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
                                         @ByVal Size imageSize, int destImageWidth,
                                         int m1type, @ByVal Mat map1, @ByVal Mat map2,
                                         int projType/*=PROJ_SPHERICAL_EQRECT*/, double alpha/*=0*/);
@Namespace("cv") public static native float initWideAngleProjMap( @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
                                         @ByVal Size imageSize, int destImageWidth,
                                         int m1type, @ByVal Mat map1, @ByVal Mat map2);

/** returns the default new camera matrix (by default it is the same as cameraMatrix unless centerPricipalPoint=true) */
@Namespace("cv") public static native @ByVal Mat getDefaultNewCameraMatrix( @ByVal Mat cameraMatrix, @ByVal Size imgsize/*=Size()*/,
                                            @Cast("bool") boolean centerPrincipalPoint/*=false*/ );
@Namespace("cv") public static native @ByVal Mat getDefaultNewCameraMatrix( @ByVal Mat cameraMatrix );

/** returns points' coordinates after lens distortion correction */
@Namespace("cv") public static native void undistortPoints( @ByVal Mat src, @ByVal Mat dst,
                                   @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs,
                                   @ByVal Mat R/*=noArray()*/, @ByVal Mat P/*=noArray()*/);
@Namespace("cv") public static native void undistortPoints( @ByVal Mat src, @ByVal Mat dst,
                                   @ByVal Mat cameraMatrix, @ByVal Mat distCoeffs);

/** computes the joint dense histogram for a set of images. */
@Namespace("cv") public static native void calcHist( @Const Mat images, int nimages,
                          @Const IntPointer channels, @ByVal Mat mask,
                          @ByVal Mat hist, int dims, @Const IntPointer histSize,
                          @Cast("const float**") PointerPointer ranges, @Cast("bool") boolean uniform/*=true*/, @Cast("bool") boolean accumulate/*=false*/ );
@Namespace("cv") public static native void calcHist( @Const Mat images, int nimages,
                          @Const IntPointer channels, @ByVal Mat mask,
                          @ByVal Mat hist, int dims, @Const IntPointer histSize,
                          @Const @ByPtrPtr FloatPointer ranges );
@Namespace("cv") public static native void calcHist( @Const Mat images, int nimages,
                          @Const IntPointer channels, @ByVal Mat mask,
                          @ByVal Mat hist, int dims, @Const IntPointer histSize,
                          @Const @ByPtrPtr FloatPointer ranges, @Cast("bool") boolean uniform/*=true*/, @Cast("bool") boolean accumulate/*=false*/ );
@Namespace("cv") public static native void calcHist( @Const Mat images, int nimages,
                          @Const IntBuffer channels, @ByVal Mat mask,
                          @ByVal Mat hist, int dims, @Const IntBuffer histSize,
                          @Const @ByPtrPtr FloatBuffer ranges, @Cast("bool") boolean uniform/*=true*/, @Cast("bool") boolean accumulate/*=false*/ );
@Namespace("cv") public static native void calcHist( @Const Mat images, int nimages,
                          @Const IntBuffer channels, @ByVal Mat mask,
                          @ByVal Mat hist, int dims, @Const IntBuffer histSize,
                          @Const @ByPtrPtr FloatBuffer ranges );
@Namespace("cv") public static native void calcHist( @Const Mat images, int nimages,
                          @Const int[] channels, @ByVal Mat mask,
                          @ByVal Mat hist, int dims, @Const int[] histSize,
                          @Const @ByPtrPtr float[] ranges, @Cast("bool") boolean uniform/*=true*/, @Cast("bool") boolean accumulate/*=false*/ );
@Namespace("cv") public static native void calcHist( @Const Mat images, int nimages,
                          @Const int[] channels, @ByVal Mat mask,
                          @ByVal Mat hist, int dims, @Const int[] histSize,
                          @Const @ByPtrPtr float[] ranges );

/** computes the joint sparse histogram for a set of images. */
@Namespace("cv") public static native void calcHist( @Const Mat images, int nimages,
                          @Const IntPointer channels, @ByVal Mat mask,
                          @ByRef SparseMat hist, int dims,
                          @Const IntPointer histSize, @Cast("const float**") PointerPointer ranges,
                          @Cast("bool") boolean uniform/*=true*/, @Cast("bool") boolean accumulate/*=false*/ );
@Namespace("cv") public static native void calcHist( @Const Mat images, int nimages,
                          @Const IntPointer channels, @ByVal Mat mask,
                          @ByRef SparseMat hist, int dims,
                          @Const IntPointer histSize, @Const @ByPtrPtr FloatPointer ranges );
@Namespace("cv") public static native void calcHist( @Const Mat images, int nimages,
                          @Const IntPointer channels, @ByVal Mat mask,
                          @ByRef SparseMat hist, int dims,
                          @Const IntPointer histSize, @Const @ByPtrPtr FloatPointer ranges,
                          @Cast("bool") boolean uniform/*=true*/, @Cast("bool") boolean accumulate/*=false*/ );
@Namespace("cv") public static native void calcHist( @Const Mat images, int nimages,
                          @Const IntBuffer channels, @ByVal Mat mask,
                          @ByRef SparseMat hist, int dims,
                          @Const IntBuffer histSize, @Const @ByPtrPtr FloatBuffer ranges,
                          @Cast("bool") boolean uniform/*=true*/, @Cast("bool") boolean accumulate/*=false*/ );
@Namespace("cv") public static native void calcHist( @Const Mat images, int nimages,
                          @Const IntBuffer channels, @ByVal Mat mask,
                          @ByRef SparseMat hist, int dims,
                          @Const IntBuffer histSize, @Const @ByPtrPtr FloatBuffer ranges );
@Namespace("cv") public static native void calcHist( @Const Mat images, int nimages,
                          @Const int[] channels, @ByVal Mat mask,
                          @ByRef SparseMat hist, int dims,
                          @Const int[] histSize, @Const @ByPtrPtr float[] ranges,
                          @Cast("bool") boolean uniform/*=true*/, @Cast("bool") boolean accumulate/*=false*/ );
@Namespace("cv") public static native void calcHist( @Const Mat images, int nimages,
                          @Const int[] channels, @ByVal Mat mask,
                          @ByRef SparseMat hist, int dims,
                          @Const int[] histSize, @Const @ByPtrPtr float[] ranges );

@Namespace("cv") public static native void calcHist( @ByVal MatVector images,
                            @StdVector IntPointer channels,
                            @ByVal Mat mask, @ByVal Mat hist,
                            @StdVector IntPointer histSize,
                            @StdVector FloatPointer ranges,
                            @Cast("bool") boolean accumulate/*=false*/ );
@Namespace("cv") public static native void calcHist( @ByVal MatVector images,
                            @StdVector IntPointer channels,
                            @ByVal Mat mask, @ByVal Mat hist,
                            @StdVector IntPointer histSize,
                            @StdVector FloatPointer ranges );
@Namespace("cv") public static native void calcHist( @ByVal MatVector images,
                            @StdVector IntBuffer channels,
                            @ByVal Mat mask, @ByVal Mat hist,
                            @StdVector IntBuffer histSize,
                            @StdVector FloatBuffer ranges,
                            @Cast("bool") boolean accumulate/*=false*/ );
@Namespace("cv") public static native void calcHist( @ByVal MatVector images,
                            @StdVector IntBuffer channels,
                            @ByVal Mat mask, @ByVal Mat hist,
                            @StdVector IntBuffer histSize,
                            @StdVector FloatBuffer ranges );
@Namespace("cv") public static native void calcHist( @ByVal MatVector images,
                            @StdVector int[] channels,
                            @ByVal Mat mask, @ByVal Mat hist,
                            @StdVector int[] histSize,
                            @StdVector float[] ranges,
                            @Cast("bool") boolean accumulate/*=false*/ );
@Namespace("cv") public static native void calcHist( @ByVal MatVector images,
                            @StdVector int[] channels,
                            @ByVal Mat mask, @ByVal Mat hist,
                            @StdVector int[] histSize,
                            @StdVector float[] ranges );

/** computes back projection for the set of images */
@Namespace("cv") public static native void calcBackProject( @Const Mat images, int nimages,
                                 @Const IntPointer channels, @ByVal Mat hist,
                                 @ByVal Mat backProject, @Cast("const float**") PointerPointer ranges,
                                 double scale/*=1*/, @Cast("bool") boolean uniform/*=true*/ );
@Namespace("cv") public static native void calcBackProject( @Const Mat images, int nimages,
                                 @Const IntPointer channels, @ByVal Mat hist,
                                 @ByVal Mat backProject, @Const @ByPtrPtr FloatPointer ranges );
@Namespace("cv") public static native void calcBackProject( @Const Mat images, int nimages,
                                 @Const IntPointer channels, @ByVal Mat hist,
                                 @ByVal Mat backProject, @Const @ByPtrPtr FloatPointer ranges,
                                 double scale/*=1*/, @Cast("bool") boolean uniform/*=true*/ );
@Namespace("cv") public static native void calcBackProject( @Const Mat images, int nimages,
                                 @Const IntBuffer channels, @ByVal Mat hist,
                                 @ByVal Mat backProject, @Const @ByPtrPtr FloatBuffer ranges,
                                 double scale/*=1*/, @Cast("bool") boolean uniform/*=true*/ );
@Namespace("cv") public static native void calcBackProject( @Const Mat images, int nimages,
                                 @Const IntBuffer channels, @ByVal Mat hist,
                                 @ByVal Mat backProject, @Const @ByPtrPtr FloatBuffer ranges );
@Namespace("cv") public static native void calcBackProject( @Const Mat images, int nimages,
                                 @Const int[] channels, @ByVal Mat hist,
                                 @ByVal Mat backProject, @Const @ByPtrPtr float[] ranges,
                                 double scale/*=1*/, @Cast("bool") boolean uniform/*=true*/ );
@Namespace("cv") public static native void calcBackProject( @Const Mat images, int nimages,
                                 @Const int[] channels, @ByVal Mat hist,
                                 @ByVal Mat backProject, @Const @ByPtrPtr float[] ranges );

/** computes back projection for the set of images */
@Namespace("cv") public static native void calcBackProject( @Const Mat images, int nimages,
                                 @Const IntPointer channels, @Const @ByRef SparseMat hist,
                                 @ByVal Mat backProject, @Cast("const float**") PointerPointer ranges,
                                 double scale/*=1*/, @Cast("bool") boolean uniform/*=true*/ );
@Namespace("cv") public static native void calcBackProject( @Const Mat images, int nimages,
                                 @Const IntPointer channels, @Const @ByRef SparseMat hist,
                                 @ByVal Mat backProject, @Const @ByPtrPtr FloatPointer ranges );
@Namespace("cv") public static native void calcBackProject( @Const Mat images, int nimages,
                                 @Const IntPointer channels, @Const @ByRef SparseMat hist,
                                 @ByVal Mat backProject, @Const @ByPtrPtr FloatPointer ranges,
                                 double scale/*=1*/, @Cast("bool") boolean uniform/*=true*/ );
@Namespace("cv") public static native void calcBackProject( @Const Mat images, int nimages,
                                 @Const IntBuffer channels, @Const @ByRef SparseMat hist,
                                 @ByVal Mat backProject, @Const @ByPtrPtr FloatBuffer ranges,
                                 double scale/*=1*/, @Cast("bool") boolean uniform/*=true*/ );
@Namespace("cv") public static native void calcBackProject( @Const Mat images, int nimages,
                                 @Const IntBuffer channels, @Const @ByRef SparseMat hist,
                                 @ByVal Mat backProject, @Const @ByPtrPtr FloatBuffer ranges );
@Namespace("cv") public static native void calcBackProject( @Const Mat images, int nimages,
                                 @Const int[] channels, @Const @ByRef SparseMat hist,
                                 @ByVal Mat backProject, @Const @ByPtrPtr float[] ranges,
                                 double scale/*=1*/, @Cast("bool") boolean uniform/*=true*/ );
@Namespace("cv") public static native void calcBackProject( @Const Mat images, int nimages,
                                 @Const int[] channels, @Const @ByRef SparseMat hist,
                                 @ByVal Mat backProject, @Const @ByPtrPtr float[] ranges );

@Namespace("cv") public static native void calcBackProject( @ByVal MatVector images, @StdVector IntPointer channels,
                                   @ByVal Mat hist, @ByVal Mat dst,
                                   @StdVector FloatPointer ranges,
                                   double scale );
@Namespace("cv") public static native void calcBackProject( @ByVal MatVector images, @StdVector IntBuffer channels,
                                   @ByVal Mat hist, @ByVal Mat dst,
                                   @StdVector FloatBuffer ranges,
                                   double scale );
@Namespace("cv") public static native void calcBackProject( @ByVal MatVector images, @StdVector int[] channels,
                                   @ByVal Mat hist, @ByVal Mat dst,
                                   @StdVector float[] ranges,
                                   double scale );

/** compares two histograms stored in dense arrays */
@Namespace("cv") public static native double compareHist( @ByVal Mat H1, @ByVal Mat H2, int method );

/** compares two histograms stored in sparse arrays */
@Namespace("cv") public static native double compareHist( @Const @ByRef SparseMat H1, @Const @ByRef SparseMat H2, int method );

/** normalizes the grayscale image brightness and contrast by normalizing its histogram */
@Namespace("cv") public static native void equalizeHist( @ByVal Mat src, @ByVal Mat dst );

@Namespace("cv") public static native float EMD( @ByVal Mat signature1, @ByVal Mat signature2,
                      int distType, @ByVal Mat cost/*=noArray()*/,
                      FloatPointer lowerBound/*=0*/, @ByVal Mat flow/*=noArray()*/ );
@Namespace("cv") public static native float EMD( @ByVal Mat signature1, @ByVal Mat signature2,
                      int distType );
@Namespace("cv") public static native float EMD( @ByVal Mat signature1, @ByVal Mat signature2,
                      int distType, @ByVal Mat cost/*=noArray()*/,
                      FloatBuffer lowerBound/*=0*/, @ByVal Mat flow/*=noArray()*/ );
@Namespace("cv") public static native float EMD( @ByVal Mat signature1, @ByVal Mat signature2,
                      int distType, @ByVal Mat cost/*=noArray()*/,
                      float[] lowerBound/*=0*/, @ByVal Mat flow/*=noArray()*/ );

/** segments the image using watershed algorithm */
@Namespace("cv") public static native void watershed( @ByVal Mat image, @ByVal Mat markers );

/** filters image using meanshift algorithm */
@Namespace("cv") public static native void pyrMeanShiftFiltering( @ByVal Mat src, @ByVal Mat dst,
                                         double sp, double sr, int maxLevel/*=1*/,
                                         @ByVal TermCriteria termcrit/*=TermCriteria(TermCriteria::MAX_ITER+TermCriteria::EPS,5,1)*/ );
@Namespace("cv") public static native void pyrMeanShiftFiltering( @ByVal Mat src, @ByVal Mat dst,
                                         double sp, double sr );

/** segments the image using GrabCut algorithm */
@Namespace("cv") public static native void grabCut( @ByVal Mat img, @ByVal Mat mask, @ByVal Rect rect,
                           @ByVal Mat bgdModel, @ByVal Mat fgdModel,
                           int iterCount, int mode/*=GC_EVAL*/ );
@Namespace("cv") public static native void grabCut( @ByVal Mat img, @ByVal Mat mask, @ByVal Rect rect,
                           @ByVal Mat bgdModel, @ByVal Mat fgdModel,
                           int iterCount );


/** builds the discrete Voronoi diagram */
@Namespace("cv") public static native @Name("distanceTransform") void distanceTransformWithLabels( @ByVal Mat src, @ByVal Mat dst,
                                     @ByVal Mat labels, int distanceType, int maskSize,
                                     int labelType/*=DIST_LABEL_CCOMP*/ );
@Namespace("cv") public static native @Name("distanceTransform") void distanceTransformWithLabels( @ByVal Mat src, @ByVal Mat dst,
                                     @ByVal Mat labels, int distanceType, int maskSize );

/** computes the distance transform map */
@Namespace("cv") public static native void distanceTransform( @ByVal Mat src, @ByVal Mat dst,
                                     int distanceType, int maskSize, int dstType/*=CV_32F*/);
@Namespace("cv") public static native void distanceTransform( @ByVal Mat src, @ByVal Mat dst,
                                     int distanceType, int maskSize);


/** fills the semi-uniform image region starting from the specified seed point */
@Namespace("cv") public static native int floodFill( @ByVal Mat image,
                          @ByVal Point seedPoint, @ByVal Scalar newVal, Rect rect/*=0*/,
                          @ByVal Scalar loDiff/*=Scalar()*/, @ByVal Scalar upDiff/*=Scalar()*/,
                          int flags/*=4*/ );
@Namespace("cv") public static native int floodFill( @ByVal Mat image,
                          @ByVal Point seedPoint, @ByVal Scalar newVal );

/** fills the semi-uniform image region and/or the mask starting from the specified seed point */
@Namespace("cv") public static native int floodFill( @ByVal Mat image, @ByVal Mat mask,
                            @ByVal Point seedPoint, @ByVal Scalar newVal, Rect rect/*=0*/,
                            @ByVal Scalar loDiff/*=Scalar()*/, @ByVal Scalar upDiff/*=Scalar()*/,
                            int flags/*=4*/ );
@Namespace("cv") public static native int floodFill( @ByVal Mat image, @ByVal Mat mask,
                            @ByVal Point seedPoint, @ByVal Scalar newVal );

/** converts image from one color space to another */
@Namespace("cv") public static native void cvtColor( @ByVal Mat src, @ByVal Mat dst, int code, int dstCn/*=0*/ );
@Namespace("cv") public static native void cvtColor( @ByVal Mat src, @ByVal Mat dst, int code );

// main function for all demosaicing procceses
@Namespace("cv") public static native void demosaicing(@ByVal Mat _src, @ByVal Mat _dst, int code, int dcn/*=0*/);
@Namespace("cv") public static native void demosaicing(@ByVal Mat _src, @ByVal Mat _dst, int code);

/** computes moments of the rasterized shape or a vector of points */
@Namespace("cv") public static native @ByVal Moments moments( @ByVal Mat array, @Cast("bool") boolean binaryImage/*=false*/ );
@Namespace("cv") public static native @ByVal Moments moments( @ByVal Mat array );

/** computes 7 Hu invariants from the moments */
@Namespace("cv") public static native void HuMoments( @Const @ByRef Moments moments, DoublePointer hu );
@Namespace("cv") public static native void HuMoments( @Const @ByRef Moments moments, DoubleBuffer hu );
@Namespace("cv") public static native void HuMoments( @Const @ByRef Moments moments, double[] hu );

@Namespace("cv") public static native void HuMoments( @Const @ByRef Moments m, @ByVal Mat hu );

/** computes the proximity map for the raster template and the image where the template is searched for */
@Namespace("cv") public static native void matchTemplate( @ByVal Mat image, @ByVal Mat templ,
                                 @ByVal Mat result, int method );


// computes the connected components labeled image of boolean image ``image``
// with 4 or 8 way connectivity - returns N, the total
// number of labels [0, N-1] where 0 represents the background label.
// ltype specifies the output label image type, an important
// consideration based on the total number of labels or
// alternatively the total number of pixels in the source image.
@Namespace("cv") public static native int connectedComponents(@ByVal Mat image, @ByVal Mat labels,
                                     int connectivity/*=8*/, int ltype/*=CV_32S*/);
@Namespace("cv") public static native int connectedComponents(@ByVal Mat image, @ByVal Mat labels);

@Namespace("cv") public static native int connectedComponentsWithStats(@ByVal Mat image, @ByVal Mat labels,
                                              @ByVal Mat stats, @ByVal Mat centroids,
                                              int connectivity/*=8*/, int ltype/*=CV_32S*/);
@Namespace("cv") public static native int connectedComponentsWithStats(@ByVal Mat image, @ByVal Mat labels,
                                              @ByVal Mat stats, @ByVal Mat centroids);


/** retrieves contours and the hierarchical information from black-n-white image. */
@Namespace("cv") public static native void findContours( @ByVal Mat image, @ByVal MatVector contours,
                              @ByVal Mat hierarchy, int mode,
                              int method, @ByVal Point offset/*=Point()*/);
@Namespace("cv") public static native void findContours( @ByVal Mat image, @ByVal MatVector contours,
                              @ByVal Mat hierarchy, int mode,
                              int method);

/** retrieves contours from black-n-white image. */
@Namespace("cv") public static native void findContours( @ByVal Mat image, @ByVal MatVector contours,
                              int mode, int method, @ByVal Point offset/*=Point()*/);
@Namespace("cv") public static native void findContours( @ByVal Mat image, @ByVal MatVector contours,
                              int mode, int method);

/** approximates contour or a curve using Douglas-Peucker algorithm */
@Namespace("cv") public static native void approxPolyDP( @ByVal Mat curve,
                                @ByVal Mat approxCurve,
                                double epsilon, @Cast("bool") boolean closed );

/** computes the contour perimeter (closed=true) or a curve length */
@Namespace("cv") public static native double arcLength( @ByVal Mat curve, @Cast("bool") boolean closed );

/** computes the bounding rectangle for a contour */
@Namespace("cv") public static native @ByVal Rect boundingRect( @ByVal Mat points );

/** computes the contour area */
@Namespace("cv") public static native double contourArea( @ByVal Mat contour, @Cast("bool") boolean oriented/*=false*/ );
@Namespace("cv") public static native double contourArea( @ByVal Mat contour );

/** computes the minimal rotated rectangle for a set of points */
@Namespace("cv") public static native @ByVal RotatedRect minAreaRect( @ByVal Mat points );

/** computes boxpoints */
@Namespace("cv") public static native void boxPoints(@ByVal RotatedRect box, @ByVal Mat points);

/** computes the minimal enclosing circle for a set of points */
@Namespace("cv") public static native void minEnclosingCircle( @ByVal Mat points,
                                      @ByRef Point2f center, @ByRef FloatPointer radius );
@Namespace("cv") public static native void minEnclosingCircle( @ByVal Mat points,
                                      @ByRef Point2f center, @ByRef FloatBuffer radius );
@Namespace("cv") public static native void minEnclosingCircle( @ByVal Mat points,
                                      @ByRef Point2f center, @ByRef float[] radius );

/** computes the minimal enclosing triangle for a set of points and returns its area */
@Namespace("cv") public static native double minEnclosingTriangle( @ByVal Mat points, @ByVal Mat triangle );

/** matches two contours using one of the available algorithms */
@Namespace("cv") public static native double matchShapes( @ByVal Mat contour1, @ByVal Mat contour2,
                                 int method, double parameter );

/** computes convex hull for a set of 2D points. */
@Namespace("cv") public static native void convexHull( @ByVal Mat points, @ByVal Mat hull,
                              @Cast("bool") boolean clockwise/*=false*/, @Cast("bool") boolean returnPoints/*=true*/ );
@Namespace("cv") public static native void convexHull( @ByVal Mat points, @ByVal Mat hull );

/** computes the contour convexity defects */
@Namespace("cv") public static native void convexityDefects( @ByVal Mat contour, @ByVal Mat convexhull, @ByVal Mat convexityDefects );

/** returns true if the contour is convex. Does not support contours with self-intersection */
@Namespace("cv") public static native @Cast("bool") boolean isContourConvex( @ByVal Mat contour );

/** finds intersection of two convex polygons */
@Namespace("cv") public static native float intersectConvexConvex( @ByVal Mat _p1, @ByVal Mat _p2,
                                          @ByVal Mat _p12, @Cast("bool") boolean handleNested/*=true*/ );
@Namespace("cv") public static native float intersectConvexConvex( @ByVal Mat _p1, @ByVal Mat _p2,
                                          @ByVal Mat _p12 );

/** fits ellipse to the set of 2D points */
@Namespace("cv") public static native @ByVal RotatedRect fitEllipse( @ByVal Mat points );

/** fits line to the set of 2D points using M-estimator algorithm */
@Namespace("cv") public static native void fitLine( @ByVal Mat points, @ByVal Mat line, int distType,
                           double param, double reps, double aeps );

/** checks if the point is inside the contour. Optionally computes the signed distance from the point to the contour boundary */
@Namespace("cv") public static native double pointPolygonTest( @ByVal Mat contour, @ByVal Point2f pt, @Cast("bool") boolean measureDist );

/** computes whether two rotated rectangles intersect and returns the vertices of the intersecting region */
@Namespace("cv") public static native int rotatedRectangleIntersection( @Const @ByRef RotatedRect rect1, @Const @ByRef RotatedRect rect2, @ByVal Mat intersectingRegion  );

@Namespace("cv") public static native @Ptr CLAHE createCLAHE(double clipLimit/*=40.0*/, @ByVal Size tileGridSize/*=Size(8, 8)*/);
@Namespace("cv") public static native @Ptr CLAHE createCLAHE();

/** Ballard, D.H. (1981). Generalizing the Hough transform to detect arbitrary shapes. Pattern Recognition 13 (2): 111-122.
 *  Detects position only without traslation and rotation */
@Namespace("cv") public static native @Ptr GeneralizedHoughBallard createGeneralizedHoughBallard();

/** Guil, N., González-Linares, J.M. and Zapata, E.L. (1999). Bidimensional shape detection using an invariant approach. Pattern Recognition 32 (6): 1025-1038.
 *  Detects position, traslation and rotation */
@Namespace("cv") public static native @Ptr GeneralizedHoughGuil createGeneralizedHoughGuil();

/** Performs linear blending of two images */
@Namespace("cv") public static native void blendLinear(@ByVal Mat src1, @ByVal Mat src2, @ByVal Mat weights1, @ByVal Mat weights2, @ByVal Mat dst);

/** enum cv:: */
public static final int
    COLORMAP_AUTUMN = 0,
    COLORMAP_BONE = 1,
    COLORMAP_JET = 2,
    COLORMAP_WINTER = 3,
    COLORMAP_RAINBOW = 4,
    COLORMAP_OCEAN = 5,
    COLORMAP_SUMMER = 6,
    COLORMAP_SPRING = 7,
    COLORMAP_COOL = 8,
    COLORMAP_HSV = 9,
    COLORMAP_PINK = 10,
    COLORMAP_HOT = 11;

@Namespace("cv") public static native void applyColorMap(@ByVal Mat src, @ByVal Mat dst, int colormap);


/** draws the line segment (pt1, pt2) in the image */
@Namespace("cv") public static native void line(@ByVal Mat img, @ByVal Point pt1, @ByVal Point pt2, @Const @ByRef Scalar color,
                     int thickness/*=1*/, int lineType/*=LINE_8*/, int shift/*=0*/);
@Namespace("cv") public static native void line(@ByVal Mat img, @ByVal Point pt1, @ByVal Point pt2, @Const @ByRef Scalar color);

/** draws an arrow from pt1 to pt2 in the image */
@Namespace("cv") public static native void arrowedLine(@ByVal Mat img, @ByVal Point pt1, @ByVal Point pt2, @Const @ByRef Scalar color,
                     int thickness/*=1*/, int line_type/*=8*/, int shift/*=0*/, double tipLength/*=0.1*/);
@Namespace("cv") public static native void arrowedLine(@ByVal Mat img, @ByVal Point pt1, @ByVal Point pt2, @Const @ByRef Scalar color);

/** draws the rectangle outline or a solid rectangle with the opposite corners pt1 and pt2 in the image */
@Namespace("cv") public static native void rectangle(@ByVal Mat img, @ByVal Point pt1, @ByVal Point pt2,
                          @Const @ByRef Scalar color, int thickness/*=1*/,
                          int lineType/*=LINE_8*/, int shift/*=0*/);
@Namespace("cv") public static native void rectangle(@ByVal Mat img, @ByVal Point pt1, @ByVal Point pt2,
                          @Const @ByRef Scalar color);

/** draws the rectangle outline or a solid rectangle covering rec in the image */
@Namespace("cv") public static native void rectangle(@ByRef Mat img, @ByVal Rect rec,
                          @Const @ByRef Scalar color, int thickness/*=1*/,
                          int lineType/*=LINE_8*/, int shift/*=0*/);
@Namespace("cv") public static native void rectangle(@ByRef Mat img, @ByVal Rect rec,
                          @Const @ByRef Scalar color);

/** draws the circle outline or a solid circle in the image */
@Namespace("cv") public static native void circle(@ByVal Mat img, @ByVal Point center, int radius,
                       @Const @ByRef Scalar color, int thickness/*=1*/,
                       int lineType/*=LINE_8*/, int shift/*=0*/);
@Namespace("cv") public static native void circle(@ByVal Mat img, @ByVal Point center, int radius,
                       @Const @ByRef Scalar color);

/** draws an elliptic arc, ellipse sector or a rotated ellipse in the image */
@Namespace("cv") public static native void ellipse(@ByVal Mat img, @ByVal Point center, @ByVal Size axes,
                        double angle, double startAngle, double endAngle,
                        @Const @ByRef Scalar color, int thickness/*=1*/,
                        int lineType/*=LINE_8*/, int shift/*=0*/);
@Namespace("cv") public static native void ellipse(@ByVal Mat img, @ByVal Point center, @ByVal Size axes,
                        double angle, double startAngle, double endAngle,
                        @Const @ByRef Scalar color);

/** draws a rotated ellipse in the image */
@Namespace("cv") public static native void ellipse(@ByVal Mat img, @Const @ByRef RotatedRect box, @Const @ByRef Scalar color,
                        int thickness/*=1*/, int lineType/*=LINE_8*/);
@Namespace("cv") public static native void ellipse(@ByVal Mat img, @Const @ByRef RotatedRect box, @Const @ByRef Scalar color);

/** draws a filled convex polygon in the image */
@Namespace("cv") public static native void fillConvexPoly(@ByRef Mat img, @Const Point pts, int npts,
                               @Const @ByRef Scalar color, int lineType/*=LINE_8*/,
                               int shift/*=0*/);
@Namespace("cv") public static native void fillConvexPoly(@ByRef Mat img, @Const Point pts, int npts,
                               @Const @ByRef Scalar color);

@Namespace("cv") public static native void fillConvexPoly(@ByVal Mat img, @ByVal Mat points,
                                 @Const @ByRef Scalar color, int lineType/*=LINE_8*/,
                                 int shift/*=0*/);
@Namespace("cv") public static native void fillConvexPoly(@ByVal Mat img, @ByVal Mat points,
                                 @Const @ByRef Scalar color);

/** fills an area bounded by one or more polygons */
@Namespace("cv") public static native void fillPoly(@ByRef Mat img, @Cast("const cv::Point**") PointerPointer pts,
                         @Const IntPointer npts, int ncontours,
                         @Const @ByRef Scalar color, int lineType/*=LINE_8*/, int shift/*=0*/,
                         @ByVal Point offset/*=Point()*/ );
@Namespace("cv") public static native void fillPoly(@ByRef Mat img, @Const @ByPtrPtr Point pts,
                         @Const IntPointer npts, int ncontours,
                         @Const @ByRef Scalar color );
@Namespace("cv") public static native void fillPoly(@ByRef Mat img, @Const @ByPtrPtr Point pts,
                         @Const IntPointer npts, int ncontours,
                         @Const @ByRef Scalar color, int lineType/*=LINE_8*/, int shift/*=0*/,
                         @ByVal Point offset/*=Point()*/ );
@Namespace("cv") public static native void fillPoly(@ByRef Mat img, @Const @ByPtrPtr Point pts,
                         @Const IntBuffer npts, int ncontours,
                         @Const @ByRef Scalar color, int lineType/*=LINE_8*/, int shift/*=0*/,
                         @ByVal Point offset/*=Point()*/ );
@Namespace("cv") public static native void fillPoly(@ByRef Mat img, @Const @ByPtrPtr Point pts,
                         @Const IntBuffer npts, int ncontours,
                         @Const @ByRef Scalar color );
@Namespace("cv") public static native void fillPoly(@ByRef Mat img, @Const @ByPtrPtr Point pts,
                         @Const int[] npts, int ncontours,
                         @Const @ByRef Scalar color, int lineType/*=LINE_8*/, int shift/*=0*/,
                         @ByVal Point offset/*=Point()*/ );
@Namespace("cv") public static native void fillPoly(@ByRef Mat img, @Const @ByPtrPtr Point pts,
                         @Const int[] npts, int ncontours,
                         @Const @ByRef Scalar color );

@Namespace("cv") public static native void fillPoly(@ByVal Mat img, @ByVal MatVector pts,
                           @Const @ByRef Scalar color, int lineType/*=LINE_8*/, int shift/*=0*/,
                           @ByVal Point offset/*=Point()*/ );
@Namespace("cv") public static native void fillPoly(@ByVal Mat img, @ByVal MatVector pts,
                           @Const @ByRef Scalar color );

/** draws one or more polygonal curves */
@Namespace("cv") public static native void polylines(@ByRef Mat img, @Cast("const cv::Point*const*") PointerPointer pts, @Const IntPointer npts,
                          int ncontours, @Cast("bool") boolean isClosed, @Const @ByRef Scalar color,
                          int thickness/*=1*/, int lineType/*=LINE_8*/, int shift/*=0*/ );
@Namespace("cv") public static native void polylines(@ByRef Mat img, @Const @ByPtrPtr Point pts, @Const IntPointer npts,
                          int ncontours, @Cast("bool") boolean isClosed, @Const @ByRef Scalar color );
@Namespace("cv") public static native void polylines(@ByRef Mat img, @Const @ByPtrPtr Point pts, @Const IntPointer npts,
                          int ncontours, @Cast("bool") boolean isClosed, @Const @ByRef Scalar color,
                          int thickness/*=1*/, int lineType/*=LINE_8*/, int shift/*=0*/ );
@Namespace("cv") public static native void polylines(@ByRef Mat img, @Const @ByPtrPtr Point pts, @Const IntBuffer npts,
                          int ncontours, @Cast("bool") boolean isClosed, @Const @ByRef Scalar color,
                          int thickness/*=1*/, int lineType/*=LINE_8*/, int shift/*=0*/ );
@Namespace("cv") public static native void polylines(@ByRef Mat img, @Const @ByPtrPtr Point pts, @Const IntBuffer npts,
                          int ncontours, @Cast("bool") boolean isClosed, @Const @ByRef Scalar color );
@Namespace("cv") public static native void polylines(@ByRef Mat img, @Const @ByPtrPtr Point pts, @Const int[] npts,
                          int ncontours, @Cast("bool") boolean isClosed, @Const @ByRef Scalar color,
                          int thickness/*=1*/, int lineType/*=LINE_8*/, int shift/*=0*/ );
@Namespace("cv") public static native void polylines(@ByRef Mat img, @Const @ByPtrPtr Point pts, @Const int[] npts,
                          int ncontours, @Cast("bool") boolean isClosed, @Const @ByRef Scalar color );

@Namespace("cv") public static native void polylines(@ByVal Mat img, @ByVal MatVector pts,
                            @Cast("bool") boolean isClosed, @Const @ByRef Scalar color,
                            int thickness/*=1*/, int lineType/*=LINE_8*/, int shift/*=0*/ );
@Namespace("cv") public static native void polylines(@ByVal Mat img, @ByVal MatVector pts,
                            @Cast("bool") boolean isClosed, @Const @ByRef Scalar color );

/** draws contours in the image */
@Namespace("cv") public static native void drawContours( @ByVal Mat image, @ByVal MatVector contours,
                              int contourIdx, @Const @ByRef Scalar color,
                              int thickness/*=1*/, int lineType/*=LINE_8*/,
                              @ByVal Mat hierarchy/*=noArray()*/,
                              int maxLevel/*=INT_MAX*/, @ByVal Point offset/*=Point()*/ );
@Namespace("cv") public static native void drawContours( @ByVal Mat image, @ByVal MatVector contours,
                              int contourIdx, @Const @ByRef Scalar color );

/** clips the line segment by the rectangle Rect(0, 0, imgSize.width, imgSize.height) */
@Namespace("cv") public static native @Cast("bool") boolean clipLine(@ByVal Size imgSize, @ByRef Point pt1, @ByRef Point pt2);

/** clips the line segment by the rectangle imgRect */
@Namespace("cv") public static native @Cast("bool") boolean clipLine(@ByVal Rect imgRect, @ByRef Point pt1, @ByRef Point pt2);

/** converts elliptic arc to a polygonal curve */
@Namespace("cv") public static native void ellipse2Poly( @ByVal Point center, @ByVal Size axes, int angle,
                                int arcStart, int arcEnd, int delta,
                                @StdVector Point pts );

/** renders text string in the image */
@Namespace("cv") public static native void putText( @ByVal Mat img, @Str BytePointer text, @ByVal Point org,
                         int fontFace, double fontScale, @ByVal Scalar color,
                         int thickness/*=1*/, int lineType/*=LINE_8*/,
                         @Cast("bool") boolean bottomLeftOrigin/*=false*/ );
@Namespace("cv") public static native void putText( @ByVal Mat img, @Str BytePointer text, @ByVal Point org,
                         int fontFace, double fontScale, @ByVal Scalar color );
@Namespace("cv") public static native void putText( @ByVal Mat img, @Str String text, @ByVal Point org,
                         int fontFace, double fontScale, @ByVal Scalar color,
                         int thickness/*=1*/, int lineType/*=LINE_8*/,
                         @Cast("bool") boolean bottomLeftOrigin/*=false*/ );
@Namespace("cv") public static native void putText( @ByVal Mat img, @Str String text, @ByVal Point org,
                         int fontFace, double fontScale, @ByVal Scalar color );

/** returns bounding box of the text string */
@Namespace("cv") public static native @ByVal Size getTextSize(@Str BytePointer text, int fontFace,
                            double fontScale, int thickness,
                            IntPointer baseLine);
@Namespace("cv") public static native @ByVal Size getTextSize(@Str String text, int fontFace,
                            double fontScale, int thickness,
                            IntBuffer baseLine);
@Namespace("cv") public static native @ByVal Size getTextSize(@Str BytePointer text, int fontFace,
                            double fontScale, int thickness,
                            int[] baseLine);
@Namespace("cv") public static native @ByVal Size getTextSize(@Str String text, int fontFace,
                            double fontScale, int thickness,
                            IntPointer baseLine);
@Namespace("cv") public static native @ByVal Size getTextSize(@Str BytePointer text, int fontFace,
                            double fontScale, int thickness,
                            IntBuffer baseLine);
@Namespace("cv") public static native @ByVal Size getTextSize(@Str String text, int fontFace,
                            double fontScale, int thickness,
                            int[] baseLine);

 // cv

// #endif


}
