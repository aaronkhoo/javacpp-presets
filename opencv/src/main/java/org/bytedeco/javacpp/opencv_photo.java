// Targeted by JavaCPP version 0.11

package org.bytedeco.javacpp;

import java.nio.*;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import static org.bytedeco.javacpp.opencv_core.*;
import static org.bytedeco.javacpp.opencv_imgproc.*;

public class opencv_photo extends org.bytedeco.javacpp.presets.opencv_photo {
    static { Loader.load(); }

// Parsed from <opencv2/photo/photo_c.h>

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
// Copyright (C) 2008-2012, Willow Garage Inc., all rights reserved.
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

// #ifndef __OPENCV_PHOTO_C_H__
// #define __OPENCV_PHOTO_C_H__

// #include "opencv2/core/core_c.h"

// #ifdef __cplusplus
// #endif

/* Inpainting algorithms */
/** enum  */
public static final int
    CV_INPAINT_NS      = 0,
    CV_INPAINT_TELEA   = 1;


/* Inpaints the selected region in the image */
public static native void cvInpaint( @Const CvArr src, @Const CvArr inpaint_mask,
                       CvArr dst, double inpaintRange, int flags );


// #ifdef __cplusplus //extern "C"
// #endif

// #endif //__OPENCV_PHOTO_C_H__


// Parsed from <opencv2/photo.hpp>

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
// Copyright (C) 2008-2012, Willow Garage Inc., all rights reserved.
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

// #ifndef __OPENCV_PHOTO_HPP__
// #define __OPENCV_PHOTO_HPP__

// #include "opencv2/core.hpp"
// #include "opencv2/imgproc.hpp"

/** \namespace cv
  Namespace where all the C++ OpenCV functionality resides
 */

/** the inpainting algorithm */
/** enum cv:: */
public static final int
    INPAINT_NS    = 0, // Navier-Stokes algorithm
    INPAINT_TELEA = 1; // A. Telea algorithm

/** enum cv:: */
public static final int
    NORMAL_CLONE = 1,
    MIXED_CLONE  = 2,
    MONOCHROME_TRANSFER = 3;

/** enum cv:: */
public static final int
    RECURS_FILTER = 1,
    NORMCONV_FILTER = 2;

/** restores the damaged image areas using one of the available intpainting algorithms */
@Namespace("cv") public static native void inpaint( @ByVal Mat src, @ByVal Mat inpaintMask,
        @ByVal Mat dst, double inpaintRadius, int flags );


@Namespace("cv") public static native void fastNlMeansDenoising( @ByVal Mat src, @ByVal Mat dst, float h/*=3*/,
        int templateWindowSize/*=7*/, int searchWindowSize/*=21*/);
@Namespace("cv") public static native void fastNlMeansDenoising( @ByVal Mat src, @ByVal Mat dst);

@Namespace("cv") public static native void fastNlMeansDenoisingColored( @ByVal Mat src, @ByVal Mat dst,
        float h/*=3*/, float hColor/*=3*/,
        int templateWindowSize/*=7*/, int searchWindowSize/*=21*/);
@Namespace("cv") public static native void fastNlMeansDenoisingColored( @ByVal Mat src, @ByVal Mat dst);

@Namespace("cv") public static native void fastNlMeansDenoisingMulti( @ByVal MatVector srcImgs, @ByVal Mat dst,
        int imgToDenoiseIndex, int temporalWindowSize,
        float h/*=3*/, int templateWindowSize/*=7*/, int searchWindowSize/*=21*/);
@Namespace("cv") public static native void fastNlMeansDenoisingMulti( @ByVal MatVector srcImgs, @ByVal Mat dst,
        int imgToDenoiseIndex, int temporalWindowSize);

@Namespace("cv") public static native void fastNlMeansDenoisingColoredMulti( @ByVal MatVector srcImgs, @ByVal Mat dst,
        int imgToDenoiseIndex, int temporalWindowSize,
        float h/*=3*/, float hColor/*=3*/,
        int templateWindowSize/*=7*/, int searchWindowSize/*=21*/);
@Namespace("cv") public static native void fastNlMeansDenoisingColoredMulti( @ByVal MatVector srcImgs, @ByVal Mat dst,
        int imgToDenoiseIndex, int temporalWindowSize);

@Namespace("cv") public static native void denoise_TVL1(@Const @ByRef MatVector observations,@ByRef Mat result, double lambda/*=1.0*/, int niters/*=30*/);
@Namespace("cv") public static native void denoise_TVL1(@Const @ByRef MatVector observations,@ByRef Mat result);

/** enum cv:: */
public static final int LDR_SIZE = 256;

@Namespace("cv") public static class Tonemap extends Algorithm {
    static { Loader.load(); }
    /** Empty constructor. */
    public Tonemap() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Tonemap(Pointer p) { super(p); }

    public native void process(@ByVal Mat src, @ByVal Mat dst);

    public native float getGamma();
    public native void setGamma(float gamma);
}

@Namespace("cv") public static native @Ptr Tonemap createTonemap(float gamma/*=1.0f*/);
@Namespace("cv") public static native @Ptr Tonemap createTonemap();

// "Adaptive Logarithmic Mapping For Displaying HighContrast Scenes", Drago et al., 2003

@Namespace("cv") public static class TonemapDrago extends Tonemap {
    static { Loader.load(); }
    /** Empty constructor. */
    public TonemapDrago() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public TonemapDrago(Pointer p) { super(p); }


    public native float getSaturation();
    public native void setSaturation(float saturation);

    public native float getBias();
    public native void setBias(float bias);
}

@Namespace("cv") public static native @Ptr TonemapDrago createTonemapDrago(float gamma/*=1.0f*/, float saturation/*=1.0f*/, float bias/*=0.85f*/);
@Namespace("cv") public static native @Ptr TonemapDrago createTonemapDrago();

// "Fast Bilateral Filtering for the Display of High-Dynamic-Range Images", Durand, Dorsey, 2002

@Namespace("cv") public static class TonemapDurand extends Tonemap {
    static { Loader.load(); }
    /** Empty constructor. */
    public TonemapDurand() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public TonemapDurand(Pointer p) { super(p); }


    public native float getSaturation();
    public native void setSaturation(float saturation);

    public native float getContrast();
    public native void setContrast(float contrast);

    public native float getSigmaSpace();
    public native void setSigmaSpace(float sigma_space);

    public native float getSigmaColor();
    public native void setSigmaColor(float sigma_color);
}

@Namespace("cv") public static native @Ptr TonemapDurand createTonemapDurand(float gamma/*=1.0f*/, float contrast/*=4.0f*/, float saturation/*=1.0f*/, float sigma_space/*=2.0f*/, float sigma_color/*=2.0f*/);
@Namespace("cv") public static native @Ptr TonemapDurand createTonemapDurand();

// "Dynamic Range Reduction Inspired by Photoreceptor Physiology", Reinhard, Devlin, 2005

@Namespace("cv") public static class TonemapReinhard extends Tonemap {
    static { Loader.load(); }
    /** Empty constructor. */
    public TonemapReinhard() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public TonemapReinhard(Pointer p) { super(p); }

    public native float getIntensity();
    public native void setIntensity(float intensity);

    public native float getLightAdaptation();
    public native void setLightAdaptation(float light_adapt);

    public native float getColorAdaptation();
    public native void setColorAdaptation(float color_adapt);
}

@Namespace("cv") public static native @Ptr TonemapReinhard createTonemapReinhard(float gamma/*=1.0f*/, float intensity/*=0.0f*/, float light_adapt/*=1.0f*/, float color_adapt/*=0.0f*/);
@Namespace("cv") public static native @Ptr TonemapReinhard createTonemapReinhard();

// "Perceptual Framework for Contrast Processing of High Dynamic Range Images", Mantiuk et al., 2006

@Namespace("cv") public static class TonemapMantiuk extends Tonemap {
    static { Loader.load(); }
    /** Empty constructor. */
    public TonemapMantiuk() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public TonemapMantiuk(Pointer p) { super(p); }

    public native float getScale();
    public native void setScale(float scale);

    public native float getSaturation();
    public native void setSaturation(float saturation);
}

@Namespace("cv") public static native @Ptr TonemapMantiuk createTonemapMantiuk(float gamma/*=1.0f*/, float scale/*=0.7f*/, float saturation/*=1.0f*/);
@Namespace("cv") public static native @Ptr TonemapMantiuk createTonemapMantiuk();

@Namespace("cv") public static class AlignExposures extends Algorithm {
    static { Loader.load(); }
    /** Empty constructor. */
    public AlignExposures() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public AlignExposures(Pointer p) { super(p); }

    public native void process(@ByVal MatVector src, @ByRef MatVector dst,
                                     @ByVal Mat times, @ByVal Mat response);
}

// "Fast, Robust Image Registration for Compositing High Dynamic Range Photographs from Handheld Exposures", Ward, 2003

@Namespace("cv") public static class AlignMTB extends AlignExposures {
    static { Loader.load(); }
    /** Empty constructor. */
    public AlignMTB() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public AlignMTB(Pointer p) { super(p); }

    public native void process(@ByVal MatVector src, @ByRef MatVector dst,
                                     @ByVal Mat times, @ByVal Mat response);

    public native void process(@ByVal MatVector src, @ByRef MatVector dst);

    public native @ByVal Point calculateShift(@ByVal Mat img0, @ByVal Mat img1);
    public native void shiftMat(@ByVal Mat src, @ByVal Mat dst, @Const @ByVal Point shift);
    public native void computeBitmaps(@ByVal Mat img, @ByVal Mat tb, @ByVal Mat eb);

    public native int getMaxBits();
    public native void setMaxBits(int max_bits);

    public native int getExcludeRange();
    public native void setExcludeRange(int exclude_range);

    public native @Cast("bool") boolean getCut();
    public native void setCut(@Cast("bool") boolean value);
}

@Namespace("cv") public static native @Ptr AlignMTB createAlignMTB(int max_bits/*=6*/, int exclude_range/*=4*/, @Cast("bool") boolean cut/*=true*/);
@Namespace("cv") public static native @Ptr AlignMTB createAlignMTB();

@Namespace("cv") public static class CalibrateCRF extends Algorithm {
    static { Loader.load(); }
    /** Empty constructor. */
    public CalibrateCRF() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CalibrateCRF(Pointer p) { super(p); }

    public native void process(@ByVal MatVector src, @ByVal Mat dst, @ByVal Mat times);
}

// "Recovering High Dynamic Range Radiance Maps from Photographs", Debevec, Malik, 1997

@Namespace("cv") public static class CalibrateDebevec extends CalibrateCRF {
    static { Loader.load(); }
    /** Empty constructor. */
    public CalibrateDebevec() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CalibrateDebevec(Pointer p) { super(p); }

    public native float getLambda();
    public native void setLambda(float lambda);

    public native int getSamples();
    public native void setSamples(int samples);

    public native @Cast("bool") boolean getRandom();
    public native void setRandom(@Cast("bool") boolean random);
}

@Namespace("cv") public static native @Ptr CalibrateDebevec createCalibrateDebevec(int samples/*=70*/, float lambda/*=10.0f*/, @Cast("bool") boolean random/*=false*/);
@Namespace("cv") public static native @Ptr CalibrateDebevec createCalibrateDebevec();

// "Dynamic range improvement through multiple exposures", Robertson et al., 1999

@Namespace("cv") public static class CalibrateRobertson extends CalibrateCRF {
    static { Loader.load(); }
    /** Empty constructor. */
    public CalibrateRobertson() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CalibrateRobertson(Pointer p) { super(p); }

    public native int getMaxIter();
    public native void setMaxIter(int max_iter);

    public native float getThreshold();
    public native void setThreshold(float threshold);

    public native @ByVal Mat getRadiance();
}

@Namespace("cv") public static native @Ptr CalibrateRobertson createCalibrateRobertson(int max_iter/*=30*/, float threshold/*=0.01f*/);
@Namespace("cv") public static native @Ptr CalibrateRobertson createCalibrateRobertson();

@Namespace("cv") public static class MergeExposures extends Algorithm {
    static { Loader.load(); }
    /** Empty constructor. */
    public MergeExposures() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public MergeExposures(Pointer p) { super(p); }

    public native void process(@ByVal MatVector src, @ByVal Mat dst,
                                     @ByVal Mat times, @ByVal Mat response);
}

// "Recovering High Dynamic Range Radiance Maps from Photographs", Debevec, Malik, 1997

@Namespace("cv") public static class MergeDebevec extends MergeExposures {
    static { Loader.load(); }
    /** Empty constructor. */
    public MergeDebevec() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public MergeDebevec(Pointer p) { super(p); }

    public native void process(@ByVal MatVector src, @ByVal Mat dst,
                                     @ByVal Mat times, @ByVal Mat response);
    public native void process(@ByVal MatVector src, @ByVal Mat dst, @ByVal Mat times);
}

@Namespace("cv") public static native @Ptr MergeDebevec createMergeDebevec();

// "Exposure Fusion", Mertens et al., 2007

@Namespace("cv") public static class MergeMertens extends MergeExposures {
    static { Loader.load(); }
    /** Empty constructor. */
    public MergeMertens() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public MergeMertens(Pointer p) { super(p); }

    public native void process(@ByVal MatVector src, @ByVal Mat dst,
                                     @ByVal Mat times, @ByVal Mat response);
    public native void process(@ByVal MatVector src, @ByVal Mat dst);

    public native float getContrastWeight();
    public native void setContrastWeight(float contrast_weiht);

    public native float getSaturationWeight();
    public native void setSaturationWeight(float saturation_weight);

    public native float getExposureWeight();
    public native void setExposureWeight(float exposure_weight);
}

@Namespace("cv") public static native @Ptr MergeMertens createMergeMertens(float contrast_weight/*=1.0f*/, float saturation_weight/*=1.0f*/, float exposure_weight/*=0.0f*/);
@Namespace("cv") public static native @Ptr MergeMertens createMergeMertens();

// "Dynamic range improvement through multiple exposures", Robertson et al., 1999

@Namespace("cv") public static class MergeRobertson extends MergeExposures {
    static { Loader.load(); }
    /** Empty constructor. */
    public MergeRobertson() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public MergeRobertson(Pointer p) { super(p); }

    public native void process(@ByVal MatVector src, @ByVal Mat dst,
                                     @ByVal Mat times, @ByVal Mat response);
    public native void process(@ByVal MatVector src, @ByVal Mat dst, @ByVal Mat times);
}

@Namespace("cv") public static native @Ptr MergeRobertson createMergeRobertson();

@Namespace("cv") public static native void decolor( @ByVal Mat src, @ByVal Mat grayscale, @ByVal Mat color_boost);

@Namespace("cv") public static native void seamlessClone( @ByVal Mat src, @ByVal Mat dst, @ByVal Mat mask, @ByVal Point p,
        @ByVal Mat blend, int flags);

@Namespace("cv") public static native void colorChange(@ByVal Mat src, @ByVal Mat mask, @ByVal Mat dst, float red_mul/*=1.0f*/,
        float green_mul/*=1.0f*/, float blue_mul/*=1.0f*/);
@Namespace("cv") public static native void colorChange(@ByVal Mat src, @ByVal Mat mask, @ByVal Mat dst);

@Namespace("cv") public static native void illuminationChange(@ByVal Mat src, @ByVal Mat mask, @ByVal Mat dst,
        float alpha/*=0.2f*/, float beta/*=0.4f*/);
@Namespace("cv") public static native void illuminationChange(@ByVal Mat src, @ByVal Mat mask, @ByVal Mat dst);

@Namespace("cv") public static native void textureFlattening(@ByVal Mat src, @ByVal Mat mask, @ByVal Mat dst,
        float low_threshold/*=30*/, float high_threshold/*=45*/,
        int kernel_size/*=3*/);
@Namespace("cv") public static native void textureFlattening(@ByVal Mat src, @ByVal Mat mask, @ByVal Mat dst);

@Namespace("cv") public static native void edgePreservingFilter(@ByVal Mat src, @ByVal Mat dst, int flags/*=1*/,
        float sigma_s/*=60*/, float sigma_r/*=0.4f*/);
@Namespace("cv") public static native void edgePreservingFilter(@ByVal Mat src, @ByVal Mat dst);

@Namespace("cv") public static native void detailEnhance(@ByVal Mat src, @ByVal Mat dst, float sigma_s/*=10*/,
        float sigma_r/*=0.15f*/);
@Namespace("cv") public static native void detailEnhance(@ByVal Mat src, @ByVal Mat dst);

@Namespace("cv") public static native void pencilSketch(@ByVal Mat src, @ByVal Mat dst1, @ByVal Mat dst2,
        float sigma_s/*=60*/, float sigma_r/*=0.07f*/, float shade_factor/*=0.02f*/);
@Namespace("cv") public static native void pencilSketch(@ByVal Mat src, @ByVal Mat dst1, @ByVal Mat dst2);

@Namespace("cv") public static native void stylization(@ByVal Mat src, @ByVal Mat dst, float sigma_s/*=60*/,
        float sigma_r/*=0.45f*/);
@Namespace("cv") public static native void stylization(@ByVal Mat src, @ByVal Mat dst);

 // cv

// #endif


// Parsed from <opencv2/photo/cuda.hpp>

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
// Copyright (C) 2008-2012, Willow Garage Inc., all rights reserved.
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

// #ifndef __OPENCV_PHOTO_CUDA_HPP__
// #define __OPENCV_PHOTO_CUDA_HPP__

// #include "opencv2/core/cuda.hpp"

/** Brute force non-local means algorith (slow but universal) */
@Namespace("cv::cuda") public static native void nonLocalMeans(@Const @ByRef GpuMat src, @ByRef GpuMat dst, float h, int search_window/*=21*/, int block_size/*=7*/, int borderMode/*=BORDER_DEFAULT*/, @ByRef Stream s/*=Stream::Null()*/);
@Namespace("cv::cuda") public static native void nonLocalMeans(@Const @ByRef GpuMat src, @ByRef GpuMat dst, float h);

/** Fast (but approximate)version of non-local means algorith similar to CPU function (running sums technique) */
@Namespace("cv::cuda") public static class FastNonLocalMeansDenoising extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public FastNonLocalMeansDenoising() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public FastNonLocalMeansDenoising(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public FastNonLocalMeansDenoising(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public FastNonLocalMeansDenoising position(int position) {
        return (FastNonLocalMeansDenoising)super.position(position);
    }

    /** Simple method, recommended for grayscale images (though it supports multichannel images) */
    public native void simpleMethod(@Const @ByRef GpuMat src, @ByRef GpuMat dst, float h, int search_window/*=21*/, int block_size/*=7*/, @ByRef Stream s/*=Stream::Null()*/);
    public native void simpleMethod(@Const @ByRef GpuMat src, @ByRef GpuMat dst, float h);

    /** Processes luminance and color components separatelly */
    public native void labMethod(@Const @ByRef GpuMat src, @ByRef GpuMat dst, float h_luminance, float h_color, int search_window/*=21*/, int block_size/*=7*/, @ByRef Stream s/*=Stream::Null()*/);
    public native void labMethod(@Const @ByRef GpuMat src, @ByRef GpuMat dst, float h_luminance, float h_color);
}

 // namespace cv { namespace cuda {

// #endif /* __OPENCV_PHOTO_CUDA_HPP__ */


}
