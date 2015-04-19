// Targeted by JavaCPP version 0.11

package org.bytedeco.javacpp;

import org.bytedeco.javacpp.annotation.Index;
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

public class opencv_features2d extends org.bytedeco.javacpp.presets.opencv_features2d {
    static { Loader.load(); }

@Name("std::vector<std::vector<cv::KeyPoint> >") public static class KeyPointVectorVector extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public KeyPointVectorVector(Pointer p) { super(p); }
    public KeyPointVectorVector(KeyPoint[] ... array) { this(array.length); put(array); }
    public KeyPointVectorVector()       { allocate();  }
    public KeyPointVectorVector(long n) { allocate(n); }
    private native void allocate();
    private native void allocate(@Cast("size_t") long n);
    public native @Name("operator=") @ByRef KeyPointVectorVector put(@ByRef KeyPointVectorVector x);

    public native long size();
    public native void resize(@Cast("size_t") long n);
    public native @Index long size(@Cast("size_t") long i);
    public native @Index void resize(@Cast("size_t") long i, @Cast("size_t") long n);

    @Index public native @ByRef KeyPoint get(@Cast("size_t") long i, @Cast("size_t") long j);
    public native KeyPointVectorVector put(@Cast("size_t") long i, @Cast("size_t") long j, KeyPoint value);

    public KeyPointVectorVector put(KeyPoint[] ... array) {
        if (size() != array.length) { resize(array.length); }
        for (int i = 0; i < array.length; i++) {
            if (size(i) != array[i].length) { resize(i, array[i].length); }
            for (int j = 0; j < array[i].length; j++) {
                put(i, j, array[i][j]);
            }
        }
        return this;
    }
}

@Name("std::vector<std::vector<cv::DMatch> >") public static class DMatchVectorVector extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public DMatchVectorVector(Pointer p) { super(p); }
    public DMatchVectorVector(DMatch[] ... array) { this(array.length); put(array); }
    public DMatchVectorVector()       { allocate();  }
    public DMatchVectorVector(long n) { allocate(n); }
    private native void allocate();
    private native void allocate(@Cast("size_t") long n);
    public native @Name("operator=") @ByRef DMatchVectorVector put(@ByRef DMatchVectorVector x);

    public native long size();
    public native void resize(@Cast("size_t") long n);
    public native @Index long size(@Cast("size_t") long i);
    public native @Index void resize(@Cast("size_t") long i, @Cast("size_t") long n);

    @Index public native @ByRef DMatch get(@Cast("size_t") long i, @Cast("size_t") long j);
    public native DMatchVectorVector put(@Cast("size_t") long i, @Cast("size_t") long j, DMatch value);

    public DMatchVectorVector put(DMatch[] ... array) {
        if (size() != array.length) { resize(array.length); }
        for (int i = 0; i < array.length; i++) {
            if (size(i) != array[i].length) { resize(i, array[i].length); }
            for (int j = 0; j < array[i].length; j++) {
                put(i, j, array[i][j]);
            }
        }
        return this;
    }
}

// Parsed from <opencv2/features2d.hpp>

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

// #ifndef __OPENCV_FEATURES_2D_HPP__
// #define __OPENCV_FEATURES_2D_HPP__

// #include "opencv2/core.hpp"
// #include "opencv2/flann/miniflann.hpp"

// //! writes vector of keypoints to the file storage
// CV_EXPORTS void write(FileStorage& fs, const String& name, const std::vector<KeyPoint>& keypoints);
// //! reads vector of keypoints from the specified file storage node
// CV_EXPORTS void read(const FileNode& node, CV_OUT std::vector<KeyPoint>& keypoints);

/*
 * A class filters a vector of keypoints.
 * Because now it is difficult to provide a convenient interface for all usage scenarios of the keypoints filter class,
 * it has only several needed by now static methods.
 */
@Namespace("cv") public static class KeyPointsFilter extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public KeyPointsFilter(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public KeyPointsFilter(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public KeyPointsFilter position(int position) {
        return (KeyPointsFilter)super.position(position);
    }

    public KeyPointsFilter() { allocate(); }
    private native void allocate();

    /*
     * Remove keypoints within borderPixels of an image edge.
     */
    public static native void runByImageBorder( @StdVector KeyPoint keypoints, @ByVal Size imageSize, int borderSize );
    /*
     * Remove keypoints of sizes out of range.
     */
    public static native void runByKeypointSize( @StdVector KeyPoint keypoints, float minSize,
                                       float maxSize/*=FLT_MAX*/ );
    public static native void runByKeypointSize( @StdVector KeyPoint keypoints, float minSize );
    /*
     * Remove keypoints from some image by mask for pixels of this image.
     */
    public static native void runByPixelsMask( @StdVector KeyPoint keypoints, @Const @ByRef Mat mask );
    /*
     * Remove duplicated keypoints.
     */
    public static native void removeDuplicated( @StdVector KeyPoint keypoints );

    /*
     * Retain the specified number of the best keypoints (according to the response)
     */
    public static native void retainBest( @StdVector KeyPoint keypoints, int npoints );
}


/************************************ Base Classes ************************************/

/*
 * Abstract base class for 2D image feature detectors and descriptor extractors
 */
@Namespace("cv") public static class Feature2D extends Algorithm {
    static { Loader.load(); }
    /** Default native constructor. */
    public Feature2D() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Feature2D(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Feature2D(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public Feature2D position(int position) {
        return (Feature2D)super.position(position);
    }


    /*
     * Detect keypoints in an image.
     * image        The image.
     * keypoints    The detected keypoints.
     * mask         Mask specifying where to look for keypoints (optional). Must be a char
     *              matrix with non-zero values in the region of interest.
     */
    public native void detect( @ByVal Mat image,
                                     @StdVector KeyPoint keypoints,
                                     @ByVal Mat mask/*=noArray()*/ );
    public native void detect( @ByVal Mat image,
                                     @StdVector KeyPoint keypoints );

    public native void detect( @ByVal MatVector images,
                             @ByRef KeyPointVectorVector keypoints,
                             @ByVal MatVector masks/*=noArray()*/ );
    public native void detect( @ByVal MatVector images,
                             @ByRef KeyPointVectorVector keypoints );

    /*
     * Compute the descriptors for a set of keypoints in an image.
     * image        The image.
     * keypoints    The input keypoints. Keypoints for which a descriptor cannot be computed are removed.
     * descriptors  Copmputed descriptors. Row i is the descriptor for keypoint i.
     */
    public native void compute( @ByVal Mat image,
                                      @StdVector KeyPoint keypoints,
                                      @ByVal Mat descriptors );

    public native void compute( @ByVal MatVector images,
                              @ByRef KeyPointVectorVector keypoints,
                              @ByVal MatVector descriptors );

    /* Detects keypoints and computes the descriptors */
    public native void detectAndCompute( @ByVal Mat image, @ByVal Mat mask,
                                               @StdVector KeyPoint keypoints,
                                               @ByVal Mat descriptors,
                                               @Cast("bool") boolean useProvidedKeypoints/*=false*/ );
    public native void detectAndCompute( @ByVal Mat image, @ByVal Mat mask,
                                               @StdVector KeyPoint keypoints,
                                               @ByVal Mat descriptors );

    public native int descriptorSize();
    public native int descriptorType();
    public native int defaultNorm();

    // Return true if detector object is empty
    public native @Cast("bool") boolean empty();
}

/**
  BRISK implementation
*/
@Namespace("cv") public static class BRISK extends Feature2D {
    static { Loader.load(); }
    /** Default native constructor. */
    public BRISK() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public BRISK(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public BRISK(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public BRISK position(int position) {
        return (BRISK)super.position(position);
    }

    public static native @Ptr BRISK create(int thresh/*=30*/, int octaves/*=3*/, float patternScale/*=1.0f*/);
    public static native @Ptr BRISK create();
    // custom setup
    public static native @Ptr BRISK create(@StdVector FloatPointer radiusList, @StdVector IntPointer numberList,
            float dMax/*=5.85f*/, float dMin/*=8.2f*/, @StdVector IntPointer indexChange/*=std::vector<int>()*/);
    public static native @Ptr BRISK create(@StdVector FloatPointer radiusList, @StdVector IntPointer numberList);
    public static native @Ptr BRISK create(@StdVector FloatBuffer radiusList, @StdVector IntBuffer numberList,
            float dMax/*=5.85f*/, float dMin/*=8.2f*/, @StdVector IntBuffer indexChange/*=std::vector<int>()*/);
    public static native @Ptr BRISK create(@StdVector FloatBuffer radiusList, @StdVector IntBuffer numberList);
    public static native @Ptr BRISK create(@StdVector float[] radiusList, @StdVector int[] numberList,
            float dMax/*=5.85f*/, float dMin/*=8.2f*/, @StdVector int[] indexChange/*=std::vector<int>()*/);
    public static native @Ptr BRISK create(@StdVector float[] radiusList, @StdVector int[] numberList);
}

/**
 ORB implementation.
*/
@Namespace("cv") public static class ORB extends Feature2D {
    static { Loader.load(); }
    /** Empty constructor. */
    public ORB() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public ORB(Pointer p) { super(p); }

    /** enum cv::ORB:: */
    public static final int kBytes = 32, HARRIS_SCORE= 0, FAST_SCORE= 1;

    public static native @Ptr ORB create(int nfeatures/*=500*/, float scaleFactor/*=1.2f*/, int nlevels/*=8*/, int edgeThreshold/*=31*/,
            int firstLevel/*=0*/, int WTA_K/*=2*/, int scoreType/*=ORB::HARRIS_SCORE*/, int patchSize/*=31*/, int fastThreshold/*=20*/);
    public static native @Ptr ORB create();

    public native void setMaxFeatures(int maxFeatures);
    public native int getMaxFeatures();

    public native void setScaleFactor(double scaleFactor);
    public native double getScaleFactor();

    public native void setNLevels(int nlevels);
    public native int getNLevels();

    public native void setEdgeThreshold(int edgeThreshold);
    public native int getEdgeThreshold();

    public native void setFirstLevel(int firstLevel);
    public native int getFirstLevel();

    public native void setWTA_K(int wta_k);
    public native int getWTA_K();

    public native void setScoreType(int scoreType);
    public native int getScoreType();

    public native void setPatchSize(int patchSize);
    public native int getPatchSize();

    public native void setFastThreshold(int fastThreshold);
    public native int getFastThreshold();
}

/**
 Maximal Stable Extremal Regions class.

 The class implements MSER algorithm introduced by J. Matas.
 Unlike SIFT, SURF and many other detectors in OpenCV, this is salient region detector,
 not the salient point detector.

 It returns the regions, each of those is encoded as a contour.
*/
@Namespace("cv") public static class MSER extends Feature2D {
    static { Loader.load(); }
    /** Empty constructor. */
    public MSER() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public MSER(Pointer p) { super(p); }

    /** the full constructor */
    public static native @Ptr MSER create( int _delta/*=5*/, int _min_area/*=60*/, int _max_area/*=14400*/,
              double _max_variation/*=0.25*/, double _min_diversity/*=.2*/,
              int _max_evolution/*=200*/, double _area_threshold/*=1.01*/,
              double _min_margin/*=0.003*/, int _edge_blur_size/*=5*/ );
    public static native @Ptr MSER create( );

    public native void detectRegions( @ByVal Mat image,
                                            @ByRef PointVectorVector msers,
                                            @StdVector Rect bboxes );

    public native void setDelta(int delta);
    public native int getDelta();

    public native void setMinArea(int minArea);
    public native int getMinArea();

    public native void setMaxArea(int maxArea);
    public native int getMaxArea();

    public native void setPass2Only(@Cast("bool") boolean f);
    public native @Cast("bool") boolean getPass2Only();
}

/** detects corners using FAST algorithm by E. Rosten */
@Namespace("cv") public static native void FAST( @ByVal Mat image, @StdVector KeyPoint keypoints,
                      int threshold, @Cast("bool") boolean nonmaxSuppression/*=true*/ );
@Namespace("cv") public static native void FAST( @ByVal Mat image, @StdVector KeyPoint keypoints,
                      int threshold );

@Namespace("cv") public static native void FAST( @ByVal Mat image, @StdVector KeyPoint keypoints,
                      int threshold, @Cast("bool") boolean nonmaxSuppression, int type );

@Namespace("cv") public static class FastFeatureDetector extends Feature2D {
    static { Loader.load(); }
    /** Empty constructor. */
    public FastFeatureDetector() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public FastFeatureDetector(Pointer p) { super(p); }

    /** enum cv::FastFeatureDetector:: */
    public static final int
        TYPE_5_8 = 0, TYPE_7_12 = 1, TYPE_9_16 = 2,
        THRESHOLD = 10000, NONMAX_SUPPRESSION= 10001, FAST_N= 10002;

    public static native @Ptr FastFeatureDetector create( int threshold/*=10*/,
                                                        @Cast("bool") boolean nonmaxSuppression/*=true*/,
                                                        int type/*=FastFeatureDetector::TYPE_9_16*/ );
    public static native @Ptr FastFeatureDetector create( );

    public native void setThreshold(int threshold);
    public native int getThreshold();

    public native void setNonmaxSuppression(@Cast("bool") boolean f);
    public native @Cast("bool") boolean getNonmaxSuppression();

    public native void setType(int type);
    public native int getType();
}


@Namespace("cv") public static class GFTTDetector extends Feature2D {
    static { Loader.load(); }
    /** Empty constructor. */
    public GFTTDetector() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public GFTTDetector(Pointer p) { super(p); }

    public static native @Ptr GFTTDetector create( int maxCorners/*=1000*/, double qualityLevel/*=0.01*/, double minDistance/*=1*/,
                                                 int blockSize/*=3*/, @Cast("bool") boolean useHarrisDetector/*=false*/, double k/*=0.04*/ );
    public static native @Ptr GFTTDetector create( );
    public native void setMaxFeatures(int maxFeatures);
    public native int getMaxFeatures();

    public native void setQualityLevel(double qlevel);
    public native double getQualityLevel();

    public native void setMinDistance(double minDistance);
    public native double getMinDistance();

    public native void setBlockSize(int blockSize);
    public native int getBlockSize();

    public native void setHarrisDetector(@Cast("bool") boolean val);
    public native @Cast("bool") boolean getHarrisDetector();

    public native void setK(double k);
    public native double getK();
}


@Namespace("cv") public static class SimpleBlobDetector extends Feature2D {
    static { Loader.load(); }
    /** Default native constructor. */
    public SimpleBlobDetector() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public SimpleBlobDetector(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public SimpleBlobDetector(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public SimpleBlobDetector position(int position) {
        return (SimpleBlobDetector)super.position(position);
    }

  @NoOffset public static class Params extends Pointer {
      static { Loader.load(); }
      /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
      public Params(Pointer p) { super(p); }
      /** Native array allocator. Access with {@link Pointer#position(int)}. */
      public Params(int size) { allocateArray(size); }
      private native void allocateArray(int size);
      @Override public Params position(int position) {
          return (Params)super.position(position);
      }
  
      public Params() { allocate(); }
      private native void allocate();
      public native float thresholdStep(); public native Params thresholdStep(float thresholdStep);
      public native float minThreshold(); public native Params minThreshold(float minThreshold);
      public native float maxThreshold(); public native Params maxThreshold(float maxThreshold);
      public native @Cast("size_t") long minRepeatability(); public native Params minRepeatability(long minRepeatability);
      public native float minDistBetweenBlobs(); public native Params minDistBetweenBlobs(float minDistBetweenBlobs);

      public native @Cast("bool") boolean filterByColor(); public native Params filterByColor(boolean filterByColor);
      public native @Cast("uchar") byte blobColor(); public native Params blobColor(byte blobColor);

      public native @Cast("bool") boolean filterByArea(); public native Params filterByArea(boolean filterByArea);
      public native float minArea(); public native Params minArea(float minArea);
      public native float maxArea(); public native Params maxArea(float maxArea);

      public native @Cast("bool") boolean filterByCircularity(); public native Params filterByCircularity(boolean filterByCircularity);
      public native float minCircularity(); public native Params minCircularity(float minCircularity);
      public native float maxCircularity(); public native Params maxCircularity(float maxCircularity);

      public native @Cast("bool") boolean filterByInertia(); public native Params filterByInertia(boolean filterByInertia);
      public native float minInertiaRatio(); public native Params minInertiaRatio(float minInertiaRatio);
      public native float maxInertiaRatio(); public native Params maxInertiaRatio(float maxInertiaRatio);

      public native @Cast("bool") boolean filterByConvexity(); public native Params filterByConvexity(boolean filterByConvexity);
      public native float minConvexity(); public native Params minConvexity(float minConvexity);
      public native float maxConvexity(); public native Params maxConvexity(float maxConvexity);

      public native void read( @Const @ByRef FileNode fn );
      public native void write( @ByRef FileStorage fs );
  }

  public static native @Ptr SimpleBlobDetector create(@Const @ByRef Params parameters/*=SimpleBlobDetector::Params()*/);
  public static native @Ptr SimpleBlobDetector create();
}


/**
KAZE implementation
*/
@Namespace("cv") public static class KAZE extends Feature2D {
    static { Loader.load(); }
    /** Empty constructor. */
    public KAZE() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public KAZE(Pointer p) { super(p); }

    /** enum cv::KAZE:: */
    public static final int
        DIFF_PM_G1 = 0,
        DIFF_PM_G2 = 1,
        DIFF_WEICKERT = 2,
        DIFF_CHARBONNIER = 3;

    public static native @Ptr KAZE create(@Cast("bool") boolean extended/*=false*/, @Cast("bool") boolean upright/*=false*/,
                                        float threshold/*=0.001f*/,
                                        int nOctaves/*=4*/, int nOctaveLayers/*=4*/,
                                        int diffusivity/*=KAZE::DIFF_PM_G2*/);
    public static native @Ptr KAZE create();

    public native void setExtended(@Cast("bool") boolean extended);
    public native @Cast("bool") boolean getExtended();

    public native void setUpright(@Cast("bool") boolean upright);
    public native @Cast("bool") boolean getUpright();

    public native void setThreshold(double threshold);
    public native double getThreshold();

    public native void setNOctaves(int octaves);
    public native int getNOctaves();

    public native void setNOctaveLayers(int octaveLayers);
    public native int getNOctaveLayers();

    public native void setDiffusivity(int diff);
    public native int getDiffusivity();
}

/**
AKAZE implementation
*/
@Namespace("cv") public static class AKAZE extends Feature2D {
    static { Loader.load(); }
    /** Empty constructor. */
    public AKAZE() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public AKAZE(Pointer p) { super(p); }

    // AKAZE descriptor type
    /** enum cv::AKAZE:: */
    public static final int
        /** Upright descriptors, not invariant to rotation */
        DESCRIPTOR_KAZE_UPRIGHT = 2,
        DESCRIPTOR_KAZE = 3,
        /** Upright descriptors, not invariant to rotation */
        DESCRIPTOR_MLDB_UPRIGHT = 4,
        DESCRIPTOR_MLDB = 5;

    public static native @Ptr AKAZE create(int descriptor_type/*=AKAZE::DESCRIPTOR_MLDB*/,
                                         int descriptor_size/*=0*/, int descriptor_channels/*=3*/,
                                         float threshold/*=0.001f*/, int nOctaves/*=4*/,
                                         int nOctaveLayers/*=4*/, int diffusivity/*=KAZE::DIFF_PM_G2*/);
    public static native @Ptr AKAZE create();

    public native void setDescriptorType(int dtype);
    public native int getDescriptorType();

    public native void setDescriptorSize(int dsize);
    public native int getDescriptorSize();

    public native void setDescriptorChannels(int dch);
    public native int getDescriptorChannels();

    public native void setThreshold(double threshold);
    public native double getThreshold();

    public native void setNOctaves(int octaves);
    public native int getNOctaves();

    public native void setNOctaveLayers(int octaveLayers);
    public native int getNOctaveLayers();

    public native void setDiffusivity(int diff);
    public native int getDiffusivity();
}

/****************************************************************************************\
*                                      Distance                                          *
\****************************************************************************************/

@Name("cv::Accumulator<unsigned char>") public static class Accumulator extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public Accumulator() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Accumulator(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Accumulator(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public Accumulator position(int position) {
        return (Accumulator)super.position(position);
    }
 }

/*
 * Squared Euclidean distance functor
 */

/*
 * Euclidean distance functor
 */

/*
 * Manhattan distance (city block distance) functor
 */

/*
 * Hamming distance functor - counts the bit differences between two strings - useful for the Brief descriptor
 * bit count of A exclusive XOR'ed with B
 */
@Namespace("cv") public static class Hamming extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public Hamming() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Hamming(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Hamming(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public Hamming position(int position) {
        return (Hamming)super.position(position);
    }

    /** enum cv::Hamming:: */
    public static final int normType =  NORM_HAMMING;

    /** this will count the bits in a ^ b
     */
    public native @Cast("cv::Hamming::ResultType") @Name("operator()") int apply( @Cast("const unsigned char*") BytePointer a, @Cast("const unsigned char*") BytePointer b, int size );
    public native @Cast("cv::Hamming::ResultType") @Name("operator()") int apply( @Cast("const unsigned char*") ByteBuffer a, @Cast("const unsigned char*") ByteBuffer b, int size );
    public native @Cast("cv::Hamming::ResultType") @Name("operator()") int apply( @Cast("const unsigned char*") byte[] a, @Cast("const unsigned char*") byte[] b, int size );
}

/****************************************************************************************\
*                                  DescriptorMatcher                                     *
\****************************************************************************************/
/*
 * Abstract base class for matching two sets of descriptors.
 */
@Namespace("cv") @NoOffset public static class DescriptorMatcher extends Algorithm {
    static { Loader.load(); }
    /** Empty constructor. */
    public DescriptorMatcher() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public DescriptorMatcher(Pointer p) { super(p); }


    /*
     * Add descriptors to train descriptor collection.
     * descriptors      Descriptors to add. Each descriptors[i] is a descriptors set from one image.
     */
    public native void add( @ByVal MatVector descriptors );
    /*
     * Get train descriptors collection.
     */
    public native @Const @ByRef MatVector getTrainDescriptors();
    /*
     * Clear train descriptors collection.
     */
    public native void clear();

    /*
     * Return true if there are not train descriptors in collection.
     */
    public native @Cast("bool") boolean empty();
    /*
     * Return true if the matcher supports mask in match methods.
     */
    public native @Cast("bool") boolean isMaskSupported();

    /*
     * Train matcher (e.g. train flann index).
     * In all methods to match the method train() is run every time before matching.
     * Some descriptor matchers (e.g. BruteForceMatcher) have empty implementation
     * of this method, other matchers really train their inner structures
     * (e.g. FlannBasedMatcher trains flann::Index). So nonempty implementation
     * of train() should check the class object state and do traing/retraining
     * only if the state requires that (e.g. FlannBasedMatcher trains flann::Index
     * if it has not trained yet or if new descriptors have been added to the train
     * collection).
     */
    public native void train();
    /*
     * Group of methods to match descriptors from image pair.
     * Method train() is run in this methods.
     */
    // Find one best match for each query descriptor (if mask is empty).
    public native void match( @ByVal Mat queryDescriptors, @ByVal Mat trainDescriptors,
                    @StdVector DMatch matches, @ByVal Mat mask/*=noArray()*/ );
    public native void match( @ByVal Mat queryDescriptors, @ByVal Mat trainDescriptors,
                    @StdVector DMatch matches );
    // Find k best matches for each query descriptor (in increasing order of distances).
    // compactResult is used when mask is not empty. If compactResult is false matches
    // vector will have the same size as queryDescriptors rows. If compactResult is true
    // matches vector will not contain matches for fully masked out query descriptors.
    public native void knnMatch( @ByVal Mat queryDescriptors, @ByVal Mat trainDescriptors,
                       @ByRef DMatchVectorVector matches, int k,
                       @ByVal Mat mask/*=noArray()*/, @Cast("bool") boolean compactResult/*=false*/ );
    public native void knnMatch( @ByVal Mat queryDescriptors, @ByVal Mat trainDescriptors,
                       @ByRef DMatchVectorVector matches, int k );
    // Find best matches for each query descriptor which have distance less than
    // maxDistance (in increasing order of distances).
    public native void radiusMatch( @ByVal Mat queryDescriptors, @ByVal Mat trainDescriptors,
                          @ByRef DMatchVectorVector matches, float maxDistance,
                          @ByVal Mat mask/*=noArray()*/, @Cast("bool") boolean compactResult/*=false*/ );
    public native void radiusMatch( @ByVal Mat queryDescriptors, @ByVal Mat trainDescriptors,
                          @ByRef DMatchVectorVector matches, float maxDistance );
    /*
     * Group of methods to match descriptors from one image to image set.
     * See description of similar methods for matching image pair above.
     */
    public native void match( @ByVal Mat queryDescriptors, @StdVector DMatch matches,
                            @ByVal MatVector masks/*=noArray()*/ );
    public native void match( @ByVal Mat queryDescriptors, @StdVector DMatch matches );
    public native void knnMatch( @ByVal Mat queryDescriptors, @ByRef DMatchVectorVector matches, int k,
                               @ByVal MatVector masks/*=noArray()*/, @Cast("bool") boolean compactResult/*=false*/ );
    public native void knnMatch( @ByVal Mat queryDescriptors, @ByRef DMatchVectorVector matches, int k );
    public native void radiusMatch( @ByVal Mat queryDescriptors, @ByRef DMatchVectorVector matches, float maxDistance,
                          @ByVal MatVector masks/*=noArray()*/, @Cast("bool") boolean compactResult/*=false*/ );
    public native void radiusMatch( @ByVal Mat queryDescriptors, @ByRef DMatchVectorVector matches, float maxDistance );

    // Reads matcher object from a file node
    public native void read( @Const @ByRef FileNode arg0 );
    // Writes matcher object to a file storage
    public native void write( @ByRef FileStorage arg0 );

    // Clone the matcher. If emptyTrainData is false the method create deep copy of the object, i.e. copies
    // both parameters and train data. If emptyTrainData is true the method create object copy with current parameters
    // but with empty train data.
    public native @Ptr DescriptorMatcher clone( @Cast("bool") boolean emptyTrainData/*=false*/ );
    public native @Ptr DescriptorMatcher clone( );

    public static native @Ptr DescriptorMatcher create( @Str BytePointer descriptorMatcherType );
    public static native @Ptr DescriptorMatcher create( @Str String descriptorMatcherType );
}

/*
 * Brute-force descriptor matcher.
 *
 * For each descriptor in the first set, this matcher finds the closest
 * descriptor in the second set by trying each one.
 *
 * For efficiency, BruteForceMatcher is templated on the distance metric.
 * For float descriptors, a common choice would be cv::L2<float>.
 */
@Namespace("cv") @NoOffset public static class BFMatcher extends DescriptorMatcher {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public BFMatcher(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public BFMatcher(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public BFMatcher position(int position) {
        return (BFMatcher)super.position(position);
    }

    public BFMatcher( int normType/*=NORM_L2*/, @Cast("bool") boolean crossCheck/*=false*/ ) { allocate(normType, crossCheck); }
    private native void allocate( int normType/*=NORM_L2*/, @Cast("bool") boolean crossCheck/*=false*/ );
    public BFMatcher( ) { allocate(); }
    private native void allocate( );

    public native @Cast("bool") boolean isMaskSupported();

    public native @Ptr DescriptorMatcher clone( @Cast("bool") boolean emptyTrainData/*=false*/ );
    public native @Ptr DescriptorMatcher clone( );
}


/*
 * Flann based matcher
 */
@Namespace("cv") @NoOffset public static class FlannBasedMatcher extends DescriptorMatcher {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public FlannBasedMatcher(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public FlannBasedMatcher(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public FlannBasedMatcher position(int position) {
        return (FlannBasedMatcher)super.position(position);
    }

    public FlannBasedMatcher( @Ptr IndexParams indexParams/*=makePtr<flann::KDTreeIndexParams>()*/,
                           @Ptr SearchParams searchParams/*=makePtr<flann::SearchParams>()*/ ) { allocate(indexParams, searchParams); }
    private native void allocate( @Ptr IndexParams indexParams/*=makePtr<flann::KDTreeIndexParams>()*/,
                           @Ptr SearchParams searchParams/*=makePtr<flann::SearchParams>()*/ );
    public FlannBasedMatcher( ) { allocate(); }
    private native void allocate( );

    public native void add( @ByVal MatVector descriptors );
    public native void clear();

    // Reads matcher object from a file node
    public native void read( @Const @ByRef FileNode arg0 );
    // Writes matcher object to a file storage
    public native void write( @ByRef FileStorage arg0 );

    public native void train();
    public native @Cast("bool") boolean isMaskSupported();

    public native @Ptr DescriptorMatcher clone( @Cast("bool") boolean emptyTrainData/*=false*/ );
    public native @Ptr DescriptorMatcher clone( );
}


/****************************************************************************************\
*                                   Drawing functions                                    *
\****************************************************************************************/
@Namespace("cv") public static class DrawMatchesFlags extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public DrawMatchesFlags() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public DrawMatchesFlags(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public DrawMatchesFlags(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public DrawMatchesFlags position(int position) {
        return (DrawMatchesFlags)super.position(position);
    }

    /** enum cv::DrawMatchesFlags:: */
    public static final int DEFAULT = 0, // Output image matrix will be created (Mat::create),
                       // i.e. existing memory of output image may be reused.
                       // Two source image, matches and single keypoints will be drawn.
                       // For each keypoint only the center point will be drawn (without
                       // the circle around keypoint with keypoint size and orientation).
          DRAW_OVER_OUTIMG = 1, // Output image matrix will not be created (Mat::create).
                                // Matches will be drawn on existing content of output image.
          NOT_DRAW_SINGLE_POINTS = 2, // Single keypoints will not be drawn.
          DRAW_RICH_KEYPOINTS = 4; // For each keypoint the circle around keypoint with keypoint size and
                                  // orientation will be drawn.
}

// Draw keypoints.
@Namespace("cv") public static native void drawKeypoints( @ByVal Mat image, @StdVector KeyPoint keypoints, @ByVal Mat outImage,
                               @Const @ByRef Scalar color/*=Scalar::all(-1)*/, int flags/*=DrawMatchesFlags::DEFAULT*/ );
@Namespace("cv") public static native void drawKeypoints( @ByVal Mat image, @StdVector KeyPoint keypoints, @ByVal Mat outImage );

// Draws matches of keypints from two images on output image.
@Namespace("cv") public static native void drawMatches( @ByVal Mat img1, @StdVector KeyPoint keypoints1,
                             @ByVal Mat img2, @StdVector KeyPoint keypoints2,
                             @StdVector DMatch matches1to2, @ByVal Mat outImg,
                             @Const @ByRef Scalar matchColor/*=Scalar::all(-1)*/, @Const @ByRef Scalar singlePointColor/*=Scalar::all(-1)*/,
                             @Cast("char*") @StdVector BytePointer matchesMask/*=std::vector<char>()*/, int flags/*=DrawMatchesFlags::DEFAULT*/ );
@Namespace("cv") public static native void drawMatches( @ByVal Mat img1, @StdVector KeyPoint keypoints1,
                             @ByVal Mat img2, @StdVector KeyPoint keypoints2,
                             @StdVector DMatch matches1to2, @ByVal Mat outImg );
@Namespace("cv") public static native void drawMatches( @ByVal Mat img1, @StdVector KeyPoint keypoints1,
                             @ByVal Mat img2, @StdVector KeyPoint keypoints2,
                             @StdVector DMatch matches1to2, @ByVal Mat outImg,
                             @Const @ByRef Scalar matchColor/*=Scalar::all(-1)*/, @Const @ByRef Scalar singlePointColor/*=Scalar::all(-1)*/,
                             @Cast("char*") @StdVector ByteBuffer matchesMask/*=std::vector<char>()*/, int flags/*=DrawMatchesFlags::DEFAULT*/ );
@Namespace("cv") public static native void drawMatches( @ByVal Mat img1, @StdVector KeyPoint keypoints1,
                             @ByVal Mat img2, @StdVector KeyPoint keypoints2,
                             @StdVector DMatch matches1to2, @ByVal Mat outImg,
                             @Const @ByRef Scalar matchColor/*=Scalar::all(-1)*/, @Const @ByRef Scalar singlePointColor/*=Scalar::all(-1)*/,
                             @Cast("char*") @StdVector byte[] matchesMask/*=std::vector<char>()*/, int flags/*=DrawMatchesFlags::DEFAULT*/ );

@Namespace("cv") public static native @Name("drawMatches") void drawMatchesKnn( @ByVal Mat img1, @StdVector KeyPoint keypoints1,
                             @ByVal Mat img2, @StdVector KeyPoint keypoints2,
                             @Const @ByRef DMatchVectorVector matches1to2, @ByVal Mat outImg,
                             @Const @ByRef Scalar matchColor/*=Scalar::all(-1)*/, @Const @ByRef Scalar singlePointColor/*=Scalar::all(-1)*/,
                             @Cast("const std::vector<std::vector<char> >*") @ByRef ByteVectorVector matchesMask/*=std::vector<std::vector<char> >()*/, int flags/*=DrawMatchesFlags::DEFAULT*/ );
@Namespace("cv") public static native @Name("drawMatches") void drawMatchesKnn( @ByVal Mat img1, @StdVector KeyPoint keypoints1,
                             @ByVal Mat img2, @StdVector KeyPoint keypoints2,
                             @Const @ByRef DMatchVectorVector matches1to2, @ByVal Mat outImg );

/****************************************************************************************\
*   Functions to evaluate the feature detectors and [generic] descriptor extractors      *
\****************************************************************************************/

@Namespace("cv") public static native void evaluateFeatureDetector( @Const @ByRef Mat img1, @Const @ByRef Mat img2, @Const @ByRef Mat H1to2,
                                         @StdVector KeyPoint keypoints1, @StdVector KeyPoint keypoints2,
                                         @ByRef FloatPointer repeatability, @ByRef IntPointer correspCount,
                                         @Cast("cv::FeatureDetector*") @Ptr Feature2D fdetector/*=Ptr<FeatureDetector>()*/ );
@Namespace("cv") public static native void evaluateFeatureDetector( @Const @ByRef Mat img1, @Const @ByRef Mat img2, @Const @ByRef Mat H1to2,
                                         @StdVector KeyPoint keypoints1, @StdVector KeyPoint keypoints2,
                                         @ByRef FloatPointer repeatability, @ByRef IntPointer correspCount );
@Namespace("cv") public static native void evaluateFeatureDetector( @Const @ByRef Mat img1, @Const @ByRef Mat img2, @Const @ByRef Mat H1to2,
                                         @StdVector KeyPoint keypoints1, @StdVector KeyPoint keypoints2,
                                         @ByRef FloatBuffer repeatability, @ByRef IntBuffer correspCount,
                                         @Cast("cv::FeatureDetector*") @Ptr Feature2D fdetector/*=Ptr<FeatureDetector>()*/ );
@Namespace("cv") public static native void evaluateFeatureDetector( @Const @ByRef Mat img1, @Const @ByRef Mat img2, @Const @ByRef Mat H1to2,
                                         @StdVector KeyPoint keypoints1, @StdVector KeyPoint keypoints2,
                                         @ByRef FloatBuffer repeatability, @ByRef IntBuffer correspCount );
@Namespace("cv") public static native void evaluateFeatureDetector( @Const @ByRef Mat img1, @Const @ByRef Mat img2, @Const @ByRef Mat H1to2,
                                         @StdVector KeyPoint keypoints1, @StdVector KeyPoint keypoints2,
                                         @ByRef float[] repeatability, @ByRef int[] correspCount,
                                         @Cast("cv::FeatureDetector*") @Ptr Feature2D fdetector/*=Ptr<FeatureDetector>()*/ );
@Namespace("cv") public static native void evaluateFeatureDetector( @Const @ByRef Mat img1, @Const @ByRef Mat img2, @Const @ByRef Mat H1to2,
                                         @StdVector KeyPoint keypoints1, @StdVector KeyPoint keypoints2,
                                         @ByRef float[] repeatability, @ByRef int[] correspCount );

@Namespace("cv") public static native void computeRecallPrecisionCurve( @Const @ByRef DMatchVectorVector matches1to2,
                                             @Cast("const std::vector<std::vector<unsigned char> >*") @ByRef ByteVectorVector correctMatches1to2Mask,
                                             @StdVector Point2f recallPrecisionCurve );

@Namespace("cv") public static native float getRecall( @StdVector Point2f recallPrecisionCurve, float l_precision );
@Namespace("cv") public static native int getNearestPoint( @StdVector Point2f recallPrecisionCurve, float l_precision );

/****************************************************************************************\
*                                     Bag of visual words                                *
\****************************************************************************************/
/*
 * Abstract base class for training of a 'bag of visual words' vocabulary from a set of descriptors
 */
@Namespace("cv") @NoOffset public static class BOWTrainer extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public BOWTrainer() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public BOWTrainer(Pointer p) { super(p); }


    public native void add( @Const @ByRef Mat descriptors );
    public native @Const @ByRef MatVector getDescriptors();
    public native int descriptorsCount();

    public native void clear();

    /*
     * Train visual words vocabulary, that is cluster training descriptors and
     * compute cluster centers.
     * Returns cluster centers.
     *
     * descriptors      Training descriptors computed on images keypoints.
     */
    public native @ByVal Mat cluster();
    public native @ByVal Mat cluster( @Const @ByRef Mat descriptors );
}

/*
 * This is BOWTrainer using cv::kmeans to get vocabulary.
 */
@Namespace("cv") @NoOffset public static class BOWKMeansTrainer extends BOWTrainer {
    static { Loader.load(); }
    /** Empty constructor. */
    public BOWKMeansTrainer() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public BOWKMeansTrainer(Pointer p) { super(p); }

    public BOWKMeansTrainer( int clusterCount, @Const @ByRef TermCriteria termcrit/*=TermCriteria()*/,
                          int attempts/*=3*/, int flags/*=KMEANS_PP_CENTERS*/ ) { allocate(clusterCount, termcrit, attempts, flags); }
    private native void allocate( int clusterCount, @Const @ByRef TermCriteria termcrit/*=TermCriteria()*/,
                          int attempts/*=3*/, int flags/*=KMEANS_PP_CENTERS*/ );
    public BOWKMeansTrainer( int clusterCount ) { allocate(clusterCount); }
    private native void allocate( int clusterCount );

    // Returns trained vocabulary (i.e. cluster centers).
    public native @ByVal Mat cluster();
    public native @ByVal Mat cluster( @Const @ByRef Mat descriptors );
}

/*
 * Class to compute image descriptor using bag of visual words.
 */
@Namespace("cv") @NoOffset public static class BOWImgDescriptorExtractor extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public BOWImgDescriptorExtractor() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public BOWImgDescriptorExtractor(Pointer p) { super(p); }

    public BOWImgDescriptorExtractor( @Cast("cv::DescriptorExtractor*") @Ptr Feature2D dextractor,
                                   @Ptr DescriptorMatcher dmatcher ) { allocate(dextractor, dmatcher); }
    private native void allocate( @Cast("cv::DescriptorExtractor*") @Ptr Feature2D dextractor,
                                   @Ptr DescriptorMatcher dmatcher );
    public BOWImgDescriptorExtractor( @Ptr DescriptorMatcher dmatcher ) { allocate(dmatcher); }
    private native void allocate( @Ptr DescriptorMatcher dmatcher );

    public native void setVocabulary( @Const @ByRef Mat vocabulary );
    public native @Const @ByRef Mat getVocabulary();
    public native void compute( @ByVal Mat image, @StdVector KeyPoint keypoints, @ByVal Mat imgDescriptor,
                      IntVectorVector pointIdxsOfClusters/*=0*/, Mat descriptors/*=0*/ );
    public native void compute( @ByVal Mat image, @StdVector KeyPoint keypoints, @ByVal Mat imgDescriptor );
    public native void compute( @ByVal Mat keypointDescriptors, @ByVal Mat imgDescriptor,
                      IntVectorVector pointIdxsOfClusters/*=0*/ );
    public native void compute( @ByVal Mat keypointDescriptors, @ByVal Mat imgDescriptor );
    // compute() is not constant because DescriptorMatcher::match is not constant

    public native int descriptorSize();
    public native int descriptorType();
}

 /* namespace cv */

// #endif


}
