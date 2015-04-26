// Targeted by JavaCPP version 0.11

package org.bytedeco.javacpp;

import java.nio.*;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

public class opencv_core extends org.bytedeco.javacpp.helper.opencv_core {
    static { Loader.load(); }

@Name("std::vector<std::vector<char> >") public static class ByteVectorVector extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public ByteVectorVector(Pointer p) { super(p); }
    public ByteVectorVector(byte[] ... array) { this(array.length); put(array); }
    public ByteVectorVector()       { allocate();  }
    public ByteVectorVector(long n) { allocate(n); }
    private native void allocate();
    private native void allocate(@Cast("size_t") long n);
    public native @Name("operator=") @ByRef ByteVectorVector put(@ByRef ByteVectorVector x);

    public native long size();
    public native void resize(@Cast("size_t") long n);
    public native @Index long size(@Cast("size_t") long i);
    public native @Index void resize(@Cast("size_t") long i, @Cast("size_t") long n);

    @Index public native byte get(@Cast("size_t") long i, @Cast("size_t") long j);
    public native ByteVectorVector put(@Cast("size_t") long i, @Cast("size_t") long j, byte value);

    public ByteVectorVector put(byte[] ... array) {
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

@Name("std::vector<std::vector<int> >") public static class IntVectorVector extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public IntVectorVector(Pointer p) { super(p); }
    public IntVectorVector(int[] ... array) { this(array.length); put(array); }
    public IntVectorVector()       { allocate();  }
    public IntVectorVector(long n) { allocate(n); }
    private native void allocate();
    private native void allocate(@Cast("size_t") long n);
    public native @Name("operator=") @ByRef IntVectorVector put(@ByRef IntVectorVector x);

    public native long size();
    public native void resize(@Cast("size_t") long n);
    public native @Index long size(@Cast("size_t") long i);
    public native @Index void resize(@Cast("size_t") long i, @Cast("size_t") long n);

    @Index public native int get(@Cast("size_t") long i, @Cast("size_t") long j);
    public native IntVectorVector put(@Cast("size_t") long i, @Cast("size_t") long j, int value);

    public IntVectorVector put(int[] ... array) {
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

@Name("std::vector<cv::String>") public static class StringVector extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public StringVector(Pointer p) { super(p); }
    public StringVector(BytePointer ... array) { this(array.length); put(array); }
    public StringVector()       { allocate();  }
    public StringVector(long n) { allocate(n); }
    private native void allocate();
    private native void allocate(@Cast("size_t") long n);
    public native @Name("operator=") @ByRef StringVector put(@ByRef StringVector x);

    public native long size();
    public native void resize(@Cast("size_t") long n);

    @Index public native @Str BytePointer get(@Cast("size_t") long i);
    public native StringVector put(@Cast("size_t") long i, BytePointer value);

    public StringVector put(BytePointer ... array) {
        if (size() != array.length) { resize(array.length); }
        for (int i = 0; i < array.length; i++) {
            put(i, array[i]);
        }
        return this;
    }
}

@Name("std::vector<std::vector<cv::Point_<int> > >") public static class PointVectorVector extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public PointVectorVector(Pointer p) { super(p); }
    public PointVectorVector(Point[] ... array) { this(array.length); put(array); }
    public PointVectorVector()       { allocate();  }
    public PointVectorVector(long n) { allocate(n); }
    private native void allocate();
    private native void allocate(@Cast("size_t") long n);
    public native @Name("operator=") @ByRef PointVectorVector put(@ByRef PointVectorVector x);

    public native long size();
    public native void resize(@Cast("size_t") long n);
    public native @Index long size(@Cast("size_t") long i);
    public native @Index void resize(@Cast("size_t") long i, @Cast("size_t") long n);

    @Index public native @ByRef Point get(@Cast("size_t") long i, @Cast("size_t") long j);
    public native PointVectorVector put(@Cast("size_t") long i, @Cast("size_t") long j, Point value);

    public PointVectorVector put(Point[] ... array) {
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

@Name("std::vector<std::vector<cv::Point_<float> > >") public static class Point2fVectorVector extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Point2fVectorVector(Pointer p) { super(p); }
    public Point2fVectorVector(Point2f[] ... array) { this(array.length); put(array); }
    public Point2fVectorVector()       { allocate();  }
    public Point2fVectorVector(long n) { allocate(n); }
    private native void allocate();
    private native void allocate(@Cast("size_t") long n);
    public native @Name("operator=") @ByRef Point2fVectorVector put(@ByRef Point2fVectorVector x);

    public native long size();
    public native void resize(@Cast("size_t") long n);
    public native @Index long size(@Cast("size_t") long i);
    public native @Index void resize(@Cast("size_t") long i, @Cast("size_t") long n);

    @Index public native @ByRef Point2f get(@Cast("size_t") long i, @Cast("size_t") long j);
    public native Point2fVectorVector put(@Cast("size_t") long i, @Cast("size_t") long j, Point2f value);

    public Point2fVectorVector put(Point2f[] ... array) {
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

@Name("std::vector<std::vector<cv::Point_<double> > >") public static class Point2dVectorVector extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Point2dVectorVector(Pointer p) { super(p); }
    public Point2dVectorVector(Point2d[] ... array) { this(array.length); put(array); }
    public Point2dVectorVector()       { allocate();  }
    public Point2dVectorVector(long n) { allocate(n); }
    private native void allocate();
    private native void allocate(@Cast("size_t") long n);
    public native @Name("operator=") @ByRef Point2dVectorVector put(@ByRef Point2dVectorVector x);

    public native long size();
    public native void resize(@Cast("size_t") long n);
    public native @Index long size(@Cast("size_t") long i);
    public native @Index void resize(@Cast("size_t") long i, @Cast("size_t") long n);

    @Index public native @ByRef Point2d get(@Cast("size_t") long i, @Cast("size_t") long j);
    public native Point2dVectorVector put(@Cast("size_t") long i, @Cast("size_t") long j, Point2d value);

    public Point2dVectorVector put(Point2d[] ... array) {
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

@Name("std::vector<std::vector<cv::Rect_<int> > >") public static class RectVectorVector extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public RectVectorVector(Pointer p) { super(p); }
    public RectVectorVector(Rect[] ... array) { this(array.length); put(array); }
    public RectVectorVector()       { allocate();  }
    public RectVectorVector(long n) { allocate(n); }
    private native void allocate();
    private native void allocate(@Cast("size_t") long n);
    public native @Name("operator=") @ByRef RectVectorVector put(@ByRef RectVectorVector x);

    public native long size();
    public native void resize(@Cast("size_t") long n);
    public native @Index long size(@Cast("size_t") long i);
    public native @Index void resize(@Cast("size_t") long i, @Cast("size_t") long n);

    @Index public native @ByRef Rect get(@Cast("size_t") long i, @Cast("size_t") long j);
    public native RectVectorVector put(@Cast("size_t") long i, @Cast("size_t") long j, Rect value);

    public RectVectorVector put(Rect[] ... array) {
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

@Name("std::vector<cv::Mat>") public static class MatVector extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public MatVector(Pointer p) { super(p); }
    public MatVector(Mat ... array) { this(array.length); put(array); }
    public MatVector()       { allocate();  }
    public MatVector(long n) { allocate(n); }
    private native void allocate();
    private native void allocate(@Cast("size_t") long n);
    public native @Name("operator=") @ByRef MatVector put(@ByRef MatVector x);

    public native long size();
    public native void resize(@Cast("size_t") long n);

    @Index public native @ByRef Mat get(@Cast("size_t") long i);
    public native MatVector put(@Cast("size_t") long i, Mat value);

    public MatVector put(Mat ... array) {
        if (size() != array.length) { resize(array.length); }
        for (int i = 0; i < array.length; i++) {
            put(i, array[i]);
        }
        return this;
    }
}

@Name("std::vector<std::pair<int,int> >") public static class IntIntPairVector extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public IntIntPairVector(Pointer p) { super(p); }
    public IntIntPairVector()       { allocate();  }
    public IntIntPairVector(long n) { allocate(n); }
    private native void allocate();
    private native void allocate(@Cast("size_t") long n);
    public native @Name("operator=") @ByRef IntIntPairVector put(@ByRef IntIntPairVector x);

    public native long size();
    public native void resize(@Cast("size_t") long n);

    @Index public native int first(@Cast("size_t") long i); public native IntIntPairVector first(@Cast("size_t") long i, int first);
    @Index public native int second(@Cast("size_t") long i);  public native IntIntPairVector second(@Cast("size_t") long i, int second);
}

@Name("std::vector<std::pair<cv::Mat,unsigned char> >") public static class MatBytePairVector extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public MatBytePairVector(Pointer p) { super(p); }
    public MatBytePairVector()       { allocate();  }
    public MatBytePairVector(long n) { allocate(n); }
    private native void allocate();
    private native void allocate(@Cast("size_t") long n);
    public native @Name("operator=") @ByRef MatBytePairVector put(@ByRef MatBytePairVector x);

    public native long size();
    public native void resize(@Cast("size_t") long n);

    @Index public native @ByRef Mat first(@Cast("size_t") long i); public native MatBytePairVector first(@Cast("size_t") long i, Mat first);
    @Index public native byte second(@Cast("size_t") long i);  public native MatBytePairVector second(@Cast("size_t") long i, byte second);
}

@Name("std::vector<std::pair<cv::UMat,unsigned char> >") public static class UMatBytePairVector extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public UMatBytePairVector(Pointer p) { super(p); }
    public UMatBytePairVector()       { allocate();  }
    public UMatBytePairVector(long n) { allocate(n); }
    private native void allocate();
    private native void allocate(@Cast("size_t") long n);
    public native @Name("operator=") @ByRef UMatBytePairVector put(@ByRef UMatBytePairVector x);

    public native long size();
    public native void resize(@Cast("size_t") long n);

    @Index public native @ByRef UMat first(@Cast("size_t") long i); public native UMatBytePairVector first(@Cast("size_t") long i, UMat first);
    @Index public native byte second(@Cast("size_t") long i);  public native UMatBytePairVector second(@Cast("size_t") long i, byte second);
}

@NoOffset @Name("std::pair<int,int>") public static class IntIntPair extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public IntIntPair(Pointer p) { super(p); }
    public IntIntPair()       { allocate();  }
    private native void allocate();
    public native @Name("operator=") @ByRef IntIntPair put(@ByRef IntIntPair x);


    @MemberGetter public native int first(); public native IntIntPair first(int first);
    @MemberGetter public native int second();  public native IntIntPair second(int second);
}

// Parsed from <opencv2/core/cvdef.h>

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

// #ifndef __OPENCV_CORE_CVDEF_H__
// #define __OPENCV_CORE_CVDEF_H__

// #if !defined _CRT_SECURE_NO_DEPRECATE && defined _MSC_VER && _MSC_VER > 1300
// #  define _CRT_SECURE_NO_DEPRECATE /* to avoid multiple Visual Studio warnings */
// #endif

// undef problematic defines sometimes defined by system headers (windows.h in particular)
// #undef small
// #undef min
// #undef max
// #undef abs
// #undef Complex

// #if defined __ICL
// #elif defined __ICC
// #elif defined __ECL
// #elif defined __ECC
// #elif defined __INTEL_COMPILER
// #endif

// #if defined CV_ICC && !defined CV_ENABLE_UNROLLED
// #  define CV_ENABLE_UNROLLED 0
// #else
// #  define CV_ENABLE_UNROLLED 1
// #endif

// #if (defined WIN32 || defined _WIN32 || defined WINCE || defined __CYGWIN__) && defined CVAPI_EXPORTS
// #  define CV_EXPORTS __declspec(dllexport)
// #elif defined __GNUC__ && __GNUC__ >= 4
// #  define CV_EXPORTS __attribute__ ((visibility ("default")))
// #else
// #  define CV_EXPORTS
// #endif

// #ifndef CV_INLINE
// #  if defined __cplusplus
// #    define CV_INLINE static inline
// #  elif defined _MSC_VER
// #    define CV_INLINE __inline
// #  else
// #    define CV_INLINE static
// #  endif
// #endif

// #ifndef CV_EXTERN_C
// #  ifdef __cplusplus
// #    define CV_EXTERN_C extern "C"
// #  else
// #  endif
// #endif

/* CPU features and intrinsics support */
public static final int CV_CPU_NONE =    0;
public static final int CV_CPU_MMX =     1;
public static final int CV_CPU_SSE =     2;
public static final int CV_CPU_SSE2 =    3;
public static final int CV_CPU_SSE3 =    4;
public static final int CV_CPU_SSSE3 =   5;
public static final int CV_CPU_SSE4_1 =  6;
public static final int CV_CPU_SSE4_2 =  7;
public static final int CV_CPU_POPCNT =  8;
public static final int CV_CPU_AVX =    10;
public static final int CV_CPU_NEON =   11;
// when adding to this list remember to update the enum in core/utility.cpp
public static final int CV_HARDWARE_MAX_FEATURE = 255;

// do not include SSE/AVX/NEON headers for NVCC compiler
// #ifndef __CUDACC__

// #if defined __SSE2__ || defined _M_X64  || (defined _M_IX86_FP && _M_IX86_FP >= 2)
// #  include <emmintrin.h>
public static final int CV_SSE = 1;
public static final int CV_SSE2 = 1;
// #  if defined __SSE3__ || (defined _MSC_VER && _MSC_VER >= 1500)
// #    include <pmmintrin.h>
public static final int CV_SSE3 = 1;
// #  endif
// #  if defined __SSSE3__  || (defined _MSC_VER && _MSC_VER >= 1500)
// #    include <tmmintrin.h>
public static final int CV_SSSE3 = 1;
// #  endif
// #  if defined __SSE4_1__ || (defined _MSC_VER && _MSC_VER >= 1500)
// #    include <smmintrin.h>
public static final int CV_SSE4_1 = 1;
// #  endif
// #  if defined __SSE4_2__ || (defined _MSC_VER && _MSC_VER >= 1500)
// #    include <nmmintrin.h>
public static final int CV_SSE4_2 = 1;
// #  endif
// #  if defined __AVX__ || (defined _MSC_FULL_VER && _MSC_FULL_VER >= 160040219)
// MS Visual Studio 2010 (2012?) has no macro pre-defined to identify the use of /arch:AVX
// See: http://connect.microsoft.com/VisualStudio/feedback/details/605858/arch-avx-should-define-a-predefined-macro-in-x64-and-set-a-unique-value-for-m-ix86-fp-in-win32
// #    include <immintrin.h>
public static final int CV_AVX = 1;
// #    if defined(_XCR_XFEATURE_ENABLED_MASK)
// #      define __xgetbv() _xgetbv(_XCR_XFEATURE_ENABLED_MASK)
// #    else
// #      define __xgetbv() 0
// #    endif
// #  endif
// #endif

// #if (defined WIN32 || defined _WIN32) && defined(_M_ARM)
// # include <Intrin.h>
// # include "arm_neon.h"
// # define CV_NEON 1
// # define CPU_HAS_NEON_FEATURE (true)
// #elif defined(__ARM_NEON__)
// #  include <arm_neon.h>
// #  define CV_NEON 1
// #endif

// #endif // __CUDACC__

// #ifndef CV_SSE
// #endif
// #ifndef CV_SSE2
// #endif
// #ifndef CV_SSE3
// #endif
// #ifndef CV_SSSE3
// #endif
// #ifndef CV_SSE4_1
// #endif
// #ifndef CV_SSE4_2
// #endif
// #ifndef CV_AVX
// #endif
// #ifndef CV_NEON
// #  define CV_NEON 0
// #endif

/* primitive types */
/*
  schar  - signed 1 byte integer
  uchar  - unsigned 1 byte integer
  short  - signed 2 byte integer
  ushort - unsigned 2 byte integer
  int    - signed 4 byte integer
  uint   - unsigned 4 byte integer
  int64  - signed 8 byte integer
  uint64 - unsigned 8 byte integer
*/

// #if !defined _MSC_VER && !defined __BORLANDC__
// #  if defined __cplusplus && __cplusplus >= 201103L
// #    include <cstdint>
// #  else
// #    include <stdint.h>
// #  endif
// #else
// #endif

// #ifndef __IPL_H__
// #endif

// #if defined _MSC_VER || defined __BORLANDC__
// #  define CV_BIG_INT(n)   n##I64
// #  define CV_BIG_UINT(n)  n##UI64
// #else
// #  define CV_BIG_INT(n)   n##LL
// #  define CV_BIG_UINT(n)  n##ULL
// #endif

/* special informative macros for wrapper generators */
// #define CV_EXPORTS_W CV_EXPORTS
// #define CV_EXPORTS_W_SIMPLE CV_EXPORTS
// #define CV_EXPORTS_AS(synonym) CV_EXPORTS
// #define CV_EXPORTS_W_MAP CV_EXPORTS
// #define CV_IN_OUT
// #define CV_OUT
// #define CV_PROP
// #define CV_PROP_RW
// #define CV_WRAP
// #define CV_WRAP_AS(synonym)

/* fundamental constants */
public static final double CV_PI =   3.1415926535897932384626433832795;
public static final double CV_2PI = 6.283185307179586476925286766559;
public static final double CV_LOG2 = 0.69314718055994530941723212145818;

/****************************************************************************************\
*                                  Matrix type (Mat)                                     *
\****************************************************************************************/

public static final int CV_CN_MAX =     512;
public static final int CV_CN_SHIFT =   3;
public static final int CV_DEPTH_MAX =  (1 << CV_CN_SHIFT);

public static final int CV_8U =   0;
public static final int CV_8S =   1;
public static final int CV_16U =  2;
public static final int CV_16S =  3;
public static final int CV_32S =  4;
public static final int CV_32F =  5;
public static final int CV_64F =  6;
public static final int CV_USRTYPE1 = 7;

public static final int CV_MAT_DEPTH_MASK =       (CV_DEPTH_MAX - 1);
public static native int CV_MAT_DEPTH(int flags);

public static native int CV_MAKETYPE(int depth, int cn);
public static native int CV_MAKE_TYPE(int arg1, int arg2);

public static final int CV_8UC1 = CV_MAKETYPE(CV_8U,1);
public static final int CV_8UC2 = CV_MAKETYPE(CV_8U,2);
public static final int CV_8UC3 = CV_MAKETYPE(CV_8U,3);
public static final int CV_8UC4 = CV_MAKETYPE(CV_8U,4);
public static native int CV_8UC(int n);

public static final int CV_8SC1 = CV_MAKETYPE(CV_8S,1);
public static final int CV_8SC2 = CV_MAKETYPE(CV_8S,2);
public static final int CV_8SC3 = CV_MAKETYPE(CV_8S,3);
public static final int CV_8SC4 = CV_MAKETYPE(CV_8S,4);
public static native int CV_8SC(int n);

public static final int CV_16UC1 = CV_MAKETYPE(CV_16U,1);
public static final int CV_16UC2 = CV_MAKETYPE(CV_16U,2);
public static final int CV_16UC3 = CV_MAKETYPE(CV_16U,3);
public static final int CV_16UC4 = CV_MAKETYPE(CV_16U,4);
public static native int CV_16UC(int n);

public static final int CV_16SC1 = CV_MAKETYPE(CV_16S,1);
public static final int CV_16SC2 = CV_MAKETYPE(CV_16S,2);
public static final int CV_16SC3 = CV_MAKETYPE(CV_16S,3);
public static final int CV_16SC4 = CV_MAKETYPE(CV_16S,4);
public static native int CV_16SC(int n);

public static final int CV_32SC1 = CV_MAKETYPE(CV_32S,1);
public static final int CV_32SC2 = CV_MAKETYPE(CV_32S,2);
public static final int CV_32SC3 = CV_MAKETYPE(CV_32S,3);
public static final int CV_32SC4 = CV_MAKETYPE(CV_32S,4);
public static native int CV_32SC(int n);

public static final int CV_32FC1 = CV_MAKETYPE(CV_32F,1);
public static final int CV_32FC2 = CV_MAKETYPE(CV_32F,2);
public static final int CV_32FC3 = CV_MAKETYPE(CV_32F,3);
public static final int CV_32FC4 = CV_MAKETYPE(CV_32F,4);
public static native int CV_32FC(int n);

public static final int CV_64FC1 = CV_MAKETYPE(CV_64F,1);
public static final int CV_64FC2 = CV_MAKETYPE(CV_64F,2);
public static final int CV_64FC3 = CV_MAKETYPE(CV_64F,3);
public static final int CV_64FC4 = CV_MAKETYPE(CV_64F,4);
public static native int CV_64FC(int n);

public static final int CV_MAT_CN_MASK =          ((CV_CN_MAX - 1) << CV_CN_SHIFT);
public static native int CV_MAT_CN(int flags);
public static final int CV_MAT_TYPE_MASK =        (CV_DEPTH_MAX*CV_CN_MAX - 1);
public static native int CV_MAT_TYPE(int flags);
public static final int CV_MAT_CONT_FLAG_SHIFT =  14;
public static final int CV_MAT_CONT_FLAG =        (1 << CV_MAT_CONT_FLAG_SHIFT);
public static native int CV_IS_MAT_CONT(int flags);
public static native int CV_IS_CONT_MAT(int arg1);
public static final int CV_SUBMAT_FLAG_SHIFT =    15;
public static final int CV_SUBMAT_FLAG =          (1 << CV_SUBMAT_FLAG_SHIFT);
// #define CV_IS_SUBMAT(flags)     ((flags) & CV_MAT_SUBMAT_FLAG)

/* Size of each channel item,
   0x124489 = 1000 0100 0100 0010 0010 0001 0001 ~ array of sizeof(arr_type_elem) */
// #define CV_ELEM_SIZE1(type)
//     ((((sizeof(size_t)<<28)|0x8442211) >> CV_MAT_DEPTH(type)*4) & 15)

/* 0x3a50 = 11 10 10 01 01 00 00 ~ array of log2(sizeof(arr_type_elem)) */
// #define CV_ELEM_SIZE(type)
//     (CV_MAT_CN(type) << ((((sizeof(size_t)/4+1)*16384|0x3a50) >> CV_MAT_DEPTH(type)*2) & 3))


/****************************************************************************************\
*                                      fast math                                         *
\****************************************************************************************/

// #if defined __BORLANDC__
// #  include <fastmath.h>
// #elif defined __cplusplus
// #  include <cmath>
// #else
// #  include <math.h>
// #endif

// #ifndef MIN
// #  define MIN(a,b)  ((a) > (b) ? (b) : (a))
// #endif

// #ifndef MAX
// #  define MAX(a,b)  ((a) < (b) ? (b) : (a))
// #endif

// #ifdef HAVE_TEGRA_OPTIMIZATION
// #  include "tegra_round.hpp"
// #endif

public static native int cvRound( double value );

public static native int cvFloor( double value );

public static native int cvCeil( double value );

public static native int cvIsNaN( double value );

public static native int cvIsInf( double value );

/****************************************************************************************\
*          exchange-add operation for atomic operations on reference counters            *
\****************************************************************************************/

// #if defined __INTEL_COMPILER && !(defined WIN32 || defined _WIN32)
// #elif defined __GNUC__
// #elif defined _MSC_VER && !defined RC_INVOKED
// #  include <intrin.h>
// #  define CV_XADD(addr, delta) (int)_InterlockedExchangeAdd((long volatile*)addr, delta)
// #else
   
// #endif


/****************************************************************************************\
*                                  CV_NORETURN attribute                                 *
\****************************************************************************************/

// #ifndef CV_NORETURN
// #  if defined(__GNUC__)
// #  elif defined(_MSC_VER) && (_MSC_VER >= 1300)
// #    define CV_NORETURN __declspec(noreturn)
// #  else
// #    define CV_NORETURN /* nothing by default */
// #  endif
// #endif

// #endif // __OPENCV_CORE_CVDEF_H__


// Parsed from <opencv2/core/version.hpp>

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright( C) 2000, Intel Corporation, all rights reserved.
// Copyright (C) 2011-2013, NVIDIA Corporation, all rights reserved.
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
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
//(including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort(including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/*
  definition of the current version of OpenCV
  Usefull to test in user programs
*/

// #ifndef __OPENCV_VERSION_HPP__
// #define __OPENCV_VERSION_HPP__

public static final int CV_VERSION_MAJOR =    3;
public static final int CV_VERSION_MINOR =    0;
public static final int CV_VERSION_REVISION = 0;
public static final String CV_VERSION_STATUS =   "-beta";

// #define CVAUX_STR_EXP(__A)  #__A
// #define CVAUX_STR(__A)      CVAUX_STR_EXP(__A)

// #define CVAUX_STRW_EXP(__A)  L#__A
// #define CVAUX_STRW(__A)      CVAUX_STRW_EXP(__A)

public static native @MemberGetter String CV_VERSION();
public static final String CV_VERSION = CV_VERSION();

/* old  style version constants*/
public static final int CV_MAJOR_VERSION =    CV_VERSION_MAJOR;
public static final int CV_MINOR_VERSION =    CV_VERSION_MINOR;
public static final int CV_SUBMINOR_VERSION = CV_VERSION_REVISION;

// #endif


// Parsed from <opencv2/core/base.hpp>

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

// #ifndef __OPENCV_CORE_BASE_HPP__
// #define __OPENCV_CORE_BASE_HPP__

// #ifndef __cplusplus
// #endif

// #include <climits>

// #include "opencv2/core/cvdef.h"
// #include "opencv2/core/cvstd.hpp"

// error codes
/** enum cv::Error:: */
public static final int
    StsOk= 0,  /* everithing is ok                */
    StsBackTrace= -1,  /* pseudo error for back trace     */
    StsError= -2,  /* unknown /unspecified error      */
    StsInternal= -3,  /* internal error (bad state)      */
    StsNoMem= -4,  /* insufficient memory             */
    StsBadArg= -5,  /* function arg/param is bad       */
    StsBadFunc= -6,  /* unsupported function            */
    StsNoConv= -7,  /* iter. didn't converge           */
    StsAutoTrace= -8,  /* tracing                         */
    HeaderIsNull= -9,  /* image header is NULL            */
    BadImageSize= -10,  /* image size is invalid           */
    BadOffset= -11,  /* offset is invalid               */
    BadDataPtr= -12,  /**/
    BadStep= -13,  /**/
    BadModelOrChSeq= -14,  /**/
    BadNumChannels= -15,  /**/
    BadNumChannel1U= -16,  /**/
    BadDepth= -17,  /**/
    BadAlphaChannel= -18,  /**/
    BadOrder= -19,  /**/
    BadOrigin= -20,  /**/
    BadAlign= -21,  /**/
    BadCallBack= -22,  /**/
    BadTileSize= -23,  /**/
    BadCOI= -24,  /**/
    BadROISize= -25,  /**/
    MaskIsTiled= -26,  /**/
    StsNullPtr= -27,  /* null pointer */
    StsVecLengthErr= -28,  /* incorrect vector length */
    StsFilterStructContentErr= -29,  /* incorr. filter structure content */
    StsKernelStructContentErr= -30,  /* incorr. transform kernel content */
    StsFilterOffsetErr= -31,  /* incorrect filter ofset value */
    StsBadSize= -201, /* the input/output structure size is incorrect  */
    StsDivByZero= -202, /* division by zero */
    StsInplaceNotSupported= -203, /* in-place operation is not supported */
    StsObjectNotFound= -204, /* request can't be completed */
    StsUnmatchedFormats= -205, /* formats of input/output arrays differ */
    StsBadFlag= -206, /* flag is wrong or not supported */
    StsBadPoint= -207, /* bad CvPoint */
    StsBadMask= -208, /* bad format of mask (neither 8uC1 nor 8sC1)*/
    StsUnmatchedSizes= -209, /* sizes of input/output structures do not match */
    StsUnsupportedFormat= -210, /* the data format/type is not supported by the function*/
    StsOutOfRange= -211, /* some of parameters are out of range */
    StsParseError= -212, /* invalid syntax/structure of the parsed file */
    StsNotImplemented= -213, /* the requested function/feature is not implemented */
    StsBadMemBlock= -214, /* an allocated block has been corrupted */
    StsAssert= -215, /* assertion failed */
    GpuNotSupported= -216,
    GpuApiCallError= -217,
    OpenGlNotSupported= -218,
    OpenGlApiCallError= -219,
    OpenCLApiCallError= -220,
    OpenCLDoubleNotSupported= -221,
    OpenCLInitError= -222,
    OpenCLNoAMDBlasFft= -223;
 //Error

// matrix decomposition types
/** enum cv:: */
public static final int DECOMP_LU       = 0,
       DECOMP_SVD      = 1,
       DECOMP_EIG      = 2,
       DECOMP_CHOLESKY = 3,
       DECOMP_QR       = 4,
       DECOMP_NORMAL   = 16;

// norm types
/** enum cv:: */
public static final int NORM_INF       = 1,
       NORM_L1        = 2,
       NORM_L2        = 4,
       NORM_L2SQR     = 5,
       NORM_HAMMING   = 6,
       NORM_HAMMING2  = 7,
       NORM_TYPE_MASK = 7,
       NORM_RELATIVE  = 8,
       NORM_MINMAX    = 32;

// comparison types
/** enum cv:: */
public static final int CMP_EQ = 0,
       CMP_GT = 1,
       CMP_GE = 2,
       CMP_LT = 3,
       CMP_LE = 4,
       CMP_NE = 5;

/** enum cv:: */
public static final int GEMM_1_T = 1,
       GEMM_2_T = 2,
       GEMM_3_T = 4;

/** enum cv:: */
public static final int DFT_INVERSE        = 1,
       DFT_SCALE          = 2,
       DFT_ROWS           = 4,
       DFT_COMPLEX_OUTPUT = 16,
       DFT_REAL_OUTPUT    = 32,
       DCT_INVERSE        =  DFT_INVERSE,
       DCT_ROWS           =  DFT_ROWS;

/** Various border types, image boundaries are denoted with '|' */
/** enum cv:: */
public static final int
       BORDER_CONSTANT    = 0, // iiiiii|abcdefgh|iiiiiii  with some specified 'i'
       BORDER_REPLICATE   = 1, // aaaaaa|abcdefgh|hhhhhhh
       BORDER_REFLECT     = 2, // fedcba|abcdefgh|hgfedcb
       BORDER_WRAP        = 3, // cdefgh|abcdefgh|abcdefg
       BORDER_REFLECT_101 = 4, // gfedcb|abcdefgh|gfedcba
       BORDER_TRANSPARENT = 5, // uvwxyz|absdefgh|ijklmno

       BORDER_REFLECT101  =  BORDER_REFLECT_101,
       BORDER_DEFAULT     =  BORDER_REFLECT_101,
       BORDER_ISOLATED    = 16; // do not look outside of ROI



//////////////// static assert /////////////////

// #define CVAUX_CONCAT_EXP(a, b) a##b
// #define CVAUX_CONCAT(a, b) CVAUX_CONCAT_EXP(a,b)

// #if defined(__clang__)
// #elif defined(__GNUC__)
// #elif defined(_MSC_VER)
// #endif
// #ifndef CV_StaticAssert
// #endif

/** Suppress warning "-Wdeprecated-declarations" / C4996 */

// #if defined(_MSC_VER)
// #elif defined(__GNUC__)
// #else
//     #define CV_DO_PRAGMA(x)
// #endif

// #ifdef _MSC_VER
// #define CV_SUPPRESS_DEPRECATED_START
//     CV_DO_PRAGMA(warning(push))
//     CV_DO_PRAGMA(warning(disable: 4996))
// #define CV_SUPPRESS_DEPRECATED_END CV_DO_PRAGMA(warning(pop))
// #elif defined (__clang__) || ((__GNUC__)  && ((__GNUC__ > 4) || ((__GNUC__ == 4) && (__GNUC_MINOR__ > 5))))
// #define CV_SUPPRESS_DEPRECATED_START
//     CV_DO_PRAGMA(GCC diagnostic push)
//     CV_DO_PRAGMA(GCC diagnostic ignored "-Wdeprecated-declarations")
// #define CV_SUPPRESS_DEPRECATED_END CV_DO_PRAGMA(GCC diagnostic pop)
// #else
// #define CV_SUPPRESS_DEPRECATED_START
// #define CV_SUPPRESS_DEPRECATED_END
// #endif

/** @brief Signals an error and raises the exception.
  By default the function prints information about the error to stderr,
  then it either stops if setBreakOnError() had been called before or raises the exception.
  It is possible to alternate error processing by using redirectError().
  @param _code - error code @see CVStatus
  @param _err - error description
  @param _func - function name. Available only when the compiler supports getting it
  @param _file - source file name where the error has occured
  @param _line - line number in the source file where the error has occured
 */
@Namespace("cv") public static native void error(int _code, @Str BytePointer _err, @Cast("const char*") BytePointer _func, @Cast("const char*") BytePointer _file, int _line);
@Namespace("cv") public static native void error(int _code, @Str String _err, String _func, String _file, int _line);

// #ifdef __GNUC__
// # if defined __clang__ || defined __APPLE__
// #   pragma GCC diagnostic push
// #   pragma GCC diagnostic ignored "-Winvalid-noreturn"
// # endif
// #endif
@Namespace("cv") public static native void errorNoReturn(int _code, @Str BytePointer _err, @Cast("const char*") BytePointer _func, @Cast("const char*") BytePointer _file, int _line);
@Namespace("cv") public static native void errorNoReturn(int _code, @Str String _err, String _func, String _file, int _line);
// #ifdef __GNUC__
// # if defined __clang__ || defined __APPLE__
// #   pragma GCC diagnostic pop
// # endif
// #endif


// #if defined __GNUC__
// #elif defined _MSC_VER
// #define CV_Func __FUNCTION__
// #else
// #define CV_Func ""
// #endif

// #define CV_Error( code, msg ) cv::error( code, msg, CV_Func, __FILE__, __LINE__ )
// #define CV_Error_( code, args ) cv::error( code, cv::format args, CV_Func, __FILE__, __LINE__ )
// #define CV_Assert( expr ) if(!!(expr)) ; else cv::error( cv::Error::StsAssert, #expr, CV_Func, __FILE__, __LINE__ )

// #define CV_ErrorNoReturn( code, msg ) cv::errorNoReturn( code, msg, CV_Func, __FILE__, __LINE__ )
// #define CV_ErrorNoReturn_( code, args ) cv::errorNoReturn( code, cv::format args, CV_Func, __FILE__, __LINE__ )

// #ifdef _DEBUG
// #  define CV_DbgAssert(expr) CV_Assert(expr)
// #else
// #  define CV_DbgAssert(expr)
// #endif



/////////////// saturate_cast (used in image & signal processing) ///////////////////

@Namespace("cv") public static native @Cast("uchar") @Name("saturate_cast<uchar>") byte saturateCastUchar(@Cast("uchar") byte v);

@Namespace("cv") public static native @Cast("schar") @Name("saturate_cast<schar>") byte saturateCastSchar(@Cast("uchar") byte v);

@Namespace("cv") public static native @Cast("ushort") @Name("saturate_cast<ushort>") short saturateCastUshort(@Cast("uchar") byte v);

@Namespace("cv") public static native @Name("saturate_cast<short>") short saturateCastShort(@Cast("uchar") byte v);

@Namespace("cv") public static native @Name("saturate_cast<int>") int saturateCastInt(@Cast("uchar") byte v);

@Namespace("cv") public static native @Cast("unsigned") @Name("saturate_cast<unsigned>") int saturateCastUnsigned(@Cast("uchar") byte v);
@Namespace("cv") public static native @Cast("uchar") @Name("saturate_cast<uchar>") byte saturateCastUchar(@Cast("ushort") short v);
@Namespace("cv") public static native @Cast("schar") @Name("saturate_cast<schar>") byte saturateCastSchar(@Cast("ushort") short v);
@Namespace("cv") public static native @Cast("ushort") @Name("saturate_cast<ushort>") short saturateCastUshort(@Cast("ushort") short v);
@Namespace("cv") public static native @Name("saturate_cast<short>") short saturateCastShort(@Cast("ushort") short v);
@Namespace("cv") public static native @Name("saturate_cast<int>") int saturateCastInt(@Cast("ushort") short v);
@Namespace("cv") public static native @Cast("unsigned") @Name("saturate_cast<unsigned>") int saturateCastUnsigned(@Cast("ushort") short v);
@Namespace("cv") public static native @Cast("uchar") @Name("saturate_cast<uchar>") byte saturateCastUchar(@Cast("unsigned") int v);
@Namespace("cv") public static native @Cast("schar") @Name("saturate_cast<schar>") byte saturateCastSchar(@Cast("unsigned") int v);
@Namespace("cv") public static native @Cast("ushort") @Name("saturate_cast<ushort>") short saturateCastUshort(@Cast("unsigned") int v);
@Namespace("cv") public static native @Name("saturate_cast<short>") short saturateCastShort(@Cast("unsigned") int v);
@Namespace("cv") public static native @Name("saturate_cast<int>") int saturateCastInt(@Cast("unsigned") int v);
@Namespace("cv") public static native @Cast("unsigned") @Name("saturate_cast<unsigned>") int saturateCastUnsigned(@Cast("unsigned") int v);
@Namespace("cv") public static native @Cast("uchar") @Name("saturate_cast<uchar>") byte saturateCastUchar(float v);
@Namespace("cv") public static native @Cast("schar") @Name("saturate_cast<schar>") byte saturateCastSchar(float v);
@Namespace("cv") public static native @Cast("ushort") @Name("saturate_cast<ushort>") short saturateCastUshort(float v);
@Namespace("cv") public static native @Name("saturate_cast<short>") short saturateCastShort(float v);
@Namespace("cv") public static native @Name("saturate_cast<int>") int saturateCastInt(float v);
@Namespace("cv") public static native @Cast("unsigned") @Name("saturate_cast<unsigned>") int saturateCastUnsigned(float v);
@Namespace("cv") public static native @Cast("uchar") @Name("saturate_cast<uchar>") byte saturateCastUchar(double v);
@Namespace("cv") public static native @Cast("schar") @Name("saturate_cast<schar>") byte saturateCastSchar(double v);
@Namespace("cv") public static native @Cast("ushort") @Name("saturate_cast<ushort>") short saturateCastUshort(double v);
@Namespace("cv") public static native @Name("saturate_cast<short>") short saturateCastShort(double v);
@Namespace("cv") public static native @Name("saturate_cast<int>") int saturateCastInt(double v);
@Namespace("cv") public static native @Cast("unsigned") @Name("saturate_cast<unsigned>") int saturateCastUnsigned(double v);

// we intentionally do not clip negative numbers, to make -1 become 0xffffffff etc.



//////////////////////////////// low-level functions ////////////////////////////////

@Namespace("cv") public static native int LU(FloatPointer A, @Cast("size_t") long astep, int m, FloatPointer b, @Cast("size_t") long bstep, int n);
@Namespace("cv") public static native int LU(FloatBuffer A, @Cast("size_t") long astep, int m, FloatBuffer b, @Cast("size_t") long bstep, int n);
@Namespace("cv") public static native int LU(float[] A, @Cast("size_t") long astep, int m, float[] b, @Cast("size_t") long bstep, int n);
@Namespace("cv") public static native int LU(DoublePointer A, @Cast("size_t") long astep, int m, DoublePointer b, @Cast("size_t") long bstep, int n);
@Namespace("cv") public static native int LU(DoubleBuffer A, @Cast("size_t") long astep, int m, DoubleBuffer b, @Cast("size_t") long bstep, int n);
@Namespace("cv") public static native int LU(double[] A, @Cast("size_t") long astep, int m, double[] b, @Cast("size_t") long bstep, int n);
@Namespace("cv") public static native @Cast("bool") boolean Cholesky(FloatPointer A, @Cast("size_t") long astep, int m, FloatPointer b, @Cast("size_t") long bstep, int n);
@Namespace("cv") public static native @Cast("bool") boolean Cholesky(FloatBuffer A, @Cast("size_t") long astep, int m, FloatBuffer b, @Cast("size_t") long bstep, int n);
@Namespace("cv") public static native @Cast("bool") boolean Cholesky(float[] A, @Cast("size_t") long astep, int m, float[] b, @Cast("size_t") long bstep, int n);
@Namespace("cv") public static native @Cast("bool") boolean Cholesky(DoublePointer A, @Cast("size_t") long astep, int m, DoublePointer b, @Cast("size_t") long bstep, int n);
@Namespace("cv") public static native @Cast("bool") boolean Cholesky(DoubleBuffer A, @Cast("size_t") long astep, int m, DoubleBuffer b, @Cast("size_t") long bstep, int n);
@Namespace("cv") public static native @Cast("bool") boolean Cholesky(double[] A, @Cast("size_t") long astep, int m, double[] b, @Cast("size_t") long bstep, int n);

@Namespace("cv") public static native int normL1_(@Cast("const uchar*") BytePointer a, @Cast("const uchar*") BytePointer b, int n);
@Namespace("cv") public static native int normL1_(@Cast("const uchar*") ByteBuffer a, @Cast("const uchar*") ByteBuffer b, int n);
@Namespace("cv") public static native int normL1_(@Cast("const uchar*") byte[] a, @Cast("const uchar*") byte[] b, int n);
@Namespace("cv") public static native int normHamming(@Cast("const uchar*") BytePointer a, @Cast("const uchar*") BytePointer b, int n);
@Namespace("cv") public static native int normHamming(@Cast("const uchar*") ByteBuffer a, @Cast("const uchar*") ByteBuffer b, int n);
@Namespace("cv") public static native int normHamming(@Cast("const uchar*") byte[] a, @Cast("const uchar*") byte[] b, int n);
@Namespace("cv") public static native int normHamming(@Cast("const uchar*") BytePointer a, @Cast("const uchar*") BytePointer b, int n, int cellSize);
@Namespace("cv") public static native int normHamming(@Cast("const uchar*") ByteBuffer a, @Cast("const uchar*") ByteBuffer b, int n, int cellSize);
@Namespace("cv") public static native int normHamming(@Cast("const uchar*") byte[] a, @Cast("const uchar*") byte[] b, int n, int cellSize);
@Namespace("cv") public static native float normL1_(@Const FloatPointer a, @Const FloatPointer b, int n);
@Namespace("cv") public static native float normL1_(@Const FloatBuffer a, @Const FloatBuffer b, int n);
@Namespace("cv") public static native float normL1_(@Const float[] a, @Const float[] b, int n);
@Namespace("cv") public static native float normL2Sqr_(@Const FloatPointer a, @Const FloatPointer b, int n);
@Namespace("cv") public static native float normL2Sqr_(@Const FloatBuffer a, @Const FloatBuffer b, int n);
@Namespace("cv") public static native float normL2Sqr_(@Const float[] a, @Const float[] b, int n);

@Namespace("cv") public static native void exp(@Const FloatPointer src, FloatPointer dst, int n);
@Namespace("cv") public static native void exp(@Const FloatBuffer src, FloatBuffer dst, int n);
@Namespace("cv") public static native void exp(@Const float[] src, float[] dst, int n);
@Namespace("cv") public static native void log(@Const FloatPointer src, FloatPointer dst, int n);
@Namespace("cv") public static native void log(@Const FloatBuffer src, FloatBuffer dst, int n);
@Namespace("cv") public static native void log(@Const float[] src, float[] dst, int n);
@Namespace("cv") public static native void fastAtan2(@Const FloatPointer y, @Const FloatPointer x, FloatPointer dst, int n, @Cast("bool") boolean angleInDegrees);
@Namespace("cv") public static native void fastAtan2(@Const FloatBuffer y, @Const FloatBuffer x, FloatBuffer dst, int n, @Cast("bool") boolean angleInDegrees);
@Namespace("cv") public static native void fastAtan2(@Const float[] y, @Const float[] x, float[] dst, int n, @Cast("bool") boolean angleInDegrees);
@Namespace("cv") public static native void magnitude(@Const FloatPointer x, @Const FloatPointer y, FloatPointer dst, int n);
@Namespace("cv") public static native void magnitude(@Const FloatBuffer x, @Const FloatBuffer y, FloatBuffer dst, int n);
@Namespace("cv") public static native void magnitude(@Const float[] x, @Const float[] y, float[] dst, int n);

/** computes cube root of the argument */
@Namespace("cv") public static native float cubeRoot(float val);
/** computes the angle in degrees (0..360) of the vector (x,y) */
@Namespace("cv") public static native float fastAtan2(float y, float x);



/////////////////////////////////// inline norms ////////////////////////////////////

















////////////////// forward declarations for important OpenCV types //////////////////
    @Namespace("cv::ogl") @Opaque public static class Buffer extends Pointer {
        /** Empty constructor. */
        public Buffer() { }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public Buffer(Pointer p) { super(p); }
    }
    @Namespace("cv::ogl") @Opaque public static class Texture2D extends Pointer {
        /** Empty constructor. */
        public Texture2D() { }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public Texture2D(Pointer p) { super(p); }
    }
    @Namespace("cv::ogl") @Opaque public static class Arrays extends Pointer {
        /** Empty constructor. */
        public Arrays() { }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public Arrays(Pointer p) { super(p); }
    }

    @Namespace("cv::cuda") @Opaque public static class GpuMat extends Pointer {
        /** Empty constructor. */
        public GpuMat() { }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public GpuMat(Pointer p) { super(p); }
    }
    @Namespace("cv::cuda") @Opaque public static class CudaMem extends Pointer {
        /** Empty constructor. */
        public CudaMem() { }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public CudaMem(Pointer p) { super(p); }
    }
    @Namespace("cv::cuda") @Opaque public static class Stream extends Pointer {
        /** Empty constructor. */
        public Stream() { }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public Stream(Pointer p) { super(p); }
    }
    @Namespace("cv::cuda") @Opaque public static class Event extends Pointer {
        /** Empty constructor. */
        public Event() { }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public Event(Pointer p) { super(p); }
    }


@Namespace("cv::ipp") public static native void setIppStatus(int status, @Cast("const char*") BytePointer funcname/*=NULL*/, @Cast("const char*") BytePointer filename/*=NULL*/,
                             int line/*=0*/);
@Namespace("cv::ipp") public static native void setIppStatus(int status);
@Namespace("cv::ipp") public static native void setIppStatus(int status, String funcname/*=NULL*/, String filename/*=NULL*/,
                             int line/*=0*/);
@Namespace("cv::ipp") public static native int getIppStatus();
@Namespace("cv::ipp") public static native @Str BytePointer getIppErrorLocation();
@Namespace("cv::ipp") public static native @Cast("bool") boolean useIPP();
@Namespace("cv::ipp") public static native void setUseIPP(@Cast("bool") boolean flag);

 // ipp

// #if CV_NEON

// #endif

 // cv

// #endif //__OPENCV_CORE_BASE_HPP__


// Parsed from <opencv2/core/cvstd.hpp>

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

// #ifndef __OPENCV_CORE_CVSTD_HPP__
// #define __OPENCV_CORE_CVSTD_HPP__

// #ifndef __cplusplus
// #endif

// #include "opencv2/core/cvdef.h"

// #include <cstddef>
// #include <cstring>
// #include <cctype>

// #ifndef OPENCV_NOSTL
// #  include <string>
// #endif

// import useful primitives from stl
// #ifndef OPENCV_NOSTL_TRANSITIONAL
// #  include <algorithm>
// #  include <utility>
// #  include <cstdlib> //for abs(int)
// #  include <cmath>

    @Namespace("std") public static native @Cast("uchar") byte abs(@Cast("uchar") byte a);
    @Namespace("std") public static native @Cast("ushort") short abs(@Cast("ushort") short a);
    @Namespace("std") public static native @Cast("unsigned") int abs(@Cast("unsigned") int a);


// #else
// #endif

//////////////////////////// memory management functions ////////////////////////////

/**
  Allocates memory buffer

  This is specialized OpenCV memory allocation function that returns properly aligned memory buffers.
  The usage is identical to malloc(). The allocated buffers must be freed with cv::fastFree().
  If there is not enough memory, the function calls cv::error(), which raises an exception.

  \param bufSize buffer size in bytes
  \return the allocated memory buffer.
*/
@Namespace("cv") public static native Pointer fastMalloc(@Cast("size_t") long bufSize);

/**
  Frees the memory allocated with cv::fastMalloc

  This is the corresponding deallocation function for cv::fastMalloc().
  When ptr==NULL, the function has no effect.
*/
@Namespace("cv") public static native void fastFree(Pointer ptr);

/**
  The STL-compilant memory Allocator based on cv::fastMalloc() and cv::fastFree()
*/

// Metafunction to avoid taking a reference to void.

@Name("cv::detail::RefOrVoid<void>") public static class RefOrVoid extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public RefOrVoid() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public RefOrVoid(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public RefOrVoid(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public RefOrVoid position(int position) {
        return (RefOrVoid)super.position(position);
    }
 @Opaque public static class type extends Pointer {
    /** Empty constructor. */
    public type() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public type(Pointer p) { super(p); }
} }

// This class would be private to Ptr, if it didn't have to be a non-template.
@Namespace("cv::detail") @Opaque public static class PtrOwner extends Pointer {
    /** Empty constructor. */
    public PtrOwner() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public PtrOwner(Pointer p) { super(p); }
}



/*
  A smart shared pointer class with reference counting.

  A Ptr<T> stores a pointer and owns a (potentially different) pointer.
  The stored pointer has type T and is the one returned by get() et al,
  while the owned pointer can have any type and is the one deleted
  when there are no more Ptrs that own it. You can't directly obtain the
  owned pointer.

  The interface of this class is mostly a subset of that of C++11's
  std::shared_ptr.
*/

/* Overload of the generic swap. */

/* Obvious comparisons. */

/* Convenience creation functions. In the far future, there may be variadic templates here. */


//////////////////////////////// string class //////////////////////////////// //for string constructor from FileNode


////////////////////////// cv::String implementation /////////////////////////



























































































// ************************* cv::String non-member functions *************************

@Namespace("cv") public static native @Str @Name("operator+") BytePointer add(@Str BytePointer lhs, @Str BytePointer rhs);
@Namespace("cv") public static native @Str @Name("operator+") String add(@Str String lhs, @Str String rhs);

@Namespace("cv") public static native @Str @Name("operator+") BytePointer add(@Str BytePointer lhs, @Cast("char") byte rhs);
@Namespace("cv") public static native @Str @Name("operator+") String add(@Str String lhs, @Cast("char") byte rhs);

@Namespace("cv") public static native @Str @Name("operator+") BytePointer add(@Cast("char") byte lhs, @Str BytePointer rhs);
@Namespace("cv") public static native @Str @Name("operator+") String add(@Cast("char") byte lhs, @Str String rhs);

@Namespace("cv") public static native @Cast("bool") @Name("operator==") boolean equals(@Str BytePointer lhs, @Str BytePointer rhs);
@Namespace("cv") public static native @Cast("bool") @Name("operator==") boolean equals(@Str String lhs, @Str String rhs);
@Namespace("cv") public static native @Cast("bool") @Name("operator!=") boolean notEquals(@Str BytePointer lhs, @Str BytePointer rhs);
@Namespace("cv") public static native @Cast("bool") @Name("operator!=") boolean notEquals(@Str String lhs, @Str String rhs);
@Namespace("cv") public static native @Cast("bool") @Name("operator<") boolean lessThan(@Str BytePointer lhs, @Str BytePointer rhs);
@Namespace("cv") public static native @Cast("bool") @Name("operator<") boolean lessThan(@Str String lhs, @Str String rhs);
@Namespace("cv") public static native @Cast("bool") @Name("operator<=") boolean lessThanEquals(@Str BytePointer lhs, @Str BytePointer rhs);
@Namespace("cv") public static native @Cast("bool") @Name("operator<=") boolean lessThanEquals(@Str String lhs, @Str String rhs);
@Namespace("cv") public static native @Cast("bool") @Name("operator>") boolean greaterThan(@Str BytePointer lhs, @Str BytePointer rhs);
@Namespace("cv") public static native @Cast("bool") @Name("operator>") boolean greaterThan(@Str String lhs, @Str String rhs);
@Namespace("cv") public static native @Cast("bool") @Name("operator>=") boolean greaterThanEquals(@Str BytePointer lhs, @Str BytePointer rhs);
@Namespace("cv") public static native @Cast("bool") @Name("operator>=") boolean greaterThanEquals(@Str String lhs, @Str String rhs);

 // cv

// #ifndef OPENCV_NOSTL_TRANSITIONAL
    


// #include "opencv2/core/ptr.inl.hpp"

// #endif //__OPENCV_CORE_CVSTD_HPP__


// Parsed from <opencv2/core/types_c.h>

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

// #ifndef __OPENCV_CORE_TYPES_H__
// #define __OPENCV_CORE_TYPES_H__

// #ifdef HAVE_IPL
// #  ifndef __IPL_H__
// #    if defined WIN32 || defined _WIN32
// #    else
// #      include <ipl/ipl.h>
// #    endif
// #  endif
// #elif defined __IPL_H__
// #  define HAVE_IPL
// #endif

// #include "opencv2/core/cvdef.h"

// #ifndef SKIP_INCLUDES
// #include <assert.h>
// #include <stdlib.h>
// #include <string.h>
// #include <float.h>
// #endif // SKIP_INCLUDES

// #if defined WIN32 || defined _WIN32
// #else
// #  define CV_CDECL
// #  define CV_STDCALL
// #endif

// #ifndef CV_DEFAULT
// #  ifdef __cplusplus
// #    define CV_DEFAULT(val) = val
// #  else
// #  endif
// #endif

// #ifndef CV_EXTERN_C_FUNCPTR
// #  ifdef __cplusplus
// #    define CV_EXTERN_C_FUNCPTR(x) extern "C" { typedef x; }
// #  else
// #  endif
// #endif

// #ifndef CVAPI
// #  define CVAPI(rettype) CV_EXTERN_C CV_EXPORTS rettype CV_CDECL
// #endif

// #ifndef CV_IMPL
// #  define CV_IMPL CV_EXTERN_C
// #endif

// #ifdef __cplusplus
// #  include "opencv2/core.hpp"
// #endif

/* CvArr* is used to pass arbitrary
 * array-like data structures
 * into functions where the particular
 * array type is recognized at runtime:
 */

public static class Cv32suf extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public Cv32suf() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Cv32suf(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Cv32suf(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public Cv32suf position(int position) {
        return (Cv32suf)super.position(position);
    }

    public native int i(); public native Cv32suf i(int i);
    public native @Cast("unsigned") int u(); public native Cv32suf u(int u);
    public native float f(); public native Cv32suf f(float f);
}

public static class Cv64suf extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public Cv64suf() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Cv64suf(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Cv64suf(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public Cv64suf position(int position) {
        return (Cv64suf)super.position(position);
    }

    public native @Cast("int64") long i(); public native Cv64suf i(long i);
    public native @Cast("uint64") int u(); public native Cv64suf u(int u);
    public native double f(); public native Cv64suf f(double f);
}

/** enum  */
public static final int
 CV_StsOk= 0,  /* everithing is ok                */
 CV_StsBackTrace= -1,  /* pseudo error for back trace     */
 CV_StsError= -2,  /* unknown /unspecified error      */
 CV_StsInternal= -3,  /* internal error (bad state)      */
 CV_StsNoMem= -4,  /* insufficient memory             */
 CV_StsBadArg= -5,  /* function arg/param is bad       */
 CV_StsBadFunc= -6,  /* unsupported function            */
 CV_StsNoConv= -7,  /* iter. didn't converge           */
 CV_StsAutoTrace= -8,  /* tracing                         */
 CV_HeaderIsNull= -9,  /* image header is NULL            */
 CV_BadImageSize= -10, /* image size is invalid           */
 CV_BadOffset= -11, /* offset is invalid               */
 CV_BadDataPtr= -12, /**/
 CV_BadStep= -13, /**/
 CV_BadModelOrChSeq= -14, /**/
 CV_BadNumChannels= -15, /**/
 CV_BadNumChannel1U= -16, /**/
 CV_BadDepth= -17, /**/
 CV_BadAlphaChannel= -18, /**/
 CV_BadOrder= -19, /**/
 CV_BadOrigin= -20, /**/
 CV_BadAlign= -21, /**/
 CV_BadCallBack= -22, /**/
 CV_BadTileSize= -23, /**/
 CV_BadCOI= -24, /**/
 CV_BadROISize= -25, /**/
 CV_MaskIsTiled= -26, /**/
 CV_StsNullPtr= -27, /* null pointer */
 CV_StsVecLengthErr= -28, /* incorrect vector length */
 CV_StsFilterStructContentErr= -29, /* incorr. filter structure content */
 CV_StsKernelStructContentErr= -30, /* incorr. transform kernel content */
 CV_StsFilterOffsetErr= -31, /* incorrect filter offset value */
 CV_StsBadSize= -201, /* the input/output structure size is incorrect  */
 CV_StsDivByZero= -202, /* division by zero */
 CV_StsInplaceNotSupported= -203, /* in-place operation is not supported */
 CV_StsObjectNotFound= -204, /* request can't be completed */
 CV_StsUnmatchedFormats= -205, /* formats of input/output arrays differ */
 CV_StsBadFlag= -206, /* flag is wrong or not supported */
 CV_StsBadPoint= -207, /* bad CvPoint */
 CV_StsBadMask= -208, /* bad format of mask (neither 8uC1 nor 8sC1)*/
 CV_StsUnmatchedSizes= -209, /* sizes of input/output structures do not match */
 CV_StsUnsupportedFormat= -210, /* the data format/type is not supported by the function*/
 CV_StsOutOfRange= -211, /* some of parameters are out of range */
 CV_StsParseError= -212, /* invalid syntax/structure of the parsed file */
 CV_StsNotImplemented= -213, /* the requested function/feature is not implemented */
 CV_StsBadMemBlock= -214, /* an allocated block has been corrupted */
 CV_StsAssert= -215, /* assertion failed */
 CV_GpuNotSupported= -216,
 CV_GpuApiCallError= -217,
 CV_OpenGlNotSupported= -218,
 CV_OpenGlApiCallError= -219,
 CV_OpenCLApiCallError= -220,
 CV_OpenCLDoubleNotSupported= -221,
 CV_OpenCLInitError= -222,
 CV_OpenCLNoAMDBlasFft= -223;

/****************************************************************************************\
*                             Common macros and inline functions                         *
\****************************************************************************************/

// #define CV_SWAP(a,b,t) ((t) = (a), (a) = (b), (b) = (t))

/* min & max without jumps */
// #define  CV_IMIN(a, b)  ((a) ^ (((a)^(b)) & (((a) < (b)) - 1)))

// #define  CV_IMAX(a, b)  ((a) ^ (((a)^(b)) & (((a) > (b)) - 1)))

/* absolute value without jumps */
// #ifndef __cplusplus
// #else
// #  define  CV_IABS(a)     abs(a)
// #endif
// #define  CV_CMP(a,b)    (((a) > (b)) - ((a) < (b)))
// #define  CV_SIGN(a)     CV_CMP((a),0)

// #define cvInvSqrt(value) ((float)(1./sqrt(value)))
// #define cvSqrt(value)  ((float)sqrt(value))


/*************** Random number generation *******************/

public static final long CV_RNG_COEFF = 4164903690L;

public static native @Cast("CvRNG") long cvRNG( @Cast("int64") long seed/*=-1*/);
public static native @Cast("CvRNG") long cvRNG();

/* Return random 32-bit unsigned integer: */
public static native @Cast("unsigned") int cvRandInt( @Cast("CvRNG*") LongPointer rng );
public static native @Cast("unsigned") int cvRandInt( @Cast("CvRNG*") LongBuffer rng );
public static native @Cast("unsigned") int cvRandInt( @Cast("CvRNG*") long[] rng );

/* Returns random floating-point number between 0 and 1: */
public static native double cvRandReal( @Cast("CvRNG*") LongPointer rng );
public static native double cvRandReal( @Cast("CvRNG*") LongBuffer rng );
public static native double cvRandReal( @Cast("CvRNG*") long[] rng );

/****************************************************************************************\
*                                  Image type (IplImage)                                 *
\****************************************************************************************/

// #ifndef HAVE_IPL

/*
 * The following definitions (until #endif)
 * is an extract from IPL headers.
 * Copyright (c) 1995 Intel Corporation.
 */
public static final int IPL_DEPTH_SIGN = 0x80000000;

public static final int IPL_DEPTH_1U =     1;
public static final int IPL_DEPTH_8U =     8;
public static final int IPL_DEPTH_16U =   16;
public static final int IPL_DEPTH_32F =   32;

public static final int IPL_DEPTH_8S =  (IPL_DEPTH_SIGN| 8);
public static final int IPL_DEPTH_16S = (IPL_DEPTH_SIGN|16);
public static final int IPL_DEPTH_32S = (IPL_DEPTH_SIGN|32);

public static final int IPL_DATA_ORDER_PIXEL =  0;
public static final int IPL_DATA_ORDER_PLANE =  1;

public static final int IPL_ORIGIN_TL = 0;
public static final int IPL_ORIGIN_BL = 1;

public static final int IPL_ALIGN_4BYTES =   4;
public static final int IPL_ALIGN_8BYTES =   8;
public static final int IPL_ALIGN_16BYTES = 16;
public static final int IPL_ALIGN_32BYTES = 32;

public static final int IPL_ALIGN_DWORD =   IPL_ALIGN_4BYTES;
public static final int IPL_ALIGN_QWORD =   IPL_ALIGN_8BYTES;

public static final int IPL_BORDER_CONSTANT =   0;
public static final int IPL_BORDER_REPLICATE =  1;
public static final int IPL_BORDER_REFLECT =    2;
public static final int IPL_BORDER_WRAP =       3;

@NoOffset public static class IplImage extends AbstractIplImage {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public IplImage(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public IplImage(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public IplImage position(int position) {
        return (IplImage)super.position(position);
    }

    public native int nSize(); public native IplImage nSize(int nSize);             /* sizeof(IplImage) */
    public native int ID(); public native IplImage ID(int ID);                /* version (=0)*/
    public native int nChannels(); public native IplImage nChannels(int nChannels);         /* Most of OpenCV functions support 1,2,3 or 4 channels */
    public native int alphaChannel(); public native IplImage alphaChannel(int alphaChannel);      /* Ignored by OpenCV */
    public native int depth(); public native IplImage depth(int depth);             /* Pixel depth in bits: IPL_DEPTH_8U, IPL_DEPTH_8S, IPL_DEPTH_16S,
                               IPL_DEPTH_32S, IPL_DEPTH_32F and IPL_DEPTH_64F are supported.  */
    public native @Cast("char") byte colorModel(int i); public native IplImage colorModel(int i, byte colorModel);
    @MemberGetter public native @Cast("char*") BytePointer colorModel();     /* Ignored by OpenCV */
    public native @Cast("char") byte channelSeq(int i); public native IplImage channelSeq(int i, byte channelSeq);
    @MemberGetter public native @Cast("char*") BytePointer channelSeq();     /* ditto */
    public native int dataOrder(); public native IplImage dataOrder(int dataOrder);         /* 0 - interleaved color channels, 1 - separate color channels.
                               cvCreateImage can only create interleaved images */
    public native int origin(); public native IplImage origin(int origin);            /* 0 - top-left origin,
                               1 - bottom-left origin (Windows bitmaps style).  */
    public native int align(); public native IplImage align(int align);             /* Alignment of image rows (4 or 8).
                               OpenCV ignores it and uses widthStep instead.    */
    public native int width(); public native IplImage width(int width);             /* Image width in pixels.                           */
    public native int height(); public native IplImage height(int height);            /* Image height in pixels.                          */
    public native IplROI roi(); public native IplImage roi(IplROI roi);    /* Image ROI. If NULL, the whole image is selected. */
    public native IplImage maskROI(); public native IplImage maskROI(IplImage maskROI);      /* Must be NULL. */
    public native Pointer imageId(); public native IplImage imageId(Pointer imageId);                 /* "           " */
    public native IplTileInfo tileInfo(); public native IplImage tileInfo(IplTileInfo tileInfo);  /* "           " */
    public native int imageSize(); public native IplImage imageSize(int imageSize);         /* Image data size in bytes
                               (==image->height*image->widthStep
                               in case of interleaved data)*/
    public native @Cast("char*") BytePointer imageData(); public native IplImage imageData(BytePointer imageData);        /* Pointer to aligned image data.         */
    public native int widthStep(); public native IplImage widthStep(int widthStep);         /* Size of aligned image row in bytes.    */
    public native int BorderMode(int i); public native IplImage BorderMode(int i, int BorderMode);
    @MemberGetter public native IntPointer BorderMode();     /* Ignored by OpenCV.                     */
    public native int BorderConst(int i); public native IplImage BorderConst(int i, int BorderConst);
    @MemberGetter public native IntPointer BorderConst();    /* Ditto.                                 */
    public native @Cast("char*") BytePointer imageDataOrigin(); public native IplImage imageDataOrigin(BytePointer imageDataOrigin);  /* Pointer to very origin of image data
                               (not necessarily aligned) -
                               needed for correct deallocation */

// #ifdef __cplusplus
    public IplImage() { allocate(); }
    private native void allocate();
    public IplImage(@Const @ByRef Mat m) { allocate(m); }
    private native void allocate(@Const @ByRef Mat m);
// #endif
}

@Opaque public static class IplTileInfo extends Pointer {
    /** Empty constructor. */
    public IplTileInfo() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public IplTileInfo(Pointer p) { super(p); }
}

public static class IplROI extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public IplROI() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public IplROI(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public IplROI(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public IplROI position(int position) {
        return (IplROI)super.position(position);
    }

    public native int coi(); public native IplROI coi(int coi); /* 0 - no COI (all channels are selected), 1 - 0th channel is selected ...*/
    public native int xOffset(); public native IplROI xOffset(int xOffset);
    public native int yOffset(); public native IplROI yOffset(int yOffset);
    public native int width(); public native IplROI width(int width);
    public native int height(); public native IplROI height(int height);
}

public static class IplConvKernel extends org.bytedeco.javacpp.helper.opencv_imgproc.AbstractIplConvKernel {
    static { Loader.load(); }
    /** Default native constructor. */
    public IplConvKernel() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public IplConvKernel(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public IplConvKernel(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public IplConvKernel position(int position) {
        return (IplConvKernel)super.position(position);
    }

    public native int nCols(); public native IplConvKernel nCols(int nCols);
    public native int nRows(); public native IplConvKernel nRows(int nRows);
    public native int anchorX(); public native IplConvKernel anchorX(int anchorX);
    public native int anchorY(); public native IplConvKernel anchorY(int anchorY);
    public native IntPointer values(); public native IplConvKernel values(IntPointer values);
    public native int nShiftR(); public native IplConvKernel nShiftR(int nShiftR);
}

public static class IplConvKernelFP extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public IplConvKernelFP() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public IplConvKernelFP(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public IplConvKernelFP(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public IplConvKernelFP position(int position) {
        return (IplConvKernelFP)super.position(position);
    }

    public native int nCols(); public native IplConvKernelFP nCols(int nCols);
    public native int nRows(); public native IplConvKernelFP nRows(int nRows);
    public native int anchorX(); public native IplConvKernelFP anchorX(int anchorX);
    public native int anchorY(); public native IplConvKernelFP anchorY(int anchorY);
    public native FloatPointer values(); public native IplConvKernelFP values(FloatPointer values);
}

public static final int IPL_IMAGE_HEADER = 1;
public static final int IPL_IMAGE_DATA =   2;
public static final int IPL_IMAGE_ROI =    4;

// #endif/*HAVE_IPL*/

/* extra border mode */
public static final int IPL_BORDER_REFLECT_101 =    4;
public static final int IPL_BORDER_TRANSPARENT =    5;

public static native @MemberGetter int IPL_IMAGE_MAGIC_VAL();
public static final int IPL_IMAGE_MAGIC_VAL = IPL_IMAGE_MAGIC_VAL();
public static final String CV_TYPE_NAME_IMAGE = "opencv-image";

// #define CV_IS_IMAGE_HDR(img)
//     ((img) != NULL && ((const IplImage*)(img))->nSize == sizeof(IplImage))

// #define CV_IS_IMAGE(img)
//     (CV_IS_IMAGE_HDR(img) && ((IplImage*)img)->imageData != NULL)

/* for storing double-precision
   floating point data in IplImage's */
public static final int IPL_DEPTH_64F =  64;

/* get reference to pixel at (col,row),
   for multi-channel images (col) should be multiplied by number of channels */
// #define CV_IMAGE_ELEM( image, elemtype, row, col )
//     (((elemtype*)((image)->imageData + (image)->widthStep*(row)))[(col)])

/****************************************************************************************\
*                                  Matrix type (CvMat)                                   *
\****************************************************************************************/

public static final int CV_AUTO_STEP =  0x7fffffff;
public static final CvSlice CV_WHOLE_ARR =  cvSlice( 0, 0x3fffffff );

public static final int CV_MAGIC_MASK =       0xFFFF0000;
public static final int CV_MAT_MAGIC_VAL =    0x42420000;
public static final String CV_TYPE_NAME_MAT =    "opencv-matrix";

@NoOffset public static class CvMat extends AbstractCvMat {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvMat(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvMat(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public CvMat position(int position) {
        return (CvMat)super.position(position);
    }

    public native int type(); public native CvMat type(int type);
    public native int step(); public native CvMat step(int step);

    /* for internal use only */
    public native IntPointer refcount(); public native CvMat refcount(IntPointer refcount);
    public native int hdr_refcount(); public native CvMat hdr_refcount(int hdr_refcount);

        @Name("data.ptr") public native @Cast("uchar*") BytePointer data_ptr(); public native CvMat data_ptr(BytePointer data_ptr);
        @Name("data.s") public native ShortPointer data_s(); public native CvMat data_s(ShortPointer data_s);
        @Name("data.i") public native IntPointer data_i(); public native CvMat data_i(IntPointer data_i);
        @Name("data.fl") public native FloatPointer data_fl(); public native CvMat data_fl(FloatPointer data_fl);
        @Name("data.db") public native DoublePointer data_db(); public native CvMat data_db(DoublePointer data_db);

// #ifdef __cplusplus
        public native int rows(); public native CvMat rows(int rows);
        public native int height(); public native CvMat height(int height);
        public native int cols(); public native CvMat cols(int cols);
        public native int width(); public native CvMat width(int width);
// #else
// #endif


// #ifdef __cplusplus
    public CvMat() { allocate(); }
    private native void allocate();
    public CvMat(@Const @ByRef CvMat m) { allocate(m); }
    private native void allocate(@Const @ByRef CvMat m);
    public CvMat(@Const @ByRef Mat m) { allocate(m); }
    private native void allocate(@Const @ByRef Mat m);
// #endif

}


// #define CV_IS_MAT_HDR(mat)
//     ((mat) != NULL &&
//     (((const CvMat*)(mat))->type & CV_MAGIC_MASK) == CV_MAT_MAGIC_VAL &&
//     ((const CvMat*)(mat))->cols > 0 && ((const CvMat*)(mat))->rows > 0)

// #define CV_IS_MAT_HDR_Z(mat)
//     ((mat) != NULL &&
//     (((const CvMat*)(mat))->type & CV_MAGIC_MASK) == CV_MAT_MAGIC_VAL &&
//     ((const CvMat*)(mat))->cols >= 0 && ((const CvMat*)(mat))->rows >= 0)

// #define CV_IS_MAT(mat)
//     (CV_IS_MAT_HDR(mat) && ((const CvMat*)(mat))->data.ptr != NULL)

// #define CV_IS_MASK_ARR(mat)
//     (((mat)->type & (CV_MAT_TYPE_MASK & ~CV_8SC1)) == 0)

// #define CV_ARE_TYPES_EQ(mat1, mat2)
//     ((((mat1)->type ^ (mat2)->type) & CV_MAT_TYPE_MASK) == 0)

// #define CV_ARE_CNS_EQ(mat1, mat2)
//     ((((mat1)->type ^ (mat2)->type) & CV_MAT_CN_MASK) == 0)

// #define CV_ARE_DEPTHS_EQ(mat1, mat2)
//     ((((mat1)->type ^ (mat2)->type) & CV_MAT_DEPTH_MASK) == 0)

// #define CV_ARE_SIZES_EQ(mat1, mat2)
//     ((mat1)->rows == (mat2)->rows && (mat1)->cols == (mat2)->cols)

// #define CV_IS_MAT_CONST(mat)
//     (((mat)->rows|(mat)->cols) == 1)

// #define IPL2CV_DEPTH(depth)
//     ((((CV_8U)+(CV_16U<<4)+(CV_32F<<8)+(CV_64F<<16)+(CV_8S<<20)+
//     (CV_16S<<24)+(CV_32S<<28)) >> ((((depth) & 0xF0) >> 2) +
//     (((depth) & IPL_DEPTH_SIGN) ? 20 : 0))) & 15)

/* Inline constructor. No data is allocated internally!!!
 * (Use together with cvCreateData, or use cvCreateMat instead to
 * get a matrix with allocated data):
 */
public static native @ByVal CvMat cvMat( int rows, int cols, int type, Pointer data/*=NULL*/);
public static native @ByVal CvMat cvMat( int rows, int cols, int type);

// #ifdef __cplusplus

// #endif


// #define CV_MAT_ELEM_PTR_FAST( mat, row, col, pix_size )
//     (assert( (unsigned)(row) < (unsigned)(mat).rows &&
//              (unsigned)(col) < (unsigned)(mat).cols ),
//      (mat).data.ptr + (size_t)(mat).step*(row) + (pix_size)*(col))

// #define CV_MAT_ELEM_PTR( mat, row, col )
//     CV_MAT_ELEM_PTR_FAST( mat, row, col, CV_ELEM_SIZE((mat).type) )

// #define CV_MAT_ELEM( mat, elemtype, row, col )
//     (*(elemtype*)CV_MAT_ELEM_PTR_FAST( mat, row, col, sizeof(elemtype)))


public static native double cvmGet( @Const CvMat mat, int row, int col );


public static native void cvmSet( CvMat mat, int row, int col, double value );


public static native int cvIplDepth( int type );


/****************************************************************************************\
*                       Multi-dimensional dense array (CvMatND)                          *
\****************************************************************************************/

public static final int CV_MATND_MAGIC_VAL =    0x42430000;
public static final String CV_TYPE_NAME_MATND =    "opencv-nd-matrix";

public static final int CV_MAX_DIM =            32;
public static final int CV_MAX_DIM_HEAP =       1024;

@NoOffset public static class CvMatND extends AbstractCvMatND {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvMatND(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvMatND(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public CvMatND position(int position) {
        return (CvMatND)super.position(position);
    }

    public native int type(); public native CvMatND type(int type);
    public native int dims(); public native CvMatND dims(int dims);

    public native IntPointer refcount(); public native CvMatND refcount(IntPointer refcount);
    public native int hdr_refcount(); public native CvMatND hdr_refcount(int hdr_refcount);

        @Name("data.ptr") public native @Cast("uchar*") BytePointer data_ptr(); public native CvMatND data_ptr(BytePointer data_ptr);
        @Name("data.fl") public native FloatPointer data_fl(); public native CvMatND data_fl(FloatPointer data_fl);
        @Name("data.db") public native DoublePointer data_db(); public native CvMatND data_db(DoublePointer data_db);
        @Name("data.i") public native IntPointer data_i(); public native CvMatND data_i(IntPointer data_i);
        @Name("data.s") public native ShortPointer data_s(); public native CvMatND data_s(ShortPointer data_s);

        @Name({"dim", ".size"}) public native int dim_size(int i); public native CvMatND dim_size(int i, int dim_size);
        @Name({"dim", ".step"}) public native int dim_step(int i); public native CvMatND dim_step(int i, int dim_step);

// #ifdef __cplusplus
    public CvMatND() { allocate(); }
    private native void allocate();
    public CvMatND(@Const @ByRef Mat m) { allocate(m); }
    private native void allocate(@Const @ByRef Mat m);
// #endif
}

// #define CV_IS_MATND_HDR(mat)
//     ((mat) != NULL && (((const CvMatND*)(mat))->type & CV_MAGIC_MASK) == CV_MATND_MAGIC_VAL)

// #define CV_IS_MATND(mat)
//     (CV_IS_MATND_HDR(mat) && ((const CvMatND*)(mat))->data.ptr != NULL)


/****************************************************************************************\
*                      Multi-dimensional sparse array (CvSparseMat)                      *
\****************************************************************************************/

public static final int CV_SPARSE_MAT_MAGIC_VAL =    0x42440000;
public static final String CV_TYPE_NAME_SPARSE_MAT =    "opencv-sparse-matrix";

public static class CvSparseMat extends AbstractCvSparseMat {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvSparseMat() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvSparseMat(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvSparseMat(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvSparseMat position(int position) {
        return (CvSparseMat)super.position(position);
    }

    public native int type(); public native CvSparseMat type(int type);
    public native int dims(); public native CvSparseMat dims(int dims);
    public native IntPointer refcount(); public native CvSparseMat refcount(IntPointer refcount);
    public native int hdr_refcount(); public native CvSparseMat hdr_refcount(int hdr_refcount);

    public native CvSet heap(); public native CvSparseMat heap(CvSet heap);
    public native Pointer hashtable(int i); public native CvSparseMat hashtable(int i, Pointer hashtable);
    @MemberGetter public native @Cast("void**") PointerPointer hashtable();
    public native int hashsize(); public native CvSparseMat hashsize(int hashsize);
    public native int valoffset(); public native CvSparseMat valoffset(int valoffset);
    public native int idxoffset(); public native CvSparseMat idxoffset(int idxoffset);
    public native int size(int i); public native CvSparseMat size(int i, int size);
    @MemberGetter public native IntPointer size();

// #ifdef __cplusplus
    public native void copyToSparseMat(@ByRef SparseMat m);
// #endif
}

// #ifdef __cplusplus
    public static native CvSparseMat cvCreateSparseMat(@Const @ByRef SparseMat m);
// #endif

// #define CV_IS_SPARSE_MAT_HDR(mat)
//     ((mat) != NULL &&
//     (((const CvSparseMat*)(mat))->type & CV_MAGIC_MASK) == CV_SPARSE_MAT_MAGIC_VAL)

// #define CV_IS_SPARSE_MAT(mat)
//     CV_IS_SPARSE_MAT_HDR(mat)

/**************** iteration through a sparse array *****************/

public static class CvSparseNode extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvSparseNode() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvSparseNode(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvSparseNode(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvSparseNode position(int position) {
        return (CvSparseNode)super.position(position);
    }

    public native @Cast("unsigned") int hashval(); public native CvSparseNode hashval(int hashval);
    public native CvSparseNode next(); public native CvSparseNode next(CvSparseNode next);
}

public static class CvSparseMatIterator extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvSparseMatIterator() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvSparseMatIterator(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvSparseMatIterator(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvSparseMatIterator position(int position) {
        return (CvSparseMatIterator)super.position(position);
    }

    public native CvSparseMat mat(); public native CvSparseMatIterator mat(CvSparseMat mat);
    public native CvSparseNode node(); public native CvSparseMatIterator node(CvSparseNode node);
    public native int curidx(); public native CvSparseMatIterator curidx(int curidx);
}

// #define CV_NODE_VAL(mat,node)   ((void*)((uchar*)(node) + (mat)->valoffset))
// #define CV_NODE_IDX(mat,node)   ((int*)((uchar*)(node) + (mat)->idxoffset))

/****************************************************************************************\
*                                         Histogram                                      *
\****************************************************************************************/

public static final int CV_HIST_MAGIC_VAL =     0x42450000;
public static final int CV_HIST_UNIFORM_FLAG =  (1 << 10);

/* indicates whether bin ranges are set already or not */
public static final int CV_HIST_RANGES_FLAG =   (1 << 11);

public static final int CV_HIST_ARRAY =         0;
public static final int CV_HIST_SPARSE =        1;
public static final int CV_HIST_TREE =          CV_HIST_SPARSE;

/* should be used as a parameter only,
   it turns to CV_HIST_UNIFORM_FLAG of hist->type */
public static final int CV_HIST_UNIFORM =       1;

public static class CvHistogram extends org.bytedeco.javacpp.helper.opencv_imgproc.AbstractCvHistogram {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvHistogram() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvHistogram(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvHistogram(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvHistogram position(int position) {
        return (CvHistogram)super.position(position);
    }

    public native int type(); public native CvHistogram type(int type);
    public native CvArr bins(); public native CvHistogram bins(CvArr bins);
    public native float thresh(int i, int j); public native CvHistogram thresh(int i, int j, float thresh);
    @MemberGetter public native @Cast("float(*)[2]") FloatPointer thresh();  /* For uniform histograms.                      */
    public native FloatPointer thresh2(int i); public native CvHistogram thresh2(int i, FloatPointer thresh2);
    @MemberGetter public native @Cast("float**") PointerPointer thresh2();                /* For non-uniform histograms.                  */
    public native @ByRef CvMatND mat(); public native CvHistogram mat(CvMatND mat);                    /* Embedded matrix header for array histograms. */
}

// #define CV_IS_HIST( hist )
//     ((hist) != NULL  &&
//      (((CvHistogram*)(hist))->type & CV_MAGIC_MASK) == CV_HIST_MAGIC_VAL &&
//      (hist)->bins != NULL)

// #define CV_IS_UNIFORM_HIST( hist )
//     (((hist)->type & CV_HIST_UNIFORM_FLAG) != 0)

// #define CV_IS_SPARSE_HIST( hist )
//     CV_IS_SPARSE_MAT((hist)->bins)

// #define CV_HIST_HAS_RANGES( hist )
//     (((hist)->type & CV_HIST_RANGES_FLAG) != 0)

/****************************************************************************************\
*                      Other supplementary data type definitions                         *
\****************************************************************************************/

/*************************************** CvRect *****************************************/

@NoOffset public static class CvRect extends AbstractCvRect {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvRect(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvRect(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public CvRect position(int position) {
        return (CvRect)super.position(position);
    }

    public native int x(); public native CvRect x(int x);
    public native int y(); public native CvRect y(int y);
    public native int width(); public native CvRect width(int width);
    public native int height(); public native CvRect height(int height);

// #ifdef __cplusplus
    public CvRect(int _x/*=0*/, int _y/*=0*/, int w/*=0*/, int h/*=0*/) { allocate(_x, _y, w, h); }
    private native void allocate(int _x/*=0*/, int _y/*=0*/, int w/*=0*/, int h/*=0*/);
    public CvRect() { allocate(); }
    private native void allocate();
// #endif
}

public static native @ByVal CvRect cvRect( int x, int y, int width, int height );


public static native @ByVal IplROI cvRectToROI( @ByVal CvRect rect, int coi );


public static native @ByVal CvRect cvROIToRect( @ByVal IplROI roi );

/*********************************** CvTermCriteria *************************************/

public static final int CV_TERMCRIT_ITER =    1;
public static final int CV_TERMCRIT_NUMBER =  CV_TERMCRIT_ITER;
public static final int CV_TERMCRIT_EPS =     2;

@NoOffset public static class CvTermCriteria extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvTermCriteria(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvTermCriteria(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public CvTermCriteria position(int position) {
        return (CvTermCriteria)super.position(position);
    }

    public native int type(); public native CvTermCriteria type(int type);  /* may be combination of
                     CV_TERMCRIT_ITER
                     CV_TERMCRIT_EPS */
    public native int max_iter(); public native CvTermCriteria max_iter(int max_iter);
    public native double epsilon(); public native CvTermCriteria epsilon(double epsilon);

// #ifdef __cplusplus
    public CvTermCriteria(int _type/*=0*/, int _iter/*=0*/, double _eps/*=0*/) { allocate(_type, _iter, _eps); }
    private native void allocate(int _type/*=0*/, int _iter/*=0*/, double _eps/*=0*/);
    public CvTermCriteria() { allocate(); }
    private native void allocate();
    public CvTermCriteria(@Const @ByRef TermCriteria t) { allocate(t); }
    private native void allocate(@Const @ByRef TermCriteria t);
    public native @ByVal @Name("operator cv::TermCriteria") TermCriteria asTermCriteria();
// #endif

}

public static native @ByVal CvTermCriteria cvTermCriteria( int type, int max_iter, double epsilon );


/******************************* CvPoint and variants ***********************************/

@NoOffset public static class CvPoint extends AbstractCvPoint {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvPoint(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvPoint(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public CvPoint position(int position) {
        return (CvPoint)super.position(position);
    }

    public native int x(); public native CvPoint x(int x);
    public native int y(); public native CvPoint y(int y);

// #ifdef __cplusplus
    public CvPoint(int _x/*=0*/, int _y/*=0*/) { allocate(_x, _y); }
    private native void allocate(int _x/*=0*/, int _y/*=0*/);
    public CvPoint() { allocate(); }
    private native void allocate();
// #endif
}


public static native @ByVal CvPoint cvPoint( int x, int y );


@NoOffset public static class CvPoint2D32f extends AbstractCvPoint2D32f {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvPoint2D32f(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvPoint2D32f(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public CvPoint2D32f position(int position) {
        return (CvPoint2D32f)super.position(position);
    }

    public native float x(); public native CvPoint2D32f x(float x);
    public native float y(); public native CvPoint2D32f y(float y);

// #ifdef __cplusplus
    public CvPoint2D32f(float _x/*=0*/, float _y/*=0*/) { allocate(_x, _y); }
    private native void allocate(float _x/*=0*/, float _y/*=0*/);
    public CvPoint2D32f() { allocate(); }
    private native void allocate();
// #endif
}


public static native @ByVal CvPoint2D32f cvPoint2D32f( double x, double y );


public static native @ByVal CvPoint2D32f cvPointTo32f( @ByVal CvPoint point );
public static native @ByVal @Cast("CvPoint2D32f*") FloatBuffer cvPointTo32f( @ByVal @Cast("CvPoint*") IntBuffer point );
public static native @ByVal @Cast("CvPoint2D32f*") float[] cvPointTo32f( @ByVal @Cast("CvPoint*") int[] point );


public static native @ByVal CvPoint cvPointFrom32f( @ByVal CvPoint2D32f point );
public static native @ByVal @Cast("CvPoint*") IntBuffer cvPointFrom32f( @ByVal @Cast("CvPoint2D32f*") FloatBuffer point );
public static native @ByVal @Cast("CvPoint*") int[] cvPointFrom32f( @ByVal @Cast("CvPoint2D32f*") float[] point );


@NoOffset public static class CvPoint3D32f extends AbstractCvPoint3D32f {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvPoint3D32f(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvPoint3D32f(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public CvPoint3D32f position(int position) {
        return (CvPoint3D32f)super.position(position);
    }

    public native float x(); public native CvPoint3D32f x(float x);
    public native float y(); public native CvPoint3D32f y(float y);
    public native float z(); public native CvPoint3D32f z(float z);

// #ifdef __cplusplus
    public CvPoint3D32f(float _x/*=0*/, float _y/*=0*/, float _z/*=0*/) { allocate(_x, _y, _z); }
    private native void allocate(float _x/*=0*/, float _y/*=0*/, float _z/*=0*/);
    public CvPoint3D32f() { allocate(); }
    private native void allocate();
// #endif
}


public static native @ByVal CvPoint3D32f cvPoint3D32f( double x, double y, double z );


public static class CvPoint2D64f extends AbstractCvPoint2D64f {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvPoint2D64f() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvPoint2D64f(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvPoint2D64f(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvPoint2D64f position(int position) {
        return (CvPoint2D64f)super.position(position);
    }

    public native double x(); public native CvPoint2D64f x(double x);
    public native double y(); public native CvPoint2D64f y(double y);
}


public static native @ByVal CvPoint2D64f cvPoint2D64f( double x, double y );


public static class CvPoint3D64f extends AbstractCvPoint3D64f {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvPoint3D64f() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvPoint3D64f(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvPoint3D64f(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvPoint3D64f position(int position) {
        return (CvPoint3D64f)super.position(position);
    }

    public native double x(); public native CvPoint3D64f x(double x);
    public native double y(); public native CvPoint3D64f y(double y);
    public native double z(); public native CvPoint3D64f z(double z);
}


public static native @ByVal CvPoint3D64f cvPoint3D64f( double x, double y, double z );


/******************************** CvSize's & CvBox **************************************/

@NoOffset public static class CvSize extends AbstractCvSize {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvSize(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvSize(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public CvSize position(int position) {
        return (CvSize)super.position(position);
    }

    public native int width(); public native CvSize width(int width);
    public native int height(); public native CvSize height(int height);

// #ifdef __cplusplus
    public CvSize(int w/*=0*/, int h/*=0*/) { allocate(w, h); }
    private native void allocate(int w/*=0*/, int h/*=0*/);
    public CvSize() { allocate(); }
    private native void allocate();
// #endif
}

public static native @ByVal CvSize cvSize( int width, int height );

@NoOffset public static class CvSize2D32f extends AbstractCvSize2D32f {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvSize2D32f(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvSize2D32f(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public CvSize2D32f position(int position) {
        return (CvSize2D32f)super.position(position);
    }

    public native float width(); public native CvSize2D32f width(float width);
    public native float height(); public native CvSize2D32f height(float height);

// #ifdef __cplusplus
    public CvSize2D32f(float w/*=0*/, float h/*=0*/) { allocate(w, h); }
    private native void allocate(float w/*=0*/, float h/*=0*/);
    public CvSize2D32f() { allocate(); }
    private native void allocate();
// #endif
}


public static native @ByVal CvSize2D32f cvSize2D32f( double width, double height );

@NoOffset public static class CvBox2D extends AbstractCvBox2D {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvBox2D(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvBox2D(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public CvBox2D position(int position) {
        return (CvBox2D)super.position(position);
    }

    public native @ByRef CvPoint2D32f center(); public native CvBox2D center(CvPoint2D32f center);  /* Center of the box.                          */
    public native @ByRef CvSize2D32f size(); public native CvBox2D size(CvSize2D32f size);    /* Box width and length.                       */
    public native float angle(); public native CvBox2D angle(float angle);          /* Angle between the horizontal axis           */
                          /* and the first side (i.e. length) in degrees */

// #ifdef __cplusplus
    public CvBox2D(@ByVal CvPoint2D32f c/*=CvPoint2D32f()*/, @ByVal CvSize2D32f s/*=CvSize2D32f()*/, float a/*=0*/) { allocate(c, s, a); }
    private native void allocate(@ByVal CvPoint2D32f c/*=CvPoint2D32f()*/, @ByVal CvSize2D32f s/*=CvSize2D32f()*/, float a/*=0*/);
    public CvBox2D() { allocate(); }
    private native void allocate();
    public CvBox2D(@ByVal @Cast("CvPoint2D32f*") FloatBuffer c/*=CvPoint2D32f()*/, @ByVal CvSize2D32f s/*=CvSize2D32f()*/, float a/*=0*/) { allocate(c, s, a); }
    private native void allocate(@ByVal @Cast("CvPoint2D32f*") FloatBuffer c/*=CvPoint2D32f()*/, @ByVal CvSize2D32f s/*=CvSize2D32f()*/, float a/*=0*/);
    public CvBox2D(@ByVal @Cast("CvPoint2D32f*") float[] c/*=CvPoint2D32f()*/, @ByVal CvSize2D32f s/*=CvSize2D32f()*/, float a/*=0*/) { allocate(c, s, a); }
    private native void allocate(@ByVal @Cast("CvPoint2D32f*") float[] c/*=CvPoint2D32f()*/, @ByVal CvSize2D32f s/*=CvSize2D32f()*/, float a/*=0*/);
    public CvBox2D(@Const @ByRef RotatedRect rr) { allocate(rr); }
    private native void allocate(@Const @ByRef RotatedRect rr);
    public native @ByVal @Name("operator cv::RotatedRect") RotatedRect asRotatedRect();
// #endif
}


/* Line iterator state: */
public static class CvLineIterator extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvLineIterator() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvLineIterator(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvLineIterator(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvLineIterator position(int position) {
        return (CvLineIterator)super.position(position);
    }

    /* Pointer to the current point: */
    public native @Cast("uchar*") BytePointer ptr(); public native CvLineIterator ptr(BytePointer ptr);

    /* Bresenham algorithm state: */
    public native int err(); public native CvLineIterator err(int err);
    public native int plus_delta(); public native CvLineIterator plus_delta(int plus_delta);
    public native int minus_delta(); public native CvLineIterator minus_delta(int minus_delta);
    public native int plus_step(); public native CvLineIterator plus_step(int plus_step);
    public native int minus_step(); public native CvLineIterator minus_step(int minus_step);
}



/************************************* CvSlice ******************************************/
public static final int CV_WHOLE_SEQ_END_INDEX = 0x3fffffff;
public static final CvSlice CV_WHOLE_SEQ =  cvSlice(0, CV_WHOLE_SEQ_END_INDEX);

@NoOffset public static class CvSlice extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvSlice(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvSlice(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public CvSlice position(int position) {
        return (CvSlice)super.position(position);
    }

    public native int start_index(); public native CvSlice start_index(int start_index);
    public native int end_index(); public native CvSlice end_index(int end_index);

// #if defined(__cplusplus) && !defined(__CUDACC__)
    public CvSlice(int start/*=0*/, int end/*=0*/) { allocate(start, end); }
    private native void allocate(int start/*=0*/, int end/*=0*/);
    public CvSlice() { allocate(); }
    private native void allocate();
    public CvSlice(@Const @ByRef Range r) { allocate(r); }
    private native void allocate(@Const @ByRef Range r);
    public native @ByVal @Name("operator cv::Range") Range asRange();
// #endif
}

public static native @ByVal CvSlice cvSlice( int start, int end );



/************************************* CvScalar *****************************************/

@NoOffset public static class CvScalar extends AbstractCvScalar {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvScalar(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvScalar(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public CvScalar position(int position) {
        return (CvScalar)super.position(position);
    }

    public native double val(int i); public native CvScalar val(int i, double val);
    @MemberGetter public native DoublePointer val();

// #ifdef __cplusplus
    public CvScalar() { allocate(); }
    private native void allocate();
    public CvScalar(double d0, double d1/*=0*/, double d2/*=0*/, double d3/*=0*/) { allocate(d0, d1, d2, d3); }
    private native void allocate(double d0, double d1/*=0*/, double d2/*=0*/, double d3/*=0*/);
    public CvScalar(double d0) { allocate(d0); }
    private native void allocate(double d0);
// #endif
}

public static native @ByVal CvScalar cvScalar( double val0, double val1/*=0*/,
                               double val2/*=0*/, double val3/*=0*/);
public static native @ByVal CvScalar cvScalar( double val0);


public static native @ByVal CvScalar cvRealScalar( double val0 );

public static native @ByVal CvScalar cvScalarAll( double val0123 );

/****************************************************************************************\
*                                   Dynamic Data structures                              *
\****************************************************************************************/

/******************************** Memory storage ****************************************/

public static class CvMemBlock extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvMemBlock() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvMemBlock(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvMemBlock(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvMemBlock position(int position) {
        return (CvMemBlock)super.position(position);
    }

    public native CvMemBlock prev(); public native CvMemBlock prev(CvMemBlock prev);
    public native CvMemBlock next(); public native CvMemBlock next(CvMemBlock next);
}

public static final int CV_STORAGE_MAGIC_VAL =    0x42890000;

public static class CvMemStorage extends AbstractCvMemStorage {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvMemStorage() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvMemStorage(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvMemStorage(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvMemStorage position(int position) {
        return (CvMemStorage)super.position(position);
    }

    public native int signature(); public native CvMemStorage signature(int signature);
    public native CvMemBlock bottom(); public native CvMemStorage bottom(CvMemBlock bottom);           /* First allocated block.                   */
    public native CvMemBlock top(); public native CvMemStorage top(CvMemBlock top);              /* Current memory block - top of the stack. */
    public native CvMemStorage parent(); public native CvMemStorage parent(CvMemStorage parent); /* We get new blocks from parent as needed. */
    public native int block_size(); public native CvMemStorage block_size(int block_size);               /* Block size.                              */
    public native int free_space(); public native CvMemStorage free_space(int free_space);               /* Remaining free space in current block.   */
}

// #define CV_IS_STORAGE(storage)
//     ((storage) != NULL &&
//     (((CvMemStorage*)(storage))->signature & CV_MAGIC_MASK) == CV_STORAGE_MAGIC_VAL)


public static class CvMemStoragePos extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvMemStoragePos() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvMemStoragePos(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvMemStoragePos(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvMemStoragePos position(int position) {
        return (CvMemStoragePos)super.position(position);
    }

    public native CvMemBlock top(); public native CvMemStoragePos top(CvMemBlock top);
    public native int free_space(); public native CvMemStoragePos free_space(int free_space);
}


/*********************************** Sequence *******************************************/

public static class CvSeqBlock extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvSeqBlock() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvSeqBlock(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvSeqBlock(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvSeqBlock position(int position) {
        return (CvSeqBlock)super.position(position);
    }

    public native CvSeqBlock prev(); public native CvSeqBlock prev(CvSeqBlock prev); /* Previous sequence block.                   */
    public native CvSeqBlock next(); public native CvSeqBlock next(CvSeqBlock next); /* Next sequence block.                       */
  public native int start_index(); public native CvSeqBlock start_index(int start_index);         /* Index of the first element in the block +  */
                              /* sequence->first->start_index.              */
    public native int count(); public native CvSeqBlock count(int count);             /* Number of elements in the block.           */
    public native @Cast("schar*") BytePointer data(); public native CvSeqBlock data(BytePointer data);              /* Pointer to the first element of the block. */
}


// #define CV_TREE_NODE_FIELDS(node_type)
//     int       flags;             /* Miscellaneous flags.     */
//     int       header_size;       /* Size of sequence header. */
//     struct    node_type* h_prev; /* Previous sequence.       */
//     struct    node_type* h_next; /* Next sequence.           */
//     struct    node_type* v_prev; /* 2nd previous sequence.   */
//     struct    node_type* v_next  /* 2nd next sequence.       */

/*
   Read/Write sequence.
   Elements can be dynamically inserted to or deleted from the sequence.
*/
// #define CV_SEQUENCE_FIELDS()
//     CV_TREE_NODE_FIELDS(CvSeq);
//     int       total;          /* Total number of elements.            */
//     int       elem_size;      /* Size of sequence element in bytes.   */
//     schar*    block_max;      /* Maximal bound of the last block.     */
//     schar*    ptr;            /* Current write pointer.               */
//     int       delta_elems;    /* Grow seq this many at a time.        */
//     CvMemStorage* storage;    /* Where the seq is stored.             */
//     CvSeqBlock* free_blocks;  /* Free blocks list.                    */
//     CvSeqBlock* first;        /* Pointer to the first sequence block. */

public static class CvSeq extends AbstractCvSeq {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvSeq() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvSeq(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvSeq(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvSeq position(int position) {
        return (CvSeq)super.position(position);
    }

    public native int flags(); public native CvSeq flags(int flags);             /* Miscellaneous flags.     */
    public native int header_size(); public native CvSeq header_size(int header_size);       /* Size of sequence header. */
    public native CvSeq h_prev(); public native CvSeq h_prev(CvSeq h_prev); /* Previous sequence.       */
    public native CvSeq h_next(); public native CvSeq h_next(CvSeq h_next); /* Next sequence.           */
    public native CvSeq v_prev(); public native CvSeq v_prev(CvSeq v_prev); /* 2nd previous sequence.   */
    public native CvSeq v_next(); public native CvSeq v_next(CvSeq v_next);
    public native int total(); public native CvSeq total(int total);          /* Total number of elements.            */
    public native int elem_size(); public native CvSeq elem_size(int elem_size);      /* Size of sequence element in bytes.   */
    public native @Cast("schar*") BytePointer block_max(); public native CvSeq block_max(BytePointer block_max);      /* Maximal bound of the last block.     */
    public native @Cast("schar*") BytePointer ptr(); public native CvSeq ptr(BytePointer ptr);            /* Current write pointer.               */
    public native int delta_elems(); public native CvSeq delta_elems(int delta_elems);    /* Grow seq this many at a time.        */
    public native CvMemStorage storage(); public native CvSeq storage(CvMemStorage storage);    /* Where the seq is stored.             */
    public native CvSeqBlock free_blocks(); public native CvSeq free_blocks(CvSeqBlock free_blocks);  /* Free blocks list.                    */
    public native CvSeqBlock first(); public native CvSeq first(CvSeqBlock first);        /* Pointer to the first sequence block. */
}

public static final String CV_TYPE_NAME_SEQ =             "opencv-sequence";
public static final String CV_TYPE_NAME_SEQ_TREE =        "opencv-sequence-tree";

/*************************************** Set ********************************************/
/*
  Set.
  Order is not preserved. There can be gaps between sequence elements.
  After the element has been inserted it stays in the same place all the time.
  The MSB(most-significant or sign bit) of the first field (flags) is 0 iff the element exists.
*/
// #define CV_SET_ELEM_FIELDS(elem_type)
//     int  flags;
//     struct elem_type* next_free;

public static class CvSetElem extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvSetElem() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvSetElem(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvSetElem(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvSetElem position(int position) {
        return (CvSetElem)super.position(position);
    }

    public native int flags(); public native CvSetElem flags(int flags);
    public native CvSetElem next_free(); public native CvSetElem next_free(CvSetElem next_free);
}

// #define CV_SET_FIELDS()
//     CV_SEQUENCE_FIELDS()
//     CvSetElem* free_elems;
//     int active_count;

public static class CvSet extends AbstractCvSet {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvSet() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvSet(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvSet(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvSet position(int position) {
        return (CvSet)super.position(position);
    }

    public native int flags(); public native CvSet flags(int flags);             /* Miscellaneous flags.     */
    public native int header_size(); public native CvSet header_size(int header_size);       /* Size of sequence header. */
    public native CvSeq h_prev(); public native CvSet h_prev(CvSeq h_prev); /* Previous sequence.       */
    public native CvSeq h_next(); public native CvSet h_next(CvSeq h_next); /* Next sequence.           */
    public native CvSeq v_prev(); public native CvSet v_prev(CvSeq v_prev); /* 2nd previous sequence.   */
    public native CvSeq v_next(); public native CvSet v_next(CvSeq v_next);
    public native int total(); public native CvSet total(int total);          /* Total number of elements.            */
    public native int elem_size(); public native CvSet elem_size(int elem_size);      /* Size of sequence element in bytes.   */
    public native @Cast("schar*") BytePointer block_max(); public native CvSet block_max(BytePointer block_max);      /* Maximal bound of the last block.     */
    public native @Cast("schar*") BytePointer ptr(); public native CvSet ptr(BytePointer ptr);            /* Current write pointer.               */
    public native int delta_elems(); public native CvSet delta_elems(int delta_elems);    /* Grow seq this many at a time.        */
    public native CvMemStorage storage(); public native CvSet storage(CvMemStorage storage);    /* Where the seq is stored.             */
    public native CvSeqBlock free_blocks(); public native CvSet free_blocks(CvSeqBlock free_blocks);  /* Free blocks list.                    */
    public native CvSeqBlock first(); public native CvSet first(CvSeqBlock first);        /* Pointer to the first sequence block. */
    public native CvSetElem free_elems(); public native CvSet free_elems(CvSetElem free_elems);
    public native int active_count(); public native CvSet active_count(int active_count);
}


public static final int CV_SET_ELEM_IDX_MASK =   ((1 << 26) - 1);
public static native @MemberGetter int CV_SET_ELEM_FREE_FLAG();
public static final int CV_SET_ELEM_FREE_FLAG = CV_SET_ELEM_FREE_FLAG();

/* Checks whether the element pointed by ptr belongs to a set or not */
// #define CV_IS_SET_ELEM( ptr )  (((CvSetElem*)(ptr))->flags >= 0)

/************************************* Graph ********************************************/

/*
  We represent a graph as a set of vertices.
  Vertices contain their adjacency lists (more exactly, pointers to first incoming or
  outcoming edge (or 0 if isolated vertex)). Edges are stored in another set.
  There is a singly-linked list of incoming/outcoming edges for each vertex.

  Each edge consists of

     o   Two pointers to the starting and ending vertices
         (vtx[0] and vtx[1] respectively).

   A graph may be oriented or not. In the latter case, edges between
   vertex i to vertex j are not distinguished during search operations.

     o   Two pointers to next edges for the starting and ending vertices, where
         next[0] points to the next edge in the vtx[0] adjacency list and
         next[1] points to the next edge in the vtx[1] adjacency list.
*/
// #define CV_GRAPH_EDGE_FIELDS()
//     int flags;
//     float weight;
//     struct CvGraphEdge* next[2];
//     struct CvGraphVtx* vtx[2];


// #define CV_GRAPH_VERTEX_FIELDS()
//     int flags;
//     struct CvGraphEdge* first;


public static class CvGraphEdge extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvGraphEdge() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvGraphEdge(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvGraphEdge(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvGraphEdge position(int position) {
        return (CvGraphEdge)super.position(position);
    }

    public native int flags(); public native CvGraphEdge flags(int flags);
    public native float weight(); public native CvGraphEdge weight(float weight);
    public native CvGraphEdge next(int i); public native CvGraphEdge next(int i, CvGraphEdge next);
    @MemberGetter public native @Cast("CvGraphEdge**") PointerPointer next();
    public native CvGraphVtx vtx(int i); public native CvGraphEdge vtx(int i, CvGraphVtx vtx);
    @MemberGetter public native @Cast("CvGraphVtx**") PointerPointer vtx();
}

public static class CvGraphVtx extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvGraphVtx() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvGraphVtx(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvGraphVtx(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvGraphVtx position(int position) {
        return (CvGraphVtx)super.position(position);
    }

    public native int flags(); public native CvGraphVtx flags(int flags);
    public native CvGraphEdge first(); public native CvGraphVtx first(CvGraphEdge first);
}

public static class CvGraphVtx2D extends CvGraphVtx {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvGraphVtx2D() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvGraphVtx2D(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvGraphVtx2D(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvGraphVtx2D position(int position) {
        return (CvGraphVtx2D)super.position(position);
    }

    public native int flags(); public native CvGraphVtx2D flags(int flags);
    public native CvGraphEdge first(); public native CvGraphVtx2D first(CvGraphEdge first);
    public native CvPoint2D32f ptr(); public native CvGraphVtx2D ptr(CvPoint2D32f ptr);
}

/*
   Graph is "derived" from the set (this is set a of vertices)
   and includes another set (edges)
*/
// #define  CV_GRAPH_FIELDS()
//     CV_SET_FIELDS()
//     CvSet* edges;

public static class CvGraph extends AbstractCvGraph {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvGraph() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvGraph(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvGraph(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvGraph position(int position) {
        return (CvGraph)super.position(position);
    }

    public native int flags(); public native CvGraph flags(int flags);             /* Miscellaneous flags.     */
    public native int header_size(); public native CvGraph header_size(int header_size);       /* Size of sequence header. */
    public native CvSeq h_prev(); public native CvGraph h_prev(CvSeq h_prev); /* Previous sequence.       */
    public native CvSeq h_next(); public native CvGraph h_next(CvSeq h_next); /* Next sequence.           */
    public native CvSeq v_prev(); public native CvGraph v_prev(CvSeq v_prev); /* 2nd previous sequence.   */
    public native CvSeq v_next(); public native CvGraph v_next(CvSeq v_next);
    public native int total(); public native CvGraph total(int total);          /* Total number of elements.            */
    public native int elem_size(); public native CvGraph elem_size(int elem_size);      /* Size of sequence element in bytes.   */
    public native @Cast("schar*") BytePointer block_max(); public native CvGraph block_max(BytePointer block_max);      /* Maximal bound of the last block.     */
    public native @Cast("schar*") BytePointer ptr(); public native CvGraph ptr(BytePointer ptr);            /* Current write pointer.               */
    public native int delta_elems(); public native CvGraph delta_elems(int delta_elems);    /* Grow seq this many at a time.        */
    public native CvMemStorage storage(); public native CvGraph storage(CvMemStorage storage);    /* Where the seq is stored.             */
    public native CvSeqBlock free_blocks(); public native CvGraph free_blocks(CvSeqBlock free_blocks);  /* Free blocks list.                    */
    public native CvSeqBlock first(); public native CvGraph first(CvSeqBlock first);        /* Pointer to the first sequence block. */
    public native CvSetElem free_elems(); public native CvGraph free_elems(CvSetElem free_elems);
    public native int active_count(); public native CvGraph active_count(int active_count);
    public native CvSet edges(); public native CvGraph edges(CvSet edges);
}

public static final String CV_TYPE_NAME_GRAPH = "opencv-graph";

/*********************************** Chain/Countour *************************************/

public static class CvChain extends CvSeq {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvChain() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvChain(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvChain(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvChain position(int position) {
        return (CvChain)super.position(position);
    }

    public native int flags(); public native CvChain flags(int flags);             /* Miscellaneous flags.     */
    public native int header_size(); public native CvChain header_size(int header_size);       /* Size of sequence header. */
    public native CvSeq h_prev(); public native CvChain h_prev(CvSeq h_prev); /* Previous sequence.       */
    public native CvSeq h_next(); public native CvChain h_next(CvSeq h_next); /* Next sequence.           */
    public native CvSeq v_prev(); public native CvChain v_prev(CvSeq v_prev); /* 2nd previous sequence.   */
    public native CvSeq v_next(); public native CvChain v_next(CvSeq v_next);
    public native int total(); public native CvChain total(int total);          /* Total number of elements.            */
    public native int elem_size(); public native CvChain elem_size(int elem_size);      /* Size of sequence element in bytes.   */
    public native @Cast("schar*") BytePointer block_max(); public native CvChain block_max(BytePointer block_max);      /* Maximal bound of the last block.     */
    public native @Cast("schar*") BytePointer ptr(); public native CvChain ptr(BytePointer ptr);            /* Current write pointer.               */
    public native int delta_elems(); public native CvChain delta_elems(int delta_elems);    /* Grow seq this many at a time.        */
    public native CvMemStorage storage(); public native CvChain storage(CvMemStorage storage);    /* Where the seq is stored.             */
    public native CvSeqBlock free_blocks(); public native CvChain free_blocks(CvSeqBlock free_blocks);  /* Free blocks list.                    */
    public native CvSeqBlock first(); public native CvChain first(CvSeqBlock first);        /* Pointer to the first sequence block. */
    public native @ByRef CvPoint origin(); public native CvChain origin(CvPoint origin);
}

// #define CV_CONTOUR_FIELDS()
//     CV_SEQUENCE_FIELDS()
//     CvRect rect;
//     int color;
//     int reserved[3];

public static class CvContour extends CvSeq {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvContour() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvContour(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvContour(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvContour position(int position) {
        return (CvContour)super.position(position);
    }

    public native int flags(); public native CvContour flags(int flags);             /* Miscellaneous flags.     */
    public native int header_size(); public native CvContour header_size(int header_size);       /* Size of sequence header. */
    public native CvSeq h_prev(); public native CvContour h_prev(CvSeq h_prev); /* Previous sequence.       */
    public native CvSeq h_next(); public native CvContour h_next(CvSeq h_next); /* Next sequence.           */
    public native CvSeq v_prev(); public native CvContour v_prev(CvSeq v_prev); /* 2nd previous sequence.   */
    public native CvSeq v_next(); public native CvContour v_next(CvSeq v_next);
    public native int total(); public native CvContour total(int total);          /* Total number of elements.            */
    public native int elem_size(); public native CvContour elem_size(int elem_size);      /* Size of sequence element in bytes.   */
    public native @Cast("schar*") BytePointer block_max(); public native CvContour block_max(BytePointer block_max);      /* Maximal bound of the last block.     */
    public native @Cast("schar*") BytePointer ptr(); public native CvContour ptr(BytePointer ptr);            /* Current write pointer.               */
    public native int delta_elems(); public native CvContour delta_elems(int delta_elems);    /* Grow seq this many at a time.        */
    public native CvMemStorage storage(); public native CvContour storage(CvMemStorage storage);    /* Where the seq is stored.             */
    public native CvSeqBlock free_blocks(); public native CvContour free_blocks(CvSeqBlock free_blocks);  /* Free blocks list.                    */
    public native CvSeqBlock first(); public native CvContour first(CvSeqBlock first);        /* Pointer to the first sequence block. */
    public native @ByRef CvRect rect(); public native CvContour rect(CvRect rect);
    public native int color(); public native CvContour color(int color);
    public native int reserved(int i); public native CvContour reserved(int i, int reserved);
    @MemberGetter public native IntPointer reserved();
}

/****************************************************************************************\
*                                    Sequence types                                      *
\****************************************************************************************/

public static final int CV_SEQ_MAGIC_VAL =             0x42990000;

// #define CV_IS_SEQ(seq)
//     ((seq) != NULL && (((CvSeq*)(seq))->flags & CV_MAGIC_MASK) == CV_SEQ_MAGIC_VAL)

public static final int CV_SET_MAGIC_VAL =             0x42980000;
// #define CV_IS_SET(set)
//     ((set) != NULL && (((CvSeq*)(set))->flags & CV_MAGIC_MASK) == CV_SET_MAGIC_VAL)

public static final int CV_SEQ_ELTYPE_BITS =           12;
public static final int CV_SEQ_ELTYPE_MASK =           ((1 << CV_SEQ_ELTYPE_BITS) - 1);

public static final int CV_SEQ_ELTYPE_POINT =          CV_32SC2;  /* (x,y) */
public static final int CV_SEQ_ELTYPE_CODE =           CV_8UC1;   /* freeman code: 0..7 */
public static final int CV_SEQ_ELTYPE_GENERIC =        0;
public static final int CV_SEQ_ELTYPE_PTR =            CV_USRTYPE1;
public static final int CV_SEQ_ELTYPE_PPOINT =         CV_SEQ_ELTYPE_PTR;  /* &(x,y) */
public static final int CV_SEQ_ELTYPE_INDEX =          CV_32SC1;  /* #(x,y) */
public static final int CV_SEQ_ELTYPE_GRAPH_EDGE =     0;  /* &next_o, &next_d, &vtx_o, &vtx_d */
public static final int CV_SEQ_ELTYPE_GRAPH_VERTEX =   0;  /* first_edge, &(x,y) */
public static final int CV_SEQ_ELTYPE_TRIAN_ATR =      0;  /* vertex of the binary tree   */
public static final int CV_SEQ_ELTYPE_CONNECTED_COMP = 0;  /* connected component  */
public static final int CV_SEQ_ELTYPE_POINT3D =        CV_32FC3;  /* (x,y,z)  */

public static final int CV_SEQ_KIND_BITS =        2;
public static final int CV_SEQ_KIND_MASK =        (((1 << CV_SEQ_KIND_BITS) - 1)<<CV_SEQ_ELTYPE_BITS);

/* types of sequences */
public static final int CV_SEQ_KIND_GENERIC =     (0 << CV_SEQ_ELTYPE_BITS);
public static final int CV_SEQ_KIND_CURVE =       (1 << CV_SEQ_ELTYPE_BITS);
public static final int CV_SEQ_KIND_BIN_TREE =    (2 << CV_SEQ_ELTYPE_BITS);

/* types of sparse sequences (sets) */
public static final int CV_SEQ_KIND_GRAPH =       (1 << CV_SEQ_ELTYPE_BITS);
public static final int CV_SEQ_KIND_SUBDIV2D =    (2 << CV_SEQ_ELTYPE_BITS);

public static final int CV_SEQ_FLAG_SHIFT =       (CV_SEQ_KIND_BITS + CV_SEQ_ELTYPE_BITS);

/* flags for curves */
public static final int CV_SEQ_FLAG_CLOSED =     (1 << CV_SEQ_FLAG_SHIFT);
public static final int CV_SEQ_FLAG_SIMPLE =     (0 << CV_SEQ_FLAG_SHIFT);
public static final int CV_SEQ_FLAG_CONVEX =     (0 << CV_SEQ_FLAG_SHIFT);
public static final int CV_SEQ_FLAG_HOLE =       (2 << CV_SEQ_FLAG_SHIFT);

/* flags for graphs */
public static final int CV_GRAPH_FLAG_ORIENTED = (1 << CV_SEQ_FLAG_SHIFT);

public static final int CV_GRAPH =               CV_SEQ_KIND_GRAPH;
public static final int CV_ORIENTED_GRAPH =      (CV_SEQ_KIND_GRAPH|CV_GRAPH_FLAG_ORIENTED);

/* point sets */
public static final int CV_SEQ_POINT_SET =       (CV_SEQ_KIND_GENERIC| CV_SEQ_ELTYPE_POINT);
public static final int CV_SEQ_POINT3D_SET =     (CV_SEQ_KIND_GENERIC| CV_SEQ_ELTYPE_POINT3D);
public static final int CV_SEQ_POLYLINE =        (CV_SEQ_KIND_CURVE  | CV_SEQ_ELTYPE_POINT);
public static final int CV_SEQ_POLYGON =         (CV_SEQ_FLAG_CLOSED | CV_SEQ_POLYLINE );
public static final int CV_SEQ_CONTOUR =         CV_SEQ_POLYGON;
public static final int CV_SEQ_SIMPLE_POLYGON =  (CV_SEQ_FLAG_SIMPLE | CV_SEQ_POLYGON  );

/* chain-coded curves */
public static final int CV_SEQ_CHAIN =           (CV_SEQ_KIND_CURVE  | CV_SEQ_ELTYPE_CODE);
public static final int CV_SEQ_CHAIN_CONTOUR =   (CV_SEQ_FLAG_CLOSED | CV_SEQ_CHAIN);

/* binary tree for the contour */
public static final int CV_SEQ_POLYGON_TREE =    (CV_SEQ_KIND_BIN_TREE  | CV_SEQ_ELTYPE_TRIAN_ATR);

/* sequence of the connected components */
public static final int CV_SEQ_CONNECTED_COMP =  (CV_SEQ_KIND_GENERIC  | CV_SEQ_ELTYPE_CONNECTED_COMP);

/* sequence of the integer numbers */
public static final int CV_SEQ_INDEX =           (CV_SEQ_KIND_GENERIC  | CV_SEQ_ELTYPE_INDEX);

// #define CV_SEQ_ELTYPE( seq )   ((seq)->flags & CV_SEQ_ELTYPE_MASK)
// #define CV_SEQ_KIND( seq )     ((seq)->flags & CV_SEQ_KIND_MASK )

/* flag checking */
// #define CV_IS_SEQ_INDEX( seq )      ((CV_SEQ_ELTYPE(seq) == CV_SEQ_ELTYPE_INDEX) &&
//                                      (CV_SEQ_KIND(seq) == CV_SEQ_KIND_GENERIC))

// #define CV_IS_SEQ_CURVE( seq )      (CV_SEQ_KIND(seq) == CV_SEQ_KIND_CURVE)
// #define CV_IS_SEQ_CLOSED( seq )     (((seq)->flags & CV_SEQ_FLAG_CLOSED) != 0)
// #define CV_IS_SEQ_CONVEX( seq )     0
// #define CV_IS_SEQ_HOLE( seq )       (((seq)->flags & CV_SEQ_FLAG_HOLE) != 0)
// #define CV_IS_SEQ_SIMPLE( seq )     1

/* type checking macros */
// #define CV_IS_SEQ_POINT_SET( seq )
//     ((CV_SEQ_ELTYPE(seq) == CV_32SC2 || CV_SEQ_ELTYPE(seq) == CV_32FC2))

// #define CV_IS_SEQ_POINT_SUBSET( seq )
//     (CV_IS_SEQ_INDEX( seq ) || CV_SEQ_ELTYPE(seq) == CV_SEQ_ELTYPE_PPOINT)

// #define CV_IS_SEQ_POLYLINE( seq )
//     (CV_SEQ_KIND(seq) == CV_SEQ_KIND_CURVE && CV_IS_SEQ_POINT_SET(seq))

// #define CV_IS_SEQ_POLYGON( seq )
//     (CV_IS_SEQ_POLYLINE(seq) && CV_IS_SEQ_CLOSED(seq))

// #define CV_IS_SEQ_CHAIN( seq )
//     (CV_SEQ_KIND(seq) == CV_SEQ_KIND_CURVE && (seq)->elem_size == 1)

// #define CV_IS_SEQ_CONTOUR( seq )
//     (CV_IS_SEQ_CLOSED(seq) && (CV_IS_SEQ_POLYLINE(seq) || CV_IS_SEQ_CHAIN(seq)))

// #define CV_IS_SEQ_CHAIN_CONTOUR( seq )
//     (CV_IS_SEQ_CHAIN( seq ) && CV_IS_SEQ_CLOSED( seq ))

// #define CV_IS_SEQ_POLYGON_TREE( seq )
//     (CV_SEQ_ELTYPE (seq) ==  CV_SEQ_ELTYPE_TRIAN_ATR &&
//     CV_SEQ_KIND( seq ) ==  CV_SEQ_KIND_BIN_TREE )

// #define CV_IS_GRAPH( seq )
//     (CV_IS_SET(seq) && CV_SEQ_KIND((CvSet*)(seq)) == CV_SEQ_KIND_GRAPH)

// #define CV_IS_GRAPH_ORIENTED( seq )
//     (((seq)->flags & CV_GRAPH_FLAG_ORIENTED) != 0)

// #define CV_IS_SUBDIV2D( seq )
//     (CV_IS_SET(seq) && CV_SEQ_KIND((CvSet*)(seq)) == CV_SEQ_KIND_SUBDIV2D)

/****************************************************************************************/
/*                            Sequence writer & reader                                  */
/****************************************************************************************/

// #define CV_SEQ_WRITER_FIELDS()
//     int          header_size;
//     CvSeq*       seq;        /* the sequence written */
//     CvSeqBlock*  block;      /* current block */
//     schar*       ptr;        /* pointer to free space */
//     schar*       block_min;  /* pointer to the beginning of block*/
//     schar*       block_max;  /* pointer to the end of block */

public static class CvSeqWriter extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvSeqWriter() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvSeqWriter(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvSeqWriter(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvSeqWriter position(int position) {
        return (CvSeqWriter)super.position(position);
    }

    public native int header_size(); public native CvSeqWriter header_size(int header_size);
    public native CvSeq seq(); public native CvSeqWriter seq(CvSeq seq);        /* the sequence written */
    public native CvSeqBlock block(); public native CvSeqWriter block(CvSeqBlock block);      /* current block */
    public native @Cast("schar*") BytePointer ptr(); public native CvSeqWriter ptr(BytePointer ptr);        /* pointer to free space */
    public native @Cast("schar*") BytePointer block_min(); public native CvSeqWriter block_min(BytePointer block_min);  /* pointer to the beginning of block*/
    public native @Cast("schar*") BytePointer block_max(); public native CvSeqWriter block_max(BytePointer block_max);  /* pointer to the end of block */
}


// #define CV_SEQ_READER_FIELDS()
//     int          header_size;
//     CvSeq*       seq;        /* sequence, beign read */
//     CvSeqBlock*  block;      /* current block */
//     schar*       ptr;        /* pointer to element be read next */
//     schar*       block_min;  /* pointer to the beginning of block */
//     schar*       block_max;  /* pointer to the end of block */
//     int          delta_index;/* = seq->first->start_index   */
//     schar*       prev_elem;  /* pointer to previous element */

public static class CvSeqReader extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvSeqReader() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvSeqReader(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvSeqReader(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvSeqReader position(int position) {
        return (CvSeqReader)super.position(position);
    }

    public native int header_size(); public native CvSeqReader header_size(int header_size);
    public native CvSeq seq(); public native CvSeqReader seq(CvSeq seq);        /* sequence, beign read */
    public native CvSeqBlock block(); public native CvSeqReader block(CvSeqBlock block);      /* current block */
    public native @Cast("schar*") BytePointer ptr(); public native CvSeqReader ptr(BytePointer ptr);        /* pointer to element be read next */
    public native @Cast("schar*") BytePointer block_min(); public native CvSeqReader block_min(BytePointer block_min);  /* pointer to the beginning of block */
    public native @Cast("schar*") BytePointer block_max(); public native CvSeqReader block_max(BytePointer block_max);  /* pointer to the end of block */
    public native int delta_index(); public native CvSeqReader delta_index(int delta_index);/* = seq->first->start_index   */
    public native @Cast("schar*") BytePointer prev_elem(); public native CvSeqReader prev_elem(BytePointer prev_elem);  /* pointer to previous element */
}

/****************************************************************************************/
/*                                Operations on sequences                               */
/****************************************************************************************/

// #define  CV_SEQ_ELEM( seq, elem_type, index )
// /* assert gives some guarantee that <seq> parameter is valid */
// (   assert(sizeof((seq)->first[0]) == sizeof(CvSeqBlock) &&
//     (seq)->elem_size == sizeof(elem_type)),
//     (elem_type*)((seq)->first && (unsigned)index <
//     (unsigned)((seq)->first->count) ?
//     (seq)->first->data + (index) * sizeof(elem_type) :
//     cvGetSeqElem( (CvSeq*)(seq), (index) )))
// #define CV_GET_SEQ_ELEM( elem_type, seq, index ) CV_SEQ_ELEM( (seq), elem_type, (index) )

/* Add element to sequence: */
// #define CV_WRITE_SEQ_ELEM_VAR( elem_ptr, writer )
// {
//     if( (writer).ptr >= (writer).bock_max )
//     {
//         cvCreateSeqBlock( &writer);
//     }
//     memcpy((writer).ptr, elem_ptr, (writer).seq->elem_size);
//     (writer).ptr += (writer).seq->elem_size;
// }

// #define CV_WRITE_SEQ_ELEM( elem, writer )
// {
//     assert( (writer).seq->elem_size == sizeof(elem));
//     if( (writer).ptr >= (writer).bock_max )
//     {
//         cvCreateSeqBlock( &writer);
//     }
//     assert( (writer).ptr <= (writer).bock_max - sizeof(elem));
//     memcpy((writer).ptr, &(elem), sizeof(elem));
//     (writer).ptr += sizeof(elem);
// }


/* Move reader position forward: */
// #define CV_NEXT_SEQ_ELEM( elem_size, reader )
// {
//     if( ((reader).ptr += (elem_size)) >= (reader).bock_max )
//     {
//         cvChangeSeqBlock( &(reader), 1 );
//     }
// }


/* Move reader position backward: */
// #define CV_PREV_SEQ_ELEM( elem_size, reader )
// {
//     if( ((reader).ptr -= (elem_size)) < (reader).bock_min )
//     {
//         cvChangeSeqBlock( &(reader), -1 );
//     }
// }

/* Read element and move read position forward: */
// #define CV_READ_SEQ_ELEM( elem, reader )
// {
//     assert( (reader).seq->elem_size == sizeof(elem));
//     memcpy( &(elem), (reader).ptr, sizeof((elem)));
//     CV_NEXT_SEQ_ELEM( sizeof(elem), reader )
// }

/* Read element and move read position backward: */
// #define CV_REV_READ_SEQ_ELEM( elem, reader )
// {
//     assert( (reader).seq->elem_size == sizeof(elem));
//     memcpy(&(elem), (reader).ptr, sizeof((elem)));
//     CV_PREV_SEQ_ELEM( sizeof(elem), reader )
// }


// #define CV_READ_CHAIN_POINT( _pt, reader )
// {
//     (_pt) = (reader).pt;
//     if( (reader).ptr )
//     {
//         CV_READ_SEQ_ELEM( (reader).code, (reader));
//         assert( ((reader).code & ~7) == 0 );
//         (reader).pt.x += (reader).detas[(int)(reader).code][0];
//         (reader).pt.y += (reader).detas[(int)(reader).code][1];
//     }
// }

// #define CV_CURRENT_POINT( reader )  (*((CvPoint*)((reader).ptr)))
// #define CV_PREV_POINT( reader )     (*((CvPoint*)((reader).prev_elem)))

// #define CV_READ_EDGE( pt1, pt2, reader )
// {
//     assert( sizeof(pt1) == sizeof(CvPoint) &&
//             sizeof(pt2) == sizeof(CvPoint) &&
//             reader.seq->elem_size == sizeof(CvPoint));
//     (pt1) = CV_PREV_POINT( reader );
//     (pt2) = CV_CURRENT_POINT( reader );
//     (reader).prev_elem = (reader).ptr;
//     CV_NEXT_SEQ_ELEM( sizeof(CvPoint), (reader));
// }

/************ Graph macros ************/

/* Return next graph edge for given vertex: */
// #define  CV_NEXT_GRAPH_EDGE( edge, vertex )
//      (assert((edge)->vtx[0] == (vertex) || (edge)->vtx[1] == (vertex)),
//       (edge)->next[(edge)->vtx[1] == (vertex)])



/****************************************************************************************\
*             Data structures for persistence (a.k.a serialization) functionality        *
\****************************************************************************************/

/* "black box" file storage */
@Opaque public static class CvFileStorage extends AbstractCvFileStorage {
    /** Empty constructor. */
    public CvFileStorage() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvFileStorage(Pointer p) { super(p); }
}

/* Storage flags: */
public static final int CV_STORAGE_READ =          0;
public static final int CV_STORAGE_WRITE =         1;
public static final int CV_STORAGE_WRITE_TEXT =    CV_STORAGE_WRITE;
public static final int CV_STORAGE_WRITE_BINARY =  CV_STORAGE_WRITE;
public static final int CV_STORAGE_APPEND =        2;
public static final int CV_STORAGE_MEMORY =        4;
public static final int CV_STORAGE_FORMAT_MASK =   (7<<3);
public static final int CV_STORAGE_FORMAT_AUTO =   0;
public static final int CV_STORAGE_FORMAT_XML =    8;
public static final int CV_STORAGE_FORMAT_YAML =  16;

/* List of attributes: */
public static class CvAttrList extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvAttrList() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvAttrList(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvAttrList(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvAttrList position(int position) {
        return (CvAttrList)super.position(position);
    }

    @MemberGetter public native @Cast("const char*") BytePointer attr(int i);
    @MemberGetter public native @Cast("const char**") PointerPointer attr();         /* NULL-terminated array of (attribute_name,attribute_value) pairs. */
    public native CvAttrList next(); public native CvAttrList next(CvAttrList next);   /* Pointer to next chunk of the attributes list.                    */
}

public static native @ByVal CvAttrList cvAttrList( @Cast("const char**") PointerPointer attr/*=NULL*/,
                                 CvAttrList next/*=NULL*/ );
public static native @ByVal CvAttrList cvAttrList( );
public static native @ByVal CvAttrList cvAttrList( @Cast("const char**") @ByPtrPtr BytePointer attr/*=NULL*/,
                                 CvAttrList next/*=NULL*/ );
public static native @ByVal CvAttrList cvAttrList( @Cast("const char**") @ByPtrPtr ByteBuffer attr/*=NULL*/,
                                 CvAttrList next/*=NULL*/ );
public static native @ByVal CvAttrList cvAttrList( @Cast("const char**") @ByPtrPtr byte[] attr/*=NULL*/,
                                 CvAttrList next/*=NULL*/ );

public static final int CV_NODE_NONE =        0;
public static final int CV_NODE_INT =         1;
public static final int CV_NODE_INTEGER =     CV_NODE_INT;
public static final int CV_NODE_REAL =        2;
public static final int CV_NODE_FLOAT =       CV_NODE_REAL;
public static final int CV_NODE_STR =         3;
public static final int CV_NODE_STRING =      CV_NODE_STR;
public static final int CV_NODE_REF =         4; /* not used */
public static final int CV_NODE_SEQ =         5;
public static final int CV_NODE_MAP =         6;
public static final int CV_NODE_TYPE_MASK =   7;

// #define CV_NODE_TYPE(flags)  ((flags) & CV_NODE_TYPE_MASK)

/* file node flags */
public static final int CV_NODE_FLOW =        8; /* Used only for writing structures in YAML format. */
public static final int CV_NODE_USER =        16;
public static final int CV_NODE_EMPTY =       32;
public static final int CV_NODE_NAMED =       64;

// #define CV_NODE_IS_INT(flags)        (CV_NODE_TYPE(flags) == CV_NODE_INT)
// #define CV_NODE_IS_REAL(flags)       (CV_NODE_TYPE(flags) == CV_NODE_REAL)
// #define CV_NODE_IS_STRING(flags)     (CV_NODE_TYPE(flags) == CV_NODE_STRING)
// #define CV_NODE_IS_SEQ(flags)        (CV_NODE_TYPE(flags) == CV_NODE_SEQ)
// #define CV_NODE_IS_MAP(flags)        (CV_NODE_TYPE(flags) == CV_NODE_MAP)
// #define CV_NODE_IS_COLLECTION(flags) (CV_NODE_TYPE(flags) >= CV_NODE_SEQ)
// #define CV_NODE_IS_FLOW(flags)       (((flags) & CV_NODE_FLOW) != 0)
// #define CV_NODE_IS_EMPTY(flags)      (((flags) & CV_NODE_EMPTY) != 0)
// #define CV_NODE_IS_USER(flags)       (((flags) & CV_NODE_USER) != 0)
// #define CV_NODE_HAS_NAME(flags)      (((flags) & CV_NODE_NAMED) != 0)

public static final int CV_NODE_SEQ_SIMPLE = 256;
// #define CV_NODE_SEQ_IS_SIMPLE(seq) (((seq)->flags & CV_NODE_SEQ_SIMPLE) != 0)

public static class CvString extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvString() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvString(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvString(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvString position(int position) {
        return (CvString)super.position(position);
    }

    public native int len(); public native CvString len(int len);
    public native @Cast("char*") BytePointer ptr(); public native CvString ptr(BytePointer ptr);
}

/* All the keys (names) of elements in the readed file storage
   are stored in the hash to speed up the lookup operations: */
public static class CvStringHashNode extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvStringHashNode() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvStringHashNode(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvStringHashNode(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvStringHashNode position(int position) {
        return (CvStringHashNode)super.position(position);
    }

    public native @Cast("unsigned") int hashval(); public native CvStringHashNode hashval(int hashval);
    public native @ByRef CvString str(); public native CvStringHashNode str(CvString str);
    public native CvStringHashNode next(); public native CvStringHashNode next(CvStringHashNode next);
}

@Opaque public static class CvFileNodeHash extends Pointer {
    /** Empty constructor. */
    public CvFileNodeHash() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvFileNodeHash(Pointer p) { super(p); }
}

/* Basic element of the file storage - scalar or collection: */
public static class CvFileNode extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvFileNode() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvFileNode(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvFileNode(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvFileNode position(int position) {
        return (CvFileNode)super.position(position);
    }

    public native int tag(); public native CvFileNode tag(int tag);
    public native CvTypeInfo info(); public native CvFileNode info(CvTypeInfo info); /* type information
            (only for user-defined object, for others it is 0) */
        @Name("data.f") public native double data_f(); public native CvFileNode data_f(double data_f); /* scalar floating-point number */
        @Name("data.i") public native int data_i(); public native CvFileNode data_i(int data_i);    /* scalar integer number */
        @Name("data.str") public native @ByRef CvString data_str(); public native CvFileNode data_str(CvString data_str); /* text string */
        @Name("data.seq") public native CvSeq data_seq(); public native CvFileNode data_seq(CvSeq data_seq); /* sequence (ordered collection of file nodes) */
        @Name("data.map") public native CvFileNodeHash data_map(); public native CvFileNode data_map(CvFileNodeHash data_map); /* map (collection of named file nodes) */
}

// #ifdef __cplusplus
// #endif
@Convention("CV_CDECL") public static class CvIsInstanceFunc extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    CvIsInstanceFunc(Pointer p) { super(p); }
    protected CvIsInstanceFunc() { allocate(); }
    private native void allocate();
    public native int call( @Const Pointer struct_ptr );
}
@Convention("CV_CDECL") public static class CvReleaseFunc extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    CvReleaseFunc(Pointer p) { super(p); }
    protected CvReleaseFunc() { allocate(); }
    private native void allocate();
    public native void call( @Cast("void**") @ByPtrPtr Pointer struct_dblptr );
}
@Convention("CV_CDECL") public static class CvReadFunc extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    CvReadFunc(Pointer p) { super(p); }
    protected CvReadFunc() { allocate(); }
    private native void allocate();
    public native Pointer call( CvFileStorage storage, CvFileNode node );
}
@Convention("CV_CDECL") public static class CvWriteFunc extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    CvWriteFunc(Pointer p) { super(p); }
    protected CvWriteFunc() { allocate(); }
    private native void allocate();
    public native void call( CvFileStorage storage, @Cast("const char*") BytePointer name,
                                      @Const Pointer struct_ptr, @ByVal CvAttrList attributes );
}
@Convention("CV_CDECL") public static class CvCloneFunc extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    CvCloneFunc(Pointer p) { super(p); }
    protected CvCloneFunc() { allocate(); }
    private native void allocate();
    public native Pointer call( @Const Pointer struct_ptr );
}
// #ifdef __cplusplus
// #endif

public static class CvTypeInfo extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvTypeInfo() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvTypeInfo(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvTypeInfo(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvTypeInfo position(int position) {
        return (CvTypeInfo)super.position(position);
    }

    public native int flags(); public native CvTypeInfo flags(int flags);
    public native int header_size(); public native CvTypeInfo header_size(int header_size);
    public native CvTypeInfo prev(); public native CvTypeInfo prev(CvTypeInfo prev);
    public native CvTypeInfo next(); public native CvTypeInfo next(CvTypeInfo next);
    @MemberGetter public native @Cast("const char*") BytePointer type_name();
    public native CvIsInstanceFunc is_instance(); public native CvTypeInfo is_instance(CvIsInstanceFunc is_instance);
    public native CvReleaseFunc release(); public native CvTypeInfo release(CvReleaseFunc release);
    public native CvReadFunc read(); public native CvTypeInfo read(CvReadFunc read);
    public native CvWriteFunc write(); public native CvTypeInfo write(CvWriteFunc write);
    public native CvCloneFunc clone(); public native CvTypeInfo clone(CvCloneFunc clone);
}


/**** System data types ******/

public static class CvPluginFuncInfo extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvPluginFuncInfo() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvPluginFuncInfo(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvPluginFuncInfo(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvPluginFuncInfo position(int position) {
        return (CvPluginFuncInfo)super.position(position);
    }

    public native Pointer func_addr(int i); public native CvPluginFuncInfo func_addr(int i, Pointer func_addr);
    @MemberGetter public native @Cast("void**") PointerPointer func_addr();
    public native Pointer default_func_addr(); public native CvPluginFuncInfo default_func_addr(Pointer default_func_addr);
    @MemberGetter public native @Cast("const char*") BytePointer func_names();
    public native int search_modules(); public native CvPluginFuncInfo search_modules(int search_modules);
    public native int loaded_from(); public native CvPluginFuncInfo loaded_from(int loaded_from);
}

public static class CvModuleInfo extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvModuleInfo() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvModuleInfo(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvModuleInfo(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvModuleInfo position(int position) {
        return (CvModuleInfo)super.position(position);
    }

    public native CvModuleInfo next(); public native CvModuleInfo next(CvModuleInfo next);
    @MemberGetter public native @Cast("const char*") BytePointer name();
    @MemberGetter public native @Cast("const char*") BytePointer version();
    public native CvPluginFuncInfo func_tab(); public native CvModuleInfo func_tab(CvPluginFuncInfo func_tab);
}

// #endif /*__OPENCV_CORE_TYPES_H__*/

/* End of file. */


// Parsed from <opencv2/core/core_c.h>

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


// #ifndef __OPENCV_CORE_C_H__
// #define __OPENCV_CORE_C_H__

// #include "opencv2/core/types_c.h"

// #ifdef __cplusplus
// #  ifdef _MSC_VER
/* disable warning C4190: 'function' has C-linkage specified, but returns UDT 'typename'
                          which is incompatible with C

   It is OK to disable it because we only extend few plain structures with
   C++ construrtors for simpler interoperability with C++ API of the library
*/
// #    pragma warning(disable:4190)
// #  elif defined __clang__ && __clang_major__ >= 3
// #    pragma GCC diagnostic ignored "-Wreturn-type-c-linkage"
// #  endif
// #endif

// #ifdef __cplusplus
// #endif

/****************************************************************************************\
*          Array allocation, deallocation, initialization and access to elements         *
\****************************************************************************************/

/* <malloc> wrapper.
   If there is no enough memory, the function
   (as well as other OpenCV functions that call cvAlloc)
   raises an error. */
public static native Pointer cvAlloc( @Cast("size_t") long size );

/* <free> wrapper.
   Here and further all the memory releasing functions
   (that all call cvFree) take double pointer in order to
   to clear pointer to the data after releasing it.
   Passing pointer to NULL pointer is Ok: nothing happens in this case
*/
public static native void cvFree_( Pointer ptr );
// #define cvFree(ptr) (cvFree_(*(ptr)), *(ptr)=0)

/* Allocates and initializes IplImage header */
public static native IplImage cvCreateImageHeader( @ByVal CvSize size, int depth, int channels );

/* Inializes IplImage header */
public static native IplImage cvInitImageHeader( IplImage image, @ByVal CvSize size, int depth,
                                   int channels, int origin/*=0*/,
                                   int align/*=4*/);
public static native IplImage cvInitImageHeader( IplImage image, @ByVal CvSize size, int depth,
                                   int channels);

/* Creates IPL image (header and data) */
public static native IplImage cvCreateImage( @ByVal CvSize size, int depth, int channels );

/* Releases (i.e. deallocates) IPL image header */
public static native void cvReleaseImageHeader( @Cast("IplImage**") PointerPointer image );
public static native void cvReleaseImageHeader( @ByPtrPtr IplImage image );

/* Releases IPL image header and data */
public static native void cvReleaseImage( @Cast("IplImage**") PointerPointer image );
public static native void cvReleaseImage( @ByPtrPtr IplImage image );

/* Creates a copy of IPL image (widthStep may differ) */
public static native IplImage cvCloneImage( @Const IplImage image );

/* Sets a Channel Of Interest (only a few functions support COI) -
   use cvCopy to extract the selected channel and/or put it back */
public static native void cvSetImageCOI( IplImage image, int coi );

/* Retrieves image Channel Of Interest */
public static native int cvGetImageCOI( @Const IplImage image );

/* Sets image ROI (region of interest) (COI is not changed) */
public static native void cvSetImageROI( IplImage image, @ByVal CvRect rect );

/* Resets image ROI and COI */
public static native void cvResetImageROI( IplImage image );

/* Retrieves image ROI */
public static native @ByVal CvRect cvGetImageROI( @Const IplImage image );

/* Allocates and initializes CvMat header */
public static native CvMat cvCreateMatHeader( int rows, int cols, int type );

public static final int CV_AUTOSTEP =  0x7fffffff;

/* Initializes CvMat header */
public static native CvMat cvInitMatHeader( CvMat mat, int rows, int cols,
                              int type, Pointer data/*=NULL*/,
                              int step/*=CV_AUTOSTEP*/ );
public static native CvMat cvInitMatHeader( CvMat mat, int rows, int cols,
                              int type );

/* Allocates and initializes CvMat header and allocates data */
public static native CvMat cvCreateMat( int rows, int cols, int type );

/* Releases CvMat header and deallocates matrix data
   (reference counting is used for data) */
public static native void cvReleaseMat( @Cast("CvMat**") PointerPointer mat );
public static native void cvReleaseMat( @ByPtrPtr CvMat mat );

/* Decrements CvMat data reference counter and deallocates the data if
   it reaches 0 */
public static native void cvDecRefData( CvArr arr );

/* Increments CvMat data reference counter */
public static native int cvIncRefData( CvArr arr );


/* Creates an exact copy of the input matrix (except, may be, step value) */
public static native CvMat cvCloneMat( @Const CvMat mat );


/* Makes a new matrix from <rect> subrectangle of input array.
   No data is copied */
public static native CvMat cvGetSubRect( @Const CvArr arr, CvMat submat, @ByVal CvRect rect );
public static native CvMat cvGetSubArr(CvArr arg1, CvMat arg2, @ByVal CvRect arg3);

/* Selects row span of the input array: arr(start_row:delta_row:end_row,:)
    (end_row is not included into the span). */
public static native CvMat cvGetRows( @Const CvArr arr, CvMat submat,
                        int start_row, int end_row,
                        int delta_row/*=1*/);
public static native CvMat cvGetRows( @Const CvArr arr, CvMat submat,
                        int start_row, int end_row);

public static native CvMat cvGetRow( @Const CvArr arr, CvMat submat, int row );


/* Selects column span of the input array: arr(:,start_col:end_col)
   (end_col is not included into the span) */
public static native CvMat cvGetCols( @Const CvArr arr, CvMat submat,
                        int start_col, int end_col );

public static native CvMat cvGetCol( @Const CvArr arr, CvMat submat, int col );

/* Select a diagonal of the input array.
   (diag = 0 means the main diagonal, >0 means a diagonal above the main one,
   <0 - below the main one).
   The diagonal will be represented as a column (nx1 matrix). */
public static native CvMat cvGetDiag( @Const CvArr arr, CvMat submat,
                            int diag/*=0*/);
public static native CvMat cvGetDiag( @Const CvArr arr, CvMat submat);

/* low-level scalar <-> raw data conversion functions */
public static native void cvScalarToRawData( @Const CvScalar scalar, Pointer data, int type,
                              int extend_to_12/*=0*/ );
public static native void cvScalarToRawData( @Const CvScalar scalar, Pointer data, int type );

public static native void cvRawDataToScalar( @Const Pointer data, int type, CvScalar scalar );

/* Allocates and initializes CvMatND header */
public static native CvMatND cvCreateMatNDHeader( int dims, @Const IntPointer sizes, int type );
public static native CvMatND cvCreateMatNDHeader( int dims, @Const IntBuffer sizes, int type );
public static native CvMatND cvCreateMatNDHeader( int dims, @Const int[] sizes, int type );

/* Allocates and initializes CvMatND header and allocates data */
public static native CvMatND cvCreateMatND( int dims, @Const IntPointer sizes, int type );
public static native CvMatND cvCreateMatND( int dims, @Const IntBuffer sizes, int type );
public static native CvMatND cvCreateMatND( int dims, @Const int[] sizes, int type );

/* Initializes preallocated CvMatND header */
public static native CvMatND cvInitMatNDHeader( CvMatND mat, int dims, @Const IntPointer sizes,
                                    int type, Pointer data/*=NULL*/ );
public static native CvMatND cvInitMatNDHeader( CvMatND mat, int dims, @Const IntPointer sizes,
                                    int type );
public static native CvMatND cvInitMatNDHeader( CvMatND mat, int dims, @Const IntBuffer sizes,
                                    int type, Pointer data/*=NULL*/ );
public static native CvMatND cvInitMatNDHeader( CvMatND mat, int dims, @Const IntBuffer sizes,
                                    int type );
public static native CvMatND cvInitMatNDHeader( CvMatND mat, int dims, @Const int[] sizes,
                                    int type, Pointer data/*=NULL*/ );
public static native CvMatND cvInitMatNDHeader( CvMatND mat, int dims, @Const int[] sizes,
                                    int type );

/* Releases CvMatND */
public static native void cvReleaseMatND( @Cast("CvMatND**") PointerPointer mat );
public static native void cvReleaseMatND( @ByPtrPtr CvMatND mat );

/* Creates a copy of CvMatND (except, may be, steps) */
public static native CvMatND cvCloneMatND( @Const CvMatND mat );

/* Allocates and initializes CvSparseMat header and allocates data */
public static native CvSparseMat cvCreateSparseMat( int dims, @Const IntPointer sizes, int type );
public static native CvSparseMat cvCreateSparseMat( int dims, @Const IntBuffer sizes, int type );
public static native CvSparseMat cvCreateSparseMat( int dims, @Const int[] sizes, int type );

/* Releases CvSparseMat */
public static native void cvReleaseSparseMat( @Cast("CvSparseMat**") PointerPointer mat );
public static native void cvReleaseSparseMat( @ByPtrPtr CvSparseMat mat );

/* Creates a copy of CvSparseMat (except, may be, zero items) */
public static native CvSparseMat cvCloneSparseMat( @Const CvSparseMat mat );

/* Initializes sparse array iterator
   (returns the first node or NULL if the array is empty) */
public static native CvSparseNode cvInitSparseMatIterator( @Const CvSparseMat mat,
                                              CvSparseMatIterator mat_iterator );

// returns next sparse array node (or NULL if there is no more nodes)
public static native CvSparseNode cvGetNextSparseNode( CvSparseMatIterator mat_iterator );

/**************** matrix iterator: used for n-ary operations on dense arrays *********/

public static final int CV_MAX_ARR = 10;

public static class CvNArrayIterator extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvNArrayIterator() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvNArrayIterator(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvNArrayIterator(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvNArrayIterator position(int position) {
        return (CvNArrayIterator)super.position(position);
    }

    public native int count(); public native CvNArrayIterator count(int count); /* number of arrays */
    public native int dims(); public native CvNArrayIterator dims(int dims); /* number of dimensions to iterate */
    public native @ByRef CvSize size(); public native CvNArrayIterator size(CvSize size); /* maximal common linear size: { width = size, height = 1 } */
    public native @Cast("uchar*") BytePointer ptr(int i); public native CvNArrayIterator ptr(int i, BytePointer ptr);
    @MemberGetter public native @Cast("uchar**") PointerPointer ptr(); /* pointers to the array slices */
    public native int stack(int i); public native CvNArrayIterator stack(int i, int stack);
    @MemberGetter public native IntPointer stack(); /* for internal use */
    public native CvMatND hdr(int i); public native CvNArrayIterator hdr(int i, CvMatND hdr);
    @MemberGetter public native @Cast("CvMatND**") PointerPointer hdr(); /* pointers to the headers of the
                                 matrices that are processed */
}

public static final int CV_NO_DEPTH_CHECK =     1;
public static final int CV_NO_CN_CHECK =        2;
public static final int CV_NO_SIZE_CHECK =      4;

/* initializes iterator that traverses through several arrays simulteneously
   (the function together with cvNextArraySlice is used for
    N-ari element-wise operations) */
public static native int cvInitNArrayIterator( int count, @Cast("CvArr**") PointerPointer arrs,
                                 @Const CvArr mask, CvMatND stubs,
                                 CvNArrayIterator array_iterator,
                                 int flags/*=0*/ );
public static native int cvInitNArrayIterator( int count, @ByPtrPtr CvArr arrs,
                                 @Const CvArr mask, CvMatND stubs,
                                 CvNArrayIterator array_iterator );
public static native int cvInitNArrayIterator( int count, @ByPtrPtr CvArr arrs,
                                 @Const CvArr mask, CvMatND stubs,
                                 CvNArrayIterator array_iterator,
                                 int flags/*=0*/ );

/* returns zero value if iteration is finished, non-zero (slice length) otherwise */
public static native int cvNextNArraySlice( CvNArrayIterator array_iterator );


/* Returns type of array elements:
   CV_8UC1 ... CV_64FC4 ... */
public static native int cvGetElemType( @Const CvArr arr );

/* Retrieves number of an array dimensions and
   optionally sizes of the dimensions */
public static native int cvGetDims( @Const CvArr arr, IntPointer sizes/*=NULL*/ );
public static native int cvGetDims( @Const CvArr arr );
public static native int cvGetDims( @Const CvArr arr, IntBuffer sizes/*=NULL*/ );
public static native int cvGetDims( @Const CvArr arr, int[] sizes/*=NULL*/ );


/* Retrieves size of a particular array dimension.
   For 2d arrays cvGetDimSize(arr,0) returns number of rows (image height)
   and cvGetDimSize(arr,1) returns number of columns (image width) */
public static native int cvGetDimSize( @Const CvArr arr, int index );


/* ptr = &arr(idx0,idx1,...). All indexes are zero-based,
   the major dimensions go first (e.g. (y,x) for 2D, (z,y,x) for 3D */
public static native @Cast("uchar*") BytePointer cvPtr1D( @Const CvArr arr, int idx0, IntPointer type/*=NULL*/);
public static native @Cast("uchar*") BytePointer cvPtr1D( @Const CvArr arr, int idx0);
public static native @Cast("uchar*") ByteBuffer cvPtr1D( @Const CvArr arr, int idx0, IntBuffer type/*=NULL*/);
public static native @Cast("uchar*") byte[] cvPtr1D( @Const CvArr arr, int idx0, int[] type/*=NULL*/);
public static native @Cast("uchar*") BytePointer cvPtr2D( @Const CvArr arr, int idx0, int idx1, IntPointer type/*=NULL*/ );
public static native @Cast("uchar*") BytePointer cvPtr2D( @Const CvArr arr, int idx0, int idx1 );
public static native @Cast("uchar*") ByteBuffer cvPtr2D( @Const CvArr arr, int idx0, int idx1, IntBuffer type/*=NULL*/ );
public static native @Cast("uchar*") byte[] cvPtr2D( @Const CvArr arr, int idx0, int idx1, int[] type/*=NULL*/ );
public static native @Cast("uchar*") BytePointer cvPtr3D( @Const CvArr arr, int idx0, int idx1, int idx2,
                      IntPointer type/*=NULL*/);
public static native @Cast("uchar*") BytePointer cvPtr3D( @Const CvArr arr, int idx0, int idx1, int idx2);
public static native @Cast("uchar*") ByteBuffer cvPtr3D( @Const CvArr arr, int idx0, int idx1, int idx2,
                      IntBuffer type/*=NULL*/);
public static native @Cast("uchar*") byte[] cvPtr3D( @Const CvArr arr, int idx0, int idx1, int idx2,
                      int[] type/*=NULL*/);

/* For CvMat or IplImage number of indices should be 2
   (row index (y) goes first, column index (x) goes next).
   For CvMatND or CvSparseMat number of infices should match number of <dims> and
   indices order should match the array dimension order. */
public static native @Cast("uchar*") BytePointer cvPtrND( @Const CvArr arr, @Const IntPointer idx, IntPointer type/*=NULL*/,
                      int create_node/*=1*/,
                      @Cast("unsigned*") IntPointer precalc_hashval/*=NULL*/);
public static native @Cast("uchar*") BytePointer cvPtrND( @Const CvArr arr, @Const IntPointer idx);
public static native @Cast("uchar*") ByteBuffer cvPtrND( @Const CvArr arr, @Const IntBuffer idx, IntBuffer type/*=NULL*/,
                      int create_node/*=1*/,
                      @Cast("unsigned*") IntBuffer precalc_hashval/*=NULL*/);
public static native @Cast("uchar*") ByteBuffer cvPtrND( @Const CvArr arr, @Const IntBuffer idx);
public static native @Cast("uchar*") byte[] cvPtrND( @Const CvArr arr, @Const int[] idx, int[] type/*=NULL*/,
                      int create_node/*=1*/,
                      @Cast("unsigned*") int[] precalc_hashval/*=NULL*/);
public static native @Cast("uchar*") byte[] cvPtrND( @Const CvArr arr, @Const int[] idx);

/* value = arr(idx0,idx1,...) */
public static native @ByVal CvScalar cvGet1D( @Const CvArr arr, int idx0 );
public static native @ByVal CvScalar cvGet2D( @Const CvArr arr, int idx0, int idx1 );
public static native @ByVal CvScalar cvGet3D( @Const CvArr arr, int idx0, int idx1, int idx2 );
public static native @ByVal CvScalar cvGetND( @Const CvArr arr, @Const IntPointer idx );
public static native @ByVal CvScalar cvGetND( @Const CvArr arr, @Const IntBuffer idx );
public static native @ByVal CvScalar cvGetND( @Const CvArr arr, @Const int[] idx );

/* for 1-channel arrays */
public static native double cvGetReal1D( @Const CvArr arr, int idx0 );
public static native double cvGetReal2D( @Const CvArr arr, int idx0, int idx1 );
public static native double cvGetReal3D( @Const CvArr arr, int idx0, int idx1, int idx2 );
public static native double cvGetRealND( @Const CvArr arr, @Const IntPointer idx );
public static native double cvGetRealND( @Const CvArr arr, @Const IntBuffer idx );
public static native double cvGetRealND( @Const CvArr arr, @Const int[] idx );

/* arr(idx0,idx1,...) = value */
public static native void cvSet1D( CvArr arr, int idx0, @ByVal CvScalar value );
public static native void cvSet2D( CvArr arr, int idx0, int idx1, @ByVal CvScalar value );
public static native void cvSet3D( CvArr arr, int idx0, int idx1, int idx2, @ByVal CvScalar value );
public static native void cvSetND( CvArr arr, @Const IntPointer idx, @ByVal CvScalar value );
public static native void cvSetND( CvArr arr, @Const IntBuffer idx, @ByVal CvScalar value );
public static native void cvSetND( CvArr arr, @Const int[] idx, @ByVal CvScalar value );

/* for 1-channel arrays */
public static native void cvSetReal1D( CvArr arr, int idx0, double value );
public static native void cvSetReal2D( CvArr arr, int idx0, int idx1, double value );
public static native void cvSetReal3D( CvArr arr, int idx0,
                        int idx1, int idx2, double value );
public static native void cvSetRealND( CvArr arr, @Const IntPointer idx, double value );
public static native void cvSetRealND( CvArr arr, @Const IntBuffer idx, double value );
public static native void cvSetRealND( CvArr arr, @Const int[] idx, double value );

/* clears element of ND dense array,
   in case of sparse arrays it deletes the specified node */
public static native void cvClearND( CvArr arr, @Const IntPointer idx );
public static native void cvClearND( CvArr arr, @Const IntBuffer idx );
public static native void cvClearND( CvArr arr, @Const int[] idx );

/* Converts CvArr (IplImage or CvMat,...) to CvMat.
   If the last parameter is non-zero, function can
   convert multi(>2)-dimensional array to CvMat as long as
   the last array's dimension is continous. The resultant
   matrix will be have appropriate (a huge) number of rows */
public static native CvMat cvGetMat( @Const CvArr arr, CvMat header,
                       IntPointer coi/*=NULL*/,
                       int allowND/*=0*/);
public static native CvMat cvGetMat( @Const CvArr arr, CvMat header);
public static native CvMat cvGetMat( @Const CvArr arr, CvMat header,
                       IntBuffer coi/*=NULL*/,
                       int allowND/*=0*/);
public static native CvMat cvGetMat( @Const CvArr arr, CvMat header,
                       int[] coi/*=NULL*/,
                       int allowND/*=0*/);

/* Converts CvArr (IplImage or CvMat) to IplImage */
public static native IplImage cvGetImage( @Const CvArr arr, IplImage image_header );


/* Changes a shape of multi-dimensional array.
   new_cn == 0 means that number of channels remains unchanged.
   new_dims == 0 means that number and sizes of dimensions remain the same
   (unless they need to be changed to set the new number of channels)
   if new_dims == 1, there is no need to specify new dimension sizes
   The resultant configuration should be achievable w/o data copying.
   If the resultant array is sparse, CvSparseMat header should be passed
   to the function else if the result is 1 or 2 dimensional,
   CvMat header should be passed to the function
   else CvMatND header should be passed */
public static native CvArr cvReshapeMatND( @Const CvArr arr,
                             int sizeof_header, CvArr header,
                             int new_cn, int new_dims, IntPointer new_sizes );
public static native CvArr cvReshapeMatND( @Const CvArr arr,
                             int sizeof_header, CvArr header,
                             int new_cn, int new_dims, IntBuffer new_sizes );
public static native CvArr cvReshapeMatND( @Const CvArr arr,
                             int sizeof_header, CvArr header,
                             int new_cn, int new_dims, int[] new_sizes );

// #define cvReshapeND( arr, header, new_cn, new_dims, new_sizes )
//       cvReshapeMatND( (arr), sizeof(*(header)), (header),
//                       (new_cn), (new_dims), (new_sizes))

public static native CvMat cvReshape( @Const CvArr arr, CvMat header,
                        int new_cn, int new_rows/*=0*/ );
public static native CvMat cvReshape( @Const CvArr arr, CvMat header,
                        int new_cn );

/* Repeats source 2d array several times in both horizontal and
   vertical direction to fill destination array */
public static native void cvRepeat( @Const CvArr src, CvArr dst );

/* Allocates array data */
public static native void cvCreateData( CvArr arr );

/* Releases array data */
public static native void cvReleaseData( CvArr arr );

/* Attaches user data to the array header. The step is reffered to
   the pre-last dimension. That is, all the planes of the array
   must be joint (w/o gaps) */
public static native void cvSetData( CvArr arr, Pointer data, int step );

/* Retrieves raw data of CvMat, IplImage or CvMatND.
   In the latter case the function raises an error if
   the array can not be represented as a matrix */
public static native void cvGetRawData( @Const CvArr arr, @Cast("uchar**") PointerPointer data,
                         IntPointer step/*=NULL*/,
                         CvSize roi_size/*=NULL*/);
public static native void cvGetRawData( @Const CvArr arr, @Cast("uchar**") @ByPtrPtr BytePointer data);
public static native void cvGetRawData( @Const CvArr arr, @Cast("uchar**") @ByPtrPtr BytePointer data,
                         IntPointer step/*=NULL*/,
                         CvSize roi_size/*=NULL*/);
public static native void cvGetRawData( @Const CvArr arr, @Cast("uchar**") @ByPtrPtr ByteBuffer data,
                         IntBuffer step/*=NULL*/,
                         CvSize roi_size/*=NULL*/);
public static native void cvGetRawData( @Const CvArr arr, @Cast("uchar**") @ByPtrPtr ByteBuffer data);
public static native void cvGetRawData( @Const CvArr arr, @Cast("uchar**") @ByPtrPtr byte[] data,
                         int[] step/*=NULL*/,
                         CvSize roi_size/*=NULL*/);
public static native void cvGetRawData( @Const CvArr arr, @Cast("uchar**") @ByPtrPtr byte[] data);

/* Returns width and height of array in elements */
public static native @ByVal CvSize cvGetSize( @Const CvArr arr );

/* Copies source array to destination array */
public static native void cvCopy( @Const CvArr src, CvArr dst,
                     @Const CvArr mask/*=NULL*/ );
public static native void cvCopy( @Const CvArr src, CvArr dst );

/* Sets all or "masked" elements of input array
   to the same value*/
public static native void cvSet( CvArr arr, @ByVal CvScalar value,
                    @Const CvArr mask/*=NULL*/ );
public static native void cvSet( CvArr arr, @ByVal CvScalar value );

/* Clears all the array elements (sets them to 0) */
public static native void cvSetZero( CvArr arr );
public static native void cvZero(CvArr arg1);


/* Splits a multi-channel array into the set of single-channel arrays or
   extracts particular [color] plane */
public static native void cvSplit( @Const CvArr src, CvArr dst0, CvArr dst1,
                      CvArr dst2, CvArr dst3 );

/* Merges a set of single-channel arrays into the single multi-channel array
   or inserts one particular [color] plane to the array */
public static native void cvMerge( @Const CvArr src0, @Const CvArr src1,
                      @Const CvArr src2, @Const CvArr src3,
                      CvArr dst );

/* Copies several channels from input arrays to
   certain channels of output arrays */
public static native void cvMixChannels( @Cast("const CvArr**") PointerPointer src, int src_count,
                            @Cast("CvArr**") PointerPointer dst, int dst_count,
                            @Const IntPointer from_to, int pair_count );
public static native void cvMixChannels( @Const @ByPtrPtr CvArr src, int src_count,
                            @ByPtrPtr CvArr dst, int dst_count,
                            @Const IntPointer from_to, int pair_count );
public static native void cvMixChannels( @Const @ByPtrPtr CvArr src, int src_count,
                            @ByPtrPtr CvArr dst, int dst_count,
                            @Const IntBuffer from_to, int pair_count );
public static native void cvMixChannels( @Const @ByPtrPtr CvArr src, int src_count,
                            @ByPtrPtr CvArr dst, int dst_count,
                            @Const int[] from_to, int pair_count );

/* Performs linear transformation on every source array element:
   dst(x,y,c) = scale*src(x,y,c)+shift.
   Arbitrary combination of input and output array depths are allowed
   (number of channels must be the same), thus the function can be used
   for type conversion */
public static native void cvConvertScale( @Const CvArr src, CvArr dst,
                             double scale/*=1*/,
                             double shift/*=0*/ );
public static native void cvConvertScale( @Const CvArr src, CvArr dst );
public static native void cvCvtScale(CvArr arg1, CvArr arg2, double arg3, double arg4);
public static native void cvScale(CvArr arg1, CvArr arg2, double arg3, double arg4);
public static native void cvConvert(CvArr src, CvArr dst);


/* Performs linear transformation on every source array element,
   stores absolute value of the result:
   dst(x,y,c) = abs(scale*src(x,y,c)+shift).
   destination array must have 8u type.
   In other cases one may use cvConvertScale + cvAbsDiffS */
public static native void cvConvertScaleAbs( @Const CvArr src, CvArr dst,
                                double scale/*=1*/,
                                double shift/*=0*/ );
public static native void cvConvertScaleAbs( @Const CvArr src, CvArr dst );
public static native void cvCvtScaleAbs(CvArr arg1, CvArr arg2, double arg3, double arg4);


/* checks termination criteria validity and
   sets eps to default_eps (if it is not set),
   max_iter to default_max_iters (if it is not set)
*/
public static native @ByVal CvTermCriteria cvCheckTermCriteria( @ByVal CvTermCriteria criteria,
                                           double default_eps,
                                           int default_max_iters );

/****************************************************************************************\
*                   Arithmetic, logic and comparison operations                          *
\****************************************************************************************/

/* dst(mask) = src1(mask) + src2(mask) */
public static native void cvAdd( @Const CvArr src1, @Const CvArr src2, CvArr dst,
                    @Const CvArr mask/*=NULL*/);
public static native void cvAdd( @Const CvArr src1, @Const CvArr src2, CvArr dst);

/* dst(mask) = src(mask) + value */
public static native void cvAddS( @Const CvArr src, @ByVal CvScalar value, CvArr dst,
                     @Const CvArr mask/*=NULL*/);
public static native void cvAddS( @Const CvArr src, @ByVal CvScalar value, CvArr dst);

/* dst(mask) = src1(mask) - src2(mask) */
public static native void cvSub( @Const CvArr src1, @Const CvArr src2, CvArr dst,
                    @Const CvArr mask/*=NULL*/);
public static native void cvSub( @Const CvArr src1, @Const CvArr src2, CvArr dst);

/* dst(mask) = src(mask) - value = src(mask) + (-value) */
public static native void cvSubS( @Const CvArr src, @ByVal CvScalar value, CvArr dst,
                         @Const CvArr mask/*=NULL*/);
public static native void cvSubS( @Const CvArr src, @ByVal CvScalar value, CvArr dst);

/* dst(mask) = value - src(mask) */
public static native void cvSubRS( @Const CvArr src, @ByVal CvScalar value, CvArr dst,
                      @Const CvArr mask/*=NULL*/);
public static native void cvSubRS( @Const CvArr src, @ByVal CvScalar value, CvArr dst);

/* dst(idx) = src1(idx) * src2(idx) * scale
   (scaled element-wise multiplication of 2 arrays) */
public static native void cvMul( @Const CvArr src1, @Const CvArr src2,
                    CvArr dst, double scale/*=1*/ );
public static native void cvMul( @Const CvArr src1, @Const CvArr src2,
                    CvArr dst );

/* element-wise division/inversion with scaling:
    dst(idx) = src1(idx) * scale / src2(idx)
    or dst(idx) = scale / src2(idx) if src1 == 0 */
public static native void cvDiv( @Const CvArr src1, @Const CvArr src2,
                    CvArr dst, double scale/*=1*/);
public static native void cvDiv( @Const CvArr src1, @Const CvArr src2,
                    CvArr dst);

/* dst = src1 * scale + src2 */
public static native void cvScaleAdd( @Const CvArr src1, @ByVal CvScalar scale,
                         @Const CvArr src2, CvArr dst );
// #define cvAXPY( A, real_scalar, B, C ) cvScaleAdd(A, cvRealScalar(real_scalar), B, C)

/* dst = src1 * alpha + src2 * beta + gamma */
public static native void cvAddWeighted( @Const CvArr src1, double alpha,
                            @Const CvArr src2, double beta,
                            double gamma, CvArr dst );

/* result = sum_i(src1(i) * src2(i)) (results for all channels are accumulated together) */
public static native double cvDotProduct( @Const CvArr src1, @Const CvArr src2 );

/* dst(idx) = src1(idx) & src2(idx) */
public static native void cvAnd( @Const CvArr src1, @Const CvArr src2,
                  CvArr dst, @Const CvArr mask/*=NULL*/);
public static native void cvAnd( @Const CvArr src1, @Const CvArr src2,
                  CvArr dst);

/* dst(idx) = src(idx) & value */
public static native void cvAndS( @Const CvArr src, @ByVal CvScalar value,
                   CvArr dst, @Const CvArr mask/*=NULL*/);
public static native void cvAndS( @Const CvArr src, @ByVal CvScalar value,
                   CvArr dst);

/* dst(idx) = src1(idx) | src2(idx) */
public static native void cvOr( @Const CvArr src1, @Const CvArr src2,
                 CvArr dst, @Const CvArr mask/*=NULL*/);
public static native void cvOr( @Const CvArr src1, @Const CvArr src2,
                 CvArr dst);

/* dst(idx) = src(idx) | value */
public static native void cvOrS( @Const CvArr src, @ByVal CvScalar value,
                  CvArr dst, @Const CvArr mask/*=NULL*/);
public static native void cvOrS( @Const CvArr src, @ByVal CvScalar value,
                  CvArr dst);

/* dst(idx) = src1(idx) ^ src2(idx) */
public static native void cvXor( @Const CvArr src1, @Const CvArr src2,
                  CvArr dst, @Const CvArr mask/*=NULL*/);
public static native void cvXor( @Const CvArr src1, @Const CvArr src2,
                  CvArr dst);

/* dst(idx) = src(idx) ^ value */
public static native void cvXorS( @Const CvArr src, @ByVal CvScalar value,
                   CvArr dst, @Const CvArr mask/*=NULL*/);
public static native void cvXorS( @Const CvArr src, @ByVal CvScalar value,
                   CvArr dst);

/* dst(idx) = ~src(idx) */
public static native void cvNot( @Const CvArr src, CvArr dst );

/* dst(idx) = lower(idx) <= src(idx) < upper(idx) */
public static native void cvInRange( @Const CvArr src, @Const CvArr lower,
                      @Const CvArr upper, CvArr dst );

/* dst(idx) = lower <= src(idx) < upper */
public static native void cvInRangeS( @Const CvArr src, @ByVal CvScalar lower,
                       @ByVal CvScalar upper, CvArr dst );

public static final int CV_CMP_EQ =   0;
public static final int CV_CMP_GT =   1;
public static final int CV_CMP_GE =   2;
public static final int CV_CMP_LT =   3;
public static final int CV_CMP_LE =   4;
public static final int CV_CMP_NE =   5;

/* The comparison operation support single-channel arrays only.
   Destination image should be 8uC1 or 8sC1 */

/* dst(idx) = src1(idx) _cmp_op_ src2(idx) */
public static native void cvCmp( @Const CvArr src1, @Const CvArr src2, CvArr dst, int cmp_op );

/* dst(idx) = src1(idx) _cmp_op_ value */
public static native void cvCmpS( @Const CvArr src, double value, CvArr dst, int cmp_op );

/* dst(idx) = min(src1(idx),src2(idx)) */
public static native void cvMin( @Const CvArr src1, @Const CvArr src2, CvArr dst );

/* dst(idx) = max(src1(idx),src2(idx)) */
public static native void cvMax( @Const CvArr src1, @Const CvArr src2, CvArr dst );

/* dst(idx) = min(src(idx),value) */
public static native void cvMinS( @Const CvArr src, double value, CvArr dst );

/* dst(idx) = max(src(idx),value) */
public static native void cvMaxS( @Const CvArr src, double value, CvArr dst );

/* dst(x,y,c) = abs(src1(x,y,c) - src2(x,y,c)) */
public static native void cvAbsDiff( @Const CvArr src1, @Const CvArr src2, CvArr dst );

/* dst(x,y,c) = abs(src(x,y,c) - value(c)) */
public static native void cvAbsDiffS( @Const CvArr src, CvArr dst, @ByVal CvScalar value );
// #define cvAbs( src, dst ) cvAbsDiffS( (src), (dst), cvScalarAll(0))

/****************************************************************************************\
*                                Math operations                                         *
\****************************************************************************************/

/* Does cartesian->polar coordinates conversion.
   Either of output components (magnitude or angle) is optional */
public static native void cvCartToPolar( @Const CvArr x, @Const CvArr y,
                            CvArr magnitude, CvArr angle/*=NULL*/,
                            int angle_in_degrees/*=0*/);
public static native void cvCartToPolar( @Const CvArr x, @Const CvArr y,
                            CvArr magnitude);

/* Does polar->cartesian coordinates conversion.
   Either of output components (magnitude or angle) is optional.
   If magnitude is missing it is assumed to be all 1's */
public static native void cvPolarToCart( @Const CvArr magnitude, @Const CvArr angle,
                            CvArr x, CvArr y,
                            int angle_in_degrees/*=0*/);
public static native void cvPolarToCart( @Const CvArr magnitude, @Const CvArr angle,
                            CvArr x, CvArr y);

/* Does powering: dst(idx) = src(idx)^power */
public static native void cvPow( @Const CvArr src, CvArr dst, double power );

/* Does exponention: dst(idx) = exp(src(idx)).
   Overflow is not handled yet. Underflow is handled.
   Maximal relative error is ~7e-6 for single-precision input */
public static native void cvExp( @Const CvArr src, CvArr dst );

/* Calculates natural logarithms: dst(idx) = log(abs(src(idx))).
   Logarithm of 0 gives large negative number(~-700)
   Maximal relative error is ~3e-7 for single-precision output
*/
public static native void cvLog( @Const CvArr src, CvArr dst );

/* Fast arctangent calculation */
public static native float cvFastArctan( float y, float x );

/* Fast cubic root calculation */
public static native float cvCbrt( float value );

/* Checks array values for NaNs, Infs or simply for too large numbers
   (if CV_CHECK_RANGE is set). If CV_CHECK_QUIET is set,
   no runtime errors is raised (function returns zero value in case of "bad" values).
   Otherwise cvError is called */
public static final int CV_CHECK_RANGE =    1;
public static final int CV_CHECK_QUIET =    2;
public static native int cvCheckArr( @Const CvArr arr, int flags/*=0*/,
                        double min_val/*=0*/, double max_val/*=0*/);
public static native int cvCheckArr( @Const CvArr arr);
public static native int cvCheckArray(CvArr arg1, int arg2, double arg3, double arg4);

public static final int CV_RAND_UNI =      0;
public static final int CV_RAND_NORMAL =   1;
public static native void cvRandArr( @Cast("CvRNG*") LongPointer rng, CvArr arr, int dist_type,
                      @ByVal CvScalar param1, @ByVal CvScalar param2 );
public static native void cvRandArr( @Cast("CvRNG*") LongBuffer rng, CvArr arr, int dist_type,
                      @ByVal CvScalar param1, @ByVal CvScalar param2 );
public static native void cvRandArr( @Cast("CvRNG*") long[] rng, CvArr arr, int dist_type,
                      @ByVal CvScalar param1, @ByVal CvScalar param2 );

public static native void cvRandShuffle( CvArr mat, @Cast("CvRNG*") LongPointer rng,
                           double iter_factor/*=1.*/);
public static native void cvRandShuffle( CvArr mat, @Cast("CvRNG*") LongPointer rng);
public static native void cvRandShuffle( CvArr mat, @Cast("CvRNG*") LongBuffer rng,
                           double iter_factor/*=1.*/);
public static native void cvRandShuffle( CvArr mat, @Cast("CvRNG*") LongBuffer rng);
public static native void cvRandShuffle( CvArr mat, @Cast("CvRNG*") long[] rng,
                           double iter_factor/*=1.*/);
public static native void cvRandShuffle( CvArr mat, @Cast("CvRNG*") long[] rng);

public static final int CV_SORT_EVERY_ROW = 0;
public static final int CV_SORT_EVERY_COLUMN = 1;
public static final int CV_SORT_ASCENDING = 0;
public static final int CV_SORT_DESCENDING = 16;

public static native void cvSort( @Const CvArr src, CvArr dst/*=NULL*/,
                    CvArr idxmat/*=NULL*/,
                    int flags/*=0*/);
public static native void cvSort( @Const CvArr src);

/* Finds real roots of a cubic equation */
public static native int cvSolveCubic( @Const CvMat coeffs, CvMat roots );

/* Finds all real and complex roots of a polynomial equation */
public static native void cvSolvePoly(@Const CvMat coeffs, CvMat roots2,
      int maxiter/*=20*/, int fig/*=100*/);
public static native void cvSolvePoly(@Const CvMat coeffs, CvMat roots2);

/****************************************************************************************\
*                                Matrix operations                                       *
\****************************************************************************************/

/* Calculates cross product of two 3d vectors */
public static native void cvCrossProduct( @Const CvArr src1, @Const CvArr src2, CvArr dst );

/* Matrix transform: dst = A*B + C, C is optional */
public static native void cvMatMulAdd(CvArr src1, CvArr src2, CvArr src3, CvArr dst);
public static native void cvMatMul(CvArr src1, CvArr src2, CvArr dst);

public static final int CV_GEMM_A_T = 1;
public static final int CV_GEMM_B_T = 2;
public static final int CV_GEMM_C_T = 4;
/* Extended matrix transform:
   dst = alpha*op(A)*op(B) + beta*op(C), where op(X) is X or X^T */
public static native void cvGEMM( @Const CvArr src1, @Const CvArr src2, double alpha,
                     @Const CvArr src3, double beta, CvArr dst,
                     int tABC/*=0*/);
public static native void cvGEMM( @Const CvArr src1, @Const CvArr src2, double alpha,
                     @Const CvArr src3, double beta, CvArr dst);
public static native void cvMatMulAddEx(CvArr arg1, CvArr arg2, double arg3, CvArr arg4, double arg5, CvArr arg6, int arg7);

/* Transforms each element of source array and stores
   resultant vectors in destination array */
public static native void cvTransform( @Const CvArr src, CvArr dst,
                          @Const CvMat transmat,
                          @Const CvMat shiftvec/*=NULL*/);
public static native void cvTransform( @Const CvArr src, CvArr dst,
                          @Const CvMat transmat);
public static native void cvMatMulAddS(CvArr arg1, CvArr arg2, CvMat arg3, CvMat arg4);

/* Does perspective transform on every element of input array */
public static native void cvPerspectiveTransform( @Const CvArr src, CvArr dst,
                                     @Const CvMat mat );

/* Calculates (A-delta)*(A-delta)^T (order=0) or (A-delta)^T*(A-delta) (order=1) */
public static native void cvMulTransposed( @Const CvArr src, CvArr dst, int order,
                             @Const CvArr delta/*=NULL*/,
                             double scale/*=1.*/ );
public static native void cvMulTransposed( @Const CvArr src, CvArr dst, int order );

/* Tranposes matrix. Square matrices can be transposed in-place */
public static native void cvTranspose( @Const CvArr src, CvArr dst );
public static native void cvT(CvArr arg1, CvArr arg2);

/* Completes the symmetric matrix from the lower (LtoR=0) or from the upper (LtoR!=0) part */
public static native void cvCompleteSymm( CvMat matrix, int LtoR/*=0*/ );
public static native void cvCompleteSymm( CvMat matrix );

/* Mirror array data around horizontal (flip=0),
   vertical (flip=1) or both(flip=-1) axises:
   cvFlip(src) flips images vertically and sequences horizontally (inplace) */
public static native void cvFlip( @Const CvArr src, CvArr dst/*=NULL*/,
                     int flip_mode/*=0*/);
public static native void cvFlip( @Const CvArr src);
public static native void cvMirror(CvArr arg1, CvArr arg2, int arg3);


public static final int CV_SVD_MODIFY_A =   1;
public static final int CV_SVD_U_T =        2;
public static final int CV_SVD_V_T =        4;

/* Performs Singular Value Decomposition of a matrix */
public static native void cvSVD( CvArr A, CvArr W, CvArr U/*=NULL*/,
                     CvArr V/*=NULL*/, int flags/*=0*/);
public static native void cvSVD( CvArr A, CvArr W);

/* Performs Singular Value Back Substitution (solves A*X = B):
   flags must be the same as in cvSVD */
public static native void cvSVBkSb( @Const CvArr W, @Const CvArr U,
                        @Const CvArr V, @Const CvArr B,
                        CvArr X, int flags );

public static final int CV_LU =  0;
public static final int CV_SVD = 1;
public static final int CV_SVD_SYM = 2;
public static final int CV_CHOLESKY = 3;
public static final int CV_QR =  4;
public static final int CV_NORMAL = 16;

/* Inverts matrix */
public static native double cvInvert( @Const CvArr src, CvArr dst,
                         int method/*=CV_LU*/);
public static native double cvInvert( @Const CvArr src, CvArr dst);
public static native void cvInv(CvArr arg1, CvArr arg2, int arg3);

/* Solves linear system (src1)*(dst) = (src2)
   (returns 0 if src1 is a singular and CV_LU method is used) */
public static native int cvSolve( @Const CvArr src1, @Const CvArr src2, CvArr dst,
                     int method/*=CV_LU*/);
public static native int cvSolve( @Const CvArr src1, @Const CvArr src2, CvArr dst);

/* Calculates determinant of input matrix */
public static native double cvDet( @Const CvArr mat );

/* Calculates trace of the matrix (sum of elements on the main diagonal) */
public static native @ByVal CvScalar cvTrace( @Const CvArr mat );

/* Finds eigen values and vectors of a symmetric matrix */
public static native void cvEigenVV( CvArr mat, CvArr evects, CvArr evals,
                        double eps/*=0*/,
                        int lowindex/*=-1*/,
                        int highindex/*=-1*/);
public static native void cvEigenVV( CvArr mat, CvArr evects, CvArr evals);

///* Finds selected eigen values and vectors of a symmetric matrix */
//CVAPI(void)  cvSelectedEigenVV( CvArr* mat, CvArr* evects, CvArr* evals,
//                                int lowindex, int highindex );

/* Makes an identity matrix (mat_ij = i == j) */
public static native void cvSetIdentity( CvArr mat, @ByVal CvScalar value/*=cvRealScalar(1)*/ );
public static native void cvSetIdentity( CvArr mat );

/* Fills matrix with given range of numbers */
public static native CvArr cvRange( CvArr mat, double start, double end );

/* Calculates covariation matrix for a set of vectors */
/* transpose([v1-avg, v2-avg,...]) * [v1-avg,v2-avg,...] */
public static final int CV_COVAR_SCRAMBLED = 0;

/* [v1-avg, v2-avg,...] * transpose([v1-avg,v2-avg,...]) */
public static final int CV_COVAR_NORMAL =    1;

/* do not calc average (i.e. mean vector) - use the input vector instead
   (useful for calculating covariance matrix by parts) */
public static final int CV_COVAR_USE_AVG =   2;

/* scale the covariance matrix coefficients by number of the vectors */
public static final int CV_COVAR_SCALE =     4;

/* all the input vectors are stored in a single matrix, as its rows */
public static final int CV_COVAR_ROWS =      8;

/* all the input vectors are stored in a single matrix, as its columns */
public static final int CV_COVAR_COLS =     16;

public static native void cvCalcCovarMatrix( @Cast("const CvArr**") PointerPointer vects, int count,
                                CvArr cov_mat, CvArr avg, int flags );
public static native void cvCalcCovarMatrix( @Const @ByPtrPtr CvArr vects, int count,
                                CvArr cov_mat, CvArr avg, int flags );

public static final int CV_PCA_DATA_AS_ROW = 0;
public static final int CV_PCA_DATA_AS_COL = 1;
public static final int CV_PCA_USE_AVG = 2;
public static native void cvCalcPCA( @Const CvArr data, CvArr mean,
                        CvArr eigenvals, CvArr eigenvects, int flags );

public static native void cvProjectPCA( @Const CvArr data, @Const CvArr mean,
                           @Const CvArr eigenvects, CvArr result );

public static native void cvBackProjectPCA( @Const CvArr proj, @Const CvArr mean,
                               @Const CvArr eigenvects, CvArr result );

/* Calculates Mahalanobis(weighted) distance */
public static native double cvMahalanobis( @Const CvArr vec1, @Const CvArr vec2, @Const CvArr mat );
public static native double cvMahalonobis(CvArr arg1, CvArr arg2, CvArr arg3);

/****************************************************************************************\
*                                    Array Statistics                                    *
\****************************************************************************************/

/* Finds sum of array elements */
public static native @ByVal CvScalar cvSum( @Const CvArr arr );

/* Calculates number of non-zero pixels */
public static native int cvCountNonZero( @Const CvArr arr );

/* Calculates mean value of array elements */
public static native @ByVal CvScalar cvAvg( @Const CvArr arr, @Const CvArr mask/*=NULL*/ );
public static native @ByVal CvScalar cvAvg( @Const CvArr arr );

/* Calculates mean and standard deviation of pixel values */
public static native void cvAvgSdv( @Const CvArr arr, CvScalar mean, CvScalar std_dev,
                       @Const CvArr mask/*=NULL*/ );
public static native void cvAvgSdv( @Const CvArr arr, CvScalar mean, CvScalar std_dev );

/* Finds global minimum, maximum and their positions */
public static native void cvMinMaxLoc( @Const CvArr arr, DoublePointer min_val, DoublePointer max_val,
                          CvPoint min_loc/*=NULL*/,
                          CvPoint max_loc/*=NULL*/,
                          @Const CvArr mask/*=NULL*/ );
public static native void cvMinMaxLoc( @Const CvArr arr, DoublePointer min_val, DoublePointer max_val );
public static native void cvMinMaxLoc( @Const CvArr arr, DoubleBuffer min_val, DoubleBuffer max_val,
                          @Cast("CvPoint*") IntBuffer min_loc/*=NULL*/,
                          @Cast("CvPoint*") IntBuffer max_loc/*=NULL*/,
                          @Const CvArr mask/*=NULL*/ );
public static native void cvMinMaxLoc( @Const CvArr arr, DoubleBuffer min_val, DoubleBuffer max_val );
public static native void cvMinMaxLoc( @Const CvArr arr, double[] min_val, double[] max_val,
                          @Cast("CvPoint*") int[] min_loc/*=NULL*/,
                          @Cast("CvPoint*") int[] max_loc/*=NULL*/,
                          @Const CvArr mask/*=NULL*/ );
public static native void cvMinMaxLoc( @Const CvArr arr, double[] min_val, double[] max_val );

/* types of array norm */
public static final int CV_C =            1;
public static final int CV_L1 =           2;
public static final int CV_L2 =           4;
public static final int CV_NORM_MASK =    7;
public static final int CV_RELATIVE =     8;
public static final int CV_DIFF =         16;
public static final int CV_MINMAX =       32;

public static final int CV_DIFF_C =       (CV_DIFF | CV_C);
public static final int CV_DIFF_L1 =      (CV_DIFF | CV_L1);
public static final int CV_DIFF_L2 =      (CV_DIFF | CV_L2);
public static final int CV_RELATIVE_C =   (CV_RELATIVE | CV_C);
public static final int CV_RELATIVE_L1 =  (CV_RELATIVE | CV_L1);
public static final int CV_RELATIVE_L2 =  (CV_RELATIVE | CV_L2);

/* Finds norm, difference norm or relative difference norm for an array (or two arrays) */
public static native double cvNorm( @Const CvArr arr1, @Const CvArr arr2/*=NULL*/,
                       int norm_type/*=CV_L2*/,
                       @Const CvArr mask/*=NULL*/ );
public static native double cvNorm( @Const CvArr arr1 );

public static native void cvNormalize( @Const CvArr src, CvArr dst,
                          double a/*=1.*/, double b/*=0.*/,
                          int norm_type/*=CV_L2*/,
                          @Const CvArr mask/*=NULL*/ );
public static native void cvNormalize( @Const CvArr src, CvArr dst );


public static final int CV_REDUCE_SUM = 0;
public static final int CV_REDUCE_AVG = 1;
public static final int CV_REDUCE_MAX = 2;
public static final int CV_REDUCE_MIN = 3;

public static native void cvReduce( @Const CvArr src, CvArr dst, int dim/*=-1*/,
                       int op/*=CV_REDUCE_SUM*/ );
public static native void cvReduce( @Const CvArr src, CvArr dst );

/****************************************************************************************\
*                      Discrete Linear Transforms and Related Functions                  *
\****************************************************************************************/

public static final int CV_DXT_FORWARD =  0;
public static final int CV_DXT_INVERSE =  1;
public static final int CV_DXT_SCALE =    2; /* divide result by size of array */
public static final int CV_DXT_INV_SCALE = (CV_DXT_INVERSE + CV_DXT_SCALE);
public static final int CV_DXT_INVERSE_SCALE = CV_DXT_INV_SCALE;
public static final int CV_DXT_ROWS =     4; /* transform each row individually */
public static final int CV_DXT_MUL_CONJ = 8; /* conjugate the second argument of cvMulSpectrums */

/* Discrete Fourier Transform:
    complex->complex,
    real->ccs (forward),
    ccs->real (inverse) */
public static native void cvDFT( @Const CvArr src, CvArr dst, int flags,
                    int nonzero_rows/*=0*/ );
public static native void cvDFT( @Const CvArr src, CvArr dst, int flags );
public static native void cvFFT(CvArr arg1, CvArr arg2, int arg3, int arg4);

/* Multiply results of DFTs: DFT(X)*DFT(Y) or DFT(X)*conj(DFT(Y)) */
public static native void cvMulSpectrums( @Const CvArr src1, @Const CvArr src2,
                             CvArr dst, int flags );

/* Finds optimal DFT vector size >= size0 */
public static native int cvGetOptimalDFTSize( int size0 );

/* Discrete Cosine Transform */
public static native void cvDCT( @Const CvArr src, CvArr dst, int flags );

/****************************************************************************************\
*                              Dynamic data structures                                   *
\****************************************************************************************/

/* Calculates length of sequence slice (with support of negative indices). */
public static native int cvSliceLength( @ByVal CvSlice slice, @Const CvSeq seq );


/* Creates new memory storage.
   block_size == 0 means that default,
   somewhat optimal size, is used (currently, it is 64K) */
public static native CvMemStorage cvCreateMemStorage( int block_size/*=0*/);
public static native CvMemStorage cvCreateMemStorage();


/* Creates a memory storage that will borrow memory blocks from parent storage */
public static native CvMemStorage cvCreateChildMemStorage( CvMemStorage parent );


/* Releases memory storage. All the children of a parent must be released before
   the parent. A child storage returns all the blocks to parent when it is released */
public static native void cvReleaseMemStorage( @Cast("CvMemStorage**") PointerPointer storage );
public static native void cvReleaseMemStorage( @ByPtrPtr CvMemStorage storage );


/* Clears memory storage. This is the only way(!!!) (besides cvRestoreMemStoragePos)
   to reuse memory allocated for the storage - cvClearSeq,cvClearSet ...
   do not free any memory.
   A child storage returns all the blocks to the parent when it is cleared */
public static native void cvClearMemStorage( CvMemStorage storage );

/* Remember a storage "free memory" position */
public static native void cvSaveMemStoragePos( @Const CvMemStorage storage, CvMemStoragePos pos );

/* Restore a storage "free memory" position */
public static native void cvRestoreMemStoragePos( CvMemStorage storage, CvMemStoragePos pos );

/* Allocates continuous buffer of the specified size in the storage */
public static native Pointer cvMemStorageAlloc( CvMemStorage storage, @Cast("size_t") long size );

/* Allocates string in memory storage */
public static native @ByVal CvString cvMemStorageAllocString( CvMemStorage storage, @Cast("const char*") BytePointer ptr,
                                         int len/*=-1*/ );
public static native @ByVal CvString cvMemStorageAllocString( CvMemStorage storage, @Cast("const char*") BytePointer ptr );
public static native @ByVal CvString cvMemStorageAllocString( CvMemStorage storage, String ptr,
                                         int len/*=-1*/ );
public static native @ByVal CvString cvMemStorageAllocString( CvMemStorage storage, String ptr );

/* Creates new empty sequence that will reside in the specified storage */
public static native CvSeq cvCreateSeq( int seq_flags, @Cast("size_t") long header_size,
                            @Cast("size_t") long elem_size, CvMemStorage storage );

/* Changes default size (granularity) of sequence blocks.
   The default size is ~1Kbyte */
public static native void cvSetSeqBlockSize( CvSeq seq, int delta_elems );


/* Adds new element to the end of sequence. Returns pointer to the element */
public static native @Cast("schar*") BytePointer cvSeqPush( CvSeq seq, @Const Pointer element/*=NULL*/);
public static native @Cast("schar*") BytePointer cvSeqPush( CvSeq seq);


/* Adds new element to the beginning of sequence. Returns pointer to it */
public static native @Cast("schar*") BytePointer cvSeqPushFront( CvSeq seq, @Const Pointer element/*=NULL*/);
public static native @Cast("schar*") BytePointer cvSeqPushFront( CvSeq seq);


/* Removes the last element from sequence and optionally saves it */
public static native void cvSeqPop( CvSeq seq, Pointer element/*=NULL*/);
public static native void cvSeqPop( CvSeq seq);


/* Removes the first element from sequence and optioanally saves it */
public static native void cvSeqPopFront( CvSeq seq, Pointer element/*=NULL*/);
public static native void cvSeqPopFront( CvSeq seq);


public static final int CV_FRONT = 1;
public static final int CV_BACK = 0;
/* Adds several new elements to the end of sequence */
public static native void cvSeqPushMulti( CvSeq seq, @Const Pointer elements,
                             int count, int in_front/*=0*/ );
public static native void cvSeqPushMulti( CvSeq seq, @Const Pointer elements,
                             int count );

/* Removes several elements from the end of sequence and optionally saves them */
public static native void cvSeqPopMulti( CvSeq seq, Pointer elements,
                            int count, int in_front/*=0*/ );
public static native void cvSeqPopMulti( CvSeq seq, Pointer elements,
                            int count );

/* Inserts a new element in the middle of sequence.
   cvSeqInsert(seq,0,elem) == cvSeqPushFront(seq,elem) */
public static native @Cast("schar*") BytePointer cvSeqInsert( CvSeq seq, int before_index,
                            @Const Pointer element/*=NULL*/);
public static native @Cast("schar*") BytePointer cvSeqInsert( CvSeq seq, int before_index);

/* Removes specified sequence element */
public static native void cvSeqRemove( CvSeq seq, int index );


/* Removes all the elements from the sequence. The freed memory
   can be reused later only by the same sequence unless cvClearMemStorage
   or cvRestoreMemStoragePos is called */
public static native void cvClearSeq( CvSeq seq );


/* Retrieves pointer to specified sequence element.
   Negative indices are supported and mean counting from the end
   (e.g -1 means the last sequence element) */
public static native @Cast("schar*") BytePointer cvGetSeqElem( @Const CvSeq seq, int index );

/* Calculates index of the specified sequence element.
   Returns -1 if element does not belong to the sequence */
public static native int cvSeqElemIdx( @Const CvSeq seq, @Const Pointer element,
                         @Cast("CvSeqBlock**") PointerPointer block/*=NULL*/ );
public static native int cvSeqElemIdx( @Const CvSeq seq, @Const Pointer element );
public static native int cvSeqElemIdx( @Const CvSeq seq, @Const Pointer element,
                         @ByPtrPtr CvSeqBlock block/*=NULL*/ );

/* Initializes sequence writer. The new elements will be added to the end of sequence */
public static native void cvStartAppendToSeq( CvSeq seq, CvSeqWriter writer );


/* Combination of cvCreateSeq and cvStartAppendToSeq */
public static native void cvStartWriteSeq( int seq_flags, int header_size,
                              int elem_size, CvMemStorage storage,
                              CvSeqWriter writer );

/* Closes sequence writer, updates sequence header and returns pointer
   to the resultant sequence
   (which may be useful if the sequence was created using cvStartWriteSeq))
*/
public static native CvSeq cvEndWriteSeq( CvSeqWriter writer );


/* Updates sequence header. May be useful to get access to some of previously
   written elements via cvGetSeqElem or sequence reader */
public static native void cvFlushSeqWriter( CvSeqWriter writer );


/* Initializes sequence reader.
   The sequence can be read in forward or backward direction */
public static native void cvStartReadSeq( @Const CvSeq seq, CvSeqReader reader,
                           int reverse/*=0*/ );
public static native void cvStartReadSeq( @Const CvSeq seq, CvSeqReader reader );


/* Returns current sequence reader position (currently observed sequence element) */
public static native int cvGetSeqReaderPos( CvSeqReader reader );


/* Changes sequence reader position. It may seek to an absolute or
   to relative to the current position */
public static native void cvSetSeqReaderPos( CvSeqReader reader, int index,
                                 int is_relative/*=0*/);
public static native void cvSetSeqReaderPos( CvSeqReader reader, int index);

/* Copies sequence content to a continuous piece of memory */
public static native Pointer cvCvtSeqToArray( @Const CvSeq seq, Pointer elements,
                               @ByVal CvSlice slice/*=CV_WHOLE_SEQ*/ );
public static native Pointer cvCvtSeqToArray( @Const CvSeq seq, Pointer elements );

/* Creates sequence header for array.
   After that all the operations on sequences that do not alter the content
   can be applied to the resultant sequence */
public static native CvSeq cvMakeSeqHeaderForArray( int seq_type, int header_size,
                                       int elem_size, Pointer elements, int total,
                                       CvSeq seq, CvSeqBlock block );

/* Extracts sequence slice (with or without copying sequence elements) */
public static native CvSeq cvSeqSlice( @Const CvSeq seq, @ByVal CvSlice slice,
                         CvMemStorage storage/*=NULL*/,
                         int copy_data/*=0*/);
public static native CvSeq cvSeqSlice( @Const CvSeq seq, @ByVal CvSlice slice);

public static native CvSeq cvCloneSeq( @Const CvSeq seq, CvMemStorage storage/*=NULL*/);
public static native CvSeq cvCloneSeq( @Const CvSeq seq);

/* Removes sequence slice */
public static native void cvSeqRemoveSlice( CvSeq seq, @ByVal CvSlice slice );

/* Inserts a sequence or array into another sequence */
public static native void cvSeqInsertSlice( CvSeq seq, int before_index, @Const CvArr from_arr );

/* a < b ? -1 : a > b ? 1 : 0 */
@Convention("CV_CDECL") public static class CvCmpFunc extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    CvCmpFunc(Pointer p) { super(p); }
    protected CvCmpFunc() { allocate(); }
    private native void allocate();
    public native int call(@Const Pointer a, @Const Pointer b, Pointer userdata );
}

/* Sorts sequence in-place given element comparison function */
public static native void cvSeqSort( CvSeq seq, CvCmpFunc func, Pointer userdata/*=NULL*/ );
public static native void cvSeqSort( CvSeq seq, CvCmpFunc func );

/* Finds element in a [sorted] sequence */
public static native @Cast("schar*") BytePointer cvSeqSearch( CvSeq seq, @Const Pointer elem, CvCmpFunc func,
                           int is_sorted, IntPointer elem_idx,
                           Pointer userdata/*=NULL*/ );
public static native @Cast("schar*") BytePointer cvSeqSearch( CvSeq seq, @Const Pointer elem, CvCmpFunc func,
                           int is_sorted, IntPointer elem_idx );
public static native @Cast("schar*") ByteBuffer cvSeqSearch( CvSeq seq, @Const Pointer elem, CvCmpFunc func,
                           int is_sorted, IntBuffer elem_idx,
                           Pointer userdata/*=NULL*/ );
public static native @Cast("schar*") ByteBuffer cvSeqSearch( CvSeq seq, @Const Pointer elem, CvCmpFunc func,
                           int is_sorted, IntBuffer elem_idx );
public static native @Cast("schar*") byte[] cvSeqSearch( CvSeq seq, @Const Pointer elem, CvCmpFunc func,
                           int is_sorted, int[] elem_idx,
                           Pointer userdata/*=NULL*/ );
public static native @Cast("schar*") byte[] cvSeqSearch( CvSeq seq, @Const Pointer elem, CvCmpFunc func,
                           int is_sorted, int[] elem_idx );

/* Reverses order of sequence elements in-place */
public static native void cvSeqInvert( CvSeq seq );

/* Splits sequence into one or more equivalence classes using the specified criteria */
public static native int cvSeqPartition( @Const CvSeq seq, CvMemStorage storage,
                            @Cast("CvSeq**") PointerPointer labels, CvCmpFunc is_equal, Pointer userdata );
public static native int cvSeqPartition( @Const CvSeq seq, CvMemStorage storage,
                            @ByPtrPtr CvSeq labels, CvCmpFunc is_equal, Pointer userdata );

/************ Internal sequence functions ************/
public static native void cvChangeSeqBlock( Pointer reader, int direction );
public static native void cvCreateSeqBlock( CvSeqWriter writer );


/* Creates a new set */
public static native CvSet cvCreateSet( int set_flags, int header_size,
                            int elem_size, CvMemStorage storage );

/* Adds new element to the set and returns pointer to it */
public static native int cvSetAdd( CvSet set_header, CvSetElem elem/*=NULL*/,
                      @Cast("CvSetElem**") PointerPointer inserted_elem/*=NULL*/ );
public static native int cvSetAdd( CvSet set_header );
public static native int cvSetAdd( CvSet set_header, CvSetElem elem/*=NULL*/,
                      @ByPtrPtr CvSetElem inserted_elem/*=NULL*/ );

/* Fast variant of cvSetAdd */
public static native CvSetElem cvSetNew( CvSet set_header );

/* Removes set element given its pointer */
public static native void cvSetRemoveByPtr( CvSet set_header, Pointer elem );

/* Removes element from the set by its index  */
public static native void cvSetRemove( CvSet set_header, int index );

/* Returns a set element by index. If the element doesn't belong to the set,
   NULL is returned */
public static native CvSetElem cvGetSetElem( @Const CvSet set_header, int idx );

/* Removes all the elements from the set */
public static native void cvClearSet( CvSet set_header );

/* Creates new graph */
public static native CvGraph cvCreateGraph( int graph_flags, int header_size,
                                int vtx_size, int edge_size,
                                CvMemStorage storage );

/* Adds new vertex to the graph */
public static native int cvGraphAddVtx( CvGraph graph, @Const CvGraphVtx vtx/*=NULL*/,
                           @Cast("CvGraphVtx**") PointerPointer inserted_vtx/*=NULL*/ );
public static native int cvGraphAddVtx( CvGraph graph );
public static native int cvGraphAddVtx( CvGraph graph, @Const CvGraphVtx vtx/*=NULL*/,
                           @ByPtrPtr CvGraphVtx inserted_vtx/*=NULL*/ );


/* Removes vertex from the graph together with all incident edges */
public static native int cvGraphRemoveVtx( CvGraph graph, int index );
public static native int cvGraphRemoveVtxByPtr( CvGraph graph, CvGraphVtx vtx );


/* Link two vertices specifed by indices or pointers if they
   are not connected or return pointer to already existing edge
   connecting the vertices.
   Functions return 1 if a new edge was created, 0 otherwise */
public static native int cvGraphAddEdge( CvGraph graph,
                            int start_idx, int end_idx,
                            @Const CvGraphEdge edge/*=NULL*/,
                            @Cast("CvGraphEdge**") PointerPointer inserted_edge/*=NULL*/ );
public static native int cvGraphAddEdge( CvGraph graph,
                            int start_idx, int end_idx );
public static native int cvGraphAddEdge( CvGraph graph,
                            int start_idx, int end_idx,
                            @Const CvGraphEdge edge/*=NULL*/,
                            @ByPtrPtr CvGraphEdge inserted_edge/*=NULL*/ );

public static native int cvGraphAddEdgeByPtr( CvGraph graph,
                               CvGraphVtx start_vtx, CvGraphVtx end_vtx,
                               @Const CvGraphEdge edge/*=NULL*/,
                               @Cast("CvGraphEdge**") PointerPointer inserted_edge/*=NULL*/ );
public static native int cvGraphAddEdgeByPtr( CvGraph graph,
                               CvGraphVtx start_vtx, CvGraphVtx end_vtx );
public static native int cvGraphAddEdgeByPtr( CvGraph graph,
                               CvGraphVtx start_vtx, CvGraphVtx end_vtx,
                               @Const CvGraphEdge edge/*=NULL*/,
                               @ByPtrPtr CvGraphEdge inserted_edge/*=NULL*/ );

/* Remove edge connecting two vertices */
public static native void cvGraphRemoveEdge( CvGraph graph, int start_idx, int end_idx );
public static native void cvGraphRemoveEdgeByPtr( CvGraph graph, CvGraphVtx start_vtx,
                                     CvGraphVtx end_vtx );

/* Find edge connecting two vertices */
public static native CvGraphEdge cvFindGraphEdge( @Const CvGraph graph, int start_idx, int end_idx );
public static native CvGraphEdge cvFindGraphEdgeByPtr( @Const CvGraph graph,
                                           @Const CvGraphVtx start_vtx,
                                           @Const CvGraphVtx end_vtx );
public static native CvGraphEdge cvGraphFindEdge(CvGraph arg1, int arg2, int arg3);
public static native CvGraphEdge cvGraphFindEdgeByPtr(CvGraph arg1, CvGraphVtx arg2, CvGraphVtx arg3);

/* Remove all vertices and edges from the graph */
public static native void cvClearGraph( CvGraph graph );


/* Count number of edges incident to the vertex */
public static native int cvGraphVtxDegree( @Const CvGraph graph, int vtx_idx );
public static native int cvGraphVtxDegreeByPtr( @Const CvGraph graph, @Const CvGraphVtx vtx );


/* Retrieves graph vertex by given index */
// #define cvGetGraphVtx( graph, idx ) (CvGraphVtx*)cvGetSetElem((CvSet*)(graph), (idx))

/* Retrieves index of a graph vertex given its pointer */
// #define cvGraphVtxIdx( graph, vtx ) ((vtx)->flags & CV_SET_ELEM_IDX_MASK)

/* Retrieves index of a graph edge given its pointer */
// #define cvGraphEdgeIdx( graph, edge ) ((edge)->flags & CV_SET_ELEM_IDX_MASK)

// #define cvGraphGetVtxCount( graph ) ((graph)->active_count)
// #define cvGraphGetEdgeCount( graph ) ((graph)->edges->active_count)

public static final int CV_GRAPH_VERTEX =        1;
public static final int CV_GRAPH_TREE_EDGE =     2;
public static final int CV_GRAPH_BACK_EDGE =     4;
public static final int CV_GRAPH_FORWARD_EDGE =  8;
public static final int CV_GRAPH_CROSS_EDGE =    16;
public static final int CV_GRAPH_ANY_EDGE =      30;
public static final int CV_GRAPH_NEW_TREE =      32;
public static final int CV_GRAPH_BACKTRACKING =  64;
public static final int CV_GRAPH_OVER =          -1;

public static final int CV_GRAPH_ALL_ITEMS =    -1;

/* flags for graph vertices and edges */
public static final int CV_GRAPH_ITEM_VISITED_FLAG =  (1 << 30);
// #define  CV_IS_GRAPH_VERTEX_VISITED(vtx)
//     (((CvGraphVtx*)(vtx))->flags & CV_GRAPH_ITEM_VISITED_FLAG)
// #define  CV_IS_GRAPH_EDGE_VISITED(edge)
//     (((CvGraphEdge*)(edge))->flags & CV_GRAPH_ITEM_VISITED_FLAG)
public static final int CV_GRAPH_SEARCH_TREE_NODE_FLAG =   (1 << 29);
public static final int CV_GRAPH_FORWARD_EDGE_FLAG =       (1 << 28);

public static class CvGraphScanner extends AbstractCvGraphScanner {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvGraphScanner() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvGraphScanner(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvGraphScanner(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvGraphScanner position(int position) {
        return (CvGraphScanner)super.position(position);
    }

    public native CvGraphVtx vtx(); public native CvGraphScanner vtx(CvGraphVtx vtx);       /* current graph vertex (or current edge origin) */
    public native CvGraphVtx dst(); public native CvGraphScanner dst(CvGraphVtx dst);       /* current graph edge destination vertex */
    public native CvGraphEdge edge(); public native CvGraphScanner edge(CvGraphEdge edge);     /* current edge */

    public native CvGraph graph(); public native CvGraphScanner graph(CvGraph graph);        /* the graph */
    public native CvSeq stack(); public native CvGraphScanner stack(CvSeq stack);        /* the graph vertex stack */
    public native int index(); public native CvGraphScanner index(int index);        /* the lower bound of certainly visited vertices */
    public native int mask(); public native CvGraphScanner mask(int mask);         /* event mask */
}

/* Creates new graph scanner. */
public static native CvGraphScanner cvCreateGraphScanner( CvGraph graph,
                                             CvGraphVtx vtx/*=NULL*/,
                                             int mask/*=CV_GRAPH_ALL_ITEMS*/);
public static native CvGraphScanner cvCreateGraphScanner( CvGraph graph);

/* Releases graph scanner. */
public static native void cvReleaseGraphScanner( @Cast("CvGraphScanner**") PointerPointer scanner );
public static native void cvReleaseGraphScanner( @ByPtrPtr CvGraphScanner scanner );

/* Get next graph element */
public static native int cvNextGraphItem( CvGraphScanner scanner );

/* Creates a copy of graph */
public static native CvGraph cvCloneGraph( @Const CvGraph graph, CvMemStorage storage );


/* Does look-up transformation. Elements of the source array
   (that should be 8uC1 or 8sC1) are used as indexes in lutarr 256-element table */
public static native void cvLUT( @Const CvArr src, CvArr dst, @Const CvArr lut );


/******************* Iteration through the sequence tree *****************/
public static class CvTreeNodeIterator extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public CvTreeNodeIterator() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public CvTreeNodeIterator(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvTreeNodeIterator(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public CvTreeNodeIterator position(int position) {
        return (CvTreeNodeIterator)super.position(position);
    }

    @MemberGetter public native @Const Pointer node();
    public native int level(); public native CvTreeNodeIterator level(int level);
    public native int max_level(); public native CvTreeNodeIterator max_level(int max_level);
}

public static native void cvInitTreeNodeIterator( CvTreeNodeIterator tree_iterator,
                                   @Const Pointer first, int max_level );
public static native Pointer cvNextTreeNode( CvTreeNodeIterator tree_iterator );
public static native Pointer cvPrevTreeNode( CvTreeNodeIterator tree_iterator );

/* Inserts sequence into tree with specified "parent" sequence.
   If parent is equal to frame (e.g. the most external contour),
   then added contour will have null pointer to parent. */
public static native void cvInsertNodeIntoTree( Pointer node, Pointer parent, Pointer frame );

/* Removes contour from tree (together with the contour children). */
public static native void cvRemoveNodeFromTree( Pointer node, Pointer frame );

/* Gathers pointers to all the sequences,
   accessible from the <first>, to the single sequence */
public static native CvSeq cvTreeToNodeSeq( @Const Pointer first, int header_size,
                              CvMemStorage storage );

/* The function implements the K-means algorithm for clustering an array of sample
   vectors in a specified number of classes */
public static final int CV_KMEANS_USE_INITIAL_LABELS =    1;
public static native int cvKMeans2( @Const CvArr samples, int cluster_count, CvArr labels,
                      @ByVal CvTermCriteria termcrit, int attempts/*=1*/,
                      @Cast("CvRNG*") LongPointer rng/*=0*/, int flags/*=0*/,
                      CvArr _centers/*=0*/, DoublePointer compactness/*=0*/ );
public static native int cvKMeans2( @Const CvArr samples, int cluster_count, CvArr labels,
                      @ByVal CvTermCriteria termcrit );
public static native int cvKMeans2( @Const CvArr samples, int cluster_count, CvArr labels,
                      @ByVal CvTermCriteria termcrit, int attempts/*=1*/,
                      @Cast("CvRNG*") LongBuffer rng/*=0*/, int flags/*=0*/,
                      CvArr _centers/*=0*/, DoubleBuffer compactness/*=0*/ );
public static native int cvKMeans2( @Const CvArr samples, int cluster_count, CvArr labels,
                      @ByVal CvTermCriteria termcrit, int attempts/*=1*/,
                      @Cast("CvRNG*") long[] rng/*=0*/, int flags/*=0*/,
                      CvArr _centers/*=0*/, double[] compactness/*=0*/ );

/****************************************************************************************\
*                                    System functions                                    *
\****************************************************************************************/

/* Loads optimized functions from IPP, MKL etc. or switches back to pure C code */
public static native int cvUseOptimized( int on_off );

@Convention("CV_STDCALL") public static class Cv_iplCreateImageHeader extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    Cv_iplCreateImageHeader(Pointer p) { super(p); }
    protected Cv_iplCreateImageHeader() { allocate(); }
    private native void allocate();
    public native IplImage call(int arg0,int arg1,int arg2,@Cast("char*") BytePointer arg3,@Cast("char*") BytePointer arg4,int arg5,int arg6,int arg7,int arg8,int arg9,
                            IplROI arg10,IplImage arg11,Pointer arg12,IplTileInfo arg13);
}
@Convention("CV_STDCALL") public static class Cv_iplAllocateImageData extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    Cv_iplAllocateImageData(Pointer p) { super(p); }
    protected Cv_iplAllocateImageData() { allocate(); }
    private native void allocate();
    public native void call(IplImage arg0,int arg1,int arg2);
}
@Convention("CV_STDCALL") public static class Cv_iplDeallocate extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    Cv_iplDeallocate(Pointer p) { super(p); }
    protected Cv_iplDeallocate() { allocate(); }
    private native void allocate();
    public native void call(IplImage arg0,int arg1);
}
@Convention("CV_STDCALL") public static class Cv_iplCreateROI extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    Cv_iplCreateROI(Pointer p) { super(p); }
    protected Cv_iplCreateROI() { allocate(); }
    private native void allocate();
    public native IplROI call(int arg0,int arg1,int arg2,int arg3,int arg4);
}
@Convention("CV_STDCALL") public static class Cv_iplCloneImage extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    Cv_iplCloneImage(Pointer p) { super(p); }
    protected Cv_iplCloneImage() { allocate(); }
    private native void allocate();
    public native IplImage call(@Const IplImage arg0);
}

/* Makes OpenCV use IPL functions for IplImage allocation/deallocation */
public static native void cvSetIPLAllocators( Cv_iplCreateImageHeader create_header,
                               Cv_iplAllocateImageData allocate_data,
                               Cv_iplDeallocate deallocate,
                               Cv_iplCreateROI create_roi,
                               Cv_iplCloneImage clone_image );

// #define CV_TURN_ON_IPL_COMPATIBILITY()
//     cvSetIPLAllocators( iplCreateImageHeader, iplAllocateImage,
//                         iplDeallocate, iplCreateROI, iplCloneImage )

/****************************************************************************************\
*                                    Data Persistence                                    *
\****************************************************************************************/

/********************************** High-level functions ********************************/

/* opens existing or creates new file storage */
public static native CvFileStorage cvOpenFileStorage( @Cast("const char*") BytePointer filename, CvMemStorage memstorage,
                                          int flags, @Cast("const char*") BytePointer encoding/*=NULL*/ );
public static native CvFileStorage cvOpenFileStorage( @Cast("const char*") BytePointer filename, CvMemStorage memstorage,
                                          int flags );
public static native CvFileStorage cvOpenFileStorage( String filename, CvMemStorage memstorage,
                                          int flags, String encoding/*=NULL*/ );
public static native CvFileStorage cvOpenFileStorage( String filename, CvMemStorage memstorage,
                                          int flags );

/* closes file storage and deallocates buffers */
public static native void cvReleaseFileStorage( @Cast("CvFileStorage**") PointerPointer fs );
public static native void cvReleaseFileStorage( @ByPtrPtr CvFileStorage fs );

/* returns attribute value or 0 (NULL) if there is no such attribute */
public static native @Cast("const char*") BytePointer cvAttrValue( @Const CvAttrList attr, @Cast("const char*") BytePointer attr_name );
public static native String cvAttrValue( @Const CvAttrList attr, String attr_name );

/* starts writing compound structure (map or sequence) */
public static native void cvStartWriteStruct( CvFileStorage fs, @Cast("const char*") BytePointer name,
                                int struct_flags, @Cast("const char*") BytePointer type_name/*=NULL*/,
                                @ByVal CvAttrList attributes/*=cvAttrList()*/);
public static native void cvStartWriteStruct( CvFileStorage fs, @Cast("const char*") BytePointer name,
                                int struct_flags);
public static native void cvStartWriteStruct( CvFileStorage fs, String name,
                                int struct_flags, String type_name/*=NULL*/,
                                @ByVal CvAttrList attributes/*=cvAttrList()*/);
public static native void cvStartWriteStruct( CvFileStorage fs, String name,
                                int struct_flags);

/* finishes writing compound structure */
public static native void cvEndWriteStruct( CvFileStorage fs );

/* writes an integer */
public static native void cvWriteInt( CvFileStorage fs, @Cast("const char*") BytePointer name, int value );
public static native void cvWriteInt( CvFileStorage fs, String name, int value );

/* writes a floating-point number */
public static native void cvWriteReal( CvFileStorage fs, @Cast("const char*") BytePointer name, double value );
public static native void cvWriteReal( CvFileStorage fs, String name, double value );

/* writes a string */
public static native void cvWriteString( CvFileStorage fs, @Cast("const char*") BytePointer name,
                           @Cast("const char*") BytePointer str, int quote/*=0*/ );
public static native void cvWriteString( CvFileStorage fs, @Cast("const char*") BytePointer name,
                           @Cast("const char*") BytePointer str );
public static native void cvWriteString( CvFileStorage fs, String name,
                           String str, int quote/*=0*/ );
public static native void cvWriteString( CvFileStorage fs, String name,
                           String str );

/* writes a comment */
public static native void cvWriteComment( CvFileStorage fs, @Cast("const char*") BytePointer comment,
                            int eol_comment );
public static native void cvWriteComment( CvFileStorage fs, String comment,
                            int eol_comment );

/* writes instance of a standard type (matrix, image, sequence, graph etc.)
   or user-defined type */
public static native void cvWrite( CvFileStorage fs, @Cast("const char*") BytePointer name, @Const Pointer ptr,
                         @ByVal CvAttrList attributes/*=cvAttrList()*/);
public static native void cvWrite( CvFileStorage fs, @Cast("const char*") BytePointer name, @Const Pointer ptr);
public static native void cvWrite( CvFileStorage fs, String name, @Const Pointer ptr,
                         @ByVal CvAttrList attributes/*=cvAttrList()*/);
public static native void cvWrite( CvFileStorage fs, String name, @Const Pointer ptr);

/* starts the next stream */
public static native void cvStartNextStream( CvFileStorage fs );

/* helper function: writes multiple integer or floating-point numbers */
public static native void cvWriteRawData( CvFileStorage fs, @Const Pointer src,
                                int len, @Cast("const char*") BytePointer dt );
public static native void cvWriteRawData( CvFileStorage fs, @Const Pointer src,
                                int len, String dt );

/* returns the hash entry corresponding to the specified literal key string or 0
   if there is no such a key in the storage */
public static native CvStringHashNode cvGetHashedKey( CvFileStorage fs, @Cast("const char*") BytePointer name,
                                        int len/*=-1*/,
                                        int create_missing/*=0*/);
public static native CvStringHashNode cvGetHashedKey( CvFileStorage fs, @Cast("const char*") BytePointer name);
public static native CvStringHashNode cvGetHashedKey( CvFileStorage fs, String name,
                                        int len/*=-1*/,
                                        int create_missing/*=0*/);
public static native CvStringHashNode cvGetHashedKey( CvFileStorage fs, String name);

/* returns file node with the specified key within the specified map
   (collection of named nodes) */
public static native CvFileNode cvGetRootFileNode( @Const CvFileStorage fs,
                                     int stream_index/*=0*/ );
public static native CvFileNode cvGetRootFileNode( @Const CvFileStorage fs );

/* returns file node with the specified key within the specified map
   (collection of named nodes) */
public static native CvFileNode cvGetFileNode( CvFileStorage fs, CvFileNode map,
                                 @Const CvStringHashNode key,
                                 int create_missing/*=0*/ );
public static native CvFileNode cvGetFileNode( CvFileStorage fs, CvFileNode map,
                                 @Const CvStringHashNode key );

/* this is a slower version of cvGetFileNode that takes the key as a literal string */
public static native CvFileNode cvGetFileNodeByName( @Const CvFileStorage fs,
                                       @Const CvFileNode map,
                                       @Cast("const char*") BytePointer name );
public static native CvFileNode cvGetFileNodeByName( @Const CvFileStorage fs,
                                       @Const CvFileNode map,
                                       String name );

public static native int cvReadInt( @Const CvFileNode node, int default_value/*=0*/ );
public static native int cvReadInt( @Const CvFileNode node );


public static native int cvReadIntByName( @Const CvFileStorage fs, @Const CvFileNode map,
                         @Cast("const char*") BytePointer name, int default_value/*=0*/ );
public static native int cvReadIntByName( @Const CvFileStorage fs, @Const CvFileNode map,
                         @Cast("const char*") BytePointer name );
public static native int cvReadIntByName( @Const CvFileStorage fs, @Const CvFileNode map,
                         String name, int default_value/*=0*/ );
public static native int cvReadIntByName( @Const CvFileStorage fs, @Const CvFileNode map,
                         String name );


public static native double cvReadReal( @Const CvFileNode node, double default_value/*=0.*/ );
public static native double cvReadReal( @Const CvFileNode node );


public static native double cvReadRealByName( @Const CvFileStorage fs, @Const CvFileNode map,
                        @Cast("const char*") BytePointer name, double default_value/*=0.*/ );
public static native double cvReadRealByName( @Const CvFileStorage fs, @Const CvFileNode map,
                        @Cast("const char*") BytePointer name );
public static native double cvReadRealByName( @Const CvFileStorage fs, @Const CvFileNode map,
                        String name, double default_value/*=0.*/ );
public static native double cvReadRealByName( @Const CvFileStorage fs, @Const CvFileNode map,
                        String name );


public static native @Cast("const char*") BytePointer cvReadString( @Const CvFileNode node,
                        @Cast("const char*") BytePointer default_value/*=NULL*/ );
public static native @Cast("const char*") BytePointer cvReadString( @Const CvFileNode node );
public static native String cvReadString( @Const CvFileNode node,
                        String default_value/*=NULL*/ );


public static native @Cast("const char*") BytePointer cvReadStringByName( @Const CvFileStorage fs, @Const CvFileNode map,
                        @Cast("const char*") BytePointer name, @Cast("const char*") BytePointer default_value/*=NULL*/ );
public static native @Cast("const char*") BytePointer cvReadStringByName( @Const CvFileStorage fs, @Const CvFileNode map,
                        @Cast("const char*") BytePointer name );
public static native String cvReadStringByName( @Const CvFileStorage fs, @Const CvFileNode map,
                        String name, String default_value/*=NULL*/ );
public static native String cvReadStringByName( @Const CvFileStorage fs, @Const CvFileNode map,
                        String name );


/* decodes standard or user-defined object and returns it */
public static native Pointer cvRead( CvFileStorage fs, CvFileNode node,
                        CvAttrList attributes/*=NULL*/);
public static native Pointer cvRead( CvFileStorage fs, CvFileNode node);

/* decodes standard or user-defined object and returns it */
public static native Pointer cvReadByName( CvFileStorage fs, @Const CvFileNode map,
                              @Cast("const char*") BytePointer name, CvAttrList attributes/*=NULL*/ );
public static native Pointer cvReadByName( CvFileStorage fs, @Const CvFileNode map,
                              @Cast("const char*") BytePointer name );
public static native Pointer cvReadByName( CvFileStorage fs, @Const CvFileNode map,
                              String name, CvAttrList attributes/*=NULL*/ );
public static native Pointer cvReadByName( CvFileStorage fs, @Const CvFileNode map,
                              String name );


/* starts reading data from sequence or scalar numeric node */
public static native void cvStartReadRawData( @Const CvFileStorage fs, @Const CvFileNode src,
                               CvSeqReader reader );

/* reads multiple numbers and stores them to array */
public static native void cvReadRawDataSlice( @Const CvFileStorage fs, CvSeqReader reader,
                               int count, Pointer dst, @Cast("const char*") BytePointer dt );
public static native void cvReadRawDataSlice( @Const CvFileStorage fs, CvSeqReader reader,
                               int count, Pointer dst, String dt );

/* combination of two previous functions for easier reading of whole sequences */
public static native void cvReadRawData( @Const CvFileStorage fs, @Const CvFileNode src,
                          Pointer dst, @Cast("const char*") BytePointer dt );
public static native void cvReadRawData( @Const CvFileStorage fs, @Const CvFileNode src,
                          Pointer dst, String dt );

/* writes a copy of file node to file storage */
public static native void cvWriteFileNode( CvFileStorage fs, @Cast("const char*") BytePointer new_node_name,
                            @Const CvFileNode node, int embed );
public static native void cvWriteFileNode( CvFileStorage fs, String new_node_name,
                            @Const CvFileNode node, int embed );

/* returns name of file node */
public static native @Cast("const char*") BytePointer cvGetFileNodeName( @Const CvFileNode node );

/*********************************** Adding own types ***********************************/

public static native void cvRegisterType( @Const CvTypeInfo info );
public static native void cvUnregisterType( @Cast("const char*") BytePointer type_name );
public static native void cvUnregisterType( String type_name );
public static native CvTypeInfo cvFirstType();
public static native CvTypeInfo cvFindType( @Cast("const char*") BytePointer type_name );
public static native CvTypeInfo cvFindType( String type_name );
public static native CvTypeInfo cvTypeOf( @Const Pointer struct_ptr );

/* universal functions */
public static native void cvRelease( @Cast("void**") PointerPointer struct_ptr );
public static native void cvRelease( @Cast("void**") @ByPtrPtr Pointer struct_ptr );
public static native Pointer cvClone( @Const Pointer struct_ptr );

/* simple API for reading/writing data */
public static native void cvSave( @Cast("const char*") BytePointer filename, @Const Pointer struct_ptr,
                    @Cast("const char*") BytePointer name/*=NULL*/,
                    @Cast("const char*") BytePointer comment/*=NULL*/,
                    @ByVal CvAttrList attributes/*=cvAttrList()*/);
public static native void cvSave( @Cast("const char*") BytePointer filename, @Const Pointer struct_ptr);
public static native void cvSave( String filename, @Const Pointer struct_ptr,
                    String name/*=NULL*/,
                    String comment/*=NULL*/,
                    @ByVal CvAttrList attributes/*=cvAttrList()*/);
public static native void cvSave( String filename, @Const Pointer struct_ptr);
public static native Pointer cvLoad( @Cast("const char*") BytePointer filename,
                     CvMemStorage memstorage/*=NULL*/,
                     @Cast("const char*") BytePointer name/*=NULL*/,
                     @Cast("const char**") PointerPointer real_name/*=NULL*/ );
public static native Pointer cvLoad( @Cast("const char*") BytePointer filename );
public static native Pointer cvLoad( @Cast("const char*") BytePointer filename,
                     CvMemStorage memstorage/*=NULL*/,
                     @Cast("const char*") BytePointer name/*=NULL*/,
                     @Cast("const char**") @ByPtrPtr BytePointer real_name/*=NULL*/ );
public static native Pointer cvLoad( String filename,
                     CvMemStorage memstorage/*=NULL*/,
                     String name/*=NULL*/,
                     @Cast("const char**") @ByPtrPtr ByteBuffer real_name/*=NULL*/ );
public static native Pointer cvLoad( String filename );
public static native Pointer cvLoad( @Cast("const char*") BytePointer filename,
                     CvMemStorage memstorage/*=NULL*/,
                     @Cast("const char*") BytePointer name/*=NULL*/,
                     @Cast("const char**") @ByPtrPtr byte[] real_name/*=NULL*/ );
public static native Pointer cvLoad( String filename,
                     CvMemStorage memstorage/*=NULL*/,
                     String name/*=NULL*/,
                     @Cast("const char**") @ByPtrPtr BytePointer real_name/*=NULL*/ );
public static native Pointer cvLoad( @Cast("const char*") BytePointer filename,
                     CvMemStorage memstorage/*=NULL*/,
                     @Cast("const char*") BytePointer name/*=NULL*/,
                     @Cast("const char**") @ByPtrPtr ByteBuffer real_name/*=NULL*/ );
public static native Pointer cvLoad( String filename,
                     CvMemStorage memstorage/*=NULL*/,
                     String name/*=NULL*/,
                     @Cast("const char**") @ByPtrPtr byte[] real_name/*=NULL*/ );

/*********************************** Measuring Execution Time ***************************/

/* helper functions for RNG initialization and accurate time measurement:
   uses internal clock counter on x86 */
public static native @Cast("int64") long cvGetTickCount( );
public static native double cvGetTickFrequency( );

/*********************************** CPU capabilities ***********************************/

public static native int cvCheckHardwareSupport(int feature);

/*********************************** Multi-Threading ************************************/

/* retrieve/set the number of threads used in OpenMP implementations */
public static native int cvGetNumThreads( );
public static native void cvSetNumThreads( int threads/*=0*/ );
public static native void cvSetNumThreads( );
/* get index of the thread being executed */
public static native int cvGetThreadNum( );


/********************************** Error Handling **************************************/

/* Get current OpenCV error status */
public static native int cvGetErrStatus( );

/* Sets error status silently */
public static native void cvSetErrStatus( int status );

public static final int CV_ErrModeLeaf =     0;   /* Print error and exit program */
public static final int CV_ErrModeParent =   1;   /* Print error and continue */
public static final int CV_ErrModeSilent =   2;   /* Don't print and continue */

/* Retrives current error processing mode */
public static native int cvGetErrMode( );

/* Sets error processing mode, returns previously used mode */
public static native int cvSetErrMode( int mode );

/* Sets error status and performs some additonal actions (displaying message box,
 writing message to stderr, terminating application etc.)
 depending on the current error mode */
public static native void cvError( int status, @Cast("const char*") BytePointer func_name,
                    @Cast("const char*") BytePointer err_msg, @Cast("const char*") BytePointer file_name, int line );
public static native void cvError( int status, String func_name,
                    String err_msg, String file_name, int line );

/* Retrieves textual description of the error given its code */
public static native @Cast("const char*") BytePointer cvErrorStr( int status );

/* Retrieves detailed information about the last error occured */
public static native int cvGetErrInfo( @Cast("const char**") PointerPointer errcode_desc, @Cast("const char**") PointerPointer description,
                        @Cast("const char**") PointerPointer filename, IntPointer line );
public static native int cvGetErrInfo( @Cast("const char**") @ByPtrPtr BytePointer errcode_desc, @Cast("const char**") @ByPtrPtr BytePointer description,
                        @Cast("const char**") @ByPtrPtr BytePointer filename, IntPointer line );
public static native int cvGetErrInfo( @Cast("const char**") @ByPtrPtr ByteBuffer errcode_desc, @Cast("const char**") @ByPtrPtr ByteBuffer description,
                        @Cast("const char**") @ByPtrPtr ByteBuffer filename, IntBuffer line );
public static native int cvGetErrInfo( @Cast("const char**") @ByPtrPtr byte[] errcode_desc, @Cast("const char**") @ByPtrPtr byte[] description,
                        @Cast("const char**") @ByPtrPtr byte[] filename, int[] line );

/* Maps IPP error codes to the counterparts from OpenCV */
public static native int cvErrorFromIppStatus( int ipp_status );

@Convention("CV_CDECL") public static class CvErrorCallback extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    CvErrorCallback(Pointer p) { super(p); }
    protected CvErrorCallback() { allocate(); }
    private native void allocate();
    public native int call( int status, @Cast("const char*") BytePointer func_name,
                                        @Cast("const char*") BytePointer err_msg, @Cast("const char*") BytePointer file_name, int line, Pointer userdata );
}

/* Assigns a new error-handling function */
public static native CvErrorCallback cvRedirectError( CvErrorCallback error_handler,
                                       Pointer userdata/*=NULL*/,
                                       @Cast("void**") PointerPointer prev_userdata/*=NULL*/ );
public static native CvErrorCallback cvRedirectError( CvErrorCallback error_handler );
public static native CvErrorCallback cvRedirectError( CvErrorCallback error_handler,
                                       Pointer userdata/*=NULL*/,
                                       @Cast("void**") @ByPtrPtr Pointer prev_userdata/*=NULL*/ );

/*
 Output to:
 cvNulDevReport - nothing
 cvStdErrReport - console(fprintf(stderr,...))
 cvGuiBoxReport - MessageBox(WIN32)
 */
public static native int cvNulDevReport( int status, @Cast("const char*") BytePointer func_name, @Cast("const char*") BytePointer err_msg,
                          @Cast("const char*") BytePointer file_name, int line, Pointer userdata );
public static native int cvNulDevReport( int status, String func_name, String err_msg,
                          String file_name, int line, Pointer userdata );

public static native int cvStdErrReport( int status, @Cast("const char*") BytePointer func_name, @Cast("const char*") BytePointer err_msg,
                          @Cast("const char*") BytePointer file_name, int line, Pointer userdata );
public static native int cvStdErrReport( int status, String func_name, String err_msg,
                          String file_name, int line, Pointer userdata );

public static native int cvGuiBoxReport( int status, @Cast("const char*") BytePointer func_name, @Cast("const char*") BytePointer err_msg,
                          @Cast("const char*") BytePointer file_name, int line, Pointer userdata );
public static native int cvGuiBoxReport( int status, String func_name, String err_msg,
                          String file_name, int line, Pointer userdata );

// #define OPENCV_ERROR(status,func,context)
// cvError((status),(func),(context),__FILE__,__LINE__)

// #define OPENCV_ASSERT(expr,func,context)
// {if (! (expr))
// {OPENCV_ERROR(CV_StsInternal,(func),(context));}}

// #define OPENCV_CALL( Func )
// {
// Func;
// }


/* CV_FUNCNAME macro defines icvFuncName constant which is used by CV_ERROR macro */
// #ifdef CV_NO_FUNC_NAMES
// #define CV_FUNCNAME( Name )
public static final String cvFuncName = "";
// #else
// #define CV_FUNCNAME( Name )
// static char cvFuncName[] = Name
// #endif


/*
 CV_ERROR macro unconditionally raises error with passed code and message.
 After raising error, control will be transferred to the exit label.
 */
// #define CV_ERROR( Code, Msg )
// {
//     cvError( (Code), cvFuncName, Msg, __FILE__, __LINE__ );
//     __CV_EXIT__;
// }

/*
 CV_CHECK macro checks error status after CV (or IPL)
 function call. If error detected, control will be transferred to the exit
 label.
 */
// #define CV_CHECK()
// {
//     if( cvGetErrStatus() < 0 )
//         CV_ERROR( CV_StsBackTrace, "Inner function failed." );
// }


/*
 CV_CALL macro calls CV (or IPL) function, checks error status and
 signals a error if the function failed. Useful in "parent node"
 error procesing mode
 */
// #define CV_CALL( Func )
// {
//     Func;
//     CV_CHECK();
// }


/* Runtime assertion macro */
// #define CV_ASSERT( Condition )
// {
//     if( !(Condition) )
//         CV_ERROR( CV_StsInternal, "Assertion: " #Condition " failed" );
// }

// #define __CV_BEGIN__       {
// #define __CV_END__         goto exit; exit: ; }
// #define __CV_EXIT__        goto exit

// #ifdef __cplusplus // extern "C"
// #endif

// #ifdef __cplusplus
// class for automatic module/RTTI data registration/unregistration
@NoOffset public static class CvType extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public CvType() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public CvType(Pointer p) { super(p); }

    public CvType( @Cast("const char*") BytePointer type_name,
                CvIsInstanceFunc is_instance, CvReleaseFunc release/*=0*/,
                CvReadFunc read/*=0*/, CvWriteFunc write/*=0*/, CvCloneFunc clone/*=0*/ ) { allocate(type_name, is_instance, release, read, write, clone); }
    private native void allocate( @Cast("const char*") BytePointer type_name,
                CvIsInstanceFunc is_instance, CvReleaseFunc release/*=0*/,
                CvReadFunc read/*=0*/, CvWriteFunc write/*=0*/, CvCloneFunc clone/*=0*/ );
    public CvType( @Cast("const char*") BytePointer type_name,
                CvIsInstanceFunc is_instance ) { allocate(type_name, is_instance); }
    private native void allocate( @Cast("const char*") BytePointer type_name,
                CvIsInstanceFunc is_instance );
    public CvType( String type_name,
                CvIsInstanceFunc is_instance, CvReleaseFunc release/*=0*/,
                CvReadFunc read/*=0*/, CvWriteFunc write/*=0*/, CvCloneFunc clone/*=0*/ ) { allocate(type_name, is_instance, release, read, write, clone); }
    private native void allocate( String type_name,
                CvIsInstanceFunc is_instance, CvReleaseFunc release/*=0*/,
                CvReadFunc read/*=0*/, CvWriteFunc write/*=0*/, CvCloneFunc clone/*=0*/ );
    public CvType( String type_name,
                CvIsInstanceFunc is_instance ) { allocate(type_name, is_instance); }
    private native void allocate( String type_name,
                CvIsInstanceFunc is_instance );
    public native CvTypeInfo info(); public native CvType info(CvTypeInfo info);

    
    
}

// #include "opencv2/core/utility.hpp"

/////////////////////////////////////////// glue ///////////////////////////////////////////

/** converts array (CvMat or IplImage) to cv::Mat */
@Namespace("cv") public static native @ByVal Mat cvarrToMat(@Const CvArr arr, @Cast("bool") boolean copyData/*=false*/,
                          @Cast("bool") boolean allowND/*=true*/, int coiMode/*=0*/,
                          @Cast("cv::AutoBuffer<double>*") Pointer buf/*=0*/);
@Namespace("cv") public static native @ByVal Mat cvarrToMat(@Const CvArr arr);

@Namespace("cv") public static native @ByVal Mat cvarrToMatND(@Const CvArr arr, @Cast("bool") boolean copyData/*=false*/, int coiMode/*=0*/);
@Namespace("cv") public static native @ByVal Mat cvarrToMatND(@Const CvArr arr);


/** extracts Channel of Interest from CvMat or IplImage and makes cv::Mat out of it. */
@Namespace("cv") public static native void extractImageCOI(@Const CvArr arr, @ByVal Mat coiimg, int coi/*=-1*/);
@Namespace("cv") public static native void extractImageCOI(@Const CvArr arr, @ByVal Mat coiimg);
/** inserts single-channel cv::Mat into a multi-channel CvMat or IplImage */
@Namespace("cv") public static native void insertImageCOI(@ByVal Mat coiimg, CvArr arr, int coi/*=-1*/);
@Namespace("cv") public static native void insertImageCOI(@ByVal Mat coiimg, CvArr arr);



////// specialized implementations of DefaultDeleter::operator() for classic OpenCV types //////







////////////// convenient wrappers for operating old-style dynamic structures //////////////

/**
 Template Sequence Class derived from CvSeq

 The class provides more convenient access to sequence elements,
 STL-style operations and iterators.

 \note The class is targeted for simple data types,
    i.e. no constructors or destructors
    are called for the sequence elements.
*/


/**
 STL-style Sequence Iterator inherited from the CvSeqReader structure
*/



// bridge C++ => C Seq API
@Namespace("cv") public static native @Cast("schar*") BytePointer seqPush( CvSeq seq, @Const Pointer element/*=0*/);
@Namespace("cv") public static native @Cast("schar*") BytePointer seqPush( CvSeq seq);
@Namespace("cv") public static native @Cast("schar*") BytePointer seqPushFront( CvSeq seq, @Const Pointer element/*=0*/);
@Namespace("cv") public static native @Cast("schar*") BytePointer seqPushFront( CvSeq seq);
@Namespace("cv") public static native void seqPop( CvSeq seq, Pointer element/*=0*/);
@Namespace("cv") public static native void seqPop( CvSeq seq);
@Namespace("cv") public static native void seqPopFront( CvSeq seq, Pointer element/*=0*/);
@Namespace("cv") public static native void seqPopFront( CvSeq seq);

@Namespace("cv") public static native void seqRemove( CvSeq seq, int index );
@Namespace("cv") public static native void clearSeq( CvSeq seq );
@Namespace("cv") public static native @Cast("schar*") BytePointer getSeqElem( @Const CvSeq seq, int index );
@Namespace("cv") public static native void seqRemoveSlice( CvSeq seq, @ByVal CvSlice slice );
@Namespace("cv") public static native void seqInsertSlice( CvSeq seq, int before_index, @Const CvArr from_arr );


























































































 // cv

// #endif

// #endif


// Parsed from <opencv2/core/utility.hpp>

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

// #ifndef __OPENCV_CORE_UTILITY_H__
// #define __OPENCV_CORE_UTILITY_H__

// #ifndef __cplusplus
// #endif

// #include "opencv2/core.hpp"

// #ifdef CV_COLLECT_IMPL_DATA
// #else
// #define CV_IMPL_ADD(impl)
// #endif

/**
 Automatically Allocated Buffer Class

 The class is used for temporary buffers in functions and methods.
 If a temporary buffer is usually small (a few K's of memory),
 but its size depends on the parameters, it makes sense to create a small
 fixed-size array on stack and use it if it's large enough. If the required buffer size
 is larger than the fixed size, another buffer of sufficient size is allocated dynamically
 and released after the processing. Therefore, in typical cases, when the buffer size is small,
 there is no overhead associated with malloc()/free().
 At the same time, there is no limit on the size of processed data.

 This is what AutoBuffer does. The template takes 2 parameters - type of the buffer elements and
 the number of stack-allocated elements. Here is how the class is used:

 \code
 void my_func(const cv::Mat& m)
 {
    cv::AutoBuffer<float> buf; // create automatic buffer containing 1000 floats

    buf.allocate(m.rows); // if m.rows <= 1000, the pre-allocated buffer is used,
                          // otherwise the buffer of "m.rows" floats will be allocated
                          // dynamically and deallocated in cv::AutoBuffer destructor
    ...
 }
 \endcode
*/

/** Sets/resets the break-on-error mode.

/**
  When the break-on-error mode is set, the default error handler
  issues a hardware exception, which can make debugging more convenient.

  \return the previous state
 */
@Namespace("cv") public static native @Cast("bool") boolean setBreakOnError(@Cast("bool") boolean flag); public static class ErrorCallback extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    ErrorCallback(Pointer p) { super(p); }
    protected ErrorCallback() { allocate(); }
    private native void allocate();
    public native int call( int status, @Cast("const char*") BytePointer func_name,
                                       @Cast("const char*") BytePointer err_msg, @Cast("const char*") BytePointer file_name,
                                       int line, Pointer userdata );
}

/** Sets the new error handler and the optional user data.

/**
  The function sets the new error handler, called from cv::error().

  \param errCallback the new error handler. If NULL, the default error handler is used.
  \param userdata the optional user data pointer, passed to the callback.
  \param prevUserdata the optional output parameter where the previous user data pointer is stored

  \return the previous error handler
*/
@Namespace("cv") public static native ErrorCallback redirectError( ErrorCallback errCallback, Pointer userdata/*=0*/, @Cast("void**") PointerPointer prevUserdata/*=0*/);
@Namespace("cv") public static native ErrorCallback redirectError( ErrorCallback errCallback);
@Namespace("cv") public static native ErrorCallback redirectError( ErrorCallback errCallback, Pointer userdata/*=0*/, @Cast("void**") @ByPtrPtr Pointer prevUserdata/*=0*/);

@Namespace("cv") public static native @Str BytePointer format( @Cast("const char*") BytePointer fmt );
@Namespace("cv") public static native @Str String format( String fmt );
@Namespace("cv") public static native @Str BytePointer tempfile( @Cast("const char*") BytePointer suffix/*=0*/);
@Namespace("cv") public static native @Str BytePointer tempfile();
@Namespace("cv") public static native @Str String tempfile( String suffix/*=0*/);
@Namespace("cv") public static native void glob(@Str BytePointer pattern, @ByRef StringVector result, @Cast("bool") boolean recursive/*=false*/);
@Namespace("cv") public static native void glob(@Str BytePointer pattern, @ByRef StringVector result);
@Namespace("cv") public static native void glob(@Str String pattern, @ByRef StringVector result, @Cast("bool") boolean recursive/*=false*/);
@Namespace("cv") public static native void glob(@Str String pattern, @ByRef StringVector result);
@Namespace("cv") public static native void setNumThreads(int nthreads);
@Namespace("cv") public static native int getNumThreads();
@Namespace("cv") public static native int getThreadNum();

@Namespace("cv") public static native @Str BytePointer getBuildInformation();

/** Returns the number of ticks.

/**
  The function returns the number of ticks since the certain event (e.g. when the machine was turned on).
  It can be used to initialize cv::RNG or to measure a function execution time by reading the tick count
  before and after the function call. The granularity of ticks depends on the hardware and OS used. Use
  cv::getTickFrequency() to convert ticks to seconds.
*/
@Namespace("cv") public static native @Cast("int64") long getTickCount();

/**
  Returns the number of ticks per seconds.

  The function returns the number of ticks (as returned by cv::getTickCount()) per second.
  The following code computes the execution time in milliseconds:

  \code
  double exec_time = (double)getTickCount();
  // do something ...
  exec_time = ((double)getTickCount() - exec_time)*1000./getTickFrequency();
  \endcode
*/
@Namespace("cv") public static native double getTickFrequency();

/**
  Returns the number of CPU ticks.

  On platforms where the feature is available, the function returns the number of CPU ticks
  since the certain event (normally, the system power-on moment). Using this function
  one can accurately measure the execution time of very small code fragments,
  for which cv::getTickCount() granularity is not enough.
*/
@Namespace("cv") public static native @Cast("int64") long getCPUTickCount();

/** Available CPU features. Currently, the following features are recognized: */
/** enum cv:: */
public static final int
      CPU_MMX       = 1,
      CPU_SSE       = 2,
      CPU_SSE2      = 3,
      CPU_SSE3      = 4,
      CPU_SSSE3     = 5,
      CPU_SSE4_1    = 6,
      CPU_SSE4_2    = 7,
      CPU_POPCNT    = 8,
      CPU_AVX       = 10,
      CPU_NEON      = 11;
// remember to keep this list identical to the one in cvdef.h

/**
  Returns SSE etc. support status

  The function returns true if certain hardware features are available.

  \note {Note that the function output is not static. Once you called cv::useOptimized(false),
  most of the hardware acceleration is disabled and thus the function will returns false,
  until you call cv::useOptimized(true)}
*/
@Namespace("cv") public static native @Cast("bool") boolean checkHardwareSupport(int feature);

/** returns the number of CPUs (including hyper-threading) */
@Namespace("cv") public static native int getNumberOfCPUs();


/**
  Aligns pointer by the certain number of bytes

  This small inline function aligns the pointer by the certian number of bytes by shifting
  it forward by 0 or a positive offset.
*/

/**
  Aligns buffer size by the certain number of bytes

  This small inline function aligns a buffer size by the certian number of bytes by enlarging it.
*/
@Namespace("cv") public static native @Cast("size_t") long alignSize(@Cast("size_t") long sz, int n);

/**
  Turns on/off available optimization

  The function turns on or off the optimized code in OpenCV. Some optimization can not be enabled
  or disabled, but, for example, most of SSE code in OpenCV can be temporarily turned on or off this way.

  \note{Since optimization may imply using special data structures, it may be unsafe
  to call this function anywhere in the code. Instead, call it somewhere at the top level.}
*/
@Namespace("cv") public static native void setUseOptimized(@Cast("bool") boolean onoff);

/**
  Returns the current optimization status

  The function returns the current optimization status, which is controlled by cv::setUseOptimized().
*/
@Namespace("cv") public static native @Cast("bool") boolean useOptimized();

@Namespace("cv") public static native @Cast("size_t") long getElemSize(int type);

/////////////////////////////// Parallel Primitives //////////////////////////////////

// a base body class
@Namespace("cv") public static class ParallelLoopBody extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public ParallelLoopBody() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public ParallelLoopBody(Pointer p) { super(p); }

    public native @Name("operator()") void apply(@Const @ByRef Range range);
}

@Namespace("cv") public static native void parallel_for_(@Const @ByRef Range range, @Const @ByRef ParallelLoopBody body, double nstripes/*=-1.*/);
@Namespace("cv") public static native void parallel_for_(@Const @ByRef Range range, @Const @ByRef ParallelLoopBody body);

/////////////////////////////// forEach method of cv::Mat ////////////////////////////


/////////////////////////// Synchronization Primitives ///////////////////////////////

@Namespace("cv") @NoOffset public static class Mutex extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Mutex(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Mutex(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Mutex position(int position) {
        return (Mutex)super.position(position);
    }

    public Mutex() { allocate(); }
    private native void allocate();
    public Mutex(@Const @ByRef Mutex m) { allocate(m); }
    private native void allocate(@Const @ByRef Mutex m);
    public native @ByRef @Name("operator=") Mutex put(@Const @ByRef Mutex m);

    public native void lock();
    public native @Cast("bool") boolean trylock();
    public native void unlock();

    @Opaque public static class Impl extends Pointer {
        /** Empty constructor. */
        public Impl() { }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public Impl(Pointer p) { super(p); }
    }
}

@Namespace("cv") @NoOffset public static class AutoLock extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public AutoLock() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public AutoLock(Pointer p) { super(p); }

    public AutoLock(@ByRef Mutex m) { allocate(m); }
    private native void allocate(@ByRef Mutex m);
}

@Namespace("cv") @NoOffset public static class TLSDataContainer extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public TLSDataContainer() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public TLSDataContainer(Pointer p) { super(p); }

    public native Pointer createDataInstance();
    public native void deleteDataInstance(Pointer data);

    public native Pointer getData();
}

// The CommandLineParser class is designed for command line arguments parsing

/////////////////////////////// AutoBuffer implementation ////////////////////////////////////////























// #ifndef OPENCV_NOSTL


// #endif // OPENCV_NOSTL

 //namespace cv

// #endif //__OPENCV_CORE_UTILITY_H__


// Parsed from <opencv2/core/types.hpp>

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

// #ifndef __OPENCV_CORE_TYPES_HPP__
// #define __OPENCV_CORE_TYPES_HPP__

// #ifndef __cplusplus
// #endif

// #include <climits>
// #include <cfloat>
// #include <vector>

// #include "opencv2/core/cvdef.h"
// #include "opencv2/core/cvstd.hpp"
// #include "opencv2/core/matx.hpp"

//////////////////////////////// Complex //////////////////////////////

/**
  A complex number class.

  The template class is similar and compatible with std::complex, however it provides slightly
  more convenient access to the real and imaginary parts using through the simple field access, as opposite
  to std::complex::real() and std::complex::imag().
*/



//////////////////////////////// Point_ ////////////////////////////////

/**
  template 2D point class.

  The class defines a point in 2D space. Data type of the point coordinates is specified
  as a template parameter. There are a few shorter aliases available for user convenience.
  See cv::Point, cv::Point2i, cv::Point2f and cv::Point2d.
*/
@Name("cv::Point_<int>") @NoOffset public static class Point extends IntPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Point(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Point(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Point position(int position) {
        return (Point)super.position(position);
    }


    // various constructors
    public Point() { allocate(); }
    private native void allocate();
    public Point(int _x, int _y) { allocate(_x, _y); }
    private native void allocate(int _x, int _y);
    public Point(@Const @ByRef Point pt) { allocate(pt); }
    private native void allocate(@Const @ByRef Point pt);
    public Point(@Const @ByRef Size sz) { allocate(sz); }
    private native void allocate(@Const @ByRef Size sz);

    public native @ByRef @Name("operator=") Point put(@Const @ByRef Point pt);
    /** conversion to another data type */

    /** conversion to the old-style C structures */

    /** dot product */
    public native int dot(@Const @ByRef Point pt);
    /** dot product computed in double-precision arithmetics */
    public native double ddot(@Const @ByRef Point pt);
    /** cross-product */
    public native double cross(@Const @ByRef Point pt);
    /** checks whether the point is inside the specified rectangle */
    public native @Cast("bool") boolean inside(@Const @ByRef Rect r);

    public native int x(); public native Point x(int x);
    public native int y(); public native Point y(int y); //< the point coordinates
}
@Name("cv::Point_<float>") @NoOffset public static class Point2f extends FloatPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Point2f(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Point2f(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Point2f position(int position) {
        return (Point2f)super.position(position);
    }


    // various constructors
    public Point2f() { allocate(); }
    private native void allocate();
    public Point2f(float _x, float _y) { allocate(_x, _y); }
    private native void allocate(float _x, float _y);
    public Point2f(@Const @ByRef Point2f pt) { allocate(pt); }
    private native void allocate(@Const @ByRef Point2f pt);
    public Point2f(@Const @ByRef Size2f sz) { allocate(sz); }
    private native void allocate(@Const @ByRef Size2f sz);

    public native @ByRef @Name("operator=") Point2f put(@Const @ByRef Point2f pt);
    /** conversion to another data type */

    /** conversion to the old-style C structures */

    /** dot product */
    public native float dot(@Const @ByRef Point2f pt);
    /** dot product computed in double-precision arithmetics */
    public native double ddot(@Const @ByRef Point2f pt);
    /** cross-product */
    public native double cross(@Const @ByRef Point2f pt);
    /** checks whether the point is inside the specified rectangle */
    public native @Cast("bool") boolean inside(@Const @ByRef Rectf r);

    public native float x(); public native Point2f x(float x);
    public native float y(); public native Point2f y(float y); //< the point coordinates
}
@Name("cv::Point_<double>") @NoOffset public static class Point2d extends DoublePointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Point2d(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Point2d(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Point2d position(int position) {
        return (Point2d)super.position(position);
    }


    // various constructors
    public Point2d() { allocate(); }
    private native void allocate();
    public Point2d(double _x, double _y) { allocate(_x, _y); }
    private native void allocate(double _x, double _y);
    public Point2d(@Const @ByRef Point2d pt) { allocate(pt); }
    private native void allocate(@Const @ByRef Point2d pt);
    public Point2d(@Const @ByRef Size2d sz) { allocate(sz); }
    private native void allocate(@Const @ByRef Size2d sz);

    public native @ByRef @Name("operator=") Point2d put(@Const @ByRef Point2d pt);
    /** conversion to another data type */

    /** conversion to the old-style C structures */

    /** dot product */
    public native double dot(@Const @ByRef Point2d pt);
    /** dot product computed in double-precision arithmetics */
    public native double ddot(@Const @ByRef Point2d pt);
    /** cross-product */
    public native double cross(@Const @ByRef Point2d pt);
    /** checks whether the point is inside the specified rectangle */
    public native @Cast("bool") boolean inside(@Const @ByRef Rectd r);

    public native double x(); public native Point2d x(double x);
    public native double y(); public native Point2d y(double y); //< the point coordinates
}



//////////////////////////////// Point3_ ////////////////////////////////

/**
  template 3D point class.

  The class defines a point in 3D space. Data type of the point coordinates is specified
  as a template parameter.

  \see cv::Point3i, cv::Point3f and cv::Point3d
*/
@Name("cv::Point3_<int>") @NoOffset public static class Point3i extends IntPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Point3i(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Point3i(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Point3i position(int position) {
        return (Point3i)super.position(position);
    }


    // various constructors
    public Point3i() { allocate(); }
    private native void allocate();
    public Point3i(int _x, int _y, int _z) { allocate(_x, _y, _z); }
    private native void allocate(int _x, int _y, int _z);
    public Point3i(@Const @ByRef Point3i pt) { allocate(pt); }
    private native void allocate(@Const @ByRef Point3i pt);
    public Point3i(@Const @ByRef Point pt) { allocate(pt); }
    private native void allocate(@Const @ByRef Point pt);

    public native @ByRef @Name("operator=") Point3i put(@Const @ByRef Point3i pt);
    /** conversion to another data type */
    /** conversion to cv::Vec<> */

    /** dot product */
    public native int dot(@Const @ByRef Point3i pt);
    /** dot product computed in double-precision arithmetics */
    public native double ddot(@Const @ByRef Point3i pt);
    /** cross product of the 2 3D points */
    public native @ByVal Point3i cross(@Const @ByRef Point3i pt);

    public native int x(); public native Point3i x(int x);
    public native int y(); public native Point3i y(int y);
    public native int z(); public native Point3i z(int z); //< the point coordinates
}
@Name("cv::Point3_<float>") @NoOffset public static class Point3f extends FloatPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Point3f(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Point3f(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Point3f position(int position) {
        return (Point3f)super.position(position);
    }


    // various constructors
    public Point3f() { allocate(); }
    private native void allocate();
    public Point3f(float _x, float _y, float _z) { allocate(_x, _y, _z); }
    private native void allocate(float _x, float _y, float _z);
    public Point3f(@Const @ByRef Point3f pt) { allocate(pt); }
    private native void allocate(@Const @ByRef Point3f pt);
    public Point3f(@Const @ByRef Point2f pt) { allocate(pt); }
    private native void allocate(@Const @ByRef Point2f pt);

    public native @ByRef @Name("operator=") Point3f put(@Const @ByRef Point3f pt);
    /** conversion to another data type */
    /** conversion to cv::Vec<> */

    /** dot product */
    public native float dot(@Const @ByRef Point3f pt);
    /** dot product computed in double-precision arithmetics */
    public native double ddot(@Const @ByRef Point3f pt);
    /** cross product of the 2 3D points */
    public native @ByVal Point3f cross(@Const @ByRef Point3f pt);

    public native float x(); public native Point3f x(float x);
    public native float y(); public native Point3f y(float y);
    public native float z(); public native Point3f z(float z); //< the point coordinates
}
@Name("cv::Point3_<double>") @NoOffset public static class Point3d extends DoublePointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Point3d(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Point3d(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Point3d position(int position) {
        return (Point3d)super.position(position);
    }


    // various constructors
    public Point3d() { allocate(); }
    private native void allocate();
    public Point3d(double _x, double _y, double _z) { allocate(_x, _y, _z); }
    private native void allocate(double _x, double _y, double _z);
    public Point3d(@Const @ByRef Point3d pt) { allocate(pt); }
    private native void allocate(@Const @ByRef Point3d pt);
    public Point3d(@Const @ByRef Point2d pt) { allocate(pt); }
    private native void allocate(@Const @ByRef Point2d pt);

    public native @ByRef @Name("operator=") Point3d put(@Const @ByRef Point3d pt);
    /** conversion to another data type */
    /** conversion to cv::Vec<> */

    /** dot product */
    public native double dot(@Const @ByRef Point3d pt);
    /** dot product computed in double-precision arithmetics */
    public native double ddot(@Const @ByRef Point3d pt);
    /** cross product of the 2 3D points */
    public native @ByVal Point3d cross(@Const @ByRef Point3d pt);

    public native double x(); public native Point3d x(double x);
    public native double y(); public native Point3d y(double y);
    public native double z(); public native Point3d z(double z); //< the point coordinates
}



//////////////////////////////// Size_ ////////////////////////////////

/**
  The 2D size class

  The class represents the size of a 2D rectangle, image size, matrix size etc.
  Normally, cv::Size ~ cv::Size_<int> is used.
*/
@Name("cv::Size_<int>") @NoOffset public static class Size extends IntPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Size(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Size(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Size position(int position) {
        return (Size)super.position(position);
    }


    /** various constructors */
    public Size() { allocate(); }
    private native void allocate();
    public Size(int _width, int _height) { allocate(_width, _height); }
    private native void allocate(int _width, int _height);
    public Size(@Const @ByRef Size sz) { allocate(sz); }
    private native void allocate(@Const @ByRef Size sz);
    public Size(@Const @ByRef Point pt) { allocate(pt); }
    private native void allocate(@Const @ByRef Point pt);

    public native @ByRef @Name("operator=") Size put(@Const @ByRef Size sz);
    /** the area (width*height) */
    public native int area();

    /** conversion of another data type. */

    public native int width(); public native Size width(int width);
    public native int height(); public native Size height(int height); // the width and the height
}
@Name("cv::Size_<float>") @NoOffset public static class Size2f extends FloatPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Size2f(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Size2f(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Size2f position(int position) {
        return (Size2f)super.position(position);
    }


    /** various constructors */
    public Size2f() { allocate(); }
    private native void allocate();
    public Size2f(float _width, float _height) { allocate(_width, _height); }
    private native void allocate(float _width, float _height);
    public Size2f(@Const @ByRef Size2f sz) { allocate(sz); }
    private native void allocate(@Const @ByRef Size2f sz);
    public Size2f(@Const @ByRef Point2f pt) { allocate(pt); }
    private native void allocate(@Const @ByRef Point2f pt);

    public native @ByRef @Name("operator=") Size2f put(@Const @ByRef Size2f sz);
    /** the area (width*height) */
    public native float area();

    /** conversion of another data type. */

    public native float width(); public native Size2f width(float width);
    public native float height(); public native Size2f height(float height); // the width and the height
}
@Name("cv::Size_<double>") @NoOffset public static class Size2d extends DoublePointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Size2d(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Size2d(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Size2d position(int position) {
        return (Size2d)super.position(position);
    }


    /** various constructors */
    public Size2d() { allocate(); }
    private native void allocate();
    public Size2d(double _width, double _height) { allocate(_width, _height); }
    private native void allocate(double _width, double _height);
    public Size2d(@Const @ByRef Size2d sz) { allocate(sz); }
    private native void allocate(@Const @ByRef Size2d sz);
    public Size2d(@Const @ByRef Point2d pt) { allocate(pt); }
    private native void allocate(@Const @ByRef Point2d pt);

    public native @ByRef @Name("operator=") Size2d put(@Const @ByRef Size2d sz);
    /** the area (width*height) */
    public native double area();

    /** conversion of another data type. */

    public native double width(); public native Size2d width(double width);
    public native double height(); public native Size2d height(double height); // the width and the height
}



//////////////////////////////// Rect_ ////////////////////////////////

/**
  The 2D up-right rectangle class

  The class represents a 2D rectangle with coordinates of the specified data type.
  Normally, cv::Rect ~ cv::Rect_<int> is used.
*/
@Name("cv::Rect_<int>") @NoOffset public static class Rect extends IntPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Rect(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Rect(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Rect position(int position) {
        return (Rect)super.position(position);
    }


    /** various constructors */
    public Rect() { allocate(); }
    private native void allocate();
    public Rect(int _x, int _y, int _width, int _height) { allocate(_x, _y, _width, _height); }
    private native void allocate(int _x, int _y, int _width, int _height);
    public Rect(@Const @ByRef Rect r) { allocate(r); }
    private native void allocate(@Const @ByRef Rect r);
    public Rect(@Const @ByRef Point org, @Const @ByRef Size sz) { allocate(org, sz); }
    private native void allocate(@Const @ByRef Point org, @Const @ByRef Size sz);
    public Rect(@Const @ByRef Point pt1, @Const @ByRef Point pt2) { allocate(pt1, pt2); }
    private native void allocate(@Const @ByRef Point pt1, @Const @ByRef Point pt2);

    public native @ByRef @Name("operator=") Rect put( @Const @ByRef Rect r );
    /** the top-left corner */
    public native @ByVal Point tl();
    /** the bottom-right corner */
    public native @ByVal Point br();

    /** size (width, height) of the rectangle */
    public native @ByVal Size size();
    /** area (width*height) of the rectangle */
    public native int area();

    /** conversion to another data type */

    /** checks whether the rectangle contains the point */
    public native @Cast("bool") boolean contains(@Const @ByRef Point pt);

    public native int x(); public native Rect x(int x);
    public native int y(); public native Rect y(int y);
    public native int width(); public native Rect width(int width);
    public native int height(); public native Rect height(int height); //< the top-left corner, as well as width and height of the rectangle
}
@Name("cv::Rect_<float>") @NoOffset public static class Rectf extends FloatPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Rectf(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Rectf(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Rectf position(int position) {
        return (Rectf)super.position(position);
    }


    /** various constructors */
    public Rectf() { allocate(); }
    private native void allocate();
    public Rectf(float _x, float _y, float _width, float _height) { allocate(_x, _y, _width, _height); }
    private native void allocate(float _x, float _y, float _width, float _height);
    public Rectf(@Const @ByRef Rectf r) { allocate(r); }
    private native void allocate(@Const @ByRef Rectf r);
    public Rectf(@Const @ByRef Point2f org, @Const @ByRef Size2f sz) { allocate(org, sz); }
    private native void allocate(@Const @ByRef Point2f org, @Const @ByRef Size2f sz);
    public Rectf(@Const @ByRef Point2f pt1, @Const @ByRef Point2f pt2) { allocate(pt1, pt2); }
    private native void allocate(@Const @ByRef Point2f pt1, @Const @ByRef Point2f pt2);

    public native @ByRef @Name("operator=") Rectf put( @Const @ByRef Rectf r );
    /** the top-left corner */
    public native @ByVal Point2f tl();
    /** the bottom-right corner */
    public native @ByVal Point2f br();

    /** size (width, height) of the rectangle */
    public native @ByVal Size2f size();
    /** area (width*height) of the rectangle */
    public native float area();

    /** conversion to another data type */

    /** checks whether the rectangle contains the point */
    public native @Cast("bool") boolean contains(@Const @ByRef Point2f pt);

    public native float x(); public native Rectf x(float x);
    public native float y(); public native Rectf y(float y);
    public native float width(); public native Rectf width(float width);
    public native float height(); public native Rectf height(float height); //< the top-left corner, as well as width and height of the rectangle
}
@Name("cv::Rect_<double>") @NoOffset public static class Rectd extends DoublePointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Rectd(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Rectd(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Rectd position(int position) {
        return (Rectd)super.position(position);
    }


    /** various constructors */
    public Rectd() { allocate(); }
    private native void allocate();
    public Rectd(double _x, double _y, double _width, double _height) { allocate(_x, _y, _width, _height); }
    private native void allocate(double _x, double _y, double _width, double _height);
    public Rectd(@Const @ByRef Rectd r) { allocate(r); }
    private native void allocate(@Const @ByRef Rectd r);
    public Rectd(@Const @ByRef Point2d org, @Const @ByRef Size2d sz) { allocate(org, sz); }
    private native void allocate(@Const @ByRef Point2d org, @Const @ByRef Size2d sz);
    public Rectd(@Const @ByRef Point2d pt1, @Const @ByRef Point2d pt2) { allocate(pt1, pt2); }
    private native void allocate(@Const @ByRef Point2d pt1, @Const @ByRef Point2d pt2);

    public native @ByRef @Name("operator=") Rectd put( @Const @ByRef Rectd r );
    /** the top-left corner */
    public native @ByVal Point2d tl();
    /** the bottom-right corner */
    public native @ByVal Point2d br();

    /** size (width, height) of the rectangle */
    public native @ByVal Size2d size();
    /** area (width*height) of the rectangle */
    public native double area();

    /** conversion to another data type */

    /** checks whether the rectangle contains the point */
    public native @Cast("bool") boolean contains(@Const @ByRef Point2d pt);

    public native double x(); public native Rectd x(double x);
    public native double y(); public native Rectd y(double y);
    public native double width(); public native Rectd width(double width);
    public native double height(); public native Rectd height(double height); //< the top-left corner, as well as width and height of the rectangle
}



///////////////////////////// RotatedRect /////////////////////////////

/**
  The rotated 2D rectangle.

  The class represents rotated (i.e. not up-right) rectangles on a plane.
  Each rectangle is described by the center point (mass center), length of each side
  (represented by cv::Size2f structure) and the rotation angle in degrees.
*/
@Namespace("cv") @NoOffset public static class RotatedRect extends FloatPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public RotatedRect(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public RotatedRect(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public RotatedRect position(int position) {
        return (RotatedRect)super.position(position);
    }

    /** various constructors */
    public RotatedRect() { allocate(); }
    private native void allocate();
    public RotatedRect(@Const @ByRef Point2f center, @Const @ByRef Size2f size, float angle) { allocate(center, size, angle); }
    private native void allocate(@Const @ByRef Point2f center, @Const @ByRef Size2f size, float angle);
    public RotatedRect(@Const @ByRef Point2f point1, @Const @ByRef Point2f point2, @Const @ByRef Point2f point3) { allocate(point1, point2, point3); }
    private native void allocate(@Const @ByRef Point2f point1, @Const @ByRef Point2f point2, @Const @ByRef Point2f point3);

    /** returns 4 vertices of the rectangle */
    public native void points(Point2f pts);
    /** returns the minimal up-right rectangle containing the rotated rectangle */
    public native @ByVal Rect boundingRect();

    public native @ByRef Point2f center(); public native RotatedRect center(Point2f center); //< the rectangle mass center
    public native @ByRef Size2f size(); public native RotatedRect size(Size2f size);    //< width and height of the rectangle
    public native float angle(); public native RotatedRect angle(float angle);    //< the rotation angle. When the angle is 0, 90, 180, 270 etc., the rectangle becomes an up-right rectangle.
}



//////////////////////////////// Range /////////////////////////////////

/**
   The 2D range class

   This is the class used to specify a continuous subsequence, i.e. part of a contour, or a column span in a matrix.
*/
@Namespace("cv") @NoOffset public static class Range extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Range(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Range(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Range position(int position) {
        return (Range)super.position(position);
    }

    public Range() { allocate(); }
    private native void allocate();
    public Range(int _start, int _end) { allocate(_start, _end); }
    private native void allocate(int _start, int _end);
    public native int size();
    public native @Cast("bool") boolean empty();
    public static native @ByVal Range all();

    public native int start(); public native Range start(int start);
    public native int end(); public native Range end(int end);
}



//////////////////////////////// Scalar_ ///////////////////////////////

/**
   The template scalar class.

   This is partially specialized cv::Vec class with the number of elements = 4, i.e. a short vector of four elements.
   Normally, cv::Scalar ~ cv::Scalar_<double> is used.
*/
@Name("cv::Scalar_<double>") public static class Scalar extends AbstractScalar {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Scalar(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Scalar(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Scalar position(int position) {
        return (Scalar)super.position(position);
    }

    /** various constructors */
    public Scalar() { allocate(); }
    private native void allocate();
    public Scalar(double v0, double v1, double v2/*=0*/, double v3/*=0*/) { allocate(v0, v1, v2, v3); }
    private native void allocate(double v0, double v1, double v2/*=0*/, double v3/*=0*/);
    public Scalar(double v0, double v1) { allocate(v0, v1); }
    private native void allocate(double v0, double v1);
    public Scalar(double v0) { allocate(v0); }
    private native void allocate(double v0);

    /** returns a scalar with all elements set to v0 */
    public static native @ByVal Scalar all(double v0);

    /** conversion to another data type */

    /** per-element product */
    public native @ByVal Scalar mul(@Const @ByRef Scalar a, double scale/*=1*/ );
    public native @ByVal Scalar mul(@Const @ByRef Scalar a );

    // returns (v0, -v1, -v2, -v3)
    public native @ByVal Scalar conj();

    // returns true iff v1 == v2 == v3 == 0
    public native @Cast("bool") boolean isReal();
}



/////////////////////////////// KeyPoint ////////////////////////////////

/**
 The Keypoint Class

 The class instance stores a keypoint, i.e. a point feature found by one of many available keypoint detectors, such as
 Harris corner detector, cv::FAST, cv::StarDetector, cv::SURF, cv::SIFT, cv::LDetector etc.

 The keypoint is characterized by the 2D position, scale
 (proportional to the diameter of the neighborhood that needs to be taken into account),
 orientation and some other parameters. The keypoint neighborhood is then analyzed by another algorithm that builds a descriptor
 (usually represented as a feature vector). The keypoints representing the same object in different images can then be matched using
 cv::KDTree or another method.
*/
@Namespace("cv") @NoOffset public static class KeyPoint extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public KeyPoint(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public KeyPoint(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public KeyPoint position(int position) {
        return (KeyPoint)super.position(position);
    }

    /** the default constructor */
    public KeyPoint() { allocate(); }
    private native void allocate();
    /** the full constructor */
    public KeyPoint(@ByVal Point2f _pt, float _size, float _angle/*=-1*/, float _response/*=0*/, int _octave/*=0*/, int _class_id/*=-1*/) { allocate(_pt, _size, _angle, _response, _octave, _class_id); }
    private native void allocate(@ByVal Point2f _pt, float _size, float _angle/*=-1*/, float _response/*=0*/, int _octave/*=0*/, int _class_id/*=-1*/);
    public KeyPoint(@ByVal Point2f _pt, float _size) { allocate(_pt, _size); }
    private native void allocate(@ByVal Point2f _pt, float _size);
    /** another form of the full constructor */
    public KeyPoint(float x, float y, float _size, float _angle/*=-1*/, float _response/*=0*/, int _octave/*=0*/, int _class_id/*=-1*/) { allocate(x, y, _size, _angle, _response, _octave, _class_id); }
    private native void allocate(float x, float y, float _size, float _angle/*=-1*/, float _response/*=0*/, int _octave/*=0*/, int _class_id/*=-1*/);
    public KeyPoint(float x, float y, float _size) { allocate(x, y, _size); }
    private native void allocate(float x, float y, float _size);

    public native @Cast("size_t") long hash();

    /** converts vector of keypoints to vector of points */
    public static native void convert(@StdVector KeyPoint keypoints,
                                    @StdVector Point2f points2f,
                                    @StdVector IntPointer keypointIndexes/*=std::vector<int>()*/);
    public static native void convert(@StdVector KeyPoint keypoints,
                                    @StdVector Point2f points2f);
    public static native void convert(@StdVector KeyPoint keypoints,
                                    @StdVector Point2f points2f,
                                    @StdVector IntBuffer keypointIndexes/*=std::vector<int>()*/);
    public static native void convert(@StdVector KeyPoint keypoints,
                                    @StdVector Point2f points2f,
                                    @StdVector int[] keypointIndexes/*=std::vector<int>()*/);
    /** converts vector of points to the vector of keypoints, where each keypoint is assigned the same size and the same orientation */
    public static native void convert(@StdVector Point2f points2f,
                                    @StdVector KeyPoint keypoints,
                                    float size/*=1*/, float response/*=1*/, int octave/*=0*/, int class_id/*=-1*/);
    public static native void convert(@StdVector Point2f points2f,
                                    @StdVector KeyPoint keypoints);

    /** computes overlap for pair of keypoints;
     *  overlap is a ratio between area of keypoint regions intersection and
     *  area of keypoint regions union (now keypoint region is circle) */
    public static native float overlap(@Const @ByRef KeyPoint kp1, @Const @ByRef KeyPoint kp2);

    /** coordinates of the keypoints */
    public native @ByRef Point2f pt(); public native KeyPoint pt(Point2f pt);
    /** diameter of the meaningful keypoint neighborhood */
    public native float size(); public native KeyPoint size(float size);
    /** computed orientation of the keypoint (-1 if not applicable);
     *  it's in [0,360) degrees and measured relative to
     *  image coordinate system, ie in clockwise. */
    public native float angle(); public native KeyPoint angle(float angle);
    /** the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling */
    public native float response(); public native KeyPoint response(float response);
    /** octave (pyramid layer) from which the keypoint has been extracted */
    public native int octave(); public native KeyPoint octave(int octave);
    /** object class (if the keypoints need to be clustered by an object they belong to) */
    public native int class_id(); public native KeyPoint class_id(int class_id);
}



//////////////////////////////// DMatch /////////////////////////////////

/*
 * Struct for matching: query descriptor index, train descriptor index, train image index and distance between descriptors.
 */
@Namespace("cv") @NoOffset public static class DMatch extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public DMatch(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public DMatch(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public DMatch position(int position) {
        return (DMatch)super.position(position);
    }

    public DMatch() { allocate(); }
    private native void allocate();
    public DMatch(int _queryIdx, int _trainIdx, float _distance) { allocate(_queryIdx, _trainIdx, _distance); }
    private native void allocate(int _queryIdx, int _trainIdx, float _distance);
    public DMatch(int _queryIdx, int _trainIdx, int _imgIdx, float _distance) { allocate(_queryIdx, _trainIdx, _imgIdx, _distance); }
    private native void allocate(int _queryIdx, int _trainIdx, int _imgIdx, float _distance);

    public native int queryIdx(); public native DMatch queryIdx(int queryIdx); // query descriptor index
    public native int trainIdx(); public native DMatch trainIdx(int trainIdx); // train descriptor index
    public native int imgIdx(); public native DMatch imgIdx(int imgIdx);   // train image index

    public native float distance(); public native DMatch distance(float distance);

    // less is better
    public native @Cast("bool") @Name("operator<") boolean lessThan(@Const @ByRef DMatch m);
}



///////////////////////////// TermCriteria //////////////////////////////

/**
 Termination criteria in iterative algorithms
 */
@Namespace("cv") @NoOffset public static class TermCriteria extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public TermCriteria(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public TermCriteria(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public TermCriteria position(int position) {
        return (TermCriteria)super.position(position);
    }

    /** enum cv::TermCriteria:: */
    public static final int
        /** the maximum number of iterations or elements to compute */
        COUNT= 1,
        /** ditto */
        MAX_ITER= COUNT,
        /** the desired accuracy or change in parameters at which the iterative algorithm stops */
        EPS= 2;

    /** default constructor */
    public TermCriteria() { allocate(); }
    private native void allocate();
    /** full constructor */
    public TermCriteria(int type, int maxCount, double epsilon) { allocate(type, maxCount, epsilon); }
    private native void allocate(int type, int maxCount, double epsilon);

    /** the type of termination criteria: COUNT, EPS or COUNT + EPS */
    public native int type(); public native TermCriteria type(int type);
    public native int maxCount(); public native TermCriteria maxCount(int maxCount); // the maximum number of iterations/elements
    public native double epsilon(); public native TermCriteria epsilon(double epsilon); // the desired accuracy
}



///////////////////////// raster image moments //////////////////////////

@Namespace("cv") @NoOffset public static class Moments extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Moments(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Moments(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Moments position(int position) {
        return (Moments)super.position(position);
    }

    /** the default constructor */
    public Moments() { allocate(); }
    private native void allocate();
    /** the full constructor */
    public Moments(double m00, double m10, double m01, double m20, double m11,
                double m02, double m30, double m21, double m12, double m03 ) { allocate(m00, m10, m01, m20, m11, m02, m30, m21, m12, m03); }
    private native void allocate(double m00, double m10, double m01, double m20, double m11,
                double m02, double m30, double m21, double m12, double m03 );
    ////! the conversion from CvMoments
    //Moments( const CvMoments& moments );
    ////! the conversion to CvMoments
    //operator CvMoments() const;

    /** spatial moments */
    public native double m00(); public native Moments m00(double m00);
    public native double m10(); public native Moments m10(double m10);
    public native double m01(); public native Moments m01(double m01);
    public native double m20(); public native Moments m20(double m20);
    public native double m11(); public native Moments m11(double m11);
    public native double m02(); public native Moments m02(double m02);
    public native double m30(); public native Moments m30(double m30);
    public native double m21(); public native Moments m21(double m21);
    public native double m12(); public native Moments m12(double m12);
    public native double m03(); public native Moments m03(double m03);
    /** central moments */
    public native double mu20(); public native Moments mu20(double mu20);
    public native double mu11(); public native Moments mu11(double mu11);
    public native double mu02(); public native Moments mu02(double mu02);
    public native double mu30(); public native Moments mu30(double mu30);
    public native double mu21(); public native Moments mu21(double mu21);
    public native double mu12(); public native Moments mu12(double mu12);
    public native double mu03(); public native Moments mu03(double mu03);
    /** central normalized moments */
    public native double nu20(); public native Moments nu20(double nu20);
    public native double nu11(); public native Moments nu11(double nu11);
    public native double nu02(); public native Moments nu02(double nu02);
    public native double nu30(); public native Moments nu30(double nu30);
    public native double nu21(); public native Moments nu21(double nu21);
    public native double nu12(); public native Moments nu12(double nu12);
    public native double nu03(); public native Moments nu03(double nu03);
}



/////////////////////////////////////////////////////////////////////////
///////////////////////////// Implementation ////////////////////////////
/////////////////////////////////////////////////////////////////////////

//////////////////////////////// Complex ////////////////////////////////











//////////////////////////////// 2D Point ///////////////////////////////



























//////////////////////////////// 3D Point ///////////////////////////////

























////////////////////////////////// Size /////////////////////////////////

















////////////////////////////////// Rect /////////////////////////////////



























////////////////////////////// RotatedRect //////////////////////////////







///////////////////////////////// Range /////////////////////////////////












@Namespace("cv") public static native @Cast("bool") @Name("operator==") boolean equals(@Const @ByRef Range r1, @Const @ByRef Range r2);

@Namespace("cv") public static native @Cast("bool") @Name("operator!=") boolean notEquals(@Const @ByRef Range r1, @Const @ByRef Range r2);

@Namespace("cv") public static native @Cast("bool") @Name("operator!") boolean not(@Const @ByRef Range r);

@Namespace("cv") public static native @ByVal @Name("operator&") Range and(@Const @ByRef Range r1, @Const @ByRef Range r2);

@Namespace("cv") public static native @ByRef @Name("operator&=") Range andPut(@ByRef Range r1, @Const @ByRef Range r2);

@Namespace("cv") public static native @ByVal @Name("operator+") Range add(@Const @ByRef Range r1, int delta);

@Namespace("cv") public static native @ByVal @Name("operator+") Range add(int delta, @Const @ByRef Range r1);

@Namespace("cv") public static native @ByVal @Name("operator-") Range subtract(@Const @ByRef Range r1, int delta);



///////////////////////////////// Scalar ////////////////////////////////























//////////////////////////////// KeyPoint ///////////////////////////////









///////////////////////////////// DMatch ////////////////////////////////











////////////////////////////// TermCriteria /////////////////////////////





 // cv

// #endif //__OPENCV_CORE_TYPES_HPP__


// Parsed from <opencv2/core.hpp>

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
// Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
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

// #ifndef __OPENCV_CORE_HPP__
// #define __OPENCV_CORE_HPP__

// #ifndef __cplusplus
// #endif

// #include "opencv2/core/cvdef.h"
// #include "opencv2/core/version.hpp"
// #include "opencv2/core/base.hpp"
// #include "opencv2/core/cvstd.hpp"
// #include "opencv2/core/traits.hpp"
// #include "opencv2/core/matx.hpp"
// #include "opencv2/core/types.hpp"
// #include "opencv2/core/mat.hpp"
// #include "opencv2/core/persistence.hpp"

/** \namespace cv
    Namespace where all the C++ OpenCV functionality resides
*/

/**
 The standard OpenCV exception class.
 Instances of the class are thrown by various functions and methods in the case of critical errors.
 */


/** Signals an error and raises the exception.

/**
  By default the function prints information about the error to stderr,
  then it either stops if setBreakOnError() had been called before or raises the exception.
  It is possible to alternate error processing by using redirectError().
  \param exc the exception raisen.
  \todo drop this version
 */


/** enum cv:: */
public static final int SORT_EVERY_ROW    = 0,
       SORT_EVERY_COLUMN = 1,
       SORT_ASCENDING    = 0,
       SORT_DESCENDING   = 16;

/** enum cv:: */
public static final int COVAR_SCRAMBLED = 0,
       COVAR_NORMAL    = 1,
       COVAR_USE_AVG   = 2,
       COVAR_SCALE     = 4,
       COVAR_ROWS      = 8,
       COVAR_COLS      = 16;

/**
 k-Means flags
*/
/** enum cv:: */
public static final int KMEANS_RANDOM_CENTERS     = 0, // Chooses random centers for k-Means initialization
       KMEANS_PP_CENTERS         = 2, // Uses k-Means++ algorithm for initialization
       KMEANS_USE_INITIAL_LABELS = 1;  // Uses the user-provided labels for K-Means initialization

/** enum cv:: */
public static final int FILLED  = -1,
       LINE_4  = 4,
       LINE_8  = 8,
       LINE_AA = 16;

/** enum cv:: */
public static final int FONT_HERSHEY_SIMPLEX        = 0,
       FONT_HERSHEY_PLAIN          = 1,
       FONT_HERSHEY_DUPLEX         = 2,
       FONT_HERSHEY_COMPLEX        = 3,
       FONT_HERSHEY_TRIPLEX        = 4,
       FONT_HERSHEY_COMPLEX_SMALL  = 5,
       FONT_HERSHEY_SCRIPT_SIMPLEX = 6,
       FONT_HERSHEY_SCRIPT_COMPLEX = 7,
       FONT_ITALIC                 = 16;

/** enum cv:: */
public static final int REDUCE_SUM = 0,
       REDUCE_AVG = 1,
       REDUCE_MAX = 2,
       REDUCE_MIN = 3;


/** swaps two matrices */
@Namespace("cv") public static native void swap(@ByRef Mat a, @ByRef Mat b);

/** swaps two umatrices */
@Namespace("cv") public static native void swap( @ByRef UMat a, @ByRef UMat b );

/** 1D interpolation function: returns coordinate of the "donor" pixel for the specified location p. */
@Namespace("cv") public static native int borderInterpolate(int p, int len, int borderType);

/** copies 2D array to a larger destination array with extrapolation of the outer part of src using the specified border mode */
@Namespace("cv") public static native void copyMakeBorder(@ByVal Mat src, @ByVal Mat dst,
                                 int top, int bottom, int left, int right,
                                 int borderType, @Const @ByRef Scalar value/*=Scalar()*/ );
@Namespace("cv") public static native void copyMakeBorder(@ByVal Mat src, @ByVal Mat dst,
                                 int top, int bottom, int left, int right,
                                 int borderType );

/** adds one matrix to another (dst = src1 + src2) */
@Namespace("cv") public static native void add(@ByVal Mat src1, @ByVal Mat src2, @ByVal Mat dst,
                      @ByVal Mat mask/*=noArray()*/, int dtype/*=-1*/);
@Namespace("cv") public static native void add(@ByVal Mat src1, @ByVal Mat src2, @ByVal Mat dst);

/** subtracts one matrix from another (dst = src1 - src2) */
@Namespace("cv") public static native void subtract(@ByVal Mat src1, @ByVal Mat src2, @ByVal Mat dst,
                           @ByVal Mat mask/*=noArray()*/, int dtype/*=-1*/);
@Namespace("cv") public static native void subtract(@ByVal Mat src1, @ByVal Mat src2, @ByVal Mat dst);

/** computes element-wise weighted product of the two arrays (dst = scale*src1*src2) */
@Namespace("cv") public static native void multiply(@ByVal Mat src1, @ByVal Mat src2,
                           @ByVal Mat dst, double scale/*=1*/, int dtype/*=-1*/);
@Namespace("cv") public static native void multiply(@ByVal Mat src1, @ByVal Mat src2,
                           @ByVal Mat dst);

/** computes element-wise weighted quotient of the two arrays (dst = scale * src1 / src2) */
@Namespace("cv") public static native void divide(@ByVal Mat src1, @ByVal Mat src2, @ByVal Mat dst,
                         double scale/*=1*/, int dtype/*=-1*/);
@Namespace("cv") public static native void divide(@ByVal Mat src1, @ByVal Mat src2, @ByVal Mat dst);

/** computes element-wise weighted reciprocal of an array (dst = scale/src2) */
@Namespace("cv") public static native void divide(double scale, @ByVal Mat src2,
                         @ByVal Mat dst, int dtype/*=-1*/);
@Namespace("cv") public static native void divide(double scale, @ByVal Mat src2,
                         @ByVal Mat dst);

/** adds scaled array to another one (dst = alpha*src1 + src2) */
@Namespace("cv") public static native void scaleAdd(@ByVal Mat src1, double alpha, @ByVal Mat src2, @ByVal Mat dst);

/** computes weighted sum of two arrays (dst = alpha*src1 + beta*src2 + gamma) */
@Namespace("cv") public static native void addWeighted(@ByVal Mat src1, double alpha, @ByVal Mat src2,
                              double beta, double gamma, @ByVal Mat dst, int dtype/*=-1*/);
@Namespace("cv") public static native void addWeighted(@ByVal Mat src1, double alpha, @ByVal Mat src2,
                              double beta, double gamma, @ByVal Mat dst);

/** scales array elements, computes absolute values and converts the results to 8-bit unsigned integers: dst(i)=saturate_cast<uchar>abs(src(i)*alpha+beta) */
@Namespace("cv") public static native void convertScaleAbs(@ByVal Mat src, @ByVal Mat dst,
                                  double alpha/*=1*/, double beta/*=0*/);
@Namespace("cv") public static native void convertScaleAbs(@ByVal Mat src, @ByVal Mat dst);

/** transforms array of numbers using a lookup table: dst(i)=lut(src(i)) */
@Namespace("cv") public static native void LUT(@ByVal Mat src, @ByVal Mat lut, @ByVal Mat dst);

/** computes sum of array elements */
@Namespace("cv") public static native @ByVal @Name("sum") Scalar sumElems(@ByVal Mat src);

/** computes the number of nonzero array elements */
@Namespace("cv") public static native int countNonZero( @ByVal Mat src );

/** returns the list of locations of non-zero pixels */
@Namespace("cv") public static native void findNonZero( @ByVal Mat src, @ByVal Mat idx );

/** computes mean value of selected array elements */
@Namespace("cv") public static native @ByVal Scalar mean(@ByVal Mat src, @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native @ByVal Scalar mean(@ByVal Mat src);

/** computes mean value and standard deviation of all or selected array elements */
@Namespace("cv") public static native void meanStdDev(@ByVal Mat src, @ByVal Mat mean, @ByVal Mat stddev,
                             @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native void meanStdDev(@ByVal Mat src, @ByVal Mat mean, @ByVal Mat stddev);

/** computes norm of the selected array part */
@Namespace("cv") public static native double norm(@ByVal Mat src1, int normType/*=NORM_L2*/, @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native double norm(@ByVal Mat src1);

/** computes norm of selected part of the difference between two arrays */
@Namespace("cv") public static native double norm(@ByVal Mat src1, @ByVal Mat src2,
                         int normType/*=NORM_L2*/, @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native double norm(@ByVal Mat src1, @ByVal Mat src2);

/** computes PSNR image/video quality metric */
@Namespace("cv") public static native double PSNR(@ByVal Mat src1, @ByVal Mat src2);

/** computes norm of a sparse matrix */
@Namespace("cv") public static native double norm( @Const @ByRef SparseMat src, int normType );

/** naive nearest neighbor finder */
@Namespace("cv") public static native void batchDistance(@ByVal Mat src1, @ByVal Mat src2,
                                @ByVal Mat dist, int dtype, @ByVal Mat nidx,
                                int normType/*=NORM_L2*/, int K/*=0*/,
                                @ByVal Mat mask/*=noArray()*/, int update/*=0*/,
                                @Cast("bool") boolean crosscheck/*=false*/);
@Namespace("cv") public static native void batchDistance(@ByVal Mat src1, @ByVal Mat src2,
                                @ByVal Mat dist, int dtype, @ByVal Mat nidx);

/** scales and shifts array elements so that either the specified norm (alpha) or the minimum (alpha) and maximum (beta) array values get the specified values */
@Namespace("cv") public static native void normalize( @ByVal Mat src, @ByVal Mat dst, double alpha/*=1*/, double beta/*=0*/,
                             int norm_type/*=NORM_L2*/, int dtype/*=-1*/, @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native void normalize( @ByVal Mat src, @ByVal Mat dst);

/** scales and shifts array elements so that either the specified norm (alpha) or the minimum (alpha) and maximum (beta) array values get the specified values */
@Namespace("cv") public static native void normalize( @Const @ByRef SparseMat src, @ByRef SparseMat dst, double alpha, int normType );

/** finds global minimum and maximum array elements and returns their values and their locations */
@Namespace("cv") public static native void minMaxLoc(@ByVal Mat src, DoublePointer minVal,
                            DoublePointer maxVal/*=0*/, Point minLoc/*=0*/,
                            Point maxLoc/*=0*/, @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native void minMaxLoc(@ByVal Mat src, DoublePointer minVal);
@Namespace("cv") public static native void minMaxLoc(@ByVal Mat src, DoubleBuffer minVal,
                            DoubleBuffer maxVal/*=0*/, Point minLoc/*=0*/,
                            Point maxLoc/*=0*/, @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native void minMaxLoc(@ByVal Mat src, DoubleBuffer minVal);
@Namespace("cv") public static native void minMaxLoc(@ByVal Mat src, double[] minVal,
                            double[] maxVal/*=0*/, Point minLoc/*=0*/,
                            Point maxLoc/*=0*/, @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native void minMaxLoc(@ByVal Mat src, double[] minVal);

@Namespace("cv") public static native void minMaxIdx(@ByVal Mat src, DoublePointer minVal, DoublePointer maxVal/*=0*/,
                          IntPointer minIdx/*=0*/, IntPointer maxIdx/*=0*/, @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native void minMaxIdx(@ByVal Mat src, DoublePointer minVal);
@Namespace("cv") public static native void minMaxIdx(@ByVal Mat src, DoubleBuffer minVal, DoubleBuffer maxVal/*=0*/,
                          IntBuffer minIdx/*=0*/, IntBuffer maxIdx/*=0*/, @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native void minMaxIdx(@ByVal Mat src, DoubleBuffer minVal);
@Namespace("cv") public static native void minMaxIdx(@ByVal Mat src, double[] minVal, double[] maxVal/*=0*/,
                          int[] minIdx/*=0*/, int[] maxIdx/*=0*/, @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native void minMaxIdx(@ByVal Mat src, double[] minVal);

/** finds global minimum and maximum sparse array elements and returns their values and their locations */
@Namespace("cv") public static native void minMaxLoc(@Const @ByRef SparseMat a, DoublePointer minVal,
                          DoublePointer maxVal, IntPointer minIdx/*=0*/, IntPointer maxIdx/*=0*/);
@Namespace("cv") public static native void minMaxLoc(@Const @ByRef SparseMat a, DoublePointer minVal,
                          DoublePointer maxVal);
@Namespace("cv") public static native void minMaxLoc(@Const @ByRef SparseMat a, DoubleBuffer minVal,
                          DoubleBuffer maxVal, IntBuffer minIdx/*=0*/, IntBuffer maxIdx/*=0*/);
@Namespace("cv") public static native void minMaxLoc(@Const @ByRef SparseMat a, DoubleBuffer minVal,
                          DoubleBuffer maxVal);
@Namespace("cv") public static native void minMaxLoc(@Const @ByRef SparseMat a, double[] minVal,
                          double[] maxVal, int[] minIdx/*=0*/, int[] maxIdx/*=0*/);
@Namespace("cv") public static native void minMaxLoc(@Const @ByRef SparseMat a, double[] minVal,
                          double[] maxVal);

/** transforms 2D matrix to 1D row or column vector by taking sum, minimum, maximum or mean value over all the rows */
@Namespace("cv") public static native void reduce(@ByVal Mat src, @ByVal Mat dst, int dim, int rtype, int dtype/*=-1*/);
@Namespace("cv") public static native void reduce(@ByVal Mat src, @ByVal Mat dst, int dim, int rtype);

/** makes multi-channel array out of several single-channel arrays */
@Namespace("cv") public static native void merge(@Const Mat mv, @Cast("size_t") long count, @ByVal Mat dst);

/** makes multi-channel array out of several single-channel arrays */
@Namespace("cv") public static native void merge(@ByVal MatVector mv, @ByVal Mat dst);

/** copies each plane of a multi-channel array to a dedicated array */
@Namespace("cv") public static native void split(@Const @ByRef Mat src, Mat mvbegin);

/** copies each plane of a multi-channel array to a dedicated array */
@Namespace("cv") public static native void split(@ByVal Mat m, @ByVal MatVector mv);

/** copies selected channels from the input arrays to the selected channels of the output arrays */
@Namespace("cv") public static native void mixChannels(@Const Mat src, @Cast("size_t") long nsrcs, Mat dst, @Cast("size_t") long ndsts,
                            @Const IntPointer fromTo, @Cast("size_t") long npairs);
@Namespace("cv") public static native void mixChannels(@Const Mat src, @Cast("size_t") long nsrcs, Mat dst, @Cast("size_t") long ndsts,
                            @Const IntBuffer fromTo, @Cast("size_t") long npairs);
@Namespace("cv") public static native void mixChannels(@Const Mat src, @Cast("size_t") long nsrcs, Mat dst, @Cast("size_t") long ndsts,
                            @Const int[] fromTo, @Cast("size_t") long npairs);

@Namespace("cv") public static native void mixChannels(@ByVal MatVector src, @ByVal MatVector dst,
                            @Const IntPointer fromTo, @Cast("size_t") long npairs);
@Namespace("cv") public static native void mixChannels(@ByVal MatVector src, @ByVal MatVector dst,
                            @Const IntBuffer fromTo, @Cast("size_t") long npairs);
@Namespace("cv") public static native void mixChannels(@ByVal MatVector src, @ByVal MatVector dst,
                            @Const int[] fromTo, @Cast("size_t") long npairs);

@Namespace("cv") public static native void mixChannels(@ByVal MatVector src, @ByVal MatVector dst,
                              @StdVector IntPointer fromTo);
@Namespace("cv") public static native void mixChannels(@ByVal MatVector src, @ByVal MatVector dst,
                              @StdVector IntBuffer fromTo);
@Namespace("cv") public static native void mixChannels(@ByVal MatVector src, @ByVal MatVector dst,
                              @StdVector int[] fromTo);

/** extracts a single channel from src (coi is 0-based index) */
@Namespace("cv") public static native void extractChannel(@ByVal Mat src, @ByVal Mat dst, int coi);

/** inserts a single channel to dst (coi is 0-based index) */
@Namespace("cv") public static native void insertChannel(@ByVal Mat src, @ByVal Mat dst, int coi);

/** reverses the order of the rows, columns or both in a matrix */
@Namespace("cv") public static native void flip(@ByVal Mat src, @ByVal Mat dst, int flipCode);

/** replicates the input matrix the specified number of times in the horizontal and/or vertical direction */
@Namespace("cv") public static native void repeat(@ByVal Mat src, int ny, int nx, @ByVal Mat dst);

@Namespace("cv") public static native @ByVal Mat repeat(@Const @ByRef Mat src, int ny, int nx);

@Namespace("cv") public static native void hconcat(@Const Mat src, @Cast("size_t") long nsrc, @ByVal Mat dst);

@Namespace("cv") public static native void hconcat(@ByVal Mat src1, @ByVal Mat src2, @ByVal Mat dst);

@Namespace("cv") public static native void hconcat(@ByVal MatVector src, @ByVal Mat dst);

@Namespace("cv") public static native void vconcat(@Const Mat src, @Cast("size_t") long nsrc, @ByVal Mat dst);

@Namespace("cv") public static native void vconcat(@ByVal Mat src1, @ByVal Mat src2, @ByVal Mat dst);

@Namespace("cv") public static native void vconcat(@ByVal MatVector src, @ByVal Mat dst);

/** computes bitwise conjunction of the two arrays (dst = src1 & src2) */
@Namespace("cv") public static native void bitwise_and(@ByVal Mat src1, @ByVal Mat src2,
                              @ByVal Mat dst, @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native void bitwise_and(@ByVal Mat src1, @ByVal Mat src2,
                              @ByVal Mat dst);

/** computes bitwise disjunction of the two arrays (dst = src1 | src2) */
@Namespace("cv") public static native void bitwise_or(@ByVal Mat src1, @ByVal Mat src2,
                             @ByVal Mat dst, @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native void bitwise_or(@ByVal Mat src1, @ByVal Mat src2,
                             @ByVal Mat dst);

/** computes bitwise exclusive-or of the two arrays (dst = src1 ^ src2) */
@Namespace("cv") public static native void bitwise_xor(@ByVal Mat src1, @ByVal Mat src2,
                              @ByVal Mat dst, @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native void bitwise_xor(@ByVal Mat src1, @ByVal Mat src2,
                              @ByVal Mat dst);

/** inverts each bit of array (dst = ~src) */
@Namespace("cv") public static native void bitwise_not(@ByVal Mat src, @ByVal Mat dst,
                              @ByVal Mat mask/*=noArray()*/);
@Namespace("cv") public static native void bitwise_not(@ByVal Mat src, @ByVal Mat dst);

/** computes element-wise absolute difference of two arrays (dst = abs(src1 - src2)) */
@Namespace("cv") public static native void absdiff(@ByVal Mat src1, @ByVal Mat src2, @ByVal Mat dst);

/** set mask elements for those array elements which are within the element-specific bounding box (dst = lowerb <= src && src < upperb) */
@Namespace("cv") public static native void inRange(@ByVal Mat src, @ByVal Mat lowerb,
                          @ByVal Mat upperb, @ByVal Mat dst);

/** compares elements of two arrays (dst = src1 \<cmpop\> src2) */
@Namespace("cv") public static native void compare(@ByVal Mat src1, @ByVal Mat src2, @ByVal Mat dst, int cmpop);

/** computes per-element minimum of two arrays (dst = min(src1, src2)) */
@Namespace("cv") public static native void min(@ByVal Mat src1, @ByVal Mat src2, @ByVal Mat dst);

/** computes per-element maximum of two arrays (dst = max(src1, src2)) */
@Namespace("cv") public static native void max(@ByVal Mat src1, @ByVal Mat src2, @ByVal Mat dst);

// the following overloads are needed to avoid conflicts with
//     const _Tp& std::min(const _Tp&, const _Tp&, _Compare)
/** computes per-element minimum of two arrays (dst = min(src1, src2)) */
/** computes per-element maximum of two arrays (dst = max(src1, src2)) */
/** computes per-element minimum of two arrays (dst = min(src1, src2)) */
@Namespace("cv") public static native void min(@Const @ByRef UMat src1, @Const @ByRef UMat src2, @ByRef UMat dst);
/** computes per-element maximum of two arrays (dst = max(src1, src2)) */
@Namespace("cv") public static native void max(@Const @ByRef UMat src1, @Const @ByRef UMat src2, @ByRef UMat dst);

/** computes square root of each matrix element (dst = src**0.5) */
@Namespace("cv") public static native void sqrt(@ByVal Mat src, @ByVal Mat dst);

/** raises the input matrix elements to the specified power (b = a**power) */
@Namespace("cv") public static native void pow(@ByVal Mat src, double power, @ByVal Mat dst);

/** computes exponent of each matrix element (dst = e**src) */
@Namespace("cv") public static native void exp(@ByVal Mat src, @ByVal Mat dst);

/** computes natural logarithm of absolute value of each matrix element: dst = log(abs(src)) */
@Namespace("cv") public static native void log(@ByVal Mat src, @ByVal Mat dst);

/** converts polar coordinates to Cartesian */
@Namespace("cv") public static native void polarToCart(@ByVal Mat magnitude, @ByVal Mat angle,
                              @ByVal Mat x, @ByVal Mat y, @Cast("bool") boolean angleInDegrees/*=false*/);
@Namespace("cv") public static native void polarToCart(@ByVal Mat magnitude, @ByVal Mat angle,
                              @ByVal Mat x, @ByVal Mat y);

/** converts Cartesian coordinates to polar */
@Namespace("cv") public static native void cartToPolar(@ByVal Mat x, @ByVal Mat y,
                              @ByVal Mat magnitude, @ByVal Mat angle,
                              @Cast("bool") boolean angleInDegrees/*=false*/);
@Namespace("cv") public static native void cartToPolar(@ByVal Mat x, @ByVal Mat y,
                              @ByVal Mat magnitude, @ByVal Mat angle);

/** computes angle (angle(i)) of each (x(i), y(i)) vector */
@Namespace("cv") public static native void phase(@ByVal Mat x, @ByVal Mat y, @ByVal Mat angle,
                        @Cast("bool") boolean angleInDegrees/*=false*/);
@Namespace("cv") public static native void phase(@ByVal Mat x, @ByVal Mat y, @ByVal Mat angle);

/** computes magnitude (magnitude(i)) of each (x(i), y(i)) vector */
@Namespace("cv") public static native void magnitude(@ByVal Mat x, @ByVal Mat y, @ByVal Mat magnitude);

/** checks that each matrix element is within the specified range. */
@Namespace("cv") public static native @Cast("bool") boolean checkRange(@ByVal Mat a, @Cast("bool") boolean quiet/*=true*/, Point pos/*=0*/,
                            double minVal/*=-DBL_MAX*/, double maxVal/*=DBL_MAX*/);
@Namespace("cv") public static native @Cast("bool") boolean checkRange(@ByVal Mat a);

/** converts NaN's to the given number */
@Namespace("cv") public static native void patchNaNs(@ByVal Mat a, double val/*=0*/);
@Namespace("cv") public static native void patchNaNs(@ByVal Mat a);

/** implements generalized matrix product algorithm GEMM from BLAS */
@Namespace("cv") public static native void gemm(@ByVal Mat src1, @ByVal Mat src2, double alpha,
                       @ByVal Mat src3, double beta, @ByVal Mat dst, int flags/*=0*/);
@Namespace("cv") public static native void gemm(@ByVal Mat src1, @ByVal Mat src2, double alpha,
                       @ByVal Mat src3, double beta, @ByVal Mat dst);

/** multiplies matrix by its transposition from the left or from the right */
@Namespace("cv") public static native void mulTransposed( @ByVal Mat src, @ByVal Mat dst, @Cast("bool") boolean aTa,
                                 @ByVal Mat delta/*=noArray()*/,
                                 double scale/*=1*/, int dtype/*=-1*/ );
@Namespace("cv") public static native void mulTransposed( @ByVal Mat src, @ByVal Mat dst, @Cast("bool") boolean aTa );

/** transposes the matrix */
@Namespace("cv") public static native void transpose(@ByVal Mat src, @ByVal Mat dst);

/** performs affine transformation of each element of multi-channel input matrix */
@Namespace("cv") public static native void transform(@ByVal Mat src, @ByVal Mat dst, @ByVal Mat m );

/** performs perspective transformation of each element of multi-channel input matrix */
@Namespace("cv") public static native void perspectiveTransform(@ByVal Mat src, @ByVal Mat dst, @ByVal Mat m );

/** extends the symmetrical matrix from the lower half or from the upper half */
@Namespace("cv") public static native void completeSymm(@ByVal Mat mtx, @Cast("bool") boolean lowerToUpper/*=false*/);
@Namespace("cv") public static native void completeSymm(@ByVal Mat mtx);

/** initializes scaled identity matrix */
@Namespace("cv") public static native void setIdentity(@ByVal Mat mtx, @Const @ByRef Scalar s/*=Scalar(1)*/);
@Namespace("cv") public static native void setIdentity(@ByVal Mat mtx);

/** computes determinant of a square matrix */
@Namespace("cv") public static native double determinant(@ByVal Mat mtx);

/** computes trace of a matrix */
@Namespace("cv") public static native @ByVal Scalar trace(@ByVal Mat mtx);

/** computes inverse or pseudo-inverse matrix */
@Namespace("cv") public static native double invert(@ByVal Mat src, @ByVal Mat dst, int flags/*=DECOMP_LU*/);
@Namespace("cv") public static native double invert(@ByVal Mat src, @ByVal Mat dst);

/** solves linear system or a least-square problem */
@Namespace("cv") public static native @Cast("bool") boolean solve(@ByVal Mat src1, @ByVal Mat src2,
                        @ByVal Mat dst, int flags/*=DECOMP_LU*/);
@Namespace("cv") public static native @Cast("bool") boolean solve(@ByVal Mat src1, @ByVal Mat src2,
                        @ByVal Mat dst);

/** sorts independently each matrix row or each matrix column */
@Namespace("cv") public static native void sort(@ByVal Mat src, @ByVal Mat dst, int flags);

/** sorts independently each matrix row or each matrix column */
@Namespace("cv") public static native void sortIdx(@ByVal Mat src, @ByVal Mat dst, int flags);

/** finds real roots of a cubic polynomial */
@Namespace("cv") public static native int solveCubic(@ByVal Mat coeffs, @ByVal Mat roots);

/** finds real and complex roots of a polynomial */
@Namespace("cv") public static native double solvePoly(@ByVal Mat coeffs, @ByVal Mat roots, int maxIters/*=300*/);
@Namespace("cv") public static native double solvePoly(@ByVal Mat coeffs, @ByVal Mat roots);

/** finds eigenvalues and eigenvectors of a symmetric matrix */
@Namespace("cv") public static native @Cast("bool") boolean eigen(@ByVal Mat src, @ByVal Mat eigenvalues,
                        @ByVal Mat eigenvectors/*=noArray()*/);
@Namespace("cv") public static native @Cast("bool") boolean eigen(@ByVal Mat src, @ByVal Mat eigenvalues);

/** computes covariation matrix of a set of samples */
@Namespace("cv") public static native void calcCovarMatrix( @Const Mat samples, int nsamples, @ByRef Mat covar, @ByRef Mat mean,
                                 int flags, int ctype/*=CV_64F*/);
@Namespace("cv") public static native void calcCovarMatrix( @Const Mat samples, int nsamples, @ByRef Mat covar, @ByRef Mat mean,
                                 int flags); //TODO: InputArrayOfArrays

/** computes covariation matrix of a set of samples */
@Namespace("cv") public static native void calcCovarMatrix( @ByVal Mat samples, @ByVal Mat covar,
                                   @ByVal Mat mean, int flags, int ctype/*=CV_64F*/);
@Namespace("cv") public static native void calcCovarMatrix( @ByVal Mat samples, @ByVal Mat covar,
                                   @ByVal Mat mean, int flags);

@Namespace("cv") public static native void PCACompute(@ByVal Mat data, @ByVal Mat mean,
                             @ByVal Mat eigenvectors, int maxComponents/*=0*/);
@Namespace("cv") public static native void PCACompute(@ByVal Mat data, @ByVal Mat mean,
                             @ByVal Mat eigenvectors);

@Namespace("cv") public static native void PCACompute(@ByVal Mat data, @ByVal Mat mean,
                             @ByVal Mat eigenvectors, double retainedVariance);

@Namespace("cv") public static native void PCAProject(@ByVal Mat data, @ByVal Mat mean,
                             @ByVal Mat eigenvectors, @ByVal Mat result);

@Namespace("cv") public static native void PCABackProject(@ByVal Mat data, @ByVal Mat mean,
                                 @ByVal Mat eigenvectors, @ByVal Mat result);

/** computes SVD of src */
@Namespace("cv") public static native void SVDecomp( @ByVal Mat src, @ByVal Mat w, @ByVal Mat u, @ByVal Mat vt, int flags/*=0*/ );
@Namespace("cv") public static native void SVDecomp( @ByVal Mat src, @ByVal Mat w, @ByVal Mat u, @ByVal Mat vt );

/** performs back substitution for the previously computed SVD */
@Namespace("cv") public static native void SVBackSubst( @ByVal Mat w, @ByVal Mat u, @ByVal Mat vt,
                               @ByVal Mat rhs, @ByVal Mat dst );

/** computes Mahalanobis distance between two vectors: sqrt((v1-v2)'*icovar*(v1-v2)), where icovar is the inverse covariation matrix */
@Namespace("cv") public static native double Mahalanobis(@ByVal Mat v1, @ByVal Mat v2, @ByVal Mat icovar);

/** performs forward or inverse 1D or 2D Discrete Fourier Transformation */
@Namespace("cv") public static native void dft(@ByVal Mat src, @ByVal Mat dst, int flags/*=0*/, int nonzeroRows/*=0*/);
@Namespace("cv") public static native void dft(@ByVal Mat src, @ByVal Mat dst);

/** performs inverse 1D or 2D Discrete Fourier Transformation */
@Namespace("cv") public static native void idft(@ByVal Mat src, @ByVal Mat dst, int flags/*=0*/, int nonzeroRows/*=0*/);
@Namespace("cv") public static native void idft(@ByVal Mat src, @ByVal Mat dst);

/** performs forward or inverse 1D or 2D Discrete Cosine Transformation */
@Namespace("cv") public static native void dct(@ByVal Mat src, @ByVal Mat dst, int flags/*=0*/);
@Namespace("cv") public static native void dct(@ByVal Mat src, @ByVal Mat dst);

/** performs inverse 1D or 2D Discrete Cosine Transformation */
@Namespace("cv") public static native void idct(@ByVal Mat src, @ByVal Mat dst, int flags/*=0*/);
@Namespace("cv") public static native void idct(@ByVal Mat src, @ByVal Mat dst);

/** computes element-wise product of the two Fourier spectrums. The second spectrum can optionally be conjugated before the multiplication */
@Namespace("cv") public static native void mulSpectrums(@ByVal Mat a, @ByVal Mat b, @ByVal Mat c,
                               int flags, @Cast("bool") boolean conjB/*=false*/);
@Namespace("cv") public static native void mulSpectrums(@ByVal Mat a, @ByVal Mat b, @ByVal Mat c,
                               int flags);

/** computes the minimal vector size vecsize1 >= vecsize so that the dft() of the vector of length vecsize1 can be computed efficiently */
@Namespace("cv") public static native int getOptimalDFTSize(int vecsize);

/** clusters the input data using k-Means algorithm */
@Namespace("cv") public static native double kmeans( @ByVal Mat data, int K, @ByVal Mat bestLabels,
                            @ByVal TermCriteria criteria, int attempts,
                            int flags, @ByVal Mat centers/*=noArray()*/ );
@Namespace("cv") public static native double kmeans( @ByVal Mat data, int K, @ByVal Mat bestLabels,
                            @ByVal TermCriteria criteria, int attempts,
                            int flags );

/** returns the thread-local Random number generator */
@Namespace("cv") public static native @ByRef RNG theRNG();

/** fills array with uniformly-distributed random numbers from the range [low, high) */
@Namespace("cv") public static native void randu(@ByVal Mat dst, @ByVal Mat low, @ByVal Mat high);

/** fills array with normally-distributed random numbers with the specified mean and the standard deviation */
@Namespace("cv") public static native void randn(@ByVal Mat dst, @ByVal Mat mean, @ByVal Mat stddev);

/** shuffles the input array elements */
@Namespace("cv") public static native void randShuffle(@ByVal Mat dst, double iterFactor/*=1.*/, RNG rng/*=0*/);
@Namespace("cv") public static native void randShuffle(@ByVal Mat dst);

/**
    Principal Component Analysis

    The class PCA is used to compute the special basis for a set of vectors.
    The basis will consist of eigenvectors of the covariance matrix computed
    from the input set of vectors. After PCA is performed, vectors can be transformed from
    the original high-dimensional space to the subspace formed by a few most
    prominent eigenvectors (called the principal components),
    corresponding to the largest eigenvalues of the covariation matrix.
    Thus the dimensionality of the vector and the correlation between the coordinates is reduced.

    The following sample is the function that takes two matrices. The first one stores the set
    of vectors (a row per vector) that is used to compute PCA, the second one stores another
    "test" set of vectors (a row per vector) that are first compressed with PCA,
    then reconstructed back and then the reconstruction error norm is computed and printed for each vector.

    \code
    using namespace cv;

    PCA compressPCA(const Mat& pcaset, int maxComponents,
                    const Mat& testset, Mat& compressed)
    {
        PCA pca(pcaset, // pass the data
                Mat(), // we do not have a pre-computed mean vector,
                       // so let the PCA engine to compute it
                PCA::DATA_AS_ROW, // indicate that the vectors
                                    // are stored as matrix rows
                                    // (use PCA::DATA_AS_COL if the vectors are
                                    // the matrix columns)
                maxComponents // specify, how many principal components to retain
                );
        // if there is no test data, just return the computed basis, ready-to-use
        if( !testset.data )
            return pca;
        CV_Assert( testset.cols == pcaset.cols );

        compressed.create(testset.rows, maxComponents, testset.type());

        Mat reconstructed;
        for( int i = 0; i < testset.rows; i++ )
        {
            Mat vec = testset.row(i), coeffs = compressed.row(i), reconstructed;
            // compress the vector, the result will be stored
            // in the i-th row of the output matrix
            pca.project(vec, coeffs);
            // and then reconstruct it
            pca.backProject(coeffs, reconstructed);
            // and measure the error
            printf("%d. diff = %g\n", i, norm(vec, reconstructed, NORM_L2));
        }
        return pca;
    }
    \endcode
*/
@Namespace("cv") @NoOffset public static class PCA extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public PCA(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public PCA(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public PCA position(int position) {
        return (PCA)super.position(position);
    }

    /** enum cv::PCA:: */
    public static final int DATA_AS_ROW = 0,
           DATA_AS_COL = 1,
           USE_AVG     = 2;

    /** default constructor */
    public PCA() { allocate(); }
    private native void allocate();

    /** the constructor that performs PCA */
    public PCA(@ByVal Mat data, @ByVal Mat mean, int flags, int maxComponents/*=0*/) { allocate(data, mean, flags, maxComponents); }
    private native void allocate(@ByVal Mat data, @ByVal Mat mean, int flags, int maxComponents/*=0*/);
    public PCA(@ByVal Mat data, @ByVal Mat mean, int flags) { allocate(data, mean, flags); }
    private native void allocate(@ByVal Mat data, @ByVal Mat mean, int flags);
    public PCA(@ByVal Mat data, @ByVal Mat mean, int flags, double retainedVariance) { allocate(data, mean, flags, retainedVariance); }
    private native void allocate(@ByVal Mat data, @ByVal Mat mean, int flags, double retainedVariance);

    /** operator that performs PCA. The previously stored data, if any, is released */
    public native @ByRef @Name("operator()") PCA apply(@ByVal Mat data, @ByVal Mat mean, int flags, int maxComponents/*=0*/);
    public native @ByRef @Name("operator()") PCA apply(@ByVal Mat data, @ByVal Mat mean, int flags);
    public native @ByRef @Name("operator()") PCA apply(@ByVal Mat data, @ByVal Mat mean, int flags, double retainedVariance);

    /** projects vector from the original space to the principal components subspace */
    public native @ByVal Mat project(@ByVal Mat vec);

    /** projects vector from the original space to the principal components subspace */
    public native void project(@ByVal Mat vec, @ByVal Mat result);

    /** reconstructs the original vector from the projection */
    public native @ByVal Mat backProject(@ByVal Mat vec);

    /** reconstructs the original vector from the projection */
    public native void backProject(@ByVal Mat vec, @ByVal Mat result);

    /** write and load PCA matrix */
    public native void write(@ByRef FileStorage fs );
    public native void read(@Const @ByRef FileNode fs);

    /** eigenvectors of the covariation matrix */
    public native @ByRef Mat eigenvectors(); public native PCA eigenvectors(Mat eigenvectors);
    /** eigenvalues of the covariation matrix */
    public native @ByRef Mat eigenvalues(); public native PCA eigenvalues(Mat eigenvalues);
    /** mean value subtracted before the projection and added after the back projection */
    public native @ByRef Mat mean(); public native PCA mean(Mat mean);
}

// Linear Discriminant Analysis
@Namespace("cv") @NoOffset public static class LDA extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public LDA(Pointer p) { super(p); }

    // Initializes a LDA with num_components (default 0) and specifies how
    // samples are aligned (default dataAsRow=true).
    public LDA(int num_components/*=0*/) { allocate(num_components); }
    private native void allocate(int num_components/*=0*/);
    public LDA() { allocate(); }
    private native void allocate();

    // Initializes and performs a Discriminant Analysis with Fisher's
    // Optimization Criterion on given data in src and corresponding labels
    // in labels. If 0 (or less) number of components are given, they are
    // automatically determined for given data in computation.
    public LDA(@ByVal MatVector src, @ByVal Mat labels, int num_components/*=0*/) { allocate(src, labels, num_components); }
    private native void allocate(@ByVal MatVector src, @ByVal Mat labels, int num_components/*=0*/);
    public LDA(@ByVal MatVector src, @ByVal Mat labels) { allocate(src, labels); }
    private native void allocate(@ByVal MatVector src, @ByVal Mat labels);

    // Serializes this object to a given filename.
    public native void save(@Str BytePointer filename);
    public native void save(@Str String filename);

    // Deserializes this object from a given filename.
    public native void load(@Str BytePointer filename);
    public native void load(@Str String filename);

    // Serializes this object to a given cv::FileStorage.
    public native void save(@ByRef FileStorage fs);

        // Deserializes this object from a given cv::FileStorage.
    public native void load(@Const @ByRef FileStorage node);

    // Destructor.

    /** Compute the discriminants for data in src and labels. */
    public native void compute(@ByVal MatVector src, @ByVal Mat labels);

    // Projects samples into the LDA subspace.
    public native @ByVal Mat project(@ByVal Mat src);

    // Reconstructs projections from the LDA subspace.
    public native @ByVal Mat reconstruct(@ByVal Mat src);

    // Returns the eigenvectors of this LDA.
    public native @ByVal Mat eigenvectors();

    // Returns the eigenvalues of this LDA.
    public native @ByVal Mat eigenvalues();

    public static native @ByVal Mat subspaceProject(@ByVal Mat W, @ByVal Mat mean, @ByVal Mat src);
    public static native @ByVal Mat subspaceReconstruct(@ByVal Mat W, @ByVal Mat mean, @ByVal Mat src);
}

/**
    Singular Value Decomposition class

    The class is used to compute Singular Value Decomposition of a floating-point matrix and then
    use it to solve least-square problems, under-determined linear systems, invert matrices,
    compute condition numbers etc.

    For a bit faster operation you can pass flags=SVD::MODIFY_A|... to modify the decomposed matrix
    when it is not necessarily to preserve it. If you want to compute condition number of a matrix
    or absolute value of its determinant - you do not need SVD::u or SVD::vt,
    so you can pass flags=SVD::NO_UV|... . Another flag SVD::FULL_UV indicates that the full-size SVD::u and SVD::vt
    must be computed, which is not necessary most of the time.
*/
@Namespace("cv") @NoOffset public static class SVD extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public SVD(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public SVD(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public SVD position(int position) {
        return (SVD)super.position(position);
    }

    /** enum cv::SVD:: */
    public static final int MODIFY_A = 1,
           NO_UV    = 2,
           FULL_UV  = 4;

    /** the default constructor */
    public SVD() { allocate(); }
    private native void allocate();

    /** the constructor that performs SVD */
    public SVD( @ByVal Mat src, int flags/*=0*/ ) { allocate(src, flags); }
    private native void allocate( @ByVal Mat src, int flags/*=0*/ );
    public SVD( @ByVal Mat src ) { allocate(src); }
    private native void allocate( @ByVal Mat src );

    /** the operator that performs SVD. The previously allocated SVD::u, SVD::w are SVD::vt are released. */
    public native @ByRef @Name("operator()") SVD apply( @ByVal Mat src, int flags/*=0*/ );
    public native @ByRef @Name("operator()") SVD apply( @ByVal Mat src );

    /** decomposes matrix and stores the results to user-provided matrices */
    public static native void compute( @ByVal Mat src, @ByVal Mat w,
                             @ByVal Mat u, @ByVal Mat vt, int flags/*=0*/ );
    public static native void compute( @ByVal Mat src, @ByVal Mat w,
                             @ByVal Mat u, @ByVal Mat vt );

    /** computes singular values of a matrix */
    public static native void compute( @ByVal Mat src, @ByVal Mat w, int flags/*=0*/ );
    public static native void compute( @ByVal Mat src, @ByVal Mat w );

    /** performs back substitution */
    public static native void backSubst( @ByVal Mat w, @ByVal Mat u,
                               @ByVal Mat vt, @ByVal Mat rhs,
                               @ByVal Mat dst );

    /** finds dst = arg min_{|dst|=1} |m*dst| */
    public static native void solveZ( @ByVal Mat src, @ByVal Mat dst );

    /** performs back substitution, so that dst is the solution or pseudo-solution of m*dst = rhs, where m is the decomposed matrix */
    public native void backSubst( @ByVal Mat rhs, @ByVal Mat dst );

    public native @ByRef Mat u(); public native SVD u(Mat u);
    public native @ByRef Mat w(); public native SVD w(Mat w);
    public native @ByRef Mat vt(); public native SVD vt(Mat vt);
}



/**
   Line iterator class

   The class is used to iterate over all the pixels on the raster line
   segment connecting two specified points.
*/
@Namespace("cv") @NoOffset public static class LineIterator extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public LineIterator() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public LineIterator(Pointer p) { super(p); }

    /** intializes the iterator */
    public LineIterator( @Const @ByRef Mat img, @ByVal Point pt1, @ByVal Point pt2,
                      int connectivity/*=8*/, @Cast("bool") boolean leftToRight/*=false*/ ) { allocate(img, pt1, pt2, connectivity, leftToRight); }
    private native void allocate( @Const @ByRef Mat img, @ByVal Point pt1, @ByVal Point pt2,
                      int connectivity/*=8*/, @Cast("bool") boolean leftToRight/*=false*/ );
    public LineIterator( @Const @ByRef Mat img, @ByVal Point pt1, @ByVal Point pt2 ) { allocate(img, pt1, pt2); }
    private native void allocate( @Const @ByRef Mat img, @ByVal Point pt1, @ByVal Point pt2 );
    /** returns pointer to the current pixel */
    public native @Cast("uchar*") @Name("operator*") BytePointer multiply();
    /** prefix increment operator (++it). shifts iterator to the next pixel */
    public native @ByRef @Name("operator++") LineIterator increment();
    /** postfix increment operator (it++). shifts iterator to the next pixel */
    public native @ByVal @Name("operator++") LineIterator increment(int arg0);
    /** returns coordinates of the current pixel */
    public native @ByVal Point pos();

    public native @Cast("uchar*") BytePointer ptr(); public native LineIterator ptr(BytePointer ptr);
    @MemberGetter public native @Cast("const uchar*") BytePointer ptr0();
    public native int step(); public native LineIterator step(int step);
    public native int elemSize(); public native LineIterator elemSize(int elemSize);
    public native int err(); public native LineIterator err(int err);
    public native int count(); public native LineIterator count(int count);
    public native int minusDelta(); public native LineIterator minusDelta(int minusDelta);
    public native int plusDelta(); public native LineIterator plusDelta(int plusDelta);
    public native int minusStep(); public native LineIterator minusStep(int minusStep);
    public native int plusStep(); public native LineIterator plusStep(int plusStep);
}

/**
   Random Number Generator

   The class implements RNG using Multiply-with-Carry algorithm
*/
@Namespace("cv") @NoOffset public static class RNG extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public RNG(Pointer p) { super(p); }

    /** enum cv::RNG:: */
    public static final int UNIFORM = 0,
           NORMAL  = 1;

    public RNG() { allocate(); }
    private native void allocate();
    public RNG(@Cast("uint64") int state) { allocate(state); }
    private native void allocate(@Cast("uint64") int state);
    /** updates the state and returns the next 32-bit unsigned integer random number */
    public native @Cast("unsigned") int next();

    public native @Name("operator uchar") byte asByte();
    public native @Name("operator ushort") short asShort();
    public native @Name("operator unsigned") int asInt();
    /** returns a random integer sampled uniformly from [0, N). */
    public native @Cast("unsigned") @Name("operator()") int apply(@Cast("unsigned") int N);
    public native @Cast("unsigned") @Name("operator()") int apply();
    public native @Name("operator float") float asFloat();
    public native @Name("operator double") double asDouble();
    /** returns uniformly distributed integer random number from [a,b) range */
    public native int uniform(int a, int b);
    /** returns uniformly distributed floating-point random number from [a,b) range */
    public native float uniform(float a, float b);
    /** returns uniformly distributed double-precision floating-point random number from [a,b) range */
    public native double uniform(double a, double b);
    public native void fill( @ByVal Mat mat, int distType, @ByVal Mat a, @ByVal Mat b, @Cast("bool") boolean saturateRange/*=false*/ );
    public native void fill( @ByVal Mat mat, int distType, @ByVal Mat a, @ByVal Mat b );
    /** returns Gaussian random variate with mean zero. */
    public native double gaussian(double sigma);

    public native @Cast("uint64") int state(); public native RNG state(int state);
}

@Namespace("cv") @NoOffset public static class RNG_MT19937 extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public RNG_MT19937(Pointer p) { super(p); }

    public RNG_MT19937() { allocate(); }
    private native void allocate();
    public RNG_MT19937(@Cast("unsigned") int s) { allocate(s); }
    private native void allocate(@Cast("unsigned") int s);
    public native void seed(@Cast("unsigned") int s);

    public native @Cast("unsigned") int next();

    public native @Name("operator int") int asInt();
    public native @Name("operator float") float asFloat();
    public native @Name("operator double") double asDouble();

    public native @Cast("unsigned") @Name("operator()") int apply(@Cast("unsigned") int N);
    public native @Cast("unsigned") @Name("operator()") int apply();

    // returns uniformly distributed integer random number from [a,b) range
    public native int uniform(int a, int b);
    // returns uniformly distributed floating-point random number from [a,b) range
    public native float uniform(float a, float b);
    // returns uniformly distributed double-precision floating-point random number from [a,b) range
    public native double uniform(double a, double b);
}



/////////////////////////////// Formatted output of cv::Mat ///////////////////////////

@Namespace("cv") public static class Formatted extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public Formatted() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Formatted(Pointer p) { super(p); }

    public native @Cast("const char*") BytePointer next();
    public native void reset();
}


@Namespace("cv") public static class Formatter extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public Formatter() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Formatter(Pointer p) { super(p); }

    /** enum cv::Formatter:: */
    public static final int FMT_DEFAULT = 0,
           FMT_MATLAB  = 1,
           FMT_CSV     = 2,
           FMT_PYTHON  = 3,
           FMT_NUMPY   = 4,
           FMT_C       = 5;

    public native @Ptr Formatted format(@Const @ByRef Mat mtx);

    public native void set32fPrecision(int p/*=8*/);
    public native void set32fPrecision();
    public native void set64fPrecision(int p/*=16*/);
    public native void set64fPrecision();
    public native void setMultiline(@Cast("bool") boolean ml/*=true*/);
    public native void setMultiline();

    public static native @Ptr Formatter get(int fmt/*=FMT_DEFAULT*/);
    public static native @Ptr Formatter get();

}



//////////////////////////////////////// Algorithm ////////////////////////////////////

/**
  Base class for high-level OpenCV algorithms
*/
@Namespace("cv") public static class Algorithm extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Algorithm(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Algorithm(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Algorithm position(int position) {
        return (Algorithm)super.position(position);
    }

    public Algorithm() { allocate(); }
    private native void allocate();
    public native @Str BytePointer name();

    public native int getInt(@Str BytePointer name);
    public native int getInt(@Str String name);
    public native double getDouble(@Str BytePointer name);
    public native double getDouble(@Str String name);
    public native @Cast("bool") boolean getBool(@Str BytePointer name);
    public native @Cast("bool") boolean getBool(@Str String name);
    public native @Str BytePointer getString(@Str BytePointer name);
    public native @Str String getString(@Str String name);
    public native @ByVal Mat getMat(@Str BytePointer name);
    public native @ByVal Mat getMat(@Str String name);
    public native @ByVal MatVector getMatVector(@Str BytePointer name);
    public native @ByVal MatVector getMatVector(@Str String name);
    public native @Ptr Algorithm getAlgorithm(@Str BytePointer name);
    public native @Ptr Algorithm getAlgorithm(@Str String name);

    public native void set(@Str BytePointer name, int value);
    public native void set(@Str String name, int value);
    public native void set(@Str BytePointer name, double value);
    public native void set(@Str String name, double value);
    public native void set(@Str BytePointer name, @Cast("bool") boolean value);
    public native void set(@Str String name, @Cast("bool") boolean value);
    public native void set(@Str BytePointer name, @Str BytePointer value);
    public native void set(@Str String name, @Str String value);
    public native void set(@Str BytePointer name, @Const @ByRef Mat value);
    public native void set(@Str String name, @Const @ByRef Mat value);
    public native void set(@Str BytePointer name, @Const @ByRef MatVector value);
    public native void set(@Str String name, @Const @ByRef MatVector value);
    public native void set(@Str BytePointer name, @Ptr Algorithm value);
    public native void set(@Str String name, @Ptr Algorithm value);

    public native void setInt(@Str BytePointer name, int value);
    public native void setInt(@Str String name, int value);
    public native void setDouble(@Str BytePointer name, double value);
    public native void setDouble(@Str String name, double value);
    public native void setBool(@Str BytePointer name, @Cast("bool") boolean value);
    public native void setBool(@Str String name, @Cast("bool") boolean value);
    public native void setString(@Str BytePointer name, @Str BytePointer value);
    public native void setString(@Str String name, @Str String value);
    public native void setMat(@Str BytePointer name, @Const @ByRef Mat value);
    public native void setMat(@Str String name, @Const @ByRef Mat value);
    public native void setMatVector(@Str BytePointer name, @Const @ByRef MatVector value);
    public native void setMatVector(@Str String name, @Const @ByRef MatVector value);
    public native void setAlgorithm(@Str BytePointer name, @Ptr Algorithm value);
    public native void setAlgorithm(@Str String name, @Ptr Algorithm value);

    public native @Str BytePointer paramHelp(@Str BytePointer name);
    public native @Str String paramHelp(@Str String name);
    public native int paramType(@Cast("const char*") BytePointer name);
    public native int paramType(String name);
    public native void getParams(@ByRef StringVector names);


    public native void write(@ByRef FileStorage fs);
    public native void read(@Const @ByRef FileNode fn);

    public static class Constructor extends FunctionPointer {
        static { Loader.load(); }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public    Constructor(Pointer p) { super(p); }
        protected Constructor() { allocate(); }
        private native void allocate();
        public native Algorithm call();
    }
    @Namespace("cv::Algorithm") @Const public static class Getter extends FunctionPointer {
        static { Loader.load(); }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public    Getter(Pointer p) { super(p); }
        public native int call(Algorithm o);
    }
    @Namespace("cv::Algorithm") public static class Setter extends FunctionPointer {
        static { Loader.load(); }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public    Setter(Pointer p) { super(p); }
        public native void call(Algorithm o, int arg0);
    }

    public static native void getList(@ByRef StringVector algorithms);
    public static native @Ptr Algorithm _create(@Str BytePointer name);
    public static native @Ptr Algorithm _create(@Str String name);

    public native AlgorithmInfo info();
}


@Namespace("cv") public static class AlgorithmInfo extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public AlgorithmInfo() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public AlgorithmInfo(Pointer p) { super(p); }

    public AlgorithmInfo(@Str BytePointer name, Algorithm.Constructor create) { allocate(name, create); }
    private native void allocate(@Str BytePointer name, Algorithm.Constructor create);
    public AlgorithmInfo(@Str String name, Algorithm.Constructor create) { allocate(name, create); }
    private native void allocate(@Str String name, Algorithm.Constructor create);
    public native void get(@Const Algorithm algo, @Cast("const char*") BytePointer name, int argType, Pointer value);
    public native void get(@Const Algorithm algo, String name, int argType, Pointer value);
    public native void addParam_(@ByRef Algorithm algo, @Cast("const char*") BytePointer name, int argType,
                       Pointer value, @Cast("bool") boolean readOnly,
                       Algorithm.Getter getter, Algorithm.Setter setter,
                       @Str BytePointer help/*=String()*/);
    public native void addParam_(@ByRef Algorithm algo, @Cast("const char*") BytePointer name, int argType,
                       Pointer value, @Cast("bool") boolean readOnly,
                       Algorithm.Getter getter, Algorithm.Setter setter);
    public native void addParam_(@ByRef Algorithm algo, String name, int argType,
                       Pointer value, @Cast("bool") boolean readOnly,
                       Algorithm.Getter getter, Algorithm.Setter setter,
                       @Str String help/*=String()*/);
    public native void addParam_(@ByRef Algorithm algo, String name, int argType,
                       Pointer value, @Cast("bool") boolean readOnly,
                       Algorithm.Getter getter, Algorithm.Setter setter);
    public native @Str BytePointer paramHelp(@Cast("const char*") BytePointer name);
    public native @Str String paramHelp(String name);
    public native int paramType(@Cast("const char*") BytePointer name);
    public native int paramType(String name);
    public native void getParams(@ByRef StringVector names);

    public native void write(@Const Algorithm algo, @ByRef FileStorage fs);
    public native void read(Algorithm algo, @Const @ByRef FileNode fn);
    public native @Str BytePointer name();

    
    
    
    
    
    
    
    
    
    
    
    
    
}


@Namespace("cv") @NoOffset public static class Param extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Param(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Param(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Param position(int position) {
        return (Param)super.position(position);
    }

    /** enum cv::Param:: */
    public static final int INT= 0, BOOLEAN= 1, REAL= 2, STRING= 3, MAT= 4, MAT_VECTOR= 5, ALGORITHM= 6, FLOAT= 7, UNSIGNED_INT= 8, UINT64= 9, UCHAR= 11;

    public Param() { allocate(); }
    private native void allocate();
    public Param(int _type, @Cast("bool") boolean _readonly, int _offset,
              Algorithm.Getter _getter/*=0*/,
              Algorithm.Setter _setter/*=0*/,
              @Str BytePointer _help/*=String()*/) { allocate(_type, _readonly, _offset, _getter, _setter, _help); }
    private native void allocate(int _type, @Cast("bool") boolean _readonly, int _offset,
              Algorithm.Getter _getter/*=0*/,
              Algorithm.Setter _setter/*=0*/,
              @Str BytePointer _help/*=String()*/);
    public Param(int _type, @Cast("bool") boolean _readonly, int _offset) { allocate(_type, _readonly, _offset); }
    private native void allocate(int _type, @Cast("bool") boolean _readonly, int _offset);
    public Param(int _type, @Cast("bool") boolean _readonly, int _offset,
              Algorithm.Getter _getter/*=0*/,
              Algorithm.Setter _setter/*=0*/,
              @Str String _help/*=String()*/) { allocate(_type, _readonly, _offset, _getter, _setter, _help); }
    private native void allocate(int _type, @Cast("bool") boolean _readonly, int _offset,
              Algorithm.Getter _getter/*=0*/,
              Algorithm.Setter _setter/*=0*/,
              @Str String _help/*=String()*/);
    public native int type(); public native Param type(int type);
    public native int offset(); public native Param offset(int offset);
    public native @Cast("bool") boolean readonly(); public native Param readonly(boolean readonly);
    public native Algorithm.Getter getter(); public native Param getter(Algorithm.Getter getter);
    public native Algorithm.Setter setter(); public native Param setter(Algorithm.Setter setter);
    public native @Str BytePointer help(); public native Param help(BytePointer help);
}

 //namespace cv

// #include "opencv2/core/operations.hpp"
// #include "opencv2/core/cvstd.inl.hpp"
// #include "opencv2/core/utility.hpp"
// #include "opencv2/core/optim.hpp"

// #endif /*__OPENCV_CORE_HPP__*/


// Parsed from <opencv2/core/operations.hpp>

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

// #ifndef __OPENCV_CORE_OPERATIONS_HPP__
// #define __OPENCV_CORE_OPERATIONS_HPP__

// #ifndef __cplusplus
// #endif

// #include <cstdio>

////////////////////////////// Matx methods depending on core API /////////////////////////////

 // internal











////////////////////////// Augmenting algebraic & logical operations //////////////////////////

// #define CV_MAT_AUG_OPERATOR1(op, cvop, A, B)
//     static inline A& operator op (A& a, const B& b) { cvop; return a; }

// #define CV_MAT_AUG_OPERATOR(op, cvop, A, B)
//     CV_MAT_AUG_OPERATOR1(op, cvop, A, B)
//     CV_MAT_AUG_OPERATOR1(op, cvop, const A, B)

// #define CV_MAT_AUG_OPERATOR_T(op, cvop, A, B)
//     template<typename _Tp> CV_MAT_AUG_OPERATOR1(op, cvop, A, B)
//     template<typename _Tp> CV_MAT_AUG_OPERATOR1(op, cvop, const A, B)

@Namespace("cv") public static native @ByRef @Name("operator+=") Mat addPut( @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByRef @Name("operator+=") Mat addPut( @ByRef Mat a, @Const @ByRef Scalar b);

@Namespace("cv") public static native @ByRef @Name("operator-=") Mat subtractPut( @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByRef @Name("operator-=") Mat subtractPut( @ByRef Mat a, @Const @ByRef Scalar b);

@Namespace("cv") public static native @ByRef @Name("operator*=") Mat multiplyPut( @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByRef @Name("operator*=") Mat multiplyPut( @ByRef Mat a, double b);

@Namespace("cv") public static native @ByRef @Name("operator/=") Mat dividePut( @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByRef @Name("operator/=") Mat dividePut( @ByRef Mat a, double b);

@Namespace("cv") public static native @ByRef @Name("operator&=") Mat andPut( @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByRef @Name("operator&=") Mat andPut( @ByRef Mat a, @Const @ByRef Scalar b);

@Namespace("cv") public static native @ByRef @Name("operator|=") Mat orPut( @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByRef @Name("operator|=") Mat orPut( @ByRef Mat a, @Const @ByRef Scalar b);

@Namespace("cv") public static native @ByRef @Name("operator^=") Mat xorPut( @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByRef @Name("operator^=") Mat xorPut( @ByRef Mat a, @Const @ByRef Scalar b);

// #undef CV_MAT_AUG_OPERATOR_T
// #undef CV_MAT_AUG_OPERATOR
// #undef CV_MAT_AUG_OPERATOR1



///////////////////////////////////////////// SVD /////////////////////////////////////////////













/////////////////////////////////// Multiply-with-Carry RNG ///////////////////////////////////
























///////////////////////////////////////// LineIterator ////////////////////////////////////////










/** returns the next unifomly-distributed random number of the specified type */
@Namespace("cv") public static native @Name("randu<int>") int randInt();
@Namespace("cv") public static native @Name("randu<float>") float randFloat();
@Namespace("cv") public static native @Name("randu<double>") double randDouble();

///////////////////////////////// Formatted string generation /////////////////////////////////

///////////////////////////////// Formatted output of cv::Mat /////////////////////////////////

@Namespace("cv") public static native @Ptr Formatted format(@ByVal Mat mtx, int fmt);

@Namespace("cv") public static native int print(@Ptr Formatted fmtd, @Cast("FILE*") Pointer stream/*=stdout*/);
@Namespace("cv") public static native int print(@Ptr Formatted fmtd);

@Namespace("cv") public static native int print(@Const @ByRef Mat mtx, @Cast("FILE*") Pointer stream/*=stdout*/);
@Namespace("cv") public static native int print(@Const @ByRef Mat mtx);

@Namespace("cv") public static native int print(@Const @ByRef UMat mtx, @Cast("FILE*") Pointer stream/*=stdout*/);
@Namespace("cv") public static native int print(@Const @ByRef UMat mtx);



////////////////////////////////////////// Algorithm //////////////////////////////////////////




















/****************************************************************************************\
*                                  Auxiliary algorithms                                  *
\****************************************************************************************/

// This function splits the input sequence or set into one or more equivalence classes and
// returns the vector of labels - 0-based class indexes for each element.
// predicate(a,b) returns true if the two sequence elements certainly belong to the same class.
//
// The algorithm is described in "Introduction to Algorithms"
// by Cormen, Leiserson and Rivest, the chapter "Data structures for disjoint sets"

 // cv

// #endif


// Parsed from <opencv2/core/bufferpool.hpp>

// This file is part of OpenCV project.
// It is subject to the license terms in the LICENSE file found in the top-level directory
// of this distribution and at http://opencv.org/license.html.
//
// Copyright (C) 2014, Advanced Micro Devices, Inc., all rights reserved.

// #ifndef __OPENCV_CORE_BUFFER_POOL_HPP__
// #define __OPENCV_CORE_BUFFER_POOL_HPP__

@Namespace("cv") public static class BufferPoolController extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public BufferPoolController() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public BufferPoolController(Pointer p) { super(p); }

    public native @Cast("size_t") long getReservedSize();
    public native @Cast("size_t") long getMaxReservedSize();
    public native void setMaxReservedSize(@Cast("size_t") long size);
    public native void freeAllReservedBuffers();
}



// #endif // __OPENCV_CORE_BUFFER_POOL_HPP__


// Parsed from <opencv2/core/mat.hpp>

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

// #ifndef __OPENCV_CORE_MAT_HPP__
// #define __OPENCV_CORE_MAT_HPP__

// #ifndef __cplusplus
// #endif

// #include "opencv2/core/matx.hpp"
// #include "opencv2/core/types.hpp"

// #include "opencv2/core/bufferpool.hpp"

/** enum cv:: */
public static final int ACCESS_READ= 1<<24, ACCESS_WRITE= 1<<25,
    ACCESS_RW= 3<<24, ACCESS_MASK= ACCESS_RW, ACCESS_FAST= 1<<26;

//////////////////////// Input/Output Array Arguments /////////////////////////////////

/**
 Proxy datatype for passing Mat's and vector<>'s as input parameters
 */


/**
 Proxy datatype for passing Mat's and vector<>'s as input parameters
 */



/////////////////////////////////// MatAllocator //////////////////////////////////////

/** Usage flags for allocator */
/** enum cv::UMatUsageFlags */
public static final int
    USAGE_DEFAULT = 0,

    // default allocation policy is platform and usage specific
    USAGE_ALLOCATE_HOST_MEMORY =  1 << 0,
    USAGE_ALLOCATE_DEVICE_MEMORY =  1 << 1,

    __UMAT_USAGE_FLAGS_32BIT =  0x7fffffff; // Binary compatibility hint

/**
   Custom array allocator

*/
@Namespace("cv") public static class MatAllocator extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public MatAllocator() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public MatAllocator(Pointer p) { super(p); }


    // let's comment it off for now to detect and fix all the uses of allocator
    //virtual void allocate(int dims, const int* sizes, int type, int*& refcount,
    //                      uchar*& datastart, uchar*& data, size_t* step) = 0;
    //virtual void deallocate(int* refcount, uchar* datastart, uchar* data) = 0;
    public native @Name("allocate") UMatData _allocate(int dims, @Const IntPointer sizes, int type,
                                   Pointer data, @Cast("size_t*") SizeTPointer step, int flags, @Cast("cv::UMatUsageFlags") int usageFlags);
    public native @Name("allocate") UMatData _allocate(int dims, @Const IntBuffer sizes, int type,
                                   Pointer data, @Cast("size_t*") SizeTPointer step, int flags, @Cast("cv::UMatUsageFlags") int usageFlags);
    public native @Name("allocate") UMatData _allocate(int dims, @Const int[] sizes, int type,
                                   Pointer data, @Cast("size_t*") SizeTPointer step, int flags, @Cast("cv::UMatUsageFlags") int usageFlags);
    public native @Cast("bool") @Name("allocate") boolean _allocate(UMatData data, int accessflags, @Cast("cv::UMatUsageFlags") int usageFlags);
    public native @Name("deallocate") void _deallocate(UMatData data);
    public native void map(UMatData data, int accessflags);
    public native void unmap(UMatData data);
    public native void download(UMatData data, Pointer dst, int dims, @Cast("const size_t*") SizeTPointer sz,
                              @Cast("const size_t*") SizeTPointer srcofs, @Cast("const size_t*") SizeTPointer srcstep,
                              @Cast("const size_t*") SizeTPointer dststep);
    public native void upload(UMatData data, @Const Pointer src, int dims, @Cast("const size_t*") SizeTPointer sz,
                            @Cast("const size_t*") SizeTPointer dstofs, @Cast("const size_t*") SizeTPointer dststep,
                            @Cast("const size_t*") SizeTPointer srcstep);
    public native void copy(UMatData srcdata, UMatData dstdata, int dims, @Cast("const size_t*") SizeTPointer sz,
                          @Cast("const size_t*") SizeTPointer srcofs, @Cast("const size_t*") SizeTPointer srcstep,
                          @Cast("const size_t*") SizeTPointer dstofs, @Cast("const size_t*") SizeTPointer dststep, @Cast("bool") boolean sync);

    // default implementation returns DummyBufferPoolController
    public native BufferPoolController getBufferPoolController();
}


//////////////////////////////// MatCommaInitializer //////////////////////////////////

/**
 Comma-separated Matrix Initializer

 The class instances are usually not created explicitly.
 Instead, they are created on "matrix << firstValue" operator.

 The sample below initializes 2x2 rotation matrix:

 \code
 double angle = 30, a = cos(angle*CV_PI/180), b = sin(angle*CV_PI/180);
 Mat R = (Mat_<double>(2,2) << a, -b, b, a);
 \endcode
*/


/////////////////////////////////////// Mat ///////////////////////////////////////////

// note that umatdata might be allocated together
// with the matrix data, not as a separate object.
// therefore, it does not have constructor or destructor;
// it should be explicitly initialized using init().
@Namespace("cv") @NoOffset public static class UMatData extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public UMatData() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public UMatData(Pointer p) { super(p); }

    /** enum cv::UMatData:: */
    public static final int COPY_ON_MAP= 1, HOST_COPY_OBSOLETE= 2,
        DEVICE_COPY_OBSOLETE= 4, TEMP_UMAT= 8, TEMP_COPIED_UMAT= 24,
        USER_ALLOCATED= 32, DEVICE_MEM_MAPPED= 64;
    public UMatData(@Const MatAllocator allocator) { allocate(allocator); }
    private native void allocate(@Const MatAllocator allocator);

    // provide atomic access to the structure
    public native void lock();
    public native void unlock();

    public native @Cast("bool") boolean hostCopyObsolete();
    public native @Cast("bool") boolean deviceCopyObsolete();
    public native @Cast("bool") boolean deviceMemMapped();
    public native @Cast("bool") boolean copyOnMap();
    public native @Cast("bool") boolean tempUMat();
    public native @Cast("bool") boolean tempCopiedUMat();
    public native void markHostCopyObsolete(@Cast("bool") boolean flag);
    public native void markDeviceCopyObsolete(@Cast("bool") boolean flag);
    public native void markDeviceMemMapped(@Cast("bool") boolean flag);

    @MemberGetter public native @Const MatAllocator prevAllocator();
    @MemberGetter public native @Const MatAllocator currAllocator();
    public native int urefcount(); public native UMatData urefcount(int urefcount);
    public native int refcount(); public native UMatData refcount(int refcount);
    public native @Cast("uchar*") BytePointer data(); public native UMatData data(BytePointer data);
    public native @Cast("uchar*") BytePointer origdata(); public native UMatData origdata(BytePointer origdata);
    public native @Cast("size_t") long size(); public native UMatData size(long size);
    public native @Cast("size_t") @Name("capacity") long _capacity(); public native UMatData _capacity(long _capacity);

    public native int flags(); public native UMatData flags(int flags);
    public native Pointer handle(); public native UMatData handle(Pointer handle);
    public native Pointer userdata(); public native UMatData userdata(Pointer userdata);
    public native int allocatorFlags_(); public native UMatData allocatorFlags_(int allocatorFlags_);
}


@Namespace("cv") @NoOffset public static class UMatDataAutoLock extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public UMatDataAutoLock() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public UMatDataAutoLock(Pointer p) { super(p); }

    public UMatDataAutoLock(UMatData u) { allocate(u); }
    private native void allocate(UMatData u);
    public native UMatData u(); public native UMatDataAutoLock u(UMatData u);
}


@Namespace("cv") @NoOffset public static class MatSize extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public MatSize() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public MatSize(Pointer p) { super(p); }

    public MatSize(IntPointer _p) { allocate(_p); }
    private native void allocate(IntPointer _p);
    public MatSize(IntBuffer _p) { allocate(_p); }
    private native void allocate(IntBuffer _p);
    public MatSize(int[] _p) { allocate(_p); }
    private native void allocate(int[] _p);
    public native @ByVal @Name("operator()") Size apply();
    public native @ByRef @Name("operator[]") IntPointer get(int i);
    public native @Const @Name("operator const int*") IntPointer asIntPointer();
    public native @Cast("bool") @Name("operator==") boolean equals(@Const @ByRef MatSize sz);
    public native @Cast("bool") @Name("operator!=") boolean notEquals(@Const @ByRef MatSize sz);

    public native IntPointer p(); public native MatSize p(IntPointer p);
}

@Namespace("cv") @NoOffset public static class MatStep extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public MatStep(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public MatStep(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public MatStep position(int position) {
        return (MatStep)super.position(position);
    }

    public MatStep() { allocate(); }
    private native void allocate();
    public MatStep(@Cast("size_t") long s) { allocate(s); }
    private native void allocate(@Cast("size_t") long s);
    public native @Cast("size_t*") @ByRef @Name("operator[]") SizeTPointer get(int i);
    public native @Name("operator size_t") long asLong();
    public native @ByRef @Name("operator=") MatStep put(@Cast("size_t") long s);

    public native @Cast("size_t*") SizeTPointer p(); public native MatStep p(SizeTPointer p);
    public native @Cast("size_t") long buf(int i); public native MatStep buf(int i, long buf);
    @MemberGetter public native @Cast("size_t*") SizeTPointer buf();
}

 /**
   The n-dimensional matrix class.

   The class represents an n-dimensional dense numerical array that can act as
   a matrix, image, optical flow map, 3-focal tensor etc.
   It is very similar to CvMat and CvMatND types from earlier versions of OpenCV,
   and similarly to those types, the matrix can be multi-channel. It also fully supports ROI mechanism.

   There are many different ways to create cv::Mat object. Here are the some popular ones:
   <ul>
   <li> using cv::Mat::create(nrows, ncols, type) method or
     the similar constructor cv::Mat::Mat(nrows, ncols, type[, fill_value]) constructor.
     A new matrix of the specified size and specifed type will be allocated.
     "type" has the same meaning as in cvCreateMat function,
     e.g. CV_8UC1 means 8-bit single-channel matrix, CV_32FC2 means 2-channel (i.e. complex)
     floating-point matrix etc:

     \code
     // make 7x7 complex matrix filled with 1+3j.
     cv::Mat M(7,7,CV_32FC2,Scalar(1,3));
     // and now turn M to 100x60 15-channel 8-bit matrix.
     // The old content will be deallocated
     M.create(100,60,CV_8UC(15));
     \endcode

     As noted in the introduction of this chapter, Mat::create()
     will only allocate a new matrix when the current matrix dimensionality
     or type are different from the specified.

   <li> by using a copy constructor or assignment operator, where on the right side it can
     be a matrix or expression, see below. Again, as noted in the introduction,
     matrix assignment is O(1) operation because it only copies the header
     and increases the reference counter. cv::Mat::clone() method can be used to get a full
     (a.k.a. deep) copy of the matrix when you need it.

   <li> by constructing a header for a part of another matrix. It can be a single row, single column,
     several rows, several columns, rectangular region in the matrix (called a minor in algebra) or
     a diagonal. Such operations are also O(1), because the new header will reference the same data.
     You can actually modify a part of the matrix using this feature, e.g.

     \code
     // add 5-th row, multiplied by 3 to the 3rd row
     M.row(3) = M.row(3) + M.row(5)*3;

     // now copy 7-th column to the 1-st column
     // M.col(1) = M.col(7); // this will not work
     Mat M1 = M.col(1);
     M.col(7).copyTo(M1);

     // create new 320x240 image
     cv::Mat img(Size(320,240),CV_8UC3);
     // select a roi
     cv::Mat roi(img, Rect(10,10,100,100));
     // fill the ROI with (0,255,0) (which is green in RGB space);
     // the original 320x240 image will be modified
     roi = Scalar(0,255,0);
     \endcode

     Thanks to the additional cv::Mat::datastart and cv::Mat::dataend members, it is possible to
     compute the relative sub-matrix position in the main "container" matrix using cv::Mat::locateROI():

     \code
     Mat A = Mat::eye(10, 10, CV_32S);
     // extracts A columns, 1 (inclusive) to 3 (exclusive).
     Mat B = A(Range::all(), Range(1, 3));
     // extracts B rows, 5 (inclusive) to 9 (exclusive).
     // that is, C ~ A(Range(5, 9), Range(1, 3))
     Mat C = B(Range(5, 9), Range::all());
     Size size; Point ofs;
     C.locateROI(size, ofs);
     // size will be (width=10,height=10) and the ofs will be (x=1, y=5)
     \endcode

     As in the case of whole matrices, if you need a deep copy, use cv::Mat::clone() method
     of the extracted sub-matrices.

   <li> by making a header for user-allocated-data. It can be useful for
      <ol>
      <li> processing "foreign" data using OpenCV (e.g. when you implement
         a DirectShow filter or a processing module for gstreamer etc.), e.g.

         \code
         void process_video_frame(const unsigned char* pixels,
                                  int width, int height, int step)
         {
            cv::Mat img(height, width, CV_8UC3, pixels, step);
            cv::GaussianBlur(img, img, cv::Size(7,7), 1.5, 1.5);
         }
         \endcode

      <li> for quick initialization of small matrices and/or super-fast element access

         \code
         double m[3][3] = {{a, b, c}, {d, e, f}, {g, h, i}};
         cv::Mat M = cv::Mat(3, 3, CV_64F, m).inv();
         \endcode
      </ol>

       partial yet very common cases of this "user-allocated data" case are conversions
       from CvMat and IplImage to cv::Mat. For this purpose there are special constructors
       taking pointers to CvMat or IplImage and the optional
       flag indicating whether to copy the data or not.

       Backward conversion from cv::Mat to CvMat or IplImage is provided via cast operators
       cv::Mat::operator CvMat() an cv::Mat::operator IplImage().
       The operators do not copy the data.


       \code
       IplImage* img = cvLoadImage("greatwave.jpg", 1);
       Mat mtx(img); // convert IplImage* -> cv::Mat
       CvMat oldmat = mtx; // convert cv::Mat -> CvMat
       CV_Assert(oldmat.cols == img->width && oldmat.rows == img->height &&
           oldmat.data.ptr == (uchar*)img->imageData && oldmat.step == img->widthStep);
       \endcode

   <li> by using MATLAB-style matrix initializers, cv::Mat::zeros(), cv::Mat::ones(), cv::Mat::eye(), e.g.:

   \code
   // create a double-precision identity martix and add it to M.
   M += Mat::eye(M.rows, M.cols, CV_64F);
   \endcode

   <li> by using comma-separated initializer:

   \code
   // create 3x3 double-precision identity matrix
   Mat M = (Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
   \endcode

   here we first call constructor of cv::Mat_ class (that we describe further) with the proper matrix,
   and then we just put "<<" operator followed by comma-separated values that can be constants,
   variables, expressions etc. Also, note the extra parentheses that are needed to avoid compiler errors.

   </ul>

   Once matrix is created, it will be automatically managed by using reference-counting mechanism
   (unless the matrix header is built on top of user-allocated data,
   in which case you should handle the data by yourself).
   The matrix data will be deallocated when no one points to it;
   if you want to release the data pointed by a matrix header before the matrix destructor is called,
   use cv::Mat::release().

   The next important thing to learn about the matrix class is element access. Here is how the matrix is stored.
   The elements are stored in row-major order (row by row). The cv::Mat::data member points to the first element of the first row,
   cv::Mat::rows contains the number of matrix rows and cv::Mat::cols - the number of matrix columns. There is yet another member,
   cv::Mat::step that is used to actually compute address of a matrix element. cv::Mat::step is needed because the matrix can be
   a part of another matrix or because there can some padding space in the end of each row for a proper alignment.

   Given these parameters, address of the matrix element M_{ij} is computed as following:

   addr(M_{ij})=M.data + M.step*i + j*M.elemSize()

   if you know the matrix element type, e.g. it is float, then you can use cv::Mat::at() method:

   addr(M_{ij})=&M.at<float>(i,j)

   (where & is used to convert the reference returned by cv::Mat::at() to a pointer).
   if you need to process a whole row of matrix, the most efficient way is to get
   the pointer to the row first, and then just use plain C operator []:

   \code
   // compute sum of positive matrix elements
   // (assuming that M is double-precision matrix)
   double sum=0;
   for(int i = 0; i < M.rows; i++)
   {
       const double* Mi = M.ptr<double>(i);
       for(int j = 0; j < M.cols; j++)
           sum += std::max(Mi[j], 0.);
   }
   \endcode

   Some operations, like the above one, do not actually depend on the matrix shape,
   they just process elements of a matrix one by one (or elements from multiple matrices
   that are sitting in the same place, e.g. matrix addition). Such operations are called
   element-wise and it makes sense to check whether all the input/output matrices are continuous,
   i.e. have no gaps in the end of each row, and if yes, process them as a single long row:

   \code
   // compute sum of positive matrix elements, optimized variant
   double sum=0;
   int cols = M.cols, rows = M.rows;
   if(M.isContinuous())
   {
       cols *= rows;
       rows = 1;
   }
   for(int i = 0; i < rows; i++)
   {
       const double* Mi = M.ptr<double>(i);
       for(int j = 0; j < cols; j++)
           sum += std::max(Mi[j], 0.);
   }
   \endcode
   in the case of continuous matrix the outer loop body will be executed just once,
   so the overhead will be smaller, which will be especially noticeable in the case of small matrices.

   Finally, there are STL-style iterators that are smart enough to skip gaps between successive rows:
   \code
   // compute sum of positive matrix elements, iterator-based variant
   double sum=0;
   MatConstIterator_<double> it = M.begin<double>(), it_end = M.end<double>();
   for(; it != it_end; ++it)
       sum += std::max(*it, 0.);
   \endcode

   The matrix iterators are random-access iterators, so they can be passed
   to any STL algorithm, including std::sort().
*/
@Namespace("cv") @NoOffset public static class Mat extends AbstractMat {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Mat(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public Mat(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public Mat position(int position) {
        return (Mat)super.position(position);
    }

    /** default constructor */
    public Mat() { allocate(); }
    private native void allocate();
    /** constructs 2D matrix of the specified size and type */
    // (_type is CV_8UC1, CV_64FC3, CV_32SC(12) etc.)
    public Mat(int rows, int cols, int type) { allocate(rows, cols, type); }
    private native void allocate(int rows, int cols, int type);
    public Mat(@ByVal Size size, int type) { allocate(size, type); }
    private native void allocate(@ByVal Size size, int type);
    /** constucts 2D matrix and fills it with the specified value _s. */
    public Mat(int rows, int cols, int type, @Const @ByRef Scalar s) { allocate(rows, cols, type, s); }
    private native void allocate(int rows, int cols, int type, @Const @ByRef Scalar s);
    public Mat(@ByVal Size size, int type, @Const @ByRef Scalar s) { allocate(size, type, s); }
    private native void allocate(@ByVal Size size, int type, @Const @ByRef Scalar s);

    /** constructs n-dimensional matrix */
    public Mat(int ndims, @Const IntPointer sizes, int type) { allocate(ndims, sizes, type); }
    private native void allocate(int ndims, @Const IntPointer sizes, int type);
    public Mat(int ndims, @Const IntBuffer sizes, int type) { allocate(ndims, sizes, type); }
    private native void allocate(int ndims, @Const IntBuffer sizes, int type);
    public Mat(int ndims, @Const int[] sizes, int type) { allocate(ndims, sizes, type); }
    private native void allocate(int ndims, @Const int[] sizes, int type);
    public Mat(int ndims, @Const IntPointer sizes, int type, @Const @ByRef Scalar s) { allocate(ndims, sizes, type, s); }
    private native void allocate(int ndims, @Const IntPointer sizes, int type, @Const @ByRef Scalar s);
    public Mat(int ndims, @Const IntBuffer sizes, int type, @Const @ByRef Scalar s) { allocate(ndims, sizes, type, s); }
    private native void allocate(int ndims, @Const IntBuffer sizes, int type, @Const @ByRef Scalar s);
    public Mat(int ndims, @Const int[] sizes, int type, @Const @ByRef Scalar s) { allocate(ndims, sizes, type, s); }
    private native void allocate(int ndims, @Const int[] sizes, int type, @Const @ByRef Scalar s);

    /** copy constructor */
    public Mat(@Const @ByRef Mat m) { allocate(m); }
    private native void allocate(@Const @ByRef Mat m);
    /** constructor for matrix headers pointing to user-allocated data */
    public Mat(int rows, int cols, int type, Pointer data, @Cast("size_t") long step/*=AUTO_STEP*/) { allocate(rows, cols, type, data, step); this.data = data; }
    private native void allocate(int rows, int cols, int type, Pointer data, @Cast("size_t") long step/*=AUTO_STEP*/);
    private Pointer data; // a reference to prevent deallocation
    public Mat(int rows, int cols, int type, Pointer data) { this(rows, cols, type, data, AUTO_STEP); }
    public Mat(byte ... b) { this(b, false); }
    public Mat(byte[] b, boolean signed) { this(new BytePointer(b), signed); }
    public Mat(short ... s) { this(s, true); }
    public Mat(short[] s, boolean signed) { this(new ShortPointer(s), signed); }
    public Mat(int ... n) { this(new IntPointer(n)); }
    public Mat(double ... d) { this(new DoublePointer(d)); }
    public Mat(float ... f) { this(new FloatPointer(f)); }
    public Mat(BytePointer p, boolean signed) { this(p.limit - p.position, 1, signed ? CV_8SC1 : CV_8UC1, p); }
    public Mat(ShortPointer p, boolean signed) { this(p.limit - p.position, 1, signed ? CV_16SC1 : CV_16UC1, p); }
    public Mat(IntPointer p) { this(p.limit - p.position, 1, CV_32SC1, p); }
    public Mat(FloatPointer p) { this(p.limit - p.position, 1, CV_32FC1, p); }
    public Mat(DoublePointer p) { this(p.limit - p.position, 1, CV_64FC1, p); }
    public Mat(@ByVal Size size, int type, Pointer data, @Cast("size_t") long step/*=AUTO_STEP*/) { allocate(size, type, data, step); }
    private native void allocate(@ByVal Size size, int type, Pointer data, @Cast("size_t") long step/*=AUTO_STEP*/);
    public Mat(@ByVal Size size, int type, Pointer data) { allocate(size, type, data); }
    private native void allocate(@ByVal Size size, int type, Pointer data);
    public Mat(int ndims, @Const IntPointer sizes, int type, Pointer data, @Cast("const size_t*") SizeTPointer steps/*=0*/) { allocate(ndims, sizes, type, data, steps); }
    private native void allocate(int ndims, @Const IntPointer sizes, int type, Pointer data, @Cast("const size_t*") SizeTPointer steps/*=0*/);
    public Mat(int ndims, @Const IntPointer sizes, int type, Pointer data) { allocate(ndims, sizes, type, data); }
    private native void allocate(int ndims, @Const IntPointer sizes, int type, Pointer data);
    public Mat(int ndims, @Const IntBuffer sizes, int type, Pointer data, @Cast("const size_t*") SizeTPointer steps/*=0*/) { allocate(ndims, sizes, type, data, steps); }
    private native void allocate(int ndims, @Const IntBuffer sizes, int type, Pointer data, @Cast("const size_t*") SizeTPointer steps/*=0*/);
    public Mat(int ndims, @Const IntBuffer sizes, int type, Pointer data) { allocate(ndims, sizes, type, data); }
    private native void allocate(int ndims, @Const IntBuffer sizes, int type, Pointer data);
    public Mat(int ndims, @Const int[] sizes, int type, Pointer data, @Cast("const size_t*") SizeTPointer steps/*=0*/) { allocate(ndims, sizes, type, data, steps); }
    private native void allocate(int ndims, @Const int[] sizes, int type, Pointer data, @Cast("const size_t*") SizeTPointer steps/*=0*/);
    public Mat(int ndims, @Const int[] sizes, int type, Pointer data) { allocate(ndims, sizes, type, data); }
    private native void allocate(int ndims, @Const int[] sizes, int type, Pointer data);

    /** creates a matrix header for a part of the bigger matrix */
    public Mat(@Const @ByRef Mat m, @Const @ByRef Range rowRange, @Const @ByRef Range colRange/*=Range::all()*/) { allocate(m, rowRange, colRange); }
    private native void allocate(@Const @ByRef Mat m, @Const @ByRef Range rowRange, @Const @ByRef Range colRange/*=Range::all()*/);
    public Mat(@Const @ByRef Mat m, @Const @ByRef Range rowRange) { allocate(m, rowRange); }
    private native void allocate(@Const @ByRef Mat m, @Const @ByRef Range rowRange);
    public Mat(@Const @ByRef Mat m, @Const @ByRef Rect roi) { allocate(m, roi); }
    private native void allocate(@Const @ByRef Mat m, @Const @ByRef Rect roi);
    /** builds matrix from std::vector with or without copying the data */
    /** builds matrix from cv::Vec; the data is copied by default */
    /** builds matrix from cv::Matx; the data is copied by default */
    /** builds matrix from a 2D point */
    /** builds matrix from a 3D point */
    /** builds matrix from comma initializer */

    /** download data from GpuMat */
    public Mat(@Const @ByRef GpuMat m) { allocate(m); }
    private native void allocate(@Const @ByRef GpuMat m);

    /** destructor - calls release() */
    /** assignment operators */
    public native @ByRef @Name("operator=") Mat put(@Const @ByRef Mat m);
    public native @ByRef @Name("operator=") Mat put(@Const @ByRef MatExpr expr);

    /** retrieve UMat from Mat */
    public native @ByVal UMat getUMat(int accessFlags, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public native @ByVal UMat getUMat(int accessFlags);

    /** returns a new matrix header for the specified row */
    public native @ByVal Mat row(int y);
    /** returns a new matrix header for the specified column */
    public native @ByVal Mat col(int x);
    /** ... for the specified row span */
    public native @ByVal Mat rowRange(int startrow, int endrow);
    public native @ByVal Mat rowRange(@Const @ByRef Range r);
    /** ... for the specified column span */
    public native @ByVal Mat colRange(int startcol, int endcol);
    public native @ByVal Mat colRange(@Const @ByRef Range r);
    /** ... for the specified diagonal */
    // (d=0 - the main diagonal,
    //  >0 - a diagonal from the lower half,
    //  <0 - a diagonal from the upper half)
    public native @ByVal Mat diag(int d/*=0*/);
    public native @ByVal Mat diag();
    /** constructs a square diagonal matrix which main diagonal is vector "d" */
    public static native @ByVal Mat diag(@Const @ByRef Mat d);

    /** returns deep copy of the matrix, i.e. the data is copied */
    public native @ByVal Mat clone();
    /** copies the matrix content to "m". */
    // It calls m.create(this->size(), this->type()).
    public native void copyTo( @ByVal Mat m );
    /** copies those matrix elements to "m" that are marked with non-zero mask elements. */
    public native void copyTo( @ByVal Mat m, @ByVal Mat mask );
    /** converts matrix to another datatype with optional scalng. See cvConvertScale. */
    public native void convertTo( @ByVal Mat m, int rtype, double alpha/*=1*/, double beta/*=0*/ );
    public native void convertTo( @ByVal Mat m, int rtype );

    public native void assignTo( @ByRef Mat m, int type/*=-1*/ );
    public native void assignTo( @ByRef Mat m );

    /** sets every matrix element to s */
    public native @ByRef @Name("operator=") Mat put(@Const @ByRef Scalar s);
    /** sets some of the matrix elements to s, according to the mask */
    public native @ByRef Mat setTo(@ByVal Mat value, @ByVal Mat mask/*=noArray()*/);
    public native @ByRef Mat setTo(@ByVal Mat value);
    /** creates alternative matrix header for the same data, with different */
    // number of channels and/or different number of rows. see cvReshape.
    public native @ByVal Mat reshape(int cn, int rows/*=0*/);
    public native @ByVal Mat reshape(int cn);
    public native @ByVal Mat reshape(int cn, int newndims, @Const IntPointer newsz);
    public native @ByVal Mat reshape(int cn, int newndims, @Const IntBuffer newsz);
    public native @ByVal Mat reshape(int cn, int newndims, @Const int[] newsz);

    /** matrix transposition by means of matrix expressions */
    public native @ByVal MatExpr t();
    /** matrix inversion by means of matrix expressions */
    public native @ByVal MatExpr inv(int method/*=DECOMP_LU*/);
    public native @ByVal MatExpr inv();
    /** per-element matrix multiplication by means of matrix expressions */
    public native @ByVal MatExpr mul(@ByVal Mat m, double scale/*=1*/);
    public native @ByVal MatExpr mul(@ByVal Mat m);

    /** computes cross-product of 2 3D vectors */
    public native @ByVal Mat cross(@ByVal Mat m);
    /** computes dot-product */
    public native double dot(@ByVal Mat m);

    /** Matlab-style matrix initialization */
    public static native @ByVal MatExpr zeros(int rows, int cols, int type);
    public static native @ByVal MatExpr zeros(@ByVal Size size, int type);
    
    public static native @ByVal MatExpr ones(int rows, int cols, int type);
    public static native @ByVal MatExpr ones(@ByVal Size size, int type);
    
    public static native @ByVal MatExpr eye(int rows, int cols, int type);
    public static native @ByVal MatExpr eye(@ByVal Size size, int type);

    /** allocates new matrix data unless the matrix already has specified size and type. */
    // previous data is unreferenced if needed.
    public native void create(int rows, int cols, int type);
    public native void create(@ByVal Size size, int type);
    public native void create(int ndims, @Const IntPointer sizes, int type);
    public native void create(int ndims, @Const IntBuffer sizes, int type);
    public native void create(int ndims, @Const int[] sizes, int type);

    /** increases the reference counter; use with care to avoid memleaks */
    public native void addref();
    /** decreases reference counter; */
    // deallocates the data when reference counter reaches 0.
    public native void release();

    /** deallocates the matrix data */
    public native @Name("deallocate") void _deallocate();
    /** internal use function; properly re-allocates _size, _step arrays */
    public native void copySize(@Const @ByRef Mat m);

    /** reserves enough space to fit sz hyper-planes */
    public native void reserve(@Cast("size_t") long sz);
    /** resizes matrix to the specified number of hyper-planes */
    public native void resize(@Cast("size_t") long sz);
    /** resizes matrix to the specified number of hyper-planes; initializes the newly added elements */
    public native void resize(@Cast("size_t") long sz, @Const @ByRef Scalar s);
    /** internal function */
    public native void push_back_(@Const Pointer elem);
    /** adds element to the end of 1d matrix (or possibly multiple elements when _Tp=Mat) */
    public native void push_back(@Const @ByRef Mat m);
    /** removes several hyper-planes from bottom of the matrix */
    public native void pop_back(@Cast("size_t") long nelems/*=1*/);
    public native void pop_back();

    /** locates matrix header within a parent matrix. See below */
    public native void locateROI( @ByRef Size wholeSize, @ByRef Point ofs );
    /** moves/resizes the current matrix ROI inside the parent matrix. */
    public native @ByRef Mat adjustROI( int dtop, int dbottom, int dleft, int dright );
    /** extracts a rectangular sub-matrix */
    // (this is a generalized form of row, rowRange etc.)
    public native @ByVal @Name("operator()") Mat apply( @ByVal Range rowRange, @ByVal Range colRange );
    public native @ByVal @Name("operator()") Mat apply( @Const @ByRef Rect roi );
    public native @ByVal @Name("operator()") Mat apply( @Const Range ranges );

    // //! converts header to CvMat; no data is copied
    // operator CvMat() const;
    // //! converts header to CvMatND; no data is copied
    // operator CvMatND() const;
    // //! converts header to IplImage; no data is copied
    // operator IplImage() const;

    /** returns true iff the matrix data is continuous */
    // (i.e. when there are no gaps between successive rows).
    // similar to CV_IS_MAT_CONT(cvmat->type)
    public native @Cast("bool") boolean isContinuous();

    /** returns true if the matrix is a submatrix of another matrix */
    public native @Cast("bool") boolean isSubmatrix();

    /** returns element size in bytes, */
    // similar to CV_ELEM_SIZE(cvmat->type)
    public native @Cast("size_t") long elemSize();
    /** returns the size of element channel in bytes. */
    public native @Cast("size_t") long elemSize1();
    /** returns element type, similar to CV_MAT_TYPE(cvmat->type) */
    public native int type();
    /** returns element type, similar to CV_MAT_DEPTH(cvmat->type) */
    public native int depth();
    /** returns element type, similar to CV_MAT_CN(cvmat->type) */
    public native int channels();
    /** returns step/elemSize1() */
    public native @Cast("size_t") long step1(int i/*=0*/);
    public native @Cast("size_t") long step1();
    /** returns true if matrix data is NULL */
    public native @Cast("bool") boolean empty();
    /** returns the total number of matrix elements */
    public native @Cast("size_t") long total();

    /** returns N if the matrix is 1-channel (N x ptdim) or ptdim-channel (1 x N) or (N x 1); negative number otherwise */
    public native int checkVector(int elemChannels, int depth/*=-1*/, @Cast("bool") boolean requireContinuous/*=true*/);
    public native int checkVector(int elemChannels);

    /** returns pointer to i0-th submatrix along the dimension #0 */
    public native @Cast("uchar*") BytePointer ptr(int i0/*=0*/);
    public native @Cast("uchar*") BytePointer ptr();

    /** returns pointer to (i0,i1) submatrix along the dimensions #0 and #1 */
    public native @Cast("uchar*") BytePointer ptr(int i0, int i1);

    /** returns pointer to (i0,i1,i3) submatrix along the dimensions #0, #1, #2 */
    public native @Cast("uchar*") BytePointer ptr(int i0, int i1, int i2);

    /** returns pointer to the matrix element */
    public native @Cast("uchar*") BytePointer ptr(@Const IntPointer idx);
    public native @Cast("uchar*") ByteBuffer ptr(@Const IntBuffer idx);
    public native @Cast("uchar*") byte[] ptr(@Const int[] idx);
    /** returns read-only pointer to the matrix element */

    /** template version of the above method */

    /** the same as above, with the pointer dereferencing */

    /** special versions for 2D arrays (especially convenient for referencing image pixels) */

    /** template methods for iteration over matrix elements. */
    // the iterators take care of skipping gaps in the end of rows (if any)

    /** template methods for for operation over all matrix elements. */
    // the operations take care of skipping gaps in the end of rows (if any)

    /** enum cv::Mat:: */
    public static final int MAGIC_VAL  =  0x42FF0000, AUTO_STEP = 0, CONTINUOUS_FLAG =  CV_MAT_CONT_FLAG, SUBMATRIX_FLAG =  CV_SUBMAT_FLAG;
    /** enum cv::Mat:: */
    public static final int MAGIC_MASK =  0xFFFF0000, TYPE_MASK =  0x00000FFF, DEPTH_MASK = 7;

    /** includes several bit-fields:
         - the magic signature
         - continuity flag
         - depth
         - number of channels
     */
    public native int flags(); public native Mat flags(int flags);
    /** the matrix dimensionality, >= 2 */
    public native int dims(); public native Mat dims(int dims);
    /** the number of rows and columns or (-1, -1) when the matrix has more than 2 dimensions */
    public native int rows(); public native Mat rows(int rows);
    public native int cols(); public native Mat cols(int cols);
    /** pointer to the data */
    public native @Cast("uchar*") BytePointer data(); public native Mat data(BytePointer data);

    /** helper fields used in locateROI and adjustROI */
    @MemberGetter public native @Cast("const uchar*") BytePointer datastart();
    @MemberGetter public native @Cast("const uchar*") BytePointer dataend();
    @MemberGetter public native @Cast("const uchar*") BytePointer datalimit();

    /** custom allocator */
    public native MatAllocator allocator(); public native Mat allocator(MatAllocator allocator);
    /** and the standard allocator */
    public static native MatAllocator getStdAllocator();

    /** interaction with UMat */
    public native UMatData u(); public native Mat u(UMatData u);

    public native @ByVal Size size();
    @MemberGetter public native int size(int i);
    @MemberGetter public native long step();
    @MemberGetter public native int step(int i);
}


///////////////////////////////// Mat_<_Tp> ////////////////////////////////////

/**
 Template matrix class derived from Mat

 The class Mat_ is a "thin" template wrapper on top of cv::Mat. It does not have any extra data fields,
 nor it or cv::Mat have any virtual methods and thus references or pointers to these two classes
 can be safely converted one to another. But do it with care, for example:

 \code
 // create 100x100 8-bit matrix
 Mat M(100,100,CV_8U);
 // this will compile fine. no any data conversion will be done.
 Mat_<float>& M1 = (Mat_<float>&)M;
 // the program will likely crash at the statement below
 M1(99,99) = 1.f;
 \endcode

 While cv::Mat is sufficient in most cases, cv::Mat_ can be more convenient if you use a lot of element
 access operations and if you know matrix type at compile time. Note that cv::Mat::at and
 cv::Mat::operator() do absolutely the same thing and run at the same speed, but the latter is certainly shorter:

 \code
 Mat_<double> M(20,20);
 for(int i = 0; i < M.rows; i++)
    for(int j = 0; j < M.cols; j++)
       M(i,j) = 1./(i+j+1);
 Mat E, V;
 eigen(M,E,V);
 cout << E.at<double>(0,0)/E.at<double>(M.rows-1,0);
 \endcode

 It is easy to use Mat_ for multi-channel images/matrices - just pass cv::Vec as cv::Mat_ template parameter:

 \code
 // allocate 320x240 color image and fill it with green (in RGB space)
 Mat_<Vec3b> img(240, 320, Vec3b(0,255,0));
 // now draw a diagonal white line
 for(int i = 0; i < 100; i++)
     img(i,i)=Vec3b(255,255,255);
 // and now modify the 2nd (red) channel of each pixel
 for(int i = 0; i < img.rows; i++)
    for(int j = 0; j < img.cols; j++)
       img(i,j)[2] ^= (uchar)(i ^ j); // img(y,x)[c] accesses c-th channel of the pixel (x,y)
 \endcode
*/

@Namespace("cv") @NoOffset public static class UMat extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public UMat(Pointer p) { super(p); }

    /** default constructor */
    public UMat(@Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/) { allocate(usageFlags); }
    private native void allocate(@Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public UMat() { allocate(); }
    private native void allocate();
    /** constructs 2D matrix of the specified size and type */
    // (_type is CV_8UC1, CV_64FC3, CV_32SC(12) etc.)
    public UMat(int rows, int cols, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/) { allocate(rows, cols, type, usageFlags); }
    private native void allocate(int rows, int cols, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public UMat(int rows, int cols, int type) { allocate(rows, cols, type); }
    private native void allocate(int rows, int cols, int type);
    public UMat(@ByVal Size size, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/) { allocate(size, type, usageFlags); }
    private native void allocate(@ByVal Size size, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public UMat(@ByVal Size size, int type) { allocate(size, type); }
    private native void allocate(@ByVal Size size, int type);
    /** constucts 2D matrix and fills it with the specified value _s. */
    public UMat(int rows, int cols, int type, @Const @ByRef Scalar s, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/) { allocate(rows, cols, type, s, usageFlags); }
    private native void allocate(int rows, int cols, int type, @Const @ByRef Scalar s, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public UMat(int rows, int cols, int type, @Const @ByRef Scalar s) { allocate(rows, cols, type, s); }
    private native void allocate(int rows, int cols, int type, @Const @ByRef Scalar s);
    public UMat(@ByVal Size size, int type, @Const @ByRef Scalar s, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/) { allocate(size, type, s, usageFlags); }
    private native void allocate(@ByVal Size size, int type, @Const @ByRef Scalar s, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public UMat(@ByVal Size size, int type, @Const @ByRef Scalar s) { allocate(size, type, s); }
    private native void allocate(@ByVal Size size, int type, @Const @ByRef Scalar s);

    /** constructs n-dimensional matrix */
    public UMat(int ndims, @Const IntPointer sizes, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/) { allocate(ndims, sizes, type, usageFlags); }
    private native void allocate(int ndims, @Const IntPointer sizes, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public UMat(int ndims, @Const IntPointer sizes, int type) { allocate(ndims, sizes, type); }
    private native void allocate(int ndims, @Const IntPointer sizes, int type);
    public UMat(int ndims, @Const IntBuffer sizes, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/) { allocate(ndims, sizes, type, usageFlags); }
    private native void allocate(int ndims, @Const IntBuffer sizes, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public UMat(int ndims, @Const IntBuffer sizes, int type) { allocate(ndims, sizes, type); }
    private native void allocate(int ndims, @Const IntBuffer sizes, int type);
    public UMat(int ndims, @Const int[] sizes, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/) { allocate(ndims, sizes, type, usageFlags); }
    private native void allocate(int ndims, @Const int[] sizes, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public UMat(int ndims, @Const int[] sizes, int type) { allocate(ndims, sizes, type); }
    private native void allocate(int ndims, @Const int[] sizes, int type);
    public UMat(int ndims, @Const IntPointer sizes, int type, @Const @ByRef Scalar s, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/) { allocate(ndims, sizes, type, s, usageFlags); }
    private native void allocate(int ndims, @Const IntPointer sizes, int type, @Const @ByRef Scalar s, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public UMat(int ndims, @Const IntPointer sizes, int type, @Const @ByRef Scalar s) { allocate(ndims, sizes, type, s); }
    private native void allocate(int ndims, @Const IntPointer sizes, int type, @Const @ByRef Scalar s);
    public UMat(int ndims, @Const IntBuffer sizes, int type, @Const @ByRef Scalar s, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/) { allocate(ndims, sizes, type, s, usageFlags); }
    private native void allocate(int ndims, @Const IntBuffer sizes, int type, @Const @ByRef Scalar s, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public UMat(int ndims, @Const IntBuffer sizes, int type, @Const @ByRef Scalar s) { allocate(ndims, sizes, type, s); }
    private native void allocate(int ndims, @Const IntBuffer sizes, int type, @Const @ByRef Scalar s);
    public UMat(int ndims, @Const int[] sizes, int type, @Const @ByRef Scalar s, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/) { allocate(ndims, sizes, type, s, usageFlags); }
    private native void allocate(int ndims, @Const int[] sizes, int type, @Const @ByRef Scalar s, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public UMat(int ndims, @Const int[] sizes, int type, @Const @ByRef Scalar s) { allocate(ndims, sizes, type, s); }
    private native void allocate(int ndims, @Const int[] sizes, int type, @Const @ByRef Scalar s);

    /** copy constructor */
    public UMat(@Const @ByRef UMat m) { allocate(m); }
    private native void allocate(@Const @ByRef UMat m);

    /** creates a matrix header for a part of the bigger matrix */
    public UMat(@Const @ByRef UMat m, @Const @ByRef Range rowRange, @Const @ByRef Range colRange/*=Range::all()*/) { allocate(m, rowRange, colRange); }
    private native void allocate(@Const @ByRef UMat m, @Const @ByRef Range rowRange, @Const @ByRef Range colRange/*=Range::all()*/);
    public UMat(@Const @ByRef UMat m, @Const @ByRef Range rowRange) { allocate(m, rowRange); }
    private native void allocate(@Const @ByRef UMat m, @Const @ByRef Range rowRange);
    public UMat(@Const @ByRef UMat m, @Const @ByRef Rect roi) { allocate(m, roi); }
    private native void allocate(@Const @ByRef UMat m, @Const @ByRef Rect roi);
    /** builds matrix from std::vector with or without copying the data */
    /** builds matrix from cv::Vec; the data is copied by default */
    /** builds matrix from cv::Matx; the data is copied by default */
    /** builds matrix from a 2D point */
    /** builds matrix from a 3D point */
    /** builds matrix from comma initializer */

    /** destructor - calls release() */
    /** assignment operators */
    public native @ByRef @Name("operator=") UMat put(@Const @ByRef UMat m);

    public native @ByVal Mat getMat(int flags);

    /** returns a new matrix header for the specified row */
    public native @ByVal UMat row(int y);
    /** returns a new matrix header for the specified column */
    public native @ByVal UMat col(int x);
    /** ... for the specified row span */
    public native @ByVal UMat rowRange(int startrow, int endrow);
    public native @ByVal UMat rowRange(@Const @ByRef Range r);
    /** ... for the specified column span */
    public native @ByVal UMat colRange(int startcol, int endcol);
    public native @ByVal UMat colRange(@Const @ByRef Range r);
    /** ... for the specified diagonal */
    // (d=0 - the main diagonal,
    //  >0 - a diagonal from the lower half,
    //  <0 - a diagonal from the upper half)
    public native @ByVal UMat diag(int d/*=0*/);
    public native @ByVal UMat diag();
    /** constructs a square diagonal matrix which main diagonal is vector "d" */
    public static native @ByVal UMat diag(@Const @ByRef UMat d);

    /** returns deep copy of the matrix, i.e. the data is copied */
    public native @ByVal UMat clone();
    /** copies the matrix content to "m". */
    // It calls m.create(this->size(), this->type()).
    public native void copyTo( @ByVal Mat m );
    /** copies those matrix elements to "m" that are marked with non-zero mask elements. */
    public native void copyTo( @ByVal Mat m, @ByVal Mat mask );
    /** converts matrix to another datatype with optional scalng. See cvConvertScale. */
    public native void convertTo( @ByVal Mat m, int rtype, double alpha/*=1*/, double beta/*=0*/ );
    public native void convertTo( @ByVal Mat m, int rtype );

    public native void assignTo( @ByRef UMat m, int type/*=-1*/ );
    public native void assignTo( @ByRef UMat m );

    /** sets every matrix element to s */
    public native @ByRef @Name("operator=") UMat put(@Const @ByRef Scalar s);
    /** sets some of the matrix elements to s, according to the mask */
    public native @ByRef UMat setTo(@ByVal Mat value, @ByVal Mat mask/*=noArray()*/);
    public native @ByRef UMat setTo(@ByVal Mat value);
    /** creates alternative matrix header for the same data, with different */
    // number of channels and/or different number of rows. see cvReshape.
    public native @ByVal UMat reshape(int cn, int rows/*=0*/);
    public native @ByVal UMat reshape(int cn);
    public native @ByVal UMat reshape(int cn, int newndims, @Const IntPointer newsz);
    public native @ByVal UMat reshape(int cn, int newndims, @Const IntBuffer newsz);
    public native @ByVal UMat reshape(int cn, int newndims, @Const int[] newsz);

    /** matrix transposition by means of matrix expressions */
    public native @ByVal UMat t();
    /** matrix inversion by means of matrix expressions */
    public native @ByVal UMat inv(int method/*=DECOMP_LU*/);
    public native @ByVal UMat inv();
    /** per-element matrix multiplication by means of matrix expressions */
    public native @ByVal UMat mul(@ByVal Mat m, double scale/*=1*/);
    public native @ByVal UMat mul(@ByVal Mat m);

    /** computes dot-product */
    public native double dot(@ByVal Mat m);

    /** Matlab-style matrix initialization */
    public static native @ByVal UMat zeros(int rows, int cols, int type);
    public static native @ByVal UMat zeros(@ByVal Size size, int type);
    
    public static native @ByVal UMat ones(int rows, int cols, int type);
    public static native @ByVal UMat ones(@ByVal Size size, int type);
    
    public static native @ByVal UMat eye(int rows, int cols, int type);
    public static native @ByVal UMat eye(@ByVal Size size, int type);

    /** allocates new matrix data unless the matrix already has specified size and type. */
    // previous data is unreferenced if needed.
    public native void create(int rows, int cols, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public native void create(int rows, int cols, int type);
    public native void create(@ByVal Size size, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public native void create(@ByVal Size size, int type);
    public native void create(int ndims, @Const IntPointer sizes, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public native void create(int ndims, @Const IntPointer sizes, int type);
    public native void create(int ndims, @Const IntBuffer sizes, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public native void create(int ndims, @Const IntBuffer sizes, int type);
    public native void create(int ndims, @Const int[] sizes, int type, @Cast("cv::UMatUsageFlags") int usageFlags/*=USAGE_DEFAULT*/);
    public native void create(int ndims, @Const int[] sizes, int type);

    /** increases the reference counter; use with care to avoid memleaks */
    public native void addref();
    /** decreases reference counter; */
    // deallocates the data when reference counter reaches 0.
    public native void release();

    /** deallocates the matrix data */
    public native @Name("deallocate") void _deallocate();
    /** internal use function; properly re-allocates _size, _step arrays */
    public native void copySize(@Const @ByRef UMat m);

    /** locates matrix header within a parent matrix. See below */
    public native void locateROI( @ByRef Size wholeSize, @ByRef Point ofs );
    /** moves/resizes the current matrix ROI inside the parent matrix. */
    public native @ByRef UMat adjustROI( int dtop, int dbottom, int dleft, int dright );
    /** extracts a rectangular sub-matrix */
    // (this is a generalized form of row, rowRange etc.)
    public native @ByVal @Name("operator()") UMat apply( @ByVal Range rowRange, @ByVal Range colRange );
    public native @ByVal @Name("operator()") UMat apply( @Const @ByRef Rect roi );
    public native @ByVal @Name("operator()") UMat apply( @Const Range ranges );

    /** returns true iff the matrix data is continuous */
    // (i.e. when there are no gaps between successive rows).
    // similar to CV_IS_MAT_CONT(cvmat->type)
    public native @Cast("bool") boolean isContinuous();

    /** returns true if the matrix is a submatrix of another matrix */
    public native @Cast("bool") boolean isSubmatrix();

    /** returns element size in bytes, */
    // similar to CV_ELEM_SIZE(cvmat->type)
    public native @Cast("size_t") long elemSize();
    /** returns the size of element channel in bytes. */
    public native @Cast("size_t") long elemSize1();
    /** returns element type, similar to CV_MAT_TYPE(cvmat->type) */
    public native int type();
    /** returns element type, similar to CV_MAT_DEPTH(cvmat->type) */
    public native int depth();
    /** returns element type, similar to CV_MAT_CN(cvmat->type) */
    public native int channels();
    /** returns step/elemSize1() */
    public native @Cast("size_t") long step1(int i/*=0*/);
    public native @Cast("size_t") long step1();
    /** returns true if matrix data is NULL */
    public native @Cast("bool") boolean empty();
    /** returns the total number of matrix elements */
    public native @Cast("size_t") long total();

    /** returns N if the matrix is 1-channel (N x ptdim) or ptdim-channel (1 x N) or (N x 1); negative number otherwise */
    public native int checkVector(int elemChannels, int depth/*=-1*/, @Cast("bool") boolean requireContinuous/*=true*/);
    public native int checkVector(int elemChannels);

    public native Pointer handle(int accessFlags);
    public native void ndoffset(@Cast("size_t*") SizeTPointer ofs);

    /** enum cv::UMat:: */
    public static final int MAGIC_VAL  =  0x42FF0000, AUTO_STEP = 0, CONTINUOUS_FLAG =  CV_MAT_CONT_FLAG, SUBMATRIX_FLAG =  CV_SUBMAT_FLAG;
    /** enum cv::UMat:: */
    public static final int MAGIC_MASK =  0xFFFF0000, TYPE_MASK =  0x00000FFF, DEPTH_MASK = 7;

    /** includes several bit-fields:
         - the magic signature
         - continuity flag
         - depth
         - number of channels
     */
    public native int flags(); public native UMat flags(int flags);
    /** the matrix dimensionality, >= 2 */
    public native int dims(); public native UMat dims(int dims);
    /** the number of rows and columns or (-1, -1) when the matrix has more than 2 dimensions */
    public native int rows(); public native UMat rows(int rows);
    public native int cols(); public native UMat cols(int cols);

    /** custom allocator */
    public native MatAllocator allocator(); public native UMat allocator(MatAllocator allocator);
    public native @Cast("cv::UMatUsageFlags") int usageFlags(); public native UMat usageFlags(int usageFlags); // usage flags for allocator
    /** and the standard allocator */
    public static native MatAllocator getStdAllocator();

    // black-box container of UMat data
    public native UMatData u(); public native UMat u(UMatData u);

    // offset of the submatrix (or 0)
    public native @Cast("size_t") long offset(); public native UMat offset(long offset);

    public native @ByVal Size size();
    @MemberGetter public native int size(int i);
    @MemberGetter public native long step();
    @MemberGetter public native int step(int i);
}


/////////////////////////// multi-dimensional sparse matrix //////////////////////////

/**
 Sparse matrix class.

 The class represents multi-dimensional sparse numerical arrays. Such a sparse array can store elements
 of any type that cv::Mat is able to store. "Sparse" means that only non-zero elements
 are stored (though, as a result of some operations on a sparse matrix, some of its stored elements
 can actually become 0. It's user responsibility to detect such elements and delete them using cv::SparseMat::erase().
 The non-zero elements are stored in a hash table that grows when it's filled enough,
 so that the search time remains O(1) in average. Elements can be accessed using the following methods:

 <ol>
 <li>Query operations: cv::SparseMat::ptr() and the higher-level cv::SparseMat::ref(),
      cv::SparseMat::value() and cv::SparseMat::find, for example:
 \code
 const int dims = 5;
 int size[] = {10, 10, 10, 10, 10};
 SparseMat sparse_mat(dims, size, CV_32F);
 for(int i = 0; i < 1000; i++)
 {
     int idx[dims];
     for(int k = 0; k < dims; k++)
        idx[k] = rand()%sparse_mat.size(k);
     sparse_mat.ref<float>(idx) += 1.f;
 }
 \endcode

 <li>Sparse matrix iterators. Like cv::Mat iterators and unlike cv::Mat iterators, the sparse matrix iterators are STL-style,
 that is, the iteration is done as following:
 \code
 // prints elements of a sparse floating-point matrix and the sum of elements.
 SparseMatConstIterator_<float>
        it = sparse_mat.begin<float>(),
        it_end = sparse_mat.end<float>();
 double s = 0;
 int dims = sparse_mat.dims();
 for(; it != it_end; ++it)
 {
     // print element indices and the element value
     const Node* n = it.node();
     printf("(")
     for(int i = 0; i < dims; i++)
        printf("%3d%c", n->idx[i], i < dims-1 ? ',' : ')');
     printf(": %f\n", *it);
     s += *it;
 }
 printf("Element sum is %g\n", s);
 \endcode
 If you run this loop, you will notice that elements are enumerated
 in no any logical order (lexicographical etc.),
 they come in the same order as they stored in the hash table, i.e. semi-randomly.

 You may collect pointers to the nodes and sort them to get the proper ordering.
 Note, however, that pointers to the nodes may become invalid when you add more
 elements to the matrix; this is because of possible buffer reallocation.

 <li>A combination of the above 2 methods when you need to process 2 or more sparse
 matrices simultaneously, e.g. this is how you can compute unnormalized
 cross-correlation of the 2 floating-point sparse matrices:
 \code
 double crossCorr(const SparseMat& a, const SparseMat& b)
 {
     const SparseMat *_a = &a, *_b = &b;
     // if b contains less elements than a,
     // it's faster to iterate through b
     if(_a->nzcount() > _b->nzcount())
        std::swap(_a, _b);
     SparseMatConstIterator_<float> it = _a->begin<float>(),
                                    it_end = _a->end<float>();
     double ccorr = 0;
     for(; it != it_end; ++it)
     {
         // take the next element from the first matrix
         float avalue = *it;
         const Node* anode = it.node();
         // and try to find element with the same index in the second matrix.
         // since the hash value depends only on the element index,
         // we reuse hashvalue stored in the node
         float bvalue = _b->value<float>(anode->idx,&anode->hashval);
         ccorr += avalue*bvalue;
     }
     return ccorr;
 }
 \endcode
 </ol>
*/
@Namespace("cv") @NoOffset public static class SparseMat extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public SparseMat(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public SparseMat(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public SparseMat position(int position) {
        return (SparseMat)super.position(position);
    }


    /** enum cv::SparseMat:: */
    public static final int MAGIC_VAL= 0x42FD0000, MAX_DIM= 32, HASH_SCALE= 0x5bd1e995, HASH_BIT= 0x80000000;

    /** the sparse matrix header */
    @NoOffset public static class Hdr extends Pointer {
        static { Loader.load(); }
        /** Empty constructor. */
        public Hdr() { }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public Hdr(Pointer p) { super(p); }
    
        public Hdr(int _dims, @Const IntPointer _sizes, int _type) { allocate(_dims, _sizes, _type); }
        private native void allocate(int _dims, @Const IntPointer _sizes, int _type);
        public Hdr(int _dims, @Const IntBuffer _sizes, int _type) { allocate(_dims, _sizes, _type); }
        private native void allocate(int _dims, @Const IntBuffer _sizes, int _type);
        public Hdr(int _dims, @Const int[] _sizes, int _type) { allocate(_dims, _sizes, _type); }
        private native void allocate(int _dims, @Const int[] _sizes, int _type);
        public native void clear();
        public native int refcount(); public native Hdr refcount(int refcount);
        public native int dims(); public native Hdr dims(int dims);
        public native int valueOffset(); public native Hdr valueOffset(int valueOffset);
        public native @Cast("size_t") long nodeSize(); public native Hdr nodeSize(long nodeSize);
        public native @Cast("size_t") long nodeCount(); public native Hdr nodeCount(long nodeCount);
        public native @Cast("size_t") long freeList(); public native Hdr freeList(long freeList);
        public native @Cast("uchar*") @StdVector BytePointer pool(); public native Hdr pool(BytePointer pool);
        public native @Cast("size_t*") @StdVector SizeTPointer hashtab(); public native Hdr hashtab(SizeTPointer hashtab);
        public native int size(int i); public native Hdr size(int i, int size);
        @MemberGetter public native IntPointer size();
    }

    /** sparse matrix node - element of a hash table */
    public static class Node extends Pointer {
        static { Loader.load(); }
        /** Default native constructor. */
        public Node() { allocate(); }
        /** Native array allocator. Access with {@link Pointer#position(int)}. */
        public Node(int size) { allocateArray(size); }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public Node(Pointer p) { super(p); }
        private native void allocate();
        private native void allocateArray(int size);
        @Override public Node position(int position) {
            return (Node)super.position(position);
        }
    
        /** hash value */
        public native @Cast("size_t") long hashval(); public native Node hashval(long hashval);
        /** index of the next node in the same hash table entry */
        public native @Cast("size_t") long next(); public native Node next(long next);
        /** index of the matrix element */
        public native int idx(int i); public native Node idx(int i, int idx);
        @MemberGetter public native IntPointer idx();
    }

    /** default constructor */
    public SparseMat() { allocate(); }
    private native void allocate();
    /** creates matrix of the specified size and type */
    public SparseMat(int dims, @Const IntPointer _sizes, int _type) { allocate(dims, _sizes, _type); }
    private native void allocate(int dims, @Const IntPointer _sizes, int _type);
    public SparseMat(int dims, @Const IntBuffer _sizes, int _type) { allocate(dims, _sizes, _type); }
    private native void allocate(int dims, @Const IntBuffer _sizes, int _type);
    public SparseMat(int dims, @Const int[] _sizes, int _type) { allocate(dims, _sizes, _type); }
    private native void allocate(int dims, @Const int[] _sizes, int _type);
    /** copy constructor */
    public SparseMat(@Const @ByRef SparseMat m) { allocate(m); }
    private native void allocate(@Const @ByRef SparseMat m);
    /** converts dense 2d matrix to the sparse form
    /**
     \param m the input matrix
    */
    public SparseMat(@Const @ByRef Mat m) { allocate(m); }
    private native void allocate(@Const @ByRef Mat m);
    /** converts old-style sparse matrix to the new-style. All the data is copied */
    //SparseMat(const CvSparseMat* m);
    /** the destructor */

    /** assignment operator. This is O(1) operation, i.e. no data is copied */
    public native @ByRef @Name("operator=") SparseMat put(@Const @ByRef SparseMat m);
    /** equivalent to the corresponding constructor */
    public native @ByRef @Name("operator=") SparseMat put(@Const @ByRef Mat m);

    /** creates full copy of the matrix */
    public native @ByVal SparseMat clone();

    /** copies all the data to the destination matrix. All the previous content of m is erased */
    public native void copyTo( @ByRef SparseMat m );
    /** converts sparse matrix to dense matrix. */
    public native void copyTo( @ByRef Mat m );
    /** multiplies all the matrix elements by the specified scale factor alpha and converts the results to the specified data type */
    public native void convertTo( @ByRef SparseMat m, int rtype, double alpha/*=1*/ );
    public native void convertTo( @ByRef SparseMat m, int rtype );
    /** converts sparse matrix to dense n-dim matrix with optional type conversion and scaling.
    /**
        @param [out] m - output matrix; if it does not have a proper size or type before the operation,
            it is reallocated
        @param [in] rtype – desired output matrix type or, rather, the depth since the number of channels
            are the same as the input has; if rtype is negative, the output matrix will have the
            same type as the input.
        @param [in] alpha – optional scale factor
        @param [in] beta – optional delta added to the scaled values
    */
    public native void convertTo( @ByRef Mat m, int rtype, double alpha/*=1*/, double beta/*=0*/ );
    public native void convertTo( @ByRef Mat m, int rtype );

    // not used now
    public native void assignTo( @ByRef SparseMat m, int type/*=-1*/ );
    public native void assignTo( @ByRef SparseMat m );

    /** reallocates sparse matrix.
    /**
        If the matrix already had the proper size and type,
        it is simply cleared with clear(), otherwise,
        the old matrix is released (using release()) and the new one is allocated.
    */
    public native void create(int dims, @Const IntPointer _sizes, int _type);
    public native void create(int dims, @Const IntBuffer _sizes, int _type);
    public native void create(int dims, @Const int[] _sizes, int _type);
    /** sets all the sparse matrix elements to 0, which means clearing the hash table. */
    public native void clear();
    /** manually increments the reference counter to the header. */
    public native void addref();
    // decrements the header reference counter. When the counter reaches 0, the header and all the underlying data are deallocated.
    public native void release();

    /** converts sparse matrix to the old-style representation; all the elements are copied. */
    //operator CvSparseMat*() const;
    /** returns the size of each element in bytes (not including the overhead - the space occupied by SparseMat::Node elements) */
    public native @Cast("size_t") long elemSize();
    /** returns elemSize()/channels() */
    public native @Cast("size_t") long elemSize1();

    /** returns type of sparse matrix elements */
    public native int type();
    /** returns the depth of sparse matrix elements */
    public native int depth();
    /** returns the number of channels */
    public native int channels();

    /** returns the array of sizes, or NULL if the matrix is not allocated */
    public native @Const IntPointer size();
    /** returns the size of i-th matrix dimension (or 0) */
    public native int size(int i);
    /** returns the matrix dimensionality */
    public native int dims();
    /** returns the number of non-zero elements (=the number of hash table nodes) */
    public native @Cast("size_t") long nzcount();

    /** computes the element hash value (1D case) */
    public native @Cast("size_t") long hash(int i0);
    /** computes the element hash value (2D case) */
    public native @Cast("size_t") long hash(int i0, int i1);
    /** computes the element hash value (3D case) */
    public native @Cast("size_t") long hash(int i0, int i1, int i2);
    /** computes the element hash value (nD case) */
    public native @Cast("size_t") long hash(@Const IntPointer idx);
    public native @Cast("size_t") long hash(@Const IntBuffer idx);
    public native @Cast("size_t") long hash(@Const int[] idx);

    //@{
    /**
     specialized variants for 1D, 2D, 3D cases and the generic_type one for n-D case.

     return pointer to the matrix element.
     <ul>
      <li>if the element is there (it's non-zero), the pointer to it is returned
      <li>if it's not there and createMissing=false, NULL pointer is returned
      <li>if it's not there and createMissing=true, then the new element
        is created and initialized with 0. Pointer to it is returned
      <li>if the optional hashval pointer is not NULL, the element hash value is
      not computed, but *hashval is taken instead.
     </ul>
    */
    /** returns pointer to the specified element (1D case) */
    public native @Cast("uchar*") BytePointer ptr(int i0, @Cast("bool") boolean createMissing, @Cast("size_t*") SizeTPointer hashval/*=0*/);
    public native @Cast("uchar*") BytePointer ptr(int i0, @Cast("bool") boolean createMissing);
    /** returns pointer to the specified element (2D case) */
    public native @Cast("uchar*") BytePointer ptr(int i0, int i1, @Cast("bool") boolean createMissing, @Cast("size_t*") SizeTPointer hashval/*=0*/);
    public native @Cast("uchar*") BytePointer ptr(int i0, int i1, @Cast("bool") boolean createMissing);
    /** returns pointer to the specified element (3D case) */
    public native @Cast("uchar*") BytePointer ptr(int i0, int i1, int i2, @Cast("bool") boolean createMissing, @Cast("size_t*") SizeTPointer hashval/*=0*/);
    public native @Cast("uchar*") BytePointer ptr(int i0, int i1, int i2, @Cast("bool") boolean createMissing);
    /** returns pointer to the specified element (nD case) */
    public native @Cast("uchar*") BytePointer ptr(@Const IntPointer idx, @Cast("bool") boolean createMissing, @Cast("size_t*") SizeTPointer hashval/*=0*/);
    public native @Cast("uchar*") BytePointer ptr(@Const IntPointer idx, @Cast("bool") boolean createMissing);
    public native @Cast("uchar*") ByteBuffer ptr(@Const IntBuffer idx, @Cast("bool") boolean createMissing, @Cast("size_t*") SizeTPointer hashval/*=0*/);
    public native @Cast("uchar*") ByteBuffer ptr(@Const IntBuffer idx, @Cast("bool") boolean createMissing);
    public native @Cast("uchar*") byte[] ptr(@Const int[] idx, @Cast("bool") boolean createMissing, @Cast("size_t*") SizeTPointer hashval/*=0*/);
    public native @Cast("uchar*") byte[] ptr(@Const int[] idx, @Cast("bool") boolean createMissing);
    //@}

    //@{
    /**
     return read-write reference to the specified sparse matrix element.

     ref<_Tp>(i0,...[,hashval]) is equivalent to *(_Tp*)ptr(i0,...,true[,hashval]).
     The methods always return a valid reference.
     If the element did not exist, it is created and initialiazed with 0.
    */
    /** returns reference to the specified element (1D case) */
    /** returns reference to the specified element (2D case) */
    /** returns reference to the specified element (3D case) */
    /** returns reference to the specified element (nD case) */
    //@}

    //@{
    /**
     return value of the specified sparse matrix element.

     value<_Tp>(i0,...[,hashval]) is equivalent

     \code
     { const _Tp* p = find<_Tp>(i0,...[,hashval]); return p ? *p : _Tp(); }
     \endcode

     That is, if the element did not exist, the methods return 0.
     */
    /** returns value of the specified element (1D case) */
    /** returns value of the specified element (2D case) */
    /** returns value of the specified element (3D case) */
    /** returns value of the specified element (nD case) */
    //@}

    //@{
    /**
     Return pointer to the specified sparse matrix element if it exists

     find<_Tp>(i0,...[,hashval]) is equivalent to (_const Tp*)ptr(i0,...false[,hashval]).

     If the specified element does not exist, the methods return NULL.
    */
    /** returns pointer to the specified element (1D case) */
    /** returns pointer to the specified element (2D case) */
    /** returns pointer to the specified element (3D case) */
    /** returns pointer to the specified element (nD case) */

    /** erases the specified element (2D case) */
    public native void erase(int i0, int i1, @Cast("size_t*") SizeTPointer hashval/*=0*/);
    public native void erase(int i0, int i1);
    /** erases the specified element (3D case) */
    public native void erase(int i0, int i1, int i2, @Cast("size_t*") SizeTPointer hashval/*=0*/);
    public native void erase(int i0, int i1, int i2);
    /** erases the specified element (nD case) */
    public native void erase(@Const IntPointer idx, @Cast("size_t*") SizeTPointer hashval/*=0*/);
    public native void erase(@Const IntPointer idx);
    public native void erase(@Const IntBuffer idx, @Cast("size_t*") SizeTPointer hashval/*=0*/);
    public native void erase(@Const IntBuffer idx);
    public native void erase(@Const int[] idx, @Cast("size_t*") SizeTPointer hashval/*=0*/);
    public native void erase(@Const int[] idx);

    //@{
    /**
       return the sparse matrix iterator pointing to the first sparse matrix element
    */
    /** returns the sparse matrix iterator at the matrix beginning */
    public native @ByVal SparseMatIterator begin();
    /** returns the sparse matrix iterator at the matrix beginning */
    /** returns the read-only sparse matrix iterator at the matrix beginning */
    /** returns the read-only sparse matrix iterator at the matrix beginning */
    //@}
    /**
       return the sparse matrix iterator pointing to the element following the last sparse matrix element
    */
    /** returns the sparse matrix iterator at the matrix end */
    public native @ByVal SparseMatIterator end();
    /** returns the read-only sparse matrix iterator at the matrix end */
    /** returns the typed sparse matrix iterator at the matrix end */
    /** returns the typed read-only sparse matrix iterator at the matrix end */

    /** returns the value stored in the sparse martix node */
    /** returns the value stored in the sparse martix node */

    ////////////// some internal-use methods ///////////////
    public native Node node(@Cast("size_t") long nidx);

    public native @Cast("uchar*") BytePointer newNode(@Const IntPointer idx, @Cast("size_t") long hashval);
    public native @Cast("uchar*") ByteBuffer newNode(@Const IntBuffer idx, @Cast("size_t") long hashval);
    public native @Cast("uchar*") byte[] newNode(@Const int[] idx, @Cast("size_t") long hashval);
    public native void removeNode(@Cast("size_t") long hidx, @Cast("size_t") long nidx, @Cast("size_t") long previdx);
    public native void resizeHashTab(@Cast("size_t") long newsize);

    public native int flags(); public native SparseMat flags(int flags);
    public native Hdr hdr(); public native SparseMat hdr(Hdr hdr);
}



///////////////////////////////// SparseMat_<_Tp> ////////////////////////////////////

/**
 The Template Sparse Matrix class derived from cv::SparseMat

 The class provides slightly more convenient operations for accessing elements.

 \code
 SparseMat m;
 ...
 SparseMat_<int> m_ = (SparseMat_<int>&)m;
 m_.ref(1)++; // equivalent to m.ref<int>(1)++;
 m_.ref(2) += m_(3); // equivalent to m.ref<int>(2) += m.value<int>(3);
 \endcode
*/



////////////////////////////////// MatConstIterator //////////////////////////////////

@Namespace("cv") @NoOffset public static class MatConstIterator extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public MatConstIterator(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public MatConstIterator(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public MatConstIterator position(int position) {
        return (MatConstIterator)super.position(position);
    }


// #ifndef OPENCV_NOSTL
// #endif

    /** default constructor */
    public MatConstIterator() { allocate(); }
    private native void allocate();
    /** constructor that sets the iterator to the beginning of the matrix */
    public MatConstIterator(@Const Mat _m) { allocate(_m); }
    private native void allocate(@Const Mat _m);
    /** constructor that sets the iterator to the specified element of the matrix */
    public MatConstIterator(@Const Mat _m, int _row, int _col/*=0*/) { allocate(_m, _row, _col); }
    private native void allocate(@Const Mat _m, int _row, int _col/*=0*/);
    public MatConstIterator(@Const Mat _m, int _row) { allocate(_m, _row); }
    private native void allocate(@Const Mat _m, int _row);
    /** constructor that sets the iterator to the specified element of the matrix */
    public MatConstIterator(@Const Mat _m, @ByVal Point _pt) { allocate(_m, _pt); }
    private native void allocate(@Const Mat _m, @ByVal Point _pt);
    /** constructor that sets the iterator to the specified element of the matrix */
    
    /** copy constructor */
    public MatConstIterator(@Const @ByRef MatConstIterator it) { allocate(it); }
    private native void allocate(@Const @ByRef MatConstIterator it);

    /** copy operator */
    public native @ByRef @Name("operator=") MatConstIterator put(@Const @ByRef MatConstIterator it);
    /** returns the current matrix element */
    public native @Cast("const uchar*") @Name("operator*") BytePointer multiply();
    /** returns the i-th matrix element, relative to the current */
    public native @Cast("const uchar*") @Name("operator[]") BytePointer get(@Cast("ptrdiff_t") long i);

    /** shifts the iterator forward by the specified number of elements */
    public native @ByRef @Name("operator+=") MatConstIterator addPut(@Cast("ptrdiff_t") long ofs);
    /** shifts the iterator backward by the specified number of elements */
    public native @ByRef @Name("operator-=") MatConstIterator subtractPut(@Cast("ptrdiff_t") long ofs);
    /** decrements the iterator */
    public native @ByRef @Name("operator--") MatConstIterator decrement();
    /** decrements the iterator */
    public native @ByVal @Name("operator--") MatConstIterator decrement(int arg0);
    /** increments the iterator */
    public native @ByRef @Name("operator++") MatConstIterator increment();
    /** increments the iterator */
    public native @ByVal @Name("operator++") MatConstIterator increment(int arg0);
    /** returns the current iterator position */
    public native @ByVal Point pos();
    /** returns the current iterator position */
    public native void pos(IntPointer _idx);
    public native void pos(IntBuffer _idx);
    public native void pos(int[] _idx);

    public native @Cast("ptrdiff_t") long lpos();
    public native void seek(@Cast("ptrdiff_t") long ofs, @Cast("bool") boolean relative/*=false*/);
    public native void seek(@Cast("ptrdiff_t") long ofs);
    public native void seek(@Const IntPointer _idx, @Cast("bool") boolean relative/*=false*/);
    public native void seek(@Const IntPointer _idx);
    public native void seek(@Const IntBuffer _idx, @Cast("bool") boolean relative/*=false*/);
    public native void seek(@Const IntBuffer _idx);
    public native void seek(@Const int[] _idx, @Cast("bool") boolean relative/*=false*/);
    public native void seek(@Const int[] _idx);

    @MemberGetter public native @Const Mat m();
    public native @Cast("size_t") long elemSize(); public native MatConstIterator elemSize(long elemSize);
    @MemberGetter public native @Cast("const uchar*") BytePointer ptr();
    @MemberGetter public native @Cast("const uchar*") BytePointer sliceStart();
    @MemberGetter public native @Cast("const uchar*") BytePointer sliceEnd();
}



////////////////////////////////// MatConstIterator_ /////////////////////////////////

/**
 Matrix read-only iterator
 */



//////////////////////////////////// MatIterator_ ////////////////////////////////////

/**
 Matrix read-write iterator
*/



/////////////////////////////// SparseMatConstIterator ///////////////////////////////

/**
 Read-Only Sparse Matrix Iterator.
 Here is how to use the iterator to compute the sum of floating-point sparse matrix elements:

 \code
 SparseMatConstIterator it = m.begin(), it_end = m.end();
 double s = 0;
 CV_Assert( m.type() == CV_32F );
 for( ; it != it_end; ++it )
    s += it.value<float>();
 \endcode
*/
@Namespace("cv") @NoOffset public static class SparseMatConstIterator extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public SparseMatConstIterator(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public SparseMatConstIterator(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public SparseMatConstIterator position(int position) {
        return (SparseMatConstIterator)super.position(position);
    }

    /** the default constructor */
    public SparseMatConstIterator() { allocate(); }
    private native void allocate();
    /** the full constructor setting the iterator to the first sparse matrix element */
    public SparseMatConstIterator(@Const SparseMat _m) { allocate(_m); }
    private native void allocate(@Const SparseMat _m);
    /** the copy constructor */
    public SparseMatConstIterator(@Const @ByRef SparseMatConstIterator it) { allocate(it); }
    private native void allocate(@Const @ByRef SparseMatConstIterator it);

    /** the assignment operator */
    public native @ByRef @Name("operator=") SparseMatConstIterator put(@Const @ByRef SparseMatConstIterator it);

    /** template method returning the current matrix element */
    /** returns the current node of the sparse matrix. it.node->idx is the current element index */
    public native @Const SparseMat.Node node();

    /** moves iterator to the previous element */
    
    /** moves iterator to the previous element */
    
    /** moves iterator to the next element */
    public native @ByRef @Name("operator++") SparseMatConstIterator increment();
    /** moves iterator to the next element */
    public native @ByVal @Name("operator++") SparseMatConstIterator increment(int arg0);

    /** moves iterator to the element after the last element */
    public native void seekEnd();

    @MemberGetter public native @Const SparseMat m();
    public native @Cast("size_t") long hashidx(); public native SparseMatConstIterator hashidx(long hashidx);
    public native @Cast("uchar*") BytePointer ptr(); public native SparseMatConstIterator ptr(BytePointer ptr);
}



////////////////////////////////// SparseMatIterator /////////////////////////////////

/**
 Read-write Sparse Matrix Iterator

 The class is similar to cv::SparseMatConstIterator,
 but can be used for in-place modification of the matrix elements.
*/
@Namespace("cv") public static class SparseMatIterator extends SparseMatConstIterator {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public SparseMatIterator(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public SparseMatIterator(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public SparseMatIterator position(int position) {
        return (SparseMatIterator)super.position(position);
    }

    /** the default constructor */
    public SparseMatIterator() { allocate(); }
    private native void allocate();
    /** the full constructor setting the iterator to the first sparse matrix element */
    public SparseMatIterator(SparseMat _m) { allocate(_m); }
    private native void allocate(SparseMat _m);
    /** the full constructor setting the iterator to the specified sparse matrix element */
    
    /** the copy constructor */
    public SparseMatIterator(@Const @ByRef SparseMatIterator it) { allocate(it); }
    private native void allocate(@Const @ByRef SparseMatIterator it);

    /** the assignment operator */
    public native @ByRef @Name("operator=") SparseMatIterator put(@Const @ByRef SparseMatIterator it);
    /** returns read-write reference to the current sparse matrix element */
    /** returns pointer to the current sparse matrix node. it.node->idx is the index of the current element (do not modify it!) */
    public native SparseMat.Node node();

    /** moves iterator to the next element */
    public native @ByRef @Name("operator++") SparseMatIterator increment();
    /** moves iterator to the next element */
    public native @ByVal @Name("operator++") SparseMatIterator increment(int arg0);
}



/////////////////////////////// SparseMatConstIterator_ //////////////////////////////

/**
 Template Read-Only Sparse Matrix Iterator Class.

 This is the derived from SparseMatConstIterator class that
 introduces more convenient operator *() for accessing the current element.
*/



///////////////////////////////// SparseMatIterator_ /////////////////////////////////

/**
 Template Read-Write Sparse Matrix Iterator Class.

 This is the derived from cv::SparseMatConstIterator_ class that
 introduces more convenient operator *() for accessing the current element.
*/



/////////////////////////////////// NAryMatIterator //////////////////////////////////

/**
 n-Dimensional Dense Matrix Iterator Class.

 The class cv::NAryMatIterator is used for iterating over one or more n-dimensional dense arrays (cv::Mat's).

 The iterator is completely different from cv::Mat_ and cv::SparseMat_ iterators.
 It iterates through the slices (or planes), not the elements, where "slice" is a continuous part of the arrays.

 Here is the example on how the iterator can be used to normalize 3D histogram:

 \code
 void normalizeColorHist(Mat& hist)
 {
 #if 1
     // intialize iterator (the style is different from STL).
     // after initialization the iterator will contain
     // the number of slices or planes
     // the iterator will go through
     Mat* arrays[] = { &hist, 0 };
     Mat planes[1];
     NAryMatIterator it(arrays, planes);
     double s = 0;
     // iterate through the matrix. on each iteration
     // it.planes[i] (of type Mat) will be set to the current plane of
     // i-th n-dim matrix passed to the iterator constructor.
     for(int p = 0; p < it.nplanes; p++, ++it)
        s += sum(it.planes[0])[0];
     it = NAryMatIterator(hist);
     s = 1./s;
     for(int p = 0; p < it.nplanes; p++, ++it)
        it.planes[0] *= s;
 #elif 1
     // this is a shorter implementation of the above
     // using built-in operations on Mat
     double s = sum(hist)[0];
     hist.convertTo(hist, hist.type(), 1./s, 0);
 #else
     // and this is even shorter one
     // (assuming that the histogram elements are non-negative)
     normalize(hist, hist, 1, 0, NORM_L1);
 #endif
 }
 \endcode

 You can iterate through several matrices simultaneously as long as they have the same geometry
 (dimensionality and all the dimension sizes are the same), which is useful for binary
 and n-ary operations on such matrices. Just pass those matrices to cv::MatNDIterator.
 Then, during the iteration it.planes[0], it.planes[1], ... will
 be the slices of the corresponding matrices
*/
@Namespace("cv") @NoOffset public static class NAryMatIterator extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public NAryMatIterator(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public NAryMatIterator(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public NAryMatIterator position(int position) {
        return (NAryMatIterator)super.position(position);
    }

    /** the default constructor */
    public NAryMatIterator() { allocate(); }
    private native void allocate();
    /** the full constructor taking arbitrary number of n-dim matrices */
    public NAryMatIterator(@Cast("const cv::Mat**") PointerPointer arrays, @Cast("uchar**") PointerPointer ptrs, int narrays/*=-1*/) { allocate(arrays, ptrs, narrays); }
    private native void allocate(@Cast("const cv::Mat**") PointerPointer arrays, @Cast("uchar**") PointerPointer ptrs, int narrays/*=-1*/);
    public NAryMatIterator(@Const @ByPtrPtr Mat arrays, @Cast("uchar**") @ByPtrPtr BytePointer ptrs) { allocate(arrays, ptrs); }
    private native void allocate(@Const @ByPtrPtr Mat arrays, @Cast("uchar**") @ByPtrPtr BytePointer ptrs);
    public NAryMatIterator(@Const @ByPtrPtr Mat arrays, @Cast("uchar**") @ByPtrPtr BytePointer ptrs, int narrays/*=-1*/) { allocate(arrays, ptrs, narrays); }
    private native void allocate(@Const @ByPtrPtr Mat arrays, @Cast("uchar**") @ByPtrPtr BytePointer ptrs, int narrays/*=-1*/);
    public NAryMatIterator(@Const @ByPtrPtr Mat arrays, @Cast("uchar**") @ByPtrPtr ByteBuffer ptrs, int narrays/*=-1*/) { allocate(arrays, ptrs, narrays); }
    private native void allocate(@Const @ByPtrPtr Mat arrays, @Cast("uchar**") @ByPtrPtr ByteBuffer ptrs, int narrays/*=-1*/);
    public NAryMatIterator(@Const @ByPtrPtr Mat arrays, @Cast("uchar**") @ByPtrPtr ByteBuffer ptrs) { allocate(arrays, ptrs); }
    private native void allocate(@Const @ByPtrPtr Mat arrays, @Cast("uchar**") @ByPtrPtr ByteBuffer ptrs);
    public NAryMatIterator(@Const @ByPtrPtr Mat arrays, @Cast("uchar**") @ByPtrPtr byte[] ptrs, int narrays/*=-1*/) { allocate(arrays, ptrs, narrays); }
    private native void allocate(@Const @ByPtrPtr Mat arrays, @Cast("uchar**") @ByPtrPtr byte[] ptrs, int narrays/*=-1*/);
    public NAryMatIterator(@Const @ByPtrPtr Mat arrays, @Cast("uchar**") @ByPtrPtr byte[] ptrs) { allocate(arrays, ptrs); }
    private native void allocate(@Const @ByPtrPtr Mat arrays, @Cast("uchar**") @ByPtrPtr byte[] ptrs);
    /** the full constructor taking arbitrary number of n-dim matrices */
    public NAryMatIterator(@Cast("const cv::Mat**") PointerPointer arrays, Mat planes, int narrays/*=-1*/) { allocate(arrays, planes, narrays); }
    private native void allocate(@Cast("const cv::Mat**") PointerPointer arrays, Mat planes, int narrays/*=-1*/);
    public NAryMatIterator(@Const @ByPtrPtr Mat arrays, Mat planes) { allocate(arrays, planes); }
    private native void allocate(@Const @ByPtrPtr Mat arrays, Mat planes);
    public NAryMatIterator(@Const @ByPtrPtr Mat arrays, Mat planes, int narrays/*=-1*/) { allocate(arrays, planes, narrays); }
    private native void allocate(@Const @ByPtrPtr Mat arrays, Mat planes, int narrays/*=-1*/);
    /** the separate iterator initialization method */
    public native void init(@Cast("const cv::Mat**") PointerPointer arrays, Mat planes, @Cast("uchar**") PointerPointer ptrs, int narrays/*=-1*/);
    public native void init(@Const @ByPtrPtr Mat arrays, Mat planes, @Cast("uchar**") @ByPtrPtr BytePointer ptrs);
    public native void init(@Const @ByPtrPtr Mat arrays, Mat planes, @Cast("uchar**") @ByPtrPtr BytePointer ptrs, int narrays/*=-1*/);
    public native void init(@Const @ByPtrPtr Mat arrays, Mat planes, @Cast("uchar**") @ByPtrPtr ByteBuffer ptrs, int narrays/*=-1*/);
    public native void init(@Const @ByPtrPtr Mat arrays, Mat planes, @Cast("uchar**") @ByPtrPtr ByteBuffer ptrs);
    public native void init(@Const @ByPtrPtr Mat arrays, Mat planes, @Cast("uchar**") @ByPtrPtr byte[] ptrs, int narrays/*=-1*/);
    public native void init(@Const @ByPtrPtr Mat arrays, Mat planes, @Cast("uchar**") @ByPtrPtr byte[] ptrs);

    /** proceeds to the next plane of every iterated matrix */
    public native @ByRef @Name("operator++") NAryMatIterator increment();
    /** proceeds to the next plane of every iterated matrix (postfix increment operator) */
    public native @ByVal @Name("operator++") NAryMatIterator increment(int arg0);

    /** the iterated arrays */
    @MemberGetter public native @Const Mat arrays(int i);
    @MemberGetter public native @Cast("const cv::Mat**") PointerPointer arrays();
    /** the current planes */
    public native Mat planes(); public native NAryMatIterator planes(Mat planes);
    /** data pointers */
    public native @Cast("uchar*") BytePointer ptrs(int i); public native NAryMatIterator ptrs(int i, BytePointer ptrs);
    @MemberGetter public native @Cast("uchar**") PointerPointer ptrs();
    /** the number of arrays */
    public native int narrays(); public native NAryMatIterator narrays(int narrays);
    /** the number of hyper-planes that the iterator steps through */
    public native @Cast("size_t") long nplanes(); public native NAryMatIterator nplanes(long nplanes);
    /** the size of each segment (in elements) */
    public native @Cast("size_t") long size(); public native NAryMatIterator size(long size);
}



///////////////////////////////// Matrix Expressions /////////////////////////////////

@Namespace("cv") public static class MatOp extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public MatOp() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public MatOp(Pointer p) { super(p); }


    public native @Cast("bool") boolean elementWise(@Const @ByRef MatExpr expr);
    public native void assign(@Const @ByRef MatExpr expr, @ByRef Mat m, int type/*=-1*/);
    public native void assign(@Const @ByRef MatExpr expr, @ByRef Mat m);
    public native void roi(@Const @ByRef MatExpr expr, @Const @ByRef Range rowRange,
                         @Const @ByRef Range colRange, @ByRef MatExpr res);
    public native void diag(@Const @ByRef MatExpr expr, int d, @ByRef MatExpr res);
    public native void augAssignAdd(@Const @ByRef MatExpr expr, @ByRef Mat m);
    public native void augAssignSubtract(@Const @ByRef MatExpr expr, @ByRef Mat m);
    public native void augAssignMultiply(@Const @ByRef MatExpr expr, @ByRef Mat m);
    public native void augAssignDivide(@Const @ByRef MatExpr expr, @ByRef Mat m);
    public native void augAssignAnd(@Const @ByRef MatExpr expr, @ByRef Mat m);
    public native void augAssignOr(@Const @ByRef MatExpr expr, @ByRef Mat m);
    public native void augAssignXor(@Const @ByRef MatExpr expr, @ByRef Mat m);

    public native void add(@Const @ByRef MatExpr expr1, @Const @ByRef MatExpr expr2, @ByRef MatExpr res);
    public native void add(@Const @ByRef MatExpr expr1, @Const @ByRef Scalar s, @ByRef MatExpr res);

    public native void subtract(@Const @ByRef MatExpr expr1, @Const @ByRef MatExpr expr2, @ByRef MatExpr res);
    public native void subtract(@Const @ByRef Scalar s, @Const @ByRef MatExpr expr, @ByRef MatExpr res);

    public native void multiply(@Const @ByRef MatExpr expr1, @Const @ByRef MatExpr expr2, @ByRef MatExpr res, double scale/*=1*/);
    public native void multiply(@Const @ByRef MatExpr expr1, @Const @ByRef MatExpr expr2, @ByRef MatExpr res);
    public native void multiply(@Const @ByRef MatExpr expr1, double s, @ByRef MatExpr res);

    public native void divide(@Const @ByRef MatExpr expr1, @Const @ByRef MatExpr expr2, @ByRef MatExpr res, double scale/*=1*/);
    public native void divide(@Const @ByRef MatExpr expr1, @Const @ByRef MatExpr expr2, @ByRef MatExpr res);
    public native void divide(double s, @Const @ByRef MatExpr expr, @ByRef MatExpr res);

    public native void abs(@Const @ByRef MatExpr expr, @ByRef MatExpr res);

    public native void transpose(@Const @ByRef MatExpr expr, @ByRef MatExpr res);
    public native void matmul(@Const @ByRef MatExpr expr1, @Const @ByRef MatExpr expr2, @ByRef MatExpr res);
    public native void invert(@Const @ByRef MatExpr expr, int method, @ByRef MatExpr res);

    public native @ByVal Size size(@Const @ByRef MatExpr expr);
    public native int type(@Const @ByRef MatExpr expr);
}


@Namespace("cv") @NoOffset public static class MatExpr extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public MatExpr(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public MatExpr(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public MatExpr position(int position) {
        return (MatExpr)super.position(position);
    }

    public MatExpr() { allocate(); }
    private native void allocate();
    public MatExpr(@Const @ByRef Mat m) { allocate(m); }
    private native void allocate(@Const @ByRef Mat m);

    public MatExpr(@Const MatOp _op, int _flags, @Const @ByRef Mat _a/*=Mat()*/, @Const @ByRef Mat _b/*=Mat()*/,
                @Const @ByRef Mat _c/*=Mat()*/, double _alpha/*=1*/, double _beta/*=1*/, @Const @ByRef Scalar _s/*=Scalar()*/) { allocate(_op, _flags, _a, _b, _c, _alpha, _beta, _s); }
    private native void allocate(@Const MatOp _op, int _flags, @Const @ByRef Mat _a/*=Mat()*/, @Const @ByRef Mat _b/*=Mat()*/,
                @Const @ByRef Mat _c/*=Mat()*/, double _alpha/*=1*/, double _beta/*=1*/, @Const @ByRef Scalar _s/*=Scalar()*/);
    public MatExpr(@Const MatOp _op, int _flags) { allocate(_op, _flags); }
    private native void allocate(@Const MatOp _op, int _flags);

    public native @ByVal @Name("operator cv::Mat") Mat asMat();

    public native @ByVal Size size();
    public native int type();

    public native @ByVal MatExpr row(int y);
    public native @ByVal MatExpr col(int x);
    public native @ByVal MatExpr diag(int d/*=0*/);
    public native @ByVal MatExpr diag();
    public native @ByVal @Name("operator()") MatExpr apply( @Const @ByRef Range rowRange, @Const @ByRef Range colRange );
    public native @ByVal @Name("operator()") MatExpr apply( @Const @ByRef Rect roi );

    public native @ByVal MatExpr t();
    public native @ByVal MatExpr inv(int method/*=DECOMP_LU*/);
    public native @ByVal MatExpr inv();
    public native @ByVal MatExpr mul(@Const @ByRef MatExpr e, double scale/*=1*/);
    public native @ByVal MatExpr mul(@Const @ByRef MatExpr e);
    public native @ByVal MatExpr mul(@Const @ByRef Mat m, double scale/*=1*/);
    public native @ByVal MatExpr mul(@Const @ByRef Mat m);

    public native @ByVal Mat cross(@Const @ByRef Mat m);
    public native double dot(@Const @ByRef Mat m);

    @MemberGetter public native @Const MatOp op();
    public native int flags(); public native MatExpr flags(int flags);

    public native @ByRef Mat a(); public native MatExpr a(Mat a);
    public native @ByRef Mat b(); public native MatExpr b(Mat b);
    public native @ByRef Mat c(); public native MatExpr c(Mat c);
    public native double alpha(); public native MatExpr alpha(double alpha);
    public native double beta(); public native MatExpr beta(double beta);
    public native @ByRef Scalar s(); public native MatExpr s(Scalar s);
}


@Namespace("cv") public static native @ByVal @Name("operator+") MatExpr add(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal @Name("operator+") MatExpr add(@Const @ByRef Mat a, @Const @ByRef Scalar s);
@Namespace("cv") public static native @ByVal @Name("operator+") MatExpr add(@Const @ByRef Scalar s, @Const @ByRef Mat a);
@Namespace("cv") public static native @ByVal @Name("operator+") MatExpr add(@Const @ByRef MatExpr e, @Const @ByRef Mat m);
@Namespace("cv") public static native @ByVal @Name("operator+") MatExpr add(@Const @ByRef Mat m, @Const @ByRef MatExpr e);
@Namespace("cv") public static native @ByVal @Name("operator+") MatExpr add(@Const @ByRef MatExpr e, @Const @ByRef Scalar s);
@Namespace("cv") public static native @ByVal @Name("operator+") MatExpr add(@Const @ByRef Scalar s, @Const @ByRef MatExpr e);
@Namespace("cv") public static native @ByVal @Name("operator+") MatExpr add(@Const @ByRef MatExpr e1, @Const @ByRef MatExpr e2);

@Namespace("cv") public static native @ByVal @Name("operator-") MatExpr subtract(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal @Name("operator-") MatExpr subtract(@Const @ByRef Mat a, @Const @ByRef Scalar s);
@Namespace("cv") public static native @ByVal @Name("operator-") MatExpr subtract(@Const @ByRef Scalar s, @Const @ByRef Mat a);
@Namespace("cv") public static native @ByVal @Name("operator-") MatExpr subtract(@Const @ByRef MatExpr e, @Const @ByRef Mat m);
@Namespace("cv") public static native @ByVal @Name("operator-") MatExpr subtract(@Const @ByRef Mat m, @Const @ByRef MatExpr e);
@Namespace("cv") public static native @ByVal @Name("operator-") MatExpr subtract(@Const @ByRef MatExpr e, @Const @ByRef Scalar s);
@Namespace("cv") public static native @ByVal @Name("operator-") MatExpr subtract(@Const @ByRef Scalar s, @Const @ByRef MatExpr e);
@Namespace("cv") public static native @ByVal @Name("operator-") MatExpr subtract(@Const @ByRef MatExpr e1, @Const @ByRef MatExpr e2);

@Namespace("cv") public static native @ByVal @Name("operator-") MatExpr subtract(@Const @ByRef Mat m);
@Namespace("cv") public static native @ByVal @Name("operator-") MatExpr subtract(@Const @ByRef MatExpr e);

@Namespace("cv") public static native @ByVal @Name("operator*") MatExpr multiply(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal @Name("operator*") MatExpr multiply(@Const @ByRef Mat a, double s);
@Namespace("cv") public static native @ByVal @Name("operator*") MatExpr multiply(double s, @Const @ByRef Mat a);
@Namespace("cv") public static native @ByVal @Name("operator*") MatExpr multiply(@Const @ByRef MatExpr e, @Const @ByRef Mat m);
@Namespace("cv") public static native @ByVal @Name("operator*") MatExpr multiply(@Const @ByRef Mat m, @Const @ByRef MatExpr e);
@Namespace("cv") public static native @ByVal @Name("operator*") MatExpr multiply(@Const @ByRef MatExpr e, double s);
@Namespace("cv") public static native @ByVal @Name("operator*") MatExpr multiply(double s, @Const @ByRef MatExpr e);
@Namespace("cv") public static native @ByVal @Name("operator*") MatExpr multiply(@Const @ByRef MatExpr e1, @Const @ByRef MatExpr e2);

@Namespace("cv") public static native @ByVal @Name("operator/") MatExpr divide(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal @Name("operator/") MatExpr divide(@Const @ByRef Mat a, double s);
@Namespace("cv") public static native @ByVal @Name("operator/") MatExpr divide(double s, @Const @ByRef Mat a);
@Namespace("cv") public static native @ByVal @Name("operator/") MatExpr divide(@Const @ByRef MatExpr e, @Const @ByRef Mat m);
@Namespace("cv") public static native @ByVal @Name("operator/") MatExpr divide(@Const @ByRef Mat m, @Const @ByRef MatExpr e);
@Namespace("cv") public static native @ByVal @Name("operator/") MatExpr divide(@Const @ByRef MatExpr e, double s);
@Namespace("cv") public static native @ByVal @Name("operator/") MatExpr divide(double s, @Const @ByRef MatExpr e);
@Namespace("cv") public static native @ByVal @Name("operator/") MatExpr divide(@Const @ByRef MatExpr e1, @Const @ByRef MatExpr e2);

@Namespace("cv") public static native @ByVal @Name("operator<") MatExpr lessThan(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal @Name("operator<") MatExpr lessThan(@Const @ByRef Mat a, double s);
@Namespace("cv") public static native @ByVal @Name("operator<") MatExpr lessThan(double s, @Const @ByRef Mat a);

@Namespace("cv") public static native @ByVal @Name("operator<=") MatExpr lessThanEquals(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal @Name("operator<=") MatExpr lessThanEquals(@Const @ByRef Mat a, double s);
@Namespace("cv") public static native @ByVal @Name("operator<=") MatExpr lessThanEquals(double s, @Const @ByRef Mat a);

@Namespace("cv") public static native @ByVal @Name("operator==") MatExpr equals(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal @Name("operator==") MatExpr equals(@Const @ByRef Mat a, double s);
@Namespace("cv") public static native @ByVal @Name("operator==") MatExpr equals(double s, @Const @ByRef Mat a);

@Namespace("cv") public static native @ByVal @Name("operator!=") MatExpr notEquals(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal @Name("operator!=") MatExpr notEquals(@Const @ByRef Mat a, double s);
@Namespace("cv") public static native @ByVal @Name("operator!=") MatExpr notEquals(double s, @Const @ByRef Mat a);

@Namespace("cv") public static native @ByVal @Name("operator>=") MatExpr greaterThanEquals(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal @Name("operator>=") MatExpr greaterThanEquals(@Const @ByRef Mat a, double s);
@Namespace("cv") public static native @ByVal @Name("operator>=") MatExpr greaterThanEquals(double s, @Const @ByRef Mat a);

@Namespace("cv") public static native @ByVal @Name("operator>") MatExpr greaterThan(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal @Name("operator>") MatExpr greaterThan(@Const @ByRef Mat a, double s);
@Namespace("cv") public static native @ByVal @Name("operator>") MatExpr greaterThan(double s, @Const @ByRef Mat a);

@Namespace("cv") public static native @ByVal @Name("operator&") MatExpr and(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal @Name("operator&") MatExpr and(@Const @ByRef Mat a, @Const @ByRef Scalar s);
@Namespace("cv") public static native @ByVal @Name("operator&") MatExpr and(@Const @ByRef Scalar s, @Const @ByRef Mat a);

@Namespace("cv") public static native @ByVal @Name("operator|") MatExpr or(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal @Name("operator|") MatExpr or(@Const @ByRef Mat a, @Const @ByRef Scalar s);
@Namespace("cv") public static native @ByVal @Name("operator|") MatExpr or(@Const @ByRef Scalar s, @Const @ByRef Mat a);

@Namespace("cv") public static native @ByVal @Name("operator^") MatExpr xor(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal @Name("operator^") MatExpr xor(@Const @ByRef Mat a, @Const @ByRef Scalar s);
@Namespace("cv") public static native @ByVal @Name("operator^") MatExpr xor(@Const @ByRef Scalar s, @Const @ByRef Mat a);

@Namespace("cv") public static native @ByVal @Name("operator~") MatExpr not(@Const @ByRef Mat m);

@Namespace("cv") public static native @ByVal MatExpr min(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal MatExpr min(@Const @ByRef Mat a, double s);
@Namespace("cv") public static native @ByVal MatExpr min(double s, @Const @ByRef Mat a);

@Namespace("cv") public static native @ByVal MatExpr max(@Const @ByRef Mat a, @Const @ByRef Mat b);
@Namespace("cv") public static native @ByVal MatExpr max(@Const @ByRef Mat a, double s);
@Namespace("cv") public static native @ByVal MatExpr max(double s, @Const @ByRef Mat a);

@Namespace("cv") public static native @ByVal MatExpr abs(@Const @ByRef Mat m);
@Namespace("cv") public static native @ByVal MatExpr abs(@Const @ByRef MatExpr e);

 // cv

// #include "opencv2/core/mat.inl.hpp"

// #endif // __OPENCV_CORE_MAT_HPP__


// Parsed from <opencv2/core/persistence.hpp>

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

// #ifndef __OPENCV_CORE_PERSISTENCE_HPP__
// #define __OPENCV_CORE_PERSISTENCE_HPP__

// #ifndef __cplusplus
// #endif

// black-box structures used by FileStorage

// #include "opencv2/core/types.hpp"
// #include "opencv2/core/mat.hpp"

////////////////////////// XML & YAML I/O //////////////////////////

/**
 XML/YAML File Storage Class.

 The class describes an object associated with XML or YAML file.
 It can be used to store data to such a file or read and decode the data.

 The storage is organized as a tree of nested sequences (or lists) and mappings.
 Sequence is a heterogenious array, which elements are accessed by indices or sequentially using an iterator.
 Mapping is analogue of std::map or C structure, which elements are accessed by names.
 The most top level structure is a mapping.
 Leaves of the file storage tree are integers, floating-point numbers and text strings.

 For example, the following code:

 \code
 // open file storage for writing. Type of the file is determined from the extension
 FileStorage fs("test.yml", FileStorage::WRITE);
 fs << "test_int" << 5 << "test_real" << 3.1 << "test_string" << "ABCDEFGH";
 fs << "test_mat" << Mat::eye(3,3,CV_32F);

 fs << "test_list" << "[" << 0.0000000000001 << 2 << CV_PI << -3435345 << "2-502 2-029 3egegeg" <<
 "{:" << "month" << 12 << "day" << 31 << "year" << 1969 << "}" << "]";
 fs << "test_map" << "{" << "x" << 1 << "y" << 2 << "width" << 100 << "height" << 200 << "lbp" << "[:";

 const uchar arr[] = {0, 1, 1, 0, 1, 1, 0, 1};
 fs.writeRaw("u", arr, (int)(sizeof(arr)/sizeof(arr[0])));

 fs << "]" << "}";
 \endcode

 will produce the following file:

 \verbatim
 %YAML:1.0
 test_int: 5
 test_real: 3.1000000000000001e+00
 test_string: ABCDEFGH
 test_mat: !!opencv-matrix
     rows: 3
     cols: 3
     dt: f
     data: [ 1., 0., 0., 0., 1., 0., 0., 0., 1. ]
 test_list:
     - 1.0000000000000000e-13
     - 2
     - 3.1415926535897931e+00
     - -3435345
     - "2-502 2-029 3egegeg"
     - { month:12, day:31, year:1969 }
 test_map:
     x: 1
     y: 2
     width: 100
     height: 200
     lbp: [ 0, 1, 1, 0, 1, 1, 0, 1 ]
 \endverbatim

 and to read the file above, the following code can be used:

 \code
 // open file storage for reading.
 // Type of the file is determined from the content, not the extension
 FileStorage fs("test.yml", FileStorage::READ);
 int test_int = (int)fs["test_int"];
 double test_real = (double)fs["test_real"];
 String test_string = (String)fs["test_string"];

 Mat M;
 fs["test_mat"] >> M;

 FileNode tl = fs["test_list"];
 CV_Assert(tl.type() == FileNode::SEQ && tl.size() == 6);
 double tl0 = (double)tl[0];
 int tl1 = (int)tl[1];
 double tl2 = (double)tl[2];
 int tl3 = (int)tl[3];
 String tl4 = (String)tl[4];
 CV_Assert(tl[5].type() == FileNode::MAP && tl[5].size() == 3);

 int month = (int)tl[5]["month"];
 int day = (int)tl[5]["day"];
 int year = (int)tl[5]["year"];

 FileNode tm = fs["test_map"];

 int x = (int)tm["x"];
 int y = (int)tm["y"];
 int width = (int)tm["width"];
 int height = (int)tm["height"];

 int lbp_val = 0;
 FileNodeIterator it = tm["lbp"].begin();

 for(int k = 0; k < 8; k++, ++it)
    lbp_val |= ((int)*it) << k;
 \endcode
*/
@Namespace("cv") @NoOffset public static class FileStorage extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public FileStorage(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public FileStorage(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public FileStorage position(int position) {
        return (FileStorage)super.position(position);
    }

    /** file storage mode */
    /** enum cv::FileStorage:: */
    public static final int
        READ        = 0, /** read mode */
        WRITE       = 1, /** write mode */
        APPEND      = 2, /** append mode */
        MEMORY      = 4,
        FORMAT_MASK =  (7<<3),
        FORMAT_AUTO = 0,
        FORMAT_XML  =  (1<<3),
        FORMAT_YAML =  (2<<3);
    /** enum cv::FileStorage:: */
    public static final int
        UNDEFINED      = 0,
        VALUE_EXPECTED = 1,
        NAME_EXPECTED  = 2,
        INSIDE_MAP     = 4;
    /** the default constructor */
    public FileStorage() { allocate(); }
    private native void allocate();
    /** the full constructor that opens file storage for reading or writing */
    public FileStorage(@Str BytePointer source, int flags, @Str BytePointer encoding/*=String()*/) { allocate(source, flags, encoding); }
    private native void allocate(@Str BytePointer source, int flags, @Str BytePointer encoding/*=String()*/);
    public FileStorage(@Str BytePointer source, int flags) { allocate(source, flags); }
    private native void allocate(@Str BytePointer source, int flags);
    public FileStorage(@Str String source, int flags, @Str String encoding/*=String()*/) { allocate(source, flags, encoding); }
    private native void allocate(@Str String source, int flags, @Str String encoding/*=String()*/);
    public FileStorage(@Str String source, int flags) { allocate(source, flags); }
    private native void allocate(@Str String source, int flags);
    /** the constructor that takes pointer to the C FileStorage structure */
    public FileStorage(CvFileStorage fs, @Cast("bool") boolean owning/*=true*/) { allocate(fs, owning); }
    private native void allocate(CvFileStorage fs, @Cast("bool") boolean owning/*=true*/);
    public FileStorage(CvFileStorage fs) { allocate(fs); }
    private native void allocate(CvFileStorage fs);
    /** the destructor. calls release() */

    /** opens file storage for reading or writing. The previous storage is closed with release() */
    public native @Cast("bool") boolean open(@Str BytePointer filename, int flags, @Str BytePointer encoding/*=String()*/);
    public native @Cast("bool") boolean open(@Str BytePointer filename, int flags);
    public native @Cast("bool") boolean open(@Str String filename, int flags, @Str String encoding/*=String()*/);
    public native @Cast("bool") boolean open(@Str String filename, int flags);
    /** returns true if the object is associated with currently opened file. */
    public native @Cast("bool") boolean isOpened();
    /** closes the file and releases all the memory buffers */
    public native void release();
    /** closes the file, releases all the memory buffers and returns the text string */
    public native @Str BytePointer releaseAndGetString();

    /** returns the first element of the top-level mapping */
    public native @ByVal FileNode getFirstTopLevelNode();
    /** returns the top-level mapping. YAML supports multiple streams */
    public native @ByVal FileNode root(int streamidx/*=0*/);
    public native @ByVal FileNode root();
    /** returns the specified element of the top-level mapping */
    public native @ByVal @Name("operator[]") FileNode get(@Str BytePointer nodename);
    public native @ByVal @Name("operator[]") FileNode get(@Str String nodename);
    /** returns the specified element of the top-level mapping */

    /** returns pointer to the underlying C FileStorage structure */
    public native @Name("operator*") CvFileStorage multiply();
    /** returns pointer to the underlying C FileStorage structure */
    /** writes one or more numbers of the specified format to the currently written structure */
    public native void writeRaw( @Str BytePointer fmt, @Cast("const uchar*") BytePointer vec, @Cast("size_t") long len );
    public native void writeRaw( @Str String fmt, @Cast("const uchar*") ByteBuffer vec, @Cast("size_t") long len );
    public native void writeRaw( @Str BytePointer fmt, @Cast("const uchar*") byte[] vec, @Cast("size_t") long len );
    public native void writeRaw( @Str String fmt, @Cast("const uchar*") BytePointer vec, @Cast("size_t") long len );
    public native void writeRaw( @Str BytePointer fmt, @Cast("const uchar*") ByteBuffer vec, @Cast("size_t") long len );
    public native void writeRaw( @Str String fmt, @Cast("const uchar*") byte[] vec, @Cast("size_t") long len );
    /** writes the registered C structure (CvMat, CvMatND, CvSeq). See cvWrite() */
    public native void writeObj( @Str BytePointer name, @Const Pointer obj );
    public native void writeObj( @Str String name, @Const Pointer obj );

    /** returns the normalized object name for the specified file name */
    public static native @Str BytePointer getDefaultObjectName(@Str BytePointer filename);
    public static native @Str String getDefaultObjectName(@Str String filename);

    /** the underlying C FileStorage structure */
    public native @Ptr CvFileStorage fs(); public native FileStorage fs(CvFileStorage fs);
    /** the currently written element */
    public native @Str BytePointer elname(); public native FileStorage elname(BytePointer elname);
    /** the stack of written structures */
    public native @Cast("char*") @StdVector BytePointer structs(); public native FileStorage structs(BytePointer structs);
    /** the writer state */
    public native int state(); public native FileStorage state(int state);
}



/**
 File Storage Node class

 The node is used to store each and every element of the file storage opened for reading -
 from the primitive objects, such as numbers and text strings, to the complex nodes:
 sequences, mappings and the registered objects.

 Note that file nodes are only used for navigating file storages opened for reading.
 When a file storage is opened for writing, no data is stored in memory after it is written.
*/
@Namespace("cv") @NoOffset public static class FileNode extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public FileNode(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public FileNode(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public FileNode position(int position) {
        return (FileNode)super.position(position);
    }

    /** type of the file storage node */
    /** enum cv::FileNode:: */
    public static final int
        /** empty node */
        NONE      = 0,
        /** an integer */
        INT       = 1,
        /** floating-point number */
        REAL      = 2,
        /** synonym or REAL */
        FLOAT     =  REAL,
        /** text string in UTF-8 encoding */
        STR       = 3,
        /** synonym for STR */
        STRING    =  STR,
        /** integer of size size_t. Typically used for storing complex dynamic structures where some elements reference the others */
        REF       = 4,
        /** sequence */
        SEQ       = 5,
        /** mapping */
        MAP       = 6,
        TYPE_MASK = 7,
        /** compact representation of a sequence or mapping. Used only by YAML writer */
        FLOW      = 8,
        /** a registered object (e.g. a matrix) */
        USER      = 16,
        /** empty structure (sequence or mapping) */
        EMPTY     = 32,
        /** the node has a name (i.e. it is element of a mapping) */
        NAMED     = 64;
    /** the default constructor */
    public FileNode() { allocate(); }
    private native void allocate();
    /** the full constructor wrapping CvFileNode structure. */
    public FileNode(@Const CvFileStorage fs, @Const CvFileNode node) { allocate(fs, node); }
    private native void allocate(@Const CvFileStorage fs, @Const CvFileNode node);
    /** the copy constructor */
    public FileNode(@Const @ByRef FileNode node) { allocate(node); }
    private native void allocate(@Const @ByRef FileNode node);
    /** returns element of a mapping node */
    public native @ByVal @Name("operator[]") FileNode get(@Str BytePointer nodename);
    public native @ByVal @Name("operator[]") FileNode get(@Str String nodename);
    /** returns element of a mapping node */
    /** returns element of a sequence node */
    public native @ByVal @Name("operator[]") FileNode get(int i);
    /** returns type of the node */
    public native int type();

    /** returns true if the node is empty */
    public native @Cast("bool") boolean empty();
    /** returns true if the node is a "none" object */
    public native @Cast("bool") boolean isNone();
    /** returns true if the node is a sequence */
    public native @Cast("bool") boolean isSeq();
    /** returns true if the node is a mapping */
    public native @Cast("bool") boolean isMap();
    /** returns true if the node is an integer */
    public native @Cast("bool") boolean isInt();
    /** returns true if the node is a floating-point number */
    public native @Cast("bool") boolean isReal();
    /** returns true if the node is a text string */
    public native @Cast("bool") boolean isString();
    /** returns true if the node has a name */
    public native @Cast("bool") boolean isNamed();
    /** returns the node name or an empty string if the node is nameless */
    public native @Str BytePointer name();
    /** returns the number of elements in the node, if it is a sequence or mapping, or 1 otherwise. */
    public native @Cast("size_t") long size();
    /** returns the node content as an integer. If the node stores floating-point number, it is rounded. */
    public native @Name("operator int") int asInt();
    /** returns the node content as float */
    public native @Name("operator float") float asFloat();
    /** returns the node content as double */
    public native @Name("operator double") double asDouble();
    /** returns the node content as text string */
    public native @Name("operator cv::String") @Str BytePointer asBytePointer();
// #ifndef OPENCV_NOSTL
// #endif

    /** returns pointer to the underlying file node */
    public native @Name("operator*") CvFileNode multiply();
    /** returns pointer to the underlying file node */

    /** returns iterator pointing to the first node element */
    public native @ByVal FileNodeIterator begin();
    /** returns iterator pointing to the element following the last node element */
    public native @ByVal FileNodeIterator end();

    /** reads node elements to the buffer with the specified format */
    public native void readRaw( @Str BytePointer fmt, @Cast("uchar*") BytePointer vec, @Cast("size_t") long len );
    public native void readRaw( @Str String fmt, @Cast("uchar*") ByteBuffer vec, @Cast("size_t") long len );
    public native void readRaw( @Str BytePointer fmt, @Cast("uchar*") byte[] vec, @Cast("size_t") long len );
    public native void readRaw( @Str String fmt, @Cast("uchar*") BytePointer vec, @Cast("size_t") long len );
    public native void readRaw( @Str BytePointer fmt, @Cast("uchar*") ByteBuffer vec, @Cast("size_t") long len );
    public native void readRaw( @Str String fmt, @Cast("uchar*") byte[] vec, @Cast("size_t") long len );
    /** reads the registered object and returns pointer to it */
    public native Pointer readObj();

    // do not use wrapper pointer classes for better efficiency
    @MemberGetter public native @Const CvFileStorage fs();
    @MemberGetter public native @Const CvFileNode node();
}


/**
 File Node Iterator

 The class is used for iterating sequences (usually) and mappings.
 */
@Namespace("cv") @NoOffset public static class FileNodeIterator extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public FileNodeIterator(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public FileNodeIterator(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public FileNodeIterator position(int position) {
        return (FileNodeIterator)super.position(position);
    }

    /** the default constructor */
    public FileNodeIterator() { allocate(); }
    private native void allocate();
    /** the full constructor set to the ofs-th element of the node */
    public FileNodeIterator(@Const CvFileStorage fs, @Const CvFileNode node, @Cast("size_t") long ofs/*=0*/) { allocate(fs, node, ofs); }
    private native void allocate(@Const CvFileStorage fs, @Const CvFileNode node, @Cast("size_t") long ofs/*=0*/);
    public FileNodeIterator(@Const CvFileStorage fs, @Const CvFileNode node) { allocate(fs, node); }
    private native void allocate(@Const CvFileStorage fs, @Const CvFileNode node);
    /** the copy constructor */
    public FileNodeIterator(@Const @ByRef FileNodeIterator it) { allocate(it); }
    private native void allocate(@Const @ByRef FileNodeIterator it);
    /** returns the currently observed element */
    public native @ByVal @Name("operator*") FileNode multiply();
    /** accesses the currently observed element methods */
    public native @ByVal @Name("operator->") FileNode access();

    /** moves iterator to the next node */
    public native @ByRef @Name("operator++") FileNodeIterator increment();
    /** moves iterator to the next node */
    public native @ByVal @Name("operator++") FileNodeIterator increment(int arg0);
    /** moves iterator to the previous node */
    public native @ByRef @Name("operator--") FileNodeIterator decrement();
    /** moves iterator to the previous node */
    public native @ByVal @Name("operator--") FileNodeIterator decrement(int arg0);
    /** moves iterator forward by the specified offset (possibly negative) */
    public native @ByRef @Name("operator+=") FileNodeIterator addPut(int ofs);
    /** moves iterator backward by the specified offset (possibly negative) */
    public native @ByRef @Name("operator-=") FileNodeIterator subtractPut(int ofs);

    /** reads the next maxCount elements (or less, if the sequence/mapping last element occurs earlier) to the buffer with the specified format */
    public native @ByRef FileNodeIterator readRaw( @Str BytePointer fmt, @Cast("uchar*") BytePointer vec,
                                   @Cast("size_t") long maxCount/*=(size_t)INT_MAX*/ );
    public native @ByRef FileNodeIterator readRaw( @Str BytePointer fmt, @Cast("uchar*") BytePointer vec );
    public native @ByRef FileNodeIterator readRaw( @Str String fmt, @Cast("uchar*") ByteBuffer vec,
                                   @Cast("size_t") long maxCount/*=(size_t)INT_MAX*/ );
    public native @ByRef FileNodeIterator readRaw( @Str String fmt, @Cast("uchar*") ByteBuffer vec );
    public native @ByRef FileNodeIterator readRaw( @Str BytePointer fmt, @Cast("uchar*") byte[] vec,
                                   @Cast("size_t") long maxCount/*=(size_t)INT_MAX*/ );
    public native @ByRef FileNodeIterator readRaw( @Str BytePointer fmt, @Cast("uchar*") byte[] vec );
    public native @ByRef FileNodeIterator readRaw( @Str String fmt, @Cast("uchar*") BytePointer vec,
                                   @Cast("size_t") long maxCount/*=(size_t)INT_MAX*/ );
    public native @ByRef FileNodeIterator readRaw( @Str String fmt, @Cast("uchar*") BytePointer vec );
    public native @ByRef FileNodeIterator readRaw( @Str BytePointer fmt, @Cast("uchar*") ByteBuffer vec,
                                   @Cast("size_t") long maxCount/*=(size_t)INT_MAX*/ );
    public native @ByRef FileNodeIterator readRaw( @Str BytePointer fmt, @Cast("uchar*") ByteBuffer vec );
    public native @ByRef FileNodeIterator readRaw( @Str String fmt, @Cast("uchar*") byte[] vec,
                                   @Cast("size_t") long maxCount/*=(size_t)INT_MAX*/ );
    public native @ByRef FileNodeIterator readRaw( @Str String fmt, @Cast("uchar*") byte[] vec );

    public static class SeqReader extends Pointer {
        static { Loader.load(); }
        /** Default native constructor. */
        public SeqReader() { allocate(); }
        /** Native array allocator. Access with {@link Pointer#position(int)}. */
        public SeqReader(int size) { allocateArray(size); }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public SeqReader(Pointer p) { super(p); }
        private native void allocate();
        private native void allocateArray(int size);
        @Override public SeqReader position(int position) {
            return (SeqReader)super.position(position);
        }
    
      public native int header_size(); public native SeqReader header_size(int header_size);
      public native Pointer seq(); public native SeqReader seq(Pointer seq);        /* sequence, beign read; CvSeq      */
      public native Pointer block(); public native SeqReader block(Pointer block);      /* current block;        CvSeqBlock */
      public native @Cast("schar*") BytePointer ptr(); public native SeqReader ptr(BytePointer ptr);        /* pointer to element be read next */
      public native @Cast("schar*") BytePointer block_min(); public native SeqReader block_min(BytePointer block_min);  /* pointer to the beginning of block */
      public native @Cast("schar*") BytePointer block_max(); public native SeqReader block_max(BytePointer block_max);  /* pointer to the end of block */
      public native int delta_index(); public native SeqReader delta_index(int delta_index);/* = seq->first->start_index   */
      public native @Cast("schar*") BytePointer prev_elem(); public native SeqReader prev_elem(BytePointer prev_elem);  /* pointer to previous element */
    }

    @MemberGetter public native @Const CvFileStorage fs();
    @MemberGetter public native @Const CvFileNode container();
    public native @ByRef SeqReader reader(); public native FileNodeIterator reader(SeqReader reader);
    public native @Cast("size_t") long remaining(); public native FileNodeIterator remaining(long remaining);
}



/////////////////// XML & YAML I/O implementation //////////////////

@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str BytePointer name, int value );
@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str String name, int value );
@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str BytePointer name, float value );
@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str String name, float value );
@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str BytePointer name, double value );
@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str String name, double value );
@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str BytePointer name, @Str BytePointer value );
@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str String name, @Str String value );
@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str BytePointer name, @Const @ByRef Mat value );
@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str String name, @Const @ByRef Mat value );
@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str BytePointer name, @Const @ByRef SparseMat value );
@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str String name, @Const @ByRef SparseMat value );
@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str BytePointer name, @StdVector KeyPoint value);
@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str String name, @StdVector KeyPoint value);

@Namespace("cv") public static native void writeScalar( @ByRef FileStorage fs, int value );
@Namespace("cv") public static native void writeScalar( @ByRef FileStorage fs, float value );
@Namespace("cv") public static native void writeScalar( @ByRef FileStorage fs, double value );
@Namespace("cv") public static native void writeScalar( @ByRef FileStorage fs, @Str BytePointer value );
@Namespace("cv") public static native void writeScalar( @ByRef FileStorage fs, @Str String value );

@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @ByRef IntPointer value, int default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @ByRef IntBuffer value, int default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @ByRef int[] value, int default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @ByRef FloatPointer value, float default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @ByRef FloatBuffer value, float default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @ByRef float[] value, float default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @ByRef DoublePointer value, double default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @ByRef DoubleBuffer value, double default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @ByRef double[] value, double default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @Str BytePointer value, @Str BytePointer default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @Str String value, @Str String default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @ByRef Mat mat, @Const @ByRef Mat default_mat/*=Mat()*/ );
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @ByRef Mat mat );
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @ByRef SparseMat mat, @Const @ByRef SparseMat default_mat/*=SparseMat()*/ );
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @ByRef SparseMat mat );
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @StdVector KeyPoint keypoints);

@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @ByRef Range value, @Const @ByRef Range default_value);


@Namespace("cv") public static native @ByRef @Name("operator<<") FileStorage shiftLeft(@ByRef FileStorage fs, @Str BytePointer str);
@Namespace("cv") public static native @ByRef @Name("operator<<") FileStorage shiftLeft(@ByRef FileStorage fs, @Str String str);
    @Namespace("cv::internal") @NoOffset public static class WriteStructContext extends Pointer {
        static { Loader.load(); }
        /** Empty constructor. */
        public WriteStructContext() { }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public WriteStructContext(Pointer p) { super(p); }
    
        public WriteStructContext(@ByRef FileStorage _fs, @Str BytePointer name, int flags, @Str BytePointer typeName/*=String()*/) { allocate(_fs, name, flags, typeName); }
        private native void allocate(@ByRef FileStorage _fs, @Str BytePointer name, int flags, @Str BytePointer typeName/*=String()*/);
        public WriteStructContext(@ByRef FileStorage _fs, @Str BytePointer name, int flags) { allocate(_fs, name, flags); }
        private native void allocate(@ByRef FileStorage _fs, @Str BytePointer name, int flags);
        public WriteStructContext(@ByRef FileStorage _fs, @Str String name, int flags, @Str String typeName/*=String()*/) { allocate(_fs, name, flags, typeName); }
        private native void allocate(@ByRef FileStorage _fs, @Str String name, int flags, @Str String typeName/*=String()*/);
        public WriteStructContext(@ByRef FileStorage _fs, @Str String name, int flags) { allocate(_fs, name, flags); }
        private native void allocate(@ByRef FileStorage _fs, @Str String name, int flags);
    }

 // internal

@Namespace("cv") public static native void write( @ByRef FileStorage fs, int value );

@Namespace("cv") public static native void write( @ByRef FileStorage fs, float value );

@Namespace("cv") public static native void write( @ByRef FileStorage fs, double value );

@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str BytePointer value );
@Namespace("cv") public static native void write( @ByRef FileStorage fs, @Str String value );

@Namespace("cv") public static native void write(@ByRef FileStorage fs, @Const @ByRef Range r );

@Namespace("cv") public static native void write(@ByRef FileStorage fs, @Str BytePointer name, @Const @ByRef Range r );
@Namespace("cv") public static native void write(@ByRef FileStorage fs, @Str String name, @Const @ByRef Range r );


@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @Cast("bool*") @ByRef BoolPointer value, @Cast("bool") boolean default_value);

@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @Cast("uchar*") @ByRef BytePointer value, @Cast("uchar") byte default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @Cast("uchar*") @ByRef ByteBuffer value, @Cast("uchar") byte default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @Cast("uchar*") @ByRef byte[] value, @Cast("uchar") byte default_value);

@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @Cast("ushort*") @ByRef ShortPointer value, @Cast("ushort") short default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @Cast("ushort*") @ByRef ShortBuffer value, @Cast("ushort") short default_value);
@Namespace("cv") public static native void read(@Const @ByRef FileNode node, @Cast("ushort*") @ByRef short[] value, @Cast("ushort") short default_value);

@Namespace("cv") public static native @ByRef @Name("operator<<") FileStorage shiftLeft(@ByRef FileStorage fs, @Cast("char*") ByteBuffer value);
@Namespace("cv") public static native @ByRef @Name("operator<<") FileStorage shiftLeft(@ByRef FileStorage fs, @Cast("char*") byte[] value);


@Namespace("cv") public static native @Cast("bool") @Name("operator==") boolean equals(@Const @ByRef FileNodeIterator it1, @Const @ByRef FileNodeIterator it2);

@Namespace("cv") public static native @Cast("bool") @Name("operator!=") boolean notEquals(@Const @ByRef FileNodeIterator it1, @Const @ByRef FileNodeIterator it2);

@Namespace("cv") public static native @Cast("ptrdiff_t") @Name("operator-") long subtract(@Const @ByRef FileNodeIterator it1, @Const @ByRef FileNodeIterator it2);

@Namespace("cv") public static native @Cast("bool") @Name("operator<") boolean lessThan(@Const @ByRef FileNodeIterator it1, @Const @ByRef FileNodeIterator it2);

























 // cv

// #endif // __OPENCV_CORE_PERSISTENCE_HPP__


// Parsed from <opencv2/core/optim.hpp>

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
// In no event shall the OpenCV Foundation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

// #ifndef __OPENCV_OPTIM_HPP__
// #define __OPENCV_OPTIM_HPP__

// #include "opencv2/core.hpp"

@Namespace("cv") public static class MinProblemSolver extends Algorithm {
    static { Loader.load(); }
    /** Empty constructor. */
    public MinProblemSolver() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public MinProblemSolver(Pointer p) { super(p); }

    @Const public static abstract class Function extends Pointer {
        static { Loader.load(); }
        /** Default native constructor. */
        public Function() { allocate(); }
        /** Native array allocator. Access with {@link Pointer#position(int)}. */
        public Function(int size) { allocateArray(size); }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public Function(Pointer p) { super(p); }
        private native void allocate();
        private native void allocateArray(int size);
        @Override public Function position(int position) {
            return (Function)super.position(position);
        }
    
       @Virtual public abstract double calc(@Const DoublePointer x);
       @Virtual public native void getGradient(@Const DoublePointer arg0,DoublePointer arg1);
    }

    public native @Ptr Function getFunction();
    public native void setFunction(@Ptr Function f);

    public native @ByVal TermCriteria getTermCriteria();
    public native void setTermCriteria(@Const @ByRef TermCriteria termcrit);

    // x contain the initial point before the call and the minima position (if algorithm converged) after. x is assumed to be (something that
    // after getMat() will return) row-vector or column-vector. *It's size  and should
    // be consisted with previous dimensionality data given, if any (otherwise, it determines dimensionality)*
    public native double minimize(@ByVal Mat x);
}

/** downhill simplex class */
@Namespace("cv") public static class DownhillSolver extends MinProblemSolver {
    static { Loader.load(); }
    /** Empty constructor. */
    public DownhillSolver() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public DownhillSolver(Pointer p) { super(p); }

    /** returns row-vector, even if the column-vector was given */
    public native void getInitStep(@ByVal Mat step);
    /**This should be called at least once before the first call to minimize() and step is assumed to be (something that
     *  after getMat() will return) row-vector or column-vector. *It's dimensionality determines the dimensionality of a problem.* */
    public native void setInitStep(@ByVal Mat step);

    // both minRange & minError are specified by termcrit.epsilon;
    // In addition, user may specify the number of iterations that the algorithm does.
    public static native @Ptr DownhillSolver create(@Ptr Function f/*=Ptr<MinProblemSolver::Function>()*/,
                                          @ByVal Mat initStep/*=Mat_<double>(1,1,0.0)*/,
                                          @ByVal TermCriteria termcrit/*=TermCriteria(TermCriteria::MAX_ITER+TermCriteria::EPS,5000,0.000001)*/);
    public static native @Ptr DownhillSolver create();
}

/** conjugate gradient method */
@Namespace("cv") public static class ConjGradSolver extends MinProblemSolver {
    static { Loader.load(); }
    /** Empty constructor. */
    public ConjGradSolver() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public ConjGradSolver(Pointer p) { super(p); }

    public static native @Ptr ConjGradSolver create(@Ptr Function f/*=Ptr<ConjGradSolver::Function>()*/,
                                          @ByVal TermCriteria termcrit/*=TermCriteria(TermCriteria::MAX_ITER+TermCriteria::EPS,5000,0.000001)*/);
    public static native @Ptr ConjGradSolver create();
}

/**the return codes for solveLP() function */
/** enum cv:: */
public static final int
    SOLVELP_UNBOUNDED    = -2, //problem is unbounded (target function can achieve arbitrary high values)
    SOLVELP_UNFEASIBLE    = -1, //problem is unfeasible (there are no points that satisfy all the constraints imposed)
    SOLVELP_SINGLE    = 0, //there is only one maximum for target function
    SOLVELP_MULTI    = 1; //there are multiple maxima for target function - the arbitrary one is returned

@Namespace("cv") public static native int solveLP(@Const @ByRef Mat Func, @Const @ByRef Mat Constr, @ByRef Mat z);

// cv

// #endif


}
