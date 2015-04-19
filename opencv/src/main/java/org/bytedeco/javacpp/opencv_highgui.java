// Targeted by JavaCPP version 0.11

package org.bytedeco.javacpp;

import java.nio.*;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import static org.bytedeco.javacpp.opencv_core.*;
import static org.bytedeco.javacpp.opencv_imgproc.*;
import static org.bytedeco.javacpp.opencv_imgcodecs.*;
import static org.bytedeco.javacpp.opencv_videoio.*;

public class opencv_highgui extends org.bytedeco.javacpp.presets.opencv_highgui {
    static { Loader.load(); }

// Parsed from <opencv2/highgui/highgui_c.h>

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
// Copyright (C) 2000, Intel Corporation, all rights reserved.
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
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

// #ifndef __OPENCV_HIGHGUI_H__
// #define __OPENCV_HIGHGUI_H__

// #include "opencv2/core/core_c.h"
// #include "opencv2/imgproc/imgproc_c.h"
// #include "opencv2/imgcodecs/imgcodecs_c.h"
// #include "opencv2/videoio/videoio_c.h"

// #ifdef __cplusplus
// #endif /* __cplusplus */

/****************************************************************************************\
*                                  Basic GUI functions                                   *
\****************************************************************************************/
//YV
//-----------New for Qt
/* For font */
/** enum  */
public static final int  CV_FONT_LIGHT           = 25,//QFont::Light,
        CV_FONT_NORMAL          = 50,//QFont::Normal,
        CV_FONT_DEMIBOLD        = 63,//QFont::DemiBold,
        CV_FONT_BOLD            = 75,//QFont::Bold,
        CV_FONT_BLACK           = 87; //QFont::Black

/** enum  */
public static final int  CV_STYLE_NORMAL         = 0,//QFont::StyleNormal,
        CV_STYLE_ITALIC         = 1,//QFont::StyleItalic,
        CV_STYLE_OBLIQUE        = 2; //QFont::StyleOblique
/* ---------*/

//for color cvScalar(blue_component, green_component, red\_component[, alpha_component])
//and alpha= 0 <-> 0xFF (not transparent <-> transparent)
public static native @ByVal @Platform("linux") CvFont cvFontQt(@Cast("const char*") BytePointer nameFont, int pointSize/*=-1*/, @ByVal CvScalar color/*=cvScalarAll(0)*/, int weight/*=CV_FONT_NORMAL*/,  int style/*=CV_STYLE_NORMAL*/, int spacing/*=0*/);
public static native @ByVal @Platform("linux") CvFont cvFontQt(@Cast("const char*") BytePointer nameFont);
public static native @ByVal @Platform("linux") CvFont cvFontQt(String nameFont, int pointSize/*=-1*/, @ByVal CvScalar color/*=cvScalarAll(0)*/, int weight/*=CV_FONT_NORMAL*/,  int style/*=CV_STYLE_NORMAL*/, int spacing/*=0*/);
public static native @ByVal @Platform("linux") CvFont cvFontQt(String nameFont);

public static native @Platform("linux") void cvAddText(@Const CvArr img, @Cast("const char*") BytePointer text, @ByVal CvPoint org, CvFont arg2);
public static native @Platform("linux") void cvAddText(@Const CvArr img, String text, @ByVal @Cast("CvPoint*") IntBuffer org, CvFont arg2);
public static native @Platform("linux") void cvAddText(@Const CvArr img, @Cast("const char*") BytePointer text, @ByVal @Cast("CvPoint*") int[] org, CvFont arg2);
public static native @Platform("linux") void cvAddText(@Const CvArr img, String text, @ByVal CvPoint org, CvFont arg2);
public static native @Platform("linux") void cvAddText(@Const CvArr img, @Cast("const char*") BytePointer text, @ByVal @Cast("CvPoint*") IntBuffer org, CvFont arg2);
public static native @Platform("linux") void cvAddText(@Const CvArr img, String text, @ByVal @Cast("CvPoint*") int[] org, CvFont arg2);

public static native @Platform("linux") void cvDisplayOverlay(@Cast("const char*") BytePointer name, @Cast("const char*") BytePointer text, int delayms/*=0*/);
public static native @Platform("linux") void cvDisplayOverlay(@Cast("const char*") BytePointer name, @Cast("const char*") BytePointer text);
public static native @Platform("linux") void cvDisplayOverlay(String name, String text, int delayms/*=0*/);
public static native @Platform("linux") void cvDisplayOverlay(String name, String text);
public static native @Platform("linux") void cvDisplayStatusBar(@Cast("const char*") BytePointer name, @Cast("const char*") BytePointer text, int delayms/*=0*/);
public static native @Platform("linux") void cvDisplayStatusBar(@Cast("const char*") BytePointer name, @Cast("const char*") BytePointer text);
public static native @Platform("linux") void cvDisplayStatusBar(String name, String text, int delayms/*=0*/);
public static native @Platform("linux") void cvDisplayStatusBar(String name, String text);

public static native @Platform("linux") void cvSaveWindowParameters(@Cast("const char*") BytePointer name);
public static native @Platform("linux") void cvSaveWindowParameters(String name);
public static native @Platform("linux") void cvLoadWindowParameters(@Cast("const char*") BytePointer name);
public static native @Platform("linux") void cvLoadWindowParameters(String name);
public static class Pt2Func_int_PointerPointer extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    Pt2Func_int_PointerPointer(Pointer p) { super(p); }
    protected Pt2Func_int_PointerPointer() { allocate(); }
    private native void allocate();
    public native int call(int argc, @Cast("char**") PointerPointer argv);
}
public static native @Platform("linux") int cvStartLoop(Pt2Func_int_PointerPointer pt2Func, int argc, @Cast("char**") PointerPointer argv);
public static class Pt2Func_int_BytePointer extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    Pt2Func_int_BytePointer(Pointer p) { super(p); }
    protected Pt2Func_int_BytePointer() { allocate(); }
    private native void allocate();
    public native int call(int argc, @Cast("char**") @ByPtrPtr BytePointer argv);
}
public static native @Platform("linux") int cvStartLoop(Pt2Func_int_BytePointer pt2Func, int argc, @Cast("char**") @ByPtrPtr BytePointer argv);
public static class Pt2Func_int_ByteBuffer extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    Pt2Func_int_ByteBuffer(Pointer p) { super(p); }
    protected Pt2Func_int_ByteBuffer() { allocate(); }
    private native void allocate();
    public native int call(int argc, @Cast("char**") @ByPtrPtr ByteBuffer argv);
}
public static native @Platform("linux") int cvStartLoop(Pt2Func_int_ByteBuffer pt2Func, int argc, @Cast("char**") @ByPtrPtr ByteBuffer argv);
public static class Pt2Func_int_byte__ extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    Pt2Func_int_byte__(Pointer p) { super(p); }
    protected Pt2Func_int_byte__() { allocate(); }
    private native void allocate();
    public native int call(int argc, @Cast("char**") @ByPtrPtr byte[] argv);
}
public static native @Platform("linux") int cvStartLoop(Pt2Func_int_byte__ pt2Func, int argc, @Cast("char**") @ByPtrPtr byte[] argv);
public static native @Platform("linux") void cvStopLoop( );

@Convention("CV_CDECL") public static class CvButtonCallback extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    CvButtonCallback(Pointer p) { super(p); }
    protected CvButtonCallback() { allocate(); }
    private native void allocate();
    public native void call(int state, Pointer userdata);
}
/** enum  */
public static final int CV_PUSH_BUTTON = 0, CV_CHECKBOX = 1, CV_RADIOBOX = 2;
public static native @Platform("linux") int cvCreateButton( @Cast("const char*") BytePointer button_name/*=NULL*/,CvButtonCallback on_change/*=NULL*/, Pointer userdata/*=NULL*/, int button_type/*=CV_PUSH_BUTTON*/, int initial_button_state/*=0*/);
public static native @Platform("linux") int cvCreateButton();
public static native @Platform("linux") int cvCreateButton( String button_name/*=NULL*/,CvButtonCallback on_change/*=NULL*/, Pointer userdata/*=NULL*/, int button_type/*=CV_PUSH_BUTTON*/, int initial_button_state/*=0*/);
//----------------------


/* this function is used to set some external parameters in case of X Window */
public static native int cvInitSystem( int argc, @Cast("char**") PointerPointer argv );
public static native int cvInitSystem( int argc, @Cast("char**") @ByPtrPtr BytePointer argv );
public static native int cvInitSystem( int argc, @Cast("char**") @ByPtrPtr ByteBuffer argv );
public static native int cvInitSystem( int argc, @Cast("char**") @ByPtrPtr byte[] argv );

public static native int cvStartWindowThread( );

// ---------  YV ---------
/** enum  */
public static final int
    //These 3 flags are used by cvSet/GetWindowProperty
    CV_WND_PROP_FULLSCREEN = 0, //to change/get window's fullscreen property
    CV_WND_PROP_AUTOSIZE   = 1, //to change/get window's autosize property
    CV_WND_PROP_ASPECTRATIO= 2, //to change/get window's aspectratio property
    CV_WND_PROP_OPENGL     = 3, //to change/get window's opengl support

    //These 2 flags are used by cvNamedWindow and cvSet/GetWindowProperty
    CV_WINDOW_NORMAL       =  0x00000000, //the user can resize the window (no constraint)  / also use to switch a fullscreen window to a normal size
    CV_WINDOW_AUTOSIZE     =  0x00000001, //the user cannot resize the window, the size is constrainted by the image displayed
    CV_WINDOW_OPENGL       =  0x00001000, //window with opengl support

    //Those flags are only for Qt
    CV_GUI_EXPANDED         =  0x00000000, //status bar and tool bar
    CV_GUI_NORMAL           =  0x00000010, //old fashious way

    //These 3 flags are used by cvNamedWindow and cvSet/GetWindowProperty
    CV_WINDOW_FULLSCREEN   = 1,//change the window to fullscreen
    CV_WINDOW_FREERATIO    =  0x00000100,//the image expends as much as it can (no ratio constraint)
    CV_WINDOW_KEEPRATIO    =  0x00000000;//the ration image is respected.

/* create window */
public static native int cvNamedWindow( @Cast("const char*") BytePointer name, int flags/*=CV_WINDOW_AUTOSIZE*/ );
public static native int cvNamedWindow( @Cast("const char*") BytePointer name );
public static native int cvNamedWindow( String name, int flags/*=CV_WINDOW_AUTOSIZE*/ );
public static native int cvNamedWindow( String name );

/* Set and Get Property of the window */
public static native void cvSetWindowProperty(@Cast("const char*") BytePointer name, int prop_id, double prop_value);
public static native void cvSetWindowProperty(String name, int prop_id, double prop_value);
public static native double cvGetWindowProperty(@Cast("const char*") BytePointer name, int prop_id);
public static native double cvGetWindowProperty(String name, int prop_id);

/* display image within window (highgui windows remember their content) */
public static native void cvShowImage( @Cast("const char*") BytePointer name, @Const CvArr image );
public static native void cvShowImage( String name, @Const CvArr image );

/* resize/move window */
public static native void cvResizeWindow( @Cast("const char*") BytePointer name, int width, int height );
public static native void cvResizeWindow( String name, int width, int height );
public static native void cvMoveWindow( @Cast("const char*") BytePointer name, int x, int y );
public static native void cvMoveWindow( String name, int x, int y );


/* destroy window and all the trackers associated with it */
public static native void cvDestroyWindow( @Cast("const char*") BytePointer name );
public static native void cvDestroyWindow( String name );

public static native void cvDestroyAllWindows();

/* get native window handle (HWND in case of Win32 and Widget in case of X Window) */
public static native Pointer cvGetWindowHandle( @Cast("const char*") BytePointer name );
public static native Pointer cvGetWindowHandle( String name );

/* get name of highgui window given its native handle */
public static native @Cast("const char*") BytePointer cvGetWindowName( Pointer window_handle );


@Convention("CV_CDECL") public static class CvTrackbarCallback extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    CvTrackbarCallback(Pointer p) { super(p); }
    protected CvTrackbarCallback() { allocate(); }
    private native void allocate();
    public native void call(int pos);
}

/* create trackbar and display it on top of given window, set callback */
public static native int cvCreateTrackbar( @Cast("const char*") BytePointer trackbar_name, @Cast("const char*") BytePointer window_name,
                             IntPointer value, int count, CvTrackbarCallback on_change/*=NULL*/);
public static native int cvCreateTrackbar( @Cast("const char*") BytePointer trackbar_name, @Cast("const char*") BytePointer window_name,
                             IntPointer value, int count);
public static native int cvCreateTrackbar( String trackbar_name, String window_name,
                             IntBuffer value, int count, CvTrackbarCallback on_change/*=NULL*/);
public static native int cvCreateTrackbar( String trackbar_name, String window_name,
                             IntBuffer value, int count);
public static native int cvCreateTrackbar( @Cast("const char*") BytePointer trackbar_name, @Cast("const char*") BytePointer window_name,
                             int[] value, int count, CvTrackbarCallback on_change/*=NULL*/);
public static native int cvCreateTrackbar( @Cast("const char*") BytePointer trackbar_name, @Cast("const char*") BytePointer window_name,
                             int[] value, int count);
public static native int cvCreateTrackbar( String trackbar_name, String window_name,
                             IntPointer value, int count, CvTrackbarCallback on_change/*=NULL*/);
public static native int cvCreateTrackbar( String trackbar_name, String window_name,
                             IntPointer value, int count);
public static native int cvCreateTrackbar( @Cast("const char*") BytePointer trackbar_name, @Cast("const char*") BytePointer window_name,
                             IntBuffer value, int count, CvTrackbarCallback on_change/*=NULL*/);
public static native int cvCreateTrackbar( @Cast("const char*") BytePointer trackbar_name, @Cast("const char*") BytePointer window_name,
                             IntBuffer value, int count);
public static native int cvCreateTrackbar( String trackbar_name, String window_name,
                             int[] value, int count, CvTrackbarCallback on_change/*=NULL*/);
public static native int cvCreateTrackbar( String trackbar_name, String window_name,
                             int[] value, int count);

@Convention("CV_CDECL") public static class CvTrackbarCallback2 extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    CvTrackbarCallback2(Pointer p) { super(p); }
    protected CvTrackbarCallback2() { allocate(); }
    private native void allocate();
    public native void call(int pos, Pointer userdata);
}

public static native int cvCreateTrackbar2( @Cast("const char*") BytePointer trackbar_name, @Cast("const char*") BytePointer window_name,
                              IntPointer value, int count, CvTrackbarCallback2 on_change,
                              Pointer userdata/*=0*/);
public static native int cvCreateTrackbar2( @Cast("const char*") BytePointer trackbar_name, @Cast("const char*") BytePointer window_name,
                              IntPointer value, int count, CvTrackbarCallback2 on_change);
public static native int cvCreateTrackbar2( String trackbar_name, String window_name,
                              IntBuffer value, int count, CvTrackbarCallback2 on_change,
                              Pointer userdata/*=0*/);
public static native int cvCreateTrackbar2( String trackbar_name, String window_name,
                              IntBuffer value, int count, CvTrackbarCallback2 on_change);
public static native int cvCreateTrackbar2( @Cast("const char*") BytePointer trackbar_name, @Cast("const char*") BytePointer window_name,
                              int[] value, int count, CvTrackbarCallback2 on_change,
                              Pointer userdata/*=0*/);
public static native int cvCreateTrackbar2( @Cast("const char*") BytePointer trackbar_name, @Cast("const char*") BytePointer window_name,
                              int[] value, int count, CvTrackbarCallback2 on_change);
public static native int cvCreateTrackbar2( String trackbar_name, String window_name,
                              IntPointer value, int count, CvTrackbarCallback2 on_change,
                              Pointer userdata/*=0*/);
public static native int cvCreateTrackbar2( String trackbar_name, String window_name,
                              IntPointer value, int count, CvTrackbarCallback2 on_change);
public static native int cvCreateTrackbar2( @Cast("const char*") BytePointer trackbar_name, @Cast("const char*") BytePointer window_name,
                              IntBuffer value, int count, CvTrackbarCallback2 on_change,
                              Pointer userdata/*=0*/);
public static native int cvCreateTrackbar2( @Cast("const char*") BytePointer trackbar_name, @Cast("const char*") BytePointer window_name,
                              IntBuffer value, int count, CvTrackbarCallback2 on_change);
public static native int cvCreateTrackbar2( String trackbar_name, String window_name,
                              int[] value, int count, CvTrackbarCallback2 on_change,
                              Pointer userdata/*=0*/);
public static native int cvCreateTrackbar2( String trackbar_name, String window_name,
                              int[] value, int count, CvTrackbarCallback2 on_change);

/* retrieve or set trackbar position */
public static native int cvGetTrackbarPos( @Cast("const char*") BytePointer trackbar_name, @Cast("const char*") BytePointer window_name );
public static native int cvGetTrackbarPos( String trackbar_name, String window_name );
public static native void cvSetTrackbarPos( @Cast("const char*") BytePointer trackbar_name, @Cast("const char*") BytePointer window_name, int pos );
public static native void cvSetTrackbarPos( String trackbar_name, String window_name, int pos );

/** enum  */
public static final int
    CV_EVENT_MOUSEMOVE      = 0,
    CV_EVENT_LBUTTONDOWN    = 1,
    CV_EVENT_RBUTTONDOWN    = 2,
    CV_EVENT_MBUTTONDOWN    = 3,
    CV_EVENT_LBUTTONUP      = 4,
    CV_EVENT_RBUTTONUP      = 5,
    CV_EVENT_MBUTTONUP      = 6,
    CV_EVENT_LBUTTONDBLCLK  = 7,
    CV_EVENT_RBUTTONDBLCLK  = 8,
    CV_EVENT_MBUTTONDBLCLK  = 9,
    CV_EVENT_MOUSEWHEEL     = 10,
    CV_EVENT_MOUSEHWHEEL    = 11;

/** enum  */
public static final int
    CV_EVENT_FLAG_LBUTTON   = 1,
    CV_EVENT_FLAG_RBUTTON   = 2,
    CV_EVENT_FLAG_MBUTTON   = 4,
    CV_EVENT_FLAG_CTRLKEY   = 8,
    CV_EVENT_FLAG_SHIFTKEY  = 16,
    CV_EVENT_FLAG_ALTKEY    = 32;


// #define CV_GET_WHEEL_DELTA(flags) ((short)((flags >> 16) & 0xffff)) // upper 16 bits

@Convention("CV_CDECL") public static class CvMouseCallback extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    CvMouseCallback(Pointer p) { super(p); }
    protected CvMouseCallback() { allocate(); }
    private native void allocate();
    public native void call(int event, int x, int y, int flags, Pointer param);
}

/* assign callback for mouse events */
public static native void cvSetMouseCallback( @Cast("const char*") BytePointer window_name, CvMouseCallback on_mouse,
                                Pointer param/*=NULL*/);
public static native void cvSetMouseCallback( @Cast("const char*") BytePointer window_name, CvMouseCallback on_mouse);
public static native void cvSetMouseCallback( String window_name, CvMouseCallback on_mouse,
                                Pointer param/*=NULL*/);
public static native void cvSetMouseCallback( String window_name, CvMouseCallback on_mouse);

/* wait for key event infinitely (delay<=0) or for "delay" milliseconds */
public static native int cvWaitKey(int delay/*=0*/);
public static native int cvWaitKey();

// OpenGL support

@Convention("CV_CDECL") public static class CvOpenGlDrawCallback extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    CvOpenGlDrawCallback(Pointer p) { super(p); }
    protected CvOpenGlDrawCallback() { allocate(); }
    private native void allocate();
    public native void call(Pointer userdata);
}
public static native void cvSetOpenGlDrawCallback(@Cast("const char*") BytePointer window_name, CvOpenGlDrawCallback callback, Pointer userdata/*=NULL*/);
public static native void cvSetOpenGlDrawCallback(@Cast("const char*") BytePointer window_name, CvOpenGlDrawCallback callback);
public static native void cvSetOpenGlDrawCallback(String window_name, CvOpenGlDrawCallback callback, Pointer userdata/*=NULL*/);
public static native void cvSetOpenGlDrawCallback(String window_name, CvOpenGlDrawCallback callback);

public static native void cvSetOpenGlContext(@Cast("const char*") BytePointer window_name);
public static native void cvSetOpenGlContext(String window_name);
public static native void cvUpdateWindow(@Cast("const char*") BytePointer window_name);
public static native void cvUpdateWindow(String window_name);


/****************************************************************************************\

*                              Obsolete functions/synonyms                               *
\****************************************************************************************/

public static native void cvAddSearchPath(@Cast("const char*") BytePointer path);
public static native void cvAddSearchPath(String path);
public static native int cvvInitSystem(int arg1, @Cast("char**") PointerPointer arg2);
public static native int cvvInitSystem(int arg1, @Cast("char**") @ByPtrPtr BytePointer arg2);
public static native int cvvInitSystem(int arg1, @Cast("char**") @ByPtrPtr ByteBuffer arg2);
public static native int cvvInitSystem(int arg1, @Cast("char**") @ByPtrPtr byte[] arg2);
public static native void cvvNamedWindow(@Cast("const char*") BytePointer arg1, int arg2);
public static native void cvvNamedWindow(String arg1, int arg2);
public static native void cvvShowImage(@Cast("const char*") BytePointer arg1, CvArr arg2);
public static native void cvvShowImage(String arg1, CvArr arg2);
public static native void cvvResizeWindow(@Cast("const char*") BytePointer arg1, int arg2, int arg3);
public static native void cvvResizeWindow(String arg1, int arg2, int arg3);
public static native void cvvDestroyWindow(@Cast("const char*") BytePointer arg1);
public static native void cvvDestroyWindow(String arg1);
public static native int cvvCreateTrackbar(@Cast("const char*") BytePointer arg1, @Cast("const char*") BytePointer arg2, IntPointer arg3, int arg4, CvTrackbarCallback arg5);
public static native int cvvCreateTrackbar(String arg1, String arg2, IntBuffer arg3, int arg4, CvTrackbarCallback arg5);
public static native int cvvCreateTrackbar(@Cast("const char*") BytePointer arg1, @Cast("const char*") BytePointer arg2, int[] arg3, int arg4, CvTrackbarCallback arg5);
public static native int cvvCreateTrackbar(String arg1, String arg2, IntPointer arg3, int arg4, CvTrackbarCallback arg5);
public static native int cvvCreateTrackbar(@Cast("const char*") BytePointer arg1, @Cast("const char*") BytePointer arg2, IntBuffer arg3, int arg4, CvTrackbarCallback arg5);
public static native int cvvCreateTrackbar(String arg1, String arg2, int[] arg3, int arg4, CvTrackbarCallback arg5);
public static native void cvvAddSearchPath(@Cast("const char*") BytePointer arg1);
public static native void cvvAddSearchPath(String arg1);
public static native int cvvWaitKey(@Cast("const char*") BytePointer name);
public static native int cvvWaitKey(String name);
public static native int cvvWaitKeyEx(@Cast("const char*") BytePointer name, int delay);
public static native int cvvWaitKeyEx(String name, int delay);
public static final int HG_AUTOSIZE = CV_WINDOW_AUTOSIZE;
// #define set_preprocess_func cvSetPreprocessFuncWin32
// #define set_postprocess_func cvSetPostprocessFuncWin32

// #if defined WIN32 || defined _WIN32

// #endif

// #ifdef __cplusplus
// #endif

// #endif


// Parsed from <opencv2/highgui.hpp>

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

// #ifndef __OPENCV_HIGHGUI_HPP__
// #define __OPENCV_HIGHGUI_HPP__

// #include "opencv2/core.hpp"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/videoio.hpp"


///////////////////////// graphical user interface //////////////////////////

// Flags for namedWindow
/** enum cv:: */
public static final int WINDOW_NORMAL     =  0x00000000, // the user can resize the window (no constraint) / also use to switch a fullscreen window to a normal size
       WINDOW_AUTOSIZE   =  0x00000001, // the user cannot resize the window, the size is constrainted by the image displayed
       WINDOW_OPENGL     =  0x00001000, // window with opengl support

       WINDOW_FULLSCREEN = 1,          // change the window to fullscreen
       WINDOW_FREERATIO  =  0x00000100, // the image expends as much as it can (no ratio constraint)
       WINDOW_KEEPRATIO  =  0x00000000;  // the ratio of the image is respected

// Flags for set / getWindowProperty
/** enum cv:: */
public static final int WND_PROP_FULLSCREEN   = 0, // fullscreen property    (can be WINDOW_NORMAL or WINDOW_FULLSCREEN)
       WND_PROP_AUTOSIZE     = 1, // autosize property      (can be WINDOW_NORMAL or WINDOW_AUTOSIZE)
       WND_PROP_ASPECT_RATIO = 2, // window's aspect ration (can be set to WINDOW_FREERATIO or WINDOW_KEEPRATIO);
       WND_PROP_OPENGL       = 3;  // opengl support

/** enum cv:: */
public static final int EVENT_MOUSEMOVE      = 0,
       EVENT_LBUTTONDOWN    = 1,
       EVENT_RBUTTONDOWN    = 2,
       EVENT_MBUTTONDOWN    = 3,
       EVENT_LBUTTONUP      = 4,
       EVENT_RBUTTONUP      = 5,
       EVENT_MBUTTONUP      = 6,
       EVENT_LBUTTONDBLCLK  = 7,
       EVENT_RBUTTONDBLCLK  = 8,
       EVENT_MBUTTONDBLCLK  = 9,
       EVENT_MOUSEWHEEL     = 10,
       EVENT_MOUSEHWHEEL    = 11;

/** enum cv:: */
public static final int EVENT_FLAG_LBUTTON   = 1,
       EVENT_FLAG_RBUTTON   = 2,
       EVENT_FLAG_MBUTTON   = 4,
       EVENT_FLAG_CTRLKEY   = 8,
       EVENT_FLAG_SHIFTKEY  = 16,
       EVENT_FLAG_ALTKEY    = 32;

// Qt font
/** enum cv:: */
public static final int  QT_FONT_LIGHT           = 25, //QFont::Light,
        QT_FONT_NORMAL          = 50, //QFont::Normal,
        QT_FONT_DEMIBOLD        = 63, //QFont::DemiBold,
        QT_FONT_BOLD            = 75, //QFont::Bold,
        QT_FONT_BLACK           = 87;  //QFont::Black

// Qt font style
/** enum cv:: */
public static final int  QT_STYLE_NORMAL         = 0, //QFont::StyleNormal,
        QT_STYLE_ITALIC         = 1, //QFont::StyleItalic,
        QT_STYLE_OBLIQUE        = 2;  //QFont::StyleOblique

// Qt "button" type
/** enum cv:: */
public static final int QT_PUSH_BUTTON = 0,
       QT_CHECKBOX    = 1,
       QT_RADIOBOX    = 2;


public static class MouseCallback extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    MouseCallback(Pointer p) { super(p); }
    protected MouseCallback() { allocate(); }
    private native void allocate();
    public native void call(int event, int x, int y, int flags, Pointer userdata);
}
public static class TrackbarCallback extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    TrackbarCallback(Pointer p) { super(p); }
    protected TrackbarCallback() { allocate(); }
    private native void allocate();
    public native void call(int pos, Pointer userdata);
}
public static class OpenGlDrawCallback extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    OpenGlDrawCallback(Pointer p) { super(p); }
    protected OpenGlDrawCallback() { allocate(); }
    private native void allocate();
    public native void call(Pointer userdata);
}
public static class ButtonCallback extends FunctionPointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public    ButtonCallback(Pointer p) { super(p); }
    protected ButtonCallback() { allocate(); }
    private native void allocate();
    public native void call(int state, Pointer userdata);
}


@Namespace("cv") public static native void namedWindow(@Str BytePointer winname, int flags/*=WINDOW_AUTOSIZE*/);
@Namespace("cv") public static native void namedWindow(@Str BytePointer winname);
@Namespace("cv") public static native void namedWindow(@Str String winname, int flags/*=WINDOW_AUTOSIZE*/);
@Namespace("cv") public static native void namedWindow(@Str String winname);

@Namespace("cv") public static native void destroyWindow(@Str BytePointer winname);
@Namespace("cv") public static native void destroyWindow(@Str String winname);

@Namespace("cv") public static native void destroyAllWindows();

@Namespace("cv") public static native int startWindowThread();

@Namespace("cv") public static native int waitKey(int delay/*=0*/);
@Namespace("cv") public static native int waitKey();

@Namespace("cv") public static native void imshow(@Str BytePointer winname, @ByVal Mat mat);
@Namespace("cv") public static native void imshow(@Str String winname, @ByVal Mat mat);

@Namespace("cv") public static native void resizeWindow(@Str BytePointer winname, int width, int height);
@Namespace("cv") public static native void resizeWindow(@Str String winname, int width, int height);

@Namespace("cv") public static native void moveWindow(@Str BytePointer winname, int x, int y);
@Namespace("cv") public static native void moveWindow(@Str String winname, int x, int y);

@Namespace("cv") public static native void setWindowProperty(@Str BytePointer winname, int prop_id, double prop_value);
@Namespace("cv") public static native void setWindowProperty(@Str String winname, int prop_id, double prop_value);

@Namespace("cv") public static native double getWindowProperty(@Str BytePointer winname, int prop_id);
@Namespace("cv") public static native double getWindowProperty(@Str String winname, int prop_id);

/** assigns callback for mouse events */
@Namespace("cv") public static native void setMouseCallback(@Str BytePointer winname, MouseCallback onMouse, Pointer userdata/*=0*/);
@Namespace("cv") public static native void setMouseCallback(@Str BytePointer winname, MouseCallback onMouse);
@Namespace("cv") public static native void setMouseCallback(@Str String winname, MouseCallback onMouse, Pointer userdata/*=0*/);
@Namespace("cv") public static native void setMouseCallback(@Str String winname, MouseCallback onMouse);

@Namespace("cv") public static native int getMouseWheelDelta(int flags);

@Namespace("cv") public static native int createTrackbar(@Str BytePointer trackbarname, @Str BytePointer winname,
                              IntPointer value, int count,
                              TrackbarCallback onChange/*=0*/,
                              Pointer userdata/*=0*/);
@Namespace("cv") public static native int createTrackbar(@Str BytePointer trackbarname, @Str BytePointer winname,
                              IntPointer value, int count);
@Namespace("cv") public static native int createTrackbar(@Str String trackbarname, @Str String winname,
                              IntBuffer value, int count,
                              TrackbarCallback onChange/*=0*/,
                              Pointer userdata/*=0*/);
@Namespace("cv") public static native int createTrackbar(@Str String trackbarname, @Str String winname,
                              IntBuffer value, int count);
@Namespace("cv") public static native int createTrackbar(@Str BytePointer trackbarname, @Str BytePointer winname,
                              int[] value, int count,
                              TrackbarCallback onChange/*=0*/,
                              Pointer userdata/*=0*/);
@Namespace("cv") public static native int createTrackbar(@Str BytePointer trackbarname, @Str BytePointer winname,
                              int[] value, int count);
@Namespace("cv") public static native int createTrackbar(@Str String trackbarname, @Str String winname,
                              IntPointer value, int count,
                              TrackbarCallback onChange/*=0*/,
                              Pointer userdata/*=0*/);
@Namespace("cv") public static native int createTrackbar(@Str String trackbarname, @Str String winname,
                              IntPointer value, int count);
@Namespace("cv") public static native int createTrackbar(@Str BytePointer trackbarname, @Str BytePointer winname,
                              IntBuffer value, int count,
                              TrackbarCallback onChange/*=0*/,
                              Pointer userdata/*=0*/);
@Namespace("cv") public static native int createTrackbar(@Str BytePointer trackbarname, @Str BytePointer winname,
                              IntBuffer value, int count);
@Namespace("cv") public static native int createTrackbar(@Str String trackbarname, @Str String winname,
                              int[] value, int count,
                              TrackbarCallback onChange/*=0*/,
                              Pointer userdata/*=0*/);
@Namespace("cv") public static native int createTrackbar(@Str String trackbarname, @Str String winname,
                              int[] value, int count);

@Namespace("cv") public static native int getTrackbarPos(@Str BytePointer trackbarname, @Str BytePointer winname);
@Namespace("cv") public static native int getTrackbarPos(@Str String trackbarname, @Str String winname);

@Namespace("cv") public static native void setTrackbarPos(@Str BytePointer trackbarname, @Str BytePointer winname, int pos);
@Namespace("cv") public static native void setTrackbarPos(@Str String trackbarname, @Str String winname, int pos);


// OpenGL support
@Namespace("cv") public static native void imshow(@Str BytePointer winname, @Const @ByRef Texture2D tex);
@Namespace("cv") public static native void imshow(@Str String winname, @Const @ByRef Texture2D tex);

@Namespace("cv") public static native void setOpenGlDrawCallback(@Str BytePointer winname, OpenGlDrawCallback onOpenGlDraw, Pointer userdata/*=0*/);
@Namespace("cv") public static native void setOpenGlDrawCallback(@Str BytePointer winname, OpenGlDrawCallback onOpenGlDraw);
@Namespace("cv") public static native void setOpenGlDrawCallback(@Str String winname, OpenGlDrawCallback onOpenGlDraw, Pointer userdata/*=0*/);
@Namespace("cv") public static native void setOpenGlDrawCallback(@Str String winname, OpenGlDrawCallback onOpenGlDraw);

@Namespace("cv") public static native void setOpenGlContext(@Str BytePointer winname);
@Namespace("cv") public static native void setOpenGlContext(@Str String winname);

@Namespace("cv") public static native void updateWindow(@Str BytePointer winname);
@Namespace("cv") public static native void updateWindow(@Str String winname);


// Only for Qt

@Namespace("cv") public static class QtFont extends Pointer {
    static { Loader.load(); }
    /** Default native constructor. */
    public QtFont() { allocate(); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public QtFont(int size) { allocateArray(size); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public QtFont(Pointer p) { super(p); }
    private native void allocate();
    private native void allocateArray(int size);
    @Override public QtFont position(int position) {
        return (QtFont)super.position(position);
    }

    @MemberGetter public native @Cast("const char*") BytePointer nameFont();  // Qt: nameFont
    public native @ByRef Scalar color(); public native QtFont color(Scalar color);     // Qt: ColorFont -> cvScalar(blue_component, green_component, red\_component[, alpha_component])
    public native int font_face(); public native QtFont font_face(int font_face); // Qt: bool italic
    @MemberGetter public native @Const IntPointer ascii();     // font data and metrics
    @MemberGetter public native @Const IntPointer greek();
    @MemberGetter public native @Const IntPointer cyrillic();
    public native float hscale(); public native QtFont hscale(float hscale);
    public native float vscale(); public native QtFont vscale(float vscale);
    public native float shear(); public native QtFont shear(float shear);     // slope coefficient: 0 - normal, >0 - italic
    public native int thickness(); public native QtFont thickness(int thickness); // Qt: weight
    public native float dx(); public native QtFont dx(float dx);        // horizontal interval between letters
    public native int line_type(); public native QtFont line_type(int line_type); // Qt: PointSize
}

@Namespace("cv") public static native @ByVal QtFont fontQt(@Str BytePointer nameFont, int pointSize/*=-1*/,
                         @ByVal Scalar color/*=Scalar::all(0)*/, int weight/*=QT_FONT_NORMAL*/,
                         int style/*=QT_STYLE_NORMAL*/, int spacing/*=0*/);
@Namespace("cv") public static native @ByVal QtFont fontQt(@Str BytePointer nameFont);
@Namespace("cv") public static native @ByVal QtFont fontQt(@Str String nameFont, int pointSize/*=-1*/,
                         @ByVal Scalar color/*=Scalar::all(0)*/, int weight/*=QT_FONT_NORMAL*/,
                         int style/*=QT_STYLE_NORMAL*/, int spacing/*=0*/);
@Namespace("cv") public static native @ByVal QtFont fontQt(@Str String nameFont);

@Namespace("cv") public static native void addText( @Const @ByRef Mat img, @Str BytePointer text, @ByVal Point org, @Const @ByRef QtFont font);
@Namespace("cv") public static native void addText( @Const @ByRef Mat img, @Str String text, @ByVal Point org, @Const @ByRef QtFont font);

@Namespace("cv") public static native void displayOverlay(@Str BytePointer winname, @Str BytePointer text, int delayms/*=0*/);
@Namespace("cv") public static native void displayOverlay(@Str BytePointer winname, @Str BytePointer text);
@Namespace("cv") public static native void displayOverlay(@Str String winname, @Str String text, int delayms/*=0*/);
@Namespace("cv") public static native void displayOverlay(@Str String winname, @Str String text);

@Namespace("cv") public static native void displayStatusBar(@Str BytePointer winname, @Str BytePointer text, int delayms/*=0*/);
@Namespace("cv") public static native void displayStatusBar(@Str BytePointer winname, @Str BytePointer text);
@Namespace("cv") public static native void displayStatusBar(@Str String winname, @Str String text, int delayms/*=0*/);
@Namespace("cv") public static native void displayStatusBar(@Str String winname, @Str String text);

@Namespace("cv") public static native void saveWindowParameters(@Str BytePointer windowName);
@Namespace("cv") public static native void saveWindowParameters(@Str String windowName);

@Namespace("cv") public static native void loadWindowParameters(@Str BytePointer windowName);
@Namespace("cv") public static native void loadWindowParameters(@Str String windowName);

@Namespace("cv") public static native int startLoop(Pt2Func_int_PointerPointer pt2Func, int argc, @Cast("char**") PointerPointer argv);
@Namespace("cv") public static native int startLoop(Pt2Func_int_BytePointer pt2Func, int argc, @Cast("char**") @ByPtrPtr BytePointer argv);
@Namespace("cv") public static native int startLoop(Pt2Func_int_ByteBuffer pt2Func, int argc, @Cast("char**") @ByPtrPtr ByteBuffer argv);
@Namespace("cv") public static native int startLoop(Pt2Func_int_byte__ pt2Func, int argc, @Cast("char**") @ByPtrPtr byte[] argv);

@Namespace("cv") public static native void stopLoop();

@Namespace("cv") public static native int createButton( @Str BytePointer bar_name, ButtonCallback on_change,
                             Pointer userdata/*=0*/, int type/*=QT_PUSH_BUTTON*/,
                             @Cast("bool") boolean initial_button_state/*=false*/);
@Namespace("cv") public static native int createButton( @Str BytePointer bar_name, ButtonCallback on_change);
@Namespace("cv") public static native int createButton( @Str String bar_name, ButtonCallback on_change,
                             Pointer userdata/*=0*/, int type/*=QT_PUSH_BUTTON*/,
                             @Cast("bool") boolean initial_button_state/*=false*/);
@Namespace("cv") public static native int createButton( @Str String bar_name, ButtonCallback on_change);

 // cv
// #endif


}
