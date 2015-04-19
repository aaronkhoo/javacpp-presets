// Targeted by JavaCPP version 0.11

package org.bytedeco.javacpp;

import java.nio.*;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import static org.bytedeco.javacpp.opencv_core.*;

public class opencv_ml extends org.bytedeco.javacpp.presets.opencv_ml {
    static { Loader.load(); }

@Name("std::map<std::string,int>") public static class StringIntMap extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public StringIntMap(Pointer p) { super(p); }
    public StringIntMap()       { allocate();  }
    private native void allocate();
    public native @Name("operator=") @ByRef StringIntMap put(@ByRef StringIntMap x);

    public native long size();

    @Index public native int get(@StdString BytePointer i);
    public native StringIntMap put(@StdString BytePointer i, int value);
}

// Parsed from <opencv2/ml.hpp>

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
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Copyright (C) 2014, Itseez Inc, all rights reserved.
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

// #ifndef __OPENCV_ML_HPP__
// #define __OPENCV_ML_HPP__

// #ifdef __cplusplus
// #  include "opencv2/core.hpp"
// #endif

// #ifdef __cplusplus

// #include <float.h>
// #include <map>
// #include <iostream>

/* Variable type */
/** enum cv::ml:: */
public static final int
    VAR_NUMERICAL    = 0,
    VAR_ORDERED      = 0,
    VAR_CATEGORICAL  = 1;

/** enum cv::ml:: */
public static final int
    TEST_ERROR = 0,
    TRAIN_ERROR = 1;

/** enum cv::ml:: */
public static final int
    ROW_SAMPLE = 0,
    COL_SAMPLE = 1;

@Namespace("cv::ml") @NoOffset public static class ParamGrid extends Pointer {
    static { Loader.load(); }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public ParamGrid(Pointer p) { super(p); }
    /** Native array allocator. Access with {@link Pointer#position(int)}. */
    public ParamGrid(int size) { allocateArray(size); }
    private native void allocateArray(int size);
    @Override public ParamGrid position(int position) {
        return (ParamGrid)super.position(position);
    }

    public ParamGrid() { allocate(); }
    private native void allocate();
    public ParamGrid(double _minVal, double _maxVal, double _logStep) { allocate(_minVal, _maxVal, _logStep); }
    private native void allocate(double _minVal, double _maxVal, double _logStep);

    public native double minVal(); public native ParamGrid minVal(double minVal);
    public native double maxVal(); public native ParamGrid maxVal(double maxVal);
    public native double logStep(); public native ParamGrid logStep(double logStep);
}

@Namespace("cv::ml") public static class TrainData extends Pointer {
    static { Loader.load(); }
    /** Empty constructor. */
    public TrainData() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public TrainData(Pointer p) { super(p); }

    public static native float missingValue();

    public native int getLayout();
    public native int getNTrainSamples();
    public native int getNTestSamples();
    public native int getNSamples();
    public native int getNVars();
    public native int getNAllVars();

    public native void getSample(@ByVal Mat varIdx, int sidx, FloatPointer buf);
    public native void getSample(@ByVal Mat varIdx, int sidx, FloatBuffer buf);
    public native void getSample(@ByVal Mat varIdx, int sidx, float[] buf);
    public native @ByVal Mat getSamples();
    public native @ByVal Mat getMissing();
    public native @ByVal Mat getTrainSamples(int layout/*=ROW_SAMPLE*/,
                                    @Cast("bool") boolean compressSamples/*=true*/,
                                    @Cast("bool") boolean compressVars/*=true*/);
    public native @ByVal Mat getTrainSamples();
    public native @ByVal Mat getTrainResponses();
    public native @ByVal Mat getTrainNormCatResponses();
    public native @ByVal Mat getTestResponses();
    public native @ByVal Mat getTestNormCatResponses();
    public native @ByVal Mat getResponses();
    public native @ByVal Mat getNormCatResponses();
    public native @ByVal Mat getSampleWeights();
    public native @ByVal Mat getTrainSampleWeights();
    public native @ByVal Mat getTestSampleWeights();
    public native @ByVal Mat getVarIdx();
    public native @ByVal Mat getVarType();
    public native int getResponseType();
    public native @ByVal Mat getTrainSampleIdx();
    public native @ByVal Mat getTestSampleIdx();
    public native void getValues(int vi, @ByVal Mat sidx, FloatPointer values);
    public native void getValues(int vi, @ByVal Mat sidx, FloatBuffer values);
    public native void getValues(int vi, @ByVal Mat sidx, float[] values);
    public native void getNormCatValues(int vi, @ByVal Mat sidx, IntPointer values);
    public native void getNormCatValues(int vi, @ByVal Mat sidx, IntBuffer values);
    public native void getNormCatValues(int vi, @ByVal Mat sidx, int[] values);
    public native @ByVal Mat getDefaultSubstValues();

    public native int getCatCount(int vi);
    public native @ByVal Mat getClassLabels();

    public native @ByVal Mat getCatOfs();
    public native @ByVal Mat getCatMap();

    public native void setTrainTestSplit(int count, @Cast("bool") boolean shuffle/*=true*/);
    public native void setTrainTestSplit(int count);
    public native void setTrainTestSplitRatio(double ratio, @Cast("bool") boolean shuffle/*=true*/);
    public native void setTrainTestSplitRatio(double ratio);
    public native void shuffleTrainTest();

    public static native @ByVal Mat getSubVector(@Const @ByRef Mat vec, @Const @ByRef Mat idx);
    public static native @Ptr TrainData loadFromCSV(@Str BytePointer filename,
                                          int headerLineCount,
                                          int responseStartIdx/*=-1*/,
                                          int responseEndIdx/*=-1*/,
                                          @Str BytePointer varTypeSpec/*=String()*/,
                                          @Cast("char") byte delimiter/*=','*/,
                                          @Cast("char") byte missch/*='?'*/);
    public static native @Ptr TrainData loadFromCSV(@Str BytePointer filename,
                                          int headerLineCount);
    public static native @Ptr TrainData loadFromCSV(@Str String filename,
                                          int headerLineCount,
                                          int responseStartIdx/*=-1*/,
                                          int responseEndIdx/*=-1*/,
                                          @Str String varTypeSpec/*=String()*/,
                                          @Cast("char") byte delimiter/*=','*/,
                                          @Cast("char") byte missch/*='?'*/);
    public static native @Ptr TrainData loadFromCSV(@Str String filename,
                                          int headerLineCount);
    public static native @Ptr TrainData create(@ByVal Mat samples, int layout, @ByVal Mat responses,
                                     @ByVal Mat varIdx/*=noArray()*/, @ByVal Mat sampleIdx/*=noArray()*/,
                                     @ByVal Mat sampleWeights/*=noArray()*/, @ByVal Mat varType/*=noArray()*/);
    public static native @Ptr TrainData create(@ByVal Mat samples, int layout, @ByVal Mat responses);
}


@Namespace("cv::ml") public static class StatModel extends Algorithm {
    static { Loader.load(); }
    /** Empty constructor. */
    public StatModel() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public StatModel(Pointer p) { super(p); }

    /** enum cv::ml::StatModel:: */
    public static final int UPDATE_MODEL = 1, RAW_OUTPUT= 1, COMPRESSED_INPUT= 2, PREPROCESSED_INPUT= 4;
    public native void clear();

    public native int getVarCount();

    public native @Cast("bool") boolean isTrained();
    public native @Cast("bool") boolean isClassifier();

    public native @Cast("bool") boolean train( @Ptr TrainData trainData, int flags/*=0*/ );
    public native @Cast("bool") boolean train( @Ptr TrainData trainData );
    public native @Cast("bool") boolean train( @ByVal Mat samples, int layout, @ByVal Mat responses );
    public native float calcError( @Ptr TrainData data, @Cast("bool") boolean test, @ByVal Mat resp );
    public native float predict( @ByVal Mat samples, @ByVal Mat results/*=noArray()*/, int flags/*=0*/ );
    public native float predict( @ByVal Mat samples );

    public native void save(@Str BytePointer filename);
    public native void save(@Str String filename);
    public native @Str BytePointer getDefaultModelName();
}

/****************************************************************************************\
*                                 Normal Bayes Classifier                                *
\****************************************************************************************/

/* The structure, representing the grid range of statmodel parameters.
   It is used for optimizing statmodel accuracy by varying model parameters,
   the accuracy estimate being computed by cross-validation.
   The grid is logarithmic, so <step> must be greater then 1. */

@Namespace("cv::ml") public static class NormalBayesClassifier extends StatModel {
    static { Loader.load(); }
    /** Empty constructor. */
    public NormalBayesClassifier() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public NormalBayesClassifier(Pointer p) { super(p); }

    public static class Params extends Pointer {
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
    }
    public native float predictProb( @ByVal Mat inputs, @ByVal Mat outputs,
                                   @ByVal Mat outputProbs, int flags/*=0*/ );
    public native float predictProb( @ByVal Mat inputs, @ByVal Mat outputs,
                                   @ByVal Mat outputProbs );
    public native void setParams(@Const @ByRef Params params);
    public native @ByVal Params getParams();

    public static native @Ptr NormalBayesClassifier create(@Const @ByRef Params params/*=Params()*/);
    public static native @Ptr NormalBayesClassifier create();
}

/****************************************************************************************\
*                          K-Nearest Neighbour Classifier                                *
\****************************************************************************************/

// k Nearest Neighbors
@Namespace("cv::ml") public static class KNearest extends StatModel {
    static { Loader.load(); }
    /** Empty constructor. */
    public KNearest() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public KNearest(Pointer p) { super(p); }

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
    
        public Params(int defaultK/*=10*/, @Cast("bool") boolean isclassifier_/*=true*/, int Emax_/*=INT_MAX*/, int algorithmType_/*=BRUTE_FORCE*/) { allocate(defaultK, isclassifier_, Emax_, algorithmType_); }
        private native void allocate(int defaultK/*=10*/, @Cast("bool") boolean isclassifier_/*=true*/, int Emax_/*=INT_MAX*/, int algorithmType_/*=BRUTE_FORCE*/);
        public Params() { allocate(); }
        private native void allocate();

        public native int defaultK(); public native Params defaultK(int defaultK);
        public native @Cast("bool") boolean isclassifier(); public native Params isclassifier(boolean isclassifier);
        public native int Emax(); public native Params Emax(int Emax); // for implementation with KDTree
        public native int algorithmType(); public native Params algorithmType(int algorithmType);
    }
    public native void setParams(@Const @ByRef Params p);
    public native @ByVal Params getParams();
    public native float findNearest( @ByVal Mat samples, int k,
                                   @ByVal Mat results,
                                   @ByVal Mat neighborResponses/*=noArray()*/,
                                   @ByVal Mat dist/*=noArray()*/ );
    public native float findNearest( @ByVal Mat samples, int k,
                                   @ByVal Mat results );

    /** enum cv::ml::KNearest:: */
    public static final int BRUTE_FORCE= 1, KDTREE= 2;

    public static native @Ptr KNearest create(@Const @ByRef Params params/*=Params()*/);
    public static native @Ptr KNearest create();
}

/****************************************************************************************\
*                                   Support Vector Machines                              *
\****************************************************************************************/

// SVM model
@Namespace("cv::ml") public static class SVM extends StatModel {
    static { Loader.load(); }
    /** Empty constructor. */
    public SVM() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public SVM(Pointer p) { super(p); }

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
        public Params( int svm_type, int kernel_type,
                        double degree, double gamma, double coef0,
                        double Cvalue, double nu, double p,
                        @Const @ByRef Mat classWeights, @ByVal TermCriteria termCrit ) { allocate(svm_type, kernel_type, degree, gamma, coef0, Cvalue, nu, p, classWeights, termCrit); }
        private native void allocate( int svm_type, int kernel_type,
                        double degree, double gamma, double coef0,
                        double Cvalue, double nu, double p,
                        @Const @ByRef Mat classWeights, @ByVal TermCriteria termCrit );

        public native int svmType(); public native Params svmType(int svmType);
        public native int kernelType(); public native Params kernelType(int kernelType);
        public native double gamma(); public native Params gamma(double gamma);
        public native double coef0(); public native Params coef0(double coef0);
        public native double degree(); public native Params degree(double degree);

        public native double C(); public native Params C(double C);  // for CV_SVM_C_SVC, CV_SVM_EPS_SVR and CV_SVM_NU_SVR
        public native double nu(); public native Params nu(double nu); // for CV_SVM_NU_SVC, CV_SVM_ONE_CLASS, and CV_SVM_NU_SVR
        public native double p(); public native Params p(double p); // for CV_SVM_EPS_SVR
        public native @ByRef Mat classWeights(); public native Params classWeights(Mat classWeights); // for CV_SVM_C_SVC
        public native @ByRef TermCriteria termCrit(); public native Params termCrit(TermCriteria termCrit); // termination criteria
    }

    public static class Kernel extends Algorithm {
        static { Loader.load(); }
        /** Empty constructor. */
        public Kernel() { }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public Kernel(Pointer p) { super(p); }
    
        public native int getType();
        public native void calc( int vcount, int n, @Const FloatPointer vecs, @Const FloatPointer another, FloatPointer results );
        public native void calc( int vcount, int n, @Const FloatBuffer vecs, @Const FloatBuffer another, FloatBuffer results );
        public native void calc( int vcount, int n, @Const float[] vecs, @Const float[] another, float[] results );
    }

    // SVM type
    /** enum cv::ml::SVM:: */
    public static final int C_SVC= 100, NU_SVC= 101, ONE_CLASS= 102, EPS_SVR= 103, NU_SVR= 104;

    // SVM kernel type
    /** enum cv::ml::SVM:: */
    public static final int CUSTOM= -1, LINEAR= 0, POLY= 1, RBF= 2, SIGMOID= 3, CHI2= 4, INTER= 5;

    // SVM params type
    /** enum cv::ml::SVM:: */
    public static final int C= 0, GAMMA= 1, P= 2, NU= 3, COEF= 4, DEGREE= 5;

    public native @Cast("bool") boolean trainAuto( @Ptr TrainData data, int kFold/*=10*/,
                        @ByVal ParamGrid Cgrid/*=SVM::getDefaultGrid(SVM::C)*/,
                        @ByVal ParamGrid gammaGrid/*=SVM::getDefaultGrid(SVM::GAMMA)*/,
                        @ByVal ParamGrid pGrid/*=SVM::getDefaultGrid(SVM::P)*/,
                        @ByVal ParamGrid nuGrid/*=SVM::getDefaultGrid(SVM::NU)*/,
                        @ByVal ParamGrid coeffGrid/*=SVM::getDefaultGrid(SVM::COEF)*/,
                        @ByVal ParamGrid degreeGrid/*=SVM::getDefaultGrid(SVM::DEGREE)*/,
                        @Cast("bool") boolean balanced/*=false*/);
    public native @Cast("bool") boolean trainAuto( @Ptr TrainData data);

    public native @ByVal Mat getSupportVectors();

    public native void setParams(@Const @ByRef Params p, @Ptr Kernel customKernel/*=Ptr<Kernel>()*/);
    public native void setParams(@Const @ByRef Params p);
    public native @ByVal Params getParams();
    public native @Ptr Kernel getKernel();
    public native double getDecisionFunction(int i, @ByVal Mat alpha, @ByVal Mat svidx);

    public static native @ByVal ParamGrid getDefaultGrid( int param_id );
    public static native @Ptr SVM create(@Const @ByRef Params p/*=Params()*/, @Ptr Kernel customKernel/*=Ptr<Kernel>()*/);
    public static native @Ptr SVM create();
}

/****************************************************************************************\
*                              Expectation - Maximization                                *
\****************************************************************************************/
@Namespace("cv::ml") public static class EM extends StatModel {
    static { Loader.load(); }
    /** Empty constructor. */
    public EM() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public EM(Pointer p) { super(p); }

    // Type of covariation matrices
    /** enum cv::ml::EM:: */
    public static final int COV_MAT_SPHERICAL= 0, COV_MAT_DIAGONAL= 1, COV_MAT_GENERIC= 2, COV_MAT_DEFAULT= COV_MAT_DIAGONAL;

    // Default parameters
    /** enum cv::ml::EM:: */
    public static final int DEFAULT_NCLUSTERS= 5, DEFAULT_MAX_ITERS= 100;

    // The initial step
    /** enum cv::ml::EM:: */
    public static final int START_E_STEP= 1, START_M_STEP= 2, START_AUTO_STEP= 0;

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
    
        public Params(int nclusters/*=DEFAULT_NCLUSTERS*/, int covMatType/*=EM::COV_MAT_DIAGONAL*/,
                                @Const @ByRef TermCriteria termCrit/*=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
                                                                          EM::DEFAULT_MAX_ITERS, 1e-6)*/) { allocate(nclusters, covMatType, termCrit); }
        private native void allocate(int nclusters/*=DEFAULT_NCLUSTERS*/, int covMatType/*=EM::COV_MAT_DIAGONAL*/,
                                @Const @ByRef TermCriteria termCrit/*=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
                                                                          EM::DEFAULT_MAX_ITERS, 1e-6)*/);
        public Params() { allocate(); }
        private native void allocate();
        public native int nclusters(); public native Params nclusters(int nclusters);
        public native int covMatType(); public native Params covMatType(int covMatType);
        public native @ByRef TermCriteria termCrit(); public native Params termCrit(TermCriteria termCrit);
    }

    public native void setParams(@Const @ByRef Params p);
    public native @ByVal Params getParams();
    public native @ByVal Mat getWeights();
    public native @ByVal Mat getMeans();
    public native void getCovs(@ByRef MatVector covs);

    public native @ByVal Point2d predict2(@ByVal Mat sample, @ByVal Mat probs);

    public native @Cast("bool") boolean train( @Ptr TrainData trainData, int flags/*=0*/ );
    public native @Cast("bool") boolean train( @Ptr TrainData trainData );

    public static native @Ptr EM train(@ByVal Mat samples,
                              @ByVal Mat logLikelihoods/*=noArray()*/,
                              @ByVal Mat labels/*=noArray()*/,
                              @ByVal Mat probs/*=noArray()*/,
                              @Const @ByRef Params params/*=Params()*/);
    public static native @Ptr EM train(@ByVal Mat samples);

    public static native @Ptr EM train_startWithE(@ByVal Mat samples, @ByVal Mat means0,
                                         @ByVal Mat covs0/*=noArray()*/,
                                         @ByVal Mat weights0/*=noArray()*/,
                                         @ByVal Mat logLikelihoods/*=noArray()*/,
                                         @ByVal Mat labels/*=noArray()*/,
                                         @ByVal Mat probs/*=noArray()*/,
                                         @Const @ByRef Params params/*=Params()*/);
    public static native @Ptr EM train_startWithE(@ByVal Mat samples, @ByVal Mat means0);

    public static native @Ptr EM train_startWithM(@ByVal Mat samples, @ByVal Mat probs0,
                                         @ByVal Mat logLikelihoods/*=noArray()*/,
                                         @ByVal Mat labels/*=noArray()*/,
                                         @ByVal Mat probs/*=noArray()*/,
                                         @Const @ByRef Params params/*=Params()*/);
    public static native @Ptr EM train_startWithM(@ByVal Mat samples, @ByVal Mat probs0);
    public static native @Ptr EM create(@Const @ByRef Params params/*=Params()*/);
    public static native @Ptr EM create();
}


/****************************************************************************************\
*                                      Decision Tree                                     *
\****************************************************************************************/

@Namespace("cv::ml") public static class DTrees extends StatModel {
    static { Loader.load(); }
    /** Empty constructor. */
    public DTrees() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public DTrees(Pointer p) { super(p); }

    /** enum cv::ml::DTrees:: */
    public static final int PREDICT_AUTO= 0, PREDICT_SUM= (1<<8), PREDICT_MAX_VOTE= (2<<8), PREDICT_MASK= (3<<8);

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
        public Params( int maxDepth, int minSampleCount,
                       double regressionAccuracy, @Cast("bool") boolean useSurrogates,
                       int maxCategories, int CVFolds,
                       @Cast("bool") boolean use1SERule, @Cast("bool") boolean truncatePrunedTree,
                       @Const @ByRef Mat priors ) { allocate(maxDepth, minSampleCount, regressionAccuracy, useSurrogates, maxCategories, CVFolds, use1SERule, truncatePrunedTree, priors); }
        private native void allocate( int maxDepth, int minSampleCount,
                       double regressionAccuracy, @Cast("bool") boolean useSurrogates,
                       int maxCategories, int CVFolds,
                       @Cast("bool") boolean use1SERule, @Cast("bool") boolean truncatePrunedTree,
                       @Const @ByRef Mat priors );

        public native int maxCategories(); public native Params maxCategories(int maxCategories);
        public native int maxDepth(); public native Params maxDepth(int maxDepth);
        public native int minSampleCount(); public native Params minSampleCount(int minSampleCount);
        public native int CVFolds(); public native Params CVFolds(int CVFolds);
        public native @Cast("bool") boolean useSurrogates(); public native Params useSurrogates(boolean useSurrogates);
        public native @Cast("bool") boolean use1SERule(); public native Params use1SERule(boolean use1SERule);
        public native @Cast("bool") boolean truncatePrunedTree(); public native Params truncatePrunedTree(boolean truncatePrunedTree);
        public native float regressionAccuracy(); public native Params regressionAccuracy(float regressionAccuracy);
        public native @ByRef Mat priors(); public native Params priors(Mat priors);
    }

    @NoOffset public static class Node extends Pointer {
        static { Loader.load(); }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public Node(Pointer p) { super(p); }
        /** Native array allocator. Access with {@link Pointer#position(int)}. */
        public Node(int size) { allocateArray(size); }
        private native void allocateArray(int size);
        @Override public Node position(int position) {
            return (Node)super.position(position);
        }
    
        public Node() { allocate(); }
        private native void allocate();
        public native double value(); public native Node value(double value);
        public native int classIdx(); public native Node classIdx(int classIdx);

        public native int parent(); public native Node parent(int parent);
        public native int left(); public native Node left(int left);
        public native int right(); public native Node right(int right);
        public native int defaultDir(); public native Node defaultDir(int defaultDir);

        public native int split(); public native Node split(int split);
    }

    @NoOffset public static class Split extends Pointer {
        static { Loader.load(); }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public Split(Pointer p) { super(p); }
        /** Native array allocator. Access with {@link Pointer#position(int)}. */
        public Split(int size) { allocateArray(size); }
        private native void allocateArray(int size);
        @Override public Split position(int position) {
            return (Split)super.position(position);
        }
    
        public Split() { allocate(); }
        private native void allocate();
        public native int varIdx(); public native Split varIdx(int varIdx);
        public native @Cast("bool") boolean inversed(); public native Split inversed(boolean inversed);
        public native float quality(); public native Split quality(float quality);
        public native int next(); public native Split next(int next);
        public native float c(); public native Split c(float c);
        public native int subsetOfs(); public native Split subsetOfs(int subsetOfs);
    }

    public native void setDParams(@Const @ByRef Params p);
    public native @ByVal Params getDParams();

    public native @StdVector IntPointer getRoots();
    public native @StdVector Node getNodes();
    public native @StdVector Split getSplits();
    public native @StdVector IntPointer getSubsets();

    public static native @Ptr DTrees create(@Const @ByRef Params params/*=Params()*/);
    public static native @Ptr DTrees create();
}

/****************************************************************************************\
*                                   Random Trees Classifier                              *
\****************************************************************************************/

@Namespace("cv::ml") public static class RTrees extends DTrees {
    static { Loader.load(); }
    /** Empty constructor. */
    public RTrees() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public RTrees(Pointer p) { super(p); }

    @NoOffset public static class Params extends DTrees.Params {
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
        public Params( int maxDepth, int minSampleCount,
                        double regressionAccuracy, @Cast("bool") boolean useSurrogates,
                        int maxCategories, @Const @ByRef Mat priors,
                        @Cast("bool") boolean calcVarImportance, int nactiveVars,
                        @ByVal TermCriteria termCrit ) { allocate(maxDepth, minSampleCount, regressionAccuracy, useSurrogates, maxCategories, priors, calcVarImportance, nactiveVars, termCrit); }
        private native void allocate( int maxDepth, int minSampleCount,
                        double regressionAccuracy, @Cast("bool") boolean useSurrogates,
                        int maxCategories, @Const @ByRef Mat priors,
                        @Cast("bool") boolean calcVarImportance, int nactiveVars,
                        @ByVal TermCriteria termCrit );

        public native @Cast("bool") boolean calcVarImportance(); public native Params calcVarImportance(boolean calcVarImportance); // true <=> RF processes variable importance
        public native int nactiveVars(); public native Params nactiveVars(int nactiveVars);
        public native @ByRef TermCriteria termCrit(); public native Params termCrit(TermCriteria termCrit);
    }

    public native void setRParams(@Const @ByRef Params p);
    public native @ByVal Params getRParams();

    public native @ByVal Mat getVarImportance();

    public static native @Ptr RTrees create(@Const @ByRef Params params/*=Params()*/);
    public static native @Ptr RTrees create();
}

/****************************************************************************************\
*                                   Boosted tree classifier                              *
\****************************************************************************************/

@Namespace("cv::ml") public static class Boost extends DTrees {
    static { Loader.load(); }
    /** Empty constructor. */
    public Boost() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public Boost(Pointer p) { super(p); }

    @NoOffset public static class Params extends DTrees.Params {
        static { Loader.load(); }
        /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
        public Params(Pointer p) { super(p); }
        /** Native array allocator. Access with {@link Pointer#position(int)}. */
        public Params(int size) { allocateArray(size); }
        private native void allocateArray(int size);
        @Override public Params position(int position) {
            return (Params)super.position(position);
        }
    
        public native int boostType(); public native Params boostType(int boostType);
        public native int weakCount(); public native Params weakCount(int weakCount);
        public native double weightTrimRate(); public native Params weightTrimRate(double weightTrimRate);

        public Params() { allocate(); }
        private native void allocate();
        public Params( int boostType, int weakCount, double weightTrimRate,
                        int maxDepth, @Cast("bool") boolean useSurrogates, @Const @ByRef Mat priors ) { allocate(boostType, weakCount, weightTrimRate, maxDepth, useSurrogates, priors); }
        private native void allocate( int boostType, int weakCount, double weightTrimRate,
                        int maxDepth, @Cast("bool") boolean useSurrogates, @Const @ByRef Mat priors );
    }

    // Boosting type
    /** enum cv::ml::Boost:: */
    public static final int DISCRETE= 0, REAL= 1, LOGIT= 2, GENTLE= 3;

    public native @ByVal Params getBParams();
    public native void setBParams(@Const @ByRef Params p);

    public static native @Ptr Boost create(@Const @ByRef Params params/*=Params()*/);
    public static native @Ptr Boost create();
}

/****************************************************************************************\
*                                   Gradient Boosted Trees                               *
\****************************************************************************************/

/*class CV_EXPORTS_W GBTrees : public DTrees
{
public:
    struct CV_EXPORTS_W_MAP Params : public DTrees::Params
    {
        CV_PROP_RW int weakCount;
        CV_PROP_RW int lossFunctionType;
        CV_PROP_RW float subsamplePortion;
        CV_PROP_RW float shrinkage;

        Params();
        Params( int lossFunctionType, int weakCount, float shrinkage,
                float subsamplePortion, int maxDepth, bool useSurrogates );
    };

    enum {SQUARED_LOSS=0, ABSOLUTE_LOSS, HUBER_LOSS=3, DEVIANCE_LOSS};

    virtual void setK(int k) = 0;

    virtual float predictSerial( InputArray samples,
                                 OutputArray weakResponses, int flags) const = 0;

    static Ptr<GBTrees> create(const Params& p);
};*/

/****************************************************************************************\
*                              Artificial Neural Networks (ANN)                          *
\****************************************************************************************/

/////////////////////////////////// Multi-Layer Perceptrons //////////////////////////////

@Namespace("cv::ml") public static class ANN_MLP extends StatModel {
    static { Loader.load(); }
    /** Empty constructor. */
    public ANN_MLP() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public ANN_MLP(Pointer p) { super(p); }

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
        public Params( @Const @ByRef Mat layerSizes, int activateFunc, double fparam1, double fparam2,
                        @ByVal TermCriteria termCrit, int trainMethod, double param1, double param2/*=0*/ ) { allocate(layerSizes, activateFunc, fparam1, fparam2, termCrit, trainMethod, param1, param2); }
        private native void allocate( @Const @ByRef Mat layerSizes, int activateFunc, double fparam1, double fparam2,
                        @ByVal TermCriteria termCrit, int trainMethod, double param1, double param2/*=0*/ );
        public Params( @Const @ByRef Mat layerSizes, int activateFunc, double fparam1, double fparam2,
                        @ByVal TermCriteria termCrit, int trainMethod, double param1 ) { allocate(layerSizes, activateFunc, fparam1, fparam2, termCrit, trainMethod, param1); }
        private native void allocate( @Const @ByRef Mat layerSizes, int activateFunc, double fparam1, double fparam2,
                        @ByVal TermCriteria termCrit, int trainMethod, double param1 );

        /** enum cv::ml::ANN_MLP::Params:: */
        public static final int BACKPROP= 0, RPROP= 1;

        public native @ByRef Mat layerSizes(); public native Params layerSizes(Mat layerSizes);
        public native int activateFunc(); public native Params activateFunc(int activateFunc);
        public native double fparam1(); public native Params fparam1(double fparam1);
        public native double fparam2(); public native Params fparam2(double fparam2);

        public native @ByRef TermCriteria termCrit(); public native Params termCrit(TermCriteria termCrit);
        public native int trainMethod(); public native Params trainMethod(int trainMethod);

        // backpropagation parameters
        public native double bpDWScale(); public native Params bpDWScale(double bpDWScale);
        public native double bpMomentScale(); public native Params bpMomentScale(double bpMomentScale);

        // rprop parameters
        public native double rpDW0(); public native Params rpDW0(double rpDW0);
        public native double rpDWPlus(); public native Params rpDWPlus(double rpDWPlus);
        public native double rpDWMinus(); public native Params rpDWMinus(double rpDWMinus);
        public native double rpDWMin(); public native Params rpDWMin(double rpDWMin);
        public native double rpDWMax(); public native Params rpDWMax(double rpDWMax);
    }

    // possible activation functions
    /** enum cv::ml::ANN_MLP:: */
    public static final int IDENTITY = 0, SIGMOID_SYM = 1, GAUSSIAN = 2;

    // available training flags
    /** enum cv::ml::ANN_MLP:: */
    public static final int UPDATE_WEIGHTS = 1, NO_INPUT_SCALE = 2, NO_OUTPUT_SCALE = 4;

    public native @ByVal Mat getWeights(int layerIdx);
    public native void setParams(@Const @ByRef Params p);
    public native @ByVal Params getParams();

    public static native @Ptr ANN_MLP create(@Const @ByRef Params params/*=Params()*/);
    public static native @Ptr ANN_MLP create();
}

/****************************************************************************************\
*                           Logistic Regression                                          *
\****************************************************************************************/

@Namespace("cv::ml") public static class LogisticRegression extends StatModel {
    static { Loader.load(); }
    /** Empty constructor. */
    public LogisticRegression() { }
    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public LogisticRegression(Pointer p) { super(p); }

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
    
        public Params(double learning_rate/*=0.001*/,
                       int iters/*=1000*/,
                       int method/*=LogisticRegression::BATCH*/,
                       int normlization/*=LogisticRegression::REG_L2*/,
                       int reg/*=1*/,
                       int batch_size/*=1*/) { allocate(learning_rate, iters, method, normlization, reg, batch_size); }
        private native void allocate(double learning_rate/*=0.001*/,
                       int iters/*=1000*/,
                       int method/*=LogisticRegression::BATCH*/,
                       int normlization/*=LogisticRegression::REG_L2*/,
                       int reg/*=1*/,
                       int batch_size/*=1*/);
        public Params() { allocate(); }
        private native void allocate();
        public native double alpha(); public native Params alpha(double alpha);
        public native int num_iters(); public native Params num_iters(int num_iters);
        public native int norm(); public native Params norm(int norm);
        public native int regularized(); public native Params regularized(int regularized);
        public native int train_method(); public native Params train_method(int train_method);
        public native int mini_batch_size(); public native Params mini_batch_size(int mini_batch_size);
        public native @ByRef TermCriteria term_crit(); public native Params term_crit(TermCriteria term_crit);
    }

    /** enum cv::ml::LogisticRegression:: */
    public static final int REG_L1 = 0, REG_L2 = 1;
    /** enum cv::ml::LogisticRegression:: */
    public static final int BATCH = 0, MINI_BATCH = 1;

    // Algorithm interface
    public native void write( @ByRef FileStorage fs );
    public native void read( @Const @ByRef FileNode fn );

    // StatModel interface
    public native @Cast("bool") boolean train( @Ptr TrainData trainData, int flags/*=0*/ );
    public native @Cast("bool") boolean train( @Ptr TrainData trainData );
    public native float predict( @ByVal Mat samples, @ByVal Mat results/*=noArray()*/, int flags/*=0*/ );
    public native float predict( @ByVal Mat samples );
    public native void clear();

    public native @ByVal Mat get_learnt_thetas();

    public static native @Ptr LogisticRegression create( @Const @ByRef Params params/*=Params()*/ );
    public static native @Ptr LogisticRegression create( );
}

/****************************************************************************************\
*                           Auxilary functions declarations                              *
\****************************************************************************************/

/* Generates <sample> from multivariate normal distribution, where <mean> - is an
   average row vector, <cov> - symmetric covariation matrix */
@Namespace("cv::ml") public static native void randMVNormal( @ByVal Mat mean, @ByVal Mat cov, int nsamples, @ByVal Mat samples);

/* Generates sample from gaussian mixture distribution */
@Namespace("cv::ml") public static native void randGaussMixture( @ByVal Mat means, @ByVal Mat covs, @ByVal Mat weights,
                                  int nsamples, @ByVal Mat samples, @ByVal Mat sampClasses );

/* creates test set */
@Namespace("cv::ml") public static native void createConcentricSpheresTestSet( int nsamples, int nfeatures, int nclasses,
                                                @ByVal Mat samples, @ByVal Mat responses);




// #endif // __cplusplus
// #endif // __OPENCV_ML_HPP__

/* End of file. */


}
