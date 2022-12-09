//
// This file is auto-generated. Please don't modify it!
//
package org.opencv.aruco;

import org.opencv.aruco.EstimateParameters;

// C++: class EstimateParameters
/**
 * @brief
 * Pose estimation parameters
 * pattern Defines center this system and axes direction (default PatternPos::CCW_center).
 * useExtrinsicGuess Parameter used for SOLVEPNP_ITERATIVE. If true (1), the function uses the provided
 * rvec and tvec values as initial approximations of the rotation and translation vectors, respectively, and further
 * optimizes them (default false).
 * solvePnPMethod Method for solving a PnP problem: see REF: calib3d_solvePnP_flags (default SOLVEPNP_ITERATIVE).
 * SEE: PatternPos, solvePnP(), REF: tutorial_aruco_detection
 */
public class EstimateParameters {

    protected final long nativeObj;
    protected EstimateParameters(long addr) { nativeObj = addr; }

    public long getNativeObjAddr() { return nativeObj; }

    // internal usage only
    public static EstimateParameters __fromPtr__(long addr) { return new EstimateParameters(addr); }

    //
    // C++: static Ptr_EstimateParameters cv::aruco::EstimateParameters::create()
    //

    public static EstimateParameters create() {
        return EstimateParameters.__fromPtr__(create_0());
    }


    //
    // C++: PatternPos EstimateParameters::pattern
    //

    // Return type 'PatternPos' is not supported, skipping the function


    //
    // C++: void EstimateParameters::pattern
    //

    // Unknown type 'PatternPos' (I), skipping the function


    //
    // C++: bool EstimateParameters::useExtrinsicGuess
    //

    public boolean get_useExtrinsicGuess() {
        return get_useExtrinsicGuess_0(nativeObj);
    }


    //
    // C++: void EstimateParameters::useExtrinsicGuess
    //

    public void set_useExtrinsicGuess(boolean useExtrinsicGuess) {
        set_useExtrinsicGuess_0(nativeObj, useExtrinsicGuess);
    }


    //
    // C++: SolvePnPMethod EstimateParameters::solvePnPMethod
    //

    public int get_solvePnPMethod() {
        return get_solvePnPMethod_0(nativeObj);
    }


    //
    // C++: void EstimateParameters::solvePnPMethod
    //

    public void set_solvePnPMethod(int solvePnPMethod) {
        set_solvePnPMethod_0(nativeObj, solvePnPMethod);
    }


    @Override
    protected void finalize() throws Throwable {
        delete(nativeObj);
    }



    // C++: static Ptr_EstimateParameters cv::aruco::EstimateParameters::create()
    private static native long create_0();

    // C++: bool EstimateParameters::useExtrinsicGuess
    private static native boolean get_useExtrinsicGuess_0(long nativeObj);

    // C++: void EstimateParameters::useExtrinsicGuess
    private static native void set_useExtrinsicGuess_0(long nativeObj, boolean useExtrinsicGuess);

    // C++: SolvePnPMethod EstimateParameters::solvePnPMethod
    private static native int get_solvePnPMethod_0(long nativeObj);

    // C++: void EstimateParameters::solvePnPMethod
    private static native void set_solvePnPMethod_0(long nativeObj, int solvePnPMethod);

    // native support for java finalize()
    private static native void delete(long nativeObj);

}
