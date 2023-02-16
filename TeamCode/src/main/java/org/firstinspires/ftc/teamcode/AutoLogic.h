//
// Created by Newman on 16/02/2023.
//

#ifndef INC_2022_2023_FTCLIB_OPENCV_INTEGRATION_AUTOLOGIC_H
#define INC_2022_2023_FTCLIB_OPENCV_INTEGRATION_AUTOLOGIC_H
#include "include/jni.h"

extern "C" /*this should be unnecessary*/ {
JNIEXPORT jint
JNICALL Java_org_firstinspires_ftc_teamcode_Vision_ConeColorPipeline_findZone(JNIEnv *env,
                                                                              jobject object,
                                                                              jintArray pixels)
}

#endif //INC_2022_2023_FTCLIB_OPENCV_INTEGRATION_AUTOLOGIC_H
