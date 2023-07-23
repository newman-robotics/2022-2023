//Not sure what this is supposed to do, but I'll not get rid of it for now
#ifndef INC_2022_2023_FTCLIB_OPENCV_INTEGRATION_LIBCARDINAL_H
#define INC_2022_2023_FTCLIB_OPENCV_INTEGRATION_LIBCARDINAL_H
#endif //INC_2022_2023_FTCLIB_OPENCV_INTEGRATION_LIBCARDINAL_H

#if __has_include("include/jni.h")
#include "include/jni.h"
#else
#include "jni.h"
#endif //__has_include("jni.h")

//Header file for libcardinal.cpp; see everything there

namespace libcardinal {

    class TelemetryStream;

    void exception_check();

    void setenv(JNIEnv *);

    JNIEnv *getenv();

    void setvm(JavaVM *);

    JavaVM *getvm();

    void terminate();

    jobject new_instance(const char *, const char *, const jvalue *);

    jobject alloc_instance(const char *);

    jvalue get_field(jobject, const char *, const char *);

    template<typename>
    jarray get_array_field(jobject, const char *, const char *);

    jvalue call_instance(jobject, const char *, const char *, jvalue *);

    void call_void_instance(jobject, const char *, const char *, jvalue *);

    jobject call_nonvirtual(jobject, const char *, const char *, jvalue *);

    jvalue call_static(jobject, const char *, const char *, jvalue *);

    jvalue call_static(jclass, const char *, const char *, jvalue *);

    std::string get_enum_name(jobject);

    jobject get_telemetry(jobject);

    jobject get_hardware_map(jobject);

    void report_telemetry_for(jobject, const std::string &, const std::string &);

    jobject get_device_from_hardware_map(jobject, const std::string &, const std::string &);

    jobject altenv_new_instance(JNIEnv *, const char *, const char *, const jvalue *);

    jobject altenv_alloc_instance(JNIEnv *, const char *);

    jvalue altenv_get_field(JNIEnv *, jobject, const char *, const char *);

    template<typename>
    jarray altenv_get_array_field(JNIEnv *, jobject, const char *, const char *);

    jvalue altenv_call_instance(JNIEnv *, jobject, const char *, const char *, jvalue *);

    void altenv_call_void_instance(JNIEnv *, jobject, const char *, const char *, jvalue *);

    jobject altenv_call_nonvirtual(JNIEnv *, jobject, const char *, const char *, jvalue *);

    jobject altenv_call_static(JNIEnv *, jobject, const char *, const char *, jvalue *);

    jvalue altenv_call_static(JNIEnv *, jclass, const char *, const char *, jvalue *);

    std::string altenv_get_enum_name(JNIEnv *, jobject);

    jobject altenv_get_telemetry(JNIEnv *, jobject);

    jobject altenv_get_hardware_map(JNIEnv *, jobject);

    void altenv_report_telemetry_for(JNIEnv *, jobject, const std::string &, const std::string &);

    jobject altenv_get_device_from_hardware_map(JNIEnv *, jobject, const std::string &,
                                                const std::string &);

}