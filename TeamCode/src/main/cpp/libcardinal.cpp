#include <string>
#include <map>
#include <future>
#include <thread>
#include <chrono>
#include <exception>
#include <locale>
#include <codecvt>
#include "libcardinal.h"

//We want to use a JNI that exists in include, if it exists
//Why? Personal preference, really.
#if __has_include("include/jni.h")
#include "include/jni.h"
#else
#include "jni.h"
#endif //__has_include("jni.h")

namespace libcardinal {

    void exception_check();

//The JNI environment and Java virtual machine that libcardinal will use

    JNIEnv * jnienv;
    JavaVM * javavm;

//Helper functions to set, get, and initialise the VM and environment

/*If you are using a function called from Java, use the environment that Java provides you*/
    void setenv(JNIEnv *env) {
        libcardinal::jnienv = env;
    }

    JNIEnv * getenv() {
        return libcardinal::jnienv;
    }

    void setvm(JavaVM *vm) {
        libcardinal::javavm = vm;
    }

    JavaVM * getvm() {
        return libcardinal::javavm;
    }

/*void initenv() {
    JavaVMInitArgs args;
    args.version = JNI_VERSION_1_2;
    args.nOptions = 0;
    args.ignoreUnrecognized = 1;
    JNI_CreateJavaVM(&libcardinal_javavm, (void **)&libcardinal::jnienv, (void *)(&args));
}*/

    void terminate() {
        libcardinal::javavm->DestroyJavaVM();
    }

//Java interface functions for generic usage

/*
Creates a new instance of a Java class.
id: The name of the Java class
constructor_sig: The signature of the desired constructor (always has a return type of V)
constructor_args: The arguments to the constructor
The relevant docs are at https://docs.oracle.com/javase/8/docs/technotes/guides/jni/spec/types.html#type_signatures
*/
    jobject
    new_instance(const char *id, const char *constructor_sig, const jvalue *constructor_args) {
        libcardinal::exception_check();
        jclass target = libcardinal::jnienv->FindClass(id);
        jmethodID constructor = libcardinal::jnienv->GetMethodID(target, "<init>", constructor_sig);
        return libcardinal::jnienv->NewObjectA(target, constructor, constructor_args);
    }

    jobject altenv_new_instance(JNIEnv *env, const char *id, const char *constructor_sig,
                                const jvalue *constructor_args) {
        jclass target = env->FindClass(id);
        jmethodID constructor = env->GetMethodID(target, "<init>", constructor_sig);
        return env->NewObjectA(target, constructor, constructor_args);
    }

    /*
    Creates a new instance of a Java class, but doesn't call the constructor.
    id: the name of the Java class
    */

    jobject alloc_instance(const char *id) {
        libcardinal::exception_check();
        jclass target = libcardinal::jnienv->FindClass(id);
        return libcardinal::jnienv->AllocObject(target);
    }

    jobject altenv_alloc_instance(JNIEnv *env, const char *id) {
        jclass target = env->FindClass(id);
        return env->AllocObject(target);
    }

/*
Gets the value of a field of an object
target: The instance to get the value of the field
id: The name of the field to access
object_sig: The signature of the type of the field to access
*/
    jvalue get_field(jobject target, const char *id, const char *object_sig) {
        libcardinal::exception_check();
        jfieldID field = libcardinal::jnienv->GetFieldID(libcardinal::jnienv->GetObjectClass(target),
                                                        id,
                                                        object_sig);
        jvalue output;
        std::string sig = std::string(object_sig);
        if (sig == "Z") {
            output = {.z=libcardinal::jnienv->GetBooleanField(target, field)};
        } else if (sig == "B") {
            output = {.b=libcardinal::jnienv->GetByteField(target, field)};
        } else if (sig == "C") {
            output = {.c=libcardinal::jnienv->GetCharField(target, field)};
        } else if (sig == "S") {
            output = {.s=libcardinal::jnienv->GetShortField(target, field)};
        } else if (sig == "I") {
            output = {.i=libcardinal::jnienv->GetIntField(target, field)};
        } else if (sig == "L") {
            output = {.j=libcardinal::jnienv->GetLongField(target, field)};
        } else if (sig == "F") {
            output = {.f=libcardinal::jnienv->GetFloatField(target, field)};
        } else if (sig == "D") {
            output = {.d=libcardinal::jnienv->GetDoubleField(target, field)};
        } else {
            output = {.l=libcardinal::jnienv->GetObjectField(target, field)};
        }
        return output;
    }

    jvalue altenv_get_field(JNIEnv *env, jobject target, const char *id, const char *object_sig) {
        jfieldID field = env->GetFieldID(env->GetObjectClass(target), id,
                                         object_sig);
        jvalue output;
        std::string sig = std::string(object_sig);
        if (sig == "Z") {
            output = {.z=env->GetBooleanField(target, field)};
        } else if (sig == "B") {
            output = {.b=env->GetByteField(target, field)};
        } else if (sig == "C") {
            output = {.c=env->GetCharField(target, field)};
        } else if (sig == "S") {
            output = {.s=env->GetShortField(target, field)};
        } else if (sig == "I") {
            output = {.i=env->GetIntField(target, field)};
        } else if (sig == "L") {
            output = {.j=env->GetLongField(target, field)};
        } else if (sig == "F") {
            output = {.f=env->GetFloatField(target, field)};
        } else if (sig == "D") {
            output = {.d=env->GetDoubleField(target, field)};
        } else {
            output = {.l=env->GetObjectField(target, field)};
        }
        return output;
    }

/*
Same as get_field, but if the field is static (i.e. an enum member)
*/

    jvalue get_static_field(jclass target, const char *id, const char *object_sig) {
        libcardinal::exception_check();
        jfieldID field = libcardinal::jnienv->GetStaticFieldID(target,id,object_sig);
        jvalue output;
        std::string sig = std::string(object_sig);
        if (sig == "Z") {
            output = {.z=libcardinal::jnienv->GetStaticBooleanField(target, field)};
        } else if (sig == "B") {
            output = {.b=libcardinal::jnienv->GetStaticByteField(target, field)};
        } else if (sig == "C") {
            output = {.c=libcardinal::jnienv->GetStaticCharField(target, field)};
        } else if (sig == "S") {
            output = {.s=libcardinal::jnienv->GetStaticShortField(target, field)};
        } else if (sig == "I") {
            output = {.i=libcardinal::jnienv->GetStaticIntField(target, field)};
        } else if (sig == "L") {
            output = {.j=libcardinal::jnienv->GetStaticLongField(target, field)};
        } else if (sig == "F") {
            output = {.f=libcardinal::jnienv->GetStaticFloatField(target, field)};
        } else if (sig == "D") {
            output = {.d=libcardinal::jnienv->GetStaticDoubleField(target, field)};
        } else {
            output = {.l=libcardinal::jnienv->GetStaticObjectField(target, field)};
        }
        return output;
    }

    jvalue altenv_get_static_field(JNIEnv * env, jclass target, const char *id, const char *object_sig) {
        jfieldID field = env->GetStaticFieldID(target,id,object_sig);
        jvalue output;
        std::string sig = std::string(object_sig);
        if (sig == "Z") {
            output = {.z=env->GetStaticBooleanField(target, field)};
        } else if (sig == "B") {
            output = {.b=env->GetStaticByteField(target, field)};
        } else if (sig == "C") {
            output = {.c=env->GetStaticCharField(target, field)};
        } else if (sig == "S") {
            output = {.s=env->GetStaticShortField(target, field)};
        } else if (sig == "I") {
            output = {.i=env->GetStaticIntField(target, field)};
        } else if (sig == "L") {
            output = {.j=env->GetStaticLongField(target, field)};
        } else if (sig == "F") {
            output = {.f=env->GetStaticFloatField(target, field)};
        } else if (sig == "D") {
            output = {.d=env->GetStaticDoubleField(target, field)};
        } else {
            output = {.l=env->GetStaticObjectField(target, field)};
        }
        return output;
    }

/*
Similar to get_field, but also performs postprocessing to correctly return an array
object_sig will always be prefixed with [
Probably wrong
*/
    template<typename ArrayOf>
    jarray get_array_field(jobject target, const char *id, const char *object_sig) {
        libcardinal::exception_check();
        jobject array_holder = libcardinal::get_field(target, id, object_sig).l;
        return static_cast<ArrayOf>(array_holder);
    }

    //Probably wrong
    template<typename ArrayOf>
    jarray
    altenv_get_array_field(JNIEnv *env, jobject target, const char *id, const char *object_sig) {
        jobject array_holder = libcardinal::altenv_get_field(env, target, id, object_sig).l;
        return static_cast<ArrayOf>(array_holder);
    }

/*
Calls an instance method on target
target: The instance to call the method on
name: The name of the function
sig: The signature of the function
*/
    jvalue call_instance(jobject target, const char *name, const char *sig, jvalue *args) {
        libcardinal::exception_check();
        jmethodID id = libcardinal::jnienv->GetMethodID(libcardinal::jnienv->GetObjectClass(target),
                                                       name, sig);
        char sig_last_char = sig[strlen(sig) - 1];
        char sig_second_last_char = sig[strlen(sig) - 2];
        if (sig_second_last_char == '[') {
            return {.l=libcardinal::jnienv->CallObjectMethodA(target, id, args)};
        }
        if (sig_last_char == 'Z') {
            return {.z=libcardinal::jnienv->CallBooleanMethodA(target, id, args)};
        } else if (sig_last_char == 'B') {
            return {.b=libcardinal::jnienv->CallByteMethodA(target, id, args)};
        } else if (sig_last_char == 'S') {
            return {.s=libcardinal::jnienv->CallShortMethodA(target, id, args)};
        } else if (sig_last_char == 'I') {
            return {.i=libcardinal::jnienv->CallIntMethodA(target, id, args)};
        } else if (sig_last_char == 'J') {
            return {.j=libcardinal::jnienv->CallLongMethodA(target, id, args)};
        } else if (sig_last_char == 'C') {
            return {.c=libcardinal::jnienv->CallCharMethodA(target, id, args)};
        } else if (sig_last_char == 'F') {
            return {.f=libcardinal::jnienv->CallFloatMethodA(target, id, args)};
        } else if (sig_last_char == 'D') {
            return {.d=libcardinal::jnienv->CallDoubleMethodA(target, id, args)};
        }
        return {.l=libcardinal::jnienv->NewGlobalRef(
                libcardinal::jnienv->CallObjectMethodA(target, id, args))};
    }

    jvalue altenv_call_instance(JNIEnv *env, jobject target, const char *name, const char *sig,
                                jvalue *args) {
        jmethodID id = env->GetMethodID(env->GetObjectClass(target), name, sig);
        char sig_last_char = sig[strlen(sig) - 1];
        char sig_second_last_char = sig[strlen(sig) - 2];
        if (sig_second_last_char == '[') {
            return {.l=env->CallObjectMethodA(target, id, args)};
        }
        if (sig_last_char == 'Z') {
            return {.z=env->CallBooleanMethodA(target, id, args)};
        } else if (sig_last_char == 'B') {
            return {.b=env->CallByteMethodA(target, id, args)};
        } else if (sig_last_char == 'S') {
            return {.s=env->CallShortMethodA(target, id, args)};
        } else if (sig_last_char == 'I') {
            return {.i=env->CallIntMethodA(target, id, args)};
        } else if (sig_last_char == 'J') {
            return {.j=env->CallLongMethodA(target, id, args)};
        } else if (sig_last_char == 'C') {
            return {.c=env->CallCharMethodA(target, id, args)};
        } else if (sig_last_char == 'F') {
            return {.f=env->CallFloatMethodA(target, id, args)};
        } else if (sig_last_char == 'D') {
            return {.d=env->CallDoubleMethodA(target, id, args)};
        }
        return {.l=env->NewGlobalRef(env->CallObjectMethodA(target, id, args))};
    }

//Same as call_instance, but for methods that return void (yikes the JNI is annoying)
    void call_void_instance(jobject target, const char *name, const char *sig, jvalue *args) {
        libcardinal::exception_check();
        jmethodID id = libcardinal::jnienv->GetMethodID(libcardinal::jnienv->GetObjectClass(target),
                                                       name, sig);
        return libcardinal::jnienv->CallVoidMethodA(target, id, args);
    }

    void altenv_call_void_instance(JNIEnv *env, jobject target, const char *name, const char *sig,
                                   jvalue *args) {
        jmethodID id = env->GetMethodID(env->GetObjectClass(target), name, sig);
        return env->CallVoidMethodA(target, id, args);
    }

/*
Calls a nonvirtual instance method on target
Arguments same as call_instance
Read https://docs.oracle.com/javase/8/docs/technotes/guides/jni/spec/functions.html#CallNonvirtual_type_Method_routines for details
*/
    jobject call_nonvirtual(jobject target, const char *name, const char *sig, jvalue *args) {
        libcardinal::exception_check();
        jmethodID id = libcardinal::jnienv->GetMethodID(libcardinal::jnienv->GetObjectClass(target),
                                                       name, sig);
        return libcardinal::jnienv->CallNonvirtualObjectMethodA(target,
                                                               libcardinal::jnienv->GetObjectClass(
                                                                       target), id, args);
    }

    jobject altenv_call_nonvirtual(JNIEnv *env, jobject target, const char *name, const char *sig,
                                   jvalue *args) {
        jmethodID id = env->GetMethodID(env->GetObjectClass(target), name, sig);
        return env->CallNonvirtualObjectMethodA(target, env->GetObjectClass(target), id, args);
    }

/*
Calls a static method of instance target
Arguments same as call_instance
*/
    jvalue call_static(jobject target, const char *name, const char *sig, jvalue *args) {
        libcardinal::exception_check();
        jmethodID id = libcardinal::jnienv->GetStaticMethodID(
                libcardinal::jnienv->GetObjectClass(target), name, sig);
        char sig_last_char = sig[strlen(sig) - 1];
        char sig_second_last_char = sig[strlen(sig) - 2];
        if (sig_second_last_char == '[') {
            return {.l=libcardinal::jnienv->CallObjectMethodA(libcardinal::jnienv->GetObjectClass(target), id, args)};
        }
        if (sig_last_char == 'Z') {
            return {.z=libcardinal::jnienv->CallStaticBooleanMethodA(libcardinal::jnienv->GetObjectClass(target), id, args)};
        } else if (sig_last_char == 'B') {
            return {.b=libcardinal::jnienv->CallStaticByteMethodA(libcardinal::jnienv->GetObjectClass(target), id, args)};
        } else if (sig_last_char == 'S') {
            return {.s=libcardinal::jnienv->CallStaticShortMethodA(libcardinal::jnienv->GetObjectClass(target), id, args)};
        } else if (sig_last_char == 'I') {
            return {.i=libcardinal::jnienv->CallStaticIntMethodA(libcardinal::jnienv->GetObjectClass(target), id, args)};
        } else if (sig_last_char == 'J') {
            return {.j=libcardinal::jnienv->CallStaticLongMethodA(libcardinal::jnienv->GetObjectClass(target), id, args)};
        } else if (sig_last_char == 'C') {
            return {.c=libcardinal::jnienv->CallStaticCharMethodA(libcardinal::jnienv->GetObjectClass(target), id, args)};
        } else if (sig_last_char == 'F') {
            return {.f=libcardinal::jnienv->CallStaticFloatMethodA(libcardinal::jnienv->GetObjectClass(target), id, args)};
        } else if (sig_last_char == 'D') {
            return {.d=libcardinal::jnienv->CallStaticDoubleMethodA(libcardinal::jnienv->GetObjectClass(target), id, args)};
        }
        return {.l=libcardinal::jnienv->CallStaticObjectMethodA(
                libcardinal::jnienv->GetObjectClass(target), id, args)};
    }

    jobject altenv_call_static(JNIEnv *env, jobject target, const char *name, const char *sig,
                               jvalue *args) {
        jmethodID id = env->GetStaticMethodID(env->GetObjectClass(target), name, sig);
        return env->CallStaticObjectMethodA(env->GetObjectClass(target), id, args);
    }

/*
Calls a static method of class target
Arguments same as call_instance, except it accepts a class instead of a target
*/
    jvalue call_static(jclass target, const char *name, const char *sig, jvalue *args) {
        libcardinal::exception_check();
        jmethodID id = libcardinal::jnienv->GetStaticMethodID(target, name, sig);
        char sig_last_char = sig[strlen(sig) - 1];
        char sig_second_last_char = sig[strlen(sig) - 2];
        if (sig_second_last_char == '[') {
            return {.l=libcardinal::jnienv->CallObjectMethodA(target, id, args)};
        }
        if (sig_last_char == 'Z') {
            return {.z=libcardinal::jnienv->CallStaticBooleanMethodA(target, id, args)};
        } else if (sig_last_char == 'B') {
            return {.b=libcardinal::jnienv->CallStaticByteMethodA(target, id, args)};
        } else if (sig_last_char == 'S') {
            return {.s=libcardinal::jnienv->CallStaticShortMethodA(target, id, args)};
        } else if (sig_last_char == 'I') {
            return {.i=libcardinal::jnienv->CallStaticIntMethodA(target, id, args)};
        } else if (sig_last_char == 'J') {
            return {.j=libcardinal::jnienv->CallStaticLongMethodA(target, id, args)};
        } else if (sig_last_char == 'C') {
            return {.c=libcardinal::jnienv->CallStaticCharMethodA(target, id, args)};
        } else if (sig_last_char == 'F') {
            return {.f=libcardinal::jnienv->CallStaticFloatMethodA(target, id, args)};
        } else if (sig_last_char == 'D') {
            return {.d=libcardinal::jnienv->CallStaticDoubleMethodA(target, id, args)};
        }
        return {.l=libcardinal::jnienv->CallStaticObjectMethodA(target, id, args)};
    }

    jvalue altenv_call_static(JNIEnv *env, jclass target, const char *name, const char *sig,
                               jvalue *args) {
        jmethodID id = env->GetStaticMethodID(target, name, sig);
        char sig_last_char = sig[strlen(sig) - 1];
        char sig_second_last_char = sig[strlen(sig) - 2];
        if (sig_second_last_char == '[') {
            return {.l=env->CallStaticObjectMethodA(target, id, args)};
        }
        if (sig_last_char == 'Z') {
            return {.z=env->CallStaticBooleanMethodA(target, id, args)};
        } else if (sig_last_char == 'B') {
            return {.b=env->CallStaticByteMethodA(target, id, args)};
        } else if (sig_last_char == 'S') {
            return {.s=env->CallStaticShortMethodA(target, id, args)};
        } else if (sig_last_char == 'I') {
            return {.i=env->CallStaticIntMethodA(target, id, args)};
        } else if (sig_last_char == 'J') {
            return {.j=env->CallStaticLongMethodA(target, id, args)};
        } else if (sig_last_char == 'C') {
            return {.c=env->CallStaticCharMethodA(target, id, args)};
        } else if (sig_last_char == 'F') {
            return {.f=env->CallStaticFloatMethodA(target, id, args)};
        } else if (sig_last_char == 'D') {
            return {.d=env->CallStaticDoubleMethodA(target, id, args)};
        }
        return {.l=env->NewGlobalRef(env->CallStaticObjectMethodA(target, id, args))};
    }

/*
Get name of enum object and return it
Not guaranteed to work; I may test it eventually
*/
    std::string get_enum_name(jobject target) {
        libcardinal::exception_check();
        jobject str = libcardinal::call_instance(target, "name", "()Ljava/lang/String;", nullptr).l;
        auto byteArray = (jbyteArray) (libcardinal::call_instance(str, "getBytes", "()[B",
                                                                  nullptr).l);
        size_t length = libcardinal::jnienv->GetArrayLength(byteArray);
        jbyte *pBytes = libcardinal::jnienv->GetByteArrayElements(byteArray, nullptr);
        std::string ret = std::string((char *) pBytes, length);
        libcardinal::jnienv->ReleaseByteArrayElements(byteArray, pBytes, JNI_ABORT);
        libcardinal::jnienv->DeleteLocalRef(byteArray);
        return ret;
    }

    std::string altenv_get_enum_name(JNIEnv *env, jobject target) {
        jobject str = libcardinal::altenv_call_instance(env, target, "name", "()Ljava/lang/String;",
                                                        nullptr).l;
        auto byteArray = (jbyteArray) (libcardinal::altenv_call_instance(env, str, "getBytes",
                                                                         "()[B",
                                                                         nullptr).l);
        size_t length = env->GetArrayLength(byteArray);
        jbyte *pBytes = env->GetByteArrayElements(byteArray, nullptr);
        std::string ret = std::string((char *) pBytes, length);
        env->ReleaseByteArrayElements(byteArray, pBytes, JNI_ABORT);
        env->DeleteLocalRef(byteArray);
        return ret;
    }

    class JNIException : public std::exception {
        std::runtime_error detail; //I know, I know, I know.
    public:
        explicit JNIException(jthrowable cause) : detail("") {
            jvalue message = libcardinal::call_instance(cause, "getMessage", "()Ljava/lang/String;", nullptr);
            const unsigned short * bytes = libcardinal::jnienv->GetStringChars((jstring)message.l, nullptr);
            std::u16string u16string = std::u16string((char16_t *)bytes);
            std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> converter;
            this->detail = std::runtime_error(converter.to_bytes(u16string));
        }

        const char * what() const noexcept(true) override {
            return this->detail.what();
        }
    };

//FTC-specific functions

    jobject get_telemetry(jobject opMode) {
        return libcardinal::get_field(opMode, "telemetry",
                         "Lorg/firstinspires/ftc/robotcore/external/Telemetry;").l;
    }

    jobject altenv_get_telemetry(JNIEnv *env, jobject opMode) {
        return libcardinal::altenv_get_field(env, opMode, "telemetry",
                                "Lorg/firstinspires/ftc/robotcore/external/Telemetry;").l;
    }

    jobject get_hardware_map(jobject opMode) {
        return libcardinal::get_field(opMode, "hardwareMap", "Lcom/qualcomm/robotcore/hardware/HardwareMap;").l;
    }

    jobject altenv_get_hardware_map(JNIEnv *env, jobject opMode) {
        return libcardinal::altenv_get_field(env, opMode, "hardwareMap",
                                "Lcom/qualcomm/robotcore/hardware/HardwareMap;").l;
    }

//Reports telemetry for the specified opmode
//Only accepts strings for the value
    void report_telemetry_for(jobject opMode, const std::string &key, const std::string &value) {
        jobject telemetry = libcardinal::get_telemetry(opMode);
        jvalue args[2] = {{.l=libcardinal::jnienv->NewStringUTF(key.c_str())},
                          {.l=libcardinal::jnienv->NewStringUTF(value.c_str())}};
        libcardinal::call_instance(telemetry, "addData",
                                   "(Ljava/lang/String;Ljava/lang/Object;)Lorg/firstinspires/ftc/robotcore/external/Telemetry$Item;",
                                   args);
        libcardinal::call_instance(telemetry, "update", "()Z", nullptr);
    }

    void altenv_report_telemetry_for(JNIEnv *env, jobject opMode, const std::string &key,
                                     const std::string &value) {
        jobject telemetry = libcardinal::altenv_get_telemetry(env, opMode);
        jvalue args[2] = {{.l=env->NewStringUTF(key.c_str())},
                          {.l=env->NewStringUTF(value.c_str())}};
        libcardinal::altenv_call_instance(env, telemetry, "addData",
                                          "(Ljava/lang/String;Ljava/lang/Object;)Lorg/firstinspires/ftc/robotcore/external/Telemetry$Item;",
                                          args);
        libcardinal::altenv_call_instance(env, telemetry, "update", "()Z", nullptr);
    }

    jobject get_device_from_hardware_map(jobject opMode, const std::string &id,
                                         const std::string &idClass) {
        jobject hardwareMap = libcardinal::get_hardware_map(opMode);
        jvalue args[2] = {{.l=libcardinal::jnienv->FindClass(idClass.c_str())},
                          {.l=libcardinal::jnienv->NewStringUTF(id.c_str())}};
        jobject item = libcardinal::call_instance(hardwareMap, "get",
                                                  "(Ljava/lang/Class;Ljava/lang/String;)Lcom/qualcomm/robotcore/hardware/HardwareDevice;",
                                                  args).l;
        return libcardinal::jnienv->NewGlobalRef(item);
    }

    jobject altenv_get_device_from_hardware_map(JNIEnv *env, jobject opMode, const std::string &id,
                                                const std::string &idClass) {
        jobject hardwareMap = libcardinal::altenv_get_hardware_map(env, opMode);
        jvalue args[2] = {{.l=env->FindClass(idClass.c_str())},
                          {.l=env->NewStringUTF(id.c_str())}};
        jobject item = libcardinal::altenv_call_instance(env, hardwareMap, "get",
                                                         "(Ljava/lang/Class;Ljava/lang/String;)Lcom/qualcomm/robotcore/hardware/HardwareDevice;",
                                                         args).l;
        return env->NewGlobalRef(item);
    }

    jobject get_device_from_hardware_map(jobject opMode, const std::string &id) {
        jobject hardwareMap = libcardinal::get_hardware_map(opMode);
        jvalue args[1] = {{.l=libcardinal::jnienv->NewStringUTF(id.c_str())}};
        jobject item = libcardinal::call_instance(hardwareMap, "get",
                                                  "(Ljava/lang/String;)Lcom/qualcomm/robotcore/hardware/HardwareDevice;",
                                                  args).l;
        return libcardinal::jnienv->NewGlobalRef(item);
    }

    jobject
    altenv_get_device_from_hardware_map(JNIEnv *env, jobject opMode, const std::string &id) {
        jobject hardwareMap = libcardinal::altenv_get_hardware_map(env, opMode);
        jvalue args[1] = {{.l=env->NewStringUTF(id.c_str())}};
        jobject item = libcardinal::altenv_call_instance(env, hardwareMap, "get",
                                                         "(Ljava/lang/String;)Lcom/qualcomm/robotcore/hardware/HardwareDevice;",
                                                         args).l;
        return env->NewGlobalRef(item);
    }

    /*
    A class to handle telemetry
    Useful when there are large amounts of telemetry to
    be reported, although it can't do captions
    I added it because it functions like std::cout
    */
    class TelemetryStream {
        jobject opMode;
    public:
        explicit TelemetryStream(jobject opMode) : opMode(opMode) {}
        void operator<<(const std::string& other) {
            report_telemetry_for(this->opMode, "", other);
        }
    };

    //(Internally) checks for Java exceptions and
    //throws proper C++ exceptions if one is found
    //TODO: something tells me this won't work...
    void exceptionCheck() {
        jboolean exceptionOccured = libcardinal::jnienv->ExceptionCheck();
        if (exceptionOccured == JNI_TRUE) {
            jthrowable exception = libcardinal::jnienv->ExceptionOccurred();
            libcardinal::jnienv->ExceptionClear();
            throw *(new JNIException(exception));
        }
    }
}
//Tests

/*
An example of how to interface with the LinearOpMode from C++
Notice that, in the Java file, we only have a call to System.loadLibrary()
and a declaration of runOpMode() plus some camera functions (see next
example); all the rest is done in C++
*/
extern "C" JNIEXPORT void
JNICALL Java_org_firstinspires_ftc_teamcode_libcardinal_runOpMode(JNIEnv *env, jobject thiz) {
    //Set up libcardinal's JNI environment so libcardinal works correctly
    libcardinal::setenv(env);
    //Set up a TelemetryStream
    //Not really necessary, but it's useful when there are large amounts of telemetry
    libcardinal::TelemetryStream out(thiz);
    //Call waitForStart() like a good FTC programmer
    //If the method returned, we would use call_instance
    libcardinal::call_void_instance(thiz, "waitForStart", "()V", nullptr);
    //Report some telemetry
    out << "Output from native: 5";
    //libcardinal::report_telemetry_for(thiz, "Output from native", "5");
}

/*
An example of how to interact with hardware devices from C++
(I inadvertently chose the worst one)
*/

jobject cameraCharacteristics;
jobject camera;
jobject serialThreadPool;

extern "C" JNIEXPORT void
JNICALL Java_org_firstinspires_ftc_teamcode_CameraTest_runOpMode(JNIEnv *env, jobject thiz) {
    libcardinal::setenv(env);
    jobject cameraName = libcardinal::get_device_from_hardware_map(thiz, "webcam");
    jobject cameraManager = libcardinal::call_instance(
            libcardinal::call_static(
                    env->FindClass("org/firstinspires/ftc/robotcore/external/ClassFactory"),
                    "getInstance",
                    "()Lorg/firstinspires/ftc/robotcore/external/ClassFactory;",
                    nullptr
                    ).l,
            "getCameraManager",
            "()Lorg/firstinspires/ftc/robotcore/external/hardware/camera/CameraManager;",
            nullptr
            ).l;
    jvalue cargs[2] = {
            {.j=(jlong) 5000},
            libcardinal::get_static_field(
                    env->FindClass("java/util/concurrent/TimeUnit"),
                    "MILLISECONDS",
                    "Ljava/util/concurrent/TimeUnit;"
                    )
    };
    jvalue args[3] = {
            {.l=libcardinal::new_instance(
                    "org/firstinspires/ftc/robotcore/internal/system/Deadline",
                    "(JLjava/util/concurrent/TimeUnit;)V",
                    cargs
            )},
            {.l=cameraName},
            {.l=(jobject)nullptr} //NULL i think
    };
    camera = env->NewGlobalRef(libcardinal::call_instance(
            cameraManager,
            "requestPermissionAndOpenCamera",
            "(Lorg/firstinspires/ftc/robotcore/internal/system/Deadline;Lorg/firstinspires/ftc/robotcore/external/hardware/camera/CameraName;Lorg/firstinspires/ftc/robotcore/external/function/Continuation;)Lorg/firstinspires/ftc/robotcore/external/hardware/camera/Camera;",
            args
            ).l);
    cameraCharacteristics = env->NewGlobalRef(
            libcardinal::call_instance(
                libcardinal::call_instance(
                        camera,
                        "getCameraName",
                        "()Lorg/firstinspires/ftc/robotcore/external/hardware/camera/CameraName;",
                        nullptr
                        ).l,
                "getCameraCharacteristics",
                "()Lorg/firstinspires/ftc/robotcore/external/hardware/camera/CameraCharacteristics;",
                nullptr
                ).l
            );
    libcardinal::call_void_instance(thiz, "waitForStart", "()V", nullptr);
    if (camera == nullptr) {
        libcardinal::report_telemetry_for(thiz, "Camera is", "Broken");
        return;
    }
    serialThreadPool = env->NewGlobalRef(libcardinal::call_instance(
            cameraManager,
            "getSerialThreadPool",
            "()Ljava/util/concurrent/Executor;",
            nullptr
            ).l);
    jvalue dargs[2] = {
            {.l=serialThreadPool},
            {
                .l=libcardinal::alloc_instance(
                        "org/firstinspires/ftc/teamcode/libcardinal$StateCallback"
                        )
            }
    };
    jvalue largs[1] = {
                libcardinal::call_static(
                        env->FindClass("org/firstinspires/ftc/robotcore/external/function/Continuation"),
                        "create",
                        "(Ljava/util/concurrent/Executor;Ljava/lang/Object;)Lorg/firstinspires/ftc/robotcore/external/function/Continuation;",
                        dargs
                        )
    };
    jobject session = libcardinal::call_instance(
            camera,
            "createCaptureSession",
            "(Lorg/firstinspires/ftc/robotcore/external/function/Continuation;)Lorg/firstinspires/ftc/robotcore/external/hardware/camera/CameraCaptureSession;",
            largs
            ).l;
    libcardinal::report_telemetry_for(thiz, "Status", "So far, so good...");
    std::this_thread::sleep_for(std::chrono::milliseconds(15000));
    env->DeleteGlobalRef(cameraCharacteristics);
    env->DeleteGlobalRef(camera);
    env->DeleteGlobalRef(serialThreadPool);
}

//Process a frame from the camera

extern "C" JNIEXPORT void JNICALL Java_org_firstinspires_ftc_teamcode_libcardinal_00024CaptureCallback_onNewFrame(JNIEnv * env, jobject thiz, jobject session, jobject request, jobject frame) {

}

//Set up the camera more

extern "C" JNIEXPORT void JNICALL Java_org_firstinspires_ftc_teamcode_libcardinal_00024StateCallback_onConfigured(JNIEnv * env, jobject thiz, jobject session) {
    jvalue args[1] = {
            libcardinal::altenv_get_static_field(
                    env,
                    env->FindClass("org/firstinspires/ftc/robotcore/internal/vuforia/externalprovider/FrameFormat"),
                    "YUYV",
                    "Lorg/firstinspires/ftc/robotcore/internal/vuforia/externalprovider/FrameFormat;"
            )
    };
    jvalue dargs[2] = {
            {.i=(jint)640},
            {.i=(jint)480}
    };
    jvalue cargs[2] = {
            libcardinal::altenv_call_static(
                    env,
                    env->FindClass("org/firstinspires/ftc/robotcore/internal/camera/ImageFormatMapper"),
                    "androidFromVuforiaWebcam",
                    "(Lorg/firstinspires/ftc/robotcore/internal/vuforia/externalprovider/FrameFormat;)I",
                    args
                    ),
            {
                .l=libcardinal::altenv_new_instance(
                        env,
                        "org/firstinspires/ftc/robotcore/external/android/util/Size",
                        "(II)V",
                        dargs
                        )
            }
    };
    jvalue jargs[4] = {
            {.i=(jint) 640},
            {.i=(jint) 480},
            libcardinal::altenv_call_instance(
                    env,
                    cameraCharacteristics,
                    "getMaxFramesPerSecond",
                    "(ILorg/firstinspires/ftc/robotcore/external/android/util/Size;)I",
                    cargs
                    ),
            libcardinal::altenv_get_static_field(
                    env,
                    env->FindClass("org/firstinspires/ftc/robotcore/internal/vuforia/externalprovider/FrameFormat"),
                    "YUYV",
                    "Lorg/firstinspires/ftc/robotcore/internal/vuforia/externalprovider/FrameFormat;"
            )
    };
    jobject streamingMode = libcardinal::altenv_new_instance(
            env,
            "org/firstinspires/ftc/robotcore/internal/vuforia/externalprovider/CameraMode",
            "(IIILorg/firstinspires/ftc/robotcore/internal/vuforia/externalprovider/FrameFormat;)V",
            jargs
            );
    jvalue fargs[3] = {
            libcardinal::altenv_call_instance(
                    env,
                    streamingMode,
                    "getAndroidFormat",
                    "()I",
                    nullptr
                    ),
            libcardinal::altenv_call_instance(
                    env,
                    streamingMode,
                    "getSize",
                    "()Lorg/firstinspires/ftc/robotcore/external/android/util/Size;",
                    nullptr
                    ),
            libcardinal::altenv_call_instance(
                    env,
                    streamingMode,
                    "getFramesPerSecond",
                    "()I",
                    nullptr
                    )
    };
    jobject cameraCaptureRequest = libcardinal::altenv_call_instance(
            env,
            camera,
            "createCaptureRequest",
            "(ILorg/firstinspires/ftc/robotcore/external/android/util/Size;I)Lorg/firstinspires/ftc/robotcore/external/hardware/camera/CameraCaptureRequest;",
            fargs
            ).l;
    jvalue yargs[2] {
            {.l=serialThreadPool},
            {.l=libcardinal::altenv_alloc_instance(env, "org/firstinspires/ftc/teamcode/libcardinal$StatusCallback")}
    };
    jvalue pargs[3] {
            {.l=cameraCaptureRequest},
            {.l=libcardinal::altenv_alloc_instance(env, "org/firstinspires/ftc/teamcode/libcardinal$CaptureCallback")},
            libcardinal::altenv_call_static(
                    env,
                    env->FindClass("org/firstinspires/ftc/robotcore/external/function/Continuation"),
                    "create",
                    "(Ljava/util/concurrent/Executor;Lorg/firstinspires/ftc/robotcore/external/hardware/camera/CameraCapureSession$StatusCallback;)Lorg/firstinspires/ftc/robotcore/external/function/Continuation;",
                    yargs
            )
    };
    libcardinal::altenv_call_instance(
            env,
            session,
            "startCapture",
            "(Lorg/firstinspires/ftc/robotcore/external/hardware/camera/CameraCaptureRequest;Lorg/firstinspires/ftc/robotcore/external/hardware/camera/CameraCaptureSession$CaptureCallback;Lorg/firstinspires/ftc/robotcore/external/function/Continuation;)Lorg/firstinspires/ftc/robotcore/external/hardware/camera/CameraCaptureSequenceId;",
            pargs
            );
}

/*
Notes: The JNI is very picky about how you clean up your variables.
Luckily, the JVM auto-garbage-collects after native functions return,
so for the most part, we're fine. However, if we ran a large loop
that created jobjects on each iteration, or if we got a reference
to a large Java object, among other examples, we would use
DeleteLocalRef() to avoid no-fun memory errors.

Also, if we wanted to pass jobjects between functions (which
libcardinal does with call_instance()), we have to use
env->CreateGlobalRef(<object>) and env->DeleteGlobalRef(<object>).
Read the JNI docs for details. I don't really know.


If you must make other threads, it's fine as long as you don't
call our functions. The JNIEnv is only valid in the thread it
was created in. If you absolutely must/want to call our functions,
then save the JavaVM from GetJavaVM() into our default JavaVM using
libcardinal::setvm() and then, in the other thread, get the local
env from the JavaVM using AttachCurrentThread(). We provide a
family of altenv functions that can be called using this env
instead of the default env. Note however that multithreading in
Java is impossible under the FTC and that, while multithreading
can be done in C++, it is probably against FTC rules and we do not
endorse it. (We are not legal experts, and this is not legal
advice. We haven't actually found a rule that says you can't, we
just don't think you can.) If setting our JVM, don't forget to
call libcardinal::terminate() at the end of your program.

Another reason not to use altenv functions is that they don't
do automatic exception checking before they run, so the programmer
must manually check for exceptions. This is to be fixed in a later
update, but for now, don't expect altenv functions to check for
Java errors and try to translate them into C++ errors.
TODO: add an altenv exception_check function


Now, in theory what you could do is create a C++ class that wraps a
Java class. I didn't here, but it may be a good idea for later...
*/