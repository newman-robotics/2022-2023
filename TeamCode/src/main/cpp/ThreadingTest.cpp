#include <future>
#include <atomic>
#include "include/jni.h"

//In case you're wondering if this pseudo-threading-futures-concurrency stuff works, it does

std::atomic<int> aint;

void foo() {
    for (int i = 0; i < 1000; i++) ++aint;
}

extern "C" JNIEXPORT jint JNICALL Java_org_firstinspires_ftc_teamcode_ThreadingTest_foot(JNIEnv * env, jobject thiz) {
    std::future<void> threads[10];
    for (auto & thread : threads) {
        thread = std::async(&foo);
    }
    for (auto & thread : threads) {
        thread.wait();
    }
    return (jint)aint.load();
}