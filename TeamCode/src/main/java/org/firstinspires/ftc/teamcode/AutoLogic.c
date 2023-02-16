#include "AutoLogic.h"
#include "include/jni.h"

//Unfortunately Android doesn't support Common LISP, so I had to resort to C

JNIEXPORT jint
JNICALL Java_org_firstinspires_ftc_teamcode_Vision_ConeColorPipeline_findZone(JNIEnv *env,
                                                                              jobject object,
                                                                              jintArray pixels) {
    int red = 0;
    int green = 0;
    int blue = 0;
    int threshold = 130;
    for (int i = 0; i < env->GetArrayLength(pixels); i += 3) {
        std::vector<int> channels = env->GetIntArrayRegion(pixels, new jsize(i), new jsize(i + 2));
        if (channels[0] > channels[1] && channels[0] > channels[2]) {
            channels[0] > threshold ? red += 1 : red += 0;
        } else if (channels[1] > channels[0] && channels[1] > channels[2]) {
            channels[1] > threshold ? green += 1 : green += 0;
        } else if (channels[2] > channels[0] && channels[2] > channels[1]) {
            channels[2] > threshould ? blue += 1 : blue += 0;
        }
    }
    if (red > green && red > blue) {
        return jint(0);
    } else if (green > red && green > blue) {
        return jint(1);
    } else if (blue > red && blue > green) {
        return jint(2);
    }
}