#include <jni.h>
#include <string>
#include "openMVG/image/image_io.hpp"
//#include "openMVG/image/image_concat.hpp"
//#include "openMVG/features/akaze/image_describer_akaze.hpp"
//#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp"
//#include "openMVG/features/svg_features.hpp"
//#include "openMVG/matching/regions_matcher.hpp"
//#include "openMVG/matching/svg_matches.hpp"

#include <android/log.h>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>

#define  TAG "Native"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO, TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, TAG,__VA_ARGS__)

extern "C" JNIEXPORT jstring

JNICALL
Java_com_local_a3d_1reconstruction_1poc_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}
extern "C"
JNIEXPORT void JNICALL
Java_com_local_a3d_1reconstruction_1poc_Reconstructor_matchNative(JNIEnv *env, jclass type,
                                                                  jobject assetManager, jstring a_,
                                                                  jstring b_) {
    const char *a = env->GetStringUTFChars(a_, 0);
    const char *b = env->GetStringUTFChars(b_, 0);

    AAssetManager *mgr = AAssetManager_fromJava(env, assetManager);
    if (mgr == NULL) {
        LOGE("Failed to load asset manager");
    }

    AAsset* asset = AAssetManager_open(mgr, a, AASSET_MODE_UNKNOWN);
    if (asset == NULL) {
        LOGE("Failed to read asset");
    }
    long size = AAsset_getLength(asset);
    //TODO:

    openMVG::image::Image<unsigned char> imageL, imageR;
    openMVG::image::ReadImage(a, &imageL);
    openMVG::image::ReadImage(b, &imageR);

    env->ReleaseStringUTFChars(a_, a);
    env->ReleaseStringUTFChars(b_, b);
}