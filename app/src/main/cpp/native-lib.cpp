#include <jni.h>
#include <string>
#include "openMVG/image/image_io.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_io_cereal.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include <cereal/archives/portable_binary.hpp>
#include <android/log.h>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include <iostream>

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

    std::string sSfM_Data_Filename = "/sdcard/Download/sfm_data.json";

    try {
        openMVG::sfm::SfM_Data sfm_data;
        sfm_data.s_root_path = "test";
        openMVG::sfm::Save(
            sfm_data,
            sSfM_Data_Filename,
            openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS|openMVG::sfm::INTRINSICS));
    }
    catch (const std::exception & e)
    {
        LOGE("%s", e.what());
        throw;
    }

    env->ReleaseStringUTFChars(a_, a);
    env->ReleaseStringUTFChars(b_, b);
}