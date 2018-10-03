#include <jni.h>
#include <string>
#include "openMVG/image/image_io.hpp"
//#include "openMVG/image/image_concat.hpp"
//#include "openMVG/features/akaze/image_describer_akaze.hpp"
//#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp"
//#include "openMVG/features/svg_features.hpp"
//#include "openMVG/matching/regions_matcher.hpp"
//#include "openMVG/matching/svg_matches.hpp"
//#include "openMVG/cameras/Camera_Pinhole.hpp"
//#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
//#include "openMVG/multiview/triangulation.hpp"
//
//#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"
//#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
//#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
//#include "openMVG/sfm/sfm_report.hpp"
//#include "openMVG/sfm/sfm_view.hpp"

//#include <cereal/archives/json.hpp>
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

//bool readIntrinsic(const std::string & fileName, openMVG::Mat3 & K)
//{
//    // Load the K matrix
//    std::ifstream in;
//    in.open( fileName.c_str(), std::ifstream::in);
//    if (in.is_open())  {
//        for (int j=0; j < 3; ++j)
//            for (int i=0; i < 3; ++i)
//                in >> K(j,i);
//    }
//    else  {
//        std::cerr << std::endl
//                  << "Invalid input K.txt file" << std::endl;
//        return false;
//    }
//    return true;
//}
//
///// Export 3D point vector and camera position to PLY format
//bool exportToPly(const std::vector<openMVG::Vec3> & vec_points,
//                 const std::vector<openMVG::Vec3> & vec_camPos,
//                 const std::string & sFileName)
//{
//    std::ofstream outfile;
//    outfile.open(sFileName.c_str(), std::ios_base::out);
//
//    outfile << "ply"
//            << '\n' << "format ascii 1.0"
//            << '\n' << "element vertex " << vec_points.size()+vec_camPos.size()
//            << '\n' << "property float x"
//            << '\n' << "property float y"
//            << '\n' << "property float z"
//            << '\n' << "property uchar red"
//            << '\n' << "property uchar green"
//            << '\n' << "property uchar blue"
//            << '\n' << "end_header" << std::endl;
//
//    for (size_t i=0; i < vec_points.size(); ++i)  {
//        outfile << vec_points[i].transpose()
//                << " 255 255 255" << "\n";
//    }
//
//    for (size_t i=0; i < vec_camPos.size(); ++i)  {
//        outfile << vec_camPos[i].transpose()
//                << " 0 255 0" << "\n";
//    }
//    outfile.flush();
//    const bool bOk = outfile.good();
//    outfile.close();
//    return bOk;
//}

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

    std::string sSfM_Data_Filename = "/sdcard/Download/matches/sfm_data.bin";
    std::string sMatchesDir = "/sdcard/Download/matches";

    FILE *fp1 = fopen(sSfM_Data_Filename.c_str(), "r");
    if(fp1 == NULL)
    {
        LOGE("Failed to open test file using fopen");
    }
    fclose(fp1);

    try {
        openMVG::sfm::SfM_Data data;
        bool bStatus = openMVG::sfm::Load_Cereal<cereal::PortableBinaryInputArchive>(
                data, sSfM_Data_Filename, openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS|openMVG::sfm::INTRINSICS));

//        openMVG::sfm::SfM_Data sfm_data;
//        if (!Load(sfm_data, sSfM_Data_Filename, openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS|openMVG::sfm::INTRINSICS))) {
//            LOGE("%s", "FAIL");
//        } else{
//            LOGI("%s", "SUCCESS");
//        }
    }
    catch (const std::exception & e)
    {
        LOGE("%s", e.what());
        throw;
    }




//    stream.close();
//    LOGE("GOOD DDD");

//    openMVG::sfm::SfM_Data sfm_data;
//    LOGI("testtt %s", sSfM_Data_Filename.c_str());
//    if (!Load(sfm_data, sSfM_Data_Filename,openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS|openMVG::sfm::INTRINSICS))) {
//        LOGE("The input SfM_Data file %s cannot be read.", sSfM_Data_Filename.c_str());
//    } else {
//        LOGE("GOOD GOOD");
//    }


    env->ReleaseStringUTFChars(a_, a);
    env->ReleaseStringUTFChars(b_, b);
}