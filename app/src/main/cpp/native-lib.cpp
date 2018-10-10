#include <jni.h>
#include <string>
#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_io_cereal.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/pipelines/sequential/sequential_SfM.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_matches_provider.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_report.hpp"
#include "openMVG/sfm/sfm_view.hpp"
//#include <cereal/archives/portable_binary.hpp>
#include <android/log.h>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>
#include <iostream>
#include <fstream>
//#include <openMVG/src/dependencies/cereal/include/cereal/archives/json.hpp>
#include <cereal/archives/json.hpp>

#define  TAG "Native"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO, TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, TAG,__VA_ARGS__)

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::sfm;

extern "C" JNIEXPORT jstring

JNICALL
Java_com_local_a3d_1reconstruction_1poc_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

/// From 2 given image file-names, find the two corresponding index in the View list
bool computeIndexFromImageNames(
        const SfM_Data & sfm_data,
        const std::pair<std::string,std::string>& initialPairName,
        Pair& initialPairIndex)
{
    if (initialPairName.first == initialPairName.second)
    {
        LOGE("%s", "Invalid image names. You cannot use the same image to initialize a pair.");
        return false;
    }

    initialPairIndex = {UndefinedIndexT, UndefinedIndexT};

    /// List views filenames and find the one that correspond to the user ones:
    for (Views::const_iterator it = sfm_data.GetViews().begin();
         it != sfm_data.GetViews().end(); ++it)
    {
        const View * v = it->second.get();
        const std::string filename = stlplus::filename_part(v->s_Img_path);
        if (filename == initialPairName.first)
        {
            initialPairIndex.first = v->id_view;
        }
        else{
            if (filename == initialPairName.second)
            {
                initialPairIndex.second = v->id_view;
            }
        }
    }
    return (initialPairIndex.first != UndefinedIndexT &&
            initialPairIndex.second != UndefinedIndexT);
}
extern "C"
JNIEXPORT void JNICALL
Java_com_local_a3d_1reconstruction_1poc_Reconstructor_matchNative(JNIEnv *env, jclass type,
                                                                  jobject assetManager, jstring a_,
                                                                  jstring b_) {
    const char *a = env->GetStringUTFChars(a_, 0);
    const char *b = env->GetStringUTFChars(b_, 0);

    std::string sSfM_Data_Filename = "/sdcard/Download/matches/sfm_data.json";
    std::string sMatchesDir = "/sdcard/Download/matches/";
    std::string  sMatchFilename = "";
    std::string sOutDir = "/sdcard/Download/out/";
    std::string sIntrinsic_refinement_options = "ADJUST_ALL";
    int i_User_camera_model = PINHOLE_CAMERA_RADIAL3;
    bool b_use_motion_priors = false;
    std::pair<std::string,std::string> initialPairString("","");
    try {
//        openMVG::sfm::SfM_Data sfm_data;
        //openMVG::sfm::Load(
//            sfm_data,
//            sSfM_Data_Filename,
//            openMVG::sfm::ESfM_Data(openMVG::sfm::VIEWS|openMVG::sfm::INTRINSICS));
////        LOGI("%s", sfm_data.s_root_path.c_str());
//
//        std::ofstream stream(sSfM_Data_Filename.c_str(), std::ios::binary | std::ios::out);
//        if (!stream.is_open())
//            LOGE("%s", "Failed to open");

//        std::ifstream stream(sSfM_Data_Filename.c_str(), std::ios::binary | std::ios::in);
//        if (!stream.is_open()) {
//            LOGE("%s", "Failed to open");
//            throw;
//        }
//        {
//            cereal::JSONInputArchive archive(stream);
//            archive(cereal::make_nvp("root_path", sfm_data.s_root_path));
//            //archive(cereal::make_nvp("views", sfm_data.views));
//            //archive(cereal::make_nvp("intrinsics", sfm_data.intrinsics));
//        }
//        stream.close();
//        LOGI("%s", sfm_data.s_root_path.c_str());

        const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options =
                cameras::StringTo_Intrinsic_Parameter_Type(sIntrinsic_refinement_options);
        if (intrinsic_refinement_options == static_cast<cameras::Intrinsic_Parameter_Type>(0) )
        {
            LOGE("%s", "Invalid input for Bundle Adjusment Intrinsic parameter refinement option");
        }

        // Load input SfM_Data scene
        SfM_Data sfm_data;
//        if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
//            LOGE("%s", "The input SfM_Data file cannot be read");
//        }

        const int nviews = 11;
        const int npoints = 0;
        int cx = 1416.0;
        int cy = 1064.0;
//        const nViewDatasetConfigurator config;
//        const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config
        // 1. Views
//        for (int i = 0; i < nviews; ++i)
//        {
//            const IndexT id_view = i, id_pose = i, id_intrinsic = 0; //(shared intrinsics)
//            sfm_data.views[i] = std::make_shared<View>("", id_view, id_intrinsic, id_pose, cx *2, cy *2);
//        }

        sfm_data.views[0] = std::make_shared<View>("100_7100.JPG", 0, 0, 0, cx *2, cy *2);
        sfm_data.views[1] = std::make_shared<View>("100_7101.JPG", 1, 0, 1, cx *2, cy *2);
        sfm_data.views[2] = std::make_shared<View>("100_7102.JPG", 2, 0, 2, cx *2, cy *2);
        sfm_data.views[3] = std::make_shared<View>("100_7103.JPG", 3, 0, 3, cx *2, cy *2);
        sfm_data.views[4] = std::make_shared<View>("100_7104.JPG", 4, 0, 4, cx *2, cy *2);
        sfm_data.views[5] = std::make_shared<View>("100_7105.JPG", 5, 0, 5, cx *2, cy *2);
        sfm_data.views[6] = std::make_shared<View>("100_7106.JPG", 6, 0, 6, cx *2, cy *2);
        sfm_data.views[7] = std::make_shared<View>("100_7107.JPG", 7, 0, 7, cx *2, cy *2);
        sfm_data.views[8] = std::make_shared<View>("100_7108.JPG", 8, 0, 8, cx *2, cy *2);
        sfm_data.views[9] = std::make_shared<View>("100_7109.JPG", 9, 0, 9, cx *2, cy *2);
        sfm_data.views[10] = std::make_shared<View>("100_7110.JPG", 10, 0, 10, cx *2, cy *2);


        // Add a rotation to the GT (in order to make BA do some work)
//        const Mat3 rot = RotationAroundX(D2R(6));

        // 2. Poses
//        for (int i = 0; i < nviews; ++i)
//        {
//            const Pose3 pose(rot * d._R[i], d._C[i]);
//            sfm_data.poses[i] = pose;
//        }

        sfm_data.intrinsics[0] = std::make_shared<Pinhole_Intrinsic_Radial_K3>
                (2832, 2128, 2881.25212694251, 1416.0, 1064.0, 0., 0., 0.);



        // Init the regions_type from the image describer file (used for image regions extraction)
        using namespace openMVG::features;
        const std::string sImage_describer = stlplus::create_filespec(sMatchesDir, "image_describer", "json");
        std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
        if (!regions_type)
        {
            LOGE("%s", "Invalid regions type files");
        } else {
            LOGI("%s", "Regions done");
        }

        // Features reading
        std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
        if (!feats_provider->load(sfm_data, sMatchesDir, regions_type)) {
            std::cerr << std::endl
                      << "Invalid features." << std::endl;
            LOGE("%s", "Invalid features.");
        } else {
            LOGI("%s", "Features done");
        }

        // Matches reading
        std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
        if // Try to read the provided match filename or the default one (matches.f.txt/bin)
                (
                !(matches_provider->load(sfm_data, sMatchFilename) ||
                  matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.f.txt")) ||
                  matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.f.bin")))
                )
        {
            LOGE("%s", "Invalid matches file.");
        } else {
            LOGI("%s", "Matches done");
        }

        LOGI("%s", "sfmEngine");
        std::string reportFile = stlplus::create_filespec(sOutDir, "Reconstruction_Report.html");
        LOGI("%s", reportFile.c_str());
        LOGI("%d", sfm_data.GetViews().size());
        SequentialSfMReconstructionEngine sfmEngine(
                sfm_data,
                sOutDir,
                stlplus::create_filespec(sOutDir, "Reconstruction_Report.html"));

        LOGI("%s", "Configure the features_provider & the matches_provider");
        sfmEngine.SetFeaturesProvider(feats_provider.get());
        sfmEngine.SetMatchesProvider(matches_provider.get());

        // Configure reconstruction parameters
        sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
        sfmEngine.SetUnknownCameraType(EINTRINSIC(i_User_camera_model));
        //b_use_motion_priors = cmd.used('P');
        sfmEngine.Set_Use_Motion_Prior(b_use_motion_priors);

        LOGI("%s", "Handle Initial pair parameter");

        if (!initialPairString.first.empty() && !initialPairString.second.empty())
        {
            Pair initialPairIndex;
            if (!computeIndexFromImageNames(sfm_data, initialPairString, initialPairIndex))
            {
                LOGE("%s", "Could not find the initial pairs.");
            }
            sfmEngine.setInitialPair(initialPairIndex);
        }
        else {
            LOGI("%s", "computeIndexFromImageNames done");
        }
        if (sfmEngine.Process())
        {
//            std::cout << std::endl << " Total Ac-Sfm took (s): " << timer.elapsed() << std::endl;
//
//            std::cout << "...Generating SfM_Report.html" << std::endl;
            Generate_SfM_Report(sfmEngine.Get_SfM_Data(),
                                stlplus::create_filespec(sOutDir, "SfMReconstruction_Report.html"));
//
//            //-- Export to disk computed scene (data & visualizable results)
//            std::cout << "...Export SfM_Data to disk." << std::endl;
//            Save(sfmEngine.Get_SfM_Data(),
//                 stlplus::create_filespec(sOutDir, "sfm_data", ".bin"),
//                 ESfM_Data(ALL));

            Save(sfmEngine.Get_SfM_Data(),
                 stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"),
                 ESfM_Data(ALL));

//            return EXIT_SUCCESS;
        }else {
            LOGI("%s", "sfmEngine done");
        }
    }
    catch (const std::exception & e)
    {
        LOGE("%s", e.what());
        //throw;
    }

    env->ReleaseStringUTFChars(a_, a);
    env->ReleaseStringUTFChars(b_, b);
}