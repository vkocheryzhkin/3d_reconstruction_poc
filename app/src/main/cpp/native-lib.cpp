#include <jni.h>
#include <string>
#include "openMVG/image/image_io.hpp"
#include "openMVG/image/image_concat.hpp"
#include "openMVG/features/akaze/image_describer_akaze.hpp"
#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp"
#include "openMVG/features/svg_features.hpp"
#include "openMVG/matching/regions_matcher.hpp"
#include "openMVG/matching/svg_matches.hpp"
#include "openMVG/cameras/Camera_Pinhole.hpp"
#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
#include "openMVG/multiview/triangulation.hpp"

#include <android/log.h>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>

#include <iostream>

#define  TAG "Native"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO, TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, TAG,__VA_ARGS__)

bool readIntrinsic(const std::string & fileName, openMVG::Mat3 & K)
{
    // Load the K matrix
    std::ifstream in;
    in.open( fileName.c_str(), std::ifstream::in);
    if (in.is_open())  {
        for (int j=0; j < 3; ++j)
            for (int i=0; i < 3; ++i)
                in >> K(j,i);
    }
    else  {
        std::cerr << std::endl
                  << "Invalid input K.txt file" << std::endl;
        return false;
    }
    return true;
}

/// Export 3D point vector and camera position to PLY format
bool exportToPly(const std::vector<openMVG::Vec3> & vec_points,
                 const std::vector<openMVG::Vec3> & vec_camPos,
                 const std::string & sFileName)
{
    std::ofstream outfile;
    outfile.open(sFileName.c_str(), std::ios_base::out);

    outfile << "ply"
            << '\n' << "format ascii 1.0"
            << '\n' << "element vertex " << vec_points.size()+vec_camPos.size()
            << '\n' << "property float x"
            << '\n' << "property float y"
            << '\n' << "property float z"
            << '\n' << "property uchar red"
            << '\n' << "property uchar green"
            << '\n' << "property uchar blue"
            << '\n' << "end_header" << std::endl;

    for (size_t i=0; i < vec_points.size(); ++i)  {
        outfile << vec_points[i].transpose()
                << " 255 255 255" << "\n";
    }

    for (size_t i=0; i < vec_camPos.size(); ++i)  {
        outfile << vec_camPos[i].transpose()
                << " 0 255 0" << "\n";
    }
    outfile.flush();
    const bool bOk = outfile.good();
    outfile.close();
    return bOk;
}

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

//    AAssetManager *mgr = AAssetManager_fromJava(env, assetManager);
//    if (mgr == NULL) {
//        LOGE("Failed to load asset manager");
//    }
//
//    AAsset* asset = AAssetManager_open(mgr, a, AASSET_MODE_UNKNOWN);
//    if (asset == NULL) {
//        LOGE("Failed to read asset");
//    }
//    long asset_size = AAsset_getLength(asset);
//    LOGI("asset size: %d", asset_size);

//    std::vector<unsigned char> array;
//    int w;
//    int h;
//    int depth;
//    openMVG::image::ReadPngStream((FILE*)asset, &array, &w, &h, &depth);

//    LOGI("w: %d", w);
//    LOGI("h: %d", h);

//    char* fileContent = new char[asset_size+1];
//    AAsset_read(asset, fileContent, asset_size);
//    fileContent[asset_size] = '\0';
//
//    delete [] fileContent;

    //TODO:
    LOGI("a: %s", a);
    LOGI("b: %s", b);

//    FILE *file1 = fopen(a, "r+");
//    if (file1 != NULL ) {
//        fclose(file1);
//        LOGI("GOOD");
//    } else{
//        LOGI("BAD");
//    }

//    AAssetDir* assetDir = AAssetManager_openDir(mgr, "");
//    const char* filename = (const char*)NULL;
//    while ((filename = AAssetDir_getNextFileName(assetDir)) != NULL) {
//        LOGI("filename: %s", filename);
//        AAsset* asset = AAssetManager_open(mgr, filename, AASSET_MODE_STREAMING);
//        char buf[BUFSIZ];
//        int nb_read = 0;
//        FILE* out = fopen(filename, "w");
//        while ((nb_read = AAsset_read(asset, buf, BUFSIZ)) > 0)
//            fwrite(buf, nb_read, 1, out);
//        fclose(out);
//        AAsset_close(asset);
//    }
//    AAssetDir_close(assetDir);

    openMVG::image::Image<unsigned char> imageL, imageR;
    openMVG::image::ReadImage(a, &imageL);
    openMVG::image::ReadImage(b, &imageR);

    using namespace openMVG::features;
    //std::unique_ptr<Image_describer> image_describer;
    //image_describer = AKAZE_Image_describer::create(AKAZE_Image_describer::Params(AKAZE::Params(), AKAZE_MSURF));
    std::unique_ptr<Image_describer> image_describer(new SIFT_Anatomy_Image_describer);
    if (!image_describer)
    {
        LOGE("Invalid Image_describer type");
        return;
    }

    std::map<openMVG::IndexT, std::unique_ptr<openMVG::features::Regions>> regions_perImage;
    image_describer->Describe(imageL, regions_perImage[0]);
    image_describer->Describe(imageR, regions_perImage[1]);

    const SIFT_Regions* regionsL = dynamic_cast<SIFT_Regions*>(regions_perImage.at(0).get());
    const SIFT_Regions* regionsR = dynamic_cast<SIFT_Regions*>(regions_perImage.at(1).get());


    openMVG::features::PointFeatures featsL = regions_perImage.at(0)->GetRegionsPositions();
    openMVG::features::PointFeatures featsR = regions_perImage.at(1)->GetRegionsPositions();
    LOGI("imageL w: %d", imageL.Width());
    LOGI("imageL h: %d", imageL.Height());
    LOGI("f1 size: %d", featsL.size());
    LOGI("f2 size: %d", featsR.size());

    {
        //- Draw features on the images (side by side)
        Features2SVG
            (
                a,
                {imageL.Width(), imageL.Height()},
                featsL,
                b,
                {imageR.Width(), imageR.Height()},
                featsR,
                "/sdcard/Download/01_features.svg"
            );
    }


    std::vector<openMVG::matching::IndMatch> vec_PutativeMatches;
    //-- Perform matching -> find Nearest neighbor, filtered with Distance ratio
    {
        // Find corresponding points
        openMVG::matching::DistanceRatioMatch(
                0.8, openMVG::matching::BRUTE_FORCE_L2,
                *regions_perImage.at(0).get(),
                *regions_perImage.at(1).get(),
                vec_PutativeMatches);

        openMVG::matching::IndMatchDecorator<float> matchDeduplicator(
                vec_PutativeMatches, featsL, featsR);
        matchDeduplicator.getDeduplicated(vec_PutativeMatches);

//        std::cout
//                << regions_perImage.at(0)->RegionCount() << " #Features on image A" << std::endl
//                << regions_perImage.at(1)->RegionCount() << " #Features on image B" << std::endl
//                << vec_PutativeMatches.size() << " #matches with Distance Ratio filter" << std::endl;

        // Draw correspondences after Nearest Neighbor ratio filter
        const bool bVertical = true;
        Matches2SVG
        (
            a,
            {imageL.Width(), imageL.Height()},
            regionsL->GetRegionsPositions(),
            b,
            {imageR.Width(), imageR.Height()},
            regionsR->GetRegionsPositions(),
            vec_PutativeMatches,
            "/sdcard/Download/03_Matches.svg",
            bVertical
        );
    }

    // Essential geometry filtering of putative matches
    {
        openMVG::Mat3 K;
        //read K from file
        if (!readIntrinsic("/sdcard/Download/K.txt", K)) {
            std::cerr << "Cannot read intrinsic parameters." << std::endl;
            LOGE("Cannot read intrinsic parameters");
        } else {
            LOGI("read success");
        }

        const openMVG::cameras::Pinhole_Intrinsic
                camL(imageL.Width(), imageL.Height(), K(0,0), K(0,2), K(1,2)),
                camR(imageR.Width(), imageR.Height(), K(0,0), K(0,2), K(1,2));

        //A. prepare the corresponding putatives points
        openMVG::Mat xL(2, vec_PutativeMatches.size());
        openMVG::Mat xR(2, vec_PutativeMatches.size());
        for (size_t k = 0; k < vec_PutativeMatches.size(); ++k)  {
            const PointFeature & imaL = featsL[vec_PutativeMatches[k].i_];
            const PointFeature & imaR = featsR[vec_PutativeMatches[k].j_];
            xL.col(k) = imaL.coords().cast<double>();
            xR.col(k) = imaR.coords().cast<double>();
        }

        LOGI("size %d", vec_PutativeMatches.size());

        //B. Compute the relative pose thanks to a essential matrix estimation
        const std::pair<size_t, size_t> size_imaL(imageL.Width(), imageL.Height());
        const std::pair<size_t, size_t> size_imaR(imageR.Width(), imageR.Height());
        openMVG::sfm::RelativePose_Info relativePose_info;
        if (!openMVG::sfm::robustRelativePose(&camL, &camR, xL, xR, relativePose_info, size_imaL,
                                              size_imaR, 256))
        {
            LOGE("Robust relative pose estimation failure.");
        } else {
            LOGI("sfm done");
        }
        // Show Essential validated point
        const bool bVertical = true;
        InlierMatches2SVG
        (
            a,
            {imageL.Width(), imageL.Height()},
            regionsL->GetRegionsPositions(),
            b,
            {imageR.Width(), imageR.Height()},
            regionsR->GetRegionsPositions(),
            vec_PutativeMatches,
            relativePose_info.vec_inliers,
            "/sdcard/Download/04_ACRansacEssential.svg",
            bVertical
        );

        //C. Triangulate and export inliers as a PLY scene
        std::vector<openMVG::Vec3> vec_3DPoints;

        // Setup camera intrinsic and poses
        openMVG::cameras::Pinhole_Intrinsic intrinsic0(imageL.Width(), imageL.Height(), K(0, 0), K(0, 2), K(1, 2));
        openMVG::cameras::Pinhole_Intrinsic intrinsic1(imageR.Width(), imageR.Height(), K(0, 0), K(0, 2), K(1, 2));

        const openMVG::geometry::Pose3 pose0 = openMVG::geometry::Pose3(openMVG::Mat3::Identity(), openMVG::Vec3::Zero());
        const openMVG::geometry::Pose3 pose1 = relativePose_info.relativePose;

        // Init structure by inlier triangulation
        const openMVG::Mat34 P1 = intrinsic0.get_projective_equivalent(pose0);
        const openMVG::Mat34 P2 = intrinsic1.get_projective_equivalent(pose1);
        std::vector<double> vec_residuals;
        vec_residuals.reserve(relativePose_info.vec_inliers.size() * 4);
        for (size_t i = 0; i < relativePose_info.vec_inliers.size(); ++i)  {
            const SIOPointFeature & LL = regionsL->Features()[vec_PutativeMatches[relativePose_info.vec_inliers[i]].i_];
            const SIOPointFeature & RR = regionsR->Features()[vec_PutativeMatches[relativePose_info.vec_inliers[i]].j_];
            // Point triangulation
            openMVG::Vec3 X;
            openMVG::TriangulateDLT(
                    P1, LL.coords().cast<double>().homogeneous(),
                    P2, RR.coords().cast<double>().homogeneous(), &X);
            // Reject point that is behind the camera
            if (openMVG::Depth(pose0.rotation(), pose0.translation(), X) < 0 &&
                    openMVG::Depth(pose1.rotation(), pose1.translation(), X) < 0)
                continue;

            const openMVG::Vec2 residual0 = intrinsic0.residual(pose0(X), LL.coords().cast<double>());
            const openMVG::Vec2 residual1 = intrinsic1.residual(pose1(X), RR.coords().cast<double>());
            vec_residuals.push_back(std::abs(residual0(0)));
            vec_residuals.push_back(std::abs(residual0(1)));
            vec_residuals.push_back(std::abs(residual1(0)));
            vec_residuals.push_back(std::abs(residual1(1)));
            vec_3DPoints.emplace_back(X);
        }

        LOGI("vec_3DPoints: %d", vec_3DPoints.size());

        // Export as PLY (camera pos + 3Dpoints)
        std::vector<openMVG::Vec3> vec_camPos;
        vec_camPos.push_back( pose0.center() );
        vec_camPos.push_back( pose1.center() );
        exportToPly(vec_3DPoints, vec_camPos, "/sdcard/Download/EssentialGeometry.ply");
    }

    env->ReleaseStringUTFChars(a_, a);
    env->ReleaseStringUTFChars(b_, b);
}