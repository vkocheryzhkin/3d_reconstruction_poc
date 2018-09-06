# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

add_library( libopenMVG_image SHARED IMPORTED )
set_target_properties(libopenMVG_image PROPERTIES IMPORTED_LOCATION
    $ENV{OPENMVG_ROOT}/lib/libopenMVG_image.so )

add_library( libopenMVG_features SHARED IMPORTED )
set_target_properties(libopenMVG_features PROPERTIES IMPORTED_LOCATION
    $ENV{OPENMVG_ROOT}/lib/libopenMVG_features.so )

add_library( libopenMVG_matching SHARED IMPORTED )
set_target_properties(libopenMVG_matching PROPERTIES IMPORTED_LOCATION
    $ENV{OPENMVG_ROOT}/lib/libopenMVG_matching.so )

add_library( libopenMVG_stlplus SHARED IMPORTED )
set_target_properties(libopenMVG_stlplus PROPERTIES IMPORTED_LOCATION
    $ENV{OPENMVG_ROOT}/lib/libopenMVG_stlplus.so )

add_library( libvlsift STATIC IMPORTED )
set_target_properties(libvlsift PROPERTIES IMPORTED_LOCATION
    $ENV{OPENMVG_ROOT}/lib/libvlsift.a )

message(STATUS ${ANDROID_ABI})
# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.

add_library( # Sets the name of the library.
             native-lib

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             src/main/cpp/native-lib.cpp )

# Searches for a specified prebuilt library and stores the path as a
# variable. Because CMake includes system libraries in the search path by
# default, you only need to specify the name of the public NDK library
# you want to add. CMake verifies that the library exists before
# completing its build.

find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log )

# Specifies libraries CMake should link to your target library. You
# can link multiple libraries, such as libraries you define in this
# build script, prebuilt third-party libraries, or system libraries.


target_include_directories(native-lib PRIVATE
                           $ENV{OPENMVG_ROOT}/include
                           $ENV{OPENMVG_ROOT}/include/openMVG/third_party/eigen)


add_custom_command(TARGET native-lib
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $ENV{OPENMVG_ROOT}/lib/libopenMVG_image.so
        ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI})

add_custom_command(TARGET native-lib
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $ENV{OPENMVG_ROOT}/lib/libopenMVG_features.so
        ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI})

add_custom_command(TARGET native-lib
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $ENV{OPENMVG_ROOT}/lib/libopenMVG_matching.so
        ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI})

add_custom_command(TARGET native-lib
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $ENV{OPENMVG_ROOT}/lib/libopenMVG_stlplus.so
        ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI})

add_custom_command(TARGET native-lib
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $ENV{OPENMVG_ROOT}/lib/libopenMVG_numeric.so
        ${CMAKE_SOURCE_DIR}/src/main/jniLibs/armeabi-v7a)

target_link_libraries( # Specifies the target library.
                       native-lib

                       android #required to get AAssetManager

                       #libopenMVG_tiff
                       #libopenMVG_jpeg
                       #libopenMVG_png

                       libopenMVG_image
                       libopenMVG_features
                       libopenMVG_matching
                       libopenMVG_stlplus
                       libvlsift

                       # Links the target library to the log library
                       # included in the NDK.
                       ${log-lib} )