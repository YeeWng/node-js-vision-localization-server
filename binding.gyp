{
  "targets": [
    {
      "target_name": "localizeImage",
      "sources": [
      				"src/localizeImage.cc",
      				"src/LocalizeEngine.cc",
      				"../VisionLocalizeCommon/src/AKAZEOpenCV.cpp",
      				"../VisionLocalizeCommon/src/AKAZEOption.cpp",
      				"../VisionLocalizeCommon/src/FileUtils.cpp",
      				"../VisionLocalizeCommon/src/StringUtils.cpp",
      				"../VisionLocalizeCommon/src/MatchUtils.cpp",
      				"../VisionLocalizeCommon/src/MatUtils.cpp",
      				"../VisionLocalizeCommon/src/SfMDataUtils.cpp"
      			],
	  "cflags_cc" : [ "-O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -fopenmp -fexceptions" ],
	  "cflags_cc!" : [ "-fno-rtti" ],
      "include_dirs": [
      					"/usr/local/include/openMVG_dependencies/cereal/include",
	      				"/opt/opencv-3.0.0/include",
						"/usr/include/eigen3",
						"/usr/local/include/openMVG/third_party",
						"/usr/local/include/openMVG",
						"/usr/local/include/openMVG/multiview",
						"/usr/local/include/openMVG/features",
						"/usr/local/include/openMVG/features/akaze",
						"/usr/local/include/openMVG/third_party/lemon",
						"../VisionLocalizeCommon/src",
						"../BoWCommon/src"
      				],
      "library_dirs": [
      					"/usr/lib/x86_64-linux-gnu",
						"/usr/local/lib",
      					"/opt/opencv-3.0.0/lib"
      				],
      "libraries": [	      				
	      				"-lopenMVG_sfm",
						"-lopenMVG_features",
						"-lopenMVG_matching_image_collection",
						"-lopenMVG_multiview",
						"-lopenMVG_numeric",
						"-lopenMVG_lInftyComputerVision",
						"-lopenMVG_system",
						"-lopenMVG_image",
						"-lopencv_flann",
						"-lopencv_xfeatures2d",
						"-lopencv_features2d",
						"-lopencv_imgcodecs",
						"-lopencv_highgui",
						"-lopencv_imgproc",
						"-lopencv_calib3d",
						"-lopencv_core",
						"-lopenMVG_kvld",
						"-lstlplus",
						"-lceres",
						"-lcxsparse",
						"-lcholmod",
						"-lglog",
						"-llapack",
						"-lblas",
						"-lgomp",
						"-lpthread",
						"-lpng",
						"-ljpeg",
						"-ltiff",
						"-llib_OsiClpSolver",
						"-llib_CoinUtils",
						"-llib_Osi",
						"-llib_clp",
						"-lvlsift",
						"-lflann_cpp_s",
						"-llemon",
						"-leasyexif",
						"-luuid"
      				]
    }
  ]
}