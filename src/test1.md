developer@65abafec5dc5:~$ ls
ros_ws  tgk_test.log
developer@65abafec5dc5:~$ cd ros_ws/
developer@65abafec5dc5:~/ros_ws$ ls
ego-planner  ego-planner-swarm  ego带topo-mppi-A星-B样条  fastplanner  nlopt
developer@65abafec5dc5:~/ros_ws$ cd ego-planner
developer@65abafec5dc5:~/ros_ws/ego-planner$ ls
build  Generate  modified_realsense2_camera.zip  README.md  src
devel  LICENSE   Optimize                        Select     topomppi.txt
developer@65abafec5dc5:~/ros_ws/ego-planner$ catkin_make
Base path: /home/developer/ros_ws/ego-planner
Source space: /home/developer/ros_ws/ego-planner/src
Build space: /home/developer/ros_ws/ego-planner/build
Devel space: /home/developer/ros_ws/ego-planner/devel
Install space: /home/developer/ros_ws/ego-planner/install
####
#### Running command: "make cmake_check_build_system" in "/home/developer/ros_ws/ego-planner/build"
####
-- Using CATKIN_DEVEL_PREFIX: /home/developer/ros_ws/ego-planner/devel
-- Using CMAKE_PREFIX_PATH: /opt/ros/noetic
-- This workspace overlays: /opt/ros/noetic
-- Found PythonInterp: /usr/bin/python3 (found suitable version "3.8.10", minimum required is "3") 
-- Using PYTHON_EXECUTABLE: /usr/bin/python3
-- Using Debian Python package layout
-- Using empy: /usr/lib/python3/dist-packages/em.py
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/developer/ros_ws/ego-planner/build/test_results
-- Forcing gtest/gmock from source, though one was otherwise available.
-- Found gtest sources under '/usr/src/googletest': gtests will be built
-- Found gmock sources under '/usr/src/googletest': gmock will be built
-- Found PythonInterp: /usr/bin/python3 (found version "3.8.10") 
-- Using Python nosetests: /usr/bin/nosetests3
-- catkin 0.8.12
-- BUILD_SHARED_LIBS is on
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 18 packages in topological order:
-- ~~  - quadrotor_msgs
-- ~~  - cmake_utils
-- ~~  - map_generator
-- ~~  - plan_env
-- ~~  - path_searching
-- ~~  - bspline_opt
-- ~~  - pose_utils
-- ~~  - odom_visualization
-- ~~  - local_sensing_node
-- ~~  - mockamap
-- ~~  - so3_control
-- ~~  - multi_map_server
-- ~~  - traj_utils
-- ~~  - ego_planner
-- ~~  - uav_utils
-- ~~  - so3_quadrotor_simulator
-- ~~  - rviz_plugins
-- ~~  - waypoint_generator
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'quadrotor_msgs'
-- ==> add_subdirectory(uav_simulator/Utils/quadrotor_msgs)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- quadrotor_msgs: 13 messages, 0 services
-- +++ processing catkin package: 'cmake_utils'
-- ==> add_subdirectory(uav_simulator/Utils/cmake_utils)
-- +++ processing catkin package: 'map_generator'
-- ==> add_subdirectory(uav_simulator/map_generator)
-- Found Eigen: /usr/include/eigen3 (Required is at least version "3.1") 
-- Eigen found (include: /usr/include/eigen3, version: 3.3.7)
-- The imported target "vtkParseOGLExt" references the file
   "/usr/bin/vtkParseOGLExt-7.1"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "vtkRenderingPythonTkWidgets" references the file
   "/usr/lib/x86_64-linux-gnu/libvtkRenderingPythonTkWidgets.so"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "vtk" references the file
   "/usr/bin/vtk"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "pvtk" references the file
   "/usr/bin/pvtk"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- OpenNI2 found (include: /usr/include/openni2, lib: /usr/lib/libOpenNI2.so)
** WARNING ** io features related to pcap will be disabled
** WARNING ** io features related to png will be disabled
** WARNING ** io features related to libusb-1.0 will be disabled
-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- OpenNI2 found (include: /usr/include/openni2, lib: /usr/lib/libOpenNI2.so)
-- QHULL found (include: /usr/include, lib: optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so)
-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- looking for PCL_COMMON
-- looking for PCL_KDTREE
-- looking for PCL_OCTREE
-- looking for PCL_SEARCH
-- looking for PCL_SAMPLE_CONSENSUS
-- looking for PCL_FILTERS
-- looking for PCL_2D
-- looking for PCL_GEOMETRY
-- looking for PCL_IO
-- looking for PCL_FEATURES
-- looking for PCL_ML
-- looking for PCL_SEGMENTATION
-- looking for PCL_VISUALIZATION
-- looking for PCL_SURFACE
-- looking for PCL_REGISTRATION
-- looking for PCL_KEYPOINTS
-- looking for PCL_TRACKING
-- looking for PCL_RECOGNITION
-- looking for PCL_STEREO
-- looking for PCL_APPS
-- looking for PCL_IN_HAND_SCANNER
-- looking for PCL_POINT_CLOUD_EDITOR
-- looking for PCL_OUTOFCORE
-- looking for PCL_PEOPLE
-- Found PCL: pcl_common;pcl_kdtree;pcl_octree;pcl_search;pcl_sample_consensus;pcl_filters;pcl_io;pcl_features;pcl_ml;pcl_segmentation;pcl_visualization;pcl_surface;pcl_registration;pcl_keypoints;pcl_tracking;pcl_recognition;pcl_stereo;pcl_apps;pcl_outofcore;pcl_people;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so;/usr/lib/libOpenNI.so;/usr/lib/libOpenNI2.so;vtkChartsCore;vtkCommonColor;vtkCommonCore;vtksys;vtkCommonDataModel;vtkCommonMath;vtkCommonMisc;vtkCommonSystem;vtkCommonTransforms;vtkCommonExecutionModel;vtkFiltersGeneral;vtkCommonComputationalGeometry;vtkFiltersCore;vtkInfovisCore;vtkFiltersExtraction;vtkFiltersStatistics;vtkImagingFourier;vtkImagingCore;vtkalglib;vtkRenderingContext2D;vtkRenderingCore;vtkFiltersGeometry;vtkFiltersSources;vtkRenderingFreeType;/usr/lib/x86_64-linux-gnu/libfreetype.so;/usr/lib/x86_64-linux-gnu/libz.so;vtkFiltersModeling;vtkImagingSources;vtkInteractionStyle;vtkInteractionWidgets;vtkFiltersHybrid;vtkImagingColor;vtkImagingGeneral;vtkImagingHybrid;vtkIOImage;vtkDICOMParser;vtkmetaio;/usr/lib/x86_64-linux-gnu/libjpeg.so;/usr/lib/x86_64-linux-gnu/libpng.so;/usr/lib/x86_64-linux-gnu/libtiff.so;vtkRenderingAnnotation;vtkRenderingVolume;vtkIOXML;vtkIOCore;vtkIOXMLParser;/usr/lib/x86_64-linux-gnu/libexpat.so;vtkIOGeometry;vtkIOLegacy;vtkIOPLY;vtkRenderingLOD;vtkViewsContext2D;vtkViewsCore;vtkRenderingContextOpenGL2;vtkRenderingOpenGL2;FLANN::FLANN  
-- +++ processing catkin package: 'plan_env'
-- ==> add_subdirectory(planner/plan_env)
-- Eigen found (include: /usr/include/eigen3, version: 3.3.7)
-- The imported target "vtkParseOGLExt" references the file
   "/usr/bin/vtkParseOGLExt-7.1"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "vtkRenderingPythonTkWidgets" references the file
   "/usr/lib/x86_64-linux-gnu/libvtkRenderingPythonTkWidgets.so"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "vtk" references the file
   "/usr/bin/vtk"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "pvtk" references the file
   "/usr/bin/pvtk"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- OpenNI2 found (include: /usr/include/openni2, lib: /usr/lib/libOpenNI2.so)
** WARNING ** io features related to pcap will be disabled
** WARNING ** io features related to png will be disabled
** WARNING ** io features related to libusb-1.0 will be disabled
-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- OpenNI2 found (include: /usr/include/openni2, lib: /usr/lib/libOpenNI2.so)
-- QHULL found (include: /usr/include, lib: optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so)
-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- looking for PCL_COMMON
-- looking for PCL_KDTREE
-- looking for PCL_OCTREE
-- looking for PCL_SEARCH
-- looking for PCL_SAMPLE_CONSENSUS
-- looking for PCL_FILTERS
-- looking for PCL_2D
-- looking for PCL_GEOMETRY
-- looking for PCL_IO
-- looking for PCL_FEATURES
-- looking for PCL_ML
-- looking for PCL_SEGMENTATION
-- looking for PCL_VISUALIZATION
-- looking for PCL_SURFACE
-- looking for PCL_REGISTRATION
-- looking for PCL_KEYPOINTS
-- looking for PCL_TRACKING
-- looking for PCL_RECOGNITION
-- looking for PCL_STEREO
-- looking for PCL_APPS
-- looking for PCL_IN_HAND_SCANNER
-- looking for PCL_POINT_CLOUD_EDITOR
-- looking for PCL_OUTOFCORE
-- looking for PCL_PEOPLE
-- Found PCL: pcl_common;pcl_kdtree;pcl_octree;pcl_search;pcl_sample_consensus;pcl_filters;pcl_io;pcl_features;pcl_ml;pcl_segmentation;pcl_visualization;pcl_surface;pcl_registration;pcl_keypoints;pcl_tracking;pcl_recognition;pcl_stereo;pcl_apps;pcl_outofcore;pcl_people;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_iostreams.so;/usr/lib/x86_64-linux-gnu/libboost_regex.so;optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so;/usr/lib/libOpenNI.so;/usr/lib/libOpenNI2.so;vtkChartsCore;vtkCommonColor;vtkCommonCore;vtksys;vtkCommonDataModel;vtkCommonMath;vtkCommonMisc;vtkCommonSystem;vtkCommonTransforms;vtkCommonExecutionModel;vtkFiltersGeneral;vtkCommonComputationalGeometry;vtkFiltersCore;vtkInfovisCore;vtkFiltersExtraction;vtkFiltersStatistics;vtkImagingFourier;vtkImagingCore;vtkalglib;vtkRenderingContext2D;vtkRenderingCore;vtkFiltersGeometry;vtkFiltersSources;vtkRenderingFreeType;/usr/lib/x86_64-linux-gnu/libfreetype.so;/usr/lib/x86_64-linux-gnu/libz.so;vtkFiltersModeling;vtkImagingSources;vtkInteractionStyle;vtkInteractionWidgets;vtkFiltersHybrid;vtkImagingColor;vtkImagingGeneral;vtkImagingHybrid;vtkIOImage;vtkDICOMParser;vtkmetaio;/usr/lib/x86_64-linux-gnu/libjpeg.so;/usr/lib/x86_64-linux-gnu/libpng.so;/usr/lib/x86_64-linux-gnu/libtiff.so;vtkRenderingAnnotation;vtkRenderingVolume;vtkIOXML;vtkIOCore;vtkIOXMLParser;/usr/lib/x86_64-linux-gnu/libexpat.so;vtkIOGeometry;vtkIOLegacy;vtkIOPLY;vtkRenderingLOD;vtkViewsContext2D;vtkViewsCore;vtkRenderingContextOpenGL2;vtkRenderingOpenGL2;FLANN::FLANN (Required is at least version "1.7") 
-- +++ processing catkin package: 'path_searching'
-- ==> add_subdirectory(planner/path_searching)
-- Eigen found (include: /usr/include/eigen3, version: 3.3.7)
-- The imported target "vtkParseOGLExt" references the file
   "/usr/bin/vtkParseOGLExt-7.1"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "vtkRenderingPythonTkWidgets" references the file
   "/usr/lib/x86_64-linux-gnu/libvtkRenderingPythonTkWidgets.so"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "vtk" references the file
   "/usr/bin/vtk"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "pvtk" references the file
   "/usr/bin/pvtk"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- OpenNI2 found (include: /usr/include/openni2, lib: /usr/lib/libOpenNI2.so)
** WARNING ** io features related to pcap will be disabled
** WARNING ** io features related to png will be disabled
** WARNING ** io features related to libusb-1.0 will be disabled
-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- OpenNI2 found (include: /usr/include/openni2, lib: /usr/lib/libOpenNI2.so)
-- QHULL found (include: /usr/include, lib: optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so)
-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- looking for PCL_COMMON
-- looking for PCL_KDTREE
-- looking for PCL_OCTREE
-- looking for PCL_SEARCH
-- looking for PCL_SAMPLE_CONSENSUS
-- looking for PCL_FILTERS
-- looking for PCL_2D
-- looking for PCL_GEOMETRY
-- looking for PCL_IO
-- looking for PCL_FEATURES
-- looking for PCL_ML
-- looking for PCL_SEGMENTATION
-- looking for PCL_VISUALIZATION
-- looking for PCL_SURFACE
-- looking for PCL_REGISTRATION
-- looking for PCL_KEYPOINTS
-- looking for PCL_TRACKING
-- looking for PCL_RECOGNITION
-- looking for PCL_STEREO
-- looking for PCL_APPS
-- looking for PCL_IN_HAND_SCANNER
-- looking for PCL_POINT_CLOUD_EDITOR
-- looking for PCL_OUTOFCORE
-- looking for PCL_PEOPLE
-- +++ processing catkin package: 'bspline_opt'
-- ==> add_subdirectory(planner/bspline_opt)
-- Eigen found (include: /usr/include/eigen3, version: 3.3.7)
-- The imported target "vtkParseOGLExt" references the file
   "/usr/bin/vtkParseOGLExt-7.1"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "vtkRenderingPythonTkWidgets" references the file
   "/usr/lib/x86_64-linux-gnu/libvtkRenderingPythonTkWidgets.so"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "vtk" references the file
   "/usr/bin/vtk"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "pvtk" references the file
   "/usr/bin/pvtk"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- OpenNI2 found (include: /usr/include/openni2, lib: /usr/lib/libOpenNI2.so)
** WARNING ** io features related to pcap will be disabled
** WARNING ** io features related to png will be disabled
** WARNING ** io features related to libusb-1.0 will be disabled
-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- OpenNI2 found (include: /usr/include/openni2, lib: /usr/lib/libOpenNI2.so)
-- QHULL found (include: /usr/include, lib: optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so)
-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- looking for PCL_COMMON
-- looking for PCL_KDTREE
-- looking for PCL_OCTREE
-- looking for PCL_SEARCH
-- looking for PCL_SAMPLE_CONSENSUS
-- looking for PCL_FILTERS
-- looking for PCL_2D
-- looking for PCL_GEOMETRY
-- looking for PCL_IO
-- looking for PCL_FEATURES
-- looking for PCL_ML
-- looking for PCL_SEGMENTATION
-- looking for PCL_VISUALIZATION
-- looking for PCL_SURFACE
-- looking for PCL_REGISTRATION
-- looking for PCL_KEYPOINTS
-- looking for PCL_TRACKING
-- looking for PCL_RECOGNITION
-- looking for PCL_STEREO
-- looking for PCL_APPS
-- looking for PCL_IN_HAND_SCANNER
-- looking for PCL_POINT_CLOUD_EDITOR
-- looking for PCL_OUTOFCORE
-- looking for PCL_PEOPLE
-- +++ processing catkin package: 'pose_utils'
-- ==> add_subdirectory(uav_simulator/Utils/pose_utils)
-- +++ processing catkin package: 'odom_visualization'
-- ==> add_subdirectory(uav_simulator/Utils/odom_visualization)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- +++ processing catkin package: 'local_sensing_node'
-- ==> add_subdirectory(uav_simulator/local_sensing)
-- Found Boost: /usr/lib/x86_64-linux-gnu/cmake/Boost-1.71.0/BoostConfig.cmake (found version "1.71.0") found components: system filesystem 
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
CMake Warning at /opt/ros/noetic/share/catkin/cmake/catkin_package.cmake:166 (message):
  catkin_package() DEPENDS on 'Eigen' but neither 'Eigen_INCLUDE_DIRS' nor
  'Eigen_LIBRARIES' is defined.
Call Stack (most recent call first):
  /opt/ros/noetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  uav_simulator/local_sensing/CMakeLists.txt:37 (catkin_package)


-- +++ processing catkin package: 'mockamap'
-- ==> add_subdirectory(uav_simulator/mockamap)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- +++ processing catkin package: 'so3_control'
-- ==> add_subdirectory(uav_simulator/so3_control)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- +++ processing catkin package: 'multi_map_server'
-- ==> add_subdirectory(uav_simulator/Utils/multi_map_server)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- multi_map_server: 4 messages, 0 services
-- +++ processing catkin package: 'traj_utils'
-- ==> add_subdirectory(planner/traj_utils)
-- Eigen found (include: /usr/include/eigen3, version: 3.3.7)
-- Found Boost: /usr/include (found suitable version "1.71.0", minimum required is "1.55.0") found components: system filesystem date_time iostreams regex 
-- The imported target "vtkParseOGLExt" references the file
   "/usr/bin/vtkParseOGLExt-7.1"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "vtkRenderingPythonTkWidgets" references the file
   "/usr/lib/x86_64-linux-gnu/libvtkRenderingPythonTkWidgets.so"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "vtk" references the file
   "/usr/bin/vtk"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "pvtk" references the file
   "/usr/bin/pvtk"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- OpenNI2 found (include: /usr/include/openni2, lib: /usr/lib/libOpenNI2.so)
** WARNING ** io features related to pcap will be disabled
** WARNING ** io features related to png will be disabled
** WARNING ** io features related to libusb-1.0 will be disabled
-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- OpenNI2 found (include: /usr/include/openni2, lib: /usr/lib/libOpenNI2.so)
-- QHULL found (include: /usr/include, lib: optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so)
-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- looking for PCL_COMMON
-- looking for PCL_KDTREE
-- looking for PCL_OCTREE
-- looking for PCL_SEARCH
-- looking for PCL_SAMPLE_CONSENSUS
-- looking for PCL_FILTERS
-- looking for PCL_2D
-- looking for PCL_GEOMETRY
-- looking for PCL_IO
-- looking for PCL_FEATURES
-- looking for PCL_ML
-- looking for PCL_SEGMENTATION
-- looking for PCL_VISUALIZATION
-- looking for PCL_SURFACE
-- looking for PCL_REGISTRATION
-- looking for PCL_KEYPOINTS
-- looking for PCL_TRACKING
-- looking for PCL_RECOGNITION
-- looking for PCL_STEREO
-- looking for PCL_APPS
-- looking for PCL_IN_HAND_SCANNER
-- looking for PCL_POINT_CLOUD_EDITOR
-- looking for PCL_OUTOFCORE
-- looking for PCL_PEOPLE
-- +++ processing catkin package: 'ego_planner'
-- ==> add_subdirectory(planner/plan_manage)
-- Eigen found (include: /usr/include/eigen3, version: 3.3.7)
-- The imported target "vtkParseOGLExt" references the file
   "/usr/bin/vtkParseOGLExt-7.1"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "vtkRenderingPythonTkWidgets" references the file
   "/usr/lib/x86_64-linux-gnu/libvtkRenderingPythonTkWidgets.so"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "vtk" references the file
   "/usr/bin/vtk"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- The imported target "pvtk" references the file
   "/usr/bin/pvtk"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   "/usr/lib/cmake/vtk-7.1/VTKTargets.cmake"
but not all the files it references.

-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- OpenNI2 found (include: /usr/include/openni2, lib: /usr/lib/libOpenNI2.so)
** WARNING ** io features related to pcap will be disabled
** WARNING ** io features related to png will be disabled
** WARNING ** io features related to libusb-1.0 will be disabled
-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- OpenNI2 found (include: /usr/include/openni2, lib: /usr/lib/libOpenNI2.so)
-- QHULL found (include: /usr/include, lib: optimized;/usr/lib/x86_64-linux-gnu/libqhull.so;debug;/usr/lib/x86_64-linux-gnu/libqhull.so)
-- OpenNI found (include: /usr/include/ni, lib: /usr/lib/libOpenNI.so)
-- looking for PCL_COMMON
-- looking for PCL_KDTREE
-- looking for PCL_OCTREE
-- looking for PCL_SEARCH
-- looking for PCL_SAMPLE_CONSENSUS
-- looking for PCL_FILTERS
-- looking for PCL_2D
-- looking for PCL_GEOMETRY
-- looking for PCL_IO
-- looking for PCL_FEATURES
-- looking for PCL_ML
-- looking for PCL_SEGMENTATION
-- looking for PCL_VISUALIZATION
-- looking for PCL_SURFACE
-- looking for PCL_REGISTRATION
-- looking for PCL_KEYPOINTS
-- looking for PCL_TRACKING
-- looking for PCL_RECOGNITION
-- looking for PCL_STEREO
-- looking for PCL_APPS
-- looking for PCL_IN_HAND_SCANNER
-- looking for PCL_POINT_CLOUD_EDITOR
-- looking for PCL_OUTOFCORE
-- looking for PCL_PEOPLE
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- ego_planner: 2 messages, 0 services
-- +++ processing catkin package: 'uav_utils'
-- ==> add_subdirectory(uav_simulator/Utils/uav_utils)
-- Found Eigen: /usr/include/eigen3 (found version "3.3.7") 
-- +++ processing catkin package: 'so3_quadrotor_simulator'
-- ==> add_subdirectory(uav_simulator/so3_quadrotor_simulator)
CMake Warning at /opt/ros/noetic/share/catkin/cmake/catkin_package.cmake:166 (message):
  catkin_package() DEPENDS on 'Eigen3' but neither 'Eigen3_INCLUDE_DIRS' nor
  'Eigen3_LIBRARIES' is defined.
Call Stack (most recent call first):
  /opt/ros/noetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  uav_simulator/so3_quadrotor_simulator/CMakeLists.txt:29 (catkin_package)


CMake Warning at /opt/ros/noetic/share/catkin/cmake/catkin_package.cmake:166 (message):
  catkin_package() DEPENDS on 'system_lib' but neither
  'system_lib_INCLUDE_DIRS' nor 'system_lib_LIBRARIES' is defined.
Call Stack (most recent call first):
  /opt/ros/noetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  uav_simulator/so3_quadrotor_simulator/CMakeLists.txt:29 (catkin_package)


-- +++ processing catkin package: 'rviz_plugins'
-- ==> add_subdirectory(uav_simulator/Utils/rviz_plugins)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
CMake Warning at /opt/ros/noetic/share/catkin/cmake/catkin_package.cmake:166 (message):
  catkin_package() DEPENDS on 'system_lib' but neither
  'system_lib_INCLUDE_DIRS' nor 'system_lib_LIBRARIES' is defined.
Call Stack (most recent call first):
  /opt/ros/noetic/share/catkin/cmake/catkin_package.cmake:102 (_catkin_package)
  uav_simulator/Utils/rviz_plugins/CMakeLists.txt:12 (catkin_package)


-- Using Qt5 based on the rviz_QT_VERSION: 5.12.8
-- +++ processing catkin package: 'waypoint_generator'
-- ==> add_subdirectory(uav_simulator/Utils/waypoint_generator)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- Configuring done
CMake Warning (dev) at uav_simulator/Utils/multi_map_server/CMakeLists.txt:167 (add_dependencies):
  Policy CMP0046 is not set: Error on non-existent dependency in
  add_dependencies.  Run "cmake --help-policy CMP0046" for policy details.
  Use the cmake_policy command to set the policy and suppress this warning.

  The dependency target "multi_map_server_messages_cpp" of target
  "multi_map_visualization" does not exist.
This warning is for project developers.  Use -Wno-dev to suppress it.

-- Generating done
-- Build files have been written to: /home/developer/ros_ws/ego-planner/build
####
#### Running command: "make -j32 -l32" in "/home/developer/ros_ws/ego-planner/build"
####
[  0%] Built target nav_msgs_generate_messages_nodejs
[  0%] Built target geometry_msgs_generate_messages_nodejs
[  0%] Built target geometry_msgs_generate_messages_py
[  1%] Built target nav_msgs_generate_messages_cpp
[  1%] Built target geometry_msgs_generate_messages_cpp
[  1%] Built target geometry_msgs_generate_messages_lisp
[  1%] Built target nav_msgs_generate_messages_eus
[  1%] Built target nav_msgs_generate_messages_lisp
[  3%] Built target pose_utils
[  1%] Built target geometry_msgs_generate_messages_eus
[  1%] Built target local_sensing_node_gencfg
[  1%] Built target nav_msgs_generate_messages_py
[  4%] Built target SO3Control
[  4%] Linking CXX shared library /home/developer/ros_ws/ego-planner/devel/lib/libdepth_render_cuda.so
[  5%] Built target random_forest
[  6%] Built target plan_env
[  9%] Built target mockamap_node
[  9%] Built target std_msgs_generate_messages_eus
[  9%] Built target std_msgs_generate_messages_nodejs
[  9%] Built target std_msgs_generate_messages_cpp
[  9%] Built target std_msgs_generate_messages_py
[  9%] Built target std_msgs_generate_messages_lisp
[  9%] Built target _quadrotor_msgs_generate_messages_check_deps_LQRTrajectory
[ 11%] Built target _quadrotor_msgs_generate_messages_check_deps_Serial
[ 12%] Built target quadrotor_dynamics
[ 12%] Built target waypoint_generator
[ 12%] Built target _quadrotor_msgs_generate_messages_check_deps_Gains
[ 12%] Built target _quadrotor_msgs_generate_messages_check_deps_SO3Command
[ 12%] Built target _quadrotor_msgs_generate_messages_check_deps_StatusData
[ 12%] Built target _quadrotor_msgs_generate_messages_check_deps_PositionCommand
[ 12%] Built target _quadrotor_msgs_generate_messages_check_deps_Corrections
[ 12%] Built target _quadrotor_msgs_generate_messages_check_deps_PPROutputData
[ 12%] Built target _quadrotor_msgs_generate_messages_check_deps_Odometry
[ 12%] Built target _quadrotor_msgs_generate_messages_check_deps_TRPYCommand
[ 12%] Built target _quadrotor_msgs_generate_messages_check_deps_OutputData
[ 12%] Built target _quadrotor_msgs_generate_messages_check_deps_AuxCommand
[ 12%] Built target _multi_map_server_generate_messages_check_deps_MultiOccupancyGrid
[ 12%] Built target _quadrotor_msgs_generate_messages_check_deps_PolynomialTrajectory
[ 12%] Built target _multi_map_server_generate_messages_check_deps_VerticalOccupancyGridList
[ 12%] Built target _multi_map_server_generate_messages_check_deps_SparseMap3D
[ 12%] Built target _multi_map_server_generate_messages_check_deps_MultiSparseMap3D
[ 12%] Built target _ego_planner_generate_messages_check_deps_Bspline
[ 21%] Built target quadrotor_msgs_generate_messages_lisp
[ 28%] Built target quadrotor_msgs_generate_messages_py
[ 35%] Built target quadrotor_msgs_generate_messages_cpp
[ 35%] Built target _ego_planner_generate_messages_check_deps_DataDisp
[ 47%] Built target quadrotor_msgs_generate_messages_eus
[ 50%] Built target quadrotor_msgs_generate_messages_nodejs
[ 53%] Built target path_searching
[ 55%] Built target multi_map_server_generate_messages_nodejs
[ 60%] Built target multi_map_server_generate_messages_lisp
[ 63%] Built target multi_map_server_generate_messages_cpp
[ 63%] Built target multi_map_server_generate_messages_py
[ 66%] Built target multi_map_server_generate_messages_eus
[ 69%] Built target ego_planner_generate_messages_nodejs
[ 71%] Built target ego_planner_generate_messages_py
[ 71%] Built target encode_msgs
[ 71%] Built target ego_planner_generate_messages_lisp
[ 73%] Built target ego_planner_generate_messages_eus
[ 74%] Built target ego_planner_generate_messages_cpp
[ 75%] Built target decode_msgs
[ 75%] Built target quadrotor_msgs_generate_messages
[ 75%] Built target multi_map_server_generate_messages
[ 75%] Built target ego_planner_generate_messages
[ 76%] Built target depth_render_cuda
[ 77%] Built target multi_map_visualization
[ 78%] Built target odom_visualization
[ 81%] Built target quadrotor_simulator_so3
[ 85%] Built target bspline_opt
[ 85%] Built target test_topo_mppi
[ 87%] Built target control_example
[ 87%] Built target so3_control_nodelet
[ 94%] Built target rviz_plugins
[ 96%] Linking CXX executable /home/developer/ros_ws/ego-planner/devel/lib/local_sensing_node/pcl_render_node
[ 96%] Built target traj_utils
[ 97%] Built target traj_server
[100%] Built target ego_planner_node
[100%] Built target pcl_render_node
developer@65abafec5dc5:~/ros_ws/ego-planner$ source devel/setup.bash
developer@65abafec5dc5:~/ros_ws/ego-planner$ roslaunch ego_planner simple_run.launch
... logging to /home/developer/.ros/log/57881e9e-9f22-11f0-b52e-e6bc35a3a626/roslaunch-65abafec5dc5-980.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://65abafec5dc5:40217/

SUMMARY
========

PARAMETERS
 * /ego_planner_node/bspline/limit_acc: 3.0
 * /ego_planner_node/bspline/limit_ratio: 1.1
 * /ego_planner_node/bspline/limit_vel: 2.0
 * /ego_planner_node/fsm/emergency_time_: 1.0
 * /ego_planner_node/fsm/flight_type: 2
 * /ego_planner_node/fsm/planning_horizen_time: 3.0
 * /ego_planner_node/fsm/planning_horizon: 7.5
 * /ego_planner_node/fsm/thresh_no_replan: 2.0
 * /ego_planner_node/fsm/thresh_replan: 1.5
 * /ego_planner_node/fsm/waypoint0_x: -15.0
 * /ego_planner_node/fsm/waypoint0_y: 0.0
 * /ego_planner_node/fsm/waypoint0_z: 1.0
 * /ego_planner_node/fsm/waypoint1_x: 0.0
 * /ego_planner_node/fsm/waypoint1_y: 15.0
 * /ego_planner_node/fsm/waypoint1_z: 1.0
 * /ego_planner_node/fsm/waypoint2_x: 15.0
 * /ego_planner_node/fsm/waypoint2_y: 0.0
 * /ego_planner_node/fsm/waypoint2_z: 1.0
 * /ego_planner_node/fsm/waypoint3_x: 0.0
 * /ego_planner_node/fsm/waypoint3_y: -15.0
 * /ego_planner_node/fsm/waypoint3_z: 1.0
 * /ego_planner_node/fsm/waypoint4_x: -15.0
 * /ego_planner_node/fsm/waypoint4_y: 0.0
 * /ego_planner_node/fsm/waypoint4_z: 1.0
 * /ego_planner_node/fsm/waypoint_num: 5
 * /ego_planner_node/grid_map/cx: 321.04638671875
 * /ego_planner_node/grid_map/cy: 243.44969177246094
 * /ego_planner_node/grid_map/depth_filter_margin: 1
 * /ego_planner_node/grid_map/depth_filter_maxdist: 5.0
 * /ego_planner_node/grid_map/depth_filter_mindist: 0.2
 * /ego_planner_node/grid_map/depth_filter_tolerance: 0.15
 * /ego_planner_node/grid_map/frame_id: world
 * /ego_planner_node/grid_map/fx: 387.229248046875
 * /ego_planner_node/grid_map/fy: 387.229248046875
 * /ego_planner_node/grid_map/ground_height: -0.01
 * /ego_planner_node/grid_map/k_depth_scaling_factor: 1000.0
 * /ego_planner_node/grid_map/local_map_margin: 30
 * /ego_planner_node/grid_map/local_update_range_x: 5.5
 * /ego_planner_node/grid_map/local_update_range_y: 5.5
 * /ego_planner_node/grid_map/local_update_range_z: 4.5
 * /ego_planner_node/grid_map/map_size_x: 40.0
 * /ego_planner_node/grid_map/map_size_y: 40.0
 * /ego_planner_node/grid_map/map_size_z: 3.0
 * /ego_planner_node/grid_map/max_ray_length: 4.5
 * /ego_planner_node/grid_map/min_ray_length: 0.1
 * /ego_planner_node/grid_map/obstacles_inflation: 0.099
 * /ego_planner_node/grid_map/p_hit: 0.65
 * /ego_planner_node/grid_map/p_max: 0.9
 * /ego_planner_node/grid_map/p_min: 0.12
 * /ego_planner_node/grid_map/p_miss: 0.35
 * /ego_planner_node/grid_map/p_occ: 0.8
 * /ego_planner_node/grid_map/pose_type: 2
 * /ego_planner_node/grid_map/resolution: 0.1
 * /ego_planner_node/grid_map/show_occ_time: False
 * /ego_planner_node/grid_map/skip_pixel: 2
 * /ego_planner_node/grid_map/use_depth_filter: True
 * /ego_planner_node/grid_map/virtual_ceil_height: 2.5
 * /ego_planner_node/grid_map/visualization_truncate_height: 2.4
 * /ego_planner_node/manager/control_points_distance: 0.4
 * /ego_planner_node/manager/feasibility_tolerance: 0.05
 * /ego_planner_node/manager/max_acc: 3.0
 * /ego_planner_node/manager/max_jerk: 4.0
 * /ego_planner_node/manager/max_vel: 2.0
 * /ego_planner_node/manager/planning_horizon: 7.5
 * /ego_planner_node/manager/use_parallel_mppi_optimization: True
 * /ego_planner_node/optimization/dist0: 0.5
 * /ego_planner_node/optimization/lambda_collision: 0.5
 * /ego_planner_node/optimization/lambda_feasibility: 0.1
 * /ego_planner_node/optimization/lambda_fitness: 1.0
 * /ego_planner_node/optimization/lambda_smooth: 1.0
 * /ego_planner_node/optimization/max_acc: 3.0
 * /ego_planner_node/optimization/max_vel: 2.0
 * /ego_planner_node/topo_prm/max_topo_paths: 5
 * /ego_planner_node/topo_prm/use_tgk_algorithm: True
 * /mockamap_node/attenuation: 0.1
 * /mockamap_node/complexity: 0.05
 * /mockamap_node/fill: 0.12
 * /mockamap_node/fractal: 1
 * /mockamap_node/resolution: 0.1
 * /mockamap_node/seed: 127
 * /mockamap_node/type: 1
 * /mockamap_node/update_freq: 0.5
 * /mockamap_node/x_length: 40.0
 * /mockamap_node/y_length: 40.0
 * /mockamap_node/z_length: 3.0
 * /odom_visualization/color/a: 1.0
 * /odom_visualization/color/b: 0.0
 * /odom_visualization/color/g: 0.0
 * /odom_visualization/color/r: 0.0
 * /odom_visualization/covariance_scale: 100.0
 * /odom_visualization/robot_scale: 1.0
 * /odom_visualization/tf45: True
 * /pcl_render_node/cam_cx: 321.04638671875
 * /pcl_render_node/cam_cy: 243.44969177246094
 * /pcl_render_node/cam_fx: 387.229248046875
 * /pcl_render_node/cam_fy: 387.229248046875
 * /pcl_render_node/cam_height: 480
 * /pcl_render_node/cam_width: 640
 * /pcl_render_node/estimation_rate: 30.0
 * /pcl_render_node/map/x_size: 40.0
 * /pcl_render_node/map/y_size: 40.0
 * /pcl_render_node/map/z_size: 3.0
 * /pcl_render_node/sensing_horizon: 5.0
 * /pcl_render_node/sensing_rate: 30.0
 * /quadrotor_simulator_so3/rate/odom: 200.0
 * /quadrotor_simulator_so3/simulator/init_state_x: -18.0
 * /quadrotor_simulator_so3/simulator/init_state_y: 0.0
 * /quadrotor_simulator_so3/simulator/init_state_z: 0.0
 * /rosdistro: noetic
 * /rosversion: 1.17.4
 * /so3_control/corrections/p: 0.0
 * /so3_control/corrections/r: 0.0
 * /so3_control/corrections/z: 0.0
 * /so3_control/gains/ang/x: 0.07
 * /so3_control/gains/ang/y: 0.07
 * /so3_control/gains/ang/z: 0.1
 * /so3_control/gains/pos/x: 2.0
 * /so3_control/gains/pos/y: 2.0
 * /so3_control/gains/pos/z: 3.5
 * /so3_control/gains/rot/x: 1.0
 * /so3_control/gains/rot/y: 1.0
 * /so3_control/gains/rot/z: 1.0
 * /so3_control/gains/vel/x: 1.8
 * /so3_control/gains/vel/y: 1.8
 * /so3_control/gains/vel/z: 2.0
 * /so3_control/mass: 0.98
 * /so3_control/so3_control/init_state_x: -18.0
 * /so3_control/so3_control/init_state_y: 0.0
 * /so3_control/so3_control/init_state_z: 0.0
 * /so3_control/use_angle_corrections: False
 * /so3_control/use_external_yaw: False
 * /traj_server/traj_server/time_forward: 1.0
 * /waypoint_generator/waypoint_type: manual-lonely-way...

NODES
  /
    ego_planner_node (ego_planner/ego_planner_node)
    mockamap_node (mockamap/mockamap_node)
    odom_visualization (odom_visualization/odom_visualization)
    pcl_render_node (local_sensing_node/pcl_render_node)
    quadrotor_simulator_so3 (so3_quadrotor_simulator/quadrotor_simulator_so3)
    rviz (rviz/rviz)
    so3_control (nodelet/nodelet)
    traj_server (ego_planner/traj_server)
    waypoint_generator (waypoint_generator/waypoint_generator)

auto-starting new master
process[master]: started with pid [988]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 57881e9e-9f22-11f0-b52e-e6bc35a3a626
process[rosout-1]: started with pid [998]
started core service [/rosout]
process[ego_planner_node-2]: started with pid [1005]
process[traj_server-3]: started with pid [1006]
process[waypoint_generator-4]: started with pid [1007]
process[mockamap_node-5]: started with pid [1009]
process[quadrotor_simulator_so3-6]: started with pid [1015]
process[so3_control-7]: started with pid [1021]
process[odom_visualization-8]: started with pid [1026]
type is so3_control/SO3ControlNodelet
process[pcl_render_node-9]: started with pid [1032]
process[rviz-10]: started with pid [1038]
[INFO] [1759363029.772316122]: [PlannerManager] Parallel MPPI optimization: ENABLED
QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-developer'
hit: 0.619039
miss: -0.619039
min log: -1.99243
max: 2.19722
thresh log: 1.38629
[INFO] [1759363029.817717323]: [MPPI] Initialized with 1000 samples, horizon 20 steps, dt 0.100
[INFO] [1759363029.817741277]: [MPPI] Initialized publishers on topics '/mppi_trajectories' and '/mppi_optimal_trajectory'
[INFO] [1759363029.817746868]: [MPPI] Using frame_id: world
[INFO] [1759363029.818581718]: [BiasSampler] Initialized with:
[INFO] [1759363029.818588270]:   - Corner detection radius: 3.00
[INFO] [1759363029.818593039]:   - Sampling radius: 2.00
[INFO] [1759363029.818597678]:   - Resolution: 0.10
[INFO] [1759363029.818604350]: [TopoGraphSearch] Initialized
[INFO] [1759363029.818608768]: [TopoPRM] Initialized publisher on topic '/topo_paths'
[INFO] [1759363029.818616353]: [TopoPRM] Initialized with step_size: 0.200000, search_radius: 5.000000, frame_id: world
[INFO] [1759363029.818643353]: [TopoPRM] ???? TGK algorithm: ENABLED
[INFO] [1759363029.818652019]: [TopoPRM] ???? Maximum topological paths: 5
[INFO] [1759363029.818989981]: [MPPI] Initialized with 1000 samples, horizon 20 steps, dt 0.100
[INFO] [1759363029.819009587]: [MPPI] Initialized publishers on topics '/mppi_trajectories' and '/mppi_optimal_trajectory'
[INFO] [1759363029.819018334]: [MPPI] Using frame_id: world
[INFO] [1759363029.819028773]: [PlannerManager] Initialized topological and MPPI planners
[INFO] [1759363030.235632079]: threshold: 0.066194
[INFO] [1759363030.396052432]: the number of points before optimization is 575999
[INFO] [1759363030.398283882]: finish: infill 0.120000%
[WARN] [1759363030.410089610]: Global Pointcloud received..
global map has points: 575999.
[WARN] [1759363030.496564017]: WARNING: package://odom_visualization/meshes/hummingbird.mesh is an older format ([MeshSerializer_v1.41]); you should upgrade it as soon as possible using the OgreMeshUpgrade tool.
[WARN] [1759363030.497175649]: Can't assign material _missing_material_ to SubEntity of mesh_resource_marker_0 because this Material does not exist. Have you forgotten to define it in a .material script?
[WARN] [1759363030.748138079]: [Traj server]: ready.
[TRIG]: from INIT to GEN_NEW_TRAJ

[rebo replan]: -------------------------------------0
start: -18   0   0, 0 0 0
goal:-10.9     2  1.61,    1.38    1.42 -0.0438
[INFO] [1759363031.832923747]: [PlannerManager] Attempting topological planning from [-18.00, 0.00, 0.00] to [-10.91, 2.00, 1.61]
[INFO] [1759363031.832949084]: [TopoPRM] Searching topological paths from [-18.00, 0.00, 0.00] to [-10.91, 2.00, 1.61]
[INFO] [1759363031.832959193]: [TopoPRM] ???? Using TGK algorithm for topological planning
[INFO] [1759363031.832968140]: [TopoGraphSearch] Searching multiple topological paths
[WARN] [1759363031.833011631]: [TGK Corner] ??????2????????????????????????: free=8, occupied=0 | ??????: ??????=100, ??????1=100, ??????2=100
[INFO] [1759363031.833027090]: [TGK Corner] ??? ??????corner point! pos=(-19.84,1.84,0.00), free=5, occupied=3 | ?????????: 113/113/2/1 (???/???1/???2/???3)
[INFO] [1759363031.833042208]: [TGK Corner] ??? ??????corner point! pos=(-19.91,1.91,0.00), free=5, occupied=3 | ?????????: 114/114/3/2 (???/???1/???2/???3)
[INFO] [1759363031.833055002]: [TGK Corner] ??? ??????corner point! pos=(-19.98,1.98,0.00), free=5, occupied=3 | ?????????: 115/115/4/3 (???/???1/???2/???3)
[INFO] [1759363031.833069039]: [TGK Corner] ??? ??????corner point! pos=(-19.80,0.00,0.00), free=5, occupied=3 | ?????????: 134/133/6/4 (???/???1/???2/???3)
[INFO] [1759363031.833079248]: [TGK Corner] ??? ??????corner point! pos=(-19.90,0.00,0.00), free=5, occupied=3 | ?????????: 135/134/7/5 (???/???1/???2/???3)
[INFO] [1759363031.833097432]: [TGK Corner] ??? ??????corner point! pos=(-19.84,-1.84,0.00), free=5, occupied=3 | ?????????: 171/160/9/6 (???/???1/???2/???3)
[INFO] [1759363031.833106609]: [TGK Corner] ??? ??????corner point! pos=(-19.91,-1.91,0.00), free=5, occupied=3 | ?????????: 172/161/10/7 (???/???1/???2/???3)
[INFO] [1759363031.833113341]: [TGK Corner] ??? ??????corner point! pos=(-19.98,-1.98,0.00), free=5, occupied=3 | ?????????: 173/162/11/8 (???/???1/???2/???3)
[INFO] [1759363031.833151994]: [TGK Corner] ??? ??????corner point! pos=(-19.81,1.94,0.02), free=5, occupied=3 | ?????????: 406/364/13/9 (???/???1/???2/???3)
[INFO] [1759363031.833162072]: [TGK Corner] ??? ??????corner point! pos=(-19.89,2.01,0.02), free=5, occupied=3 | ?????????: 407/365/14/10 (???/???1/???2/???3)
[INFO] [1759363031.833171911]: [TGK Corner] ??? ??????corner point! pos=(-19.96,2.08,0.02), free=5, occupied=3 | ?????????: 408/366/15/11 (???/???1/???2/???3)
[INFO] [1759363031.833183492]: [TGK Corner] ??? ??????corner point! pos=(-19.81,0.03,0.02), free=5, occupied=3 | ?????????: 427/385/17/12 (???/???1/???2/???3)
[INFO] [1759363031.833192068]: [TGK Corner] ??? ??????corner point! pos=(-19.91,0.03,0.02), free=5, occupied=3 | ?????????: 428/386/18/13 (???/???1/???2/???3)
[INFO] [1759363031.833206886]: [TGK Corner] ??? ??????corner point! pos=(-19.81,-1.88,0.02), free=5, occupied=3 | ?????????: 464/413/20/14 (???/???1/???2/???3)
[INFO] [1759363031.833215392]: [TGK Corner] ??? ??????corner point! pos=(-19.89,-1.95,0.02), free=5, occupied=3 | ?????????: 465/414/21/15 (???/???1/???2/???3)
[INFO] [1759363031.833224780]: [TGK Corner] ??? ??????corner point! pos=(-19.96,-2.02,0.02), free=5, occupied=3 | ?????????: 466/415/22/16 (???/???1/???2/???3)
[INFO] [1759363031.833261388]: [TGK Corner] ??? ??????corner point! pos=(-19.79,2.03,0.04), free=5, occupied=3 | ?????????: 699/618/24/17 (???/???1/???2/???3)
[INFO] [1759363031.833271337]: [TGK Corner] ??? ??????corner point! pos=(-19.86,2.10,0.04), free=5, occupied=3 | ?????????: 700/619/25/18 (???/???1/???2/???3)
[INFO] [1759363031.833284140]: [TGK Corner] ??? ??????corner point! pos=(-19.81,0.05,0.04), free=5, occupied=3 | ?????????: 720/639/27/19 (???/???1/???2/???3)
[INFO] [1759363031.833292656]: [TGK Corner] ??? ??????corner point! pos=(-19.91,0.05,0.04), free=5, occupied=3 | ?????????: 721/640/28/20 (???/???1/???2/???3)
[INFO] [1759363031.833307073]: [TGK Corner] ??? ??????corner point! pos=(-19.79,-1.93,0.04), free=5, occupied=3 | ?????????: 757/668/30/21 (???/???1/???2/???3)
[INFO] [1759363031.833316571]: [TGK Corner] ??? ??????corner point! pos=(-19.86,-2.00,0.04), free=5, occupied=3 | ?????????: 758/669/31/22 (???/???1/???2/???3)
[INFO] [1759363031.833354382]: [TGK Corner] ??? ??????corner point! pos=(-19.82,0.08,0.06), free=5, occupied=3 | ?????????: 1013/894/34/23 (???/???1/???2/???3)
[INFO] [1759363031.833363889]: [TGK Corner] ??? ??????corner point! pos=(-19.92,0.08,0.06), free=5, occupied=3 | ?????????: 1014/895/35/24 (???/???1/???2/???3)
[INFO] [1759363031.833408353]: [TGK Corner] ??? ??????corner point! pos=(-19.82,0.11,0.09), free=5, occupied=3 | ?????????: 1306/1150/38/25 (???/???1/???2/???3)
[INFO] [1759363031.833417890]: [TGK Corner] ??? ??????corner point! pos=(-19.92,0.11,0.09), free=5, occupied=3 | ?????????: 1307/1151/39/26 (???/???1/???2/???3)
[INFO] [1759363031.833464467]: [TGK Corner] ??? ??????corner point! pos=(-19.83,0.13,0.11), free=5, occupied=3 | ?????????: 1599/1407/41/27 (???/???1/???2/???3)
[INFO] [1759363031.833473244]: [TGK Corner] ??? ??????corner point! pos=(-19.93,0.13,0.11), free=5, occupied=3 | ?????????: 1600/1408/42/28 (???/???1/???2/???3)
[WARN] [1759363031.833498461]: [TGK Corner] ??????: ??????=1700, ?????????1=1495 ?????????2=42 ?????????3=28 | ?????????1=205 ?????????2=1453 ?????????3=14
[INFO] [1759363031.833532414]: [TGK Corner] ??? ??????corner point! pos=(-19.83,0.16,0.13), free=5, occupied=3 | ?????????: 1892/1665/44/29 (???/???1/???2/???3)
[INFO] [1759363031.833542533]: [TGK Corner] ??? ??????corner point! pos=(-19.93,0.16,0.13), free=5, occupied=3 | ?????????: 1893/1666/45/30 (???/???1/???2/???3)
[INFO] [1759363031.833585273]: [TGK Corner] ??? ??????corner point! pos=(-19.84,0.19,0.15), free=5, occupied=3 | ?????????: 2185/1924/47/31 (???/???1/???2/???3)
[INFO] [1759363031.833594360]: [TGK Corner] ??? ??????corner point! pos=(-19.94,0.19,0.15), free=5, occupied=3 | ?????????: 2186/1925/48/32 (???/???1/???2/???3)
[INFO] [1759363031.833639524]: [TGK Corner] ??? ??????corner point! pos=(-19.84,0.21,0.17), free=5, occupied=3 | ?????????: 2478/2184/50/33 (???/???1/???2/???3)
[INFO] [1759363031.833648632]: [TGK Corner] ??? ??????corner point! pos=(-19.94,0.21,0.17), free=5, occupied=3 | ?????????: 2479/2185/51/34 (???/???1/???2/???3)
[INFO] [1759363031.833693826]: [TGK Corner] ??? ??????corner point! pos=(-19.85,0.24,0.19), free=5, occupied=3 | ?????????: 2771/2445/53/35 (???/???1/???2/???3)
[INFO] [1759363031.833703815]: [TGK Corner] ??? ??????corner point! pos=(-19.95,0.24,0.19), free=5, occupied=3 | ?????????: 2772/2446/54/36 (???/???1/???2/???3)
[INFO] [1759363031.833746835]: [TGK Corner] ??? ??????corner point! pos=(-19.85,0.27,0.22), free=5, occupied=3 | ?????????: 3064/2707/56/37 (???/???1/???2/???3)
[INFO] [1759363031.833755461]: [TGK Corner] ??? ??????corner point! pos=(-19.95,0.27,0.22), free=5, occupied=3 | ?????????: 3065/2708/57/38 (???/???1/???2/???3)
[INFO] [1759363031.833797910]: [TGK Corner] ??? ??????corner point! pos=(-19.86,0.29,0.24), free=5, occupied=3 | ?????????: 3357/2970/59/39 (???/???1/???2/???3)
[INFO] [1759363031.835899958]: [TopoGraphSearch] Building graph with 7 key points
[INFO] [1759363031.835909105]: [TopoGraphSearch] Graph built with 9 nodes
[INFO] [1759363031.835922320]: [TopoGraphSearch] A* search: 9 nodes, start_id=0, goal_id=8
[INFO] [1759363031.835936136]: [TopoGraphSearch] A* found path in 2 iterations
[INFO] [1759363031.835946054]: [TopoGraphSearch] Path 1: 2 waypoints
[INFO] [1759363031.835952005]: [TopoGraphSearch] Generated 1 topological paths
[INFO] [1759363031.835958578]: [TopoGraphSearch] Found 1 topological paths
[INFO] [1759363031.835965971]: [TopoPRM-TGK] Found 1 topological paths
[INFO] [1759363031.836132453]: [TopoPRM] Generated 1 candidate paths
[INFO] [1759363031.836141740]: [TopoPRM] Visualizing 1 topological paths with frame_id: world
[INFO] [1759363031.836153322]: [TopoPRM] About to publish MarkerArray with 2 markers
[INFO] [1759363031.836167428]: [TopoPRM] Publisher has 1 subscribers
[INFO] [1759363031.846253874]: [TopoPRM] Published MarkerArray with 2 markers to topic '/topo_paths'
[INFO] [1759363031.846278971]: [TopoPRM] Found 1 topological paths
[INFO] [1759363031.846303497]: [PlannerManager] Topological planning succeeded, found 1 paths
[INFO] [1759363031.846325608]: [PlannerManager] Found 1 topological paths
[INFO] [1759363031.846349573]: [PlannerManager] Using topological path with cost 27.544
[INFO] [1759363031.846610881]: [PlannerManager] Successfully integrated topological path with 38 waypoints
[INFO] [1759363031.846651297]: [PlannerManager] Skipping STEP 2: Parallel MPPI already applied in STEP 1.5
iter(+1)=41,time(ms)=0.097,total_t(ms)=0.097,cost=0.014
bspline_optimize_success=1
total time:0.000221,optimize:0.000219,refine:2.41e-06
final_plan_success=1
[FSM]: from GEN_NEW_TRAJ to EXEC_TRAJ
^C[rviz-10] killing on exit
[pcl_render_node-9] killing on exit
[odom_visualization-8] killing on exit
[so3_control-7] killing on exit
[quadrotor_simulator_so3-6] killing on exit
[mockamap_node-5] killing on exit
[waypoint_generator-4] killing on exit
[traj_server-3] killing on exit
[ego_planner_node-2] killing on exit
terminate called after throwing an instance of 'boost::wrapexcept<boost::lock_error>'
  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
des manager
[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
developer@65abafec5dc5:~/ros_ws/ego-planner$ 
