# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(autonomos_gazebo_CONFIG_INCLUDED)
  return()
endif()
set(autonomos_gazebo_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(autonomos_gazebo_SOURCE_PREFIX /home/rriverase/Documents/AutoNOMOS/src/autonomos_gazebo)
  set(autonomos_gazebo_DEVEL_PREFIX /home/rriverase/Documents/AutoNOMOS/src/Gazebo_plugin/devel)
  set(autonomos_gazebo_INSTALL_PREFIX "")
  set(autonomos_gazebo_PREFIX ${autonomos_gazebo_DEVEL_PREFIX})
else()
  set(autonomos_gazebo_SOURCE_PREFIX "")
  set(autonomos_gazebo_DEVEL_PREFIX "")
  set(autonomos_gazebo_INSTALL_PREFIX /usr/local)
  set(autonomos_gazebo_PREFIX ${autonomos_gazebo_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'autonomos_gazebo' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(autonomos_gazebo_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/usr/include;/usr/include/pcl-1.8;/usr/include/eigen3;/usr/include/ni;/usr/include/vtk;/usr/include/freetype2;/usr/include/python2.7;/usr/include/libxml2 " STREQUAL " ")
  set(autonomos_gazebo_INCLUDE_DIRS "")
  set(_include_dirs "/usr/include;/usr/include/pcl-1.8;/usr/include/eigen3;/usr/include/ni;/usr/include/vtk;/usr/include/freetype2;/usr/include/python2.7;/usr/include/libxml2")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Gary <gary.granados@gmail.com>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${autonomos_gazebo_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'autonomos_gazebo' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'autonomos_gazebo' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/usr/local/${idir}'.  ${_report}")
    endif()
    _list_append_unique(autonomos_gazebo_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "pcl_ros_filters;pcl_ros_io;pcl_ros_tf;optimized;/usr/lib64/libpcl_common.so;debug;/usr/lib64/libpcl_common.so;optimized;/usr/lib64/libpcl_octree.so;debug;/usr/lib64/libpcl_octree.so;optimized;/usr/lib64/libpcl_io.so;debug;/usr/lib64/libpcl_io.so;optimized;/usr/lib64/libpcl_kdtree.so;debug;/usr/lib64/libpcl_kdtree.so;optimized;/usr/lib64/libpcl_search.so;debug;/usr/lib64/libpcl_search.so;optimized;/usr/lib64/libpcl_sample_consensus.so;debug;/usr/lib64/libpcl_sample_consensus.so;optimized;/usr/lib64/libpcl_filters.so;debug;/usr/lib64/libpcl_filters.so;optimized;/usr/lib64/libpcl_features.so;debug;/usr/lib64/libpcl_features.so;optimized;/usr/lib64/libpcl_ml.so;debug;/usr/lib64/libpcl_ml.so;optimized;/usr/lib64/libpcl_segmentation.so;debug;/usr/lib64/libpcl_segmentation.so;optimized;/usr/lib64/libpcl_visualization.so;debug;/usr/lib64/libpcl_visualization.so;optimized;/usr/lib64/libpcl_surface.so;debug;/usr/lib64/libpcl_surface.so;optimized;/usr/lib64/libpcl_registration.so;debug;/usr/lib64/libpcl_registration.so;optimized;/usr/lib64/libpcl_keypoints.so;debug;/usr/lib64/libpcl_keypoints.so;optimized;/usr/lib64/libpcl_tracking.so;debug;/usr/lib64/libpcl_tracking.so;optimized;/usr/lib64/libpcl_recognition.so;debug;/usr/lib64/libpcl_recognition.so;optimized;/usr/lib64/libpcl_stereo.so;debug;/usr/lib64/libpcl_stereo.so;optimized;/usr/lib64/libpcl_apps.so;debug;/usr/lib64/libpcl_apps.so;optimized;/usr/lib64/libpcl_outofcore.so;debug;/usr/lib64/libpcl_outofcore.so;optimized;/usr/lib64/libpcl_people.so;debug;/usr/lib64/libpcl_people.so;/usr/lib64/libboost_system.so;/usr/lib64/libboost_filesystem.so;/usr/lib64/libboost_thread.so;/usr/lib64/libboost_date_time.so;/usr/lib64/libboost_iostreams.so;/usr/lib64/libboost_serialization.so;/usr/lib64/libboost_chrono.so;/usr/lib64/libboost_atomic.so;/usr/lib64/libboost_regex.so;optimized;/usr/lib64/libqhull_p.so;debug;/usr/lib64/libqhull_p.so;/usr/lib64/libOpenNI.so;optimized;/usr/lib64/libflann_cpp.so;debug;/usr/lib64/libflann_cpp.so;/usr/lib64/vtk/libvtkChartsCore.so.1;/usr/lib64/vtk/libvtkCommonColor.so.1;/usr/lib64/vtk/libvtkCommonCore.so.1;/usr/lib64/vtk/libvtksys.so.1;/usr/lib64/vtk/libvtkCommonDataModel.so.1;/usr/lib64/vtk/libvtkCommonMath.so.1;/usr/lib64/vtk/libvtkCommonMisc.so.1;/usr/lib64/vtk/libvtkCommonSystem.so.1;/usr/lib64/vtk/libvtkCommonTransforms.so.1;/usr/lib64/vtk/libvtkCommonExecutionModel.so.1;/usr/lib64/vtk/libvtkFiltersGeneral.so.1;/usr/lib64/vtk/libvtkCommonComputationalGeometry.so.1;/usr/lib64/vtk/libvtkFiltersCore.so.1;/usr/lib64/vtk/libvtkInfovisCore.so.1;/usr/lib64/vtk/libvtkFiltersExtraction.so.1;/usr/lib64/vtk/libvtkFiltersStatistics.so.1;/usr/lib64/vtk/libvtkImagingFourier.so.1;/usr/lib64/vtk/libvtkImagingCore.so.1;/usr/lib64/vtk/libvtkalglib.so.1;/usr/lib64/vtk/libvtkRenderingContext2D.so.1;/usr/lib64/vtk/libvtkRenderingCore.so.1;/usr/lib64/vtk/libvtkFiltersGeometry.so.1;/usr/lib64/vtk/libvtkFiltersSources.so.1;/usr/lib64/vtk/libvtkRenderingFreeType.so.1;/usr/lib64/libfreetype.so;/usr/lib64/libz.so;/usr/lib64/vtk/libvtkDICOMParser.so.1;/usr/lib64/vtk/libvtkDomainsChemistry.so.1;/usr/lib64/vtk/libvtkIOLegacy.so.1;/usr/lib64/vtk/libvtkIOCore.so.1;/usr/lib64/vtk/libvtkIOXMLParser.so.1;/usr/lib64/libexpat.so;/usr/lib64/vtk/libvtkDomainsChemistryOpenGL2.so.1;/usr/lib64/vtk/libvtkRenderingOpenGL2.so.1;/usr/lib64/vtk/libvtkIOImage.so.1;/usr/lib64/vtk/libvtkmetaio.so.1;/usr/lib64/libjpeg.so;/usr/lib64/libpng.so;/usr/lib64/libtiff.so;/usr/lib64/vtk/libvtkglew.so.1;/usr/lib64/vtk/libvtkFiltersAMR.so.1;/usr/lib64/vtk/libvtkIOXML.so.1;/usr/lib64/vtk/libvtkParallelCore.so.1;/usr/lib64/vtk/libvtkFiltersFlowPaths.so.1;/usr/lib64/vtk/libvtkFiltersGeneric.so.1;/usr/lib64/vtk/libvtkFiltersHybrid.so.1;/usr/lib64/vtk/libvtkImagingSources.so.1;/usr/lib64/vtk/libvtkFiltersHyperTree.so.1;/usr/lib64/vtk/libvtkFiltersImaging.so.1;/usr/lib64/vtk/libvtkImagingGeneral.so.1;/usr/lib64/vtk/libvtkFiltersModeling.so.1;/usr/lib64/vtk/libvtkFiltersParallel.so.1;/usr/lib64/vtk/libvtkFiltersParallelImaging.so.1;/usr/lib64/vtk/libvtkFiltersPoints.so.1;/usr/lib64/vtk/libvtkFiltersProgrammable.so.1;/usr/lib64/vtk/libvtkFiltersPython.so.1;/usr/lib64/libpython2.7.so;/usr/lib64/vtk/libvtkWrappingPython27Core.so.1;/usr/lib64/vtk/libvtkWrappingTools.a;/usr/lib64/vtk/libvtkFiltersSMP.so.1;/usr/lib64/vtk/libvtkFiltersSelection.so.1;/usr/lib64/vtk/libvtkFiltersStatisticsGnuR.so.1;/usr/lib64/vtk/libvtkFiltersTexture.so.1;/usr/lib64/vtk/libvtkFiltersVerdict.so.1;/usr/lib64/vtk/libvtkverdict.so.1;/usr/lib64/vtk/libvtkGUISupportQt.so.1;/usr/lib64/vtk/libvtkInteractionStyle.so.1;/usr/lib64/vtk/libvtkGUISupportQtSQL.so.1;/usr/lib64/vtk/libvtkIOSQL.so.1;/usr/lib64/vtk/libvtksqlite.so.1;/usr/lib64/vtk/libvtkGeovisCore.so.1;/usr/lib64/vtk/libvtkInfovisLayout.so.1;/usr/lib64/vtk/libvtkImagingHybrid.so.1;/usr/lib64/vtk/libvtkInteractionWidgets.so.1;/usr/lib64/vtk/libvtkImagingColor.so.1;/usr/lib64/vtk/libvtkRenderingAnnotation.so.1;/usr/lib64/vtk/libvtkRenderingVolume.so.1;/usr/lib64/vtk/libvtkViewsCore.so.1;/usr/lib64/vtk/libvtkproj4.so.1;/usr/lib64/vtk/libvtkIOAMR.so.1;/usr/lib64/libhdf5.so;/usr/lib64/libdl.so;/usr/lib64/libm.so;/usr/lib64/libhdf5_hl.so;/usr/lib64/vtk/libvtkIOEnSight.so.1;/usr/lib64/vtk/libvtkIOExodus.so.1;/usr/lib64/vtk/libvtkexoIIc.so.1;/lib64/libnetcdf_c++.so;/lib64/libnetcdf.so;/usr/lib64/vtk/libvtkIOExport.so.1;/usr/lib64/vtk/libvtkRenderingGL2PSOpenGL2.so.1;/usr/lib64/vtk/libvtkgl2ps.so.1;/usr/lib64/vtk/libvtkIOExportOpenGL2.so.1;/usr/lib64/vtk/libvtkIOGeometry.so.1;/usr/lib64/vtk/libvtkIOImport.so.1;/usr/lib64/vtk/libvtkIOInfovis.so.1;/usr/lib64/libxml2.so;/usr/lib64/vtk/libvtkIOLSDyna.so.1;/usr/lib64/vtk/libvtkIOMINC.so.1;/usr/lib64/vtk/libvtkIOMovie.so.1;/usr/lib64/libtheoraenc.so;/usr/lib64/libtheoradec.so;/usr/lib64/libogg.so;/usr/lib64/vtk/libvtkIONetCDF.so.1;/usr/lib64/vtk/libvtkIOPLY.so.1;/usr/lib64/vtk/libvtkIOParallel.so.1;/usr/lib64/libjsoncpp.so;/usr/lib64/vtk/libvtkIOParallelXML.so.1;/usr/lib64/vtk/libvtkIOTecplotTable.so.1;/usr/lib64/vtk/libvtkIOVideo.so.1;/usr/lib64/vtk/libvtkImagingMath.so.1;/usr/lib64/vtk/libvtkImagingMorphological.so.1;/usr/lib64/vtk/libvtkImagingStatistics.so.1;/usr/lib64/vtk/libvtkImagingStencil.so.1;/usr/lib64/vtk/libvtkInteractionImage.so.1;/usr/lib64/vtk/libvtkLocalExample.so.1;/usr/lib64/vtk/libvtkRenderingContextOpenGL2.so.1;/usr/lib64/vtk/libvtkRenderingImage.so.1;/usr/lib64/vtk/libvtkRenderingLOD.so.1;/usr/lib64/vtk/libvtkRenderingLabel.so.1;/usr/lib64/vtk/libvtkRenderingParallel.so.1;/usr/lib64/vtk/libvtkRenderingQt.so.1;/usr/lib64/vtk/libvtkRenderingVolumeOpenGL2.so.1;/usr/lib64/vtk/libvtkTestingGenericBridge.so.1;/usr/lib64/vtk/libvtkTestingIOSQL.so.1;/usr/lib64/vtk/libvtkTestingRendering.so.1;/usr/lib64/vtk/libvtkViewsContext2D.so.1;/usr/lib64/vtk/libvtkViewsGeovis.so.1;/usr/lib64/vtk/libvtkViewsInfovis.so.1;/usr/lib64/vtk/libvtkViewsQt.so.1;/usr/lib64/vtk/libvtkWrappingJava.so.1")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND autonomos_gazebo_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND autonomos_gazebo_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND autonomos_gazebo_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /usr/local/lib;/opt/ros/kinetic/ros_catkin_ws/install_isolated/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(autonomos_gazebo_LIBRARY_DIRS ${lib_path})
      list(APPEND autonomos_gazebo_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'autonomos_gazebo'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND autonomos_gazebo_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(autonomos_gazebo_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${autonomos_gazebo_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "dynamic_reconfigure;nodelet;pcl_conversions;pcl_msgs;rosbag;roscpp;sensor_msgs;std_msgs;tf")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 autonomos_gazebo_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${autonomos_gazebo_dep}_FOUND)
      find_package(${autonomos_gazebo_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${autonomos_gazebo_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(autonomos_gazebo_INCLUDE_DIRS ${${autonomos_gazebo_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(autonomos_gazebo_LIBRARIES ${autonomos_gazebo_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${autonomos_gazebo_dep}_LIBRARIES})
  _list_append_deduplicate(autonomos_gazebo_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(autonomos_gazebo_LIBRARIES ${autonomos_gazebo_LIBRARIES})

  _list_append_unique(autonomos_gazebo_LIBRARY_DIRS ${${autonomos_gazebo_dep}_LIBRARY_DIRS})
  list(APPEND autonomos_gazebo_EXPORTED_TARGETS ${${autonomos_gazebo_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${autonomos_gazebo_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
