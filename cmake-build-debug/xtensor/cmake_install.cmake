# Install script for directory: /home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/xtensor" TYPE FILE FILES
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xaccessible.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xaccumulator.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xadapt.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xarray.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xassign.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xaxis_iterator.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xaxis_slice_iterator.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xblockwise_reducer.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xblockwise_reducer_functors.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xbroadcast.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xbuffer_adaptor.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xbuilder.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xchunked_array.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xchunked_assign.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xchunked_view.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xcomplex.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xcontainer.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xcsv.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xdynamic_view.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xeval.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xexception.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xexpression.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xexpression_holder.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xexpression_traits.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xfixed.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xfunction.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xfunctor_view.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xgenerator.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xhistogram.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xindex_view.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xinfo.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xio.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xiterable.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xiterator.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xjson.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xlayout.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xmanipulation.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xmasked_view.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xmath.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xmime.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xmultiindex_iterator.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xnoalias.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xnorm.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xnpy.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xoffset_view.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xoperation.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xoptional.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xoptional_assembly.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xoptional_assembly_base.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xoptional_assembly_storage.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xpad.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xrandom.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xreducer.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xrepeat.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xscalar.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xsemantic.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xset_operation.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xshape.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xslice.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xsort.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xstorage.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xstrided_view.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xstrided_view_base.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xstrides.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xtensor.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xtensor_config.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xtensor_forward.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xtensor_simd.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xutils.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xvectorize.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xview.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtensor/include/xtensor/xview_utils.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/xtensor" TYPE FILE FILES
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/xtensor/xtensorConfig.cmake"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/xtensor/xtensorConfigVersion.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/xtensor/xtensorTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/xtensor/xtensorTargets.cmake"
         "/home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/xtensor/CMakeFiles/Export/1b29c11efe53a30bee9d6a40fc6513c6/xtensorTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/xtensor/xtensorTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/xtensor/xtensorTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/xtensor" TYPE FILE FILES "/home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/xtensor/CMakeFiles/Export/1b29c11efe53a30bee9d6a40fc6513c6/xtensorTargets.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/xtensor/xtensor.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "/home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/xtensor.hpp")
endif()

