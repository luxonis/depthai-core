# Install script for directory: /home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/xtl" TYPE FILE FILES
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xany.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xbasic_fixed_string.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xbase64.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xclosure.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xcomplex.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xcomplex_sequence.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xspan.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xspan_impl.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xdynamic_bitset.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xfunctional.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xhalf_float.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xhalf_float_impl.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xhash.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xhierarchy_generator.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xiterator_base.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xjson.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xmasked_value_meta.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xmasked_value.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xmeta_utils.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xmultimethods.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xoptional_meta.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xoptional.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xoptional_sequence.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xplatform.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xproxy_wrapper.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xsequence.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xsystem.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xtl_config.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xtype_traits.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xvariant.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xvariant_impl.hpp"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/3rdparty/xtl/include/xtl/xvisitor.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cmake/xtl" TYPE FILE FILES
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/xtl/xtlConfig.cmake"
    "/home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/xtl/xtlConfigVersion.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cmake/xtl/xtlTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cmake/xtl/xtlTargets.cmake"
         "/home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/xtl/CMakeFiles/Export/2fc63ec57839ed115fc15a5438bb5aec/xtlTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cmake/xtl/xtlTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cmake/xtl/xtlTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cmake/xtl" TYPE FILE FILES "/home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/xtl/CMakeFiles/Export/2fc63ec57839ed115fc15a5438bb5aec/xtlTargets.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pkgconfig" TYPE FILE FILES "/home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/xtl/xtl.pc")
endif()

