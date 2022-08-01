# Install script for directory: /mnt/sdb2/Progetti/Extruder/proj-8.1.1/data

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/proj" TYPE FILE FILES
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/data/proj.ini"
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/data/world"
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/data/other.extra"
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/data/nad27"
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/data/GL27"
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/data/nad83"
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/data/nad.lst"
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/data/CH"
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/data/ITRF2000"
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/data/ITRF2008"
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/data/ITRF2014"
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/data/proj.db"
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/data/deformation_model.schema.json"
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/data/projjson.schema.json"
    "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/data/triangulation.schema.json"
    )
endif()

