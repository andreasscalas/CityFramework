# Source https://github.com/google/googletest/blob/master/googletest/README.md
cmake_minimum_required(VERSION 2.8.2) # minimum version for ExternalProject_Add

project(googletest-download NONE)

include(ExternalProject)
ExternalProject_Add(googletest
  URL https://github.com/google/googletest/archive/release-1.8.1.zip
  URL_HASH SHA1=7b41ea3682937069e3ce32cb06619fead505795e
  DOWNLOAD_NO_PROGRESS ON
  SOURCE_DIR        "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/googletest-src"
  BINARY_DIR        "/mnt/sdb2/Progetti/Extruder/proj-8.1.1/build/googletest-build"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  TEST_COMMAND      ""
  # Disable install step
  INSTALL_COMMAND   ""
)
