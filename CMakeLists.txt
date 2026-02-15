cmake_minimum_required(VERSION 3.13)
project(SoapyVahyaSDR VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SOAPY REQUIRED SoapySDR)

# libvahya from radio_gateware
set(VAHYA_LIB_DIR "${CMAKE_SOURCE_DIR}/../radio_gateware/host/lib")

add_library(SoapyVahyaSDR MODULE
    SoapyVahyaSDR.cpp
    Registration.cpp
    Streaming.cpp
    Settings.cpp
)

target_include_directories(SoapyVahyaSDR PRIVATE
    ${SOAPY_INCLUDE_DIRS}
    ${VAHYA_LIB_DIR}
)

target_link_directories(SoapyVahyaSDR PRIVATE
    ${SOAPY_LIBRARY_DIRS}
)

# Build libvahya as a static library for the module
add_library(vahya_static STATIC
    ${VAHYA_LIB_DIR}/vahya.c
)
set_property(TARGET vahya_static PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(vahya_static PUBLIC ${VAHYA_LIB_DIR})
target_link_libraries(vahya_static PUBLIC usb-1.0 pthread)

target_link_libraries(SoapyVahyaSDR PRIVATE
    ${SOAPY_LIBRARIES}
    vahya_static
)

# Hide internal symbols
set_property(TARGET SoapyVahyaSDR PROPERTY CXX_VISIBILITY_PRESET hidden)

# Install to SoapySDR modules directory
set(SOAPY_MODULE_DIR "${SOAPY_LIBRARY_DIRS}/SoapySDR/modules0.8"
    CACHE PATH "SoapySDR module install directory")
install(TARGETS SoapyVahyaSDR DESTINATION ${SOAPY_MODULE_DIR})
