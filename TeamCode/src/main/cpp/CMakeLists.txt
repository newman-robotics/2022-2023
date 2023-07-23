#This is no-no touchy
cmake_minimum_required(VERSION 3.18.1)

#Set the include directories
include_directories(src/main/cpp/include)

#The name of the project goes here
#(seems irrelevant; do whatever you want)
project("libcardinal")

#Libraries go here
add_library(libcardinal SHARED libcardinal.cpp)

#Link the libraries
target_link_libraries(libcardinal android log)