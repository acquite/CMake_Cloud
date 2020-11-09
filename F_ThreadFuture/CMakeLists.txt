cmake_minimum_required(VERSION 3.0.0)
aux_source_directory(. DIR_FThread_SRCS)


add_library(libF_Thread ${DIR_FThread_SRCS})