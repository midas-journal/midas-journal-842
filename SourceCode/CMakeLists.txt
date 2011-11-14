cmake_minimum_required(VERSION 2.6) 
PROJECT(NormalizedMutual)


FIND_PACKAGE(ITK)
  
IF(ITK_FOUND)
    
   INCLUDE(${ITK_USE_FILE})
  
ELSE(ITK_FOUND)
    
  
ENDIF(ITK_FOUND)




ADD_EXECUTABLE(NormalizedMutual NormalizedMutual.cpp)

INSTALL_TARGETS(/bin NormalizedMutual)

TARGET_LINK_LIBRARIES( NormalizedMutual ITKNumerics ITKIO ITKstatistics)
