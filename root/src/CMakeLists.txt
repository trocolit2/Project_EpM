###################
## SOURCE FOLDER ##
###################

################################
## LIBRARYS AND CLASS SESSION ##
################################

find_package(OpenCV REQUIRED)
set(SOURCE_LIBS_LOCAL ${OpenCV_LIBS})

## ADD SUBDIRECTORIES FROM LIBRARY
include_directories(core)
add_subdirectory(core)

set(SOURCE_LIBS ${SOURCE_LIBS} ${SOURCE_LIBS_LOCAL} PARENT_SCOPE)

#########################
## EXECUTABLES SESSION ##
#########################

add_executable(SfM main.cpp)
target_link_libraries(SfM ${SOURCE_LIBS})
