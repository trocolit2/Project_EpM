###################
## SOURCE FOLDER ##
###################

################################
## LIBRARYS AND CLASS SESSION ##
################################

find_package(OpenCV REQUIRED)
set(SOURCE_LIBS_LOCAL ${OpenCV_LIBS})

## ADD SUBDIRECTORIES FROM LIBRARY
include_directories(library)
add_subdirectory(library)

set(SOURCE_LIBS ${SOURCE_LIBS} ${SOURCE_LIBS_LOCAL} PARENT_SCOPE)

#########################
## EXECUTABLES SESSION ##
#########################

add_executable(AS2C main.cpp)
target_link_libraries(AS2C ${SOURCE_LIBS})
