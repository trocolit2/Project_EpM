###################
## CORE CMAKE ##
###################

include_directories(rawfeatures)
add_subdirectory(rawfeatures)

set(SOURCE_LIBS ${SOURCE_LIBS} PARENT_SCOPE)