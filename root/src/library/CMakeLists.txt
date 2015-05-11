###################
## LIBRARY CMAKE ##
###################

add_library(FeaturesDetect FeaturesDetect.cpp)

set(SOURCE_LIBS_LOCAL FeaturesDetect)
set(SOURCE_LIBS ${SOURCE_LIBS} ${SOURCE_LIBS_LOCAL})

set(SOURCE_LIBS ${SOURCE_LIBS} PARENT_SCOPE)