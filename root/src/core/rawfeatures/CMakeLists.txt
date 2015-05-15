########################
## RAW FEATURES CMAKE ##
########################

add_library(rawFeatures FeaturesDetect.cpp FeatureDescriptor.cpp FeaturesTools.cpp)

set(SOURCE_LIBS_LOCAL rawFeatures)
set(SOURCE_LIBS ${SOURCE_LIBS} ${SOURCE_LIBS_LOCAL})

set(SOURCE_LIBS ${SOURCE_LIBS} PARENT_SCOPE)