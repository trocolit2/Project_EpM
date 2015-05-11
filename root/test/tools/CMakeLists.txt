########################
## TEST TOOLS CMAKE ##
########################

add_library(TestTools_library TestTools.cpp)

set(SOURCE_LIBS_LOCAL TestTools_library)
set(SOURCE_LIBS ${SOURCE_LIBS_LOCAL} ${SOURCE_LIBS} PARENT_SCOPE)