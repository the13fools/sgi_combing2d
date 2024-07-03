include(FetchContent)
# FetchContent_Declare(
#   tinyad
#   GIT_REPOSITORY https://github.com/the13fools/TinyAD.git
#   # GIT_TAG 770dc5011371787ef784019f62794ec560623f79
#   GIT_TAG v1.0
#   # GIT_TAG 770dc5011371787ef784019f62794ec560623f79
# )

FetchContent_Declare(
  tinyad
  GIT_REPOSITORY https://github.com/patr-schm/tinyad.git
  GIT_TAG 29417031c185b6dc27b6d4b684550d844459b735
)
FetchContent_GetProperties(tinyad)
if(NOT tinyad_POPULATED)
  # Fetch the content using previously declared details
  FetchContent_Populate(tinyad)
  message(STATUS "tinyad_SOURCE_DIR: ${tinyad_SOURCE_DIR}")
  message(STATUS "tinyad_BINARY_DIR: ${tinyad_BINARY_DIR}")
  add_subdirectory(${tinyad_SOURCE_DIR} ${tinyad_BINARY_DIR})
endif()
FetchContent_MakeAvailable(tinyad)