cmake_minimum_required(VERSION 3.12)

project(autd3)

if(WIN32)
	set(OS win32)
  include(cmake/win_vs.cmake)
elseif(APPLE)
	set(OS macosx)
  include(cmake/mac.cmake)
else()
	set(OS linux)
  include(cmake/linux.cmake)
endif()

ADD_CUSTOM_TARGET(cpplint
  python
  ${PROJECT_SOURCE_DIR}/deps/cpplint.py
  --filter=-runtime/rtti,-build/c++11,-whitespace/comments
  --linelength=150
  ${PROJECT_SOURCE_DIR}/lib/*
  ${PROJECT_SOURCE_DIR}/include/*
  ${PROJECT_SOURCE_DIR}/autdsoem/*
  )

#SOEM
add_subdirectory(autdsoem)

add_subdirectory(lib)
add_subdirectory(example)

if(WIN32)
  set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT simple)
endif()
