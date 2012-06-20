PROJECT(move4films)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

#FIND_PACKAGE( OpenCV REQUIRED )

# Declare the target (an executable)
INCLUDE_DIRECTORIES(src/)
ADD_EXECUTABLE(move  move4films.cpp ../../Roboard/ad79x8.cpp ../../Roboard/pwm.cpp ../../Roboard/com.cpp ../../Roboard/common.cpp ../../Roboard/i2c.cpp ../../Roboard/i2cdx.cpp ../../Roboard/io.cpp ../../Roboard/pwmdx.cpp ../../Roboard/rcservo.cpp ../../Roboard/spi.cpp ../../Roboard/spidx.cpp)

#TARGET_LINK_LIBRARIES(move4filming .)

#MESSAGE(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
