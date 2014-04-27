cmake_minimum_required(VERSION 2.8)
project( EPnP_Demo )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( 
epnp_demo 
	src/main.cpp
	src/CsvReader.cpp 
	src/CsvWriter.cpp
	src/ModelRegistration.cpp
	src/ObjectMesh.cpp
	src/ObjectModel.cpp
	src/PnPProblem.cpp
	src/Utils.cpp
)

target_link_libraries( epnp_demo ${OpenCV_LIBS} )
