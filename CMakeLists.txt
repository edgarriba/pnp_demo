cmake_minimum_required(VERSION 2.8)
project( PNP_DEMO )

find_package( OpenCV REQUIRED )

include_directories( 
	$(PNP_DEMO_SOURCE_DIR)/include 
	${OpenCV_INCLUDE_DIRS} 
)

add_executable( 
pnp_demo 
	src/main.cpp
	src/CsvReader.cpp 
	src/CsvWriter.cpp
	src/ModelRegistration.cpp
	src/ObjectMesh.cpp
	src/ObjectModel.cpp
	src/PnPProblem.cpp
	src/Utils.cpp
)

target_link_libraries( pnp_demo ${OpenCV_LIBS} )
