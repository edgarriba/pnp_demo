cmake_minimum_required(VERSION 2.8)
project( PNP_DEMO )

find_package( OpenCV REQUIRED )

include_directories( 
	$(PNP_DEMO_SOURCE_DIR)/include 
	${OpenCV_INCLUDE_DIRS} 
)

add_executable( 
pnp_app 
	src/main.cpp
	src/CsvReader.cpp 
	src/CsvWriter.cpp
	src/ModelRegistration.cpp
	src/Mesh.cpp
	src/Model.cpp
	src/PnPProblem.cpp
	src/Utils.cpp
)

target_link_libraries( pnp_app ${OpenCV_LIBS} )
