add_library( Yslam SHARED
	camera.cpp
	config.cpp
	frame.cpp
	vodometry_main.cpp
	vodometry_point.cpp
	tool.cpp
	g2opose.cpp
)

target_link_libraries( Yslam
    ${THIRD_PARTY_LIBS}
)
