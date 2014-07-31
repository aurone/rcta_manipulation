set(IKFAST_COMPILE_FLAGS "-DIK_FAST_HAS_LIBRARY -DIKFAST_NO_MAIN -Wno-unused-variable -Wno-unused-but-set-variable")
add_library(hdt_kinematics hdt_arm_transform6d.cpp kinematics.cpp)
set_target_properties(hdt_kinematics PROPERTIES COMPILE_FLAGS "${IKFAST_COMPILE_FLAGS}")
target_link_libraries(hdt_kinematics lapack blas ${catkin_LIBRARIES})

install(TARGETS hdt_kinematics
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

