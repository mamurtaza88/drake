add_executable(example_topic_republisher example_topic_republisher.cpp)
target_link_libraries( example_topic_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(example_topic_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(other_topic_republisher other_topic_republisher.cpp)
target_link_libraries( other_topic_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(other_topic_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)
add_executable(lcmt_iiwa_ee_pose_republisher lcmt_iiwa_ee_pose_republisher.cpp)
target_link_libraries( lcmt_iiwa_ee_pose_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(lcmt_iiwa_ee_pose_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(lcmt_iiwa_status_republisher lcmt_iiwa_status_republisher.cpp)
target_link_libraries( lcmt_iiwa_status_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(lcmt_iiwa_status_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


