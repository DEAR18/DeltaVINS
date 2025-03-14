# add test_triangulation
add_executable(test_triangulation test_triangulation.cpp)
target_link_libraries(test_triangulation
    ${LINK_LIBS}
    gtest
)
install(TARGETS test_triangulation
    DESTINATION lib/${PROJECT_NAME})


add_executable(test_equidistant_camera_model test_camModel_equidistant.cpp)
target_link_libraries(test_equidistant_camera_model
    ${LINK_LIBS}
    gtest
)
install(TARGETS test_equidistant_camera_model
    DESTINATION lib/${PROJECT_NAME})
