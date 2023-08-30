find_package(serial REQUIRED)
add_library(rmd rmd.cpp rmd_impl.cpp)
target_include_directories(rmd 
  PRIVATE ${PRIVATE_INCLUDE_DIR}
)
target_link_libraries(rmd PRIVATE serial::serial)

install(
  TARGETS rmd
  EXPORT ${PROJECT_NAME}-config
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)