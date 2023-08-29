add_library(rmd rmd.cpp rmd_impl.cpp)
target_link_libraries(rmd PRIVATE serial)
target_include_directories(rmd PRIVATE ${serial_INCLUDE_PATH})

install(
  TARGETS rmd
  EXPORT ${PROJECT_NAME}-config
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)