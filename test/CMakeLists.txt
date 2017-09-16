add_executable( run_vo run_vo.cpp )
add_executable( test test.cpp )
target_link_libraries( run_vo vie_slam )
target_link_libraries( test vie_slam )
