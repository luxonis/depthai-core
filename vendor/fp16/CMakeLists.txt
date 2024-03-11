


ADD_LIBRARY(fp16 INTERFACE)
TARGET_INCLUDE_DIRECTORIES(fp16 INTERFACE include)

#INSTALL(FILES include/fp16.h
#  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#
#INSTALL(FILES
#    include/fp16/bitcasts.h
#    include/fp16/fp16.h
#    include/fp16/psimd.h
#    include/fp16/__init__.py
#    include/fp16/avx.py
#    include/fp16/avx2.py
#  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/fp16)
#
