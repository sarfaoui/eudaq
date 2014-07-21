set(name "TimepixProducer")
set(EUDAQ ${CMAKE_SOURCE_DIR})
set(tpprod ${EUDAQ}/producers/TimepixProducer)
set(LIBGPIB /afs/cern.ch/eng/clic/TBData/software/ROOT6_gcc48_python2.7/libgpib)
set(sourcefiles src/TimepixProducer.cxx src/TimepixDevice.cc src/MIMTLU.cc)
set(ext_libraries  "libstdc++.so" "libc.so" "librt.so" "libEUDAQ.so" "libdl.so" "libmpxmanager.so" "libKeithley.so" "libgpib.so" "libmpxctrl.so" "libusb.so" )
set(ext_lib_paths  ${EUDAQ}/lib ${LIBGPIB}/lib "${ROOTSYS}/lib" ${tpprod}/Pixelman_2013_09_25_x64 ${tpprod}/Pixelman_2013_09_25_x64/hwlibs ${EUDAQ}/extern/Keithley2XXXControl/lib   )
set(ext_inc_paths  ${LIBGPIB}/include "./include" ${EUDAQ}/producers/TimepixProducer/Pixelman_2013_09_25_x64/_other_/headers ${ROOT_INCLUDE_DIR} ${EUDAQ}/extern/Keithley2XXXControl/include  )

SET( CMAKE_CXX_FLAGS  "-fPIC -O2 -fpermissive -std=c++11" )

ADD_DEFINITIONS()
LINK_DIRECTORIES( ${ext_lib_paths} )
INCLUDE_DIRECTORIES( ${ext_inc_paths} )
ADD_EXECUTABLE( ${name} ${sourcefiles} )

ADD_EXECUTABLE( TimepixProducer.exe src/TimepixProducer.cxx src/TimepixDevice.cc src/MIMTLU.cc  )

TARGET_LINK_LIBRARIES( ${name} EUDAQ ${EUDAQ_THREADS_LIB} ${ext_libraries} ${ROOT_LIBRARIES} )
TARGET_LINK_LIBRARIES( TimepixProducer.exe ${ext_libraries} ${ROOT_LIBRARIES} )

INSTALL(TARGETS ${name} TimepixProducer.exe
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
