FIND_PACKAGE( ROOT REQUIRED )

set(name "Timepix3Producer.exe")
set(sourcefiles src/Timepix3Producer.cxx src/Timepix3Config.cxx)
set(ext_libraries "libSpidrTpx3Lib.so" "libKeithley.so" "libgpib.so" "libxerces-c.so")
set(ext_lib_paths "/home/vertextb/SPIDR/software/trunk/Release" "/usr/lib/x86_64-linux-gnu" "/home/vertextb/root/lib"  "/usr/local/lib" "/home/vertextb/gpib_reader/lib" )
set(ext_inc_paths "./include" "/home/vertextb/SPIDR/software/trunk/SpidrTpx3Lib" "/usr/include" ${ROOT_INCLUDE_DIR} "/home/vertextb/gpib_reader/include" "/usr/local/include" )

ADD_DEFINITIONS()
LINK_DIRECTORIES( ${ext_lib_paths} )
INCLUDE_DIRECTORIES( ${ext_inc_paths} )
ADD_EXECUTABLE( ${name} ${sourcefiles} )

ADD_EXECUTABLE( TestXMLReader src/testXMLread.cxx src/Timepix3Config.cxx )

TARGET_LINK_LIBRARIES( ${name} EUDAQ ${EUDAQ_THREADS_LIB} ${ext_libraries} ${my_qt_libraries} ${ROOT_LIBRARIES} )
TARGET_LINK_LIBRARIES( TestXMLReader ${ext_libraries} ${ROOT_LIBRARIES} )

INSTALL(TARGETS ${name} TestXMLReader
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
