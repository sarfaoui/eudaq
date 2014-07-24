#include "eudaq/DataConverterPlugin.hh"
#include "eudaq/StandardEvent.hh"
#include "eudaq/Utils.hh"

// All LCIO-specific parts are put in conditional compilation blocks
// so that the other parts may still be used if LCIO is not available.
#if USE_LCIO
#  include "IMPL/LCEventImpl.h"
#  include "IMPL/TrackerRawDataImpl.h"
#  include "IMPL/LCCollectionVec.h"
#  include "lcio.h"
#endif

namespace eudaq {

  // The event type for which this converter plugin will be registered
  // Modify this to match your actual event type (from the Producer)
  static const char* EVENT_TYPE = "Timepix3Raw";
  
  // Declare a new class that inherits from DataConverterPlugin
  class Timepix3ConverterPlugin : public DataConverterPlugin {
    
  public:
    
    // This is called once at the beginning of each run.
    // You may extract information from the BORE and/or configuration
    // and store it in member variables to use during the decoding later.
    virtual void Initialize(const Event & bore,
			    const Configuration & cnf) {
      m_XMLConfig = bore.GetTag("XMLConfig", "");
#ifndef WIN32  //some linux Stuff //$$change
      (void)cnf; // just to suppress a warning about unused parameter cnf
#endif
      
    }
    
    // This should return the trigger ID (as provided by the TLU)
    // if it was read out, otherwise it can either return (unsigned)-1,
    // or be left undefined as there is already a default version.
    virtual unsigned GetTriggerID(const Event & ev) const {
      static const unsigned TRIGGER_OFFSET = 8;
      // Make sure the event is of class RawDataEvent
      if (const RawDataEvent * rev = dynamic_cast<const RawDataEvent *> (&ev)) {
	// This is just an example, modified it to suit your raw data format
	// Make sure we have at least one block of data, and it is large enough
	if (rev->NumBlocks() > 0 && rev->GetBlock(0).size() >= (TRIGGER_OFFSET + sizeof(short))) {
	  // Read a little-endian unsigned short from offset TRIGGER_OFFSET
	  return getlittleendian<unsigned short> (&rev->GetBlock(0)[TRIGGER_OFFSET]);
	}
      }
      // If we are unable to extract the Trigger ID, signal with (unsigned)-1
      return (unsigned)-1;
    }
    
    // Here, the data from the RawDataEvent is extracted into a StandardEvent.
    // The return value indicates whether the conversion was successful.
    // Again, this is just an example, adapted it for the actual data layout.
    virtual bool GetStandardSubEvent(StandardEvent & sev, const Event & ev) const {
      // If the event type is used for different sensors
      // they can be differentiated here
      std::string sensortype = "timepix3";
      
      // Create a StandardPlane representing one sensor plane
      int id = 0;
      StandardPlane plane(id, EVENT_TYPE, sensortype);
      
      // Set the number of pixels
      int width = 256, height = 256;
      plane.SetSizeRaw(width, height);
      
      // Unpack data
      const RawDataEvent * rev = dynamic_cast<const RawDataEvent *> ( &ev );
      std::cout << "[Number of blocks] " << rev->NumBlocks() << std::endl;
      std::vector<unsigned char> data = rev->GetBlock( 0 ); // or 1?
      std::cout << "vector has size : " << data.size() << std::endl;
      
      std::vector<unsigned int> ZSDataX;
      std::vector<unsigned int> ZSDataY;
      std::vector<unsigned int> ZSDataFTOA;
      std::vector<unsigned int> ZSDataTOT;
      std::vector<unsigned int> ZSDataTOA;      
      size_t offset = 0;
      unsigned int aWord = 0;
      
      for( unsigned int i = 0; i < ( data.size() ) / 20; i++ ) { // 20 = 5*4 bytes for x,y,ftoa,tot,toa

	// // unpack(data,offset,aWord);
	// // offset+=sizeof(aWord);
	// // ZSDataX.push_back(aWord);
	
	// // unpack(data,offset,aWord);
	// // offset+=sizeof(aWord);
	// // ZSDataY.push_back(aWord);
	
	// aWord = 0;
	// for(unsigned int j=0;j<4;j++){
	//   aWord = aWord | ( data[offset+j] << j*8 );
	// }
	// offset+=sizeof(aWord);
	// ZSDataX.push_back(aWord);

	// aWord = 0;
	// for(unsigned int j=0;j<4;j++){
	//   aWord = aWord | ( data[offset+j] << j*8 );
	// }
	// offset+=sizeof(aWord);
	// ZSDataY.push_back(aWord);

	// aWord = 0;
	// for(unsigned int j=0;j<4;j++){
	//   aWord = aWord | ( data[offset+j] << j*8 );
	// }
	// offset+=sizeof(aWord);
	// ZSDataFTOA.push_back(aWord);

	// aWord = 0;
	// for(unsigned int j=0;j<4;j++){
	//   aWord = aWord | ( data[offset+j] << j*8 );
	// }
	// offset+=sizeof(aWord);
	// ZSDataTOT.push_back(aWord);

	// aWord = 0;
	// for(unsigned int j=0;j<4;j++){
	//   aWord = aWord | ( data[offset+j] << j*8 );
	// }
	// offset+=sizeof(aWord);
	// ZSDataTOA.push_back(aWord);

	// std::cout << offset << std::endl;

	ZSDataX.push_back(    unpackPixelData( data, offset + sizeof( aWord ) * 0 ) ); // first 4 bytes
	ZSDataY.push_back(    unpackPixelData( data, offset + sizeof( aWord ) * 1 ) ); // next 4 bytes
	ZSDataFTOA.push_back( unpackPixelData( data, offset + sizeof( aWord ) * 2 ) ); // and
	ZSDataTOT.push_back(  unpackPixelData( data, offset + sizeof( aWord ) * 3 ) ); // so
	ZSDataTOA.push_back(  unpackPixelData( data, offset + sizeof( aWord ) * 4 ) ); // on

	std::cout << "[DATA] "  << " " << ZSDataX[i] << " " << ZSDataY[i] << " " << ZSDataFTOA[i] << " " << ZSDataTOT[i] << " " << ZSDataTOA[i] << std::endl;

	offset += sizeof( aWord ) * 5; // shift by 20 bytes to reach next pixel info 
      }

      // Set the trigger ID
      // plane.SetTLUEvent(GetTriggerID(ev));
      
      // Add the plane to the StandardEvent
      sev.AddPlane(plane);
      
      // Indicate that data was successfully converted
      return true;
    }

    unsigned int unpackPixelData( std::vector<unsigned char> data, size_t offset ) const {

      unsigned int aWord = 0;
      for( unsigned int j=0; j<4; j++ ) {
	aWord = aWord | ( data[offset+j] << j*8 );
      }

      return aWord;
    }
    
#if USE_LCIO
    // This is where the conversion to LCIO is done
    virtual lcio::LCEvent * GetLCIOEvent(const Event * /*ev*/) const {
      return 0;
    }
#endif
    
  private:
    
    // The constructor can be private, only one static instance is created
    // The DataConverterPlugin constructor must be passed the event type
    // in order to register this converter for the corresponding conversions
    // Member variables should also be initialized to default values here.
    Timepix3ConverterPlugin()
      : DataConverterPlugin(EVENT_TYPE), m_XMLConfig("")
    {}
    
    // Information extracted in Initialize() can be stored here:
    std::string m_XMLConfig;
    
    // The single instance of this converter plugin
    static Timepix3ConverterPlugin m_instance;
  }; // class Timepix3ConverterPlugin
  
  // Instantiate the converter plugin instance
  Timepix3ConverterPlugin Timepix3ConverterPlugin::m_instance;
  
} // namespace eudaq


/*
Start Run: 42
Sample 1 size=104
128,254: 13,2,5376,52832
130,252: 11,3,5376,52832
132,254: 13,2,5376,52832
134,253: 13,2,5376,52832
136,254: 13,3,5376,52832
139,254: 13,2,5376,52832
140,254: 13,2,5376,52832
142,252: 13,2,5376,52832
144,252: 12,2,5376,52832
146,252: 12,2,5376,52832
149,254: 13,2,5376,52832
150,252: 11,3,5376,52832
152,254: 13,2,5376,52832
Sample 1 size=104
142,214: 13,3,5376,52846
145,210: 13,3,5376,52846
147,215: 13,2,5376,52846
149,212: 13,2,5376,52846
151,214: 13,3,5376,52846
152,211: 12,2,5376,52846
155,202: 13,3,5376,52846
157,201: 13,3,5376,52846
159,146: 15,2,5376,52846
161,162: 14,3,5376,52846
163,30: 15,1,5376,52846
164,73: 15,2,5376,52846
167,23: 15,1,5376,52846
Sample 1 size=8
80,166: 14,1022,5376,27944
Sample 1 size=8
80,166: 12,1022,504,28261
Sample 1 size=8
80,166: 1,1022,4492,38561
Sample 1 size=8
*/
