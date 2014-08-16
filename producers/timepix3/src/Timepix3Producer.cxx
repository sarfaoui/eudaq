#include "eudaq/Producer.hh"
#include "eudaq/Logger.hh"
#include "eudaq/RawDataEvent.hh"
#include "eudaq/Timer.hh"
#include "eudaq/Utils.hh"
#include "eudaq/OptionParser.hh"

#include <iostream>
#include <ostream>
#include <vector>
#include <unistd.h>
#include <iomanip>
#include <signal.h>

#include "Timepix3Config.h"

#include "gpib/ib.h"
#include "Keithley2450.h"

#define error_out(str) cout<<str<<": "<<spidrctrl->errorString()<<endl

//#define TPX3_VERBOSE
//#define TPX3_STORE_TXT

using namespace std;
 
// Structure to store pixel info
struct PIXEL
{
  unsigned char x, y;
  unsigned short tot;
  uint64_t ts;
};

// Structure to store trigger info
struct TRIGGER
{
  unsigned short int_nr, tlu_nr;
  uint64_t ts;
};

// A name to identify the raw data format of the events generated
// Modify this to something appropriate for your producer.
static const std::string EVENT_TYPE = "Timepix3Raw";

// Declare a new class that inherits from eudaq::Producer
class Timepix3Producer : public eudaq::Producer {
  public:
  
  //////////////////////////////////////////////////////////////////////////////////
  // Timepix3Producer
  //////////////////////////////////////////////////////////////////////////////////

  // The constructor must call the eudaq::Producer constructor with the name
  // and the runcontrol connection string, and initialize any member variables.
  Timepix3Producer(const std::string & name, const std::string & runcontrol)
    : eudaq::Producer(name, runcontrol),
      m_run(0), m_ev(0), stopping(false), done(false), started(0) {

    myTimepix3Config = new Timepix3Config();

  }
    
  //////////////////////////////////////////////////////////////////////////////////
  // OnConfigure
  //////////////////////////////////////////////////////////////////////////////////

  // This gets called whenever the DAQ is configured
  virtual void OnConfigure(const eudaq::Configuration & config) {
    std::cout << "Configuring: " << config.Name() << std::endl;
    
    // Do any configuration of the hardware here
    // Configuration file values are accessible as config.Get(name, default)

    // Timepix3 XML config
    m_xmlfileName = config.Get( "XMLConfig", "" );
    myTimepix3Config->ReadXMLConfig( m_xmlfileName );
    cout << "Configuration file created on: " << myTimepix3Config->getTime() << endl;

    // SPIDR-TPX3 IP & PORT
    m_spidrIP  = config.Get( "SPIDR_IP", "192.168.100.10" );
    int ip[4];
    vector<TString> ipstr = tokenise( m_spidrIP, ".");
    for (int i = 0; i < ipstr.size(); i++ ) ip[i] = ipstr[i].Atoi();
    m_spidrPort = config.Get( "SPIDR_Port", 50000 );

    cout << "Connecting to SPIDR at " << ip[0] << "." << ip[1] << "." << ip[2] << "." << ip[3] << ":" << m_spidrPort << endl;

    // Open a control connection to SPIDR-TPX3 module
    // with address 192.168.100.10, default port number 50000
    spidrctrl = new SpidrController( ip[0], ip[1], ip[2], ip[3], m_spidrPort );

    // Reset Device
    if( !spidrctrl->reinitDevice( device_nr ) ) error_out( "###reinitDevice" );

    // Are we connected to the SPIDR-TPX3 module?
    if( !spidrctrl->isConnected() ) {
      std::cout << spidrctrl->ipAddressString() << ": " << spidrctrl->connectionStateString() << ", " << spidrctrl->connectionErrString() << std::endl;
    } else {
      std::cout << "\n------------------------------" << std::endl;
      std::cout << "SpidrController is connected!" << std::endl;
      std::cout << "Class version: " << spidrctrl->versionToString( spidrctrl->classVersion() ) << std::endl;
      int firmwVersion, softwVersion = 0;
      if( spidrctrl->getFirmwVersion( &firmwVersion ) ) std::cout << "Firmware version: " << spidrctrl->versionToString( firmwVersion ) << std::endl;
      if( spidrctrl->getSoftwVersion( &softwVersion ) ) std::cout << "Software version: " << spidrctrl->versionToString( softwVersion ) << std::endl;
      std::cout << "------------------------------\n" << std::endl;
    }

    // DACs configuration
    if( !spidrctrl->setDacsDflt( device_nr ) ) error_out( "###setDacsDflt" );
  
    // Enable decoder
    if( !spidrctrl->setDecodersEna( 1 ) )      error_out( "###setDecodersEna" );
    
    // Pixel configuration
    if( !spidrctrl->resetPixels( device_nr ) ) error_out( "###resetPixels" );
    
    // Device ID
    int device_id = -1;
    if( !spidrctrl->getDeviceId( device_nr, &device_id ) ) error_out( "###getDeviceId" );
    //cout << "Device ID: " << device_id << endl;
    m_chipID = myTimepix3Config->getChipID( device_id );
    cout << "[Timepix3] Chip ID: " << m_chipID << endl;

    // Get DACs from XML config
    map< string, int > xml_dacs = myTimepix3Config->getDeviceDACs();
    map< string, int >::iterator xml_dacs_it;
    for( xml_dacs_it = xml_dacs.begin(); xml_dacs_it != xml_dacs.end(); ++xml_dacs_it ) {
    
      string name = xml_dacs_it->first;
      int XMLval = xml_dacs_it->second;
      int val = config.Get( name, XMLval ); // overwrite value from XML if it's uncommented in the conf

      if( TPX3_DAC_CODES.find( name ) != TPX3_DAC_CODES.end() ) {
	int daccode = TPX3_DAC_CODES.at( name );
	if( !spidrctrl->setDac( device_nr, daccode, val ) ) {
	  cout << "Error, could not set DAC: " << name << " " << daccode << " " << val << endl;
	} else {
	  int tmpval = -1;
	  spidrctrl->getDac( device_nr, daccode, &tmpval );
	  cout << "Successfully set DAC: " << name << " to " << tmpval << endl;
	}
      
      } else if( name == "VTHRESH" ) {
	int coarse = val / 160;
	int fine = val - coarse*160 + 352;
	m_xml_VTHRESH = val;
	if( !spidrctrl->setDac( device_nr, TPX3_VTHRESH_COARSE, coarse ) ) {
	  cout << "Error, could not set VTHRESH_COARSE." << endl;
	} else {
	  int tmpval = -1;
	  spidrctrl->getDac( device_nr, TPX3_VTHRESH_COARSE, &tmpval );
	  cout << "Successfully set DAC: VTHRESH_COARSE to " << tmpval << endl;
	}
	if( !spidrctrl->setDac( device_nr, TPX3_VTHRESH_FINE, fine ) ) {
	  cout << "Error, could not set VTHRESH_FINE." << endl;
	} else {
	  int tmpval = -1;
	  spidrctrl->getDac( device_nr, TPX3_VTHRESH_FINE, &tmpval );
	  cout << "Successfully set DAC: VTHRESH_FINE to " << tmpval << endl;
	}	
      
      } else if( name == "GeneralConfig" ) {
	if ( !spidrctrl->setGenConfig( device_nr, val ) ) {
	  error_out( "###setGenConfig" );
	} else {
	  int config = -1;
     
	  spidrctrl->getGenConfig( device_nr, &config );
	  cout << "Successfully set General Config to " << config << endl;
	  // Unpack general config for human readable output
	  myTimepix3Config->unpackGeneralConfig( config );
	}

      } else if( name == "PllConfig" ) {
	if ( !spidrctrl->setPllConfig( device_nr, val ) ) {
	  error_out( "###setPllConfig" );
	} else {
	  int config = -1;
	  spidrctrl->getPllConfig( device_nr, &config );
	  cout << "Successfully set PLL Config to " << config << endl;
	}

      } else if( name == "OutputBlockConfig" ) {
	if ( !spidrctrl->setOutBlockConfig( device_nr, val ) ) {
	  error_out( "###setOutBlockConfig" );
	} else {
	  int config = -1;
	  spidrctrl->getOutBlockConfig( device_nr, &config );
	  cout << "Successfully set Output Block Config to " << config << endl;
	}
      }

    }
    
    // Reset entire matrix config to zeroes
    spidrctrl->resetPixelConfig();

    // Get per-pixel thresholds from XML
    bool pixfail = false;
    vector< vector< int > > matrix_thresholds = myTimepix3Config->getMatrixDACs();
    for( int x = 0; x < NPIXX; x++ ) {
      for ( int y = 0; y < NPIXY; y++ ) {
	int threshold = matrix_thresholds[y][x]; // x & y are inverted when parsed from XML
	// cout << x << " "<< y << " " << threshold << endl;
	if( !spidrctrl->setPixelThreshold( x, y, threshold ) ) pixfail = true;
      }
    }
    if( !pixfail ) {
      cout << "Successfully built pixel thresholds." << endl;
    } else {
      cout << "Something went wrong building pixel thresholds." << endl;
    }

    // Get pixel mask from XML
    bool maskfail = false;
    vector< vector< bool > > matrix_mask = myTimepix3Config->getMatrixMask();
    for( int x = 0; x < NPIXX; x++ ) {
      for ( int y = 0; y < NPIXY; y++ ) {
	bool mask = matrix_mask[y][x]; // x & y are inverted when parsed from XML
	if( !spidrctrl->setPixelMask( x, y, mask ) ) maskfail = true;
      }
    }
    // Add pixels masked by the user in the conf
    string user_mask = config.Get( "User_Mask", "" );
    vector<TString> pairs = tokenise( user_mask, ":");
    for( int k = 0; k < pairs.size(); ++k ) {
      vector<TString> pair = tokenise( pairs[k], "," );
      int x = pair[0].Atoi();
      int y = pair[1].Atoi();
      cout << "Additinal user mask: " << x << "," << y << endl;
      if( !spidrctrl->setPixelMask( x, y, true ) ) maskfail = true;
    }
    if( !maskfail ) {
      cout << "Successfully built pixel mask." << endl;
    } else {
      cout << "Something went wrong building pixel mask." << endl;
    }

    // Enable test-pulses for some pixels
    // spidrctrl->setCtprBit( 16 );
    // spidrctrl->setCtprBit( 86 );
    // spidrctrl->setCtprBit( 26 );
    // spidrctrl->setCtprBit( 96 );
    // if( !spidrctrl->setCtpr( device_nr ) ) error_out( "###setCtpr" );
    // spidrctrl->setPixelTestEna( 16, 16, true );
    // spidrctrl->setPixelTestEna( 16, 216, true );
    // spidrctrl->setPixelTestEna( 16, 26, true );
    // spidrctrl->setPixelTestEna( 16, 146, true );
    // spidrctrl->setPixelTestEna( 26, 36, true );
    // spidrctrl->setPixelTestEna( 26, 136, true );
    // spidrctrl->setPixelTestEna( 26, 46, true );
    // spidrctrl->setPixelTestEna( 26, 126, true );
    // spidrctrl->setPixelTestEna( 86, 16, true );
    // spidrctrl->setPixelTestEna( 86, 216, true );
    // spidrctrl->setPixelTestEna( 86, 26, true );
    // spidrctrl->setPixelTestEna( 86, 146, true );
    // spidrctrl->setPixelTestEna( 96, 36, true );
    // spidrctrl->setPixelTestEna( 96, 136, true );
    // spidrctrl->setPixelTestEna( 96, 46, true );
    // spidrctrl->setPixelTestEna( 96, 126, true );

    // Actually set the pixel thresholds and mask
    if( !spidrctrl->setPixelConfig( device_nr ) ) {
      error_out( "###setPixelConfig" );
    } else {
      cout << "Successfully set pixel configuration." << endl;
    }
    unsigned char *pixconf = spidrctrl->pixelConfig();
    int x, y, cnt = 0;
    if( spidrctrl->getPixelConfig( device_nr ) ) {
      for( y = 0; y < NPIXY; ++y ) {
	for( x = 0; x < NPIXX; ++x ) {
	  std::bitset<6> bitconf( (unsigned int) pixconf[y*256+x] );
	  //if( pixconf[y*256+x] != 0 ) {
	  if ( bitconf.test(0) ) { /* masked? */
	    //cout << x << ',' << y << ": " << bitconf << endl; // hex << setw(2) << setfill('0') << (unsigned int) pixconf[y*256+x] << dec << endl;
	    ++cnt;
	  }
	}
      }
      cout << "Pixels masked = " << cnt << endl;
    } else {
      cout << "###getPixelConfig: " << spidrctrl->errorString() << endl;
    }
    
    // Keithley stuff
    m_use_k2450   = config.Get( "USE_Keithley", 0 );
    m_gpib_num    = config.Get( "Keithley_GPIB", 18 );
    m_Vbias       = config.Get( "V_Bias", 0 );
    m_Ilim        = config.Get( "I_Limit", 1e-6 );
    m_doBiasScan  = config.Get( "Do_Bias_Scan", 0 );
    m_Vstart      = config.Get( "V_start", 0 );
    m_VbiasStep   = config.Get( "V_step", 0 );
    m_VbiasMax    = config.Get( "V_max", 0 );
    m_Vreturn     = config.Get( "V_return", 0 );
    if( m_use_k2450 == 1 ) {
      cout << endl;
      k2450 = new Keithley2450( m_gpib_num );
      k2450->OutputOff();
      sleep(1);
      //k2450->SetMeasureCurrent();
      k2450->SetMeasureVoltage();
      sleep(1);
      k2450->SetSourceVoltage4W();
      sleep(1);
      k2450->SetOutputVoltageCurrentLimit( m_Ilim );
      sleep(1);
      k2450->OutputOn();
    }
    m_VstepCount = 0;

	// Threshold scan conf params
	m_do_threshold_scan = config.Get( "do_threshold_scan", 0 );
	m_threshold_start   = config.Get( "threshold_start", m_xml_VTHRESH );
	m_threshold_step    = config.Get( "threshold_step", 0 );	
	m_threshold_max     = config.Get( "threshold_max", m_xml_VTHRESH );
	m_threshold_return  = config.Get( "threshold_return", m_xml_VTHRESH );
	m_threshold_count   = 0;

    // At the end, set the status that will be displayed in the Run Control.
    SetStatus(eudaq::Status::LVL_OK, "Configured (" + config.Name() + ")");

    // Also display something for us
    cout << endl;
    cout << "Timepix3 Producer configured. Ready to start run. " << endl;
    cout << endl;
  }

  //////////////////////////////////////////////////////////////////////////////////
  // OnStartRun
  //////////////////////////////////////////////////////////////////////////////////
  
  // This gets called whenever a new run is started
  // It receives the new run number as a parameter
  virtual void OnStartRun(unsigned param) {
    m_run = param;
    m_ev = 0;
    
    std::cout << "Start Run: " << m_run << std::endl;

	// It must send a BORE to the Data Collector
    eudaq::RawDataEvent bore(eudaq::RawDataEvent::BORE(EVENT_TYPE, m_run));
    // You can set tags on the BORE that will be saved in the data file
    // and can be used later to help decoding
    
    // Bias scan
    double newBiasVoltage = m_Vstart + m_VstepCount*m_VbiasStep;
    if( m_use_k2450 == 1 ) {
      if( m_doBiasScan == 1 ) {
	if ( newBiasVoltage <= m_VbiasMax ) {
	  k2450->SetOutputVoltage( newBiasVoltage ); 
	  m_VstepCount++;
	} else {
	  k2450->SetOutputVoltage( m_Vreturn ); 
	  newBiasVoltage = m_Vreturn;
	}
      } else {
	k2450->SetOutputVoltage( m_Vbias ); 
	newBiasVoltage = m_Vbias;
      }
    }

    // Threshold scan
    int newThreshold = m_threshold_start + m_threshold_count*m_threshold_step;
    if( m_do_threshold_scan == 1 ) {
      int coarse = newThreshold / 160;
      int fine = newThreshold - coarse*160 + 352;	
      if( newThreshold <= m_threshold_max ) {
	if( !spidrctrl->setDac( device_nr, TPX3_VTHRESH_COARSE, coarse ) ) error_out( "###setDac" );
	if( !spidrctrl->setDac( device_nr, TPX3_VTHRESH_FINE, fine ) ) error_out( "###setDac" );
	m_threshold_count++;
      } else {
	int coarse = m_threshold_return / 160;
	int fine = m_threshold_return - coarse*160 + 352;
	if( !spidrctrl->setDac( device_nr, TPX3_VTHRESH_COARSE, coarse ) ) error_out( "###setDac" );
	if( !spidrctrl->setDac( device_nr, TPX3_VTHRESH_FINE, fine ) ) error_out( "###setDac" );
	newThreshold = 	m_threshold_return;
      }
    } else {
      newThreshold = m_xml_VTHRESH;
    }
    cout << "[Timepix3] Threshold = " << newThreshold << endl;
    bore.SetTag( "VTRESH", newThreshold );
    
    // TPX3 SpidrMan XML config
    bore.SetTag( "XMLConfig", eudaq::to_string( m_xmlfileName ) );

    // Put measured bias voltage in BORE
    if ( m_use_k2450 == 1 ) {
      sleep(2); // wait for ramp up
      double actualBiasVoltage = k2450->ReadVoltage();
      bore.SetTag( "VBiasActual", actualBiasVoltage );
      bore.SetTag( "VBiasSet", newBiasVoltage );
    }
    
    // Chip ID
    bore.SetTag( "ChipID", m_chipID );

    // Read band gap temperature, whatever that is
    int bg_temp_adc, bg_output_adc;
    if( !spidrctrl->setSenseDac( device_nr, TPX3_BANDGAP_TEMP ) ) error_out( "###setSenseDac" );
    if( !spidrctrl->getAdc( &bg_temp_adc, 64 ) ) error_out( "###getAdc" );   	
    float bg_temp_V = 1.5*( bg_temp_adc/64. )/4096;
    if( !spidrctrl->setSenseDac( device_nr, TPX3_BANDGAP_OUTPUT ) ) error_out( "###setSenseDac" );
    if( !spidrctrl->getAdc( &bg_output_adc, 64 ) ) error_out( "###getAdc" );   	
    float bg_output_V = 1.5*( bg_output_adc/64. )/4096;
    m_temp = 88.75 - 607.3 * ( bg_temp_V - bg_output_V);
    cout << "[Timepix3] Temperature is " << m_temp << " C" << endl;
    bore.SetTag( "Temperature", m_temp );
    
    // Send the event to the Data Collector
    SendEvent(bore);
    
    // At the end, set the status that will be displayed in the Run Control.
    SetStatus(eudaq::Status::LVL_OK, "Running");
    started=true;
  }

  //////////////////////////////////////////////////////////////////////////////////
  // OnStopRun
  //////////////////////////////////////////////////////////////////////////////////
 
  // This gets called whenever a run is stopped
  virtual void OnStopRun() {
    std::cout << "Stopping Run" << std::endl;
    started = false;
    // Set a flag to signal to the polling loop that the run is over
    stopping = true;
    
    // wait until all events have been read out from the hardware
    while( stopping ) {
      eudaq::mSleep( 20 );
    }
    
    // Send an EORE after all the real events have been sent
    // You can also set tags on it (as with the BORE) if necessary
    SendEvent(eudaq::RawDataEvent::EORE(EVENT_TYPE, m_run, ++m_ev));
  }

  //////////////////////////////////////////////////////////////////////////////////
  // OnTerminate
  //////////////////////////////////////////////////////////////////////////////////
  
  // This gets called when the Run Control is terminating,
  // we should also exit.
  virtual void OnTerminate() {
    std::cout << "Terminating..." << std::endl;
    done = true;
  }
  
  //////////////////////////////////////////////////////////////////////////////////
  // ReadoutLoop
  //////////////////////////////////////////////////////////////////////////////////

  // This is just an example, adapt it to your hardware
  void ReadoutLoop() {
    // Loop until Run Control tells us to terminate
    while( !done ) {
      if( stopping ) {
	cout << "Stopping..." << endl;
	// if so, signal that there are no events left
	stopping = false;
      }
      // Now sleep for a bit, to prevent chewing up all the CPU
      //eudaq::mSleep( 20 );
      // Then restart the loop
      //continue;
      if( !started ) {
	//cout << "Not started." << endl;
	// Now sleep for a bit, to prevent chewing up all the CPU
	eudaq::mSleep( 20 );
	// Then restart the loop
	continue;
      }
      
      // Log some info
      if( m_ev % 10000 == 0 ) {
	if( m_use_k2450 == 1 ) {
	  k2450->SetMeasureCurrent();
	  double I = k2450->ReadValue() * 1e9;
	  k2450->SetMeasureVoltage();
	  double V = k2450->ReadVoltage();
	  char kmsg[1024];
	  sprintf( kmsg, "Bias Voltage is %f V, Current is %f nA", V, I );
	  SetStatus( eudaq::Status::LVL_INFO, kmsg );  
	}  
      } 
      
      // Create SpidrDaq for later (best place to do it?)
      spidrdaq = new SpidrDaq( spidrctrl );

      // Restart timers to sync Timepix3 and TLU timestamps
      if( !spidrctrl->restartTimers() ) error_out( "###restartTimers" );
      
      // Set Timepix3 acquisition mode
      if( !spidrctrl->datadrivenReadout() ) error_out( "###datadrivenReadout" );
        
      // Sample pixel data
      spidrdaq->setSampling( true );
      spidrdaq->setSampleAll( true );

      // Open shutter
      if( !spidrctrl->openShutter() ) error_out( "###openShutter" );

      // Enable TLU
      if( !spidrctrl->tlu_enable( device_nr, 1 ) ) error_out( "###tlu_enable" );
      
#ifdef TPX3_STORE_TXT
      // Some output files for debugging
      FILE *ft, *fp, *fa, *fd;
      ft = fopen("trg.txt","w");
      fp = fopen("pix.txt","w");
      fa = fopen("all.txt","w");
      fd = fopen("diff.txt","w");
#endif
      
      // Vectors to contain pixel and trigger structures
      vector< PIXEL > pixel_vec;
      pixel_vec.reserve( 10000000 );
      vector< TRIGGER > trigger_vec;
      trigger_vec.reserve( 10000000 );
      int last_trg_timestamp=0;
      uint64_t unfolded_timestamp=0;
      uint64_t last_fpga_ts=0;
      const uint64_t TIMER_EPOCH = 0x40000000;
      int cnt = 0;      
      while( !stopping ) {

      	int size;
      	bool next_sample = true;

      	// Get a sample of pixel data packets, with timeout in ms
      	const unsigned int BUF_SIZE = 8*1024*1024;
      	next_sample = spidrdaq->getSample( BUF_SIZE, 1 );

      	if( next_sample ) {

      	  ++cnt;
      	  size = spidrdaq->sampleSize();
	  
#ifdef TPX3_STORE_TXT
	  fprintf(fa,"#\n");
#endif
	  // look inside sample buffer...
	  while( 1 ) {
	    
	    uint64_t data = spidrdaq->nextPacket();

	    // ...until the sample buffer is empty
	    if( !data ) break;

	    uint64_t header = data & 0xF000000000000000;
	    	    
	    // Data-driven or sequential readout pixel data header?
	    if( header == 0xB000000000000000 || header == 0xA000000000000000 ) {
	      struct PIXEL pixel;

	      unsigned char x, y, pixdata, ftoa, tot, toa;
	      uint64_t fpga_ts;
	      uint64_t pix_ts;
	      uint64_t dcol, spix, pix;
	      // doublecolumn * 2
	      dcol  = (( data & 0x0FE0000000000000 ) >> 52 ); //(16+28+9-1)
	      // superpixel * 4
	      spix  = (( data & 0x001F800000000000 ) >> 45 ); //(16+28+3-2)
	      // pixel
	      pix   = (( data & 0x0000700000000000) >> 44 ); //(16+28)
	      pixel.x    = (unsigned char) ( dcol + pix/4 );
	      pixel.y    = (unsigned char) ( spix + ( pix & 0x3 ) );
	      // pixel data
	      pixdata = (int) (( data & 0x00000FFFFFFF0000 ) >> 16 );
	      pixel.tot  =  ( pixdata >> 4 ) & 0x3FF;

	      // timestamp calculation
	      ftoa = pixdata & 0xF;
	      toa  = ( pixdata >> 14 ) & 0x3FFF;
	      fpga_ts = (int) (data & 0x000000000000FFFF);
	      pix_ts = ( fpga_ts << 14 ) | toa; 
	      if( fpga_ts < last_fpga_ts - 1 ) 
		{
		  unfolded_timestamp+=TIMER_EPOCH;
#ifdef TPX3_VERBOSE
		  cout << "-- unfolding (pix) --\n";
#endif
		} else if( fpga_ts == last_fpga_ts - 1 ) 
		{ 
		  pix_ts-=TIMER_EPOCH;
		}
	      pix_ts += unfolded_timestamp;
	      pix_ts <<= 4;
	      pix_ts -= ftoa;
	      pixel.ts = pix_ts;
              last_fpga_ts = fpga_ts;

	      // store PIXEL struct in vector
	      pixel_vec.push_back( pixel );
	      
	      // print it
#ifdef TPX3_VERBOSE
	      printf("[PIXDATA] (%3d,%3d) TOT:%5d TOA:%5d FPGA_TS:%6d       TS:%15llu\n", x , y , tot, toa , fpga_ts, pix_ts);
#endif
#ifdef TPX3_STORE_TXT
	      fprintf(fp,"%d\n",pix_ts);
	      fprintf(fa,"p\t%d\n",pix_ts);
#endif
	    } else if( header == 0x5000000000000000 ) { // Or TLU packet header?
	      struct TRIGGER trigger;
	      uint64_t fpga_ts;
	      unsigned short int_trg_nr, tlu_trg_nr;
	      uint64_t trg_timestamp;
	      //internal trigger number
	      trigger.int_nr = (data >> 45) & 0x7FFF;
	      //TLU trigger number
	      trigger.tlu_nr = (data >> 30) & 0x7FFF;

	      //timestamp
	      trg_timestamp = data & 0x3FFFFFFF;
	      fpga_ts = (int) ((trg_timestamp>>14) & 0x000000000000FFFF);
	      if ( fpga_ts < last_fpga_ts-1 )
		{
		  unfolded_timestamp += TIMER_EPOCH;
#ifdef TPX3_VERBOSE
		  cout << "-- unfolding (trg) --\n";
#endif
		} else if ( fpga_ts == last_fpga_ts - 1 )
		{
		  trg_timestamp-=TIMER_EPOCH;
		}
	      trg_timestamp += unfolded_timestamp;
	      trg_timestamp <<= 4;
	      trigger.ts = trg_timestamp;
              last_fpga_ts = fpga_ts;
	      
#ifdef TPX3_VERBOSE
	      printf("[TRGDATA] tlu_id:%5d int_id:%5d                          TS:%15llu\n", tlu_trg_nr , int_trg_nr, trg_timestamp);
#endif
	      // store TRIGGER struct in vector
	      trigger_vec.push_back( trigger );

#ifdef TPX3_STORE_TXT
	      // write in files
              fprintf(ft,"%d\n",trg_timestamp);
              fprintf(fa,"t\t%d\n",trg_timestamp);
#endif
	    }
	  } // End loop over sample buffer
	  
	  // If sample buffer is empty, look at available  data and try to build events
	  unsigned int trg_built = 0;
	  
	  // Loop over pixel and trigger vectors
	  while( trigger_vec.size() > 1 ) {
	    // Current event
	    eudaq::RawDataEvent ev( EVENT_TYPE, m_run, m_ev );
	    std::vector<unsigned char> bufferTrg;
	    std::vector<unsigned char> bufferPix;
	    
	    // pack( buffer, trg_id ); !! add trigger number and timestamp here !!
	    
	    uint64_t curr_trg_ts = trigger_vec[0].ts;
	    uint64_t next_trg_ts = trigger_vec[1].ts;
	    uint64_t max_pixel_ts = ( next_trg_ts + curr_trg_ts ) / 2;

	    uint64_t curr_tlu_nr = trigger_vec[0].tlu_nr;
	    uint64_t curr_int_nr = trigger_vec[0].int_nr;
	    
	    // pack TLU info in its buffer
	    pack( bufferTrg, curr_trg_ts);
	    pack( bufferTrg, curr_tlu_nr);
	    pack( bufferTrg, curr_int_nr);

	    // and add it to the event
	    ev.AddBlock( 0, bufferTrg );

#ifdef TPX3_VERBOSE
	    uint64_t fpts=0;
	    if (pixel_vec.size()>0) fpts=pixel_vec[0].ts;
	    printf("\n=> processing tr_id %5d  ts: %15llu max_ts: %15llu next_trg_ts: %15llu  (pix vec size: %d, first pix ts: %llu)\n", trigger_vec[0].tlu_nr,curr_trg_ts, max_pixel_ts, next_trg_ts , pixel_vec.size(),fpts);

#endif		
	    // Loop over pixels 
	    for( int j = 0; j < pixel_vec.size(); ++j ) {
	      uint64_t curr_pix_ts = pixel_vec[j].ts;
	      if( curr_pix_ts < max_pixel_ts ) {
		
#ifdef TPX3_STORE_TXT
		long long diff = (long long)curr_trg_ts - (long long)curr_pix_ts;
		fprintf(fd,"%lld\n",diff);
#endif
#ifdef TPX3_VERBOSE
		printf("                        +  ts: %15llu diff: %15lld  (pix %3d,%3d)\n", curr_pix_ts, diff, pixel_vec[j].x, pixel_vec[j].y );
#endif
		// Pack pixel data into event buffer
		pack( bufferPix, pixel_vec[j].x );
		pack( bufferPix, pixel_vec[j].y );
		pack( bufferPix, pixel_vec[j].tot );
		pack( bufferPix, pixel_vec[j].ts );
		
		pixel_vec.erase( pixel_vec.begin() + j );
		j--; // after removing one pixel data packet, need to go back one step in the vector to avoid skipping pixels
	      }
	      else if( curr_pix_ts > next_trg_ts ) {
#ifdef TPX3_VERBOSE
                printf("                        -> break! (%llu > %llu , diff:%llu)\n",curr_pix_ts,next_trg_ts,curr_pix_ts-next_trg_ts  );
#endif	
		break;
	      }
	    }
	    
	    // Remove trigger from vector
	    trigger_vec.erase( trigger_vec.begin() );
	    // Add buffer to block
	    ev.AddBlock( 1, bufferPix );
	    // Send the event to the Data Collector      
	    SendEvent(ev);
	    // Now increment the event number
	    m_ev++;
	    trg_built++;
	  }
	  float size_proc = size*100.0/BUF_SIZE;
	  printf( "%c [%10d] Size %7d [%6.2f%%] Triggers %5d  Pixels %6lu       ", 13, cnt, size, size_proc, trg_built, pixel_vec.size() );
	  fflush( stdout ); 
	}
      }      

#ifdef TPX3_STORE_TXT
      fclose(fp);
      fclose(ft);
      fclose(fa);
      fclose(fd);
#endif      
      
      // Guess what this does?
      spidrctrl->closeShutter();

      // Disble TLU
      if( !spidrctrl->tlu_enable( device_nr, 0 ) ) error_out( "###tlu_enable" );

      // Clean up
      delete spidrdaq;
    }
  }

private:
  // This is just a dummy class representing the hardware
  // It here basically that the example code will compile
  // but it also generates example raw data to help illustrate the decoder
  unsigned m_run, m_ev;
  int m_spidrPort;
  int device_nr = 0;
  bool stopping, done,started;
  string m_spidrIP, m_xmlfileName, m_time, m_chipID;
  Timepix3Config *myTimepix3Config;
  SpidrController *spidrctrl;
  SpidrDaq *spidrdaq;
  Keithley2450 *k2450;
  int m_use_k2450, m_gpib_num;
  double m_Vbias, m_Ilim, m_Vstart, m_Vreturn, m_VbiasMax;
  int m_doBiasScan, m_VstepCount, m_VbiasStep;
  int m_xml_VTHRESH;
  int m_do_threshold_scan, m_threshold_start, m_threshold_step, m_threshold_max, m_threshold_return, m_threshold_count;
  float m_temp;
};


//////////////////////////////////////////////////////////////////////////////////
// main
//////////////////////////////////////////////////////////////////////////////////

// The main function that will create a Producer instance and run it
int main(int /*argc*/, const char ** argv) {

  std::cout << "==============> Timepix3Producer, main()..." << std::endl;

  // You can use the OptionParser to get command-line arguments
  // then they will automatically be described in the help (-h) option
  eudaq::OptionParser op("EUDAQ Timepix3 Producer", "1.0", "Just an example, modify it to suit your own needs");
  eudaq::Option<std::string> rctrl(op, "r", "runcontrol", "tcp://localhost:44000", "address", "The address of the RunControl.");
  eudaq::Option<std::string> level(op, "l", "log-level",  "NONE",                  "level",   "The minimum level for displaying log messages locally");
  eudaq::Option<std::string> name (op, "n", "name",       "Timepix3",              "string",  "The name of this Producer");

  // Handle CTRL+C interrupt signal (?)
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  try {
    // This will look through the command-line arguments and set the options
    op.Parse(argv);
    // Set the Log level for displaying messages based on command-line
    EUDAQ_LOG_LEVEL(level.Value());
    // Create a producer
    Timepix3Producer producer(name.Value(), rctrl.Value());
    // And set it running...
    producer.ReadoutLoop();
    // When the readout loop terminates, it is time to go
    std::cout << "Quitting" << std::endl;
  } catch (...) {
    // This does some basic error handling of common exceptions
    return op.HandleMainException();
  }
  

  return 0;
}


  /*

  // Open a control connection to SPIDR-TPX3 module
  // with address 192.168.100.10, default port number 50000
  SpidrController spidrctrl( 192, 168, 100, 10 );

  // Are we connected to the SPIDR-TPX3 module?
  if( !spidrctrl.isConnected() ) 
    {
      std::cout << spidrctrl.ipAddressString() << ": " << spidrctrl.connectionStateString() << ", " << spidrctrl.connectionErrString() << std::endl;
      return 1;
    } 
  else 
    {
      std::cout << "\n------------------------------" << std::endl;
      std::cout << "SpidrController is connected!" << std::endl;
      std::cout << "Class version: " << spidrctrl.versionToString( spidrctrl.classVersion() ) << std::endl;
      int firmwVersion, softwVersion = 0;
      if( spidrctrl.getFirmwVersion( &firmwVersion ) ) std::cout << "Firmware version: " << spidrctrl.versionToString( firmwVersion ) << std::endl;
      if( spidrctrl.getSoftwVersion( &softwVersion ) ) std::cout << "Software version: " << spidrctrl.versionToString( softwVersion ) << std::endl;
      std::cout << "------------------------------\n" << std::endl;
    }

  // Interface to Timepix3 pixel data acquisition
  SpidrDaq spidrdaq( &spidrctrl );
  string errstr = spidrdaq.errorString();
  if( !errstr.empty() ) cout << "###SpidrDaq: " << errstr << endl;
  
  int device_nr = 0;
  
  // ----------------------------------------------------------
  // DACs configuration
  if( !spidrctrl.setDacsDflt( device_nr ) )
    {
      error_out( "###setDacsDflt" );
    }
  
  // ----------------------------------------------------------
  // Enable decoder
  if( !spidrctrl.setDecodersEna( 1 ) )
    {
      error_out( "###setDecodersEna" );
    }

  // ----------------------------------------------------------
  // Pixel configuration
  if( !spidrctrl.resetPixels( device_nr ) )
    error_out( "###resetPixels" );

  // Enable test-bit in all pixels
  spidrctrl.resetPixelConfig();
  spidrctrl.setPixelTestEna( ALL_PIXELS, ALL_PIXELS );
  if( !spidrctrl.setPixelConfig( device_nr ) ) 
    {
      error_out( "###setPixelConfig" );
    }

  // ----------------------------------------------------------
  // Test pulse and CTPR configuration

  // Timepix3 test pulse configuration
  if( !spidrctrl.setTpPeriodPhase( device_nr, 100, 0 ) ) 
    {
      error_out( "###setTpPeriodPhase" );
    }
  if( !spidrctrl.setTpNumber( device_nr, 1 ) ) 
    {
      error_out( "###setTpNumber" );
    }

  // Enable test-pulses for some columns
  int col;
  for( col=0; col<256; ++col ) 
    {
      if( col >= 10 && col < 12 ) 
	{
	  spidrctrl.setCtprBit( col );
	}
    }
  
  if( !spidrctrl.setCtpr( device_nr ) ) 
    {
      error_out( "###setCtpr" );
    }

  // ----------------------------------------------------------
  // SPIDR-TPX3 and Timepix3 timers
  if( !spidrctrl.restartTimers() ) 
    {
      error_out( "###restartTimers" );
    }

  // Set Timepix3 acquisition mode
  if( !spidrctrl.setGenConfig( device_nr,
			       TPX3_POLARITY_HPLUS |
			       TPX3_ACQMODE_TOA_TOT |
			       TPX3_GRAYCOUNT_ENA |
			       TPX3_TESTPULSE_ENA |o
			       TPX3_FASTLO_ENA |
			       TPX3_SELECTTP_DIGITAL ) )
    {
      error_out( "###setGenConfig" );
    }
  
  // Set Timepix3 into acquisition mode
  if( !spidrctrl.datadrivenReadout() ) 
    {
      error_out( "###datadrivenReadout" );
    }
  
  // ----------------------------------------------------------
  // Configure the shutter trigger
  int trigger_mode = 4;           // SPIDR_TRIG_AUTO;
  int trigger_length_us = 100000; // 100 ms
  int trigger_freq_hz = 2;        // 2 Hz
  int trigger_count = 10;         // 10 triggers
  if( !spidrctrl.setTriggerConfig( trigger_mode, trigger_length_us, trigger_freq_hz, trigger_count ) ) 
    {
      error_out( "###setTriggerConfig" );
    }
  
  // Sample pixel data while writing the data to file (run number 123)
  spidrdaq.setSampling( true );
  spidrdaq.startRecording( "/data/test/test.dat", 123, "This is test data." );

  // ----------------------------------------------------------
  // Get data samples and display some pixel data details
  // Start triggers
  if( !spidrctrl.startAutoTrigger() )
    {
      error_out( "###startAutoTrigger" );
    }
  int cnt = 0, size, x, y, pixdata, timestamp;
  bool next_sample = true;
  while( next_sample )
    {
      // Get a sample of (at most) 1000 pixel data packets, waiting up to 3 s for it
      next_sample = spidrdaq.getSample( 10*8, 3000 );
      if( next_sample )
	{
	  ++cnt;
	  size = spidrdaq.sampleSize();
	  cout << "Sample " << cnt << " size=" << size << endl;
	  while( spidrdaq.nextPixel( &x, &y, &pixdata, &timestamp ) ) 
	    {
	      cout << x << "," << y << ": " << dec << pixdata << endl;
	    }
	}
      else
	{
	  cout << "### Timeout --> finish" << endl;
	}
    }
  */
  
