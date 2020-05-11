/**
 * @maskro_gige_cam.cpp
 * @author  Marcel Stüttgen <stuettgen@fh-aachen.de>
 * @author  Fabian Nicolai  <fabian.nicolai@alumni.fh-aachen.de>
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * https://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * Ros based driver / tools for FLIR A325sc infrared camera
 */

#include <maskor_gige_cam/maskor_gige_cam.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>

#include <ros/assert.h>

namespace maskor
{


GigeCam::GigeCam()
{

  showImage = false;

  ImagePtr.reset(new cv::Mat());

  int x = _findDevices();

  if (x != -1)
  {
    open();
    start();
    setFocusPosition(2700);
    //setFocusDistance(3.0);
    setAutoFocusMethod(0);
    setFocusStep(100);
    //setFocusDistance(10);
    setIRFrameRate(1);
    setIRFormat(2);
    getSensorInfo();
  }
  else
  {
    std::cout << "could not find any device on the network" << std::endl;
    ROS_BREAK();
  }
}


GigeCam::~GigeCam()  {

  _freeStreamBuffers();

}


void GigeCam::open()
{
  std::cout << "GigeCam::open()" << std::endl;
  if ( DeviceInfo_ != NULL )
  {
    std::cout << "Connecting to " << DeviceInfo_ -> GetDisplayID().GetAscii() << std::endl;

    // Creates and connects the device controller based on the selected device.

    PvResult Result;

    Device_ = PvDevice::CreateAndConnect( DeviceInfo_, &Result );
    if ( !Result.IsOK() )
    {
      std::cout << "Unable to connect to " << DeviceInfo_->GetDisplayID().GetAscii() << std::endl;
      ROS_BREAK();
    }
    else
    {
      std::cout << "Successfully connected to " << DeviceInfo_->GetDisplayID().GetAscii() << std::endl;
      DeviceParams_ = Device_->GetParameters();
    }
  }
  else
  {
    std::cout << "No device found" << std::endl;
    ROS_BREAK();
  }
}

void GigeCam::close() {

  std::cout << "GigeCam::close()" << std::endl;
  std::cout << "Disconnecting device " << DeviceInfo_->GetDisplayID().GetAscii() << std::endl;
  Device_->Disconnect();

}


void GigeCam::start(){

  std::cout << "GigeCam::start()" << std::endl;

  // Open stream to the GigE Vision or USB3 Vision device
  std::cout << "Opening stream to device..." << std::endl;
  PvResult Result;
  Stream_ = PvStream::CreateAndOpen( DeviceInfo_->GetConnectionID(), &Result );
  if ( Stream_ == NULL )
  {
    std::cout << "Unable to stream from " << DeviceInfo_->GetDisplayID().GetAscii() << "!!!" << std::endl;
    ROS_BREAK();
  }

  std::cout << "Configuring stream...." << std::endl;

  // If this is a GigE Vision device, configure GigE Vision specific streaming parameters
  PvDeviceGEV* DeviceGEV = dynamic_cast<PvDeviceGEV*>(Device_);
  if ( DeviceGEV != NULL )
  {
    PvStreamGEV* StreamGEV = static_cast<PvStreamGEV *>( Stream_ );

    // Negotiate packet size
    DeviceGEV->NegotiatePacketSize();

    // Configure device streaming destination
    DeviceGEV->SetStreamDestination( StreamGEV->GetLocalIPAddress(), StreamGEV->GetLocalPort() );
  }

  // If this is a USB3 Vision device, configure GigE Vision specific streaming parameters
  PvDeviceU3V* DeviceU3V = dynamic_cast<PvDeviceU3V*>(Device_);
  if ( DeviceU3V != NULL )
  {
    //implement handling of usb device here //
  }


  // Reading payload size from device
  uint32_t payload = Device_ -> GetPayloadSize();

  std::cout << "Buffering stream...." << std::endl;
  // Use BUFFER_COUNT or the maximum number of buffers, whichever is smaller
  uint32_t lBufferCount = ( Stream_ -> GetQueuedBufferMaximum() < BUFFER_COUNT ) ?
      Stream_->GetQueuedBufferMaximum() :
      BUFFER_COUNT;

  // Allocate buffers
  for ( uint32_t i = 0; i < lBufferCount; i++ )
  {
    // Create new buffer object
    PvBuffer *lBuffer = new PvBuffer;

    // Have the new buffer object allocate payload memory
    lBuffer->Alloc( static_cast<uint32_t>( payload ) );

    // Add to external list - used to eventually release the buffers
    Bufferlist_.push_back( lBuffer );
  }

  // Queue all buffers in the stream
  std::list<PvBuffer*>::iterator it = Bufferlist_.begin();
  while ( it != Bufferlist_.end() )
  {
    Stream_->QueueBuffer( *it );
    it++;
  }

  //start the image capture thread
  ImageThread_.reset(new boost::thread (&GigeCam::_imageThread, this));

}



void GigeCam::stop() {

  std::cout << "GigeCam::stop()" << std::endl;
  std::cout << "Stopping stream from device..." << std::endl;

  ImageThread_.reset();

  // Tell the device to stop sending images
  std::cout << "Sending AcquisitionStop command to the device" << std::endl;
  DeviceParams_->ExecuteCommand( "AcquisitionStop" );

  // If present reset TLParamsLocked to 0. Must be done AFTER the
  // streaming has been stopped
  DeviceParams_->SetIntegerValue( "TLParamsLocked", 0 );

  // We stop the pipeline - letting the object lapse out of
  // scope would have had the destructor do the same, but we do it anyway
  //printf( "Stop pipeline\n" );
  //pipeline_.Stop();

  // Now close the stream. Also optionnal but nice to have
  std::cout << "Closing stream" << std::endl;
  Stream_->Close();
}



void GigeCam::_showImages(){

  cv::imshow("image_raw", *ImagePtr);

  cv::waitKey(60);
}


void GigeCam::_ImgtoBinary(){
  // cv::threshold(*ImagePtr,*ImagePtr,128,255,0);
}

void GigeCam::_changemode(int dir)
{
  static struct termios oldt, newt;
 
  if ( dir == 1 )
  {
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  }
  else
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}
 
int GigeCam::_kbhit (void)
{
  struct timeval tv;
  fd_set rdfs;
 
  tv.tv_sec = 0;
  tv.tv_usec = 0;
 
  FD_ZERO(&rdfs);
  FD_SET (STDIN_FILENO, &rdfs);
 
  select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &rdfs);
}

boost::shared_ptr<cv::Mat> GigeCam::getImage() {

  return ImagePtr;

}


void GigeCam::autoFocus()
{
  std::cout << "GigeCam::autoFocus()" << std::endl;
  DeviceParams_->ExecuteCommand( "AutoFocus" );
}

void GigeCam::setIRFormat(int i)
{
  std::cout << "GigeCam::setIRFormat(int i)" << std::endl;
  DeviceParams_->SetEnumValue( "IRFormat", i );
}

void GigeCam::setPixelFormat(int i)
{
  std::cout << "GigeCam::setPixelFormat(int i)" << std::endl;
  DeviceParams_->SetEnumValue( "PixelFormat", i );
}

void GigeCam::setAcquisitionMode(int i)
{
  std::cout << "GigeCam::setAcquisitionMode(int i)" << std::endl;
  DeviceParams_->SetEnumValue( "AcquisitionMode", i );
}

void GigeCam::setAcquisitionFrameCount(int i)
{
  std::cout << "GigeCam::setAcquisitionFrameCount(int i)" << std::endl;
  DeviceParams_->SetIntegerValue( "AcquisitionFrameCount", i );
}

void GigeCam::setTestImage(int i)
{
  std::cout << "GigeCam::setTestImage(int i)" << std::endl;
  DeviceParams_->SetEnumValue( "TestImageSelector", i );
}

void GigeCam::setSize(int w, int h)
{
  std::cout << "GigeCam::setSize(int w, int h)" << std::endl;
  DeviceParams_->SetIntegerValue( "Width", w );
  DeviceParams_->SetIntegerValue( "Height", h );
}

void GigeCam::setOffset(int x, int y)
{
  std::cout << "GigeCam::setOffset(int x, int y)" << std::endl;
  DeviceParams_->SetIntegerValue( (PvString)"OffsetX", (int64_t)x );
  DeviceParams_->SetIntegerValue( (PvString)"OffsetY", (int64_t)y );
}

void GigeCam::setFocusPosition(int i)
{
  // low = near, high = far (for Flir A325sc) min = 100, max = 3600
  std::cout << "GigeCam::setFocusPosition(double i)" << std::endl;
  DeviceParams_->SetIntegerValue("FocusPos", (int64_t)i );
}

void GigeCam::setFocusDistance(float i)
{
  //min = 0, max = 1000
  std::cout << "GigeCam::setFocusDistance (float i)" << std::endl;
  DeviceParams_->SetFloatValue("FocusDistance" , i );
}

void GigeCam::setFocusDirection(int i)
{
  //0 = stop, 1 = far, 2 = near
  std::cout << "GigeCam::setFocusDirection(int i)" << std::endl;
  DeviceParams_->SetEnumValue("FocusDirection", (int64_t)i );
}

void GigeCam::setFocusStep(int i)
{
  // min = 0, max = 1000 (< 250 could might not move the Focus)
  std::cout << "GigeCam::setFocusStep(int i)" << std::endl;
  DeviceParams_->SetIntegerValue("FocusStep", (int64_t)i );
}

void GigeCam::FocusDecrement()
{
  std::cout << "GigeCam::FocusDecrement()" << std::endl;
  DeviceParams_->ExecuteCommand( "FocusDecrement" );
}

void GigeCam::FocusIncrement()
{
  std::cout << "GigeCam::FocusIncrement()" << std::endl;
  DeviceParams_->ExecuteCommand( "FocusIncrement" );
}

void GigeCam::setAutoFocusMethod(int i)
{
  //0 = coarse, 1 = fine
  std::cout << "GigeCam::setAutoFocusMethod(int i)" << std::endl;
  DeviceParams_->SetEnumValue("AutoFocusMethod", (int64_t)i );
}

void GigeCam::setNUCMode(int i)
{
  //0 = off, 1 = auto
  std::cout << "GigeCam::setNUCMode(int i)" << std::endl;
  DeviceParams_->SetEnumValue("NUCMode", (int64_t)i );
}

void GigeCam::setNoiseReduction(int i)
{
  //0 = off, 1 = low, etc.
  std::cout << "GigeCam::setNoiseReduction(int i)" << std::endl;
  DeviceParams_->SetEnumValue("NoiseReduction",(int64_t)i );
}

void GigeCam::setIRFrameRate(int i)
{
  //0 = 60Hz, 1 = 30Hz, 2 = 15Hz
  std::cout << "GigeCam::setIRFrameRate(int i)" << std::endl;
  DeviceParams_->SetEnumValue("IRFrameRate", (int64_t)i );
}

void GigeCam::NUCAction()
{
  std::cout << "GigeCam::NUCAction()" << std::endl;
  DeviceParams_->ExecuteCommand( "NUCAction" );
}

void GigeCam::acquisitionStop()
{
  // Tell the device to stop sending images
  std::cout << "GigeCam::acquisitionStop()" << std::endl;
  DeviceParams_->ExecuteCommand( "AcquisitionStop" );
}

void GigeCam::acquisitionStart()
{
  //    // Tell the device to start sending images
  //    std::cout << "GigeCam::acquisitionStart()" << std::endl;
  //    DeviceParams_->ExecuteCommand( "AcquisitionStart" );

  PvGenCommand* p = DeviceParams_->GetCommand("AcquisitionStart");
  p->Execute();
}

void GigeCam::SensorSetDefaults()
{
  std::cout << "GigeCam::setSensorDefaults()" << std::endl;
  DeviceParams_->ExecuteCommand( "SensorSetDefaults" );
}

void GigeCam::SensorSetFactoryDefaults()
{
  std::cout << "GigeCam::SensorSetFactoryDefaults()" << std::endl;
  DeviceParams_->ExecuteCommand( "SensorSetFactoryDefaults" );
}

int GigeCam::getFocusPosition()
{
  int64_t focuspos; // low = near, high = far (for Flir A325sc)
  DeviceParams_->GetIntegerValue("FocusPos", focuspos);
  return (int)focuspos;
}

double GigeCam::getFocusDistance()
{
  double_t focusdistance; //min = 0, max = 1000
  DeviceParams_->GetFloatValue("FocusDistance" , focusdistance);
  return (double)focusdistance;
}

void GigeCam::getSensorInfo()
{
  std::cout << "\nGigeCam::getSensorInfo()" << std::endl;
  std::cout << "****** SENSOR INFO ********\n" << std::endl;

  std::cout << "\n--- Device Information registers ---\n"<< std::endl;
     
  //PvString deviceid;   // SEEMS TO BE EMPTY -_- 
  //DeviceParams_->GetStringValue("DeviceID", deviceid);
  //std::cout << "DeviceID: " << deviceid.GetAscii() << std::endl;

  PvString devicemodelname;
  DeviceParams_->GetStringValue("DeviceModelName", devicemodelname);
  std::cout << "DeviceModelName: " << devicemodelname.GetAscii() << std::endl;

  PvString devicemanufacturerinfo;
  DeviceParams_->GetStringValue("DeviceManufacturerInfo", devicemanufacturerinfo);
  std::cout << "DeviceManufacturerInfo: " << devicemanufacturerinfo.GetAscii() << std::endl;

  PvString devicevendorname;
  DeviceParams_->GetStringValue("DeviceVendorName", devicevendorname);
  std::cout << "DeviceVendorName: " << devicevendorname.GetAscii() << std::endl;

  PvString deviceversion;
  DeviceParams_->GetStringValue("DeviceVersion", deviceversion);
  std::cout << "DeviceVersion: " << deviceversion.GetAscii() << std::endl;

  int64_t camerasn;
  DeviceParams_->GetIntegerValue("CameraSN", camerasn);
  std::cout << "SerialNumber: " << camerasn << std::endl;
     

  std::cout << "\n--- GigE Vision Image size control registers ---\n" << std::endl;
     
     
  int64_t x,y;
  DeviceParams_->GetIntegerValue( "OffsetX" , x);
  DeviceParams_->GetIntegerValue( "OffsetY" , y);
  std::cout << "OffsetX: " << x << " OffsetY: " << y << std::endl;

  double_t sensortemp;
  DeviceParams_->GetFloatValue("SensorTemperature" , sensortemp);
  std::cout << "SensorTemperature: " << std::fixed << sensortemp << std::endl;

  double_t housingtemp;
  DeviceParams_->GetFloatValue("HousingTemperature" , housingtemp);
  std::cout << "HousingTemperature: " << std::fixed << housingtemp << std::endl;

  int64_t pixelformat;
  DeviceParams_->GetEnumValue("Pixelformat", pixelformat);
  std::cout << "PixelFormat: " << pixelformat << std::endl;
     
     
  std::cout << "\n--- Feature registers ---\n" << std::endl;
     
     
  int64_t cameracapabilities;
  DeviceParams_->GetIntegerValue("CameraCapabilities", cameracapabilities);
  std::cout << "CameraCapabilities: " << cameracapabilities << std::endl;
     

  std::cout << "\n--- Range (case) registers ---\n" << std::endl;
     
     
  int64_t numcases;
  DeviceParams_->GetIntegerValue("NumCases", numcases);
  std::cout << "NumCases: " << numcases << std::endl;

  int64_t currentcase;
  DeviceParams_->GetIntegerValue("CurrentCase", currentcase);
  std::cout << "CurrentCase: " << currentcase << std::endl;
     
     
  std::cout << "\n--- Focus registers ---\n" << std::endl;
     

  int64_t focusspeed;
  DeviceParams_->GetIntegerValue("FocusSpeed", focusspeed);
  std::cout << "FocusSpeed: " << focusspeed << std::endl;

  int64_t focusdirection;
  DeviceParams_->GetEnumValue("FocusDirection", focusdirection);
  std::cout << "FocusDirection: " << focusdirection << " (0 = stop, 1 = far, 2 = near)" << std::endl;

  int64_t autofocusmethod;
  DeviceParams_->GetEnumValue("AutoFocusMethod", autofocusmethod);
  std::cout << "AutoFocusMethod: " << autofocusmethod << " (0 = coarse, 1 = fine)" << std::endl;
     
  int64_t focuspos; // min = 100, max = 3600 (for Flir A325sc)
  DeviceParams_->GetIntegerValue("FocusPos", focuspos);
  std::cout << "FocusPos: " << focuspos << " (min = 100, max = 3600 (for Flir A325sc) )" << std::endl;

  double_t focusdistance; //min = 0, max = 1000
  DeviceParams_->GetFloatValue("FocusDistance" , focusdistance);
  std::cout << "FocusDistance: " << /*std::fixed* <<*/ focusdistance << " meters" << std::endl;

  int64_t focusstep; //stepsize for focusincrement and -decrement, min = 0, max = 1000, <250 might not move Focus
  DeviceParams_->GetIntegerValue("FocusStep", focusstep);
  std::cout << "FocusStep: " << focusstep << std::endl;
     
     
  std::cout << "\n--- Image quality registers ---\n" << std::endl;
     
     
  int64_t noisereduction;
  DeviceParams_->GetEnumValue("NoiseReduction", noisereduction);
  std::cout << "NoiseReduction: " << noisereduction << " (0 = off, 1 = low, etc)" << std::endl;

  int64_t nucmode;
  DeviceParams_->GetEnumValue("NUCMode",nucmode);
  std::cout << "NUCMode: " << nucmode << " (0 = off, 1 = auto)" << std::endl;


  std::cout << "\n--- Image stream registers ---\n" << std::endl;

  
  int64_t irframerate;
  DeviceParams_->GetEnumValue("IRFrameRate", irframerate);
  std::cout << "IRFrameRate: " << irframerate << " (0 = 60Hz, 1 = 30Hz, 2 = 15Hz)" << std::endl;


  int64_t irformat;
  DeviceParams_->GetEnumValue("IRFormat", irformat);
  std::cout << "IRFormat: " << irformat << " (0 = Signal linear, 1 = 0.1K resolution, 2 = 0.01K resolution)" << std::endl;

  
  std::cout << "\n***************************\n" << std::endl;

}


/// Find all devices on the network
int GigeCam::_findDevices() {

  int numberOfDevices = 0;


  std::cout << "Scanning for interfaces (network & usb)...." << std::endl;

  PvResult Result = System_.Find();

  if ( !Result.IsOK() )
  {
    std::cout << "PvSystem::Find Error: " << Result.GetCodeString().GetAscii();
    return -1;
  }
  else
  {
    int interfaceCount = System_.GetInterfaceCount();
    if (interfaceCount <= 0)
    {
      std::cout << "No Interfaces found" << std::endl;
      return -1;
    }

    std::cout << "... found the following Interface(s):" << std::endl;

    // Go through all interfaces
    for ( int i = 0; i < interfaceCount; i++ )
    {
      std::cout << "Interface: " << i << std::endl;

      PvInterface* Interface = const_cast<PvInterface*> ( System_.GetInterface(i) );

      //check if it is a network interface
      PvNetworkAdapter* NIC = dynamic_cast<PvNetworkAdapter*> ( Interface );
      if ( NIC != NULL )
      {
        std::cout << "  Type: Network Adapter (NIC)" << std::endl;
        std::cout << "  MAC Address: " << NIC-> GetMACAddress().GetAscii() << std::endl;
        std::cout << "  IP Address: " << NIC-> GetIPAddress(0).GetAscii() << std::endl;
        std::cout << "  Subnet Mask: " << NIC-> GetSubnetMask(0).GetAscii() << std::endl << std::endl;
      }

      //check if it is a USB interface
      PvUSBHostController* USB = dynamic_cast<PvUSBHostController*>( Interface );
      if ( USB != NULL )
      {
        std::cout << "  Type: USB Controller " << std::endl;
        std::cout << "  Name: " << USB->GetName().GetAscii() << std::endl << std::endl;
	std::cout << "Skipping USB device" << std::endl;
	continue;
      }

      std::cout << "Scanning interface " << i << " for Devices...." << std::endl;

      // Go through all the devices attached to the interface
      int deviceCount = Interface->GetDeviceCount();
      if (deviceCount <=0)
      {
        std::cout << "No devices found" << std::endl << std::endl;
      }
      else
      {
        for ( int j = 0; j < deviceCount ; j++ )
        {
          DeviceInfo_ = const_cast<PvDeviceInfo*> (Interface->GetDeviceInfo(j)) ;

          std::cout << "    Device: " << j << std::endl;
          std::cout << "    Display ID: " << DeviceInfo_->GetDisplayID().GetAscii() << std::endl;

          //GigE Device??
          PvDeviceInfoGEV* DeviceInfoGEV = dynamic_cast<PvDeviceInfoGEV*>(DeviceInfo_);
          if ( DeviceInfoGEV != NULL ) // GigE Vision device                                           
          {
            numberOfDevices++;
            std::cout << "    MAC Address: " << DeviceInfoGEV->GetMACAddress().GetAscii() << std::endl;
            std::cout << "    IP Address: " << DeviceInfoGEV->GetIPAddress().GetAscii() << std::endl;
            std::cout << "    Serial number: " << DeviceInfoGEV->GetSerialNumber().GetAscii() << std::endl << std::endl;
          }

          ///USB3 Device??
          PvDeviceInfoU3V* DeviceInfoU3V = dynamic_cast<PvDeviceInfoU3V*>(DeviceInfo_);
          if ( DeviceInfoU3V != NULL ) // USB3 Vision device
          {
            numberOfDevices++;
            std::cout << "    GUID: " << DeviceInfoU3V->GetDeviceGUID().GetAscii() << std::endl;
            std::cout << "    S/N: " << DeviceInfoU3V->GetSerialNumber().GetAscii() << std::endl;
            std::cout << "    Speed: " << USB->GetSpeed() << std::endl << std::endl;
          }
        }
      }
    }
  }

  if (numberOfDevices<=0)
  {
    std::cout << "return -1" << std::endl;
    return -1;
  }

  return 0;
}





void GigeCam::_imageThread()
{
  // Get device parameters need to control streaming
  DeviceParams_ = Device_->GetParameters();

  // Map the GenICam AcquisitionStart and AcquisitionStop commands
  PvGenCommand *lStart = dynamic_cast<PvGenCommand *>( DeviceParams_->Get( "AcquisitionStart" ) );
  PvGenCommand *lStop = dynamic_cast<PvGenCommand *>( DeviceParams_->Get( "AcquisitionStop" ) );

  // Get stream parameters
  StreamParams_ = Stream_->GetParameters();

  // Map a few GenICam stream stats counters
  PvGenFloat *lFrameRate = dynamic_cast<PvGenFloat *>( StreamParams_->Get( "AcquisitionRate" ) );
  PvGenFloat *lBandwidth = dynamic_cast<PvGenFloat *>( StreamParams_->Get( "Bandwidth" ) );

  // Enable streaming and send the AcquisitionStart command
  std::cout << "Enabling streaming and sending AcquisitionStart command." << std::endl;

  Device_->StreamEnable();
  lStart->Execute();

  char lDoodle[] = "|\\-|-/";
  int lDoodleIndex = 0;
  double lFrameRateVal = 0.0;
  double lBandwidthVal = 0.0;

  //main capture loop
  while ( true )
  {
    PvBuffer *lBuffer = NULL;
    PvResult lOperationResult;
    
    // Retrieve next buffer
    PvResult lResult = Stream_->RetrieveBuffer( &lBuffer, &lOperationResult, 1000 );
    
    if ( lResult.IsOK() )
    {
      if ( lOperationResult.IsOK() )
      {
        PvPayloadType lType;

        lFrameRate->GetValue( lFrameRateVal );
        lBandwidth->GetValue( lBandwidthVal );

        // If the buffer contains an image, display width and height.
        uint32_t lWidth = 0, lHeight = 0;
        lType = lBuffer->GetPayloadType();
        
        std::cout << std::fixed << std::setprecision( 1 );
        std::cout << lDoodle[ lDoodleIndex ];
        std::cout << " BlockID: " << std::uppercase << std::hex << std::setfill( '0' ) << std::setw( 16 ) << lBuffer->GetBlockID();
        
        if ( lType == PvPayloadTypeImage )
        {
          // Get image specific buffer interface.
          PvImage_ = lBuffer->GetImage();
                             
          // Read width, height.
          lWidth = lBuffer->GetImage()->GetWidth();
          lHeight = lBuffer->GetImage()->GetHeight();
          std::cout << "  W: " << std::dec << lWidth << " H: " << lHeight;
        }
        else
        {
          std::cout << " (buffer does not contain image)";
        }
        std::cout << "  " << lFrameRateVal << " FPS  " << ( lBandwidthVal / 1000000.0 ) << " Mb/s   \r";
      }
      else
      {
        // Non OK operational result
        std::cout << lDoodle[ lDoodleIndex ] << " " << lOperationResult.GetCodeString().GetAscii() << "\r";
      }

      //create opencv image and bind data from PVImage to it
      ImagePtr->create(cv::Size(PvImage_->GetWidth(),PvImage_->GetHeight()),CV_16UC1);
      PvImage_->Attach(ImagePtr->data, PvImage_->GetWidth(),PvImage_->GetHeight(),PvPixelMono16);

      //display raw image
      if(showImage)
      {
        _showImages();
      }
      
      double focusdistance = getFocusDistance(); //min = 0, max = 1000

      //Get keyboard input whithout interruption
      char ch = '0';
      _changemode(1);
      if( _kbhit() )
      {
        ch = getchar();
      }

      switch(ch){
        case 'a':
          autoFocus();
          break;
        case '+':
          FocusIncrement();
          //if(focusdistance != 1000)
          //{
          //  setFocusDistance(focusdistance + 1);
          //}
          getSensorInfo();
          break;
        case '-':
          FocusDecrement();
          //if(focusdistance != 0)
          //{
          //  setFocusDistance(focusdistance - 1);
          //}
          getSensorInfo();
          break;
        case 'i':
          getSensorInfo();
          break;
        case 'f':
          int64_t autofocusmethod;
          DeviceParams_->GetEnumValue("AutoFocusMethod", autofocusmethod);  
          if(autofocusmethod == 0)
          {
            setAutoFocusMethod(1);
          }else{
            setAutoFocusMethod(0);
          }
          break;
        case 'p':
          //Only exist to test function
          setFocusPosition(3000);
          break;
      }
       
      _changemode(0);

      int focuspos = getFocusPosition();
      std::cout << "FocusPos: " << focuspos << " (min = 100, max = 3600 (for Flir A325sc) )" << std::endl;

      focusdistance = getFocusDistance();
      //std::cout << "FocusDistance: " << /*std::fixed << */focusdistance << " meters" << std::endl;

      // Re-queue the buffer in the stream object
      Stream_->QueueBuffer( lBuffer );
    }
    else
    {
      // Retrieve buffer failure
      std::cout << lDoodle[ lDoodleIndex ] << " " << lResult.GetCodeString().GetAscii() << "\r";
    }

    ++lDoodleIndex %= 6;
  }
}


void GigeCam::_freeStreamBuffers()
{
  // Tell the device to stop sending images.
  PvGenCommand *lStop = dynamic_cast<PvGenCommand *>( DeviceParams_->Get( "AcquisitionStop" ) );
  std::cout << "Sending AcquisitionStop command to the device" << std::endl;
  lStop->Execute();

  // Disable streaming on the device
  std::cout << "Disable streaming on the controller." << std::endl;
  Device_->StreamDisable();

  // Abort all buffers from the stream and dequeue
  std::cout << "Aborting buffers still in stream" << std::endl;
  Stream_->AbortQueuedBuffers();
  while ( Stream_->GetQueuedBufferCount() > 0 )
  {
    PvBuffer *lBuffer = NULL;
    PvResult lOperationResult;

    Stream_->RetrieveBuffer( &lBuffer, &lOperationResult );
  }


  // Go through the buffer list
  std::list<PvBuffer*>::iterator lIt = Bufferlist_.begin();
  while ( lIt != Bufferlist_.end() )
  {
    delete *lIt;
    lIt++;
  }

  // Clear the buffer list
  Bufferlist_.clear();

  std::cout << "clearing buffers..." << std::endl;
}


}


