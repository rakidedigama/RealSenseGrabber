#include <rs.hpp>
#include "grabber.h"
#include "iostream"
#include "IniFile.h"
#include "QConsoleDebugStream.h"







GrabberRS::GrabberRS(std::string sIniFile): logStream(std::cout,"log_GrabberRSLib")
{
    m_sIni = sIniFile;
    m_bStopRequested = false;
    m_GrabState = STOPPED;

    m_BufferHeader.m_uImageHeight = 0;
    m_BufferHeader.m_uImageWidth = 0;
    m_BufferHeader.m_iLastFrame = -1;
    m_BufferHeader.m_iLastRead = -1;
    m_BufferHeader.m_uSizeOfFrame = 0;
    m_BufferHeader.m_uSlots = 0;

}

GrabberRS::~GrabberRS()
{
    m_bStopRequested = true;
    wait();
}

class RealSenseCamera
{
public:
    RealSenseCamera::RealSenseCamera(rs2::pipeline *p){
       pipe = p;
        profile = pipe->start();

    }

    void RealSenseCamera::initialize(){
        depth_scale = get_depth_scale(profile.get_device());
        align_to = find_stream_to_align(profile.get_streams());

    }

    rs2_stream RealSenseCamera::get_stream_to_align(){
        align_to = find_stream_to_align(profile.get_streams());
        return align_to;
    }


    rs2_stream RealSenseCamera::find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
    {
        //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
        //We prioritize color streams to make the view look better.
        //If color is not available, we take another stream that (other than depth)
        rs2_stream align_to = RS2_STREAM_ANY;
        bool depth_stream_found = false;
        bool color_stream_found = false;
        for (rs2::stream_profile sp : streams)
        {
            rs2_stream profile_stream = sp.stream_type();
            if (profile_stream != RS2_STREAM_DEPTH)
            {
                if (!color_stream_found)         //Prefer color
                    align_to = profile_stream;

                if (profile_stream == RS2_STREAM_COLOR)
                {
                    color_stream_found = true;
                }
            }
            else
            {
                depth_stream_found = true;
            }
        }

        if(!depth_stream_found)
            throw std::runtime_error("No Depth stream available");

        if (align_to == RS2_STREAM_ANY)
            throw std::runtime_error("No stream found to align with Depth");

        return align_to;
    }


    bool RealSenseCamera::profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
    {
        for (auto&& sp : prev)
        {
            //If previous profile is in current (maybe just added another)
            auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
            if (itr == std::end(current)) //If it previous stream wasn't found in current
            {
                return true;
            }
        }
        return false;
    }

    float RealSenseCamera::get_depth_scale(rs2::device dev)
    {
        // Go over the device's sensors
        for (rs2::sensor& sensor : dev.query_sensors())
        {
            // Check if the sensor if a depth sensor
            if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
            {
                return dpt.get_depth_scale();
            }
        }
        throw std::runtime_error("Device does not have a depth sensor");
    }

    rs2::pipeline_profile RealSenseCamera::get_profile(){
        return profile;
    }

    rs2::frameset RealSenseCamera::get_frameset(){
        rs2::frameset frameset = pipe->wait_for_frames();
        return frameset;
    }

    rs2::pipeline *pipe;
    rs2::pipeline_profile profile;
    float depth_scale;
    rs2_stream align_to;

};

void GrabberRS::run(){

    using namespace std;


    rs2::pipeline pipe;
//    pipe.poll_for_frames()
    //rs2::pipeline_profile profile = pipe.start();

    // Each depth camera might have different units for depth pixels, so we get it here
    // Using the pipeline's profile, we can retrieve the device that the pipeline uses
   // float depth_scale = get_depth_scale(profile.get_device());

    //Pipeline could choose a device that does not have a color stream
    //If there is no color stream, choose to align depth to another stream
    //rs2_stream align_to = find_stream_to_align(profile.get_streams());

    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    //The "align_to" is the stream type to which we plan to align depth frames.
   // rs2::align align(align_to);


//     rs2::config config;
//     config.enable_stream(RS2_STREAM_ANY, STREAM_INDEX, WIDTH, HEIGHT, FORMAT, FPS);
  //pipe.start();


    RealSenseCamera camera(&pipe);
    camera.initialize();
    rs2::align align(camera.get_stream_to_align());
    rs2::pipeline_profile profile = camera.get_profile();

    std::cout<<"Start streaming" << std::endl;

  while(!m_bStopRequested) // Application still alive?
  {
        std::cout<<"Getting frames"<<endl;
      // Alignement
      // Using the align object, we block the application until a frameset is available
      //rs2::frameset frameset = pipe.wait_for_frames();
      rs2::frameset frameset = camera.get_frameset();
      int framecount = frameset.size();
         std::cout<<"Got " << framecount << "frames"<<endl;
      // rs2::pipeline::wait_for_frames() can replace the device it uses in case of device error or disconnection.
      // Since rs2::align is aligning depth to some other stream, we need to make sure that the stream was not changed
      //  after the call to wait_for_frames();
      if (camera.profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
      {
          //If the profile was changed, update the align object, and also get the new device's depth scale
          profile = pipe.get_active_profile();
          rs2::align(camera.get_stream_to_align());
      }

      std::cout<<"aligned"<<endl;

      //Get processed aligned frame
      auto processed = align.process(frameset);

      // Trying to get both other and aligned depth frames
      rs2::video_frame other_frame = processed.first(camera.get_stream_to_align()); // color
      rs2::depth_frame aligned_depth_frame = processed.get_depth_frame(); // depth

      int width_depth = aligned_depth_frame.get_width();
      int height_depth = aligned_depth_frame.get_height();
      int bitsPerPixel_depth = aligned_depth_frame.get_bits_per_pixel();

      int width_color = other_frame.get_width();
      int height_color = other_frame.get_height();
      int bitsPerPixel_color = other_frame.get_bits_per_pixel();

      // conversion from const void* to unsigned char*
      unsigned char* frame_data = reinterpret_cast<unsigned char*>(const_cast<void*>(aligned_depth_frame.get_data()));
      //unsigned char* frame_data =

      std::cout << "Depth frames : " <<"width: " << width_depth  << "height: " << height_depth << "bits per pixel: " << bitsPerPixel_depth <<endl;
      std::cout << "Color frames : " <<"width: " << width_color << "height: " << height_color << "bits per pixel: " << bitsPerPixel_color << endl;
      std::cout << "Depth bytes " << aligned_depth_frame.get_bytes_per_pixel();

      //m_pBuffer = new unsigned char[m_BufferHeader.m_uSlots * m_BufferHeader.m_uSizeOfFrame];
      m_BufferHeader.m_uSizeOfFrame = 2*width_depth*height_depth;
      m_pBuffer = new unsigned char[1 *(m_BufferHeader.m_uSizeOfFrame)];


      if(width_depth!=0){
          std::cout<<"Frame not empty";
          std::cout<<"Frame Size : " << bitsPerPixel_depth*width_depth*height_depth;
          memcpy(m_pBuffer,frame_data,(2*width_depth*height_depth));
          std::cout<<"Frame copied to buffer" << endl;
          m_GrabState = GRABBING;

      }
      else{

          std::cout<<"Frame EMPTY" <<endl;
           m_GrabState =STOPPED;
      }





  }





   //return EXIT_SUCCESS;

}


/*void GrabberRS::run()
{
    using namespace std;


    IniFile ini(m_sIni.c_str());
    std::string sPvXML = ini.GetSetValue("Camera", "Settings", "SE.pvxml" , "PvXML config file path");
    std::string sCameraIP = ini.GetSetValue("Camera","StreamIP","16.0.0.100","Camera IP Address");
    std::string sMAC = ini.GetSetValue("Camera","MACAddress","00-11-1C-00-C3-82","Esim.00-11-1C-00-C3-82");
    std::string sHost = ini.GetSetValue("Host","IP","16.0.0.55","IP address of THIS computer");

    unsigned uRotationAngle = ini.GetSetValue("Camera", "RotationAngle", 0 , "Rotation angle 0/90/180/270");
    unsigned uOverexposeThreshold = ini.GetSetValue("Camera", "OverExposedTreshold", 1023 , "10bit = 1023, 8bit = 254");


    std::string sLedIP = ini.GetSetValue("Led", "ControllerIP", "127.0.0.1" , "Garda IP Address");



    if (ini.IsDirty())
        ini.Save();


    bool bStopped = true;

    bTrigger = false;
    bLastTrigger = false;


    m_BufferHeader.m_iLastFrame = -1;
    m_BufferHeader.m_iLastRead = -1;

    PvDeviceGEV camera;
    PvGenParameterArray*	pCameraParams;
    PvGenCommand* pStartCommand;
    PvGenCommand* pStopCommand;
    PvGenInteger* pTLLocked;
    PvGenCommand* pResetTimestamp;
    PvGenParameterArray *lCommParams;

    PvGenEnum* pTrigger;
    PvGenEnum* pTriggerSource;

    try
    {

        PvString pvMac = sMAC.c_str();
        PvString pvCameraIP = sCameraIP.c_str();
        PvString pvHostIP = sHost.c_str();

        if (!camera.SetIPConfiguration(pvMac,pvCameraIP ,"255.255.255.0",pvHostIP ).IsOK())
            cout << "GrabberRSLIB: Could not set IP configuration to device" << endl;

        if (camera.IsConnected())
        {
            cout << " GrabberRSLIB: Disconnect camera.." << endl;
            camera.Disconnect();
        }


        if(!camera.Connect(pvCameraIP,PvAccessControl).IsOK())
        {
            cout << "GrabberRSLIB: Unable to connect to camera. Returns false, no selection window opened" << endl;
            return;
        }
        else
        {
            cout << "GrabberRSLIB: Connected to camera " << sCameraIP << endl;
        }



        PvConfigurationReader pvConfigReader;
        if (!pvConfigReader.Load(sPvXML.c_str()).IsOK())
        {
            cout << "GrabberRSLIB: Could not open settings file " << sPvXML << endl;
        }
        else
        {
            if (!pvConfigReader.Restore("DeviceConfiguration",&camera).IsOK())
                cout << "GrabberRSLIB: Could not write settings to camera" << endl;
        }


        camera.NegotiatePacketSize();

        pCameraParams = camera.GetParameters();
        pStartCommand = dynamic_cast<PvGenCommand *>( pCameraParams->Get( "AcquisitionStart" ) );
        pStopCommand = dynamic_cast<PvGenCommand *>( pCameraParams->Get( "AcquisitionStop" ) );
        pResetTimestamp = dynamic_cast<PvGenCommand *>( pCameraParams->Get( "GevTimestampControlReset" ) );
        pTLLocked = dynamic_cast<PvGenInteger *>( pCameraParams->Get( "TLParamsLocked" ) );
        pTrigger = dynamic_cast<PvGenEnum *>( pCameraParams->Get( "TriggerMode" ) );
        pTriggerSource = dynamic_cast<PvGenEnum *>( pCameraParams->Get( "TriggerSource" ) );

        lCommParams = camera.GetCommunicationParameters ();// GetGenLink();
        PvGenBoolean* pLinkRecoveryEnabled = lCommParams->GetBoolean( "LinkRecoveryEnabled" );
        pLinkRecoveryEnabled->SetValue( true );

    }
    catch (...)
    {
        cout << "GrabberRSLIB: GrabberRS catch, exit" << endl;
        return;

    }

    PvStreamGEV stream;
    if (!stream.Open(sCameraIP.c_str()))
    {
        cout << "GrabberRSLIB: Could not open stream" << endl;
    }

    PvPipeline lPipeline( &stream );
    qint64 lSize = 0;
    pCameraParams->GetIntegerValue( "PayloadSize", lSize );
    m_uPayloadSize = lSize;

    // Set the Buffer size and the Buffer count
    lPipeline.SetBufferSize( static_cast<quint32>( lSize ) );
    lPipeline.SetBufferCount( 16 ); // Increase for high frame rate without missing block IDs

    m_BufferHeader.m_uSlots = 16;
    cout << "GrabberRSLIB: Payload size " << lSize << endl;
    m_BufferHeader.m_uSizeOfFrame = lSize;
    m_pBuffer = new unsigned char[m_BufferHeader.m_uSlots * m_BufferHeader.m_uSizeOfFrame];
    cout << "GrabberRSLIB: Buffer size " << m_BufferHeader.m_uSizeOfFrame << " is " << m_BufferHeader.m_uSlots << " * " << m_BufferHeader.m_uSizeOfFrame << endl;

    // Have to set the Device IP destination to the Stream
    camera.SetStreamDestination( stream.GetLocalIPAddress(), stream.GetLocalPort() );


    while (!m_bStopRequested)
    {
//       if (bSetStringParameterPending)
//        {
//            if ()
//            pTrigger->SetValue()
//            bSetStringParameterPending = false;
//        }
        if (bTrigger != bLastTrigger)
        {
            if (bTrigger)
            {
                pTrigger->SetValue(1);
                pTriggerSource->SetValue(27);
                cout << "SET TRIGGER ON" << endl;
            }
            else
            {
                pTrigger->SetValue(1);
                pTriggerSource->SetValue(29);
                cout << "SET TRIGGER OFF " << endl;
            }
            bLastTrigger = bTrigger;

        }

        if (GRABBING == m_GrabState && bStopped == true)
        {
            lPipeline.Start();
            pTLLocked->SetValue(1);
            pResetTimestamp->Execute();
            pStartCommand->Execute();
            bStopped = false;
            cout << "GrabberRSLIB: Grabbing started" << endl;
        }
        if (STOPPED == m_GrabState && bStopped == false)
        {
            lPipeline.Stop();
            pTLLocked->SetValue(0);
            pStopCommand->Execute();
            bStopped = true;
            cout << "GrabberRSLIB: Grabbing stopped" << endl;
        }

        if (GRABBING == m_GrabState && bStopped == false)
        {
            PvBuffer *lBuffer = NULL;
            PvResult  lOperationResult;
            PvResult lResult = lPipeline.RetrieveNextBuffer( &lBuffer, 1000, &lOperationResult );

            if ( lResult.IsOK() )
            {
                if ( lOperationResult.IsOK() )
                {
                   //
                   // We now have a valid buffer. This is where you would typically process the buffer.
                   // -----------------------------------------------------------------------------------------
                   // ...


//                   lStreamParams->GetIntegerValue( "ImagesCount", lImageCountVal );
//                   lStreamParams->GetFloatValue( "AcquisitionRateAverage", lFrameRateVal );
//                   lStreamParams->GetFloatValue( "BandwidthAverage", lBandwidthVal );

                   // If the buffer contains an image, display width and height
                   quint32 lWidth = 0, lHeight = 0;
                   if ( lBuffer->GetPayloadType() == PvPayloadTypeImage )
                   {
                       // Get image specific buffer interface
                       PvImage *lImage = lBuffer->GetImage();

                       // Read width, height
                       lWidth = lBuffer->GetImage()->GetWidth();
                       lHeight = lBuffer->GetImage()->GetHeight();

                       //cout << "Depth obviously " << lBuffer->GetAcquiredSize()/(lWidth*lHeight) << " vs. " << sizeof(unsigned short) << " (" << lBuffer->GetAcquiredSize() << ")" <<endl;


                       m_BufferHeader.m_uImageHeight = lHeight;
                       m_BufferHeader.m_uImageWidth = lWidth;

                       unsigned uExtras = 0;

                       if (lBuffer->GetAcquiredSize()/(lWidth*lHeight) == sizeof(unsigned short)) // 16-bit
                            m_BufferHeader.m_uSizeOfLastFrame = lBuffer->GetAcquiredSize();

                       if (lBuffer->GetAcquiredSize()/(lWidth*lHeight) == sizeof(unsigned char))// 8-bit
                       {
                           m_BufferHeader.m_uSizeOfLastFrame = lBuffer->GetAcquiredSize();
                           if (m_BufferHeader.m_uSizeOfFrame < m_BufferHeader.m_uSizeOfLastFrame)
                           {
                               uExtras = m_BufferHeader.m_uSizeOfLastFrame - m_BufferHeader.m_uSizeOfFrame;
                               cout << "Extra bits in 8-bit " << uExtras << endl;
                               m_BufferHeader.m_uSizeOfLastFrame = m_BufferHeader.m_uSizeOfLastFrame - uExtras;
                           }
                       }

                       // 24bit rgb voi menna sekaisin..?
                       if (m_BufferHeader.m_uSizeOfFrame == m_BufferHeader.m_uSizeOfLastFrame)
                       {

                           unsigned uSlot = m_BufferHeader.m_iLastFrame + 1;
                           if (uSlot > m_BufferHeader.m_uSlots - 1)
                               uSlot = 0;


                            memcpy(
                                m_pBuffer + (uSlot*m_BufferHeader.m_uSizeOfFrame),
                                lBuffer->GetDataPointer(),
                                lBuffer->GetAcquiredSize()-uExtras);


                           m_BufferHeader.m_uBlockId = lBuffer->GetBlockID();
                           m_BufferHeader.m_RecTick = lBuffer->GetTimestamp();
                           m_BufferHeader.m_iLastFrame = uSlot;

                        }
                       else
                       {
                           cout << "GrabberRSLIB: " << m_BufferHeader.m_uSizeOfFrame << "!=" << m_BufferHeader.m_uSizeOfLastFrame<<endl;
                       }

                   }


               }
               // We have an image - do some processing (...) and VERY IMPORTANT,
               // release the buffer back to the pipeline
               lPipeline.ReleaseBuffer( lBuffer );
           }
           else
           {
                cout << "GrabberRSLIB: Image wait timeout..." << endl;
           }
        } // GRABBING end

        if (STOPPED == m_GrabState)
        {
            LSleep(0.1);
        }
    }

    pStopCommand->Execute();
    pTLLocked->SetValue(0);
    lPipeline.Stop();
    stream.Close();
    camera.Disconnect();
}
*/


void GrabberRS::stopGrabbing()
{
    m_GrabState = STOPPED;
}

void GrabberRS::startGrabbing()
{
    m_GrabState = GRABBING;
}

unsigned GrabberRS::getImageSize()
{
    return m_BufferHeader.m_uSizeOfFrame;
}

unsigned GrabberRS::getImageWidth()
{
    return m_BufferHeader.m_uImageWidth;
}

unsigned GrabberRS::getImageHeight()
{
    return m_BufferHeader.m_uImageHeight;
}

int GrabberRS::getImage(unsigned short *pBuffer, unsigned uBufferSize, unsigned uTimeout){
    std::cout<<"size of frame"<< m_BufferHeader.m_uSizeOfFrame << "buffer size" << uBufferSize << std::endl;
    if (uBufferSize == m_BufferHeader.m_uSizeOfFrame && m_BufferHeader.m_uSizeOfFrame>0){
        if(m_GrabState==GRABBING){
            memcpy(pBuffer,m_pBuffer, m_BufferHeader.m_uSizeOfFrame);
         //return (m_BufferHeader.m_uSizeOfFrame/uBufferSize);
            return 1;
         }
    }
    else
        return -1;
}

//int GrabberRS::getImage(unsigned short *pBuffer, unsigned uBufferSize, unsigned uTimeout)
//{
//    using namespace std;
//    if (m_BufferHeader.m_iLastFrame < 0)
//        return -1;


//    if (uBufferSize * sizeof(unsigned short) == m_BufferHeader.m_uSizeOfFrame)
//    {
//        unsigned uTimer = 0;
//        while (m_BufferHeader.m_iLastRead == m_BufferHeader.m_iLastFrame && uTimer < uTimeout)
//        {
//            uTimer+= 10;
//            sleep(10);
//        }

//        if (m_BufferHeader.m_iLastRead == m_BufferHeader.m_iLastFrame && uTimer >= uTimeout)
//        {
//            cout << "GrabberRSLIB: Image retrieve timeout..!" << endl;
//            return -1;
//        }
//        memcpy(pBuffer,m_pBuffer + (m_BufferHeader.m_uSizeOfFrame * m_BufferHeader.m_iLastFrame), m_BufferHeader.m_uSizeOfFrame);
//        m_BufferHeader.m_iLastRead = m_BufferHeader.m_iLastFrame;
//        //return m_BufferHeader.m_iLastFrame;
//        return (m_BufferHeader.m_uSizeOfFrame/uBufferSize); // 2
//    }

//    if (uBufferSize * sizeof(unsigned char) == m_BufferHeader.m_uSizeOfFrame)
//    {
//        unsigned uTimer = 0;
//        while (m_BufferHeader.m_iLastRead == m_BufferHeader.m_iLastFrame && uTimer < uTimeout)
//        {
//            uTimer+= 10;
//            sleep(10);
//        }

//        if (m_BufferHeader.m_iLastRead == m_BufferHeader.m_iLastFrame && uTimer >= uTimeout)
//        {
//            cout << "GrabberRSLIB: Image retrieve timeout..!" << endl;
//            return -1;
//        }
//        //memcpy(pBuffer,m_pBuffer + (m_BufferHeader.m_uSizeOfFrame * m_BufferHeader.m_iLastFrame), m_BufferHeader.m_uSizeOfFrame);
//        for (int i = 0; i < m_BufferHeader.m_uSizeOfFrame; i++)
//            pBuffer[i] = reinterpret_cast<unsigned char*>(m_pBuffer + (m_BufferHeader.m_uSizeOfFrame * m_BufferHeader.m_iLastFrame))[i];
//        m_BufferHeader.m_iLastRead = m_BufferHeader.m_iLastFrame;
//        //return m_BufferHeader.m_iLastFrame;
//        return (m_BufferHeader.m_uSizeOfFrame/uBufferSize); // 1
//    }

//    cout << "GrabberRSLIB:    Buffer was of incorrect size, " << uBufferSize  << "(" << uBufferSize << ") != " << m_BufferHeader.m_uSizeOfFrame << endl;
//    lastError = "GrabberRSLIB: Buffer was of incorrect size, most probablty because camera and ini sizes differ";
//    return -1;
//}

void GrabberRS::setCameraParameter(QString param, QString value)
{
    this->qsStringParameter = param;
    this->qsStringParameterValue = value;
    bSetStringParameterPending = true;
}

void GrabberRS::setTrigger(bool b)
{
    bTrigger = b;
}






