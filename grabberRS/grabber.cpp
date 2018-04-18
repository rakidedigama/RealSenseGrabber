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

    // Connect with device
    rs2::context ctx;
    auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
    while (list.size() == 0){
        //throw std::runtime_error("No device detected. Is it plugged in?");
        std::cout<<"No device detected. Grabber not started"<< std::endl;
        m_bStopRequested = true;
    }
    //rs2::device dev = list.front();



    rs2::pipeline pipe;

    RealSenseCamera camera(&pipe);
    camera.initialize();
    rs2::align align(camera.get_stream_to_align());
    rs2::pipeline_profile profile = camera.get_profile();

    std::cout<<"Start streaming" << std::endl;

  while(!m_bStopRequested) // Application still alive?
  {
        std::cout<<"Getting frames"<<endl;

      rs2::frameset frameset = camera.get_frameset();
      int framecount = frameset.size();
       std::cout<<"Got " << framecount << "frames"<<endl;

      if (camera.profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
      {
          //If the profile was changed, update the align object, and also get the new device's depth scale
          profile = pipe.get_active_profile();
          rs2::align(camera.get_stream_to_align());
      }     

      //Get processed aligned frames
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

      m_BufferHeader.m_uImageHeight = height_depth;
      m_BufferHeader.m_uImageWidth = width_depth;

      // conversion from const void* to unsigned char*
      unsigned char* depth_frame_data = reinterpret_cast<unsigned char*>(const_cast<void*>(aligned_depth_frame.get_data()));
      unsigned char* color_frame_data = reinterpret_cast<unsigned char*>(const_cast<void*>(other_frame.get_data()));

      std::cout << "Depth frames : " <<"width: " << width_depth  << "height: " << height_depth << "bits per pixel: " << bitsPerPixel_depth <<endl;
      std::cout << "Color frames : " <<"width: " << width_color << "height: " << height_color << "bits per pixel: " << bitsPerPixel_color << endl;
      std::cout << "Depth bytes " << aligned_depth_frame.get_bytes_per_pixel();

      //m_pBuffer = new unsigned char[m_BufferHeader.m_uSlots * m_BufferHeader.m_uSizeOfFrame];
      m_BufferHeader.m_uSizeOfFrame = 5*width_depth*height_depth; // 2 bytes for depth, 3 bytes for color
      m_pBuffer = new unsigned char[1 *(m_BufferHeader.m_uSizeOfFrame)];


      if(width_depth!=0){
          std::cout<<"Frame not empty";
          std::cout<<"Frame Size : " << bitsPerPixel_depth*width_depth*height_depth;
          memcpy(m_pBuffer,depth_frame_data,(2*width_depth*height_depth));
          memcpy(m_pBuffer + (2*width_depth*height_depth),color_frame_data,(3*width_depth*height_depth));

          std::cout<<"Frames copied to buffer" << endl;
          m_GrabState = GRABBING;

      }
      else{

          std::cout<<"Frame EMPTY" <<endl;
           m_GrabState =STOPPED;
      }





  }

   //return EXIT_SUCCESS;

}




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
    //std::cout<<"size of frame"<< m_BufferHeader.m_uSizeOfFrame << "buffer size" << uBufferSize << std::endl;
    if(m_GrabState!=GRABBING){
        return -1;
    }
    else{


        if(uBufferSize==(m_BufferHeader.m_uImageHeight*m_BufferHeader.m_uImageWidth*2) && m_BufferHeader.m_uSizeOfFrame>0 ){
            std::cout<<"Copying Depth Image from Buffer to Mat"<<std::endl;
            memcpy(pBuffer,m_pBuffer, m_BufferHeader.m_uSizeOfFrame*2/5);
            return 1;

        }
        if(uBufferSize==(m_BufferHeader.m_uImageHeight*m_BufferHeader.m_uImageWidth*3) && m_BufferHeader.m_uSizeOfFrame>0){
            std::cout<<"Copying Color Image from Buffer to Mat"<<std::endl;
             memcpy(pBuffer,m_pBuffer + m_BufferHeader.m_uSizeOfFrame*2/5, m_BufferHeader.m_uSizeOfFrame*3/5);
              return 1;
        }


        else
            return -1;
    }
}


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






