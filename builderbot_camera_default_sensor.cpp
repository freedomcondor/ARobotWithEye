/**
 * @file <argos3/plugins/robots/builderbot/hardware/builderbot_camera_default_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "builderbot_camera_default_sensor.h"

#include <argos3/core/utility/logging/argos_log.h>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/image_u8.h>
#include <apriltag/zarray.h>

//------about camera ------------------
#include <fcntl.h>      // for ::open, in openCamera
#include <sys/ioctl.h>  // for ioctl, camera capture
#include <linux/videodev2.h>  // for VIDIOC_S_INPUT, in openCamera
#include <sys/mman.h>
//-------------------------------------

namespace argos {

   /****************************************/
   /****************************************/

   CBuilderBotCameraDefaultSensor::CBuilderBotCameraDefaultSensor() {
      /* initialize the apriltag components */
      m_psTagFamily = tag36h11_create();
      m_psTagFamily->black_border = 1;
      /* create the tag detector */
      m_psTagDetector = apriltag_detector_create();
      /* add the tag family to the tag detector */
      apriltag_detector_add_family(m_psTagDetector, m_psTagFamily);
      /* configure the tag detector */
      m_psTagDetector->quad_decimate = 1.0f;
      m_psTagDetector->quad_sigma = 0.0f;
      m_psTagDetector->nthreads = 1;
      m_psTagDetector->debug = 0;
      m_psTagDetector->refine_edges = 1;
      m_psTagDetector->refine_decode = 0;
      m_psTagDetector->refine_pose = 0;
   }

   /****************************************/
   /****************************************/

   CBuilderBotCameraDefaultSensor::~CBuilderBotCameraDefaultSensor() {
      /* uninitialize the apriltag components */
      apriltag_detector_remove_family(m_psTagDetector, m_psTagFamily);
      /* destroy the tag detector */
      apriltag_detector_destroy(m_psTagDetector);
      /* destroy the tag family */
      tag36h11_destroy(m_psTagFamily);
   }

   /****************************************/
   /****************************************/

   void CBuilderBotCameraDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_BuilderBotCameraSensor::Init(t_tree);
         /* parse and set the resolution */
         CVector2 cResolution;
         GetNodeAttribute(t_tree, "resolution", cResolution);
         m_unImageHeight = std::round(cResolution.GetY());
         m_unImageWidth = std::round(cResolution.GetX());
         /* allocate storage */
         m_sCurrentFrame.SetSize(m_unImageWidth, m_unImageHeight);
         /* parse input frame paths */
         TConfigurationNodeIterator itFrame("frame");
         std::string strFramePath;
         for(itFrame = itFrame.begin(&t_tree);
             itFrame != itFrame.end();
             ++itFrame) {
            /* get the path to the file */
            GetNodeAttribute(*itFrame, "src", strFramePath);
            m_vecInputFrames.emplace_back(strFramePath);
         }
         m_itInputFrameIterator = std::begin(m_vecInputFrames);

         if (OpenCamera("/dev/video0") < 0) printf("Camera Open failed\n");
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing camera sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CBuilderBotCameraDefaultSensor::Update() {
      if(m_itInputFrameIterator != std::end(m_vecInputFrames)) {
         image_u8_t* ptFrame = 
            image_u8_create_from_pnm(m_itInputFrameIterator->c_str());
         if(ptFrame == nullptr) {
            THROW_ARGOSEXCEPTION("Can not open " << *m_itInputFrameIterator
                                 << " as an 8-bit PNM image");
         }
         else {
            m_sCurrentFrame.Y.reset(ptFrame);
            /* other channels not defined */
            m_sCurrentFrame.U.release();
            m_sCurrentFrame.V.release();
            m_itInputFrameIterator++;
         }
      }
      else {
         m_sCurrentFrame.SetSize(m_unImageWidth / 2, m_unImageHeight / 2); 
         /* TODO: not /2 in the future */

         if ( GetFrame(m_sCurrentFrame.Y.get(),m_sCurrentFrame.U.get(),m_sCurrentFrame.V.get())
            <0 ) printf("get Frame failed\n");
      }

      /* detect tags */
      CVector2 c_center_pixel;
      std::array<CVector2, 4> arr_corner_pixels;
      /* run the apriltags algorithm */
      zarray_t* psDetectionArray =
         apriltag_detector_detect(m_psTagDetector, m_sCurrentFrame.Y.get());
      /* get the detected tags count */
      size_t unTagCount = static_cast<size_t>(zarray_size(psDetectionArray));

      /* clear out previous readings */
      m_tTags.clear();
      /* reserve space for the tags */
      m_tTags.reserve(unTagCount);
      /* process detections */
      for(size_t un_index = 0; un_index < unTagCount; un_index++) {
         apriltag_detection_t *psDetection;
         zarray_get(psDetectionArray, un_index, &psDetection);
         /* copy the tag corner coordinates */
         arr_corner_pixels[0].Set(psDetection->p[0][0], psDetection->p[0][1]),
         arr_corner_pixels[1].Set(psDetection->p[1][0], psDetection->p[1][1]),
         arr_corner_pixels[2].Set(psDetection->p[2][0], psDetection->p[2][1]),
         arr_corner_pixels[3].Set(psDetection->p[3][0], psDetection->p[3][1]),

         /* copy the tag center coordinate */
         c_center_pixel.Set(psDetection->c[0], psDetection->c[1]);
         /* copy reading to the control interface */
         m_tTags.emplace_back(std::to_string(psDetection->id), c_center_pixel, arr_corner_pixels);
      }
      /* destroy the readings array */
      apriltag_detections_destroy(psDetectionArray);
   }

   /****************************************/
   /****************************************/

   void CBuilderBotCameraDefaultSensor::GetPixels(const CVector2& c_offset,
                                                  const CVector2& c_size,
                                                  std::vector<SPixel>& vec_pixels) {
      /* round the given coordinates to look up the pixels */     
      UInt32 unOffsetX = std::round(c_offset.GetX());
      UInt32 unOffsetY = std::round(c_offset.GetY());
      UInt32 unWidth = std::round(c_size.GetX());
      UInt32 unHeight = std::round(c_size.GetY());
      UInt32 unFrameStride = m_sCurrentFrame.Y->stride;
      /* copy the requested pixels */
      for(UInt32 un_y = unOffsetY; un_y < unOffsetY + unHeight; un_y++) {
         for(UInt32 un_x = unOffsetX; un_x < unOffsetX + unWidth; un_x++) {
            vec_pixels.emplace_back(
               m_sCurrentFrame.Y->buf[un_y * unFrameStride + un_x],
               m_sCurrentFrame.U ? m_sCurrentFrame.U->buf[un_y * unFrameStride + un_x] : 0,
               m_sCurrentFrame.V ? m_sCurrentFrame.V->buf[un_y * unFrameStride + un_x] : 0);
         }
      }
   }

   /****************************************/
   /****************************************/

   void CBuilderBotCameraDefaultSensor::Write(const CBuilderBotCameraDefaultSensor::SFrame& s_frame,
                                              const std::string& str_file) {
      /* write to PNM file */
      static int frame = 0;
      image_u8_write_pnm(s_frame.Y.get(), (str_file + std::to_string(frame)).c_str());
      frame++;
   }

   /****************************************/
   /****************************************/

   //--- about camera -------------------------------------------
	#define NUMBUFFERS 2
	#define BYTEPERPIXEL 2     // temporary taking 1280*720 image from camera
	#define CAMERAHEIGHT 720
	#define CAMERAWIDTH 1280
   int nCamera;	// the handle for the camera

   int nCameraBytesPerPixel;
   int nCameraWidth;
   int nCameraHeight;
   int nCameraFrameSize;

   const int nNumBuffers = NUMBUFFERS;
   
   struct OvFrameBuffer {
      unsigned char* start;
      unsigned int length;
      unsigned int offset;
   };
   OvFrameBuffer buffers_[NUMBUFFERS];
   unsigned char* buffer_;
   int CBuilderBotCameraDefaultSensor::OpenCamera(const char *pch_device) {
      /* open camera */
      int input;
      struct v4l2_format fmt;
      struct v4l2_requestbuffers req;

      /* TODO: change the following to be calculated from m_unImageWidth m_unImageHeight in the future*/
      nCameraWidth = CAMERAWIDTH;  
      nCameraHeight = CAMERAHEIGHT;  
      nCameraBytesPerPixel = BYTEPERPIXEL;
      nCameraFrameSize = nCameraWidth * nCameraHeight * nCameraBytesPerPixel;

      if ((nCamera = ::open(pch_device, O_RDWR, 0)) < 0)    // declared in fcntl.h
         return -1;

      input = 0;  //default input No 0
      if (ioctl(nCamera, VIDIOC_S_INPUT, &input) < 0) 
         { close(nCamera); return -1; }

      fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      fmt.fmt.pix.width = nCameraWidth;  
      fmt.fmt.pix.height = nCameraHeight;
      fmt.fmt.pix.sizeimage = nCameraFrameSize;
      fmt.fmt.pix.bytesperline = nCameraWidth * nCameraBytesPerPixel;
      fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
      fmt.fmt.pix.field = V4L2_FIELD_NONE;
      buffer_ = new unsigned char[nCameraFrameSize];

      if (ioctl(nCamera, VIDIOC_S_FMT, &fmt) < 0)
         { close(nCamera); return -1; }

      memset(&req, 0, sizeof (req));
      req.count = nNumBuffers;
      req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      req.memory= V4L2_MEMORY_MMAP;
      if (ioctl (nCamera, VIDIOC_REQBUFS, &req) < 0) return -1;
      if (req.count < 2) return -1;

      /* Start Capture */
      int i;
      struct v4l2_buffer buf;
      enum v4l2_buf_type type;

      for (i = 0; i < nNumBuffers; i++)
      {
         memset (&buf, 0, sizeof (buf));
         buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
         buf.memory = V4L2_MEMORY_MMAP;
         buf.index = i;
         if (ioctl(nCamera, VIDIOC_QUERYBUF, &buf) < 0) return -1;
         buffers_[i].length = buf.length;
         buffers_[i].offset = (size_t) buf.m.offset;
         buffers_[i].start = static_cast<unsigned char*>(
                                 mmap (NULL, buffers_[i].length, 
                                       PROT_READ | PROT_WRITE, MAP_SHARED, 
                                       nCamera, buffers_[i].offset
                                 )
                             );
         if (ioctl (nCamera, VIDIOC_QBUF, &buf) < 0) return -1;
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (ioctl (nCamera, VIDIOC_STREAMON, &type) < 0) return -1;

      return 0;
   }

   int CBuilderBotCameraDefaultSensor::GetFrame(image_u8_t* pt_y_channel, 
                                                image_u8_t* pt_u_channel, 
                                                image_u8_t* pt_v_channel) {
      struct v4l2_buffer capture_buf;
      const unsigned int unScale = 2;
      struct SPixelData {
         unsigned char U0;
         unsigned char Y0;
         unsigned char V0;
         unsigned char Y1;
      };
      int current_buffer_index_;

      /* grab */
      memset (&capture_buf, 0, sizeof(capture_buf));
      capture_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      capture_buf.memory = V4L2_MEMORY_MMAP;
      if (ioctl(nCamera, VIDIOC_DQBUF, &capture_buf) < 0) return -1;

      buffer_ = buffers_[capture_buf.index].start;
      if (ioctl(nCamera, VIDIOC_QBUF, &capture_buf) < 0) return -1;

      /* convert */
      for (unsigned int unHeightIdx = 0; unHeightIdx < nCameraHeight; unHeightIdx++) {
         for (unsigned int unWidthIdx = 0; unWidthIdx < nCameraWidth; unWidthIdx++) {
            if((unHeightIdx % unScale == 0) && (unWidthIdx % unScale == 0)) {
               unsigned int unPixelOffset = 
                     unHeightIdx * (m_unImageWidth * nCameraBytesPerPixel) + (unWidthIdx * nCameraBytesPerPixel);
               unsigned int unDestIdx = 
                     (unHeightIdx / unScale) * (pt_y_channel->stride) + (unWidthIdx / unScale);

               SPixelData* psPixelData = reinterpret_cast<SPixelData*>(buffer_ + unPixelOffset);
            
               // split into seperate planes 
               pt_u_channel->buf[unDestIdx] = psPixelData->U0;
               pt_y_channel->buf[unDestIdx] = psPixelData->Y0;
               pt_v_channel->buf[unDestIdx] = psPixelData->V0;
            }
         }
      }

      return 0;
   }
   //end of camera------------------------------------------------------------

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CBuilderBotCameraDefaultSensor,
                   "builderbot_cam", "default",
                   "Michael Allwright [allsey87@gmail.com]",
                   "1.0",
                   "Camera sensor for the BuilderBot Robot",
                   "Camera sensor for the BuilderBot Robot\n",
                   "Under development");
}
