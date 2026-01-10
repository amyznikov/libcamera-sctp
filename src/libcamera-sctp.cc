/*
 * libcamera-sctp.cc
 *
 *  Created on: Dec 30, 2025
 *      Author: amyznikov
 *
 */

#include "c_libcamera_image.h"
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/sctp.h>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <core/io/iface.h>
#include <core/settings.h>
#include <core/debug.h>


namespace {
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
using namespace libcamera;
using CameraManager = libcamera::CameraManager;
using Camera = libcamera::Camera;
using CameraPtr = std::shared_ptr<Camera>;
using unique_lock = std::unique_lock<std::mutex>;

static constexpr StreamRole roles[] = {
    StreamRole::Raw,
    StreamRole::StillCapture,
    StreamRole::VideoRecording,
    StreamRole::Viewfinder
};

#pragma pack(push, 1)
struct ImageHeader
{
  uint32_t width;
  uint32_t height;
  uint32_t stride;
  uint32_t datasize;
  uint32_t bpp;
  uint32_t fourcc;
  uint64_t fourcc_modifier;
  uint64_t timestamp;
};
#pragma pack(pop)

enum ThreadState {
  ThreadStateIdle,
  ThreadStateStarting,
  ThreadStateRunning,
  ThreadStateFinish,
};

struct CameraThreadStatus {
  std::unique_ptr<std::thread> thread;
  std::condition_variable condvar;
  std::mutex mtx;
  volatile ThreadState state = ThreadStateIdle;
  volatile int client_socket = -1;
};

static constexpr uint32_t PPID_TEXT = 0;
static constexpr uint32_t PPID_IMGHDR = 1;
static constexpr uint32_t PPID_IMGCHUNK = 2;

static std::unique_ptr<CameraManager> cameraManager;
static std::shared_ptr<Camera> currentCamera;
static Stream * currentStream;
static std::unique_ptr<FrameBufferAllocator> bufferAllocator;
static std::vector<std::unique_ptr<Request>> requests;
static std::deque<Request*> completedRequests;
static std::map<Request*, c_libcamera_image::uptr, std::less<Request *>> images;
static CameraThreadStatus cameraThread;
static std::map<uint32_t, ControlValue> currentUserControls;

/*
 * Rpi can capture raw data in these formats but can not crop ROI for them.
 * Special processing is applied to crop the raw ROI manually and send to client app
 * */
static constexpr PixelFormat specialRawFormats[] = {
    formats::SBGGR10,
    formats::SBGGR12,
    formats::SBGGR14,
    formats::SBGGR16,

    formats::SRGGB10,
    formats::SRGGB12,
    formats::SRGGB14,
    formats::SRGGB16,

    formats::SGRBG10,
    formats::SGRBG12,
    formats::SGRBG14,
    formats::SGRBG16,

    formats::SGBRG10,
    formats::SGBRG12,
    formats::SGBRG14,
    formats::SGBRG16,
};
static Size specialRawSwCropSize;

static int isSecialRawFormat(const PixelFormat & fmt)
{
  for( size_t i = 0; i < sizeof(specialRawFormats) / sizeof(specialRawFormats[0]); ++i ) {
    if( fmt == specialRawFormats[i] ) {
      return (int) (i);
    }
  }
  return -1;
}

static const uint8_t * cropSpecialRawFormatImage(const c_libcamera_image & image,
    const PixelFormat & imageFormat,
    const Size & imageSize,
    uint32_t imageStride,
    const Size & cropSize,
    ImageHeader * outhdr)
{
  static thread_local std::vector<uint8_t> specialCropBuffer;

  const uint32_t cw = cropSize.width & ~0x1U;
  const uint32_t ch = cropSize.height & ~0x1U;
  const uint32_t cx = ((imageSize.width - cw) / 2) & ~0x1U;
  const uint32_t cy = ((imageSize.height - ch) / 2) & ~0x1U;

  if( imageFormat == formats::SBGGR12 || imageFormat == formats::SRGGB12 || imageFormat == formats::SGRBG12 ||
      imageFormat == formats::SGBRG12 ) {

    const uint32_t src_stride = imageStride;
    const uint32_t dst_stride = cw * 3 / 2;

    specialCropBuffer.resize(ch * dst_stride);

    const uint8_t * srcp = image.pane(0).data() + cy * src_stride + cx * sizeof(uint16_t);
    uint8_t * __restrict dstp = specialCropBuffer.data();

    for( uint32_t y = 0; y < ch; ++y ) {

      const uint16_t * srcRow = reinterpret_cast<const uint16_t*>(srcp + y * src_stride);
      uint8_t * __restrict dstRow = dstp + y * dst_stride;

      for (uint32_t x = 0; x < cw; x += 2) {
#if IS_PI5 // In Raspberry Pi 5 data are left-shifted by 4 bits
        uint16_t p0 = srcRow[x + 0] >> 4;
        uint16_t p1 = srcRow[x + 1] >> 4;
#else
        uint16_t p0 = srcRow[x + 0];
        uint16_t p1 = srcRow[x + 1];
#endif
        *dstRow++ = static_cast<uint8_t>(p0 >> 4);
        *dstRow++ = static_cast<uint8_t>(p1 >> 4);
        *dstRow++ = static_cast<uint8_t>((p0 & 0x0F) | ((p1 & 0x0F) << 4));
      }
    }

    outhdr->stride = dst_stride;
    outhdr->fourcc = imageFormat.fourcc();
    outhdr->fourcc_modifier = __fourcc_mod(11, 1);
    outhdr->bpp = 12;
  }
  else if( imageFormat == formats::SBGGR10 || imageFormat == formats::SRGGB10 || imageFormat == formats::SGRBG10 ||
      imageFormat == formats::SGBRG10 ) {

    const uint32_t src_stride = imageStride;
    const uint32_t dst_stride = cw * 5 / 4;

    specialCropBuffer.resize(ch * dst_stride);

    const uint8_t * srcp = image.pane(0).data() + cy * src_stride + cx * sizeof(uint16_t) ;
    uint8_t * __restrict dstp = specialCropBuffer.data();

    for( uint32_t y = 0; y < ch; ++y ) {

      const uint16_t * srcRow = reinterpret_cast<const uint16_t*>(srcp + y * src_stride);
      uint8_t * __restrict dstRow = dstp + y * dst_stride;

      for (uint32_t x = 0; x < cw; x += 4) {
#if IS_PI5  // Raspberry Pi 5 expands 10-bit sensor data by a left shift of 6
        const uint16_t p0 = (srcRow[x + 0] >> 6);// & 0x03FF; // Mask to 10 bits
        const uint16_t p1 = (srcRow[x + 1] >> 6);// & 0x03FF; // Mask to 10 bits
        const uint16_t p2 = (srcRow[x + 2] >> 6);// & 0x03FF; // Mask to 10 bits
        const uint16_t p3 = (srcRow[x + 3] >> 6);// & 0x03FF; // Mask to 10 bits
#else
        const uint16_t p0 = (srcRow[x + 0]);// & 0x03FF; // Mask to 10 bits
        const uint16_t p1 = (srcRow[x + 1]);// & 0x03FF; // Mask to 10 bits
        const uint16_t p2 = (srcRow[x + 2]);// & 0x03FF; // Mask to 10 bits
        const uint16_t p3 = (srcRow[x + 3]);// & 0x03FF; // Mask to 10 bits
#endif

        // Bytes 0-3: 8 MSBs of each pixel
        dstRow[0] = static_cast<uint8_t>(p0 >> 2);
        dstRow[1] = static_cast<uint8_t>(p1 >> 2);
        dstRow[2] = static_cast<uint8_t>(p2 >> 2);
        dstRow[3] = static_cast<uint8_t>(p3 >> 2);
        // Byte 4: Combined 2 LSBs of all 4 pixels
        dstRow[4] = static_cast<uint8_t>((p0 & 0x03) | ((p1 & 0x03) << 2) | ((p2 & 0x03) << 4) | ((p3 & 0x03) << 6));
        dstRow += 5;
      }
    }

    outhdr->stride = dst_stride;
    outhdr->fourcc = imageFormat.fourcc();
    outhdr->fourcc_modifier = __fourcc_mod(11, 1);
    outhdr->bpp = 10;
  }
  else {

    const uint32_t src_stride = imageStride;
    const uint32_t dst_stride = cw * sizeof(uint16_t);

    specialCropBuffer.resize(ch * dst_stride);

    const uint8_t * srcp = image.pane(0).data() + cx * sizeof(uint16_t);
    uint8_t * dstp = specialCropBuffer.data();

    for( uint32_t y = 0; y < ch; ++y ) {
      memcpy(dstp + y * dst_stride, srcp + (y + cy) * src_stride, cw * sizeof(uint16_t));
    }

    outhdr->stride = dst_stride;
    outhdr->fourcc = imageFormat.fourcc();
    outhdr->fourcc_modifier = imageFormat.modifier();
    outhdr->bpp = 16;
  }

  outhdr->width = cw;
  outhdr->height = ch;
  outhdr->datasize = specialCropBuffer.size();

  return specialCropBuffer.data();
}

static std::string ntoa(const struct sockaddr_in & sin)
{
  char ipaddrs[INET_ADDRSTRLEN + 8] = "";

  sprintf(ipaddrs, "%s:%u",
      inet_ntoa(sin.sin_addr),
      ntohs(sin.sin_port));

  return ipaddrs;
}

static int so_sctp_listen(uint32_t addrs, uint16_t port, int maxconn, uint16_t istreams, uint16_t ostreams, struct sockaddr_in * saddrs = nullptr)
{
  struct sockaddr_in sin = {0};
  struct sctp_initmsg initmsg = {0};
  int reuse_address = 1;
  bool fOk = false;
  int so = -1;

  if( (so = socket(AF_INET, SOCK_STREAM, IPPROTO_SCTP)) == -1 ) {
    CF_ERROR("socket(AF_INET, SOCK_SEQPACKET, IPPROTO_SCTP) fails. errnp=%d (%s)", errno, strerror(errno));
    goto end;
  }

  if( setsockopt(so, SOL_SOCKET, SO_REUSEADDR, &reuse_address, sizeof(reuse_address)) < 0 ) {
    CF_WARNING("setsockopt(SO_REUSEADDR, true) fails. errno=%d (%s)", errno, strerror(errno));
  }

  if ( true ) {
    int buffsize = 2 * 1024 * 1024;
    socklen_t buffsizelen = sizeof(buffsize);
    if (setsockopt(so, SOL_SOCKET, SO_RCVBUF, &buffsize, sizeof(buffsizelen)) < 0 ) {
      CF_ERROR("setsockopt(SO_RCVBUF) fails.errno=%d (%s)", errno, strerror(errno));
    }
    buffsize = 0;
    buffsizelen = sizeof(buffsize);
    if (getsockopt(so, SOL_SOCKET, SO_RCVBUF, &buffsize, &buffsizelen) < 0) {
      CF_ERROR("getsockopt(SO_RCVBUF) fails.errno=%d (%s)", errno, strerror(errno));
    }
    CF_DEBUG("SO_RCVBUF: %d", buffsize);
  }

  if ( true ) {
    int buffsize = 212992 / 4; // 8 * 1024 * 1024;
    socklen_t buffsizelen = sizeof(buffsize);
    if (setsockopt(so, SOL_SOCKET, SO_SNDBUF, &buffsize, sizeof(buffsizelen)) < 0 ) {
      CF_ERROR("setsockopt(SO_SNDBUF) fails.errno=%d (%s)", errno, strerror(errno));
    }
  }
  if ( true ) {
    int buffsize = 0;
    socklen_t buffsizelen = sizeof(buffsize);
    if (getsockopt(so, SOL_SOCKET, SO_SNDBUF, &buffsize, &buffsizelen) < 0) {
      CF_ERROR("getsockopt(SO_SNDBUF) fails.errno=%d (%s)", errno, strerror(errno));
    }
    CF_DEBUG("SO_SNDBUF: %d", buffsize);
  }

  sin.sin_family = AF_INET;
  sin.sin_addr.s_addr = htonl(addrs);
  sin.sin_port = htons(port);
  if ( bind(so, (struct sockaddr*)&sin, sizeof(sin)) == -1 ) {
    CF_ERROR("bind() fails. errno=%d (%s)", errno, strerror(errno));
    goto end;
  }

  initmsg.sinit_num_ostreams = ostreams;
  initmsg.sinit_max_instreams = istreams;
  initmsg.sinit_max_attempts = 2;
  if ( setsockopt(so, IPPROTO_SCTP, SCTP_INITMSG, &initmsg, sizeof(initmsg)) < 0 ) {
    CF_ERROR("setsockopt(SCTP_INITMSG) fails. errno=%d (%s)", errno, strerror(errno));
    goto end;
  }

  if ( listen(so, maxconn) == -1 ) {
    CF_ERROR("listen(maxconn=%d) fails. errno=%d (%s)", maxconn, errno, strerror(errno));
    goto end;
  }

  if ( saddrs ) {
    *saddrs = sin;
  }

  fOk = true;

end:
  if ( !fOk && so != -1 ) {
    close(so);
    so = -1;
  }

  return so;
}

static int so_sctp_sendmsg(int so, const void * msg, size_t len, uint16_t stream_no, uint32_t ppid = 0)
{
  return sctp_sendmsg(so, msg, len, nullptr, 0, htonl(ppid), 0, stream_no, 0, 0);
}


static inline void set_thread_status(CameraThreadStatus & st, ThreadState newstate)
{
  st.mtx.lock();
  st.state = newstate;
  st.condvar.notify_all();
  st.mtx.unlock();
}

static bool detect_cameras()
{
  if( !cameraManager ) {

    cameraManager.reset(new CameraManager());

    const int status = cameraManager->start();
    if( status < 0 ) {
      CF_ERROR("camera_manager->start() fails: %s", strerror(-status));
      cameraManager.reset();
      return false;
    }
  }

  const std::vector<CameraPtr> cameras = cameraManager->cameras();
  if ( cameras.empty() ) {
    CF_ERROR("ERROR: No cameras detected");
    return false;
  }

  for ( size_t i = 0, n = cameras.size(); i < n; ++i ) {

    const CameraPtr & camera = cameras[i];
    const std::string id = camera->id();
    const ControlList & properties = camera->properties();

    const std::string model =
        properties.get(properties::MODEL).toString();

    const std::string areas =
        properties.contains(properties::PIXEL_ARRAY_ACTIVE_AREAS) ?
            properties.get(properties::PIXEL_ARRAY_ACTIVE_AREAS).toString() :
            "";

    fprintf(stderr, "camera[%zu] id: '%s'  model: '%s' areas: %s\n", i,
        id.c_str(),  model.c_str(),  areas.c_str());
  }

  return true;
}

static void reset_current_camera()
{
  if ( currentCamera ) {
    currentCamera->stop();
    currentCamera->requestCompleted.disconnect();
  }

  images.clear();
  completedRequests.clear();
  requests.clear();

  if ( bufferAllocator ) {
    if ( currentStream ) {
      bufferAllocator->free(currentStream);
      currentStream = nullptr;
    }
    bufferAllocator.reset();
  }
  if ( currentCamera ) {
    currentCamera->release();
    currentCamera.reset();
  }
  if ( cameraThread.thread ) {
    cameraThread.thread.reset();
  }
}


static void camera_request_commplete(Request * request)
{
  if ( request->status() == Request::RequestComplete ) {
    unique_lock lock(cameraThread.mtx);
    completedRequests.emplace_back(request);
    cameraThread.condvar.notify_all();
  }
}


static void camera_thread()
{
  /* Start the camera and queue initial frame requests */

  CF_DEBUG("Enter");
  currentCamera->requestCompleted.connect(&camera_request_commplete);

  int status = currentCamera->start();
  if ( status < 0 ) {
    CF_ERROR("currentCamera->start() fails: %s", strerror(-status));
  }
  else {
    for ( auto &request : requests ) {

      ControlList & controls = request->controls();
      for( const auto & c: currentUserControls ) {
        controls.set(c.first, c.second);
      }

      if ( (status = currentCamera->queueRequest(request.get())) < 0 ) {
        CF_ERROR("Can't queue request : %s", strerror(-status));
        break;
      }
    }
  }

  /* Normal thread loop */
  if ( status >= 0 ) {

    CF_DEBUG("Camera Started");

    ImageHeader hdr;
    Request * currentRequest = nullptr;
    c_libcamera_image * currentImage = nullptr;
    const uint8_t * data = nullptr;
    uint32_t bytes_start = 0;
    int ChunkSize = 64 * 1024;

    set_thread_status(cameraThread, ThreadStateRunning);
    unique_lock lock(cameraThread.mtx);

    if ( true ) {
      socklen_t buffsizelen = sizeof(ChunkSize);
      if (getsockopt(cameraThread.client_socket, SOL_SOCKET, SO_SNDBUF, &ChunkSize, &buffsizelen) < 0) {
        CF_ERROR("getsockopt(SO_SNDBUF) fails.errno=%d (%s)", errno, strerror(errno));
      }
    }

    if (ChunkSize < 1024 ) {
      ChunkSize = 64 * 1024;
    }

    CF_DEBUG("ChunkSize: %d", ChunkSize);

    while ( cameraThread.state == ThreadStateRunning  ) {

      if( !currentRequest && completedRequests.empty() ) {
        cameraThread.condvar.wait(lock);
      }
      if( cameraThread.state != ThreadStateRunning ) {
        break;
      }

      if ( !currentRequest && !completedRequests.empty() ) {
        currentRequest = completedRequests.front();
        currentImage = images[currentRequest].get();
        bytes_start = 0;
        completedRequests.pop_front();

        const Request::BufferMap & bufs = currentRequest->buffers();
        const Stream * stream = bufs.begin()->first;
        const StreamConfiguration & config = stream->configuration();
        const FrameBuffer * frame = bufs.begin()->second;
        const FrameMetadata & metadata = frame->metadata();
        const uint64_t timestamp = metadata.timestamp;
        const PixelFormat & pixelFormat = config.pixelFormat;
        const Size & size = config.size;
        const uint32_t stride = config.stride;
        const uint32_t datasize = config.frameSize;

        const int specialRawFormatIndex =
            specialRawSwCropSize.width > 0 && specialRawSwCropSize.height > 0 ?
                isSecialRawFormat(pixelFormat) : -1;

        if ( specialRawFormatIndex < 0 ) {
          hdr.width = size.width;
          hdr.height = size.height;
          hdr.stride = stride;
          hdr.datasize = datasize;
          hdr.bpp = 0;
          hdr.fourcc = pixelFormat.fourcc();
          hdr.fourcc_modifier = pixelFormat.modifier();
          data = currentImage->pane(0).data();
        }
        else {
          data =
              cropSpecialRawFormatImage(*currentImage,
                  pixelFormat,
                  size,
                  stride,
                  specialRawSwCropSize,
                  &hdr);
        }

        hdr.timestamp = timestamp;
      }

      if ( currentRequest ) {

        const int so = cameraThread.client_socket;
        bool fail = false;

        lock.unlock();

        if( bytes_start == 0 ) {
          if( so_sctp_sendmsg(so, &hdr, sizeof(hdr), 1, PPID_IMGHDR) < 0 ) {
            CF_ERROR("so_sctp_sendmsg(so=%d, hdr) fails. errno=%d %s", so, errno, strerror(errno));
            fail = true;
          }
        }

        if( !fail ) {
          const uint32_t bytes_end = std::min(hdr.datasize, bytes_start + ChunkSize);
          if( so_sctp_sendmsg(so, data + bytes_start, bytes_end - bytes_start, 1, PPID_IMGCHUNK) >= 0 ) {
            bytes_start = bytes_end;
          }
          else {
            CF_ERROR("so_sctp_sendmsg(so=%d, data) fails. errno=%d %s", so, errno, strerror(errno));
            fail = true;
          }
        }

        lock.lock();

        if ( bytes_start >= hdr.datasize ) {

          currentRequest->reuse(Request::ReuseBuffers);

          ControlList & controls = currentRequest->controls();
          for( const auto & c : currentUserControls ) {
            controls.set(c.first, c.second);
          }

          currentCamera->queueRequest(currentRequest);

          currentRequest = nullptr;
          currentImage = nullptr;
          data = nullptr;
          bytes_start = 0;
        }

        if ( fail ) {
          CF_DEBUG("Break by so_sctp_sendmsg() fail");
          struct linger lo = { .l_onoff = 1, .l_linger = 0 };
          setsockopt(cameraThread.client_socket, SOL_SOCKET, SO_LINGER, &lo, sizeof(lo));
          shutdown(cameraThread.client_socket, SHUT_RDWR);
          close(cameraThread.client_socket);
          cameraThread.client_socket = -1;
          break;
        }
      }
    }
  }

  /* Stop camera and finish capture thread */
  cameraThread.mtx.lock();
  reset_current_camera();
  cameraThread.state = ThreadStateIdle;
  cameraThread.condvar.notify_all();
  cameraThread.mtx.unlock();
  CF_DEBUG("Leave");
}


static void set_status(c_config_setting output, const std::string & status, const std::string & errmsg)
{
  save_settings(output, "status", status);
  save_settings(output, "errmsg", errmsg);
}

static void get_cameras(c_config_setting output)
{
  const std::vector<CameraPtr> cameras = cameraManager->cameras();
  if ( cameras.empty() ) {
    set_status(output, "OK", std::string("No cameras detected"));
    return;
  }

  save_settings(output, "status", std::string("OK"));

  c_config_setting cameras_item = output.add_list("cameras");

  for ( size_t i = 0, n = cameras.size(); i < n; ++i ) {

    const CameraPtr & camera = cameras[i];
    const std::string id = camera->id();
    const ControlList & props = camera->properties();
    const std::string model = props.get(properties::MODEL).toString();
    const std::string areas = props.contains(properties::PIXEL_ARRAY_ACTIVE_AREAS) ? props.get(properties::PIXEL_ARRAY_ACTIVE_AREAS).toString() : "";

    c_config_setting camera_item = cameras_item.add_group();
    save_settings(camera_item, "id", id);
    save_settings(camera_item, "model", model);
    save_settings(camera_item, "areas", areas);

    c_config_setting controls_item = camera_item.add_list("controls");
    for (auto const& [id, info] : camera->controls() ) {
      if ( id->id() != controls::FrameDurationLimits.id() ) { // Skip buggy FrameDurationLimits under RPi
        c_config_setting control_item = controls_item.add_group();
        save_settings(control_item, "id", id->name());
        save_settings(control_item, "type", toString(id->type()));
        save_settings(control_item, "min", info.min().type() == ControlTypeNone ? "" : info.min().toString());
        save_settings(control_item, "max", info.max().type() == ControlTypeNone ? "" : info.max().toString());
        save_settings(control_item, "def", info.def().type() == ControlTypeNone ? "" : info.def().toString());
        std::vector<std::string> values;
        for ( const auto & v : info.values() ) {
          values.emplace_back(v.toString());
        }
        save_settings(control_item, "values", values);
      }
    }

    //    c_config_setting props_item = camera_item.add_list("props");
    //    for ( const auto & prop : props ) {
    //      uint32_t prop_id = prop.first;
    //      const ControlValue & prop_value = prop.second;
    //      c_config_setting prop_item = props_item.add_group();
    //      save_settings(prop_item, "id", prop_id);
    //      save_settings(prop_item, "value", prop_value.toString());
    //    }


    c_config_setting roles_item = camera_item.add_list("roles");
    for ( uint32_t j = 0, m = sizeof(roles)/sizeof(roles[0]); j < m; ++j ) {

      const StreamRole role = roles[j];

      std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({role});
      if ( !config ) {
        continue;
      }

      StreamConfiguration &roleConfig = config->at(0);
      const StreamFormats & formats = roleConfig.formats();
      if( formats.pixelformats().empty() ) {
        continue;
      }

      c_config_setting role_item = roles_item.add_group();
      save_settings(role_item, "role", toString(role));

      c_config_setting pixfmts_item = role_item.add_list("pixfmts");
      for ( const auto & pixfmt : formats.pixelformats() ) {

        c_config_setting pixfmt_item = pixfmts_item.add_group();
        save_settings(pixfmt_item, "format", pixfmt.toString());

        c_config_setting sizes_item = pixfmt_item.add_array("sizes");
        for ( const auto &size : formats.sizes(pixfmt) ) {
          sizes_item.add_element(size.toString());
        }
      }

      if ( role == StreamRole::Raw ) {

        const ControlList & props = camera->properties();
        if( props.contains(properties::PixelArraySize.id()) ) {

          const Size sensorSize =
              props.get(libcamera::properties::PixelArraySize).value();

          CF_DEBUG("Detected Sensor Resolution: %dx%d",
              sensorSize.width, sensorSize.height);

          /* Check if we can manually crop Full HQ resolution Bayer image */

          for ( size_t i = 0, ni = sizeof(specialRawFormats)/sizeof(specialRawFormats[0]); i < ni; ++i ) {

            roleConfig.pixelFormat = specialRawFormats[i];
            roleConfig.size = sensorSize;
            CameraConfiguration::Status status = config->validate();
            if (status == CameraConfiguration::Invalid || roleConfig.pixelFormat != specialRawFormats[i]) { // This should not happen on RPi HQ Camera
              // CF_ERROR("Unpacked format '%s' is not supported", specialRawFormats[i].toString().c_str());
            }
            else  {
              c_config_setting pixfmt_item = pixfmts_item.add_group();
              save_settings(pixfmt_item, "format", roleConfig.pixelFormat.toString());

              /* Generate some feasible crop sizes */
              c_config_setting sizes_item = pixfmt_item.add_array("sizes");
              for ( int j = 1; j < 40; ++j ) {
                const int w = 160 * j;
                const int h = 120 * j;
                if ( w < sensorSize.width && h < sensorSize.height ) {
                  sizes_item.add_element(Size(w,h).toString());
                }
              }
              sizes_item.add_element(roleConfig.size.toString());
            }
          }
        }
      }
    }
  }
}


static void stop_camera(c_config_setting input, c_config_setting output)
{
  std::unique_lock<std::mutex> lock(cameraThread.mtx);
  if( cameraThread.state != ThreadStateIdle ) {

    cameraThread.state = ThreadStateFinish;
    cameraThread.condvar.notify_all();

    CF_DEBUG("wait for cameraThread to finish...");
    cameraThread.condvar.wait(lock, [] { return cameraThread.state == ThreadStateIdle; });
    CF_DEBUG("cameraThread finished");
  }

  set_status(output, "OK", std::string("Camera thread stopped"));
}

static void set_user_controls(c_config_setting controls_item, const ControlInfoMap & availableControls)
{
  if ( controls_item.isList() ) {
    for ( int i = 0, n = controls_item.length(); i < n; ++i ) {
      c_config_setting control_item = controls_item[i];

      std::string cid, cvalue;
      ControlValue cval;

      if ( !load_settings(control_item, "id", &cid) || cid.empty() ) {
        CF_DEBUG("load_settings(id) fails: id='%s'", cid.c_str());
        continue;
      }

      if ( !load_settings(control_item, "value", &cvalue) || cvalue.empty() ) {
        CF_DEBUG("load_settings(value) fails: cvalue='%s'", cvalue.c_str());
        continue;
      }

      const ControlId* ctrlid = findControlByName(availableControls, cid);
      if ( !ctrlid ) {
        CF_DEBUG("findControlByName(cname='%s') fails'", cid.c_str());
        continue;
      }

      if( ctrlid->id() == controls::FrameDurationLimits.id() ) { // Seems buggy under Rpi
        continue;
      }

      if ( !controlValueFromString(*ctrlid, cvalue, &cval) ) {
        CF_DEBUG("controlValueFromString(cname='%s', cvalue=%s) fails'", cid.c_str(), cvalue.c_str());
        continue;
      }

      currentUserControls[ctrlid->id()] = cval;
      CF_DEBUG("SET %s = %s", cid.c_str(), cval.toString().c_str());
    }
  }

}

static void set_controls(c_config_setting input, c_config_setting output)
{
  std::unique_lock<std::mutex> lock(cameraThread.mtx);
  if ( !currentCamera ) {
    set_status(output, "error", "No current camera is active");
  }
  else {
    set_user_controls(input["controls"], currentCamera->controls());
    set_status(output, "OK", std::string("controls applied"));
  }
}

static void start_camera(c_config_setting input, c_config_setting output)
{
  if( true ) {
    std::unique_lock<std::mutex> lock(cameraThread.mtx);
    if( cameraThread.state != ThreadStateIdle ) {
      set_status(output, "error", "current camera thread is active");
      return;
    }
  }

  std::string cameraName;
  int bufferCount = 0;

  StreamRole streamRole = StreamRole::VideoRecording;
  std::string pixelFormat;
  std::string captureSize;
  int specialRawFormatIndex = -1;

  load_settings(input, "name", &cameraName);
  load_settings(input, "streamRole", &streamRole);
  load_settings(input, "format", &pixelFormat);
  load_settings(input, "buffers", &bufferCount);
  load_settings(input, "size", &captureSize);

  if ( cameraName.empty() ) {
    set_status(output, "error", "No camera name specified");
    reset_current_camera();
    return;
  }

  if( !(currentCamera = cameraManager->get(cameraName)) ) {
    set_status(output, "error", ssprintf("cameraManager->get(cameraName='%s') fails", cameraName.c_str()));
    reset_current_camera();
    return;
  }

  int status = currentCamera->acquire();
  if( status < 0 ) {
    set_status(output, "error", ssprintf("camera->acquire() fails : %s", strerror(-status)));
    reset_current_camera();
    return;
  }

  std::unique_ptr<CameraConfiguration> cameraConfig = currentCamera->generateConfiguration({streamRole});
  if ( !cameraConfig || cameraConfig->size() != 1 ) {
    set_status(output, "error", ssprintf("Failed to get default stream configuration"));
    reset_current_camera();
    return;
  }

  StreamConfiguration & streamConfig = cameraConfig->at(0);
  if( !pixelFormat.empty() ) {
    streamConfig.pixelFormat = PixelFormat::fromString(pixelFormat);
    specialRawFormatIndex = isSecialRawFormat(streamConfig.pixelFormat);
  }

  if( bufferCount > 0 ) {
    streamConfig.bufferCount = bufferCount;
  }

  if( specialRawFormatIndex >= 0 ) {
    if( specialRawFormatIndex >= 0 ) {
      CF_DEBUG("Original streamConfig.size=%dx%d", streamConfig.size.width, streamConfig.size.height);
    }
  }

  if ( !captureSize.empty() ) {
    uint32_t w = 0, h = 0;
    if ( sscanf(captureSize.c_str(), "%ux%u", &w, &h) != 2 ) {
      set_status(output, "error", ssprintf("Syntax error in captureSize parameter"));
      reset_current_camera();
      return;
    }
    streamConfig.size.width = w;
    streamConfig.size.height = h;
  }

  if( specialRawFormatIndex >= 0 ) {
    specialRawSwCropSize = streamConfig.size;
    streamConfig.size = currentCamera->properties().get(libcamera::properties::PixelArraySize).value();
    CF_DEBUG("Requested SW Crop %dx%d -> %dx%d", streamConfig.size.width, streamConfig.size.height, specialRawSwCropSize.width, specialRawSwCropSize.height);
  }

  const CameraConfiguration::Status validateStatus = cameraConfig->validate();
  switch ( validateStatus ) {
  case CameraConfiguration::Valid :
    CF_DEBUG("CameraConfiguration::Valid");
    break;
  case CameraConfiguration::Adjusted :
    CF_DEBUG("CameraConfiguration::Adjusted. bufferCount=%u", streamConfig.bufferCount);
    break;

  case CameraConfiguration::Invalid :
    set_status(output, "error", ssprintf("CameraConfiguration::Invalid"));
    reset_current_camera();
    return;

  default :
    set_status(output, "error", ssprintf("cameraConfig->validate() returns unknown value %d",(int)validateStatus));
    reset_current_camera();
    return;
  }

  if ( (status = currentCamera->configure(cameraConfig.get())) < 0 ) {
    set_status(output, "error", ssprintf("camera->configure() fails: status=%d (%s)", status, strerror(-status)));
    reset_current_camera();
    return;
  }


  if ( true ) {
    currentUserControls.clear();
    set_user_controls(input["controls"], currentCamera->controls());
  }

  /*
   * Allocate buffers and create frame requests
   * */
  bufferAllocator = std::make_unique<FrameBufferAllocator>(currentCamera);
  if ( (status = bufferAllocator->allocate(streamConfig.stream())) < 0 ) {
    set_status(output, "error", ssprintf("Can't allocate frame buffers: %s: status=%d (%s)", status, strerror(-status)));
    reset_current_camera();
    return;
  }

  const size_t allocatedBuffers = bufferAllocator->buffers(streamConfig.stream()).size();
  if ( allocatedBuffers < 1 ) {
    set_status(output, "error", ssprintf("Can't allocate frame buffers: allocatedBuffers=%zu", allocatedBuffers));
    reset_current_camera();
    return;
  }

  currentStream = streamConfig.stream();
  const std::vector<std::unique_ptr<FrameBuffer>> & frameBuffers = bufferAllocator->buffers(currentStream);
  const size_t maxBuffers = std::min((size_t )std::max(1, bufferCount), allocatedBuffers);
  CF_DEBUG("allocatedBuffers=%zu maxBuffers=%zu", allocatedBuffers, maxBuffers);

  for ( size_t i = 0; i < maxBuffers; ++i ) {

    std::unique_ptr<Request> request = currentCamera->createRequest();
    if ( !request ) {
      set_status(output, "error", "currentCamera->createRequest() fails");
      reset_current_camera();
      return;
    }

    FrameBuffer * framebuffer = frameBuffers[i].get();
    if ( (status = request->addBuffer(streamConfig.stream(), framebuffer)) < 0 ) {
      set_status(output, "error", ssprintf("request->addBuffer() fails: %s", strerror(-status)));
      reset_current_camera();
      return;
    }

    c_libcamera_image::uptr image = c_libcamera_image::fromFrameBuffer(framebuffer, c_libcamera_image::MapMode::ReadOnly);
    if( !image ) {
      set_status(output, "error", ssprintf("c_libcamera_image::fromFrameBuffer() fails: %s", strerror(errno)));
      reset_current_camera();
      return;
    }

    images.emplace(request.get(), std::move(image));
    requests.emplace_back(std::move(request));
  }


  /*
   * Start the camera and queue initial frame requests
   * */
  if ( true ) {
    set_thread_status(cameraThread, ThreadStateStarting);
    (cameraThread.thread = std::make_unique<std::thread>(camera_thread))->detach();

    std::unique_lock<std::mutex> lock(cameraThread.mtx);
    cameraThread.condvar.wait(lock, [] { return cameraThread.state != ThreadStateStarting; });
    if( cameraThread.state != ThreadStateRunning ) {
      set_status(output, "error", ssprintf("Fail to start thread: cameraThread.state=%d", cameraThread.state));
      reset_current_camera();
      return;
    }
  }

  set_status(output, "OK", ssprintf("Configuration '%s': (%dx%d %s %s) buffs=%u",
      streamConfig.toString().c_str(),
      streamConfig.size.width, streamConfig.size.height,
      streamConfig.pixelFormat.toString().c_str(),
      streamConfig.colorSpace.has_value() ? streamConfig.colorSpace.value().toString().c_str() : "",
      streamConfig.bufferCount
      ));
}

static void sctp_client_thread(int so, const struct sockaddr_in & client_addrs)
{
  std::vector<uint8_t> msgbuf(65536);
  std::string command;
  c_config cfgin, cfgout;
  int cb;

  cameraThread.client_socket = so;

  while( 42 ) {
    struct sctp_sndrcvinfo sri = {0};
    int flags = 0;

    if( (cb = sctp_recvmsg(so, msgbuf.data(), msgbuf.size() - 1, nullptr, nullptr, &sri, &flags)) <= 0 ) {
      CF_ERROR("sctp_recvmsg(so=%d) fails. cb=%d flags=0x%0X errno=%d (%s)", so, cb, flags, errno, strerror(errno));
      break;
    }
    if ( flags & MSG_NOTIFICATION ) {
      continue;
    }


    msgbuf[cb] = 0;
    CF_DEBUG("\nmsg='%s'", (const char*)msgbuf.data());

    cfgin.clear();
    cfgout.clear();

    if ( !cfgin.read_string((const char*)msgbuf.data()) ) {
      CF_ERROR("cfg.read_string() fails");
      continue;
    }

    c_config_setting input = cfgin.root();
    c_config_setting output = cfgout.root();

    if ( !load_settings(input, "command", &command) || command.empty() ) {
      CF_ERROR("load_settings('command') fails");
      continue;
    }

    if ( command == "get-cameras" ) {
      get_cameras(output);
    }
    else if ( command == "start-camera" ) {
      start_camera(input, output);
    }
    else if ( command == "stop-camera" ) {
      stop_camera(input, output);
    }
    else if ( command == "set-controls" ) {
      set_controls(input, output);
    }
    else {
      save_settings(output, "status", std::string("error"));
      save_settings(output, "errormsg", std::string("unknown or invalid command"));
    }

    const std::string responce = cfgout.write_string();
    if( (cb = so_sctp_sendmsg(so, responce.c_str(), responce.size() + 1, PPID_TEXT)) <= 0 ) { //
      CF_ERROR("so_sctp_sendmsg(so=%d,responce) fails. errno=%d %s", so, errno, strerror(errno));
      break;
    }

  }

  CF_DEBUG("close client connection to %s", ntoa(client_addrs).c_str());
  shutdown(so, SHUT_RDWR);
  close(so);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
} // namespace


int main(int argc, char *argv[])
{
  const std::string bind_address = "0.0.0.0:6008";

  uint32_t listen_address = 0;
  uint16_t listen_port = 6008;
  struct sockaddr_in listen_sockaddrs;
  int so = -1;

  cf_set_logfile(stderr);
  cf_set_loglevel(CF_LOG_DEBUG);


  if ( !detect_cameras() ) {
    CF_ERROR("detect_cameras(() fails");
    return 1;
  }

  if ( !cf_get_iface_address(bind_address.c_str(), &listen_address, &listen_port) ) {
    CF_ERROR("cf_get_iface_address('%s') fails: %s", bind_address.c_str(), strerror(errno));
    return 1;
  }

  so = so_sctp_listen(listen_address, listen_port, 1, 2, 2, &listen_sockaddrs);
  if ( so < 0 ) {
    CF_ERROR("so_listen('%s') fails. errno=%d %s", bind_address.c_str(), errno, strerror(errno));
    return 1;
  }

//  if ( true ) {
//    struct sctp_sack_info sack_info = {0};
//    socklen_t len = sizeof(sack_info);
//    if ( getsockopt(so, IPPROTO_SCTP, SCTP_DELAYED_SACK, &sack_info, &len) < 0 ) {
//      CF_DEBUG("getsockopt(SCTP_DELAYED_SACK) fails. errno=%d (%s)", errno, strerror(errno) );
//    }
//    CF_DEBUG("1: SCTP_DELAYED_SACK: Delay: %u, Freq: %u", sack_info.sack_delay, sack_info.sack_freq);
//  }

//  if ( true ) {
//    // Set to disable (freq=1, delay=0)
//    struct sctp_sack_info sack_info = {0};
//    socklen_t len = sizeof(sack_info);
//    sack_info.sack_delay = 0;
//    sack_info.sack_freq = 1; // Sets frequency to 1 for immediate ACKs
//    sack_info.sack_assoc_id = SCTP_ALL_ASSOC; // Or specific assoc ID
//    if ( setsockopt(so, IPPROTO_SCTP, SCTP_DELAYED_SACK, &sack_info, len) < 0 ) {
//      CF_DEBUG("setsockopt(SCTP_DELAYED_SACK) fails. errno=%d (%s)", errno, strerror(errno) );
//    }
//  }

//  if ( true ) {
//    struct sctp_sack_info sack_info = {0};
//    socklen_t len = sizeof(sack_info);
//    if ( getsockopt(so, IPPROTO_SCTP, SCTP_DELAYED_SACK, &sack_info, &len) < 0 ) {
//      CF_DEBUG("getsockopt(SCTP_DELAYED_SACK) fails. errno=%d (%s)", errno, strerror(errno) );
//    }
//    CF_DEBUG("2: SCTP_DELAYED_SACK: Delay: %u, Freq: %u", sack_info.sack_delay, sack_info.sack_freq);
//  }

  while (42) {
    struct sockaddr_in caddrs = {0};
    socklen_t len = sizeof(caddrs);

    int so1 = accept(so, (struct sockaddr*)&caddrs, &len);
    if ( so1 < 0 ) {
      CF_ERROR("accept(so=%d) fails. errno=%d (%s)", so, errno, strerror(errno));
      continue;
    }

    CF_DEBUG("Accepted connection from '%s'",
        ntoa(caddrs).c_str());

    if ( true ) {
      struct sctp_sack_info sack_info = {0};
      socklen_t len = sizeof(sack_info);
      if ( getsockopt(so1, IPPROTO_SCTP, SCTP_DELAYED_SACK, &sack_info, &len) < 0 ) {
        CF_DEBUG("getsockopt(so1=%d SCTP_DELAYED_SACK) fails. errno=%d (%s)", so1, errno, strerror(errno) );
      }
      CF_DEBUG("SCTP_DELAYED_SACK: Delay: %u, Freq: %u", sack_info.sack_delay, sack_info.sack_freq);
    }

    std::thread(([so1, caddrs]() {
        sctp_client_thread(so1, caddrs);
      })).detach();
  }

  shutdown(so, SHUT_RDWR);
  close(so);
  CF_DEBUG("leave");
  return 0;
}



