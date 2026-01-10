/*
 * c_libcamera_image.h
 *
 *  Created on: Jan 9, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_libcamera_image_h__
#define __c_libcamera_image_h__

#include <unistd.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <libcamera/version.h>
#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/property_ids.h>
#include <core/ssprintf.h>

#ifndef LIBCAMERA_VERSION_INT
# define LIBCAMERA_VERSION_INT(a,b,c) (((a)<<16)|((b)<<8)|(c))
#endif
#ifndef LIBCAMERA_VERSION_CURRRENT
# define LIBCAMERA_VERSION_CURRRENT LIBCAMERA_VERSION_INT(LIBCAMERA_VERSION_MAJOR, LIBCAMERA_VERSION_MINOR, LIBCAMERA_VERSION_PATCH)
#endif

class c_libcamera_image
{
  LIBCAMERA_DISABLE_COPY(c_libcamera_image)
public:
  typedef c_libcamera_image this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  enum MapMode {
    ReadOnly = 1 << 0,
    WriteOnly = 1 << 1,
    ReadWrite = ReadOnly | WriteOnly,
  };

  ~c_libcamera_image()
  {
    for ( libcamera::Span<uint8_t> &map : _maps ) {
      munmap(map.data(), map.size());
    }
  }

  size_t num_planes() const
  {
    return _planes.size();
  }

  libcamera::Span<uint8_t> & pane(uint index)
  {
    return _planes[index];
  }

  libcamera::Span<const uint8_t> pane(uint index) const
  {
    return _planes[index];
  }

  static uptr fromFrameBuffer(const libcamera::FrameBuffer *buffer, MapMode mode)
  {
    struct MappedBufferInfo {
      uint8_t * address = nullptr;
      size_t mapLength = 0;
      size_t dmabufLength = 0;
    };

    std::map<int, MappedBufferInfo> mappedBuffers;

    int mmapFlags = 0;
    if ( mode & MapMode::ReadOnly ) {
      mmapFlags |= PROT_READ;
    }
    if ( mode & MapMode::WriteOnly ) {
      mmapFlags |= PROT_WRITE;
    }

    if ( buffer->planes().empty() ) {
      return nullptr;
    }

    for ( const auto &plane : buffer->planes() ) {

      const int fd = plane.fd.get();

      if ( mappedBuffers.find(fd) == mappedBuffers.end() ) {
        const size_t length = lseek(fd, 0, SEEK_END);
        mappedBuffers[fd] = MappedBufferInfo { nullptr, 0, length };
      }

      const size_t length = mappedBuffers[fd].dmabufLength;
      if ( plane.offset > length || plane.offset + plane.length > length ) {
        return nullptr;
      }

      size_t & mapLength = mappedBuffers[fd].mapLength;
      mapLength = std::max(mapLength, static_cast<size_t>(plane.offset + plane.length));
    }

    uptr image(new this_class());

    for( const auto & plane : buffer->planes() ) {

      const int fd = plane.fd.get();
      auto & info = mappedBuffers[fd];

      if( !info.address ) {
        void * address = mmap(nullptr, info.mapLength, mmapFlags, MAP_SHARED, fd, 0);
        if( address == MAP_FAILED ) {
          return nullptr;
        }

        info.address = static_cast<uint8_t*>(address);
        image->_maps.emplace_back(info.address, info.mapLength);
      }

      image->_planes.emplace_back(info.address + plane.offset, plane.length);
    }

    return image;
  }

protected:
  c_libcamera_image() = default;
  std::vector<libcamera::Span<uint8_t>> _planes;
  std::vector<libcamera::Span<uint8_t>> _maps;
};


inline constexpr uint64_t __fourcc_mod(uint32_t vendor, uint32_t mod)
{
  return (static_cast<uint64_t>(vendor) << 56) | (static_cast<uint64_t>(mod) << 0);
}

inline const libcamera::ControlId* findControlByName(const libcamera::ControlInfoMap & availableControls, const std::string & name)
{
  for( auto const& [id, info] : availableControls ) {
    if( id->name() == name ) {
      return id;
    }
  }
  return nullptr;
}

inline const libcamera::ControlId* findControlByName(const libcamera::ControlIdMap & availableControls, const std::string & name)
{
  for (auto const& [id, ctrlIdPtr] : availableControls) {
    if( ctrlIdPtr->name() == name ) {
      return ctrlIdPtr;
    }
  }
  return nullptr;
}

template<class T>
inline bool fromString(const std::string & s, libcamera::ControlValue * value)
{
  T v;
  if( ::fromString(s, &v) ) {
    value->set<T>(v);
    return true;
  }
  return false;
}


inline bool controlValueFromString(const libcamera::ControlId & id, const std::string & s, libcamera::ControlValue * value)
{
  switch (id.type()) {
    case libcamera::ControlTypeBool:
      return fromString<bool>(s, value);
    case libcamera::ControlTypeByte:
      return fromString<uint8_t>(s, value);
    case libcamera::ControlTypeUnsigned16:
      return fromString<uint16_t>(s, value);
    case libcamera::ControlTypeUnsigned32:
      return fromString<uint32_t>(s, value);
    case libcamera::ControlTypeInteger32:
      return fromString<int32_t>(s, value);
    case libcamera::ControlTypeInteger64:
      return fromString<int64_t>(s, value);
    case libcamera::ControlTypeFloat:
      return fromString<float>(s, value);
    case libcamera::ControlTypeString:
      value->set<std::string_view>(s);
      return true;
    //  case libcamera::ControlTypeRectangle: return fromString<>(s, value);
    //  case libcamera::ControlTypeSize: return fromString<>(s, value);
    //  case libcamera::ControlTypePoint: return fromString<>(s, value);
    default:
      // CF_ERROR("Unhandled coontrol type %d", (int )id.type());
      break;
  }
  return false;
}

#endif /* __c_libcamera_image_h__ */
