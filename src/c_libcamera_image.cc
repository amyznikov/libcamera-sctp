/*
 * c_libcamera_image.cc
 *
 *  Created on: Jan 9, 2026
 *      Author: amyznikov
 */

#include "c_libcamera_image.h"

template<>
const c_enum_member * members_of<libcamera::StreamRole>()
{
  static const c_enum_member members[] = {
      { (int)libcamera::StreamRole::Raw, "Raw", "" },
      { (int)libcamera::StreamRole::StillCapture, "StillCapture", "" },
      { (int)libcamera::StreamRole::VideoRecording, "VideoRecording", "" },
      { (int)libcamera::StreamRole::Viewfinder, "Viewfinder", "" },
      { (int)libcamera::StreamRole::Raw}
  };

  return members;
}

template<>
const c_enum_member* members_of<libcamera::ControlType>()
{
  static const c_enum_member members[] = {
      { libcamera::ControlTypeNone, "None", "" },
      { libcamera::ControlTypeBool, "Bool", "" },
      { libcamera::ControlTypeByte, "Byte", "" },
      { libcamera::ControlTypeUnsigned16, "Unsigned16", "" },
      { libcamera::ControlTypeUnsigned32, "Unsigned32", "" },
      { libcamera::ControlTypeInteger32, "Integer32", "" },
      { libcamera::ControlTypeInteger64, "Integer64", "" },
      { libcamera::ControlTypeFloat, "Float", "" },
      { libcamera::ControlTypeString, "String", "" },
      { libcamera::ControlTypeRectangle, "Rectangle", "" },
      { libcamera::ControlTypeSize, "Size", "" },
      { libcamera::ControlTypePoint, "Point", "" },
      { libcamera::ControlTypeNone },
  };

  return members;
}
