#include "frame.hpp"

Frame::Ptr Frame::CreateFrame() {
  static unsigned long factory_id = 0;
  Frame::Ptr new_frame(new Frame);
  new_frame->id_ = factory_id++;
  return new_frame;
}

void Frame::SetKeyFrame() {
  static unsigned long keyframe_factory_id = 0;
  is_keyframe_ = true;
  keyframe_id_ = keyframe_factory_id++;
}