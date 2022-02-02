//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2022 Jeff Thompson
//_/_/ Copyright (c) 2022 Icelandic Institute for Intelligent Machines
//_/_/ http://www.iiim.is
//_/_/
//_/_/ --- Open-Source BSD License, with CADIA Clause v 1.0 ---
//_/_/
//_/_/ Redistribution and use in source and binary forms, with or without
//_/_/ modification, is permitted provided that the following conditions
//_/_/ are met:
//_/_/ - Redistributions of source code must retain the above copyright
//_/_/   and collaboration notice, this list of conditions and the
//_/_/   following disclaimer.
//_/_/ - Redistributions in binary form must reproduce the above copyright
//_/_/   notice, this list of conditions and the following disclaimer 
//_/_/   in the documentation and/or other materials provided with 
//_/_/   the distribution.
//_/_/
//_/_/ - Neither the name of its copyright holders nor the names of its
//_/_/   contributors may be used to endorse or promote products
//_/_/   derived from this software without specific prior 
//_/_/   written permission.
//_/_/   
//_/_/ - CADIA Clause: The license granted in and to the software 
//_/_/   under this agreement is a limited-use license. 
//_/_/   The software may not be used in furtherance of:
//_/_/    (i)   intentionally causing bodily injury or severe emotional 
//_/_/          distress to any person;
//_/_/    (ii)  invading the personal privacy or violating the human 
//_/_/          rights of any person; or
//_/_/    (iii) committing or preparing for any act of war.
//_/_/
//_/_/ THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
//_/_/ CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
//_/_/ INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
//_/_/ MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
//_/_/ DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
//_/_/ CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//_/_/ SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
//_/_/ BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
//_/_/ SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
//_/_/ INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
//_/_/ WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
//_/_/ NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//_/_/ OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
//_/_/ OF SUCH DAMAGE.
//_/_/ 
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#include "video_screen_io_device.h"

using namespace std;
using namespace std::chrono;
using namespace r_code;
using namespace r_exec;

namespace video_screen {

template<class O, class S> VideoScreenIoDevice<O, S>::VideoScreenIoDevice()
: MemExec<O, S>() {
  lastInjectTime_ = Timestamp(seconds(0));
  video_screen_ = NULL;
  ready_opcode_ = 0xFFFF;
  move_opcode_ = 0xFFFF;
  fovea_pattern_property_ = NULL;
  eye_obj_ = NULL;
  lastCommandTime_ = Timestamp(seconds(0));
}

template<class O, class S> VideoScreenIoDevice<O, S>::~VideoScreenIoDevice() {
  if (video_screen_)
    delete video_screen_;
}

template<class O, class S> bool VideoScreenIoDevice<O, S>::load
  (vector<Code*> *objects, uint32 stdin_oid, uint32 stdout_oid,
    uint32 self_oid) {
  // Call the method in the parent class.
  if (!MemExec<O, S>::load(objects, stdin_oid, stdout_oid, self_oid))
    return false;

  if (video_screen_)
    // We don't expect this. load should only be called once.
    delete video_screen_;
  video_screen_ = new VideoScreen(64, 64);
  if (!video_screen_->load(objects))
    return false;

  // Find the opcodes we need.
  ready_opcode_ = r_exec::GetOpcode("ready");
  move_opcode_ = r_exec::GetOpcode("move");

  // Find the objects we need.
  fovea_pattern_property_ = find_object(objects, "fovea_pattern");

  return true;
}

template<class O, class S> Code* VideoScreenIoDevice<O, S>::eject(Code *command) {
  uint16 function = (command->code(CMD_FUNCTION).atom_ >> 8) & 0x000000FF;

  if (function == ready_opcode_) {
    uint16 args_set_index = command->code(CMD_ARGS).asIndex();
    if (command->code_size() >= 2 && command->code(args_set_index + 1).getDescriptor() == Atom::I_PTR &&
        command->code(command->code(args_set_index + 1).asIndex()).getDescriptor() == Atom::STRING) {
      string identifier = Utils::GetString(&command->code(command->code(args_set_index + 1).asIndex()));

      if (identifier == "pong") {
        if (!(command->code_size() >= 3 && command->code(args_set_index + 2).getDescriptor() == Atom::R_PTR &&
              command->references_size() > command->code(args_set_index + 2).asIndex())) {
          cout << "WARNING: Cannot get the object for ready \"pong\"" << endl;
          return NULL;
        }
        if (!fovea_pattern_property_) {
          cout << "WARNING: Can't find the fovea_pattern property" << endl;
          return NULL;
        }

        Code* obj = command->get_reference(command->code(args_set_index + 2).asIndex());
        if (!eye_obj_) {
          // This is the first call. Remember the eye we're controlling.
          eye_obj_ = obj;
          //startTimeTickThread();
        }
        else {
          if (eye_obj_ != obj)
            // For now, don't allow tracking multiple objects.
            return NULL;
        }
        return command;
      }
      else {
        cout << "WARNING: Ignoring unrecognized ready command identifier: " << identifier << endl;
        return NULL;
      }
    }
  }
  else if (function == move_opcode_) {
    auto now = r_exec::Now();
    lastCommandTime_ = now;
    uint16 args_set_index = command->code(CMD_ARGS).asIndex();
    if (command->code(args_set_index).getAtomCount() < 3) {
      cout << "WARNING: Not enough args for (cmd move [eye1 X Y])" << endl;
      return NULL;
    }

    Code* obj = command->get_reference(command->code(args_set_index + 1).asIndex());
    // Set up eye_obj_ the same as the ready "pong" command.
    if (!eye_obj_) {
      // This is the first call. Remember the eye we're controlling.
      //startTimeTickThread();
      eye_obj_ = obj;
    }
    else {
      if (eye_obj_ != obj)
        // For now, don't allow tracking multiple objects.
        return NULL;
    }

    int delta_x = (int)command->code(args_set_index + 2).asFloat();
    int delta_y = (int)command->code(args_set_index + 3).asFloat();

    int actual_delta_x;
    int actual_delta_y;
    video_screen_->move_eye(delta_x, delta_y, actual_delta_x, actual_delta_y);
    if (actual_delta_x == delta_x && actual_delta_y == delta_y)
      // Just return the requested command.
      return command;
    else {
      // Moved by an amount different than the requested amount. Make a command from the actual values.
      // Make (cmd set_force_y [obj: acc:] 1)
      Code* actual_command = new r_exec::LObject(this);
      actual_command->code(0) = Atom::Object(r_exec::GetOpcode("cmd"), 3);
      actual_command->code(1) = Atom::DeviceFunction(move_opcode_);
      actual_command->code(2) = Atom::IPointer(4);
      actual_command->code(3) = Atom::Float(1); // psln_thr.
      actual_command->code(4) = Atom::Set(3);
      actual_command->code(5) = Atom::RPointer(0); // obj
      actual_command->code(6) = Atom::Float(actual_delta_x);
      actual_command->code(7) = Atom::Float(actual_delta_y);
      actual_command->set_reference(0, eye_obj_);

      return actual_command;
    }
  }

  return NULL;
}

template<class O, class S> void VideoScreenIoDevice<O, S>::on_time_tick() {
  auto now = r_exec::Now();
  if (now <= lastInjectTime_ + get_sampling_period() * 8 / 10)
    // Not enough time has elapsed to inject a new measurement.
    return;

  if (eye_obj_) {
#if 0
    if (lastInjectTime_.time_since_epoch().count() == 0) {
      // This is the first call, so leave the initial position.
    }
    else
      position_y_ += velocity_y_ * duration_cast<microseconds>(now - lastInjectTime_).count();
#endif

    lastInjectTime_ = now;
    // Inject the fovea value.
    inject_marker_value_from_io_device(
      eye_obj_, fovea_pattern_property_, video_screen_->get_fovea_pattern(),
      now, now + get_sampling_period());
  }
}

// Instatiate this template class as needed by main(). (Needs C++11.)
template class VideoScreenIoDevice<r_exec::LObject, r_exec::MemStatic>;
template class VideoScreenIoDevice<r_exec::LObject, r_exec::MemVolatile>;

}
