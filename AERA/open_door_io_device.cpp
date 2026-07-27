#include "open_door_io_device.h"

using namespace r_code;
using namespace r_exec;
using namespace std::chrono;

OpenDoorIODevice::OpenDoorIODevice() : handle_type_() {
}

bool OpenDoorIODevice::load(const std::vector<r_code::Code*>* objects, uint32 stdin_oid, uint32 stdout_oid, uint32 self_oid) {
	if (!MemExec::load(objects, stdin_oid, stdout_oid, self_oid))
		return false;

  move_opcode_ = r_exec::GetOpcode("move");
  turn_opcode_ = r_exec::GetOpcode("turn");
  push_opcode_ = r_exec::GetOpcode("push");
  pull_opcode_ = r_exec::GetOpcode("pull");

  door_object_ = MemStatic::find_object(objects, "d");
  handle_object_ = MemStatic::find_object(objects, "h");
  you_object_ = MemStatic::find_object(objects, "you");

  position_property_ = MemStatic::find_object(objects, "position");
  state_property_ = MemStatic::find_object(objects, "state");
  type_property_ = MemStatic::find_object(objects, "type");

  open_state_ = MemStatic::find_object(objects, "open");
  closed_state_ = MemStatic::find_object(objects, "closed");
  unlocked_state_ = MemStatic::find_object(objects, "unlocked");

  turning_type_ = MemStatic::find_object(objects, "turning");
  push_type_ = MemStatic::find_object(objects, "t_push");
  pull_type_ = MemStatic::find_object(objects, "t_pull");

  start_point_position_ = MemStatic::find_object(objects, "start_point");
  at_door_position_ = MemStatic::find_object(objects, "at_door");
  room_position_ = MemStatic::find_object(objects, "room");

  position_ = start_point_position_;
  //handle_type_ = push_type_;
  door_state_ = closed_state_;

  return true;
}

Code* OpenDoorIODevice::eject(Code* command) {
  uint16 function = (command->code(CMD_FUNCTION).atom_ >> 8) & 0x000000FF;
  uint16 args_set_index = command->code(CMD_ARGS).asIndex();

  if (function == move_opcode_) {
    Code* you = command->get_reference(command->code(args_set_index + 1).asIndex());
    Code* new_pos = command->get_reference(command->code(args_set_index + 2).asIndex());

    if (new_pos == start_point_position_)
      position_ = new_pos;
    else if (new_pos == at_door_position_) {
      position_ = new_pos;
      if (handle_type_.empty())
        handle_type_.push_back(pull_type_);
    }
    else if (new_pos == room_position_ && door_state_ == open_state_)
      position_ = new_pos;

    return command;
  }
  else if (function == turn_opcode_) {
    Code* handle = command->get_reference(command->code(args_set_index + 1).asIndex());
    if (handle_type_.size() != 0 && handle_type_[0] == turning_type_ && handle == handle_object_) {
      door_state_ = unlocked_state_;
    }

    return command;
  }
  else if (function == push_opcode_) {
    Code* door = command->get_reference(command->code(args_set_index + 1).asIndex());
    if (door == door_object_) {
      if (handle_type_.size() != 0 && handle_type_[0] == push_type_) {
        door_state_ = open_state_;
      }
      if (handle_type_.size() != 0 && handle_type_[0] == turning_type_ && door_state_ == unlocked_state_) {
        door_state_ = open_state_;
      }
    }

    return command;
  }
  else if (function == pull_opcode_) {
    Code* door = command->get_reference(command->code(args_set_index + 1).asIndex());
    if (handle_type_.size() != 0 && handle_type_[0] == pull_type_ && door == door_object_) {
      door_state_ = open_state_;
    }

    return command;
  }

  return NULL;
}

void OpenDoorIODevice::on_time_tick() {
  auto now = r_exec::Now();

  MemStatic::inject_marker_value_from_io_device(you_object_, position_property_, position_,
    now, now + MemStatic::get_sampling_period());

  MemStatic::inject_marker_value_from_io_device(door_object_, state_property_, door_state_,
    now, now + MemStatic::get_sampling_period());

  MemStatic::inject_marker_value_from_io_device(handle_object_, type_property_, handle_type_,
    now, now + MemStatic::get_sampling_period());
}