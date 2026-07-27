#ifndef open_door_io_device_h
#define open_door_io_device_h

#include "../r_exec/mem.h"

class OpenDoorIODevice : public r_exec::MemExec<r_exec::LObject, r_exec::MemStatic> {

public:
  OpenDoorIODevice();

  // Called in main.cpp
  bool load(const std::vector<r_code::Code*>* objects, uint32 stdin_oid, uint32 stdout_oid, uint32 self_oid) override;

  /**
  * Receive a command from AERA.
  *
  * \return The given command if it is executed as-is, or a new command object of the command
  * that is actually executed. The program controller will make a fact from the command and
  * inject it as the efferent copy. However, if the command is not executed, then return NULL
  * and the program controller will put an anti-fact of the command in the mk.rdx reduction.
  */
  r_code::Code* eject(r_code::Code* command) override;

  // Called in DiagnosticTimeState::step()
  void on_diagnostic_time_tick() override { on_time_tick(); }

protected:
  // Objects can be injected into the working memory from here (use injection functions from _Mem)
  void on_time_tick();

  uint16 move_opcode_;
  uint16 turn_opcode_;
  uint16 push_opcode_;
  uint16 pull_opcode_;

  r_code::Code* door_object_;
  r_code::Code* handle_object_;
  r_code::Code* you_object_;

  r_code::Code* position_property_;
  r_code::Code* state_property_;
  r_code::Code* type_property_;

  r_code::Code* open_state_;
  r_code::Code* closed_state_;
  r_code::Code* unlocked_state_;

  r_code::Code* turning_type_;
  r_code::Code* push_type_;
  r_code::Code* pull_type_;

  r_code::Code* start_point_position_;
  r_code::Code* at_door_position_;
  r_code::Code* room_position_;


  r_code::Code* position_;
  r_code::Code* door_state_;
  std::vector<r_code::Code*> handle_type_;
};

#endif