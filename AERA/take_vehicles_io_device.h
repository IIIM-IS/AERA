#ifndef take_vehicles_io_device_h
#define take_vehicles_io_device_h

#include "../r_exec/mem.h"
#include <map>

namespace take_vehicles {

class TakeVehiclesIODevice : public r_exec::MemExec<r_exec::LObject, r_exec::MemStatic> {

public:
  TakeVehiclesIODevice();

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

  uint16 check_oil_opcode_;
  uint16 enter_car_opcode_;
  uint16 open_garage_opcode_;
  uint16 start_engine_opcode_;
  uint16 drive_opcode_;
  uint16 walk_opcode_;
  uint16 take_bus_opcode_;

  r_code::Code* you_object_;
  r_code::Code* car_object_;
  r_code::Code* garage_door_object_;
  
  r_code::Code* position_property_;
  r_code::Code* oil_checked_property_;
  r_code::Code* in_car_property_;
  r_code::Code* started_property_;
  r_code::Code* opened_property_;
  r_code::Code* in_bus_property_;

  r_code::Code* home_position_;
  r_code::Code* destination_position_;

  bool oil_checked_;
  bool in_car_;
  bool garage_opened_;
  bool engine_started_;
  bool in_bus_;
  r_code::Code* position_;
  r_code::Code* destination_;

  Timestamp* start_moving_time_;


  //r_code::Code* on_vehicle_property_;
  //r_code::Code* bus_object_;
  //r_code::Code* foot_object_;

  //r_code::Code* vehicle_;
};
}

#endif
