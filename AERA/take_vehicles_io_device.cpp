#include "take_vehicles_io_device.h"

using namespace std;
using namespace std::chrono;
using namespace r_code;
using namespace r_exec;

namespace take_vehicles {

TakeVehiclesIODevice::TakeVehiclesIODevice() {
	oil_checked_ = false;
	in_car_ = false;
	garage_opened_ = false;
	engine_started_ = false;
	in_bus_ = false;

	start_moving_time_ = NULL;
	destination_ = NULL;
}

bool TakeVehiclesIODevice::load(const std::vector<r_code::Code*>* objects, uint32 stdin_oid, uint32 stdout_oid, uint32 self_oid) {
	if (!MemExec::load(objects, stdin_oid, stdout_oid, self_oid))
		return false;

	check_oil_opcode_ = r_exec::GetOpcode("check_oil");
	enter_car_opcode_ = r_exec::GetOpcode("enter_car");
	open_garage_opcode_ = r_exec::GetOpcode("open_garage");
	start_engine_opcode_ = r_exec::GetOpcode("start_engine");
	drive_opcode_ = r_exec::GetOpcode("drive");
	walk_opcode_ = r_exec::GetOpcode("walk");
	take_bus_opcode_ = r_exec::GetOpcode("take_bus");

	you_object_ = MemStatic::find_object(objects, "you");
	car_object_ = MemStatic::find_object(objects, "car");
	garage_door_object_ = MemStatic::find_object(objects, "garage_door");

	position_property_ = MemStatic::find_object(objects, "position");
	oil_checked_property_ = MemStatic::find_object(objects, "oil_checked");
	in_car_property_ = MemStatic::find_object(objects, "in_car");
	started_property_ = MemStatic::find_object(objects, "started");
	opened_property_ = MemStatic::find_object(objects, "opened");
	in_bus_property_ = MemStatic::find_object(objects, "in_bus");

	home_position_ = MemStatic::find_object(objects, "home");
	destination_position_ = MemStatic::find_object(objects, "destination");

	position_ = home_position_;

	//on_vehicle_property_ = MemStatic::find_object(objects, "on_vehicle");
	//bus_object_ = MemStatic::find_object(objects, "bus");
	//foot_object_ = MemStatic::find_object(objects, "foot");
	//vehicle_ = car_object_;
}

Code* TakeVehiclesIODevice::eject(r_code::Code* command) {
	uint16 function = (command->code(CMD_FUNCTION).atom_ >> 8) & 0x000000FF;

	if (function == check_oil_opcode_) {
		uint16 args_set_index = command->code(CMD_ARGS).asIndex();
		Code* car = command->get_reference(command->code(args_set_index + 1).asIndex());

		oil_checked_ = true;
		return command;
	}
	else if (function == enter_car_opcode_) {
		uint16 args_set_index = command->code(CMD_ARGS).asIndex();
		Code* car = command->get_reference(command->code(args_set_index + 1).asIndex());

		in_car_ = true;
		return command;
	}
	else if (function == open_garage_opcode_) {
		uint16 args_set_index = command->code(CMD_ARGS).asIndex();
		Code* garage = command->get_reference(command->code(args_set_index + 1).asIndex());

		garage_opened_ = true;
		return command;
	}
	else if (function == start_engine_opcode_) {
		uint16 args_set_index = command->code(CMD_ARGS).asIndex();
		Code* car = command->get_reference(command->code(args_set_index + 1).asIndex());

		engine_started_ = true;
		return command;
	}
	else if (function == drive_opcode_) {
		uint16 args_set_index = command->code(CMD_ARGS).asIndex();
		Code* car = command->get_reference(command->code(args_set_index + 1).asIndex());
		Code* destination = command->get_reference(command->code(args_set_index + 2).asIndex());
		
		if (!oil_checked_ || !in_car_ || !garage_opened_ || !engine_started_)
			return command;

		destination_ = destination;
		start_moving_time_ = &r_exec::Now();
		return command;
	}
	else if (function == walk_opcode_) {
		uint16 args_set_index = command->code(CMD_ARGS).asIndex();
		Code* destination = command->get_reference(command->code(args_set_index + 1).asIndex());

		if (in_car_)
			return command;

		destination_ = destination;
		start_moving_time_ = &r_exec::Now();
		return command;
	}
	else if (function == take_bus_opcode_) {
		uint16 args_set_index = command->code(CMD_ARGS).asIndex();
		Code* destination = command->get_reference(command->code(args_set_index + 1).asIndex());

		if (in_car_)
			return command;

		destination_ = destination;
		in_bus_ = true;
		start_moving_time_ = &r_exec::Now();
		return command;
	}

	return NULL;
}

void TakeVehiclesIODevice::on_time_tick() {
	auto now = r_exec::Now();

	if (in_car_ && duration_cast<milliseconds>(now - *start_moving_time_).count() >= 300) {
		position_ = destination_;
		destination_ = NULL;
		start_moving_time_ = NULL;
	}
	else if (in_bus_ && duration_cast<milliseconds>(now - *start_moving_time_).count() >= 200) {
		position_ = destination_;
		destination_ = NULL;
		start_moving_time_ = NULL;
	}

	MemStatic::inject_marker_value_from_io_device(you_object_, position_property_, position_,
		now, now + MemStatic::get_sampling_period(), r_exec::View::SYNC_HOLD);

	MemStatic::inject_marker_value_from_io_device(car_object_, oil_checked_property_, Atom::Boolean(oil_checked_),
		now, now + MemStatic::get_sampling_period(), r_exec::View::SYNC_HOLD);

	MemStatic::inject_marker_value_from_io_device(car_object_, started_property_, Atom::Boolean(engine_started_),
		now, now + MemStatic::get_sampling_period(), r_exec::View::SYNC_HOLD);

	auto vehicle = std::vector<Code*>();
	if (in_car_)
		vehicle.push_back(car_object_);
	MemStatic::inject_marker_value_from_io_device(you_object_, in_car_property_, vehicle,
		now, now + MemStatic::get_sampling_period(), r_exec::View::SYNC_HOLD);

	MemStatic::inject_marker_value_from_io_device(garage_door_object_, opened_property_, Atom::Boolean(garage_opened_),
		now, now + MemStatic::get_sampling_period(), r_exec::View::SYNC_HOLD);

	MemStatic::inject_marker_value_from_io_device(you_object_, in_bus_property_, Atom::Boolean(in_bus_),
		now, now + MemStatic::get_sampling_period(), r_exec::View::SYNC_HOLD);
}

}