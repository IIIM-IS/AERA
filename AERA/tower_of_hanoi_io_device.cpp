#include "tower_of_hanoi_io_device.h"

using namespace r_code;
using namespace r_exec;

namespace tower_of_hanoi {

TowerOfHanoiIODevice::TowerOfHanoiIODevice(std::string source_file_name) : poles_(), positions_() {
	move_opcode_ = 0xFFFF;
	left_pole_object_ = NULL;
	center_pole_object_ = NULL;
	right_pole_object_ = NULL;

	source_file_name_ = source_file_name;
}

bool TowerOfHanoiIODevice::load(const std::vector<r_code::Code*>* objects, uint32 stdin_oid, uint32 stdout_oid, uint32 self_oid) {
	if (!MemExec::load(objects, stdin_oid, stdout_oid, self_oid))
		return false;

	move_opcode_ = r_exec::GetOpcode("move");
	move_block_opcode_ = r_exec::GetOpcode("move_block");
	position_property_ = MemStatic::find_object(objects, "position");
	has_blocks_property_ = MemStatic::find_object(objects, "has_blocks");
	left_pole_object_ = MemStatic::find_object(objects, "left_pole");
	center_pole_object_ = MemStatic::find_object(objects, "center_pole");
	right_pole_object_ = MemStatic::find_object(objects, "right_pole");
	big_block_object_ = MemStatic::find_object(objects, "big_block");
	medium_block_object_ = MemStatic::find_object(objects, "medium_block");
	small_block_object_ = MemStatic::find_object(objects, "small_block");

	if (source_file_name_.find("learn") != std::string::npos) {
		positions_[2] = 1; // Big (2) at center (1)
		positions_[1] = 1; // Medium (1) at center (1)
		positions_[0] = 0; // Small (0) at left (0)

		poles_[1].push_back(big_block_object_); // Big at center (1)
		poles_[1].push_back(medium_block_object_); // Medium at center (1)
		poles_[0].push_back(small_block_object_); // Small at left (0)
	}
	else if (source_file_name_.find("full") != std::string::npos) {
		//positions_[2] = 2; // Big (2) at right (2)
		//positions_[1] = 1; // Medium (1) at center (1)
		//positions_[0] = 2; // Small (0) at right (2)

		//poles_[2].push_back(big_block_object_); // Big at right (2)
		//poles_[1].push_back(medium_block_object_); // Medium at center (1)
		//poles_[2].push_back(small_block_object_); // Small at right (2)

		positions_[2] = 2; // Big (2) at right (2)
		positions_[1] = 0; // Medium (1) at left (0)
		positions_[0] = 0; // Small (0) at left (0)

		poles_[2].push_back(big_block_object_); // Big at right (2)
		poles_[0].push_back(medium_block_object_); // Medium at left (0)
		poles_[0].push_back(small_block_object_); // Small at left (0)
	}

	return true;
}

Code* TowerOfHanoiIODevice::eject(Code* command) {
	uint16 function = (command->code(CMD_FUNCTION).atom_ >> 8) & 0x000000FF;

	if (function == move_block_opcode_) {
		uint16 args_set_index = command->code(CMD_ARGS).asIndex();

		Code* block = command->get_reference(command->code(args_set_index + 1).asIndex());
		Code* pole = command->get_reference(command->code(args_set_index + 2).asIndex());

		if (!move_block(block_object_to_index(block), pole_object_to_index(pole)))
			return command;

		return command;
	}

	return NULL;
}

void TowerOfHanoiIODevice::on_time_tick() {
	auto now = r_exec::Now();

	MemStatic::inject_marker_value_from_io_device(left_pole_object_, has_blocks_property_, poles_[0],
		now, now + MemStatic::get_sampling_period());

	MemStatic::inject_marker_value_from_io_device(center_pole_object_, has_blocks_property_, poles_[1],
		now, now + MemStatic::get_sampling_period());

	MemStatic::inject_marker_value_from_io_device(right_pole_object_, has_blocks_property_, poles_[2],
		now, now + MemStatic::get_sampling_period());
}

}