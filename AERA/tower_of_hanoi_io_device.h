#ifndef tower_of_hanoi_io_device_h
#define tower_of_hanoi_io_device_h

#include <stack>
#include <array>
#include "../r_exec/mem.h"

namespace tower_of_hanoi {

class TowerOfHanoiIODevice : public r_exec::MemExec<r_exec::LObject, r_exec::MemStatic> {

public:
  TowerOfHanoiIODevice(std::string source_file_name);

  //~TowerOfHanoiIODevice();

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

  bool move(int from, int to) {
    if (from > 2 || from < 0 || to < 0 || to > 2)
      return false;

    r_code::Code* from_block = poles_[from].back();
    if (!poles_[to].empty() && is_smaller(poles_[to].back(), from_block)) {
      // Cannot move if destination has a smaller block
      return false;
    }

    poles_[from].pop_back();
    poles_[to].push_back(from_block);
    return true;
  }

  bool move_block(int block, int pole) {
    int from = positions_[block];

    if (!isTopBlock(block))
      return false;

    if (!move(from, pole))
      return false;

    //for (int d = block; d < 2; ++d) {
    //  if (positions_[d] == pole)
    //    return false;
    //}

    positions_[block] = pole;
    return true;
  }

  // Returns true if block b1 is smaller than block b2
  bool is_smaller(r_code::Code* b1, r_code::Code* b2) {
    if (b1 == small_block_object_)
      return true;
    if (b1 == big_block_object_)
      return false;
    if (b1 == medium_block_object_)
      return b2 == big_block_object_;
  }

  bool isTopBlock(int block) const {
    int pole = positions_[block];

    for (int d = 0; d < block; ++d) {
      if (positions_[d] == pole) {
        return false;
      }
    }
    return true;
  }

  int pole_object_to_index(r_code::Code* pole_object) {
    if (pole_object == left_pole_object_)
      return 0;
    if (pole_object == center_pole_object_)
      return 1;
    if (pole_object == right_pole_object_)
      return 2;
  }

  r_code::Code* index_to_pole_object(int index) {
    if (index == 0)
      return left_pole_object_;
    if (index == 1)
      return center_pole_object_;
    if (index == 2)
      return right_pole_object_;
  }

  int block_object_to_index(r_code::Code* block_object) {
    if (block_object == small_block_object_)
      return 0;
    if (block_object == medium_block_object_)
      return 1;
    if (block_object == big_block_object_)
      return 2;
  }

  uint16 move_opcode_;
  uint16 move_block_opcode_;
  r_code::Code* position_property_;
  r_code::Code* has_blocks_property_;
  r_code::Code* left_pole_object_;
  r_code::Code* center_pole_object_;
  r_code::Code* right_pole_object_;
  r_code::Code* big_block_object_;
  r_code::Code* medium_block_object_;
  r_code::Code* small_block_object_;

  // 0: small, 1: medium, 2: big -- values are same as poles_ array
  std::array<int, 3> positions_;
  // 0: left, 1: center, 2: right
  std::array<std::vector<r_code::Code*>, 3> poles_;

  std::string source_file_name_;
};
}

#endif
