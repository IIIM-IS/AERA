//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2023 Jeff Thompson
//_/_/ Copyright (c) 2018-2023 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2023 Icelandic Institute for Intelligent Machines
//_/_/ Copyright (c) 2021 Arash Sheikhlar
//_/_/ http://www.iiim.is
//_/_/ 
//_/_/ Copyright (c) 2010-2012 Eric Nivel
//_/_/ Copyright (c) 2010 Thor List
//_/_/ Center for Analysis and Design of Intelligent Agents
//_/_/ Reykjavik University, Menntavegur 1, 102 Reykjavik, Iceland
//_/_/ http://cadia.ru.is
//_/_/ 
//_/_/ Part of this software was developed by Eric Nivel
//_/_/ in the HUMANOBS EU research project, which included
//_/_/ the following parties:
//_/_/
//_/_/ Autonomous Systems Laboratory
//_/_/ Technical University of Madrid, Spain
//_/_/ http://www.aslab.org/
//_/_/
//_/_/ Communicative Machines
//_/_/ Edinburgh, United Kingdom
//_/_/ http://www.cmlabs.com/
//_/_/
//_/_/ Istituto Dalle Molle di Studi sull'Intelligenza Artificiale
//_/_/ University of Lugano and SUPSI, Switzerland
//_/_/ http://www.idsia.ch/
//_/_/
//_/_/ Institute of Cognitive Sciences and Technologies
//_/_/ Consiglio Nazionale delle Ricerche, Italy
//_/_/ http://www.istc.cnr.it/
//_/_/
//_/_/ Dipartimento di Ingegneria Informatica
//_/_/ University of Palermo, Italy
//_/_/ http://diid.unipa.it/roboticslab/
//_/_/
//_/_/
//_/_/ --- HUMANOBS Open-Source BSD License, with CADIA Clause v 1.0 ---
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

#include "test_mem.h"

using namespace std;
using namespace std::chrono;
using namespace r_code;
using namespace r_exec;

template<class O, class S> TestMem<O, S>::TestMem()
  : MemExec<O, S>() {
  timeTickThread_ = 0;
  lastInjectTime_ = Timestamp(seconds(0));
  velocity_y_ = 0.0001;
  force_y_ = 0.1;
  next_velocity_y_ = .1;
  next_position_y_ = 1;
  next_theta_y_ = 1;
  next_omega_y_ = .1;
  position_y_ = NULL;
  position_y_obj_ = NULL;
  cart_position_y_obj_ = NULL;
  position_property_ = NULL;
  position_y_property_ = NULL;
  velocity_y_property_ = NULL;
  force_y_property_ = NULL;
  theta_y_property_ = NULL;
  omega_y_property_ = NULL;
  primary_group_ = NULL;
  ready_opcode_ = 0xFFFF;
  set_velocity_y_opcode_ = 0xFFFF;
  set_force_y_opcode_ = 0xFFFF;
  move_y_plus_opcode_ = 0xFFFF;
  move_y_minus_opcode_ = 0xFFFF;
  lastCommandTime_ = Timestamp(seconds(0));

  for (int i = 0; i <= 9; ++i)
    yEnt_[i] = NULL;
  discretePositionObj_ = NULL;
  discretePosition_ = NULL;
  nextDiscretePosition_ = NULL;
  babbleState_ = 0;
}

float max_force = 20;
template<class O, class S> TestMem<O, S>::~TestMem() {
  if (timeTickThread_)
    delete timeTickThread_;
}

template<class O, class S> bool TestMem<O, S>::load
(const vector<Code*> *objects, uint32 stdin_oid, uint32 stdout_oid,
  uint32 self_oid) {
  // Call the method in the parent class.
  if (!MemExec<O, S>::load(objects, stdin_oid, stdout_oid, self_oid))
    return false;

  // Find the opcodes we need.
  ready_opcode_ = r_exec::GetOpcode("ready");
  set_velocity_y_opcode_ = r_exec::GetOpcode("set_velocity_y");
  set_force_y_opcode_ = r_exec::GetOpcode("set_force_y");
  move_y_plus_opcode_ = r_exec::GetOpcode("move_y_plus");
  move_y_minus_opcode_ = r_exec::GetOpcode("move_y_minus");

  // Find the objects we need.
  position_property_ = find_object(objects, "position");
  position_y_property_ = find_object(objects, "position_y");
  velocity_y_property_ = find_object(objects, "velocity_y");
  force_y_property_ = find_object(objects, "force_y");
  theta_y_property_ = find_object(objects, "theta_y");
  omega_y_property_ = find_object(objects, "omega_y");
  primary_group_ = find_object(objects, "primary");

  // Find the entities we need.
  for (int i = 0; i <= 9; ++i)
    yEnt_[i] = find_object(objects, ("y" + to_string(i)).c_str());

  return true;
}

template<class O, class S> Code* TestMem<O, S>::eject(Code *command) {
  uint16 function = (command->code(CMD_FUNCTION).atom_ >> 8) & 0x000000FF;

  if (function == ready_opcode_) {
    uint16 args_set_index = command->code(CMD_ARGS).asIndex();
    if (command->code_size() >= 2 && command->code(args_set_index + 1).getDescriptor() == Atom::I_PTR &&
        command->code(command->code(args_set_index + 1).asIndex()).getDescriptor() == Atom::STRING) {
      string identifier = Utils::GetString(&command->code(command->code(args_set_index + 1).asIndex()));

      if (identifier == "ball") {
        if (!(command->code_size() >= 3 && command->code(args_set_index + 2).getDescriptor() == Atom::R_PTR &&
              command->references_size() > command->code(args_set_index + 2).asIndex())) {
          cout << "WARNING: Cannot get the object for ready \"ball\"" << endl;
          return NULL;
        }
        if (!velocity_y_property_) {
          cout << "WARNING: Can't find the velocity_y property" << endl;
          return NULL;
        }
        if (!position_y_property_) {
          cout << "WARNING: Can't find the position_y property" << endl;
          return NULL;
        }

        Code* obj = command->get_reference(command->code(args_set_index + 2).asIndex());
        if (!position_y_obj_) {
          // This is the first call. Remember the object whose position we're reporting.
          position_y_obj_ = obj;
          startTimeTickThread();
        }
        else {
          if (position_y_obj_ != obj)
            // For now, don't allow tracking multiple objects.
            return NULL;
        }
        return command;
      }
      if (identifier == "cart-pole") {
        if (!(command->code_size() >= 3 && command->code(args_set_index + 2).getDescriptor() == Atom::R_PTR &&
          command->references_size() > command->code(args_set_index + 2).asIndex())) {
          cout << "WARNING: Cannot get the object for ready \"ball\"" << endl;
          return NULL;
        }
        if (!velocity_y_property_) {
          cout << "WARNING: Can't find the velocity_y property" << endl;
          return NULL;
        }
        if (!position_y_property_) {
          cout << "WARNING: Can't find the position_y property" << endl;
          return NULL;
        }
        if (!force_y_property_) {
          cout << "WARNING: Can't find the force_y property" << endl;
          return NULL;
        }
        if (!omega_y_property_) {
          cout << "WARNING: Can't find the omega_y property" << endl;
          return NULL;
        }
        if (!theta_y_property_) {
          cout << "WARNING: Can't find the theta_y property" << endl;
          return NULL;
        }
        Code* obj = command->get_reference(command->code(args_set_index + 2).asIndex());
        if (!cart_position_y_obj_) {
          // This is the first call. Remember the object whose position we're reporting.
          cart_position_y_obj_ = obj;
          startTimeTickThread();
        }

        else {
          if (cart_position_y_obj_ != obj)
            // For now, don't allow tracking multiple objects.
            return NULL;
        }
        ofs.open("cart.out");
        return command;
      }
      else {
        cout << "WARNING: Ignoring unrecognized ready command identifier: " << identifier << endl;
        return NULL;
      }
    }
  }
  else if (function == set_velocity_y_opcode_) {
    if (!velocity_y_property_) {
      cout << "WARNING: Can't find the velocity_y property" << endl;
      return NULL;
    }
    if (!position_y_property_) {
      cout << "WARNING: Can't find the position_y property" << endl;
      return NULL;
    }

    auto now = r_exec::Now();
    lastCommandTime_ = now;
    uint16 args_set_index = command->code(CMD_ARGS).asIndex();
    Code* obj = command->get_reference(
      command->code(args_set_index + 1).asIndex());
    // Set up position_y_obj_ the same as the ready "ball" command.
    if (!position_y_obj_) {
      // This is the first call. Remember the object whose velocity we're setting.
      position_y_obj_ = obj;
      startTimeTickThread();
    }
    else {
      if (position_y_obj_ != obj)
        // For now, don't allow tracking the velocity of multiple objects.
        return NULL;
    }

    velocity_y_ = command->code(args_set_index + 2).asFloat();
    // Let on_time_tick inject the new velocity_y.
    return command;
  }
  else if (function == set_force_y_opcode_) {
    if (!velocity_y_property_) {
      cout << "WARNING: Can't find the velocity_y property" << endl;
      return NULL;
    }
    if (!position_y_property_) {
      cout << "WARNING: Can't find the position_y property" << endl;
      return NULL;
    }
    if (!omega_y_property_) {
      cout << "WARNING: Can't find the omega_y property" << endl;
      return NULL;
    }
    if (!theta_y_property_) {
      cout << "WARNING: Can't find the theta_y property" << endl;
      return NULL;
    }
    if (!force_y_property_) {
      cout << "WARNING: Can't find the force_y property" << endl;
      return NULL;
    }
    auto now = r_exec::Now();
    lastCommandTime_ = now;
    uint16 args_set_index = command->code(CMD_ARGS).asIndex();
    Code* obj = command->get_reference(
    command->code(args_set_index + 1).asIndex());
    // Set up position_y_obj_ the same as the ready "ball" command.
    if (!cart_position_y_obj_) {
      // This is the first call. Remember the object whose velocity we're setting.
      cart_position_y_obj_ = obj;
      startTimeTickThread();
    }
    else {
    if (cart_position_y_obj_ != obj)
      // For now, don't allow tracking the velocity of multiple objects.
      return NULL;
    }

    float desired_force = command->code(args_set_index + 2).asFloat();
    if ((desired_force > max_force) || (desired_force < -max_force)) {
      if (desired_force > max_force)
        desired_force = max_force;
      else if (desired_force < -max_force)
        desired_force = -max_force;

      force_y_ = desired_force;

      uint16 AccCommand;
      AccCommand = set_force_y_opcode_;
      // Make (cmd set_force_y [obj: acc:] 1)
      Code* cmdAcc = new r_exec::LObject(this);
      cmdAcc->code(0) = Atom::Object(r_exec::GetOpcode("cmd"), 3);
      cmdAcc->code(1) = Atom::DeviceFunction(AccCommand);
      cmdAcc->code(2) = Atom::IPointer(4);
      cmdAcc->code(3) = Atom::Float(1); // psln_thr.
      cmdAcc->code(4) = Atom::Set(2);
      cmdAcc->code(5) = Atom::RPointer(0); // obj
      cmdAcc->code(6) = Atom::Float(force_y_);
      cmdAcc->set_reference(0, cart_position_y_obj_);

      return cmdAcc;
    }
    else {
      force_y_ = desired_force;
      return command;
    }
    // Let on_time_tick inject the new force_y.
  }
  else if (function == move_y_plus_opcode_ ||
    function == move_y_minus_opcode_) {
    if (!position_property_) {
      cout << "WARNING: Can't find the position property" << endl;
      return NULL;
    }
    for (int i = 0; i <= 9; ++i) {
      if (!yEnt_[i]) {
        cout << "WARNING: Can't find the entities y0, y1, etc." << endl;
        return NULL;
      }
    }

    auto now = r_exec::Now();

    uint16 args_set_index = command->code(CMD_ARGS).asIndex();
    Code* obj = command->get_reference(
      command->code(args_set_index + 1).asIndex());
    if (!discretePositionObj_) {
      // This is the first call. Remember the object whose position we're setting.
      discretePositionObj_ = obj;
      discretePosition_ = yEnt_[0];
      startTimeTickThread();
    }
    else {
      if (discretePositionObj_ != obj)
        // For now, don't allow tracking multiple objects.
        return NULL;
    }

    if (nextDiscretePosition_)
      // A previous move command is still pending execution.
      return NULL;

    // nextDiscretePosition_ will become the position at the next sampling period.
    lastCommandTime_ = now;
    const int maxYPosition = 9;
    if (function == move_y_plus_opcode_) {
      for (int i = 0; i <= maxYPosition - 1; ++i) {
        if (discretePosition_ == yEnt_[i])
          nextDiscretePosition_ = yEnt_[i + 1];
      }
    }
    else if (function == move_y_minus_opcode_) {
      for (int i = 1; i <= maxYPosition; ++i) {
        if (discretePosition_ == yEnt_[i])
          nextDiscretePosition_ = yEnt_[i - 1];
      }
    }
    // Let on_time_tick inject the new position.
    return command;
  }

  return NULL;
}

template<class O, class S> void TestMem<O, S>::on_time_tick() {
  auto now = r_exec::Now();
  if (now <= lastInjectTime_ + get_sampling_period() * 8 / 10)
    // Not enough time has elapsed to inject a new position.
    return;

  if (position_y_obj_) {
    // We are updating the continuous position_y_.
    if (lastInjectTime_.time_since_epoch().count() == 0) {
      // This is the first call, so leave the initial position.
    }
    else
      position_y_ += velocity_y_ * duration_cast<microseconds>(now - lastInjectTime_).count();

    lastInjectTime_ = now;
    // Inject the velocity and position.
    // It seems that velocity_y needs SYNC_HOLD for building models.
    inject_marker_value_from_io_device(
      position_y_obj_, velocity_y_property_, Atom::Float(velocity_y_),
      now, now + get_sampling_period(), r_exec::View::SYNC_HOLD);
    inject_marker_value_from_io_device(
      position_y_obj_, position_y_property_, Atom::Float(position_y_),
      now, now + get_sampling_period());
  }
  if (cart_position_y_obj_) {
    // We are updating the cart position_y_.
    if (lastInjectTime_.time_since_epoch().count() == 0) {
      // This is the first call, so leave the initial position.
    }
    else
    {
      auto DeltaK = 1e-6 * duration_cast<microseconds>(now - lastInjectTime_).count();
      float current_position_y;
      float current_velocity_y;
      float current_theta_y;
      float current_omega_y;

      current_position_y = next_position_y_;
      current_velocity_y = next_velocity_y_;
      current_theta_y = next_theta_y_;
      current_omega_y = next_omega_y_;

      //next_velocity_y_ = (((1. / M) * force_y_) * DeltaK) + current_velocity_y;     
      next_velocity_y_ = (0.1 * force_y_) + current_velocity_y;
      //next_position_y_ = (0.5 * ((1. / M) * force_y_) * DeltaK * DeltaK) + (current_velocity_y * DeltaK) + current_position_y;
      next_position_y_ = (0.5e-2 * force_y_) + (0.1 * current_velocity_y) + current_position_y;
      //next_omega_y_ = A*current_theta_y + B*force_y_ + current_omega_y;
      next_omega_y_ = current_theta_y + 0.1 * force_y_ + current_omega_y;
      //next_theta_y_ = A * force_y_ + B * current_theta_y + C * current_omega_y;
      next_theta_y_ = 0.0371943 * force_y_ + 1.05042 * current_theta_y + 0.101675 * current_omega_y;
      /* Calculate the constants of theta and omega equations
      #include<iostream>
      #include<string>
      #include<algorithm>
      using namespace std;
      float max_force = 15; float d = 0;  float m = .00001; float M = 1;  float b = 1;  float L = 100;  float g = -10; float DeltaK = 0.1;
      int main (){ float p = (-g / L);
      float A_omg = (-g / L) * DeltaK; float B_omg = DeltaK / (M * L);
      float A_th = ((-2 + exp(sqrt(p) * DeltaK) + exp(sqrt(p) * DeltaK)) / (2 * p));
      float B_th = ((exp(-sqrt(p) * DeltaK) + exp(sqrt(p) * DeltaK)) / 2);
      float C_th = ((-sqrt(p) * exp(-sqrt(p) * DeltaK) + sqrt(p) * exp(sqrt(p) * DeltaK)) / (2 * p));
      cout << "P: " << p << endl;  cout << "A_omega: " << A_omg << endl; cout << "B_omega: " << B_omg << endl;
      cout << "A_theta: " << A_th << endl; cout << "B_theta: " << B_th << endl; cout << "C_theta: " << C_th << endl;
      return 0;}
      */
    }
    lastInjectTime_ = now;


    inject_marker_value_from_io_device(
      cart_position_y_obj_, force_y_property_, Atom::Float(force_y_),
      now, now + get_sampling_period());
    inject_marker_value_from_io_device(
      cart_position_y_obj_, velocity_y_property_, Atom::Float(next_velocity_y_),
      now, now + get_sampling_period());
    inject_marker_value_from_io_device(
      cart_position_y_obj_, position_y_property_, Atom::Float(next_position_y_),
      now, now + get_sampling_period());
    inject_marker_value_from_io_device(
      cart_position_y_obj_, theta_y_property_, Atom::Float(next_theta_y_),
      now, now + get_sampling_period());
    inject_marker_value_from_io_device(
      cart_position_y_obj_, omega_y_property_, Atom::Float(next_omega_y_),
      now, now + get_sampling_period());

    ofs << next_theta_y_ << "," << next_omega_y_ << "," << next_position_y_ << "," << next_velocity_y_ << "," << force_y_ << endl;
  }
  if (discretePositionObj_) {
    // We are updating the discretePosition_.
    if (nextDiscretePosition_ &&
        now >= lastCommandTime_ + get_sampling_period() * 4 / 10) {
      // Enough time has elapsed from the move command to update the position.
      discretePosition_ = nextDiscretePosition_;
      // Clear nextDiscretePosition_ to allow another move command.
      nextDiscretePosition_ = NULL;
    }

    lastInjectTime_ = now;
    inject_marker_value_from_io_device(
      discretePositionObj_, position_property_, discretePosition_,
      now, now + get_sampling_period());

    const microseconds babbleStopTime(3700000);
    const int maxBabblePosition = 9;
    if (now - Utils::GetTimeReference() < babbleStopTime) {
      // Babble.
      if (discretePosition_ == yEnt_[0])
        // Reset to the expected value.
        babbleState_ = 1;
      else if (discretePosition_ == yEnt_[maxBabblePosition])
        // Reset to the expected value.
        babbleState_ = maxBabblePosition + 1;
      else {
        ++babbleState_;
        if (babbleState_ >= maxBabblePosition * 2)
          babbleState_ = 0;
      }

      uint16 nextCommand;
      if (babbleState_ >= 0 && babbleState_ <= maxBabblePosition - 1)
        nextCommand = move_y_plus_opcode_;
      else
        nextCommand = move_y_minus_opcode_;

      // Inject (fact (goal (fact (cmd ...))))
      Code *cmd = new r_exec::LObject(this);
      cmd->code(0) = Atom::Object(r_exec::GetOpcode("cmd"), 3);
      cmd->code(1) = Atom::DeviceFunction(nextCommand);
      cmd->code(2) = Atom::IPointer(4);
      cmd->code(3) = Atom::Float(1); // psln_thr.
      cmd->code(4) = Atom::Set(1);
      cmd->code(5) = Atom::RPointer(0); // obj
      cmd->set_reference(0, discretePositionObj_);

      auto after = now + get_sampling_period() + 2 * Utils::GetTimeTolerance();
      r_exec::Fact* factCmd = new r_exec::Fact(cmd, after, now + 2 * get_sampling_period(), 1, 1);
      r_exec::Goal* goal = new r_exec::Goal(factCmd, get_self(), NULL, 1);
      inject_fact_from_io_device(goal, after, now + get_sampling_period(), primary_group_);
    }
  }
}

template<class O, class S> void
TestMem<O, S>::startTimeTickThread() {
  if (reduction_core_count_ == 0 && time_core_count_ == 0)
    // We don't need a timeTickThread for diagnostic time.
    return;
  if (timeTickThread_)
    // We already started the thread.
    return;

  // We are running in real time. on_diagnostic_time_tick() will not be called.
  // Set up a timer thread to call on_time_tick().
  timeTickThread_ = Thread::New<_Thread>(timeTickRun, this);
}

template<class O, class S> thread_ret thread_function_call
TestMem<O, S>::timeTickRun(void *args) {
  TestMem<O, S>* self = (TestMem *)args;

  auto sampling_period = MemExec::Get()->get_sampling_period();
  auto tickTime = r_exec::Now();
  // Call on_time_tick at the sampling period.
  while (self->state_ == RUNNING) {
    self->on_time_tick();

    tickTime += sampling_period;
    Thread::Sleep(tickTime - r_exec::Now());
  }

  thread_ret_val(0);
}

// Instatiate this template class as needed by main(). (Needs C++11.)
template class TestMem<r_exec::LObject, r_exec::MemStatic>;
template class TestMem<r_exec::LObject, r_exec::MemVolatile>;
