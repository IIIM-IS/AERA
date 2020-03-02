//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ HUMANOBS - Replicode Test
//_/_/
//_/_/ Jeff Thompson
//_/_/ Center for Analysis and Design of Intelligent Agents
//_/_/   Reykjavik University, Menntavegur 1, 101 Reykjavik, Iceland
//_/_/   http://cadia.ru.is
//_/_/ Copyright(c)2012
//_/_/
//_/_/ This software was developed by the above copyright holder as part of 
//_/_/ the HUMANOBS EU research project, in collaboration with the 
//_/_/ following parties:
//_/_/ 
//_/_/ Autonomous Systems Laboratory
//_/_/   Technical University of Madrid, Spain
//_/_/   http://www.aslab.org/
//_/_/
//_/_/ Communicative Machines
//_/_/   Edinburgh, United Kingdom
//_/_/   http://www.cmlabs.com/
//_/_/
//_/_/ Istituto Dalle Molle di Studi sull'Intelligenza Artificiale
//_/_/   University of Lugano and SUPSI, Switzerland
//_/_/   http://www.idsia.ch/
//_/_/
//_/_/ Institute of Cognitive Sciences and Technologies
//_/_/   Consiglio Nazionale delle Ricerche, Italy
//_/_/   http://www.istc.cnr.it/
//_/_/
//_/_/ Dipartimento di Ingegneria Informatica
//_/_/   University of Palermo, Italy
//_/_/   http://roboticslab.dinfo.unipa.it/index.php/Main/HomePage
//_/_/
//_/_/
//_/_/ --- HUMANOBS Open-Source BSD License, with CADIA Clause v 1.0 ---
//_/_/
//_/_/ Redistribution and use in source and binary forms, with or without 
//_/_/ modification, is permitted provided that the following conditions 
//_/_/ are met:
//_/_/
//_/_/ - Redistributions of source code must retain the above copyright 
//_/_/ and collaboration notice, this list of conditions and the 
//_/_/ following disclaimer.
//_/_/
//_/_/ - Redistributions in binary form must reproduce the above copyright 
//_/_/ notice, this list of conditions and the following
//_/_/ disclaimer in the documentation and/or other materials provided 
//_/_/ with the distribution.
//_/_/
//_/_/ - Neither the name of its copyright holders nor the names of its 
//_/_/ contributors may be used to endorse or promote products 
//_/_/ derived from this software without specific prior written permission.
//_/_/
//_/_/ - CADIA Clause: The license granted in and to the software under this 
//_/_/ agreement is a limited-use license. The software may not be used in 
//_/_/ furtherance of: 
//_/_/ (i) intentionally causing bodily injury or severe emotional distress 
//_/_/ to any person; 
//_/_/ (ii) invading the personal privacy or violating the human rights of 
//_/_/ any person; or 
//_/_/ (iii) committing or preparing for any act of war.
//_/_/
//_/_/ THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//_/_/ "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//_/_/ LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
//_/_/ A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//_/_/ OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//_/_/ SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//_/_/ LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
//_/_/ DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
//_/_/ THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//_/_/ (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//_/_/ OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//_/_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#include "test_mem.h"

template<class O, class S> TestMem<O, S>::TestMem() 
: r_exec::Mem<O, S>() {
  timeTickThread_ = 0;
  lastInjectTime_ = 0;
  speed_y_ = 0;
  position_y_ = NULL;
  position_y_obj_ = NULL;
  position_property_ = NULL;
  position_y_property_ = NULL;
  speed_y_property_ = NULL;
  primary_group_ = NULL,
  set_speed_y_opcode_ = 0xFFFF;
  move_y_plus_opcode_ = 0xFFFF;
  move_y_minus_opcode_ = 0xFFFF;
  lastCommandTime_ = 0;

  for (int i = 0; i <= 9; ++i)
    yEnt_[i] = NULL;
  discretePositionObj_ = NULL;
  discretePosition_ = NULL;
  nextDiscretePosition_ = NULL;
  babbleState_ = 0;
}

template<class O, class S> TestMem<O, S>::~TestMem() {
  if (timeTickThread_)
    delete timeTickThread_;
}

template<class O, class S> bool TestMem<O, S>::load
  (std::vector<Code*> *objects, uint32 stdin_oid, uint32 stdout_oid,
   uint32 self_oid) {
  // Call the method in the parent class.
  if (!r_exec::Mem<O, S>::load(objects, stdin_oid, stdout_oid, self_oid))
    return false;

  // Find the opcodes we need.
  set_speed_y_opcode_ = r_exec::GetOpcode("set_speed_y");
  move_y_plus_opcode_ = r_exec::GetOpcode("move_y_plus");
  move_y_minus_opcode_ = r_exec::GetOpcode("move_y_minus");

  // Find the objects we need.
  position_property_ = findObject(objects, "position");
  position_y_property_ = findObject(objects, "position_y");
  speed_y_property_ = findObject(objects, "speed_y");
  primary_group_ = findObject(objects, "primary");

  // Find the entities we need.
  for (int i = 0; i <= 9; ++i)
    yEnt_[i] = findObject(objects, ("y" + to_string(i)).c_str());

  return true;
}

template<class O, class S> Code* 
TestMem<O, S>::findObject(std::vector<Code*> *objects, const char* name) {
  // Find the object OID.
  uint32 oid = 0;
  for (UNORDERED_MAP<uint32, std::string>::const_iterator it = r_exec::Seed.object_names.symbols.begin();
       it != r_exec::Seed.object_names.symbols.end(); ++it) {
    if (it->second == name) {
      oid = it->first;
      break;
    }
  }

  if (oid == 0)
    return NULL;

  // Find the object. (Imitate the code in _Mem::load.)
  for (uint32 i = 0; i < objects->size(); ++i) {
    Code *object = (*objects)[i];
    if (object->get_oid() == oid)
      return object;
  }

  return NULL;
}

template<class O, class S> r_exec::View* TestMem<O, S>::injectMarkerValue
  (Code* obj, Code* prop, Atom val, uint64 after, uint64 before, 
   r_exec::View::SyncMode syncMode, Code* group) {
  if (!obj || !prop)
    // We don't expect this, but sanity check.
    return NULL;

  Code *object = new r_exec::LObject(this);
  object->code(0) = Atom::Marker(r_exec::GetOpcode("mk.val"), 4); // Caveat: arity does not include the opcode.
  object->code(1) = Atom::RPointer(0); // obj
  object->code(2) = Atom::RPointer(1); // prop
  object->code(3) = val;
  object->code(4) = Atom::Float(1); // psln_thr.

  object->set_reference(0, obj);
  object->set_reference(1, prop);

  return injectFact(object, after, before, syncMode, group);
}

template<class O, class S> r_exec::View* TestMem<O, S>::injectMarkerValue
  (Code* obj, Code* prop, Code* val, uint64 after, uint64 before,
    r_exec::View::SyncMode syncMode, Code* group) {
  if (!obj || !prop)
    // We don't expect this, but sanity check.
    return NULL;

  Code *object = new r_exec::LObject(this);
  object->code(0) = Atom::Marker(r_exec::GetOpcode("mk.val"), 4); // Caveat: arity does not include the opcode.
  object->code(1) = Atom::RPointer(0); // obj
  object->code(2) = Atom::RPointer(1); // prop
  object->code(3) = Atom::RPointer(2); // val
  object->code(4) = Atom::Float(1); // psln_thr.

  object->set_reference(0, obj);
  object->set_reference(1, prop);
  object->set_reference(2, val);

  return injectFact(object, after, before, syncMode, group);
}

template<class O, class S> r_exec::View* TestMem<O, S>::injectFact
  (Code* object, uint64 after, uint64 before, r_exec::View::SyncMode syncMode,
   Code* group) {
  // Build a fact.
  Code* fact = new r_exec::Fact(object, after, before, 1, 1);

  // Build a view for the fact.
  r_exec::View *view = new r_exec::View(syncMode, after, 1, 1, group, NULL, fact);

  // Inject the view.
  ((_Mem *)this)->inject(view);
  return view;
}

template<class O, class S> void TestMem<O, S>::eject(Code *command) {
  uint16 function = (command->code(CMD_FUNCTION).atom >> 8) & 0x000000FF;

  if (function == set_speed_y_opcode_) {
    if (!speed_y_property_) {
      cout << "WARNING: Can't find the speed_y property" << endl;
      return;
    }
    if (!position_y_property_) {
      cout << "WARNING: Can't find the position_y property" << endl;
      return;
    }

    uint64 now = r_exec::Now();
    lastCommandTime_ = now;
    uint16 args_set_index = command->code(CMD_ARGS).asIndex();
    Code* obj = command->get_reference
      (command->code(args_set_index + 1).asIndex());
    if (!position_y_obj_) {
      // This is the first call. Remember the object whose speed we're setting.
      position_y_obj_ = obj;
      startTimeTickThread();
    }
    else {
      if (position_y_obj_ != obj)
        // For now, don't allow tracking the speed of multiple objects.
        return;
    }

    speed_y_ = command->code(args_set_index + 2).asFloat();
    // Let onTimeTick inject the new speed_y.
  }
  else if (function == move_y_plus_opcode_ ||
           function == move_y_minus_opcode_) {
    if (!position_property_) {
      cout << "WARNING: Can't find the position property" << endl;
      return;
    }
    for (int i = 0; i <= 9; ++i) {
      if (!yEnt_[i]) {
        cout << "WARNING: Can't find the entities y0, y1, etc." << endl;
        return;
      }
    }

    uint64 now = r_exec::Now();

    uint16 args_set_index = command->code(CMD_ARGS).asIndex();
    Code* obj = command->get_reference
      (command->code(args_set_index + 1).asIndex());
    if (!discretePositionObj_) {
      // This is the first call. Remember the object whose position we're setting.
      discretePositionObj_ = obj;
      discretePosition_ = yEnt_[0];
      startTimeTickThread();
      return;
    }
    else {
      if (discretePositionObj_ != obj)
        // For now, don't allow tracking multiple objects.
        return;
    }

    if (nextDiscretePosition_)
      // A previous move command is still pending execution.
      return;

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
    // Let onTimeTick inject the new position.
  }
}

template<class O, class S> void TestMem<O, S>::onTimeTick() {
  if (position_y_obj_) {
    // We are updating the continuous position_y_.
    uint64 now = r_exec::Now();
    if (now >= lastInjectTime_ + sampling_period_us * 8 / 10) {
      // Enough time has elapsed to inject a new position.
      if (lastInjectTime_ == 0) {
        // This is the first call, so leave the initial position.
      }
      else
        position_y_ += speed_y_ * (now - lastInjectTime_);

      lastInjectTime_ = now;
      // Inject the speed and position.
      // It seems that speed_y needs SYNC_HOLD for building models.
      injectMarkerValue
        (position_y_obj_, speed_y_property_, Atom::Float(speed_y_),
         now, now + sampling_period_us, r_exec::View::SYNC_HOLD);
      injectMarkerValue
        (position_y_obj_, position_y_property_, Atom::Float(position_y_),
          now, now + sampling_period_us);
    }
  }

  if (discretePositionObj_) {
    // We are updating the discretePosition_.
    uint64 now = r_exec::Now();
    if (now >= lastInjectTime_ + sampling_period_us * 8 / 10) {
      // Enough time has elapsed to inject another position.
      if (nextDiscretePosition_ &&
          now >= lastCommandTime_ + sampling_period_us * 5 / 10) {
        // Enough time has elapsed from the move command to update the position.
        discretePosition_ = nextDiscretePosition_;
        // Clear nextDiscretePosition_ to allow another move command.
        nextDiscretePosition_ = NULL;
      }

      lastInjectTime_ = now;
      injectMarkerValue
        (discretePositionObj_, position_property_, discretePosition_,
          now, now + sampling_period_us);

      const uint64 babbleStopTime_us = 2000000;
      const int maxBabblePosition = 9;
      if (now - Utils::GetTimeReference() < babbleStopTime_us) {
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

        r_exec::Fact* factCmd = new r_exec::Fact
          (cmd, now + sampling_period_us, now + 2*sampling_period_us, 1, 1);
        r_exec::Goal* goal = new r_exec::Goal(factCmd, get_self(), 1);
        injectFact(goal, now + sampling_period_us, now + sampling_period_us, primary_group_);
      }
    }
  }
}

template<class O, class S> void
TestMem<O, S>::startTimeTickThread() {
  if (reduction_core_count == 0 && time_core_count == 0)
    // We don't need a timeTickThread for diagnostic time.
    return;
  if (timeTickThread_)
    // We already started the thread.
    return;

  // We are running in real time. onDiagnosticTimeTick() will not be called.
  // Set up a timer thread to call onTimeTick().
  timeTickThread_ = Thread::New<_Thread>(timeTickRun, this);
}

template<class O, class S> thread_ret thread_function_call
TestMem<O, S>::timeTickRun(void *args) {
  TestMem<O, S>* self = (TestMem *)args;

  uint64 tickTime = r_exec::Now();
  // Call onTimeTick at the sampling period.
  while (self->state == RUNNING) {
    self->onTimeTick();

    tickTime += sampling_period_us;
    Thread::Sleep((tickTime - r_exec::Now()) / 1000);
  }

  thread_ret_val(0);
}

// Instatiate this template class as needed by main(). (Needs C++11.)
template class TestMem<r_exec::LObject, r_exec::MemStatic>;
template class TestMem<r_exec::LObject, r_exec::MemVolatile>;
