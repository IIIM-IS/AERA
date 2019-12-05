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
  timeTickThreadEnabled_ = false;
  lastInjectTime_ = 0;
  speed_y_ = 0;
  position_y_ = 0;
  position_y_obj_ = 0;
  position_property_ = 0;
  position_y_property_ = 0;
  speed_y_property_ = 0;
  primary_group_ = 0,
  set_speed_y_opcode_ = 0xFFFF;
  move_y_plus_opcode_ = 0xFFFF;
  move_y_minus_opcode_ = 0xFFFF;
  last_command_opcode_ = 0xFFFF;

  y0_ent_ = 0;
  y1_ent_ = 0;
  y2_ent_ = 0;
  discrete_position_obj_ = 0;
  discrete_position_ = 0;
  next_discrete_position_ = 0;
}

template<class O, class S> TestMem<O, S>::~TestMem() {
  if (timeTickThread_) {
    timeTickThreadEnabled_ = false;
    delete timeTickThread_;
  }
}

template<class O, class S> bool TestMem<O, S>::load
  (std::vector<r_code::Code *> *objects, uint32 stdin_oid, uint32 stdout_oid,
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
  y0_ent_ = findObject(objects, "y0");
  y1_ent_ = findObject(objects, "y1");
  y2_ent_ = findObject(objects, "y2");

  return true;
}

template<class O, class S> Code* 
TestMem<O, S>::findObject
  (std::vector<r_code::Code *> *objects, const char* name) {
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

template<class O, class S> void TestMem<O, S>::injectMarkerValue
  (Code* obj, Code* prop, Atom val, uint64 after, uint64 before) {
  if (!obj || !prop)
    // We don't expect this, but sanity check.
    return;

  Code *object = new r_exec::LObject(this);
  object->code(0) = Atom::Marker(r_exec::GetOpcode("mk.val"), 4); // Caveat: arity does not include the opcode.
  object->code(1) = Atom::RPointer(0); // obj
  object->code(2) = Atom::RPointer(1); // prop
  object->code(3) = val;
  object->code(4) = Atom::Float(1); // psln_thr.

  object->set_reference(0, obj);
  object->set_reference(1, prop);

  injectFact(object, after, before, get_stdin());
}

template<class O, class S> void TestMem<O, S>::injectMarkerValue
  (Code* obj, Code* prop, Code* val, uint64 after, uint64 before) {
  if (!obj || !prop)
    // We don't expect this, but sanity check.
    return;

  Code *object = new r_exec::LObject(this);
  object->code(0) = Atom::Marker(r_exec::GetOpcode("mk.val"), 4); // Caveat: arity does not include the opcode.
  object->code(1) = Atom::RPointer(0); // obj
  object->code(2) = Atom::RPointer(1); // prop
  object->code(3) = Atom::RPointer(2); // val
  object->code(4) = Atom::Float(1); // psln_thr.

  object->set_reference(0, obj);
  object->set_reference(1, prop);
  object->set_reference(2, val);

  injectFact(object, after, before, get_stdin());
}

template<class O, class S> void TestMem<O, S>::injectFact
  (Code* object, uint64 after, uint64 before, Code* group) {
  // Build a fact.
  uint64 now = r_exec::Now();
  Code* fact = new r_exec::Fact(object, after, before, 1, 1);

  // Build a default view for the fact.
  r_exec::View *view = new r_exec::View
    (r_exec::View::SYNC_PERIODIC, now, 1, 1, group, NULL, fact);

  // Inject the view.
  ((_Mem *)this)->inject(view);
}

template<class O, class S> void TestMem<O, S>::eject(Code *command) {
  uint16 function = (command->code(CMD_FUNCTION).atom >> 8) & 0x000000FF;
  last_command_opcode_ = function;

  if (function == set_speed_y_opcode_) {
    if (!speed_y_property_) {
      cout << "WARNING: Can't find the speed_y property" << endl;
      return;
    }
    if (!position_y_property_) {
      cout << "WARNING: Can't find the position_y property" << endl;
      return;
    }

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
    // Inject the new speed as a fact.
    uint64 now = r_exec::Now();
    injectMarkerValue
      (position_y_obj_, speed_y_property_, Atom::Float(speed_y_),
       now, now + sampling_period_us);
  }
  else if (function == move_y_plus_opcode_ ||
           function == move_y_minus_opcode_) {
    if (!position_property_) {
      cout << "WARNING: Can't find the position property" << endl;
      return;
    }
    if (!y0_ent_ || !y1_ent_ || !y2_ent_) {
      cout << "WARNING: Can't find the entities y0, y1 and y2" << endl;
      return;
    }

    uint16 args_set_index = command->code(CMD_ARGS).asIndex();
    Code* obj = command->get_reference
      (command->code(args_set_index + 1).asIndex());
    if (!discrete_position_obj_) {
      // This is the first call. Remember the object whose position we're setting.
      discrete_position_obj_ = obj;
      discrete_position_ = y0_ent_;
      next_discrete_position_ = y0_ent_;
      startTimeTickThread();
    }
    else {
      if (discrete_position_obj_ != obj)
        // For now, don't allow tracking multiple objects.
        return;
    }

    // The discrete_position_ will become next_discrete_position_ at the
    // next sampling period.
    if (function == move_y_plus_opcode_) {
      if (discrete_position_ == y0_ent_)
        next_discrete_position_ = y1_ent_;
      else if (discrete_position_ == y1_ent_)
        next_discrete_position_ = y2_ent_;
    }
    else if (function == move_y_minus_opcode_) {
      if (discrete_position_ == y2_ent_)
        next_discrete_position_ = y1_ent_;
      else if (discrete_position_ == y1_ent_)
        next_discrete_position_ = y0_ent_;
    }
    // Let onTimeTick inject the new position.
  }
}

template<class O, class S> void TestMem<O, S>::onTimeTick() {
  if (position_y_obj_) {
    // We are updating the continuous position_y_.
    uint64 now = r_exec::Now();
    if (now >= lastInjectTime_ + sampling_period_us) {
      // Enough time has elapsed to inject a new position.
      if (lastInjectTime_ == 0) {
        // This is the first call, so leave the initial position.
      }
      else
        position_y_ += speed_y_ * (now - lastInjectTime_);

      lastInjectTime_ = now;
      injectMarkerValue
        (position_y_obj_, position_y_property_, Atom::Float(position_y_),
          now, now + sampling_period_us);
    }
  }

  if (discrete_position_obj_) {
    // We are updating the discrete_position_.
    uint64 now = r_exec::Now();
    if (now >= lastInjectTime_ + sampling_period_us) {
      // Enough time has elapsed to inject another position.
      lastInjectTime_ = now;
      injectMarkerValue
        (discrete_position_obj_, position_property_, discrete_position_,
          now, now + sampling_period_us);

      // This will be injected the next time.
      discrete_position_ = next_discrete_position_;

#if 0 // Babble.
      uint16 nextCommand;
      if (discrete_position_ == y0_ent_)
        nextCommand = move_y_plus_opcode_;
      else if (discrete_position_ == y1_ent_)
        nextCommand = (last_command_opcode_ == move_y_plus_opcode_ ?
                       move_y_plus_opcode_ : move_y_minus_opcode_);
      else // discrete_position_ == y0_ent_
        nextCommand = move_y_minus_opcode_;

      // Inject (fact (goal (fact (cmd ...))))
      Code *cmd = new r_exec::LObject(this);
      cmd->code(0) = Atom::Marker(r_exec::GetOpcode("cmd"), 3);
      cmd->code(1) = Atom::DeviceFunction(nextCommand);
      cmd->code(2) = Atom::IPointer(4);
      cmd->code(3) = Atom::Float(1); // psln_thr.
      cmd->code(4) = Atom::Set(1);
      cmd->code(5) = Atom::RPointer(0); // obj
      cmd->set_reference(0, discrete_position_obj_);

      r_exec::Fact* factCmd = new r_exec::Fact
        (cmd, now, now + sampling_period_us, 1, 1);
      r_exec::Goal* goal = new r_exec::Goal(factCmd, get_self(), 1);
      cout << "Debug inject command " << (nextCommand == move_y_plus_opcode_ ?
        "move_y_plus" : "move_y_minus") << endl;
      injectFact(goal, now, now, get_stdin());
#endif
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

  // We are running in real time. onDiagnosticTimeUpdate() will not be called.
  // Set up a timer thread to call onTimeTick().
  timeTickThreadEnabled_ = true;
  timeTickThread_ = Thread::New<_Thread>(timeTickRun, this);
}

template<class O, class S> thread_ret thread_function_call
TestMem<O, S>::timeTickRun(void *args) {
  TestMem<O, S>* self = (TestMem *)args;

  // Call onTimeTick at a faster rate than the sampling period.
  while (self->timeTickThreadEnabled_) {
    self->onTimeTick();
    Thread::Sleep((sampling_period_us / 10) / 1000);
  }

  thread_ret_val(0);
}

// Instatiate this template class as needed by main(). (Needs C++11.)
template class TestMem<r_exec::LObject, r_exec::MemStatic>;
template class TestMem<r_exec::LObject, r_exec::MemVolatile>;
