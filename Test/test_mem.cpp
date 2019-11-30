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
  set_speed_y_opcode_ = 0xFFFF;
  move_y_plus_opcode_ = 0xFFFF;
  move_y_minus_opcode_ = 0xFFFF;
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
  object->code(1) = Atom::RPointer(0); // object
  object->code(2) = Atom::RPointer(1); // prooerty
  object->code(3) = val;
  object->code(4) = Atom::Float(1); // psln_thr.

  object->set_reference(0, obj);
  object->set_reference(1, prop);

  // Build a fact.
  uint64 now = r_exec::Now();
  Code* fact = new r_exec::Fact(object, after, before, 1, 1);

  // Build a default view for the fact.
  r_exec::View *view = new r_exec::View();
  const uint32 arity = VIEW_ARITY; // reminder: opcode not included in the arity.
  uint16 extent_index = arity + 1;

  view->code(VIEW_OPCODE) = Atom::SSet(r_exec::View::ViewOpcode, arity);
  view->code(VIEW_SYNC) = Atom::Float(View::SYNC_PERIODIC);
  view->code(VIEW_IJT) = Atom::IPointer(extent_index);  // iptr to injection time.
  view->code(VIEW_SLN) = Atom::Float(1);      // sln.
  view->code(VIEW_RES) = Atom::Float(1);      // res is set to 1 upr of the destination group.
  view->code(VIEW_HOST) = Atom::RPointer(0);  // stdin/stdout is the only reference.
  view->code(VIEW_ORG) = Atom::Nil();         // orgin.

  Utils::SetTimestamp(&view->code(extent_index), now);

  view->references[0] = get_stdin();

  // Inject the view.
  view->set_object(fact);
  ((_Mem *)this)->inject(view);
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

    uint16 args_set_index = command->code(CMD_ARGS).asIndex();
    Code* obj = command->get_reference
      (command->code(args_set_index + 1).asIndex());
    if (!position_y_obj_) {
      // This is the first call. Remember the object whose speed we're setting.
      position_y_obj_ = obj;

      if (!(reduction_core_count == 0 && time_core_count == 0)) {
        // We are running in real time. onDiagnosticTimeUpdate() will not be called.
        // Set up a timer thread to call onTimeTick().
        if (!timeTickThread_) {
          timeTickThreadEnabled_ = true;
          timeTickThread_ = Thread::New<_Thread>(timeTickRun, this);
        }
      }
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
}

template<class O, class S> void TestMem<O, S>::onTimeTick() {
  if (!position_y_obj_)
    // We need to wait for the first call to eject set_speed_y.
    return;

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

template<class O, class S> thread_ret thread_function_call
TestMem<O, S>::timeTickRun(void *args) {
  TestMem<O, S>* self = (TestMem *)args;

  // Call onTimeTick at twice the rate of the sampling period.
  while (self->timeTickThreadEnabled_) {
    self->onTimeTick();
    Thread::Sleep((sampling_period_us / 2) / 1000);
  }

  thread_ret_val(0);
}

// Instatiate this template class as needed by main(). (Needs C++11.)
template class TestMem<r_exec::LObject, r_exec::MemStatic>;
template class TestMem<r_exec::LObject, r_exec::MemVolatile>;
