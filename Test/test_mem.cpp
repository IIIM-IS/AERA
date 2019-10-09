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
  lastInjectTime_ = 0;
  speed_y_ = 0;
  position_y_ = 0;
  obj_ = 0;
  position_y_property_ = 0;
  set_speed_y_opcode_ = 0xFFFF;
}

template<class O, class S> bool TestMem<O, S>::load
  (std::vector<r_code::Code *> *objects, uint32 stdin_oid, uint32 stdout_oid,
   uint32 self_oid) {
  // Call the method in the parent class.
  if (!r_exec::Mem<O, S>::load(objects, stdin_oid, stdout_oid, self_oid))
    return false;

  // Find the opcodes we need.
  set_speed_y_opcode_ = r_exec::GetOpcode("set_speed_y");
  if (set_speed_y_opcode_ == 0xFFFF)
    cout << "WARNING: Can't find the set_speed_y opcode" << endl;

  // Find the OIDs of ontology objects we need. (Imitate the code in main().)
  uint32 position_y_oid = 0;
  for (UNORDERED_MAP<uint32, std::string>::const_iterator it = r_exec::Seed.object_names.symbols.begin();
    it != r_exec::Seed.object_names.symbols.end(); ++it) {
    if (it->second == "position_y")
      position_y_oid = it->first;
  }

  if (position_y_oid == 0)
    cout << "WARNING: Can't find the position_y OID" << endl;

  // Find the objects we need. (Imitate the code in 
  for (uint32 i = 0; i < objects->size(); ++i) {
    Code *object = (*objects)[i];

    if (object->get_oid() == position_y_oid)
      position_y_property_ = (*objects)[i];
  }

  if (!position_y_property_)
    cout << "WARNING: Can't find the position_y property" << endl;

  return true;
}

template<class O, class S> void TestMem<O, S>::inject_position_y
  (Code* obj, float position_y, uint64 after, uint64 before) {
  if (!position_y_property_)
    // We don't expect this, but sanity check.
    return;

  Code *object = new r_exec::LObject(this);
  object->code(0) = Atom::Marker(r_exec::GetOpcode("mk.val"), 4); // Caveat: arity does not include the opcode.
  object->code(1) = Atom::RPointer(0);
  object->code(2) = Atom::RPointer(1);
  object->code(3) = Atom::Float(position_y);
  object->code(4) = Atom::Float(1); // psln_thr.

  object->set_reference(0, obj);
  object->set_reference(1, position_y_property_);

  // Build a fact.
  const uint64 sampling_period = 100000;
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
    uint16 args_set_index = command->code(CMD_ARGS).asIndex();
    Code* obj = command->get_reference
      (command->code(args_set_index + 1).asIndex());
    if (!obj_) {
      // This is the first call. Remember the object whose speed we're setting.
      obj_ = obj;

      if (!(reduction_core_count == 0 && time_core_count == 0)) {
        // We are running in real time. onDiagnosticTimeUpdate() will not be called.
        // TODO: Set up a timer thread to call onTimeTick().
      }
    }
    else {
      if (obj_ != obj)
        // For now, don't allow tracking the speed of multiple objects.
        return;
    }

    speed_y_ = command->code(args_set_index + 2).asFloat();
  }
}

template<class O, class S> void TestMem<O, S>::onTimeTick() {
  if (!obj_)
    // We need to wait for the first call to eject set_speed_y.
    return;

  const uint64 sampling_period = 100000;
  uint64 now = r_exec::Now();
  if (now >= lastInjectTime_ + sampling_period) {
    // Enough time has elapsed to inject a new position.
    if (lastInjectTime_ == 0) {
      // This is the first call, so leave the initial position.
    }
    else
      position_y_ += speed_y_ * (now - lastInjectTime_);

    lastInjectTime_ = now;
    inject_position_y(obj_, position_y_, now, now + sampling_period);
  }
}

// Instatiate this template class as needed by main(). (Needs C++11.)
template class TestMem<r_exec::LObject, r_exec::MemStatic>;
template class TestMem<r_exec::LObject, r_exec::MemVolatile>;
