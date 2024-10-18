//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2023 Jeff Thompson
//_/_/ Copyright (c) 2018-2023 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2023 Icelandic Institute for Intelligent Machines
//_/_/ http://www.iiim.is
//_/_/ 
//_/_/ Copyright (c) 2010-2012 Eric Nivel
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

#include "init.h"
#include "binding_map.h"
#include "../r_code/replicode_defs.h"
#include "operator.h"
#include "factory.h"
#include "cpp_programs.h"
#include "../r_code/utils.h"
#include <math.h>


namespace r_exec {

template<class O, class S> MemExec<O, S>::MemExec() : S() {
}

template<class O, class S> MemExec<O, S>::~MemExec() {

  if (state_ == RUNNING)
    S::stop();
  deleted_ = true;
  objects_.clear();
}

////////////////////////////////////////////////////////////////

template<class O, class S> r_code::Code *MemExec<O, S>::build_object(r_code::SysObject *source) const {

  Atom head = source->code_[0];
  switch (head.getDescriptor()) {
  case Atom::GROUP:
    return new Group(source);
  default: {
    uint16 opcode = head.asOpcode();
    if (opcode == Opcodes::Fact)
      return new Fact(source);
    else if (opcode == Opcodes::AntiFact)
      return new AntiFact(source);
    else if (opcode == Opcodes::Goal)
      return new Goal(source);
    else if (opcode == Opcodes::Pred)
      return new Pred(source);
    else if (opcode == Opcodes::ICst)
      return new ICST(source);
    else if (opcode == Opcodes::MkRdx)
      return new MkRdx(source);
    else if (opcode == Opcodes::MkActChg ||
      opcode == Opcodes::MkHighAct ||
      opcode == Opcodes::MkHighSln ||
      opcode == Opcodes::MkLowAct ||
      opcode == Opcodes::MkLowRes ||
      opcode == Opcodes::MkLowSln ||
      opcode == Opcodes::MkNew ||
      opcode == Opcodes::MkSlnChg ||
      opcode == Opcodes::Success ||
      opcode == Opcodes::Perf)
      return new r_code::LocalObject(source);
    else
      return new O(source);
  }
  }
}

template<class O, class S> r_code::Code *MemExec<O, S>::_build_object(Atom head) const {

  r_code::Code *object = new O();
  object->code(0) = head;
  return object;
}

template<class O, class S> r_code::Code *MemExec<O, S>::build_object(Atom head) const {

  r_code::Code *object;
  switch (head.getDescriptor()) {
  case Atom::GROUP:
    object = new Group();
    break;
  default: {
    uint16 opcode = head.asOpcode();
    if (opcode == Opcodes::Fact)
      object = new Fact();
    else if (opcode == Opcodes::AntiFact)
      object = new AntiFact();
    else if (opcode == Opcodes::Pred)
      object = new Pred();
    else if (opcode == Opcodes::Goal)
      object = new Goal();
    else if (opcode == Opcodes::ICst)
      object = new ICST();
    else if (opcode == Opcodes::MkRdx)
      object = new MkRdx();
    else if (opcode == Opcodes::MkActChg ||
      opcode == Opcodes::MkHighAct ||
      opcode == Opcodes::MkHighSln ||
      opcode == Opcodes::MkLowAct ||
      opcode == Opcodes::MkLowRes ||
      opcode == Opcodes::MkLowSln ||
      opcode == Opcodes::MkNew ||
      opcode == Opcodes::MkSlnChg ||
      opcode == Opcodes::Success ||
      opcode == Opcodes::Perf)
      object = new r_code::LocalObject();
    else if (O::RequiresPacking())
      object = new r_code::LocalObject(); // temporary sand box for assembling code; will be packed into an O at injection time.
    else
      object = new O();
    break;
  }
  }

  object->code(0) = head;
  return object;
}

////////////////////////////////////////////////////////////////

template<class O, class S> r_code::Code *MemExec<O, S>::check_existence(r_code::Code *object) {

  if (object->code(0).getDescriptor() == Atom::GROUP) // groups are always new.
    return object;

  O *_object;
  if (O::RequiresPacking()) // false if LObject, true for network-aware objects.
    _object = O::Pack(object, this); // non compact form will be deleted (P<> in view) if not an instance of O; compact forms are left unchanged.
  else
    _object = (O *)object;

  return _object;
}

template<class O, class S> void MemExec<O, S>::inject(O *object, View *view) {

  view->set_object(object);
  S::inject_new_object(view);
}
}
