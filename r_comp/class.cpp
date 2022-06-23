//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2022 Jeff Thompson
//_/_/ Copyright (c) 2018-2022 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2022 Icelandic Institute for Intelligent Machines
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

#include "class.h"
#include "segments.h"

using namespace std;
using namespace r_code;

namespace r_comp {

const char *Class::Expression = "xpr";
const char *Class::Type = "type";

bool Class::has_offset() const {

  switch (atom_.getDescriptor()) {
  case Atom::OBJECT:
  case Atom::GROUP:
  case Atom::INSTANTIATED_PROGRAM:
  case Atom::INSTANTIATED_CPP_PROGRAM:
  case Atom::S_SET:
  case Atom::MARKER: return true;
  default: return false;
  }
}

Class::Class(ReturnType t) : type(t), str_opcode("undefined") {
}

Class::Class(Atom atom,
  std::string str_opcode,
  vector<StructureMember> r,
  ReturnType t) : atom_(atom),
  str_opcode(str_opcode),
  things_to_read(r),
  type(t),
  use_as(StructureMember::I_CLASS) {
}

bool Class::is_pattern(Metadata *metadata) const {

  return (metadata->classes_.find("ptn")->second.atom_ == atom_) || (metadata->classes_.find("|ptn")->second.atom_ == atom_);
}

bool Class::is_fact(Metadata *metadata) const {

  return (metadata->classes_.find("fact")->second.atom_ == atom_) || (metadata->classes_.find("|fact")->second.atom_ == atom_);
}

bool Class::get_member_index(Metadata *metadata, std::string &name, uint16 &index, Class *&p) const {

  for (uint16 i = 0; i < things_to_read.size(); ++i)
    if (things_to_read[i].name_ == name) {

      index = (has_offset() ? i + 1 : i); // in expressions the lead r-atom is at 0; in objects, members start at 1
      if (things_to_read[i].used_as_expression()) // the class is: [::a-class]
        p = NULL;
      else
        p = things_to_read[i].get_class(metadata);
      return true;
    }
  return false;
}

std::string Class::get_member_name(uint32 index) {

  return things_to_read[has_offset() ? index - 1 : index].name_;
}

ReturnType Class::get_member_type(const uint16 index) {

  return things_to_read[has_offset() ? index - 1 : index].get_return_type();
}

Class *Class::get_member_class(Metadata *metadata, const std::string &name) {

  for (uint16 i = 0; i < things_to_read.size(); ++i)
    if (things_to_read[i].name_ == name)
      return things_to_read[i].get_class(metadata);
  return NULL;
}

void Class::write(word32 *storage) {

  storage[0] = atom_.atom_;
  r_code::Write(storage + 1, str_opcode);
  uint32 offset = 1 + r_code::GetSize(str_opcode);
  storage[offset++] = type;
  storage[offset++] = use_as;
  storage[offset++] = things_to_read.size();
  for (uint32 i = 0; i < things_to_read.size(); ++i) {

    things_to_read[i].write(storage + offset);
    offset += things_to_read[i].get_size();
  }
}

void Class::read(word32 *storage) {

  atom_ = storage[0];
  r_code::Read(storage + 1, str_opcode);
  uint32 offset = 1 + r_code::GetSize(str_opcode);
  type = (ReturnType)storage[offset++];
  use_as = (StructureMember::Iteration)storage[offset++];
  uint32 member_count = storage[offset++];
  for (uint32 i = 0; i < member_count; ++i) {

    StructureMember m;
    m.read(storage + offset);
    things_to_read.push_back(m);
    offset += m.get_size();
  }
}

uint32 Class::get_size() { // see segments.cpp for the RAM layout

  uint32 size = 4; // atom, return type, usage, number of members
  size += r_code::GetSize(str_opcode);
  for (uint32 i = 0; i < things_to_read.size(); ++i)
    size += things_to_read[i].get_size();
  return size;
}
}
