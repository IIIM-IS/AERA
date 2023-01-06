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

#include "structure_member.h"
#include "compiler.h"


namespace r_comp {

StructureMember::StructureMember() {
}

StructureMember::StructureMember(_Read r,
  std::string m,
  std::string p,
  Iteration i) : read_(r),
  name_(m),
  class_(p),
  iteration_(i) {

  if (read_ == &Compiler::read_any) type_ = ANY;
  else if (read_ == &Compiler::read_number) type_ = NUMBER;
  else if (read_ == &Compiler::read_timestamp) type_ = TIMESTAMP;
  else if (read_ == &Compiler::read_duration) type_ = DURATION;
  else if (read_ == &Compiler::read_boolean) type_ = BOOLEAN;
  else if (read_ == &Compiler::read_string) type_ = STRING;
  else if (read_ == &Compiler::read_node) type_ = NODE_ID;
  else if (read_ == &Compiler::read_device) type_ = DEVICE_ID;
  else if (read_ == &Compiler::read_function) type_ = FUNCTION_ID;
  else if (read_ == &Compiler::read_expression) type_ = ANY;
  else if (read_ == &Compiler::read_set) type_ = ReturnType::SET;
  else if (read_ == &Compiler::read_class) type_ = ReturnType::CLASS;
}

Class *StructureMember::get_class(Metadata *metadata) const {

  return class_ == "" ? NULL : &metadata->classes_.find(class_)->second;
}

ReturnType StructureMember::get_return_type() const {

  return type_;
}

bool StructureMember::used_as_expression() const {

  return iteration_ == I_EXPRESSION;
}

StructureMember::Iteration StructureMember::getIteration() const {

  return iteration_;
}

_Read StructureMember::read() const {

  return read_;
}

void StructureMember::write(word32 *storage) const {

  if (read_ == &Compiler::read_any)
    storage[0] = R_ANY;
  else if (read_ == &Compiler::read_number)
    storage[0] = R_NUMBER;
  else if (read_ == &Compiler::read_timestamp)
    storage[0] = R_TIMESTAMP;
  else if (read_ == &Compiler::read_duration)
    storage[0] = R_DURATION;
  else if (read_ == &Compiler::read_boolean)
    storage[0] = R_BOOLEAN;
  else if (read_ == &Compiler::read_string)
    storage[0] = R_STRING;
  else if (read_ == &Compiler::read_node)
    storage[0] = R_NODE;
  else if (read_ == &Compiler::read_device)
    storage[0] = R_DEVICE;
  else if (read_ == &Compiler::read_function)
    storage[0] = R_FUNCTION;
  else if (read_ == &Compiler::read_expression)
    storage[0] = R_EXPRESSION;
  else if (read_ == &Compiler::read_set)
    storage[0] = R_SET;
  else if (read_ == &Compiler::read_class)
    storage[0] = R_CLASS;
  uint32 offset = 1;
  storage[offset++] = type_;
  r_code::Write(storage + offset, class_);
  offset += r_code::GetSize(class_);
  storage[offset++] = iteration_;
  r_code::Write(storage + offset, name_);
}

void StructureMember::read(word32 *storage) {

  switch (storage[0]) {
  case R_ANY: read_ = &Compiler::read_any; break;
  case R_NUMBER: read_ = &Compiler::read_number; break;
  case R_TIMESTAMP: read_ = &Compiler::read_timestamp; break;
  case R_DURATION: read_ = &Compiler::read_duration; break;
  case R_BOOLEAN: read_ = &Compiler::read_boolean; break;
  case R_STRING: read_ = &Compiler::read_string; break;
  case R_NODE: read_ = &Compiler::read_node; break;
  case R_DEVICE: read_ = &Compiler::read_device; break;
  case R_FUNCTION: read_ = &Compiler::read_function; break;
  case R_EXPRESSION: read_ = &Compiler::read_expression; break;
  case R_SET: read_ = &Compiler::read_set; break;
  case R_CLASS: read_ = &Compiler::read_class; break;
  }
  uint32 offset = 1;
  type_ = (ReturnType)storage[offset++];
  r_code::Read(storage + offset, class_);
  offset += r_code::GetSize(class_);
  iteration_ = (Iteration)storage[offset++];
  r_code::Read(storage + offset, name_);
}

uint32 StructureMember::get_size() { // see segments.cpp for the RAM layout

  uint32 size = 3; // read ID, return type, iteration
  size += r_code::GetSize(class_);
  size += r_code::GetSize(name_);
  return size;
}
}
