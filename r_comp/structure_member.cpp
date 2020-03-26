//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ HUMANOBS - Replicode r_comp
//_/_/
//_/_/ Eric Nivel
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

#include "structure_member.h"
#include "compiler.h"


namespace r_comp {

StructureMember::StructureMember() {
}

StructureMember::StructureMember(_Read r,
  std::string m,
  std::string p,
  Iteration i) : _read(r),
  name(m),
  _class(p),
  iteration(i) {

  if (_read == &Compiler::read_any) type = ANY;
  else if (_read == &Compiler::read_number) type = NUMBER;
  else if (_read == &Compiler::read_timestamp) type = TIMESTAMP;
  else if (_read == &Compiler::read_boolean) type = BOOLEAN;
  else if (_read == &Compiler::read_string) type = STRING;
  else if (_read == &Compiler::read_node) type = NODE_ID;
  else if (_read == &Compiler::read_device) type = DEVICE_ID;
  else if (_read == &Compiler::read_function) type = FUNCTION_ID;
  else if (_read == &Compiler::read_expression) type = ANY;
  else if (_read == &Compiler::read_set) type = ReturnType::SET;
  else if (_read == &Compiler::read_class) type = ReturnType::CLASS;
}

Class *StructureMember::get_class(Metadata *metadata) const {

  return _class == "" ? NULL : &metadata->classes.find(_class)->second;
}

ReturnType StructureMember::get_return_type() const {

  return type;
}

bool StructureMember::used_as_expression() const {

  return iteration == I_EXPRESSION;
}

StructureMember::Iteration StructureMember::getIteration() const {

  return iteration;
}

_Read StructureMember::read() const {

  return _read;
}

void StructureMember::write(word32 *storage) const {

  if (_read == &Compiler::read_any)
    storage[0] = R_ANY;
  else if (_read == &Compiler::read_number)
    storage[0] = R_NUMBER;
  else if (_read == &Compiler::read_timestamp)
    storage[0] = R_TIMESTAMP;
  else if (_read == &Compiler::read_boolean)
    storage[0] = R_BOOLEAN;
  else if (_read == &Compiler::read_string)
    storage[0] = R_STRING;
  else if (_read == &Compiler::read_node)
    storage[0] = R_NODE;
  else if (_read == &Compiler::read_device)
    storage[0] = R_DEVICE;
  else if (_read == &Compiler::read_function)
    storage[0] = R_FUNCTION;
  else if (_read == &Compiler::read_expression)
    storage[0] = R_EXPRESSION;
  else if (_read == &Compiler::read_set)
    storage[0] = R_SET;
  else if (_read == &Compiler::read_class)
    storage[0] = R_CLASS;
  uint32 offset = 1;
  storage[offset++] = type;
  r_code::Write(storage + offset, _class);
  offset += r_code::GetSize(_class);
  storage[offset++] = iteration;
  r_code::Write(storage + offset, name);
}

void StructureMember::read(word32 *storage) {

  switch (storage[0]) {
  case R_ANY: _read = &Compiler::read_any; break;
  case R_NUMBER: _read = &Compiler::read_number; break;
  case R_TIMESTAMP: _read = &Compiler::read_timestamp; break;
  case R_BOOLEAN: _read = &Compiler::read_boolean; break;
  case R_STRING: _read = &Compiler::read_string; break;
  case R_NODE: _read = &Compiler::read_node; break;
  case R_DEVICE: _read = &Compiler::read_device; break;
  case R_FUNCTION: _read = &Compiler::read_function; break;
  case R_EXPRESSION: _read = &Compiler::read_expression; break;
  case R_SET: _read = &Compiler::read_set; break;
  case R_CLASS: _read = &Compiler::read_class; break;
  }
  uint32 offset = 1;
  type = (ReturnType)storage[offset++];
  r_code::Read(storage + offset, _class);
  offset += r_code::GetSize(_class);
  iteration = (Iteration)storage[offset++];
  r_code::Read(storage + offset, name);
}

uint32 StructureMember::get_size() { // see segments.cpp for the RAM layout

  uint32 size = 3; // read ID, return type, iteration
  size += r_code::GetSize(_class);
  size += r_code::GetSize(name);
  return size;
}
}
