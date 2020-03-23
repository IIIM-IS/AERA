//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ HUMANOBS - Replicode r_exec
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

#include "overlay.h"
#include "mem.h"


#define MAX_VALUE_SIZE 128

namespace r_exec {

Overlay::Overlay() : _Object(), invalidated(0) {

  values.as_std()->resize(MAX_VALUE_SIZE); // MAX_VALUE_SIZE is the limit; if the array is resized later on, some contexts with data==VALUE_ARRAY may point to invalid adresses: case of embedded contexts with both data==VALUE_ARRAY.
}

Overlay::Overlay(Controller *c, bool load_code) : _Object(), controller(c), value_commit_index(0), code(NULL), invalidated(0) {

  values.as_std()->resize(128);
  if (load_code)
    this->load_code();
}

Overlay::~Overlay() {

  if (code)
    delete[] code;
}

inline Code *Overlay::get_core_object() const {

  return controller->get_core_object();
}

void Overlay::load_code() {

  if (code)
    delete[] code;

  Code *object = get_core_object();
  // copy the original pgm/hlp code.
  code_size = object->code_size();
  code = new r_code::Atom[code_size];
  memcpy(code, &object->code(0), code_size * sizeof(r_code::Atom));
}

void Overlay::reset() {

  memcpy(code, &getObject()->get_reference(0)->code(0), code_size * sizeof(r_code::Atom)); // restore code to prisitne copy.
}

void Overlay::rollback() {

  Code *object = get_core_object();
  Atom *original_code = &object->code(0);
  for (uint16 i = 0; i < patch_indices.size(); ++i) // upatch code.
    code[patch_indices[i]] = original_code[patch_indices[i]];
  patch_indices.clear();

  if (value_commit_index != values.size()) { // shrink the values down to the last commit index.

    if (value_commit_index > 0)
      values.as_std()->resize(value_commit_index);
    else
      values.as_std()->clear();
    value_commit_index = values.size();
  }
}

void Overlay::commit() {

  patch_indices.clear();
  value_commit_index = values.size();
}

void Overlay::patch_code(uint16 index, Atom value) {

  code[index] = value;
  patch_indices.push_back(index);
}

uint16 Overlay::get_last_patch_index() {

  return patch_indices.size();
}

void Overlay::unpatch_code(uint16 patch_index) {

  Code *object = get_core_object();
  Atom *original_code = &object->code(0);
  for (uint16 i = patch_index; i < patch_indices.size(); ++i)
    code[patch_indices[i]] = original_code[patch_indices[i]];
  patch_indices.resize(patch_index);
}

Overlay *Overlay::reduce(r_exec::View *input) {

  return NULL;
}

r_code::Code *Overlay::build_object(Atom head) const {

  return _Mem::Get()->build_object(head);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Controller::Controller(r_code::View *view) : _Object(), invalidated(0), activated(0), view(view) {

  if (!view)
    return;

  switch (getObject()->code(0).getDescriptor()) {
  case Atom::INSTANTIATED_PROGRAM:
  case Atom::INSTANTIATED_INPUT_LESS_PROGRAM:
  case Atom::INSTANTIATED_ANTI_PROGRAM:
    tsc = Utils::GetTimestamp<Code>(getObject(), IPGM_TSC);
    break;
  case Atom::INSTANTIATED_CPP_PROGRAM:
    tsc = Utils::GetTimestamp<Code>(getObject(), ICPP_PGM_TSC);
    break;
  }
}

Controller::~Controller() {
}

void Controller::set_view(View *view) {

  this->view = view;
}

void Controller::_take_input(r_exec::View *input) { // called by groups at update and injection time.

  if (is_alive() && !input->object->is_invalidated())
    take_input(input);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

OController::OController(r_code::View *view) : Controller(view) {
}

OController::~OController() {
}
}
