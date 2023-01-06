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

#include "overlay.h"
#include "mem.h"

using namespace std::chrono;
using namespace r_code;

#define MAX_VALUE_SIZE 128

namespace r_exec {

Overlay::Overlay() : _Object(), invalidated_(0) {

  values_.as_std()->resize(MAX_VALUE_SIZE); // MAX_VALUE_SIZE is the limit; if the array is resized later on, some contexts with data==VALUE_ARRAY may point to invalid adresses: case of embedded contexts with both data==VALUE_ARRAY.
}

Overlay::Overlay(Controller *c, bool load_code) : _Object(), controller_(c), value_commit_index_(0), code_(NULL), invalidated_(0) {

  values_.as_std()->resize(128);
  if (load_code)
    this->load_code();
}

Overlay::~Overlay() {

  if (code_)
    delete[] code_;
}

inline Code *Overlay::get_core_object() const {

  return controller_->get_core_object();
}

void Overlay::load_code() {

  if (code_)
    delete[] code_;

  Code *object = get_core_object();
  // copy the original pgm/hlp code.
  code_size_ = object->code_size();
  code_ = new r_code::Atom[code_size_];
  memcpy(code_, &object->code(0), code_size_ * sizeof(r_code::Atom));
}

void Overlay::reset() {

  memcpy(code_, &get_object()->get_reference(0)->code(0), code_size_ * sizeof(r_code::Atom)); // restore code to prisitne copy.
}

void Overlay::rollback() {

  Code *object = get_core_object();
  Atom *original_code = &object->code(0);
  for (uint16 i = 0; i < patch_indices_.size(); ++i) // upatch code.
    code_[patch_indices_[i]] = original_code[patch_indices_[i]];
  patch_indices_.clear();

  if (value_commit_index_ != values_.size()) { // shrink the values down to the last commit index.

    if (value_commit_index_ > 0)
      values_.as_std()->resize(value_commit_index_);
    else
      values_.as_std()->clear();
    value_commit_index_ = values_.size();
  }
}

void Overlay::commit() {

  patch_indices_.clear();
  value_commit_index_ = values_.size();
}

void Overlay::patch_code(uint16 index, Atom value) {

  code_[index] = value;
  patch_indices_.push_back(index);
}

uint16 Overlay::get_last_patch_index() {

  return patch_indices_.size();
}

void Overlay::unpatch_code(uint16 patch_index) {

  Code *object = get_core_object();
  Atom *original_code = &object->code(0);
  for (uint16 i = patch_index; i < patch_indices_.size(); ++i)
    code_[patch_indices_[i]] = original_code[patch_indices_[i]];
  patch_indices_.resize(patch_index);
}

Overlay *Overlay::reduce(r_exec::View *input) {

  return NULL;
}

r_code::Code *Overlay::build_object(Atom head) const {

  return _Mem::Get()->build_object(head);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Controller::Controller(_View *view) : _Object(), invalidated_(0), activated_(0), view_(view) {

  if (!view)
    return;

  switch (get_object()->code(0).getDescriptor()) {
  case Atom::INSTANTIATED_PROGRAM:
  case Atom::INSTANTIATED_INPUT_LESS_PROGRAM:
  case Atom::INSTANTIATED_ANTI_PROGRAM:
    time_scope_ = Utils::GetDuration<Code>(get_object(), IPGM_TSC);
    break;
  case Atom::INSTANTIATED_CPP_PROGRAM:
    time_scope_ = Utils::GetDuration<Code>(get_object(), ICPP_PGM_TSC);
    break;
  }
}

Controller::~Controller() {
}

void Controller::set_view(View *view) {

  view_ = view;
}

void Controller::_take_input(r_exec::View *input) { // called by groups at update and injection time.

  if (is_alive() && !input->object_->is_invalidated())
    take_input(input);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

OController::OController(_View *view) : Controller(view) {
}

OController::~OController() {
}
}
