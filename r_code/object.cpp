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

#include "object.h"
#include "replicode_defs.h"

#include <iostream>

using namespace std;

namespace r_code {

#ifdef WITH_DETAIL_OID
// Start with a big number so it doesn't look like SysObject::detail_oid_.
static uint64 last_SysView_detail_oid = 1000;
#endif
SysView::SysView() {
#ifdef WITH_DETAIL_OID
  detail_oid_ = ++last_SysView_detail_oid;
  if (detail_oid_ == 0)
    int set_breakpoint_here = 1;
#endif
}

SysView::SysView(_View *source) {

#ifdef WITH_DETAIL_OID
  detail_oid_ = ++last_SysView_detail_oid;
  if (detail_oid_ == 0)
    int set_breakpoint_here = 1;
#endif
  for (uint32 i = 0; i < VIEW_CODE_MAX_SIZE; ++i)
    code_[i] = source->code(i);

  for (uint32 i = 0; i < 2; ++i) // to get the right size in Image::add_object().
    if (source->references_[i])
      references_.push_back(0);
}

void SysView::write(word32 *data) {

  data[0] = code_.size();
  data[1] = references_.size();
  uint32 i = 0;
  for (; i < code_.size(); ++i)
    data[2 + i] = code_[i].atom_;
  for (uint32 j = 0; j < references_.size(); ++j)
    data[2 + i + j] = references_[j];
}

void SysView::read(word32 *data) {

  uint32 code_size = data[0];
  uint32 reference_set_size = data[1];
  uint32 i;
  uint32 j;
  for (i = 0; i < code_size; ++i)
    code_.push_back(Atom(data[2 + i]));
  for (j = 0; j < reference_set_size; ++j)
    references_.push_back(data[2 + i + j]);
}

uint32 SysView::get_size() const {

  return 2 + code_.size() + references_.size();
}

void SysView::trace(std::ostream& out) const {

  out << " code size: " << code_.size() << std::endl;
  out << " reference set size: " << references_.size() << std::endl;
  out << "---code---" << std::endl;
  Atom::TraceContext context;
  for (uint32 i = 0; i < code_.size(); ++i) {

    code_[i].trace(context, out);
    out << std::endl;
  }
  out << "---reference set---" << std::endl;
  for (uint32 i = 0; i < references_.size(); ++i)
    out << references_[i] << std::endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32 SysObject::lastOID_ = 0;

SysObject::SysObject() : oid_(lastOID_++) {
#ifdef WITH_DETAIL_OID
  detail_oid_ = 0;
#endif
}

SysObject::SysObject(Code *source) {

  uint32 i;
  for (i = 0; i < source->code_size(); ++i)
    code_[i] = source->code(i);

  unordered_set<_View *, _View::Hash, _View::Equal>::const_iterator v;
  source->acq_views();
  for (i = 0, v = source->views_.begin(); v != source->views_.end(); ++i, ++v)
    views_[i] = new SysView(*v);
  source->rel_views();

  oid_ = source->get_oid();
#ifdef WITH_DETAIL_OID
  detail_oid_ = source->get_detail_oid();
#endif

  for (i = 0; i < source->references_size(); ++i) // to get the right size in Image::add_object().
    references_.push_back(0);
}

SysObject::~SysObject() {

  for (uint32 i = 0; i < views_.size(); ++i)
    delete views_[i];
}

void SysObject::write(word32 *data) {

  data[0] = oid_;
  data[1] = code_.size();
  data[2] = references_.size();
  data[3] = markers_.size();
  data[4] = views_.size();
  uint32 i;
  uint32 j;
  uint32 k;
  uint32 l;
  for (i = 0; i < code_.size(); ++i)
    data[5 + i] = code_[i].atom_;
  for (j = 0; j < references_.size(); ++j)
    data[5 + i + j] = references_[j];
  for (k = 0; k < markers_.size(); ++k)
    data[5 + i + j + k] = markers_[k];
  uint32 offset = 0;
  for (l = 0; l < views_.size(); ++l) {

    views_[l]->write(data + 5 + i + j + k + offset);
    offset += views_[l]->get_size();
  }
}

void SysObject::read(word32 *data) {

  oid_ = data[0];
  uint32 code_size = data[1];
  uint32 reference_set_size = data[2];
  uint32 marker_set_size = data[3];
  uint32 view_set_size = data[4];
  uint32 i;
  uint32 j;
  uint32 k;
  uint32 l;
  for (i = 0; i < code_size; ++i)
    code_.push_back(Atom(data[5 + i]));
  for (j = 0; j < reference_set_size; ++j)
    references_.push_back(data[5 + i + j]);
  for (k = 0; k < marker_set_size; ++k)
    markers_.push_back(data[5 + i + j + k]);
  uint32 offset = 0;
  for (l = 0; l < view_set_size; ++l) {

    SysView *v = new SysView();
    v->read(data + 5 + i + j + k + offset);
    views_.push_back(v);
    offset += v->get_size();
  }
}

uint32 SysObject::get_size() {

  uint32 view_set_size = 0;
  for (uint32 i = 0; i < views_.size(); ++i)
    view_set_size += views_[i]->get_size();
  return 5 + code_.size() + references_.size() + markers_.size() + view_set_size;
}

void SysObject::trace(std::ostream& out) const {

  out << "\n---object---\n";
  out << oid_ << std::endl;
  out << "code size: " << code_.size() << std::endl;
  out << "reference set size: " << references_.size() << std::endl;
  out << "marker set size: " << markers_.size() << std::endl;
  out << "view set size: " << views_.size() << std::endl;
  out << "\n---code---\n";
  uint32 i;
  Atom::TraceContext context;
  for (i = 0; i < code_.size(); ++i) {

    out << i << " ";
    code_[i].trace(context, out);
    out << std::endl;
  }
  out << "\n---reference set---\n";
  for (i = 0; i < references_.size(); ++i)
    out << i << " " << references_[i] << std::endl;
  out << "\n---marker set---\n";
  for (i = 0; i < markers_.size(); ++i)
    out << i << " " << markers_[i] << std::endl;
  out << "\n---view set---\n";
  for (uint32 k = 0; k < views_.size(); ++k) {

    out << "view[" << k << "]" << std::endl;
    out << "reference set size: " << views_[k]->references_.size() << std::endl;
    out << "-code-" << std::endl;
    uint32 j;
    for (j = 0; j < views_[k]->code_.size(); ++i, ++j) {

      out << j << " ";
      views_[k]->code_[j].trace(context, out);
      out << std::endl;
    }
    out << "-reference set-" << std::endl;
    for (j = 0; j < views_[k]->references_.size(); ++i, ++j)
      out << j << " " << views_[k]->references_[j] << std::endl;
  }
}

void SysObject::trace() const { trace(std::cout); }

void Code::r_trace(ostream& out) const {
  trace(out);

  if (references_size() < 1)
    // Done recursing.
    return;

  auto obj = get_reference(0);
  if (obj->code_size() <= 2)
    // Assume this is an ent or ont.
    return;

  switch (obj->code(0).getDescriptor()) {
  case Atom::MODEL:
  case Atom::COMPOSITE_STATE:
    // Don't trace these big structures.
    return;
  }

  out << endl;
  obj->r_trace(out);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Mem *Mem::singleton_ = NULL;

Mem::Mem() {

  singleton_ = this;
}

Mem *Mem::Get() {

  return singleton_;
}
}
