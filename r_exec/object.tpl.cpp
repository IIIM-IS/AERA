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



namespace r_exec {

template<class C, class U> Object<C, U>::Object() : C(), hash_value_(0), invalidated_(0) {
}

template<class C, class U> Object<C, U>::Object(r_code::Mem *mem) : C(), hash_value_(0), invalidated_(0) {

  set_oid(UNDEFINED_OID);
}

template<class C, class U> Object<C, U>::~Object() {

  invalidate();
}

template<class C, class U> bool Object<C, U>::is_invalidated() {

  return invalidated_ == 1;
}

template<class C, class U> bool Object<C, U>::invalidate() {

  if (invalidated_)
    return true;
  invalidated_ = 1;//std::cout<<std::dec<<get_oid()<<" invalidated\n";

  acq_views();
  views_.clear();
  rel_views();

  if (code(0).getDescriptor() == Atom::MARKER) {

    for (uint16 i = 0; i < references_size(); ++i)
      get_reference(i)->remove_marker(this);
  }

  if (is_registered())
    r_code::Mem::Get()->delete_object(this);

  return false;
}

template<class C, class U> void Object<C, U>::compute_hash_value() {

  hash_value_ = code(0).asOpcode() << 20; // 12 bits for the opcode.
  hash_value_ |= (code_size() & 0x00000FFF) << 8; // 12 bits for the code size.
  hash_value_ |= references_size() & 0x000000FF; // 8 bits for the reference set size.
}

template<class C, class U> float32 Object<C, U>::get_psln_thr() {

  psln_thrCS_.enter();
  float32 r = code(code(0).getAtomCount()).asFloat(); // psln is always the last member of an object.
  psln_thrCS_.leave();
  return r;
}

template<class C, class U> void Object<C, U>::mod(uint16 member_index, float32 value) {

  if (member_index != code_size() - 1)
    return;
  float32 v = code(member_index).asFloat() + value;
  if (v < 0)
    v = 0;
  else if (v > 1)
    v = 1;

  psln_thrCS_.enter();
  code(member_index) = Atom::Float(v);
  psln_thrCS_.leave();
}

template<class C, class U> void Object<C, U>::set(uint16 member_index, float32 value) {

  if (member_index != code_size() - 1)
    return;

  psln_thrCS_.enter();
  code(member_index) = Atom::Float(value);
  psln_thrCS_.leave();
}

template<class C, class U> View *Object<C, U>::get_view(r_code::Code *group, bool lock) {

  if (lock)
    acq_views();

  r_code::_View probe;
  probe.references_[0] = group;

  std::unordered_set<r_code::_View *, r_code::_View::Hash, r_code::_View::Equal>::const_iterator v = views_.find(&probe);
  if (v != views_.end()) {

    if (lock)
      rel_views();
    return (r_exec::View *)*v;
  }

  if (lock)
    rel_views();
  return NULL;
}
}
