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

#ifndef operator_h
#define operator_h

#include "../r_code/object.h"

#include "_context.h"


namespace r_exec {

// Wrapper class for evaluation contexts.
// Template operator functions is not an option since some operators are defined in usr_operators.dll.
class dll_export Context {
private:
  _Context *implementation_;
public:
  Context(_Context *implementation) : implementation_(implementation) {}
  ~Context() { delete implementation_; }

  _Context *get_implementation() const { return implementation_; }

  uint16 getChildrenCount() const { return implementation_->getChildrenCount(); }
  Context getChild(uint16 index) const { return Context(implementation_->getChild_new(index)); }

  Context operator *() const { return Context(implementation_->dereference_new()); }
  Context &operator =(const Context &c) {

    // Copy the existing implementation before deleting it.
    _Context* copy = implementation_->clone(c.get_implementation());
    delete implementation_;
    implementation_ = copy;
    return *this;
  }

  bool operator ==(const Context &c) const { return implementation_->equal(c.get_implementation()); }
  bool operator !=(const Context &c) const { return !implementation_->equal(c.get_implementation()); }

  Atom &operator [](uint16 i) const { return implementation_->get_atom(i); }

  uint16 setAtomicResult(Atom a) const { return implementation_->setAtomicResult(a); }
  uint16 setTimestampResult(Timestamp t) const { return implementation_->setTimestampResult(t); }
  uint16 setCompoundResultHead(Atom a) const { return implementation_->setCompoundResultHead(a); }
  uint16 addCompoundResultPart(Atom a) const { return implementation_->addCompoundResultPart(a); }

  void trace(std::ostream& out) const { return implementation_->trace(out); }
};

bool red(const Context &context, uint16 &index); // executive-dependent.

bool syn(const Context &context, uint16 &index);

class Operator {
private:
  static r_code::vector<Operator> Operators_; // indexed by opcodes.

  bool(*operator_)(const Context &, uint16 &);
  bool(*overload_)(const Context &, uint16 &);
public:
  static void Register(uint16 opcode, bool(*op)(const Context &, uint16 &)); // first, register std operators; next register user-defined operators (may be registered as overloads).
  static Operator Get(uint16 opcode) { return Operators_[opcode]; }
  Operator() : operator_(NULL), overload_(NULL) {}
  Operator(bool(*o)(const Context &, uint16 &)) : operator_(o), overload_(NULL) {}
  ~Operator() {}

  void setOverload(bool(*o)(const Context &, uint16 &)) { overload_ = o; }

  bool operator ()(const Context &context, uint16 &index) const {
    if (operator_(context, index))
      return true;
    if (overload_)
      return overload_(context, index);
    return false;
  }

  bool is_red() const { return operator_ == red; }
  bool is_syn() const { return operator_ == syn; }
};

// std operators ////////////////////////////////////////

bool now(const Context &context, uint16 &index);

bool rnd(const Context &context, uint16 &index);

bool equ(const Context &context, uint16 &index);
bool neq(const Context &context, uint16 &index);
bool gtr(const Context &context, uint16 &index);
bool lsr(const Context &context, uint16 &index);
bool gte(const Context &context, uint16 &index);
bool lse(const Context &context, uint16 &index);

bool add(const Context &context, uint16 &index);
bool sub(const Context &context, uint16 &index);
bool mul(const Context &context, uint16 &index);
bool div(const Context &context, uint16 &index);

bool dis(const Context &context, uint16 &index);

bool ln(const Context &context, uint16 &index);
bool exp(const Context &context, uint16 &index);
bool log(const Context &context, uint16 &index);
bool e10(const Context &context, uint16 &index);

bool ins(const Context &context, uint16 &index); // executive-dependent.

bool fvw(const Context &context, uint16 &index); // executive-dependent.

bool is_sim(const Context &context, uint16 &index);
bool minimum(const Context &context, uint16 &index);
bool maximum(const Context &context, uint16 &index);
}


#endif
