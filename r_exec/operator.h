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

  uint16 get_children_count() const { return implementation_->get_children_count(); }
  Context get_child(uint16 index) const { return Context(implementation_->get_child_new(index)); }

  Context operator *() const { return Context(implementation_->dereference_new()); }
  Context &operator =(const Context &c) {

    // Copy the existing implementation before deleting it.
    _Context* copy = c.get_implementation()->clone();
    delete implementation_;
    implementation_ = copy;
    return *this;
  }

  bool operator ==(const Context &c) const { return implementation_->equal(c.get_implementation()); }
  bool operator !=(const Context &c) const { return !implementation_->equal(c.get_implementation()); }

  Atom &operator [](uint16 i) const { return implementation_->get_atom(i); }

  void setAtomicResult(Atom a) const { implementation_->setAtomicResult(a); }
  void setTimestampResult(Timestamp t) const { implementation_->setTimestampResult(t); }
  void setDurationResult(std::chrono::microseconds d) const { implementation_->setDurationResult(d); }
  void setCompoundResultHead(Atom a) const { implementation_->setCompoundResultHead(a); }
  void addCompoundResultPart(Atom a) const { implementation_->addCompoundResultPart(a); }

  void trace(std::ostream& out) const { return implementation_->trace(out); }
};

bool red(const Context &context); // executive-dependent.

bool syn(const Context &context);

class Operator {
private:
  static r_code::resized_vector<Operator> Operators_; // indexed by opcodes.

  bool(*operator_)(const Context &);
  bool(*overload_)(const Context &);
public:
  static void Register(uint16 opcode, bool(*op)(const Context &)); // first, register std operators; next register user-defined operators (may be registered as overloads).
  static Operator Get(uint16 opcode) { return Operators_[opcode]; }
  Operator() : operator_(NULL), overload_(NULL) {}
  Operator(bool(*o)(const Context &)) : operator_(o), overload_(NULL) {}
  ~Operator() {}

  void setOverload(bool(*o)(const Context &)) { overload_ = o; }

  bool operator ()(const Context &context) const {
    if (operator_(context))
      return true;
    if (overload_)
      return overload_(context);
    return false;
  }

  bool is_red() const { return operator_ == red; }
  bool is_syn() const { return operator_ == syn; }
};

// std operators ////////////////////////////////////////

bool now(const Context &context);

bool rnd(const Context &context);

bool equ(const Context &context);
bool neq(const Context &context);
bool gtr(const Context &context);
bool lsr(const Context &context);
bool gte(const Context &context);
bool lse(const Context &context);

bool add(const Context &context);
bool sub(const Context &context);
bool mul(const Context &context);
bool div(const Context &context);

bool dis(const Context &context);

bool ln(const Context &context);
bool exp(const Context &context);
bool log(const Context &context);
bool e10(const Context &context);

bool ins(const Context &context); // executive-dependent.

bool fvw(const Context &context); // executive-dependent.

bool is_sim(const Context &context);
bool minimum(const Context &context);
bool maximum(const Context &context);
}


#endif
