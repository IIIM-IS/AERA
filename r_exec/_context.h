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

#ifndef _context_h
#define _context_h

#include "../r_code/atom.h"
#include "overlay.h"

namespace r_exec {

// Base class for evaluation contexts.
// Subclasses: IPGMContext and HLPContext.
// _Context * wrapped in Context, the latter used by operators.
class dll_export _Context {
protected:
  Overlay *const overlay_; // the overlay where the evaluation is performed; NULL when the context is dereferenced outside the original pgm or outside the value array.
  Atom *code_; // the object's code, or the code in value array, or the view's code when the context is dereferenced from Atom::VIEW.
  uint16 index_; // in the code;

  typedef enum { // indicates whether the context refers to:
    STEM = 0, // - the pgm/hlp being reducing inputs;
    REFERENCE = 1, // - a reference to another object;
    VIEW = 2, // - a view;
    MKS = 3, // - the mks of an object;
    VWS = 4, // - the vws of an object;
    VALUE_ARRAY = 5, // - code in the overlay's value array.
    BINDING_MAP = 6, // - values of a imdl/icst.
    UNDEFINED = 7
  }Data;
  Data data_;

  _Context(Atom *code, uint16 index, Overlay *overlay, Data data) : code_(code), index_(index), overlay_(overlay), data_(data) {}
public:
  virtual _Context *clone() = 0;

  virtual bool equal(const _Context *c) const = 0;

  virtual Atom &get_atom(uint16 i) const = 0;

  virtual uint16 get_object_code_size() const = 0;

  virtual uint16 getChildrenCount() const = 0;

  /**
   * Call getChild and return a new allocated copy of the child. The caller is responsible to delete it.
   */
  virtual _Context *getChild_new(uint16 index) const = 0;

  /**
   * Dereference this and return a new allocated copy. The caller is responsible to delete it.
   */
  virtual _Context *dereference_new() const = 0;

  void commit() const { overlay_->commit(); }
  void rollback() const { overlay_->rollback(); }
  void patch_code(uint16 location, Atom value) const { overlay_->patch_code(location, value); }
  void unpatch_code(uint16 patch_index) const { overlay_->unpatch_code(patch_index); }
  uint16 get_last_patch_index() const { return overlay_->get_last_patch_index(); }

  uint16 setAtomicResult(Atom a) const;
  uint16 setTimestampResult(Timestamp t) const;
  uint16 setCompoundResultHead(Atom a) const;
  uint16 addCompoundResultPart(Atom a) const;

  void trace(std::ostream& out) const;
};
}


#endif
