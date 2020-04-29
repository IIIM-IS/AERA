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

#ifndef structure_member_h
#define structure_member_h

#include <string>

#include "../r_code/image.h"
#include "../r_code/atom.h"

namespace r_comp {

class Class;
class Compiler;

typedef enum {
  ANY = 0,
  NUMBER = 1,
  TIMESTAMP = 2,
  SET = 3,
  BOOLEAN = 4,
  STRING = 5,
  NODE_ID = 6,
  DEVICE_ID = 7,
  FUNCTION_ID = 8,
  CLASS = 9
}ReturnType;

typedef bool (Compiler::*_Read)(bool &, bool, const Class *, uint16, uint16 &, bool); // reads from the stream and writes in an object.

class Metadata;
class StructureMember {
public:
  typedef enum {
    I_CLASS = 0, // iterate using the class to enumerate elements.
    I_EXPRESSION = 1, // iterate using the class in read_expression.
    I_SET = 2, // iterate using the class in read_set.
    I_DCLASS // iterate using read_class in read_set.
  }Iteration;
private:
  typedef enum {
    R_ANY = 0,
    R_NUMBER = 1,
    R_TIMESTAMP = 2,
    R_BOOLEAN = 3,
    R_STRING = 4,
    R_NODE = 5,
    R_DEVICE = 6,
    R_FUNCTION = 7,
    R_EXPRESSION = 8,
    R_SET = 9,
    R_CLASS = 10
  }ReadID; // used for serialization
  _Read read_;
  ReturnType type_;
  std::string class_; // when r==read_set or read_expression, _class specifies the class of said set/expression if one is targeted in particular; otherwise _class=="".
  Iteration iteration_; // indicates how to use the _class to read the elements of the set: as an enumeration of types, as a class of expression, or as an enumeration of types to use for reading subsets.
public:
  std::string name_; // unused for anything but set/object/marker classes.
  StructureMember();
  StructureMember(_Read r, // compiler's read function.
    std::string m, // member's name.
    std::string p = "", // class name of return type if r==Compiler::read_expression or name of the structure to enumerate elements if r==Compiler::read_set.
    Iteration i = I_CLASS); // specified only if r==Compiler::read_set.
  Class *get_class(Metadata *metadata) const;
  ReturnType get_return_type() const;
  bool used_as_expression() const;
  Iteration getIteration() const;
  _Read read() const;

  void write(word32 *storage) const;
  void read(word32 *storage);
  uint32 get_size();
};
}


#endif
