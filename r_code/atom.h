//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ HUMANOBS - Replicode r_Code
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

#ifndef r_code_atom_h
#define r_code_atom_h

#include <string>
#include "../submodules/CoreLibrary/CoreLibrary/types.h"

#undef THIS

using namespace core;

namespace r_code {

// Opcodes on 12 bits.
// Indices on 12 bits.
// Element count on 8 bits.
// To define bigger constructs (e.g. large matrices), define hooks to RAM (and derive classes from Object).
class dll_export Atom {
public:
  /**
   * Atom::TraceContext holds the indentation level and other context info
   * for the trace method. Before iterating over Atom objects (which may have
   * different indentation levels or other details), create an
   * Atom::TraceContext and pass it to Atom::trace.
   */
  class dll_export TraceContext {
  public:
    uint8 members_to_go_;
    uint8 timestamp_data_;
    uint8 string_data_;
    uint8 char_count_;
    uint64 timestamp_high_;

    TraceContext();
    void write_indents(std::ostream& out);
  };

  typedef enum {
    NIL = 0x80,
    BOOLEAN_ = 0x81,
    WILDCARD = 0x82,
    T_WILDCARD = 0x83,
    I_PTR = 0x84, // internal pointer.
    R_PTR = 0x85, // reference pointer.
    VL_PTR = 0x86, // binding map value pointer.
    IPGM_PTR = 0x87, // r_exec internal: index of data of a tpl arg held by an ipgm.
    IN_OBJ_PTR = 0x88, // r_exec internal: index of data held by an input object.
    VALUE_PTR = 0x89, // r_exec internal: index of data held by the overlay's value array.
    PROD_PTR = 0x8A, // r_exec internal: index of data held by the overlay's production array.
    OUT_OBJ_PTR = 0x8B, // r_exec internal: index of data held by a newly produced object.
    D_IN_OBJ_PTR = 0x8C, // r_exec internal: index of data held by an object referenced by an input object.
    ASSIGN_PTR = 0x8D, // r_exec internal: index of a hlp variable and to be assigned index of an expression that produces the value.
    CODE_VL_PTR = 0x8E, // pointer to a value at an index in the same code array.
    THIS = 0x90, // this pointer.
    VIEW = 0x91,
    MKS = 0x92,
    VWS = 0x93,
    NODE = 0xA0,
    DEVICE = 0xA1,
    DEVICE_FUNCTION = 0xA2,
    C_PTR = 0xC0, // chain pointer.
    SET = 0xC1,
    S_SET = 0xC2, // structured set.
    OBJECT = 0xC3,
    MARKER = 0xC4,
    OPERATOR = 0xC5,
    STRING = 0xC6,
    TIMESTAMP = 0xC7,
    GROUP = 0xC8,
    INSTANTIATED_PROGRAM = 0xC9,
    INSTANTIATED_CPP_PROGRAM = 0xCA,
    INSTANTIATED_INPUT_LESS_PROGRAM = 0xCB,
    INSTANTIATED_ANTI_PROGRAM = 0xCC,
    COMPOSITE_STATE = 0xCD,
    MODEL = 0xCE,
    NULL_PROGRAM = 0xCF
  }Type;

  // encoders
  static Atom Float(float32 f); // IEEE 754 32 bits encoding; shifted by 1 to the right (loss of precison).
  static Atom PlusInfinity();
  static Atom MinusInfinity();
  static Atom UndefinedFloat();
  static Atom Nil();
  static Atom Boolean(bool value);
  static Atom UndefinedBoolean();
  static Atom Wildcard(uint16 opcode = 0x00);
  static Atom TailWildcard();
  static Atom IPointer(uint16 index);
  static Atom RPointer(uint16 index);
  static Atom VLPointer(uint16 index);
  static Atom IPGMPointer(uint16 index);
  static Atom InObjPointer(uint8 input_index, uint16 index); // input_index: index of the input view; index: index of data in the object's code.
  static Atom DInObjPointer(uint8 relative_index, uint16 index); // relative_index: index of an in-obj-ptr in the program's (patched) code; index: index of data in the referenced object code.
  static Atom OutObjPointer(uint16 index);
  static Atom ValuePointer(uint16 index);
  static Atom ProductionPointer(uint16 index);
  static Atom AssignmentPointer(uint8 variable_index, uint16 index);
  static Atom CodeVLPointer(uint16 index, uint16 cast_opcode = 0x0FFF);
  static Atom This();
  static Atom View();
  static Atom Mks();
  static Atom Vws();
  static Atom Node(uint8 node_id);
  static Atom UndefinedNode();
  static Atom Device(uint8 node_id, uint8 class_id, uint8 dev_id);
  static Atom UndefinedDevice();
  static Atom DeviceFunction(uint16 opcode);
  static Atom UndefinedDeviceFunction();
  static Atom CPointer(uint8 element_count);
  static Atom SSet(uint16 opcode, uint8 element_count);
  static Atom Set(uint8 element_count);
  static Atom Object(uint16 opcode, uint8 arity);
  static Atom Marker(uint16 opcode, uint8 arity);
  static Atom Operator(uint16 opcode, uint8 arity);
  static Atom String(uint8 character_count);
  static Atom UndefinedString();
  static Atom Timestamp();
  static Atom UndefinedTimestamp();
  static Atom InstantiatedProgram(uint16 opcode, uint8 arity);
  static Atom Group(uint16 opcode, uint8 arity);
  static Atom InstantiatedCPPProgram(uint16 opcode, uint8 arity);
  static Atom InstantiatedAntiProgram(uint16 opcode, uint8 arity);
  static Atom InstantiatedInputLessProgram(uint16 opcode, uint8 arity);
  static Atom CompositeState(uint16 opcode, uint8 arity);
  static Atom Model(uint16 opcode, uint8 arity);

  static Atom NullProgram(bool take_past_inputs);
  static Atom RawPointer(void *pointer);

  Atom(uint32 a = 0xFFFFFFFF);
  ~Atom();

  Atom &operator =(const Atom& a);
  bool operator ==(const Atom& a) const;
  bool operator !=(const Atom& a) const;
  bool operator !() const;
  operator size_t () const;

  uint32 atom_;

  // decoders
  bool isUndefined() const;
  uint8 getDescriptor() const;
  bool isStructural() const;
  bool isFloat() const;
  bool readsAsNil() const; // returns true for all undefined values.
  float32 asFloat() const;
  bool asBoolean() const;
  bool isBooleanTrue() const;
  bool isBooleanFalse() const;
  uint16 asIndex() const; // applicable to internal, view, reference,
                                      // and value pointers.
  uint8 asInputIndex() const; // applicable to IN_OBJ_PTR.
  uint8 asRelativeIndex() const; // applicable to D_IN_OBJ_PTR.
  uint16 asOpcode() const;
  uint16 asCastOpcode() const; // applicable to CODE_VL_PTR.
  uint8 getAtomCount() const; // arity of operators and
                                      // objects/markers/structured sets,
                                      // number of atoms in pointers chains,
                                      // number of blocks of characters in
                                      // strings.
  uint8 getNodeID() const; // applicable to nodes and devices.
  uint8 getClassID() const; // applicable to devices.
  uint8 getDeviceID() const; // applicable to devices.
  uint8 asAssignmentIndex() const;

  bool takesPastInputs() const; // applicable to NULL_PROGRAM.
  template<class C> C *asRawPointer() const { return (C *)atom_; }

  void trace(TraceContext& context, std::ostream& out) const;
};

/**
 * Add to the names for the opcode. This is searched by GetOpcodeName.
 * because we don't want to export a DLL function with std::string.)</param>
 * \param opcode The opcode.
 * \param name The opcode name, which is copied.
 */
void __declspec(dllexport) AddOpcodeName(uint16 opcode, const char* name);

/**
 * Get the name or set of names of the opcode. (The same opcode can be
 * re - used for different purposes.)
 * \param opcode The opcode.
 * \return The opcode name, or a set of names separated by a slash. If not found, return "unknown".
 */
std::string GetOpcodeName(uint16 opcode);
}


#include "atom.inline.cpp"


#endif
