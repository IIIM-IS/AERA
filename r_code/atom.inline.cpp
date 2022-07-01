//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2022 Jeff Thompson
//_/_/ Copyright (c) 2018-2022 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2022 Icelandic Institute for Intelligent Machines
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

#include <iostream>
namespace r_code {

inline Atom Atom::Float(float32 f) {

  float32 _f = f;
  uint32 a = *reinterpret_cast<uint32 *>(&_f);
  return Atom(a >> 1);
}

inline Atom Atom::PlusInfinity() {

  return Atom(0x3FC00000);
}

inline Atom Atom::MinusInfinity() {

  return Atom(0x7FC00000);
}

inline Atom Atom::UndefinedFloat() {

  return Atom(0xFFFFFFF);
}

inline Atom Atom::Nil() {

  return Atom(NIL << 24);
}

inline Atom Atom::Boolean(bool value) {

  return Atom((BOOLEAN_ << 24) + value);
}

inline Atom Atom::UndefinedBoolean() {

  return Atom(0x81FFFFFF);
}

inline Atom Atom::Wildcard(uint16 opcode) {

  return Atom((WILDCARD << 24) + ((opcode & 0x0FFF) << 8));
}

inline Atom Atom::TailWildcard() {

  return Atom(T_WILDCARD << 24);
}

inline Atom Atom::IPointer(uint16 index) {

  return Atom((I_PTR << 24) + (index & 0x0FFF));
}

inline Atom Atom::VLPointer(uint16 index) {

  return Atom((VL_PTR << 24) + (index & 0x0FFF));
}

inline Atom Atom::RPointer(uint16 index) {

  return Atom((R_PTR << 24) + (index & 0x0FFF));
}

inline Atom Atom::IPGMPointer(uint16 index) {

  return Atom((IPGM_PTR << 24) + (index & 0x0FFF));
}

inline Atom Atom::InObjPointer(uint8 input_index, uint16 index) {

  return Atom((IN_OBJ_PTR << 24) + (input_index << 12) + (index & 0x0FFF));
}

inline Atom Atom::DInObjPointer(uint8 relative_index, uint16 index) {

  return Atom((D_IN_OBJ_PTR << 24) + (relative_index << 12) + (index & 0x0FFF));
}

inline Atom Atom::OutObjPointer(uint16 index) {

  return Atom((OUT_OBJ_PTR << 24) + (index & 0x0FFF));
}

inline Atom Atom::ValuePointer(uint16 index) {

  return Atom((VALUE_PTR << 24) + (index & 0x0FFF));
}

inline Atom Atom::ProductionPointer(uint16 index) {

  return Atom((PROD_PTR << 24) + (index & 0x0FFF));
}

inline Atom Atom::AssignmentPointer(uint8 variable_index, uint16 index) {

  return Atom((ASSIGN_PTR << 24) + (variable_index << 16) + (index & 0x0FFF));
}

inline Atom Atom::CodeVLPointer(uint16 index, uint16 cast_opcode) {

  return Atom((CODE_VL_PTR << 24) + ((cast_opcode & 0x0FFF) << 12) + (index & 0x0FFF));
}

inline Atom Atom::This() {

  return Atom(THIS << 24);
}

inline Atom Atom::View() {

  return Atom(VIEW << 24);
}

inline Atom Atom::Mks() {

  return Atom(MKS << 24);
}

inline Atom Atom::Vws() {

  return Atom(VWS << 24);
}

inline Atom Atom::SSet(uint16 opcode, uint8 element_count) {

  return Atom((S_SET << 24) + ((opcode & 0x0FFF) << 8) + element_count);
}

inline Atom Atom::Set(uint8 element_count) {

  return Atom((SET << 24) + element_count);
}

inline Atom Atom::CPointer(uint8 element_count) {

  return Atom((C_PTR << 24) + element_count);
}

inline Atom Atom::Object(uint16 opcode, uint8 arity) {

  return Atom((OBJECT << 24) + ((opcode & 0x0FFF) << 8) + arity);
}

inline Atom Atom::Marker(uint16 opcode, uint8 arity) {

  return Atom((MARKER << 24) + ((opcode & 0x0FFF) << 8) + arity);
}

inline Atom Atom::Operator(uint16 opcode, uint8 arity) {

  return Atom((OPERATOR << 24) + ((opcode & 0x0FFF) << 8) + arity);
}

inline Atom Atom::Node(uint8 node_id) {

  return Atom((NODE << 24) + (node_id << 8));
}

inline Atom Atom::UndefinedNode() {

  return Atom(0xA0FFFFFF);
}

inline Atom Atom::Device(uint8 node_id, uint8 class_id, uint8 dev_id) {

  return Atom((DEVICE << 24) + (node_id << 16) + (class_id << 8) + dev_id);
}

inline Atom Atom::UndefinedDevice() {

  return Atom(0xA1FFFFFF);
}

inline Atom Atom::DeviceFunction(uint16 opcode) {

  return Atom((DEVICE_FUNCTION << 24) + (opcode << 8));
}

inline Atom Atom::UndefinedDeviceFunction() {

  return Atom(0xA2FFFFFF);
}

inline Atom Atom::String(uint8 character_count) {

  uint8 blocks = character_count / 4;
  if (character_count % 4)
    ++blocks;
  return Atom((STRING << 24) + (blocks << 8) + character_count);
}

inline Atom Atom::UndefinedString() {

  return Atom(0xC6FFFFFF);
}

inline Atom Atom::Timestamp() {

  return Atom(TIMESTAMP << 24);
}

inline Atom Atom::UndefinedTimestamp() {

  return Atom(0xC7FFFFFF);
}

inline Atom Atom::Duration() {
  return Atom(DURATION << 24);
}

inline Atom Atom::InstantiatedProgram(uint16 opcode, uint8 arity) {

  return Atom((INSTANTIATED_PROGRAM << 24) + ((opcode & 0x0FFF) << 8) + arity);
}

inline Atom Atom::Group(uint16 opcode, uint8 arity) {

  return Atom((GROUP << 24) + ((opcode & 0x0FFF) << 8) + arity);
}

inline Atom Atom::InstantiatedCPPProgram(uint16 opcode, uint8 arity) {

  return Atom((INSTANTIATED_CPP_PROGRAM << 24) + ((opcode & 0x0FFF) << 8) + arity);
}

inline Atom Atom::InstantiatedAntiProgram(uint16 opcode, uint8 arity) {

  return Atom((INSTANTIATED_ANTI_PROGRAM << 24) + ((opcode & 0x0FFF) << 8) + arity);
}

inline Atom Atom::InstantiatedInputLessProgram(uint16 opcode, uint8 arity) {

  return Atom((INSTANTIATED_INPUT_LESS_PROGRAM << 24) + ((opcode & 0x0FFF) << 8) + arity);
}

inline Atom Atom::CompositeState(uint16 opcode, uint8 arity) {

  return Atom((COMPOSITE_STATE << 24) + ((opcode & 0x0FFF) << 8) + arity);
}

inline Atom Atom::Model(uint16 opcode, uint8 arity) {

  return Atom((MODEL << 24) + ((opcode & 0x0FFF) << 8) + arity);
}

inline Atom Atom::NullProgram(bool take_past_inputs) {

  return Atom((NULL_PROGRAM << 24) + (take_past_inputs ? 1 : 0));
}

// RawPointer is not used. In any case, only define it for ARCH_32. If we want to
// define it for ARCH_64, we need to change the byte code to use two uint32 code elements.
#if defined ARCH_32
inline Atom Atom::RawPointer(void *pointer) {

  return Atom((uint32)pointer);
}
#endif

inline Atom::Atom(uint32 a) : atom_(a) {
}

inline Atom::~Atom() {
}

inline Atom &Atom::operator =(const Atom& a) {

  atom_ = a.atom_;
  return *this;
}

inline bool Atom::operator ==(const Atom& a) const {

  return atom_ == a.atom_;
}

inline bool Atom::operator !=(const Atom& a) const {

  return atom_ != a.atom_;
}

inline bool Atom::operator !() const {

  return isUndefined();
}

inline Atom::operator size_t () const {

  return (size_t)atom_;
}

inline bool Atom::isUndefined() const {

  return atom_ == 0xFFFFFFFF;
}

inline uint8 Atom::getDescriptor() const {

  return atom_ >> 24;
}

inline bool Atom::isStructural() const {

  return ((atom_ & 0xC0000000) == 0xC0000000 || (atom_ & 0xD0000000) == 0xD0000000);
}

inline bool Atom::isFloat() const {

  return atom_ >> 31 == 0;
}

inline bool Atom::readsAsNil() const {

  return atom_ == 0x80000000 ||
    atom_ == 0x3FFFFFFF ||
    atom_ == 0x81FFFFFF ||
    atom_ == 0xC1000000 ||
    atom_ == 0xA0FFFFFF ||
    atom_ == 0xA1FFFFFF ||
    atom_ == 0xA2FFFFFF ||
    atom_ == 0xC6FFFFFF;
}

inline float32 Atom::asFloat() const {

  uint32 _f = atom_ << 1;
  return *reinterpret_cast<const float32 *>(&_f);
}

inline bool Atom::asBoolean() const {

  return atom_ & 0x000000FF;
}

inline bool Atom::isBooleanTrue() const { return getDescriptor() == BOOLEAN_ && asBoolean(); }

inline bool Atom::isBooleanFalse() const { return getDescriptor() == BOOLEAN_ && !asBoolean(); }

inline uint16 Atom::asIndex() const {

  return atom_ & 0x00000FFF;
}

inline uint8 Atom::asInputIndex() const {

  return (uint8)((atom_ & 0x000FF000) >> 12);
}

inline uint8 Atom::asRelativeIndex() const {

  return (uint8)((atom_ & 0x000FF000) >> 12);
}

inline uint16 Atom::asOpcode() const {

  return (atom_ >> 8) & 0x00000FFF;
}

inline uint16 Atom::asCastOpcode() const {

  return (uint16)((atom_ & 0x00FFF000) >> 12);
}

inline uint8 Atom::getNodeID() const {

  return (uint8)((atom_ & 0x00FF0000) >> 16);
}

inline uint8 Atom::getClassID() const {

  return (uint8)((atom_ & 0x0000FF00) >> 8);
}

inline uint8 Atom::getDeviceID() const {

  return (uint8)(atom_ & 0x000000FF);
}

inline uint8 Atom::asAssignmentIndex() const {

  return (uint8)((atom_ & 0x00FF0000) >> 16);
}

inline uint8 Atom::getAtomCount() const {

  switch (getDescriptor()) {
  case SET:
  case OBJECT:
  case MARKER:
  case C_PTR:
  case OPERATOR:
  case INSTANTIATED_PROGRAM:
  case INSTANTIATED_CPP_PROGRAM:
  case INSTANTIATED_INPUT_LESS_PROGRAM:
  case INSTANTIATED_ANTI_PROGRAM:
  case COMPOSITE_STATE:
  case MODEL:
  case GROUP:
  case S_SET: return atom_ & 0x000000FF;
  case STRING: return (atom_ & 0x0000FF00) >> 8;
  case TIMESTAMP: return 2;
  case DURATION: return 2;
  default:
    return 0;
  }
}

inline bool Atom::takesPastInputs() const {

  return atom_ & 0x00000001;
}
}
