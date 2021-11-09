//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2021 Jeff Thompson
//_/_/ Copyright (c) 2018-2021 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2021 Icelandic Institute for Intelligent Machines
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

#ifndef r_code_utils_h
#define r_code_utils_h

#include "atom.h"
#include "../submodules/CoreLibrary/CoreLibrary/base.h"
#include "../submodules/CoreLibrary/CoreLibrary/utils.h"


namespace r_code {

// For use in STL containers.
template<class C> class PHash {
public:
  size_t operator ()(P<C> c) const {
    return (size_t)(C *)c;
  }
};

// Debugging facility.
class NullOStream :
  public std::ostream {
public:
  NullOStream() : std::ostream(NULL) {}
  template<typename T> NullOStream& operator <<(T &t) {
    return *this;
  }
};

class Code;

class dll_export Utils {
private:
  static Timestamp TimeReference; // starting time.
  static std::chrono::microseconds BasePeriod;
  static float32 FloatTolerance;
  static std::chrono::microseconds TimeTolerance;
public:
  static Timestamp GetTimeReference();
  static std::chrono::microseconds GetBasePeriod();
  static uint32 GetFloatTolerance();
  static std::chrono::microseconds GetTimeTolerance();
  static void SetReferenceValues(std::chrono::microseconds base_period, float32 float_tolerance, std::chrono::microseconds time_tolerance);
  static void SetTimeReference(Timestamp time_reference);

  static bool Equal(float32 l, float32 r);
  static bool Synchronous(Timestamp l, Timestamp r);

  static Timestamp GetTimestamp(const Atom *iptr);
  static std::chrono::microseconds GetMicrosecondsSinceEpoch(const Atom *iptr) { 
    return std::chrono::duration_cast<std::chrono::microseconds>(GetTimestamp(iptr).time_since_epoch()); 
  }
  static void SetTimestamp(Atom *iptr, Timestamp t);
  /**
   * Set the Code array values at index with the structure for the timestamp.
   * \param object The object with the Code array.
   * \param index The index in the Code array to place the time stamp structure (not the
   * index of the I_PTR to the structure as in SetTimestamp<Code>() or  GetTimestamp()).
   * \param timestamp The time stamp to put in the structure at the index.
   */
  static void SetTimestampStruct(Code *object, uint16 index, Timestamp t); // Expands object->code to allocate atoms.

  static const uint64 MaxTHZ = 0xFFFFFFFF;

  template<class O> static bool HasTimestamp(const O *object, uint16 index) {
    if (object->code_size() <= index)
      return false;
    uint16 t_index = object->code(index).asIndex();
    return object->code_size() > t_index + 2;
  }
  /**
   * Get the time stamp pointed to by the I_PTR at index.
   * \param object The object with the Code array.
   * \param index The index in the Code array of the I_PTR to the time stamp structure (like
   * SetTimestamp<Code>() but not the index of the time stamp structure itself as in SetTimestampStruct()).
   * \return The time stamp.
   */
  template<class O> static Timestamp GetTimestamp(const O *object, uint16 index) {

    uint16 t_index = object->code(index).asIndex();
    uint64 high = object->code(t_index + 1).atom_;
    return Timestamp(std::chrono::microseconds(high << 32 | object->code(t_index + 2).atom_));
  }

  /**
   * Set the Code array values at index with the structure for the timestamp.
   * \param object The object with the Code array.
   * \param index The index in the Code array of the I_PTR to the time stamp structure (like
   * GetTimestamp() but not the index of the time stamp structure itself as in SetTimestampStruct()).
   * \param timestamp The time stamp to put in the structure at the index.
   */
  template<class O> static void SetTimestamp(O *object, uint16 index, Timestamp timestamp) {

    uint64 t = std::chrono::duration_cast<std::chrono::microseconds>(timestamp.time_since_epoch()).count();
    uint16 t_index = object->code(index).asIndex();
    object->code(t_index) = Atom::Timestamp();
    object->code(t_index + 1).atom_ = t >> 32;
    object->code(t_index + 2).atom_ = t & 0x00000000FFFFFFFF;
  }

  static std::string GetString(const Atom *iptr);
  static void SetString(Atom *iptr, const std::string &s);

  template<class O> static std::string GetString(const O *object, uint16 index) {

    uint16 s_index = object->code(index).asIndex();
    std::string s;
    char buffer[255];
    uint8 char_count = (object->code(s_index).atom_ & 0x000000FF);
    memcpy(buffer, &object->code(s_index + 1), char_count);
    buffer[char_count] = 0;
    s += buffer;
    return s;
  }

  template<class O> static void SetString(O *object, uint16 index, const std::string &s) {

    uint16 s_index = object->code(index).asIndex();
    uint8 l = (uint8)s.length();
    object->code(s_index) = Atom::String(l);
    uint32 st = 0;
    int8 shift = 0;
    for (uint8 i = 0; i < l; ++i) {

      st |= s[i] << shift;
      shift += 8;
      if (shift == 32) {

        object->code(++s_index) = st;
        st = 0;
        shift = 0;
      }
    }
    if (l % 4)
      object->code(++s_index) = st;
  }

  static int32 GetResilience(Timestamp now, std::chrono::microseconds time_to_live, uint64 upr); // ttl: us, upr: us.
  static int32 GetResilience(Timestamp now, Timestamp::duration time_to_live, uint64 upr) {
    return GetResilience(now, std::chrono::duration_cast<std::chrono::microseconds>(time_to_live), upr);
  }
  static int32 GetResilience(float32 resilience, float32 origin_upr, float32 destination_upr); // express the res in destination group, given the res in origin group.

  static std::string RelativeTime(Timestamp t);

  /**
   * Check the code at index has a reference, also following I_PTR and recursing into sets.
   * \param code The code to check.
   * \param index start checking at code[index].
   * \return True if the element has a reference.
   */
  static bool has_reference(const Atom* code, uint16 index);
};

// This is Timestamp::max() duration_cast to microseconds and used to initialize a Timestamp.
// The result is a slightly different value from Timestamp::max(), but convertible to/from microseconds.
const Timestamp Utils_MaxTime(std::chrono::microseconds(922337203685477580LL));

}


#endif
