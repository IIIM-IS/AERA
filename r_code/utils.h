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

  /**
   * Get the signed 64-bit integer at code[index] and code[index + 1].
   * \param code The Atom array.
   * \param index The index in code of the first uint32 value.
   * \return The signed 64-bit integer.
   */
  static int64 GetInt64(const Atom *code, size_t index) {
    uint64 high = code[index].atom_;
    return high << 32 | code[index + 1].atom_;
  }

  /**
   * Set the uint32 values at code[index] and code[index + 1] to the big-endian of value.
   * \param code The Atom array. You must make sure the array has room up to code[index + 1].
   * \param index The index in code for the first uint32 value.
   * \param value The signed 64-bit integer to set.
   */
  static void SetInt64(Atom *code, size_t index, int64 value) {
    code[index].atom_ = (uint64)value >> 32;
    code[index + 1].atom_ = value & 0x00000000FFFFFFFF;
  }

  /**
   * Interpret iptr[1] and iptr[2] as an signed 64-bit integer and return it as time stamp.
   * This assumes that the caller has already checked iptr[0] == Atom::TIMESTAMP, if needed.
   * \param iptr A pointer into the Atom array.
   * \return The time stamp.
   */
  static Timestamp GetTimestamp(const Atom *iptr);

  static std::chrono::microseconds GetMicrosecondsSinceEpoch(const Atom *iptr) { 
    return std::chrono::duration_cast<std::chrono::microseconds>(GetTimestamp(iptr).time_since_epoch()); 
  }

  /**
   * Set iptr[0] to Atom::Timestamp() and set iptr[1] and iptr[2] to the signed 64-bit value
   * of the time stamp.
   * \param iptr A pointer into the Atom array.
   * \param timestamp The time stamp.
   */
  static void SetTimestamp(Atom *iptr, Timestamp timestamp);

  /**
   * Set the Code array values at index with the structure for the timestamp.
   * \param object The object with the Code array.
   * \param index The index in the Code array to place the time stamp structure (not the
   * index of the I_PTR to the structure as in SetTimestamp<Code>() or  GetTimestamp()).
   * \param timestamp The time stamp to put in the structure at the index.
   */
  static void SetTimestampStruct(Code *object, uint16 index, Timestamp timestamp); // Expands object->code to allocate atoms.

  static const uint64 MaxTHZ = 0xFFFFFFFF;

  template<class O> static bool HasTimestamp(const O *object, uint16 index) {
    if (object->code_size() <= index)
      return false;
    Atom a = object->code(index);
    if (a.getDescriptor() != Atom::I_PTR)
      return false;
    uint16 t_index = a.asIndex();
    if (object->code_size() <= t_index + 2)
      return false;
    return object->code(t_index).getDescriptor() == Atom::TIMESTAMP;
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
    return Timestamp(std::chrono::microseconds(GetInt64(&object->code(0), t_index + 1)));
  }

  /**
   * Set the Code array values at index with the structure for the timestamp.
   * \param object The object with the Code array.
   * \param index The index in the Code array of the I_PTR to the time stamp structure (like
   * GetTimestamp() but not the index of the time stamp structure itself as in SetTimestampStruct()).
   * \param timestamp The time stamp to put in the structure at the index.
   */
  template<class O> static void SetTimestamp(O *object, uint16 index, Timestamp timestamp) {

    uint16 t_index = object->code(index).asIndex();
    object->code(t_index) = Atom::Timestamp();
    // This will resize the code array if needed.
    object->code(t_index + 2).atom_ = 0;
    SetInt64(&object->code(0), t_index + 1, std::chrono::duration_cast<std::chrono::microseconds>(timestamp.time_since_epoch()).count());
  }

  /**
   * Make a string from (timestamp - time_reference) in the form XXXs:YYYms:ZZZus, with a minus sign
   * if it is negative.
   * \param timestamp The time stamp.
   * \param time_reference The reference time to subtract from timestamp, usually the session start time.
   * We do this because timestamp is seconds since 01/01/1970, so the seconds would be very large.
   * \return The formatted time string.
   */
  static std::string ToString_s_ms_us(Timestamp timestamp, Timestamp time_reference);

  /**
   * Interpret iptr[1] and iptr[2] as an signed 64-bit integer and return it as a duration.
   * This assumes that the caller has already checked iptr[0] == Atom::DURATION, if needed.
   * \param iptr A pointer into the Atom array.
   * \return The duration.
   */
  static std::chrono::microseconds GetDuration(const Atom *iptr) {
    return std::chrono::microseconds(GetInt64(iptr, 1));
  }

  /**
   * Set iptr[0] to Atom::Duration() and set iptr[1] and iptr[2] to the signed 64-bit value of the duration.
   * \param iptr A pointer into the Atom array.
   * \param duration The duration.
   */
  static void SetDuration(Atom *iptr, std::chrono::microseconds duration) {
    iptr[0] = Atom::Duration();
    SetInt64(iptr, 1, duration.count());
  }

  /**
   * Set the Code array values at index with the structure for the duration.
   * \param object The object with the Code array. This expands object->code to allocate atoms.
   * \param index The index in the Code array to place the duration structure (not the
   * index of the I_PTR to the structure as in SetDuration<Code>() or  GetDuration()).
   * \param duration The duration to put in the structure at the index.
   */
  static void SetDurationStruct(Code *object, uint16 index, std::chrono::microseconds duration);

  /**
   * Get the duration pointed to by the I_PTR at index.
   * This assumes that the caller has already checked that the code pointed to is
   * Atom::DURATION, if needed.
   * \param object The object with the Code array.
   * \param index The index in the Code array of the I_PTR to the duration structure (like
   * SetDuration<Code>() but not the index of the duration structure itself as in SetDurationStruct()).
   * \return The duration.
   */
  template<class O> static std::chrono::microseconds GetDuration(const O *object, uint16 index) {

    uint16 t_index = object->code(index).asIndex();
    return std::chrono::microseconds(GetInt64(&object->code(0), t_index + 1));
  }

  /**
   * Set the Code array values at index with the structure for the duration.
   * \param object The object with the Code array.
   * \param index The index in the Code array of the I_PTR to the duration structure (like
   * GetDuration() but not the index of the time stamp structure itself as in SetDurationStruct()).
   * \param duration The duration to put in the structure at the index.
   */
  template<class O> static void SetDuration(O *object, uint16 index, std::chrono::microseconds duration) {

    uint16 t_index = object->code(index).asIndex();
    object->code(t_index) = Atom::Duration();
    // This will resize the code array if needed.
    object->code(t_index + 2).atom_ = 0;
    SetInt64(&object->code(0), t_index + 1, duration.count());
  }

  /**
   * Make a string from duration in the form XXXus, with a minus sign if it is negative. However if
   * the microseconds portion is zero, then use YYYms. Or if the microseconds and milliseconds portions
   * are zero, then use ZZZs. (This is the complement to how the compiler parses durations.)
   * \param duration The duration.
   * \return The formatted time string.
   */
  static std::string ToString_us(std::chrono::microseconds duration);

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
