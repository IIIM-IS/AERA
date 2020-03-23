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
  static uint64 TimeReference; // starting time.
  static uint32 BasePeriod;
  static float32 FloatTolerance;
  static uint32 TimeTolerance;
public:
  static uint64 GetTimeReference();
  static uint32 GetBasePeriod();
  static uint32 GetFloatTolerance();
  static uint32 GetTimeTolerance();
  static void SetReferenceValues(uint32 base_period, float32 float_tolerance, uint32 time_tolerance);
  static void SetTimeReference(uint64 time_reference);

  static bool Equal(float32 l, float32 r);
  static bool Synchronous(uint64 l, uint64 r);

  static uint64 GetTimestamp(const Atom *iptr);
  static void SetTimestamp(Atom *iptr, uint64 t);
  static void SetTimestamp(Code *object, uint16 index, uint64 t); // allocates atoms.

  static const uint64 MaxTime = 0xFFFFFFFFFFFFFFFF;
  static const uint64 MaxTHZ = 0xFFFFFFFF;

  template<class O> static bool HasTimestamp(const O *object, uint16 index) {
    if (object->code_size() <= index)
      return false;
    uint16 t_index = object->code(index).asIndex();
    return object->code_size() > t_index + 2;
  }
  template<class O> static uint64 GetTimestamp(const O *object, uint16 index) {

    uint16 t_index = object->code(index).asIndex();
    uint64 high = object->code(t_index + 1).atom;
    return high << 32 | object->code(t_index + 2).atom;
  }

  template<class O> static void SetTimestamp(O *object, uint16 index, uint64 t) {

    uint16 t_index = object->code(index).asIndex();
    object->code(t_index) = Atom::Timestamp();
    object->code(t_index + 1).atom = t >> 32;
    object->code(t_index + 2).atom = t & 0x00000000FFFFFFFF;
  }

  static std::string GetString(const Atom *iptr);
  static void SetString(Atom *iptr, const std::string &s);

  template<class O> static std::string GetString(const O *object, uint16 index) {

    uint16 s_index = object->code(index).asIndex();
    std::string s;
    char buffer[255];
    uint8 char_count = (object->code(s_index).atom & 0x000000FF);
    memcpy(buffer, &object->code(s_index + 1), char_count);
    buffer[char_count] = 0;
    s += buffer;
    return s;
  }

  template<class O> static void SetString(O *object, uint16 index, const std::string &s) {

    uint16 s_index = object->code(index).asIndex();
    uint8 l = (uint8)s.length();
    object->code(s_index) = Atom::String(l);
    uint32 _st = 0;
    int8 shift = 0;
    for (uint8 i = 0; i < l; ++i) {

      _st |= s[i] << shift;
      shift += 8;
      if (shift == 32) {

        object->code(++s_index) = _st;
        _st = 0;
        shift = 0;
      }
    }
    if (l % 4)
      object->code(++s_index) = _st;
  }

  static int32 GetResilience(uint64 now, uint64 time_to_live, uint64 upr); // ttl: us, upr: us.
  static int32 GetResilience(float32 resilience, float32 origin_upr, float32 destination_upr); // express the res in destination group, given the res in origin group.

  static std::string RelativeTime(uint64 t);
};
}


#endif
