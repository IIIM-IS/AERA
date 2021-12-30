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

#include "../submodules/CoreLibrary/CoreLibrary/utils.h"
#include "object.h"

#include <math.h>

using namespace std;
using namespace std::chrono;

namespace r_code {

Timestamp Utils::TimeReference = Timestamp(microseconds(0));
microseconds Utils::BasePeriod = microseconds(0);
float32 Utils::FloatTolerance = 0;
microseconds Utils::TimeTolerance = microseconds(0);

Timestamp Utils::GetTimeReference() { return TimeReference; }
microseconds Utils::GetBasePeriod() { return BasePeriod; }
uint32 Utils::GetFloatTolerance() { return FloatTolerance; }
microseconds Utils::GetTimeTolerance() { return TimeTolerance; }

void Utils::SetReferenceValues(microseconds base_period, float32 float_tolerance, microseconds time_tolerance) {

  BasePeriod = base_period;
  FloatTolerance = float_tolerance;
  TimeTolerance = time_tolerance;
}

void Utils::SetTimeReference(Timestamp time_reference) {

  TimeReference = time_reference;
}

bool Utils::Equal(float32 l, float32 r) {

  if (l == r)
    return true;
  return fabs(l - r) < FloatTolerance;
}

bool Utils::Synchronous(Timestamp l, Timestamp r) {

  return abs(l - r) < TimeTolerance;
}

Timestamp Utils::GetTimestamp(const Atom *iptr) {

  return Timestamp(microseconds(GetInt64(iptr, 1)));
}

void Utils::SetTimestamp(Atom *iptr, Timestamp timestamp) {

  iptr[0] = Atom::Timestamp();
  SetInt64(iptr, 1, duration_cast<microseconds>(timestamp.time_since_epoch()).count());
}

void Utils::SetTimestampStruct(Code *object, uint16 index, Timestamp timestamp) {

  object->code(index) = Atom::Timestamp();
  // This will resize the code array if needed.
  object->code(index + 2) = 0;
  SetInt64(&object->code(0), index + 1, duration_cast<microseconds>(timestamp.time_since_epoch()).count());
}

string Utils::ToString_s_ms_us(Timestamp timestamp, Timestamp time_reference) {
  auto duration = timestamp - time_reference;
  uint64 t = abs(duration_cast<microseconds>(duration).count());

  uint64 us = t % 1000;
  uint64 ms = t / 1000;
  uint64 s = ms / 1000;
  ms = ms % 1000;

  std::string result = (duration < microseconds(0) ? "-" : "");
  result += std::to_string(s);
  result += "s:";
  result += std::to_string(ms);
  result += "ms:";
  result += std::to_string(us);
  result += "us";

  return result;
}

void Utils::SetDurationStruct(Code *object, uint16 index, microseconds duration) {
  object->resize_code(index + 3);
  object->code(index) = Atom::Duration();
  SetInt64(&object->code(0), index + 1, duration.count());
}

string Utils::ToString_us(microseconds duration) {
  uint64 us = abs(duration_cast<microseconds>(duration).count());

  std::string sign = (duration < microseconds(0) ? "-" : "");
  if (us % 1000 != 0)
    return sign + std::to_string(us) + "us";
  else {
    uint64 ms = us / 1000;
    if (ms % 1000 != 0)
      return sign + std::to_string(ms) + "ms";
    else {
      uint64 s = ms / 1000;
      return sign + std::to_string(s) + "s";
    }
  }
}

std::string Utils::GetString(const Atom *iptr) {

  std::string s;
  char buffer[255];
  uint8 char_count = (iptr[0].atom_ & 0x000000FF);
  memcpy(buffer, iptr + 1, char_count);
  buffer[char_count] = 0;
  s += buffer;
  return s;
}

void Utils::SetString(Atom *iptr, const std::string &s) {

  uint8 l = (uint8)s.length();
  uint8 index = 0;
  iptr[index] = Atom::String(l);
  uint32 st = 0;
  int8 shift = 0;
  for (uint8 i = 0; i < l; ++i) {

    st |= s[i] << shift;
    shift += 8;
    if (shift == 32) {

      iptr[++index] = st;
      st = 0;
      shift = 0;
    }
  }
  if (l % 4)
    iptr[++index] = st;
}

int32 Utils::GetResilience(Timestamp now, microseconds time_to_live, uint64 upr) {

  if (time_to_live.count() == 0 || upr == 0)
    return 1;
  auto deadline = now + time_to_live;
  uint64 last_upr = duration_cast<microseconds>(now - TimeReference).count() / upr;
  uint64 next_upr = duration_cast<microseconds>(deadline - TimeReference).count() / upr;
  if (duration_cast<microseconds>(deadline - TimeReference).count() % upr > 0)
    ++next_upr;
  return next_upr - last_upr;
}

int32 Utils::GetResilience(float32 resilience, float32 origin_upr, float32 destination_upr) {

  if (origin_upr == 0)
    return 1;
  if (destination_upr <= origin_upr)
    return 1;
  float32 r = origin_upr / destination_upr;
  float32 res = resilience * r;
  if (res < 1)
    return 1;
  return res;
}

std::string Utils::RelativeTime(Timestamp t) {

  return ToString_s_ms_us(t, TimeReference);
}

bool Utils::has_reference(const Atom* code, uint16 index) {
  Atom atom = code[index];

  switch (atom.getDescriptor()) {
  case Atom::R_PTR:
    return true;
  case Atom::I_PTR:
    return has_reference(code, atom.asIndex());
  case Atom::C_PTR:
  case Atom::SET:
  case Atom::OBJECT:
  case Atom::S_SET:
  case Atom::MARKER:
  case Atom::OPERATOR:
  case Atom::TIMESTAMP:
  case Atom::GROUP: {
    uint16 count = atom.getAtomCount();
    for (uint16 i = 1; i <= count; ++i) {
      if (has_reference(code, index + i))
        return true;
    }
    return false;
  }
  default:
    return false;
  }
}

}
