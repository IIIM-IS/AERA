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

#include "../submodules/CoreLibrary/CoreLibrary/utils.h"
#include "object.h"

#include <math.h>

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

  uint64 high = iptr[1].atom_;
  return Timestamp(microseconds(high << 32 | iptr[2].atom_));
}

void Utils::SetTimestamp(Atom *iptr, Timestamp timestamp) {

  uint64 t = duration_cast<microseconds>(timestamp.time_since_epoch()).count();
  iptr[0] = Atom::Timestamp();
  iptr[1].atom_ = t >> 32;
  iptr[2].atom_ = t & 0x00000000FFFFFFFF;
}

void Utils::SetTimestampStruct(Code *object, uint16 index, Timestamp timestamp) {

  uint64 t = duration_cast<microseconds>(timestamp.time_since_epoch()).count();
  object->code(index) = Atom::Timestamp();
  object->code(++index) = Atom(t >> 32);
  object->code(++index) = Atom(t & 0x00000000FFFFFFFF);
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

  return Time::ToString_seconds(t - TimeReference);
}
}
