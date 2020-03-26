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

#ifndef out_stream_h
#define out_stream_h

#include <iostream>

#include "../r_code/vector.h"


using namespace r_code;

namespace r_comp {

// Allows inserting data at a specified index and right shifting the current content;
// ex: labels and variables, i.e. when iptrs are discovered and these hold indexes are < read_index and do not point to variables
class OutStream {
public:
  r_code::vector<uint16> code_indexes_to_stream_indexes;
  uint16 code_index;
  r_code::vector<std::streampos> positions;
  OutStream(std::ostringstream *s) : stream(s) {}
  std::ostringstream *stream;
  template<typename T> OutStream &push(const T &t, uint16 code_index) {
    positions.push_back(stream->tellp());
    code_indexes_to_stream_indexes[code_index] = positions.size() - 1;
    return *this << t;
  }
  OutStream &push() { // to keep adding entries in v without outputing anything (e.g. for wildcards after ::)
    std::streampos p;
    positions.push_back(p);
    return *this;
  }
  template<typename T> OutStream &operator <<(const T &t) {
    *stream << t;
    return *this;
  }
  template<typename T> OutStream &insert(uint32 index, const T &t) { // inserts before code_indexes_to_stream_indexes[index]
    uint16 stream_index = code_indexes_to_stream_indexes[index];
    stream->seekp(positions[stream_index]);
    std::string s = stream->str().substr(positions[stream_index]);
    *stream << t;
    std::streamoff offset = stream->tellp() - positions[stream_index];
    *stream << s;
    for (uint16 i = stream_index + 1; i < positions.size(); ++i) // right-shift
      positions[i] += offset;
    return *this;
  }
};

class NoStream :
  public std::ostream {
public:
  NoStream() : std::ostream(NULL) {}
  template<typename T> NoStream& operator <<(T &t) {
    return *this;
  }
};

class CompilerOutput :
  public std::ostream {
public:
  CompilerOutput() : std::ostream(NULL) {}
  template<typename T> std::ostream& operator <<(T &t) {
    if (1) // njt: was if (Output); HACK for linux compatibility
      return std::cout << t;
    return *this;
  }
};
}


#endif
