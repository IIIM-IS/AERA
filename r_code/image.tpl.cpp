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
#include <iostream>
#include "atom.h"


namespace r_code {

template<class I> Image<I> *Image<I>::Build(Timestamp timestamp, uint32 map_size, uint32 code_size, uint32 names_size) {

  I *image = new(map_size + code_size) I(timestamp, map_size, code_size, names_size);
  return (Image<I> *)image;
}

template<class I> Image<I> *Image<I>::Read(ifstream &stream) {

  Timestamp timestamp;
  uint32 map_size;
  uint32 code_size;
  uint32 names_size;
  stream.read((char *)&timestamp, sizeof(uint64));
  stream.read((char *)&map_size, sizeof(uint32));
  stream.read((char *)&code_size, sizeof(uint32));
  stream.read((char *)&names_size, sizeof(uint32));
  Image *image = Build(timestamp, map_size, code_size, names_size);
  stream.read((char *)image->data(), image->get_size() * sizeof(word32));
  return image;
}

template<class I> void Image<I>::Write(Image<I> *image, ofstream &stream) {

  Timestamp timestamp = image->timestamp();
  uint32 map_size = image->map_size();
  uint32 code_size = image->code_size();
  uint32 names_size = image->names_size();
  stream.write((char *)&timestamp, sizeof(uint64));
  stream.write((char *)&map_size, sizeof(uint32));
  stream.write((char *)&code_size, sizeof(uint32));
  stream.write((char *)&names_size, sizeof(uint32));
  stream.write((char *)image->data(), image->get_size() * sizeof(word32));
}

template<class I> Image<I>::Image() : I() {
}

template<class I> Image<I>::~Image() {
}

template<class I> uint32 Image<I>::get_size() const {

  return map_size() + code_size() + names_size();
}

template<class I> uint32 Image<I>::getObjectCount() const {

  return map_size();
}

template<class I> word32 *Image<I>::getObject(uint32 i) {

  return data() + data(i);
}

template<class I> word32 *Image<I>::getCodeSegment() {

  return data() + map_size();
}

template<class I> uint32 Image<I>::getCodeSegmentSize() const {

  return code_size();
}

template<class I> void Image<I>::trace() const {

  std::cout << "---Image---\n";
  std::cout << "Size: " << get_size() << std::endl;
  std::cout << "Object Map Size: " << map_size() << std::endl;
  std::cout << "Code Segment Size: " << code_size() << std::endl;
  std::cout << "Names Size: " << names_size() << std::endl;

  uint32 i = 0;

  std::cout << "===Object Map===" << std::endl;
  for (; i < map_size(); ++i)
    std::cout << i << " " << data(i) << std::endl;

  // at this point, i is at the first word32 of the first object in the code segment
  std::cout << "===Code Segment===" << std::endl;
  uint32 code_start = map_size();
  for (uint32 j = 0; j < code_start; ++j) { // read object map: data[data[j]] is the first word32 of an object, data[data[j]+5] is the first atom

    uint32 object_axiom = data(data(j));
    uint32 object_code_size = data(data(j) + 1);
    uint32 object_reference_set_size = data(data(j) + 2);
    uint32 object_marker_set_size = data(data(j) + 3);
    uint32 object_view_set_size = data(data(j) + 4);
    std::cout << "---object---\n";
    std::cout << i++;
    /*switch(object_axiom){
    case SysObject::ROOT_GRP: std::cout<<" root\n"; break;
    case SysObject::STDIN_GRP: std::cout<<" stdin\n"; break;
    case SysObject::STDOUT_GRP: std::cout<<" stdout\n"; break;
    case SysObject::SELF_ENT: std::cout<<" self\n"; break;
    default: std::cout<<" non standard\n"; break;
    }*/
    std::cout << i++ << " code size: " << object_code_size << std::endl;
    std::cout << i++ << " reference set size: " << object_reference_set_size << std::endl;
    std::cout << i++ << " marker set size: " << object_marker_set_size << std::endl;
    std::cout << i++ << " view set size: " << object_view_set_size << std::endl;

    std::cout << "---code---\n";
    for (; i < data(j) + 5 + object_code_size; ++i) {

      std::cout << i << " ";
      ((Atom *)&data(i))->trace();
      std::cout << std::endl;
    }

    std::cout << "---reference set---\n";
    for (; i < data(j) + 5 + object_code_size + object_reference_set_size; ++i)
      std::cout << i << " " << data(i) << std::endl;

    std::cout << "---marker set---\n";
    for (; i < data(j) + 5 + object_code_size + object_reference_set_size + object_marker_set_size; ++i)
      std::cout << i << " " << data(i) << std::endl;

    std::cout << "---view set---\n";
    for (uint32 k = 0; k < object_view_set_size; ++k) {

      uint32 view_code_size = data(i);
      uint32 view_reference_set_size = data(i + 1);

      std::cout << "view[" << k << "]\n";
      std::cout << i++ << " code size: " << view_code_size << std::endl;
      std::cout << i++ << " reference set size: " << view_reference_set_size << std::endl;

      std::cout << "---code---\n";
      uint32 l;
      for (l = 0; l < view_code_size; ++i, ++l) {

        std::cout << i << " ";
        ((Atom *)&data(i))->trace();
        std::cout << std::endl;
      }

      std::cout << "---reference set---\n";
      for (l = 0; l < view_reference_set_size; ++i, ++l)
        std::cout << i << " " << data(i) << std::endl;
    }
  }
}
}
