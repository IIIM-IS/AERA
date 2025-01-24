//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2025 Jeff Thompson
//_/_/ Copyright (c) 2018-2025 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2025 Icelandic Institute for Intelligent Machines
//_/_/ Copyright (c) 2018 Throstur Thorarensen
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

#ifndef r_code_vector_h
#define r_code_vector_h

#include <vector>
#include "../submodules/CoreLibrary/CoreLibrary/types.h"


using namespace core;

namespace r_code {

// auto-resized vector
#if 0 // If resize causes a crash, then enable this temporary fix which overrides resize but leaks memory.
  template<typename T> class resized_vector {
  public:
    resized_vector() : vector_(new std::vector<T>()) {}
    resized_vector(size_t n) : vector_(new std::vector<T>(n)) {}
    ~resized_vector() { delete vector_; }
    size_t size() const { return vector_->size(); }
    T& operator [](size_t i) {

      if (i >= size())
        resize(i + 1);
      return (*vector_)[i];
    }
    const T& operator [](size_t i) const {

      return (*vector_)[i];
    }
    void push_back(T t) { vector_->push_back(t); }
    void clear() { vector_->clear(); }
    void resize(size_t new_size) {
      if (new_size <= vector_->size()) {
        // No need to realloc.
        vector_->resize(new_size);
        return;
      }

      std::vector<T>* new_vector = new std::vector<T>(new_size);
      for (size_t i = 0; i < vector_->size(); ++i)
        (*new_vector)[i] = (*vector_)[i];
#if 0 // This fails, so comment it out. It's a memory leak, but the code runs.
      delete vector_;
#endif
      vector_ = new_vector;
    }
    const std::vector<T>* as_std() const { return vector_; }
  private:
    std::vector<T>* vector_;
  };
#else
template<typename T> class resized_vector {
public:
  resized_vector() {}
  resized_vector(size_t n) : vector_(n) {}
  size_t size() const { return vector_.size(); }
  T &operator [](size_t i) {

    if (i >= size())
      vector_.resize(i + 1);
    return vector_[i];
  }
  const T &operator [](size_t i) const {

    return vector_[i];
  }
  void push_back(T t) { vector_.push_back(t); }
  void clear() { vector_.clear(); }
  void resize(size_t new_size) { vector_.resize(new_size); }
  const std::vector<T>* as_std() const { return &vector_; }
private:
  std::vector<T> vector_;
};
#endif
}


#endif
