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

#ifndef r_code_time_buffer_h
#define r_code_time_buffer_h

#include "list.h"


using namespace core;

namespace r_code {

// Time limited buffer.
// T is expected a function: bool is_invalidated(uint64 time_reference,uint32 thz) const where time_reference and thz are valuated with the buffer's own.
template<typename T, class IsInvalidated> class time_buffer :
  public list<T> {
protected:
  std::chrono::microseconds thz_; // time horizon.
  Timestamp time_reference_;
public:
  time_buffer() : list(), thz_(Utils::MaxTHZ) {}

  void set_thz(std::chrono::microseconds thz) { this->thz_ = thz; }

  class iterator {
    friend class time_buffer;
  private:
    time_buffer *buffer_;
    int32 cell_;
    iterator(time_buffer *b, int32 c) : buffer_(b), cell_(c) {}
  public:
    iterator() : buffer_(NULL), cell_(null_) {}
    T &operator *() const { return buffer_->cells_[cell_].data_; }
    T *operator ->() const { return &(buffer_->cells_[cell_].data_); }
    iterator &operator ++() { // moves to the next time-compliant cell and erase old cells met in the process.

      cell_ = buffer_->cells_[cell_].next_;
      if (cell_ != null_) {

        IsInvalidated i;
      check: if (i(buffer_->cells_[cell_].data_, buffer_->time_reference_, buffer_->thz_)) {

        cell_ = buffer_->_erase(cell_);
        if (cell_ != null_)
          goto check;
      }
      }
      return *this;
    }
    bool operator==(iterator &i) const { return cell_ == i.cell_; }
    bool operator!=(iterator &i) const { return cell_ != i.cell_; }
  };
private:
  static iterator end_iterator_;
public:
  iterator begin(Timestamp time_reference) {

    this->time_reference_ = time_reference;
    return iterator(this, used_cells_head_);
  }
  iterator &end() { return end_iterator_; }
  iterator find(Timestamp time_reference, const T &t) {

    iterator i;
    for (i = begin(time_reference); i != end(); ++i) {

      if ((*i) == t)
        return i;
    }
    return end_iterator_;
  }
  iterator find(const T &t) {

    for (int32 c = used_cells_head_; c != null_; c = cells_[c].next_) {

      if (cells_[c].data_ == t)
        return iterator(this, c);
    }
    return end_iterator_;
  }
  iterator erase(iterator &i) { return iterator(this, _erase(i.cell_)); }
};

template<typename T, class IsInvalidated> typename time_buffer<T, IsInvalidated>::iterator time_buffer<T, IsInvalidated>::end_iterator_;
}


#endif
