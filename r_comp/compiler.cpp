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
//_/_/ Copyright (c) 2010 Nathaniel Thurston
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

#include "compiler.h"
#include <string.h>

using namespace std;
using namespace r_code;

namespace r_comp {

static bool Output = true;

static inline bool is_decimal(char c) { return c >= '0' && c <= '9'; }

Compiler::Compiler(bool allow_variables_and_wildcards_outside_pattern_skeleton)
: out_stream_(NULL), current_object_(NULL), error_(std::string("")),
  allow_variables_and_wildcards_outside_pattern_skeleton_(allow_variables_and_wildcards_outside_pattern_skeleton)
{
}

Compiler::~Compiler() {

  if (out_stream_)
    delete out_stream_;
}

Compiler::State Compiler::save_state() {

  State s(this);
  return s;
}

void Compiler::restore_state(State s) {

  in_stream_->seekg(s.stream_ptr);
  state_ = s;
}

void Compiler::set_error(const std::string &s) {

  if (!err_ && Output) {

    err_ = true;
    error_ = s;
  }
}

void Compiler::set_arity_error(uint16 expected, uint16 got) {

  set_error("error: got " + std::to_string(got) + " elements, expected " +
    std::to_string(expected));
}

////////////////////////////////////////////////////////////////////////////////////////////////

bool Compiler::compile(std::istream *stream, r_comp::Image *image, r_comp::Metadata *metadata, std::string &error, bool trace) {

  in_stream_ = stream;
  err_ = false;
  trace_ = trace;

  image_ = image;
  metadata_ = metadata;
  current_object_index_ = image->object_map_.objects_.size();
  while (!in_stream_->eof()) {

    switch (in_stream_->peek()) {
    case '!':
      set_error("error: found preprocessor directive");
      error = error_;
      return false;
    default:
      if (in_stream_->eof())
        return true;
      if (!read_sys_object()) {

        error = error_;
        return false;
      }
      current_object_index_++;
      break;
    }
    char c = (char)in_stream_->get();
    if (c != -1)
      in_stream_->putback(c);
  }

  return true;
}

bool Compiler::read_sys_object() {

  local_references_.clear();
  hlp_references_.clear();
  bool indented = false;
  bool lbl = false;

  current_view_index_ = -1;

  std::string l;
  while (indent(false));
  char c = (char)in_stream_->get();
  if (c != -1)
    in_stream_->putback(c);
  else
    return true;

  in_hlp_ = false;

  if (label(l))
    lbl = true;
  if (!expression_begin(indented)) {

    if (lbl)
      set_error("error: label not followed by an expression");
    else
      set_error("syntax error: missing expression opening");
    return false;
  }
  indent(false);

  if (!sys_object(current_class_)) {

    set_error("error: unknown class");
    return false;
  } else {

    if (current_class_.str_opcode == "mdl" || current_class_.str_opcode == "cst")
      in_hlp_ = true;
    current_object_ = new SysObject();
    if (lbl) {
      if (global_references_.find(l) != global_references_.end()) {
        set_error("error: redefinition of label `" + l + "`");
        return false;
      }
      global_references_[l] = Reference(image_->code_segment_.objects_.size(), current_class_, Class());
    }
  }

  current_object_->code_[0] = current_class_.atom_;
  if (current_class_.atom_.getAtomCount()) {

    if (!right_indent(true)) {

      if (!separator(false)) {

        set_error("syntax error: missing separator/right_indent after head");
        return false;
      }
    }
    uint16 extent_index = current_class_.atom_.getAtomCount() + 1;
    if (!expression_tail(indented, current_class_, 1, extent_index, true))
      return false;
  }

  SysObject *sys_object = (SysObject *)current_object_; // current_object_ will point to views_, if any.

  // compile view set
  // input format:
  // []
  // [view-data]
  // ...
  // [view-data]
  // or:
  // |[]

  while (indent(false));
  std::streampos i = in_stream_->tellg();
  if (!match_symbol("|[]", false)) {

    in_stream_->seekg(i);
    if (!set_begin(indented)) {

      set_error(" error: expected a view set");
      return false;
    }

    indent(false);

    if (current_class_.str_opcode == "grp")
      current_class_ = metadata_->classes_.find("grp_view")->second;
    else if (current_class_.str_opcode == "ipgm" ||
      current_class_.str_opcode == "icpp_pgm" ||
      current_class_.str_opcode == "mdl" ||
      current_class_.str_opcode == "cst")
      current_class_ = metadata_->classes_.find("pgm_view")->second;
    else
      current_class_ = metadata_->classes_.find("view")->second;
    current_class_.use_as = StructureMember::I_CLASS;

    uint16 count = 0;
    bool _indented = false;
    while (!in_stream_->eof()) {

      current_object_ = new SysView();
      current_view_index_ = count;
      uint16 extent_index = 0;

      if (set_end(indented)) {

        if (!count) {

          set_error(" syntax error: use |[] for empty sets");
          delete current_object_;
          return false;
        } else {

          delete current_object_;
          break;
        }
      }
      if (count) {

        if (!_indented) {

          if (!right_indent(true)) {

            if (!separator(false)) {

              set_error("syntax error: missing separator between 2 elements");
              delete current_object_;
              return false;
            }
          }
        } else
          _indented = false;
      }
      if (!read_set(_indented, true, &current_class_, 0, extent_index, true)) {

        set_error(" error: illegal element in set");
        delete current_object_;
        return false;
      }
      count++;
      sys_object->views_.push_back((SysView *)current_object_);
    }
  }

  if (trace_)
    sys_object->trace();

  image_->add_sys_object(sys_object, l);
  return true;
}

bool Compiler::read(const StructureMember &m, bool &indented, bool enforce, uint16 write_index, uint16 &extent_index, bool write) {

  if (Class *p = m.get_class(metadata_)) {

    p->use_as = m.getIteration();
    return (this->*m.read())(indented, enforce, p, write_index, extent_index, write);
  }
  return (this->*m.read())(indented, enforce, NULL, write_index, extent_index, write);
}

bool Compiler::getGlobalReferenceIndex(const std::string reference_name, const ReturnType t, ImageObject *object, uint16 &index, Class *&_class) {

  unordered_map<std::string, Reference>::iterator it = global_references_.find(reference_name);
  if (it != global_references_.end() && (t == ANY || (t != ANY && it->second.class_.type == t))) {

    _class = &it->second.class_;
    for (uint16 j = 0; j < object->references_.size(); ++j)
      if (object->references_[j] == it->second.index_) { // the object has already been referenced.

        index = j; // rptr points to object->reference_set[j], which in turn points to it->second.index.
        return true;
      }
    object->references_.push_back(it->second.index_); // add new reference to the object.
    index = object->references_.size() - 1; // rptr points to the last element of object->reference_set, which in turn points to it->second.index.
    return true;
  }
  return false;
}

void Compiler::addLocalReference(const std::string reference_name, const uint16 index, const Class &p) {

  // cast detection.
  size_t pos = reference_name.find('#');
  if (pos != string::npos) {

    std::string class_name = reference_name.substr(pos + 1);
    std::string ref_name = reference_name.substr(0, pos);

    unordered_map<std::string, Class>::iterator it = metadata_->classes_.find(class_name);
    if (it != metadata_->classes_.end())
      local_references_[ref_name] = Reference(index, p, it->second);
    else
      set_error(" error: cast to " + class_name + ": unknown class");
  } else
    local_references_[reference_name] = Reference(index, p, Class());
}

uint8 Compiler::add_hlp_reference(std::string reference_name) {

  for (uint8 i = 0; i < hlp_references_.size(); ++i)
    if (reference_name == hlp_references_[i])
      return i;
  hlp_references_.push_back(reference_name);
  return hlp_references_.size() - 1;
}

uint8 Compiler::get_hlp_reference(std::string reference_name) {

  for (uint8 i = 0; i < hlp_references_.size(); ++i)
    if (reference_name == hlp_references_[i])
      return i;
  return 0xFF;
}

////////////////////////////////////////////////////////////////////////////////////////////////

bool Compiler::comment() {

  std::streampos i = in_stream_->tellg();
  bool started = false;
  bool continuation = false; // continuation mark detected
  bool period = false; // to detect 2 subsequent '.'
  while (!in_stream_->eof()) {

    switch (char c = (char)in_stream_->get()) {
    case ';':
      if (!started)
        started = true;
      break;
    case '.':
      if (!started)
        goto return_false;
      if (continuation) {

        set_error(" syntax error: ...");
        goto return_false;
      }
      if (period)
        continuation = true;
      period = true;
      break;
    case NEWLINE:
      if (!continuation) {

        in_stream_->putback(c);
        return true;
      }
      continuation = period = false;
      break;
    default:
      if (!started)
        goto return_false;
      period = false;
      break;
    }
  }
return_false:
  in_stream_->seekg(i);
  in_stream_->clear();
  return false;
}

bool Compiler::indent(bool pushback) {

  comment();
  std::string s;
  s += NEWLINE;
  for (uint16 j = 0; j < 3 * state_.indents; j++)
    s += ' ';
  return match_symbol(s.c_str(), pushback);
}

bool Compiler::right_indent(bool pushback) { // no look ahead when pushback==true

  comment();
  if (pushback) {

    if (state_.right_indents_ahead)
      return true;
    std::string s;
    s += NEWLINE;
    for (uint16 j = 0; j < 3 * (state_.indents + 1); j++)
      s += ' ';
    return match_symbol(s.c_str(), true);
  }
  if (state_.right_indents_ahead) {

    state_.indents++;
    state_.right_indents_ahead--;
    return true;
  }
  std::string s;
  s += NEWLINE;
  for (uint16 j = 0; j < 3 * (state_.indents + 1); j++)
    s += ' ';
  if (!match_symbol(s.c_str(), false))
    return false;
  state_.indents++;
  s = "   ";
  while (match_symbol(s.c_str(), false)) // look ahead for more indents
    state_.right_indents_ahead++;
  return true;
}

bool Compiler::left_indent(bool pushback) { // no look ahead when pushback==true

  comment();
  if (indent(true))
    return false;
  if (pushback) {

    if (state_.left_indents_ahead)
      return true;
    std::string s;
    s += NEWLINE;
    for (uint16 j = 0; j < 3 * (state_.indents - 1); j++)
      s += ' ';
    return match_symbol(s.c_str(), true);
  }
  if (state_.left_indents_ahead) {

    if (state_.indents)
      state_.indents--;
    state_.left_indents_ahead--;
    return true;
  }
  std::string s;
  s += NEWLINE;
  if (!match_symbol(s.c_str(), false))
    return false;
  uint16 expected = state_.indents - 1;
  if (expected <= 0) {

    if (state_.indents)
      state_.indents--;
    return true;
  }
  state_.left_indents_ahead = expected; // look ahead for more indents
  s = "   ";
  for (uint16 j = 0; j < expected; j++) {

    if (match_symbol(s.c_str(), false))
      state_.left_indents_ahead--;
  }
  if (state_.indents)
    state_.indents--;
  return true;
}

bool Compiler::separator(bool pushback) {

  if (indent(pushback))
    return true;
  char c = (char)in_stream_->get();
  if (c == ' ') {

    if (pushback)
      in_stream_->putback(c);
    return true;
  }
  in_stream_->clear();
  in_stream_->putback(c);
  return false;
}

bool Compiler::symbol_expr(std::string &s) {

  std::streampos i = in_stream_->tellg();
  uint16 count = 0;
  while (!in_stream_->eof()) {

    switch (char c = (char)in_stream_->get()) {
    case ':':
    case ' ':
    case NEWLINE:
    case ';':
    case ')':
      if (count) {

        in_stream_->putback(c);
        return true;
      }
    case '(':
    case '[':
    case ']':
    case '.':
      if (s == "mk") {

        s += '.';
        break;
      }
      in_stream_->seekg(i);
      return false;
    default:
      count++;
      s += c;
      break;
    }
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::symbol_expr_set(std::string &s) {

  std::streampos i = in_stream_->tellg();
  uint16 count = 0;
  while (!in_stream_->eof()) {

    switch (char c = (char)in_stream_->get()) {
    case ' ':
    case NEWLINE:
    case ';':
    case ')':
    case ']':
      if (count) {

        in_stream_->putback(c);
        return true;
      }
    case '(':
    case '[':
    case '.':
    case ':':
      in_stream_->seekg(i);
      return false;
    default:
      count++;
      s += c;
      break;
    }
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::match_symbol_separator(const char *symbol, bool pushback) {

  if (match_symbol(symbol, pushback)) {

    if (separator(true) || right_indent(true) || left_indent(true))
      return true;
    char c = (char)in_stream_->peek();
    if (c == ')' || c == ']')
      return true;
  }
  return false;
}

bool Compiler::match_symbol(const char *symbol, bool pushback) {

  std::streampos i = in_stream_->tellg();
  for (uint32 j = 0; j < strlen(symbol); j++) {

    if (in_stream_->eof() || ((char)in_stream_->get()) != symbol[j]) {

      in_stream_->clear();
      in_stream_->seekg(i);
      return false;
    }
  }
  if (pushback)
    in_stream_->seekg(i);
  return true;
}

bool Compiler::member(std::string &s) {

  std::streampos i = in_stream_->tellg();
  s = "";
  uint16 count = 0;
  while (!in_stream_->eof()) {

    switch (char c = (char)in_stream_->get()) {
    case ' ':
    case NEWLINE:
    case ';':
    case ')':
    case ']':
    case '.':
      if (count) {

        in_stream_->putback(c);
        return true;
      }
    case '(':
    case '[':
    case ':':
      in_stream_->seekg(i);
      return false;
    default:
      count++;
      s += c;
      break;
    }
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::expression_begin(bool &indented) {

  if (right_indent(false)) {

    indented = true;
    return true;
  }
  std::streampos i = in_stream_->tellg();
  if (indent(false)) {

    char c = (char)in_stream_->get();
    if (c == '(')
      return true;
    in_stream_->clear();
  }
  in_stream_->seekg(i);
  char c = (char)in_stream_->get();
  if (c == '(')
    return true;
  in_stream_->clear();
  in_stream_->putback(c);
  return false;
}

bool Compiler::expression_end(bool indented) {

  if (indented)
    return left_indent(false);
  std::streampos i = in_stream_->tellg();
  if (indent(false)) {

    char c = (char)in_stream_->get();
    if (c == ')')
      return true;
    in_stream_->clear();
  }
  in_stream_->seekg(i);
  char c = (char)in_stream_->get();
  if (c == ')')
    return true;
  in_stream_->clear();
  in_stream_->putback(c);
  return false;
}

bool Compiler::set_begin(bool &indented) {

  std::streampos i = in_stream_->tellg();
  if (match_symbol("[]", false)) {

    if (right_indent(false)) {

      indented = true;
      return true;
    } else {

      set_error(" syntax error: [] not followed by indent");
      return false;
    }
  }
  in_stream_->seekg(i);
  if (indent(false)) {

    char c = (char)in_stream_->get();
    if (c == '[')
      return true;
    in_stream_->clear();
  }
  in_stream_->seekg(i);
  char c = (char)in_stream_->get();
  if (c == '[')
    return true;
  in_stream_->clear();
  in_stream_->putback(c);
  return false;
}

bool Compiler::set_end(bool indented) {

  if (indented)
    return left_indent(false);
  std::streampos i = in_stream_->tellg();
  if (indent(false)) {

    char c = (char)in_stream_->get();
    if (c == ']')
      return true;
    in_stream_->clear();
  }
  in_stream_->seekg(i);
  char c = (char)in_stream_->get();
  if (c == ']')
    return true;
  in_stream_->clear();
  in_stream_->putback(c);
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////

bool Compiler::nil() {

  std::streampos i = in_stream_->tellg();
  if (match_symbol_separator("nil", false))
    return true;
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::nil_nb() {

  std::streampos i = in_stream_->tellg();
  if (match_symbol_separator("|nb", false))
    return true;
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::nil_us() {

  std::streampos i = in_stream_->tellg();
  if (match_symbol_separator("|ms", false))
    return true;
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::forever() {

  std::streampos i = in_stream_->tellg();
  if (match_symbol_separator("forever", false))
    return true;
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::nil_nid() {

  std::streampos i = in_stream_->tellg();
  if (match_symbol_separator("|nid", false))
    return true;
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::nil_did() {

  std::streampos i = in_stream_->tellg();
  if (match_symbol_separator("|did", false))
    return true;
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::nil_fid() {

  std::streampos i = in_stream_->tellg();
  if (match_symbol_separator("|fid", false))
    return true;
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::nil_bl() {

  std::streampos i = in_stream_->tellg();
  if (match_symbol_separator("|bl", false))
    return true;
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::nil_st() {

  std::streampos i = in_stream_->tellg();
  if (match_symbol_separator("|st", false))
    return true;
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::label(std::string &l) {

  std::streampos i = in_stream_->tellg();
  if (symbol_expr(l) && !is_decimal(l[0]) && (char)in_stream_->get() == ':')
    return true;
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::variable(std::string &l) {

  std::streampos i = in_stream_->tellg();
  if (symbol_expr(l) && !is_decimal(l[0]) && (char)in_stream_->get() == ':') {

    in_stream_->seekg(i);
    std::string _l = l + ':';
    if (match_symbol_separator(_l.c_str(), false))
      return true;
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::this_() {

  std::streampos i = in_stream_->tellg();
  if (match_symbol_separator("this", false))
    return true;
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::local_reference(uint16 &index, const ReturnType t) {

  std::streampos i = in_stream_->tellg();
  std::string r;
  if (symbol_expr_set(r)) {

    unordered_map<std::string, Reference>::iterator it = local_references_.find(r);
    if (it != local_references_.end() && (t == ANY || it->second.class_.type == ANY || (t != ANY && it->second.class_.type == t))) {

      index = it->second.index_;
      return true;
    }
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::global_reference(uint16 &index, const ReturnType t) {

  std::streampos i = in_stream_->tellg();
  std::string r;
  if (symbol_expr_set(r)) {

    Class *unused;
    if (getGlobalReferenceIndex(r, t, current_object_, index, unused))
      return true;
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::hlp_reference(uint16 &index) {

  std::string r;
  std::streampos i = in_stream_->tellg();
  if (label(r)) {

    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }
  r = "";
  if (symbol_expr(r)) {

    for (uint8 i = 0; i < hlp_references_.size(); ++i)
      if (r == hlp_references_[i]) {

        index = i;
        return true;
      }
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::this_indirection(vector<int16> &v, const ReturnType t) {

  std::streampos i = in_stream_->tellg();
  if (match_symbol("this.", false)) {

    Class *p; // in general, p starts as the current_class_; exception: in pgm, this refers to the instantiated program.
    if (current_class_.str_opcode == "pgm")
      p = &metadata_->sys_classes_["ipgm"];
    Class *_p;
    std::string m;
    uint16 index;
    ReturnType type;
    while (member(m)) {

      if (m == "vw") {

        _p = &metadata_->classes_.find("pgm_view")->second;
        type = ANY;
        v.push_back(-1);
      } else if (m == "mks") {

        _p = NULL;
        type = SET;
        v.push_back(-2);
      } else if (m == "vws") {

        _p = NULL;
        type = SET;
        v.push_back(-3);
      } else if (!p->get_member_index(metadata_, m, index, _p)) {

        set_error(" error: " + m + " is not a member of " + p->str_opcode);
        break;
      } else {

        type = p->get_member_type(index);
        v.push_back(index);
      }

      char c = (char)in_stream_->get();
      if (c == '.') {

        if (!_p) {

          set_error(" error: " + m + " is not a structure");
          break;
        }
        p = _p;
      } else {

        if (t == ANY || (t != ANY && type == t)) {

          in_stream_->putback(c);
          return true;
        }
        break;
      }
    }
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::local_indirection(vector<int16> &v, const ReturnType t, uint16 &cast_opcode) {

  std::streampos i = in_stream_->tellg();
  std::string m;
  std::string path = "";
  Class *p;
  if (member(m) && (char)in_stream_->get() == '.') { // first m is a reference to a label or a variable

    uint16 index;
    ReturnType type;
    unordered_map<std::string, Reference>::iterator it = local_references_.find(m);
    if (it != local_references_.end()) {

      index = it->second.index_;
      v.push_back(index);
      if (it->second.cast_class_.str_opcode == "undefined") { // find out if there was a cast for this reference.

        p = &it->second.class_;
        cast_opcode = 0x0FFF;
      } else {

        p = &it->second.cast_class_;
        cast_opcode = p->atom_.asOpcode();
      }
      Class *_p;
      while (member(m)) {

        if (m == "vw") {

          _p = &metadata_->classes_.find("pgm_view")->second;
          type = ANY;
          v.push_back(-1);
        } else if (m == "mks") {

          _p = NULL;
          type = SET;
          v.push_back(-2);
        } else if (m == "vws") {

          _p = NULL;
          type = SET;
          v.push_back(-3);
        } else if (!p->get_member_index(metadata_, m, index, _p)) {

          set_error(" error: " + m + " is not a member of " + p->str_opcode);
          break;
        } else {

          type = p->get_member_type(index);
          v.push_back(index);
        }

        path += '.';
        path += m;
        char c = (char)in_stream_->get();
        if (c == '.') {

          if (!_p) {

            set_error(" error: " + path + " is not an addressable structure");
            break;
          }
          p = _p;
        } else {

          if (t == ANY || (t != ANY && type == t)) {

            in_stream_->putback(c);
            return true;
          }
          break;
        }
      }
    }
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::global_indirection(vector<int16> &v, const ReturnType t) {

  std::streampos i = in_stream_->tellg();
  std::string m;
  Class *p;
  if (member(m) && (char)in_stream_->get() == '.') { // first m is a reference

    uint16 index;
    ReturnType type;
    if (getGlobalReferenceIndex(m, ANY, current_object_, index, p)) {

      v.push_back(index);
      Class *_p;
      bool first_member = true;
      while (member(m)) {

        if (m == "vw") {

          set_error(" error: vw is not accessible on global references");
          break;
        } else if (m == "mks") {

          _p = NULL;
          type = SET;
          v.push_back(-2);
        } else if (m == "vws") {

          _p = NULL;
          type = SET;
          v.push_back(-3);
        } else if (!p->get_member_index(metadata_, m, index, _p)) {

          set_error(" error: " + m + " is not a member of " + p->str_opcode);
          break;
        } else {

          type = p->get_member_type(index);
          if (first_member && index == 0) // indicates the first member; store in the RObject, after the leading atom, hence index=1.
            index = 1;
          v.push_back(index);
        }

        first_member = false;

        char c = (char)in_stream_->get();
        if (c == '.') {

          if (!_p) {

            set_error(" error: " + m + " is not a structure");
            break;
          }
          p = _p;
        } else {

          if (t == ANY || (t != ANY && type == t)) {

            in_stream_->putback(c);
            return true;
          }
          break;
        }
      }
    }
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::wildcard() {

  std::streampos i = in_stream_->tellg();
  if (match_symbol_separator(":", false))
    return true;
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::tail_wildcard() {

  std::streampos i = in_stream_->tellg();
  if (match_symbol("::", false)) {

    if (left_indent(true)) {

      state_.no_arity_check = true;
      return true;
    }
    char c = (char)in_stream_->peek();
    if (c == ')' || c == ']') {

      state_.no_arity_check = true;
      return true;
    }
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::number(float32 &n) {

  std::streampos i = in_stream_->tellg();
  if (match_symbol("0x", true)) {

    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }
  *in_stream_ >> std::dec >> n;
  if (in_stream_->fail() || in_stream_->eof()) {

    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }
  if (match_symbol("us", true)) {
    // Assume this is a timestamp, not a number.
    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }
  if (match_symbol("s", true)) {
    // Assume this is a timestamp, not a number.
    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }
  return true;
}

bool Compiler::hex(uint32 &h) {

  std::streampos i = in_stream_->tellg();
  if (!match_symbol("0x", false)) {

    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }
  *in_stream_ >> std::hex >> h;
  if (in_stream_->fail() || in_stream_->eof()) {

    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }
  return true;
}

bool Compiler::boolean(bool &b) {

  std::streampos i = in_stream_->tellg();
  if (match_symbol_separator("true", false)) {

    b = true;
    return true;
  }
  if (match_symbol_separator("false", false)) {

    b = false;
    return true;
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::timestamp(uint64 &ts) {

  std::streampos i = in_stream_->tellg();
  if (match_symbol("0x", true)) {

    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }
  *in_stream_ >> std::dec >> ts;
  if (in_stream_->fail() || in_stream_->eof()) {

    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }
  if (!match_symbol("us", false)) {

    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }
  return true;
}

bool Compiler::timestamp_s_ms_us(uint64 &ts) {

  std::streampos i = in_stream_->tellg();
  if (match_symbol("0x", true)) {
    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }

  uint64 s;
  *in_stream_ >> std::dec >> s;
  if (in_stream_->fail() || in_stream_->eof()) {
    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }
  if (!match_symbol("s:", false)) {
    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }

  uint64 ms;
  *in_stream_ >> std::dec >> ms;
  if (in_stream_->fail() || in_stream_->eof()) {
    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }
  if (!match_symbol("ms:", false)) {
    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }

  uint64 us;
  *in_stream_ >> std::dec >> us;
  if (in_stream_->fail() || in_stream_->eof()) {
    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }
  if (!match_symbol("us", false)) {
    in_stream_->clear();
    in_stream_->seekg(i);
    return false;
  }

  ts = s * 1000000 + ms * 1000 + us;
  return true;
}

bool Compiler::str(std::string &s) {

  std::streampos i = in_stream_->tellg();
  uint16 count = 0;
  bool started = false;
  while (!in_stream_->eof()) {

    switch (char c = (char)in_stream_->get()) {
    case '"':
      if (!started) {

        started = true;
        break;
      }
      return true;
    default:
      if (!started) {

        in_stream_->clear();
        in_stream_->seekg(i);
        return false;
      }
      count++;
      s += c;
      break;
    }
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::object(Class &p) {

  if (sys_object(p))
    return true;
  std::streampos i = in_stream_->tellg();
  std::string s;
  if (!symbol_expr(s)) {

    in_stream_->seekg(i);
    s = "";
    if (!symbol_expr_set(s)) {

      in_stream_->seekg(i);
      return false;
    }
  }
  unordered_map<std::string, Class>::const_iterator it = metadata_->classes_.find(s);
  if (it == metadata_->classes_.end()) {

    in_stream_->seekg(i);
    return false;
  }
  p = it->second;
  return true;
}

bool Compiler::object(const Class &p) {

  if (sys_object(p))
    return true;
  std::streampos i = in_stream_->tellg();
  if (!match_symbol_separator(p.str_opcode.c_str(), false)) {

    in_stream_->seekg(i);
    return false;
  }
  return true;
}

bool Compiler::sys_object(Class &p) {

  std::streampos i = in_stream_->tellg();
  std::string s;
  if (!symbol_expr(s)) {

    in_stream_->seekg(i);
    s = "";
    if (!symbol_expr_set(s)) {

      in_stream_->seekg(i);
      return false;
    }
  }
  unordered_map<std::string, Class>::const_iterator it = metadata_->sys_classes_.find(s);
  if (it == metadata_->sys_classes_.end()) {

    in_stream_->seekg(i);
    return false;
  }
  p = it->second;
  return true;
}

bool Compiler::sys_object(const Class &p) {

  std::streampos i = in_stream_->tellg();
  if (!match_symbol_separator(p.str_opcode.c_str(), false)) {

    in_stream_->seekg(i);
    return false;
  }
  return true;
}

bool Compiler::marker(Class &p) {

  std::streampos i = in_stream_->tellg();
  if (!match_symbol("mk.", false)) {

    in_stream_->seekg(i);
    return false;
  }
  std::streampos j = in_stream_->tellg();
  std::string s;
  if (!symbol_expr(s)) {

    in_stream_->seekg(j);
    s = "";
    if (!symbol_expr_set(s)) {

      in_stream_->seekg(i);
      return false;
    }
  }
  unordered_map<std::string, Class>::const_iterator it = metadata_->sys_classes_.find("mk." + s);
  if (it == metadata_->sys_classes_.end()) {

    in_stream_->seekg(i);
    return false;
  }
  p = it->second;
  return true;
}

bool Compiler::op(Class &p, const ReturnType t) { // return true if type matches t or ANY

  std::streampos i = in_stream_->tellg();
  std::string s;
  if (!symbol_expr(s)) {

    in_stream_->seekg(i);
    return false;
  }
  unordered_map<std::string, Class>::const_iterator it = metadata_->classes_.find(s);
  if (it == metadata_->classes_.end() || (t != ANY && it->second.type != ANY && it->second.type != t)) {

    in_stream_->seekg(i);
    return false;
  }
  p = it->second;
  return true;
}

bool Compiler::op(const Class &p) {

  std::streampos i = in_stream_->tellg();
  if (!match_symbol_separator(p.str_opcode.c_str(), false)) {

    in_stream_->seekg(i);
    return false;
  }
  return true;
}

bool Compiler::function(Class &p) {

  std::streampos i = in_stream_->tellg();
  std::string s;
  if (!symbol_expr(s)) {

    in_stream_->seekg(i);
    return false;
  }
  unordered_map<std::string, Class>::const_iterator it = metadata_->classes_.find(s);
  if (it == metadata_->classes_.end()) {

    in_stream_->seekg(i);
    return false;
  }
  p = it->second;
  return true;
}

bool Compiler::expression_head(Class &p, const ReturnType t) {

  indent(false);
  if (t == ANY) {

    if (!object(p))
      if (!marker(p))
        if (!op(p, ANY))
          return false;
  } else if (!op(p, t))
    return false;
  if (p.atom_.getAtomCount()) {

    if (!right_indent(true)) {

      if (!separator(false)) {

        set_error("syntax error: missing separator/right_indent after head");
        return false;
      }
    }
  }
  return true;
}

bool Compiler::expression_head(const Class &p) {

  indent(false);
  if (!object(p))
    if (!op(p))
      return false;
  if (p.atom_.getAtomCount()) {

    if (!right_indent(true)) {

      if (!separator(false)) {

        set_error("syntax error: missing separator/right_indent after head");
        return false;
      }
    }
  }
  return true;
}

bool Compiler::expression_tail(bool indented, const Class &p, uint16 write_index, uint16 &extent_index, bool write) { // arity>0.

  uint16 count = 0;
  bool _indented = false;
  bool entered_pattern;
  uint16 pattern_end_index;
  if (in_hlp_) {

    entered_pattern = true;//p.is_fact(metadata_);
    pattern_end_index = p.atom_.getAtomCount() - 1;
  } else {

    entered_pattern = p.is_pattern(metadata_);
    pattern_end_index = 0;
  }
  if (write && state_.pattern_lvl) // fill up with wildcards that will be overwritten up to ::.
    for (uint16 j = write_index; j < write_index + p.atom_.getAtomCount(); ++j)
      current_object_->code_[j] = Atom::Wildcard();
  std::streampos i = in_stream_->tellg();
  while (!in_stream_->eof()) {

    if (expression_end(indented)) {

      if (state_.no_arity_check) {

        state_.no_arity_check = false;
        if (entered_pattern && pattern_end_index > 0 && count - 1 < pattern_end_index)
          // state_.pattern_lvl was incremented below, but the tail wildcard prevented decrementing it. Do it here.
          --state_.pattern_lvl;
        return true;
      }
      if (count == p.atom_.getAtomCount())
        return true;
      set_arity_error(p.atom_.getAtomCount(), count);
      return false;
    }
    if (count >= p.atom_.getAtomCount()) {

      set_arity_error(p.atom_.getAtomCount(), count + 1);
      return false;
    }
    if (count) {

      if (!_indented) {

        if (!right_indent(true)) {

          if (!separator(false)) {

            set_error("syntax error: missing separator between 2 elements");
            return false;
          }
        }
      } else
        _indented = false;
    }
    if (entered_pattern && count == 0) // pattern begin.
      ++state_.pattern_lvl;
    if (!read(p.things_to_read[count], _indented, true, write_index + count, extent_index, write)) {

      set_error(" error: parsing element in expression");
      return false;
    }
    if (entered_pattern && count == pattern_end_index) // pattern end.
      --state_.pattern_lvl;
    ++count;
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////

bool Compiler::expression(bool &indented, const ReturnType t, uint16 write_index, uint16 &extent_index, bool write) {

  bool lbl = false;
  std::streampos i = in_stream_->tellg();
  std::string l;
  if (label(l))
    lbl = true;
  if (!expression_begin(indented)) {

    if (lbl)
      set_error(" error: label not followed by an expression");
    return false;
  }
  Class p;
  if (!expression_head(p, t)) {

    in_stream_->seekg(i);
    return false;
  }
  if (lbl && !in_hlp_)
    addLocalReference(l, write_index, p);
  uint16 tail_write_index = 0;
  if (write) {

    if (lbl && in_hlp_) {

      uint8 variable_index = get_hlp_reference(l);
      if (variable_index == 0xFF) {

        set_error(" error: undeclared variable");
        return false;
      }
      current_object_->code_[write_index] = Atom::AssignmentPointer(variable_index, extent_index);
    } else
      current_object_->code_[write_index] = Atom::IPointer(extent_index);
    current_object_->code_[extent_index++] = p.atom_;
    tail_write_index = extent_index;
    extent_index += p.atom_.getAtomCount();
  }
  if (!expression_tail(indented, p, tail_write_index, extent_index, write))
    return false;
  return true;
}

bool Compiler::expression(bool &indented, const Class &p, uint16 write_index, uint16 &extent_index, bool write) {

  bool lbl = false;
  std::streampos i = in_stream_->tellg();
  std::string l;
  if (label(l))
    lbl = true;
  if (!expression_begin(indented)) {

    if (lbl)
      set_error(" error: label not followed by an expression");
    return false;
  }
  if (!expression_head(p)) {

    in_stream_->seekg(i);
    return false;
  }
  if (lbl)
    addLocalReference(l, write_index, p);
  uint16 tail_write_index = 0;
  if (write) {

    if (lbl && in_hlp_) {

      uint8 variable_index = get_hlp_reference(l);
      if (variable_index == 0xFF) {

        set_error(" error: undeclared variable");
        return false;
      }
      current_object_->code_[write_index] = Atom::AssignmentPointer(variable_index, extent_index);
    } else
      current_object_->code_[write_index] = Atom::IPointer(extent_index);
    current_object_->code_[extent_index++] = p.atom_;
    tail_write_index = extent_index;
    extent_index += p.atom_.getAtomCount();
  }
  if (!expression_tail(indented, p, tail_write_index, extent_index, write))
    return false;
  return true;
}

bool Compiler::set(bool &indented, uint16 write_index, uint16 &extent_index, bool write) { // [ ] is illegal; use |[] instead, or [nil].

  std::streampos i = in_stream_->tellg();
  bool lbl = false;
  std::string l;
  if (label(l))
    lbl = true;
  if (!set_begin(indented)) {

    if (lbl)
      set_error(" error: label not followed by a structure");
    return false;
  }
  if (lbl)
    addLocalReference(l, write_index, Class(SET));
  indent(false);
  uint16 content_write_index = 0;
  if (write) {

    current_object_->code_[write_index] = Atom::IPointer(extent_index);
    uint8 element_count = set_element_count(indented);
    current_object_->code_[extent_index++] = Atom::Set(element_count);
    content_write_index = extent_index;
    extent_index += element_count;
  }
  uint16 count = 0;
  bool _indented = false;
  while (!in_stream_->eof()) {

    if (set_end(indented)) {

      if (!count) {

        set_error(" syntax error: use |[] for empty sets");
        return false;
      }
      return true;
    }
    if (count) {

      if (!_indented) {

        if (!right_indent(true)) {

          if (!separator(false)) {

            set_error("syntax error: missing separator between 2 elements");
            return false;
          }
        }
      } else
        _indented = false;
    }
    if (!read_any(_indented, false, NULL, content_write_index + count, extent_index, write)) {

      set_error(" error: illegal element in set");
      return false;
    }
    count++;
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

bool Compiler::set(bool &indented, const Class &p, uint16 write_index, uint16 &extent_index, bool write) { // for class defs like member-name:[member-list] or !class (name[] member-list).

  std::streampos i = in_stream_->tellg();
  bool lbl = false;
  std::string l;
  if (label(l))
    lbl = true;
  if (!set_begin(indented)) {

    if (lbl)
      set_error(" error: label not followed by a structure");
    return false;
  }
  if (lbl)
    addLocalReference(l, write_index, p);
  indent(false);
  uint16 content_write_index = 0;
  if (write) {

    current_object_->code_[write_index] = Atom::IPointer(extent_index);
    uint8 element_count;
    if (p.atom_.getDescriptor() == Atom::S_SET && p.use_as != StructureMember::I_SET) {

      element_count = p.atom_.getAtomCount();
      current_object_->code_[extent_index++] = p.atom_;
    } else {

      element_count = set_element_count(indented);
      current_object_->code_[extent_index++] = Atom::Set(element_count);
    }
    content_write_index = extent_index;
    extent_index += element_count;
  }
  uint16 count = 0;
  bool _indented = false;
  uint16 arity = 0xFFFF;
  if (p.use_as == StructureMember::I_CLASS) { // undefined arity for unstructured sets.

    arity = p.atom_.getAtomCount();
    if (write) // fill up with wildcards that will be overwritten up to ::.
      for (uint16 j = content_write_index; j < content_write_index + arity; ++j)
        current_object_->code_[j] = Atom::Wildcard();
  }
  while (!in_stream_->eof()) {

    if (set_end(indented)) {

      if (!count) {

        set_error(" syntax error: use |[] for empty sets");
        return false;
      }
      if (count == arity || arity == 0xFFFF)
        return true;
      if (state_.no_arity_check) {

        state_.no_arity_check = false;
        return true;
      }
      set_arity_error(arity, count);
      return false;
    }
    if (count >= arity) {

      set_arity_error(arity, count + 1);
      return false;
    }
    if (count) {

      if (!_indented) {

        if (!right_indent(true)) {

          if (!separator(false)) {

            set_error("syntax error: missing separator between 2 elements");
            return false;
          }
        }
      } else
        _indented = false;
    }
    bool r;
    switch (p.use_as) {
    case StructureMember::I_EXPRESSION:
      r = read_expression(_indented, true, &p, content_write_index + count, extent_index, write);
      break;
    case StructureMember::I_SET:
    {
      Class _p = p;
      _p.use_as = StructureMember::I_CLASS;
      r = read_set(_indented, true, &_p, content_write_index + count, extent_index, write);
      break;
    }
    case StructureMember::I_CLASS:
      r = read(p.things_to_read[count], _indented, true, content_write_index + count, extent_index, write);
      break;
    case StructureMember::I_DCLASS:
      r = read_class(_indented, true, NULL, content_write_index + count, extent_index, write);
      break;
    }
    if (!r) {

      set_error(" error: illegal element in set");
      return false;
    }
    count++;
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

uint8 Compiler::set_element_count(bool indented) { // error checking is done in set(). This is a naive implementation: basically it parses the whole set depth-first. That's very slow and shall be replaced by a more clever design (avoiding deliving in the depths of the elements of the set).

  Output = false;
  uint8 count = 0;
  State s = save_state();
  indent(false);
  bool _indented = false;
  uint16 unused_index = 0;
  while (!in_stream_->eof()) {

    if (set_end(indented))
      break;
    if (count) {

      if (!_indented) {

        if (!right_indent(true)) {

          if (!separator(false))
            break;
        }
      } else
        _indented = false;
    }
    if (!read_any(_indented, false, NULL, 0, unused_index, false))
      break;
    count++;
  }
  in_stream_->clear();
  restore_state(s);
  Output = true;
  return count;
}

////////////////////////////////////////////////////////////////////////////////////////////////

bool Compiler::read_any(bool &indented, bool enforce, const Class *p, uint16 write_index, uint16 &extent_index, bool write) { // enforce always false, p always NULL.

  indented = false;
  if (read_number(indented, false, NULL, write_index, extent_index, write))
    return true;
  if (err_)
    return false;
  if (read_timestamp(indented, false, NULL, write_index, extent_index, write))
    return true;
  if (err_)
    return false;
  if (read_string(indented, false, NULL, write_index, extent_index, write))
    return true;
  if (err_)
    return false;
  if (read_boolean(indented, false, NULL, write_index, extent_index, write))
    return true;
  if (err_)
    return false;
  if (read_function(indented, false, NULL, write_index, extent_index, write))
    return true;
  if (err_)
    return false;
  if (read_node(indented, false, NULL, write_index, extent_index, write))
    return true;
  if (err_)
    return false;
  if (read_device(indented, false, NULL, write_index, extent_index, write))
    return true;
  if (read_class(indented, false, NULL, write_index, extent_index, write))
    return true;
  if (err_)
    return false;
  if (read_expression(indented, false, NULL, write_index, extent_index, write))
    return true;
  if (err_)
    return false;
  if (read_set(indented, false, NULL, write_index, extent_index, write))
    return true;
  if (write)
    set_error(" error: expecting more elements");
  return false;
}

bool Compiler::read_number(bool &indented, bool enforce, const Class *p, uint16 write_index, uint16 &extent_index, bool write) { // p always NULL

  if (read_nil_nb(write_index, extent_index, write))
    return true;
  if (read_forever_nb(write_index, extent_index, write))
    return true;
  if (read_variable(write_index, extent_index, write, NUMBER))
    return true;
  if (read_reference(write_index, extent_index, write, NUMBER))
    return true;
  if (read_wildcard(write_index, extent_index, write))
    return true;
  if (read_tail_wildcard(write_index, extent_index, write))
    return true;

  float32 n;
  if (number(n)) {

    if (write)
      current_object_->code_[write_index] = Atom::Float(n);
    return true;
  }
  State s = save_state();
  if (expression(indented, NUMBER, write_index, extent_index, write))
    return true;
  restore_state(s);
  if (enforce) {

    set_error(" error: expected a number or an expr evaluating to a number");
    return false;
  }
  return false;
}

bool Compiler::read_boolean(bool &indented, bool enforce, const Class *p, uint16 write_index, uint16 &extent_index, bool write) { // p always NULL

  if (read_nil_bl(write_index, extent_index, write))
    return true;
  if (read_variable(write_index, extent_index, write, BOOLEAN))
    return true;
  if (read_reference(write_index, extent_index, write, BOOLEAN))
    return true;
  if (read_wildcard(write_index, extent_index, write))
    return true;
  if (read_tail_wildcard(write_index, extent_index, write))
    return true;

  bool b;
  if (boolean(b)) {

    if (write)
      current_object_->code_[write_index] = Atom::Boolean(b);
    return true;
  }
  State s = save_state();
  if (expression(indented, BOOLEAN, write_index, extent_index, write))
    return true;
  restore_state(s);
  if (enforce) {

    set_error(" error: expected a Boolean or an expr evaluating to a Boolean");
    return false;
  }
  return false;
}

bool Compiler::read_timestamp(bool &indented, bool enforce, const Class *p, uint16 write_index, uint16 &extent_index, bool write) { // p always NULL

  if (read_nil_us(write_index, extent_index, write))
    return true;
  if (read_variable(write_index, extent_index, write, TIMESTAMP))
    return true;
  if (read_reference(write_index, extent_index, write, TIMESTAMP))
    return true;
  if (read_wildcard(write_index, extent_index, write))
    return true;
  if (read_tail_wildcard(write_index, extent_index, write))
    return true;

  uint64 ts;
  if (timestamp(ts)) {

    if (write) {

      current_object_->code_[write_index] = Atom::IPointer(extent_index);
      current_object_->code_[extent_index++] = Atom::Timestamp();
      current_object_->code_[extent_index++] = ts >> 32;
      current_object_->code_[extent_index++] = (ts & 0x00000000FFFFFFFF);
    }
    return true;
  }
  if (timestamp_s_ms_us(ts)) {
    if (write) {

      current_object_->code_[write_index] = Atom::IPointer(extent_index);
      current_object_->code_[extent_index++] = Atom::Timestamp();
      current_object_->code_[extent_index++] = ts >> 32;
      current_object_->code_[extent_index++] = (ts & 0x00000000FFFFFFFF);
    }
    return true;
  }

  State s = save_state();
  if (expression(indented, TIMESTAMP, write_index, extent_index, write))
    return true;
  restore_state(s);
  if (enforce) {

    set_error(" error: expected a timestamp or an expr evaluating to a timestamp");
    return false;
  }
  return false;
}

bool Compiler::read_string(bool &indented, bool enforce, const Class *p, uint16 write_index, uint16 &extent_index, bool write) { // p always NULL

  if (read_nil_st(write_index, extent_index, write))
    return true;
  if (read_variable(write_index, extent_index, write, STRING))
    return true;
  if (read_reference(write_index, extent_index, write, STRING))
    return true;
  if (read_wildcard(write_index, extent_index, write))
    return true;
  if (read_tail_wildcard(write_index, extent_index, write))
    return true;

  std::string st;
  if (str(st)) {

    if (write) {

      uint16 l = (uint16)st.length();
      current_object_->code_[write_index] = Atom::IPointer(extent_index);
      current_object_->code_[extent_index++] = Atom::String(l);
      uint32 _st = 0;
      int8 shift = 0;
      for (uint16 i = 0; i < l; ++i) {

        _st |= st[i] << shift;
        shift += 8;
        if (shift == 32) {

          current_object_->code_[extent_index++] = _st;
          _st = 0;
          shift = 0;
        }
      }
      if (l % 4)
        current_object_->code_[extent_index++] = _st;
    }
    return true;
  }
  State s = save_state();
  if (expression(indented, STRING, write_index, extent_index, write))
    return true;
  restore_state(s);
  if (enforce) {

    set_error(" error: expected a string");
    return false;
  }
  return false;
}

bool Compiler::read_node(bool &indented, bool enforce, const Class *p, uint16 write_index, uint16 &extent_index, bool write) { // p always NULL

  if (read_nil_nid(write_index, extent_index, write))
    return true;
  if (read_variable(write_index, extent_index, write, NODE_ID))
    return true;
  if (read_reference(write_index, extent_index, write, NODE_ID))
    return true;
  if (read_wildcard(write_index, extent_index, write))
    return true;
  if (read_tail_wildcard(write_index, extent_index, write))
    return true;

  std::streampos i = in_stream_->tellg();
  uint32 h;
  if (hex(h) && Atom(h).getDescriptor() == Atom::NODE) {

    if (write)
      current_object_->code_[write_index] = Atom::Atom(h);
    return true;
  }
  in_stream_->seekg(i);
  State s = save_state();
  if (expression(indented, NODE_ID, write_index, extent_index, write))
    return true;
  restore_state(s);
  if (enforce) {

    set_error(" error: expected a node id");
    return false;
  }
  return false;
}

bool Compiler::read_device(bool &indented, bool enforce, const Class *p, uint16 write_index, uint16 &extent_index, bool write) { // p always NULL.

  if (read_nil_did(write_index, extent_index, write))
    return true;
  if (read_variable(write_index, extent_index, write, DEVICE_ID))
    return true;
  if (read_reference(write_index, extent_index, write, DEVICE_ID))
    return true;
  if (read_wildcard(write_index, extent_index, write))
    return true;
  if (read_tail_wildcard(write_index, extent_index, write))
    return true;

  std::streampos i = in_stream_->tellg();
  uint32 h;
  if (hex(h) && Atom(h).getDescriptor() == Atom::DEVICE) {

    if (write)
      current_object_->code_[write_index] = Atom::Atom(h);
    return true;
  }
  in_stream_->seekg(i);
  State s = save_state();
  if (expression(indented, DEVICE_ID, write_index, extent_index, write))
    return true;
  restore_state(s);
  if (enforce) {

    set_error(" error: expected a device id");
    return false;
  }
  return false;
}

bool Compiler::read_function(bool &indented, bool enforce, const Class *p, uint16 write_index, uint16 &extent_index, bool write) { // p always NULL

  if (read_nil_fid(write_index, extent_index, write))
    return true;
  if (read_variable(write_index, extent_index, write, FUNCTION_ID))
    return true;
  if (read_reference(write_index, extent_index, write, FUNCTION_ID))
    return true;
  if (read_wildcard(write_index, extent_index, write))
    return true;
  if (read_tail_wildcard(write_index, extent_index, write))
    return true;

  Class _p;
  if (function(_p)) { // TODO: _p shall be used to parse the args in the embedding expression

    if (write)
      current_object_->code_[write_index] = _p.atom_;
    return true;
  }
  State s = save_state();
  if (expression(indented, FUNCTION_ID, write_index, extent_index, write))
    return true;
  restore_state(s);
  if (enforce) {

    set_error(" error: expected a device function");
    return false;
  }
  return false;
}

bool Compiler::read_expression(bool &indented, bool enforce, const Class *p, uint16 write_index, uint16 &extent_index, bool write) {

  if (read_nil(write_index, extent_index, write))
    return true;
  if (p && p->str_opcode != Class::Expression) {

    if (read_variable(write_index, extent_index, write, *p))
      return true;
    if (read_reference(write_index, extent_index, write, p->type))
      return true;
  } else {

    if (read_variable(write_index, extent_index, write, Class()))
      return true;
    if (read_reference(write_index, extent_index, write, ANY))
      return true;
  }
  if (read_wildcard(write_index, extent_index, write))
    return true;
  if (read_tail_wildcard(write_index, extent_index, write))
    return true;

  indented = false;
  if (p && p->str_opcode != Class::Expression) {

    if (expression(indented, *p, write_index, extent_index, write))
      return true;
  } else if (expression(indented, ANY, write_index, extent_index, write))
    return true;
  if (enforce) {

    std::string s = " error: expected an expression";
    if (p) {

      s += " of type: ";
      s += p->str_opcode;
    }
    set_error(s);
    return false;
  }
  return false;
}

bool Compiler::read_set(bool &indented, bool enforce, const Class *p, uint16 write_index, uint16 &extent_index, bool write) {

  if (read_nil_set(write_index, extent_index, write))
    return true;
  if (read_variable(write_index, extent_index, write, Class(SET)))
    return true;
  if (read_reference(write_index, extent_index, write, SET))
    return true;
  if (read_wildcard(write_index, extent_index, write))
    return true;
  if (read_tail_wildcard(write_index, extent_index, write))
    return true;

  indented = false;
  if (p) {

    if (set(indented, *p, write_index, extent_index, write))
      return true;
  } else if (set(indented, write_index, extent_index, write))
    return true;
  if (enforce) {

    set_error(" error: expected a set");
    return false;
  }
  return false;
}

bool Compiler::read_class(bool &indented, bool enforce, const Class *p, uint16 write_index, uint16 &extent_index, bool write) { // p always NULL.

  std::streampos i = in_stream_->tellg();
  std::string l;
  if (label(l)) {

    Class _p;
    if (!object(_p))
      if (!marker(_p))
        return false;
    local_references_[l] = Reference(write_index, _p, Class());
    if (write)
      current_object_->code_[write_index] = _p.atom_;
    return true;
  }
  in_stream_->clear();
  in_stream_->seekg(i);
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////

bool Compiler::read_nil(uint16 write_index, uint16 &extent_index, bool write) {

  if (nil()) {

    if (write)
      current_object_->code_[write_index] = Atom::Nil();
    return true;
  }
  return false;
}

bool Compiler::read_nil_set(uint16 write_index, uint16 &extent_index, bool write) {

  std::streampos i = in_stream_->tellg();
  if (match_symbol("|[]", false)) {

    if (write) {

      current_object_->code_[write_index] = Atom::IPointer(extent_index);
      current_object_->code_[extent_index++] = Atom::Set(0);
    }
    return true;
  }
  in_stream_->seekg(i);
  return false;
}

bool Compiler::read_nil_nb(uint16 write_index, uint16 &extent_index, bool write) {

  if (nil_nb()) {

    if (write)
      current_object_->code_[write_index] = Atom::UndefinedFloat();
    return true;
  }
  return false;
}

bool Compiler::read_nil_us(uint16 write_index, uint16 &extent_index, bool write) {

  if (nil_us()) {

    if (write) {

      current_object_->code_[write_index] = Atom::IPointer(extent_index);
      current_object_->code_[extent_index++] = Atom::UndefinedTimestamp();
      current_object_->code_[extent_index++] = 0xFFFFFFFF;
      current_object_->code_[extent_index++] = 0xFFFFFFFF;
    }
    return true;
  }
  return false;
}

bool Compiler::read_forever_nb(uint16 write_index, uint16 &extent_index, bool write) {

  if (forever()) {

    if (write) {

      current_object_->code_[write_index] = Atom::PlusInfinity();
    }
    return true;
  }
  return false;
}

bool Compiler::read_nil_nid(uint16 write_index, uint16 &extent_index, bool write) {

  if (nil_nid()) {

    if (write)
      current_object_->code_[write_index] = Atom::UndefinedNode();
    return true;
  }
  return false;
}

bool Compiler::read_nil_did(uint16 write_index, uint16 &extent_index, bool write) {

  if (nil_did()) {

    if (write)
      current_object_->code_[write_index] = Atom::UndefinedDevice();
    return true;
  }
  return false;
}

bool Compiler::read_nil_fid(uint16 write_index, uint16 &extent_index, bool write) {

  if (nil_fid()) {

    if (write)
      current_object_->code_[write_index] = Atom::UndefinedDeviceFunction();
    return true;
  }
  return false;
}

bool Compiler::read_nil_bl(uint16 write_index, uint16 &extent_index, bool write) {

  if (nil_bl()) {

    if (write)
      current_object_->code_[write_index] = Atom::UndefinedBoolean();
    return true;
  }
  return false;
}

bool Compiler::read_nil_st(uint16 write_index, uint16 &extent_index, bool write) {

  if (nil_st()) {

    if (write)
      current_object_->code_[write_index] = Atom::UndefinedString();
    return true;
  }
  return false;
}

bool Compiler::read_variable(uint16 write_index, uint16 &extent_index, bool write, const Class p) {

  std::string v;
  if (variable(v)) {

    if (state_.pattern_lvl || allow_variables_and_wildcards_outside_pattern_skeleton_) {

      if (in_hlp_) {

        uint8 variable_index = add_hlp_reference(v);
        if (write)
          current_object_->code_[write_index] = Atom::VLPointer(variable_index);
      } else {

        addLocalReference(v, write_index, p);
        if (write)
          current_object_->code_[write_index] = Atom::Wildcard(p.atom_.asOpcode()); // useless in skeleton expressions (already filled up in expression_head); usefull when the skeleton itself is a variable
      }
      return true;
    } else {

      set_error(" error: no variables allowed outside a pattern skeleton");
      return false;
    }
  }
  return false;
}

bool Compiler::read_reference(uint16 write_index, uint16 &extent_index, bool write, const ReturnType t) {

  uint16 index;
  if ((t == ANY || (t != ANY && current_class_.type == t)) && this_()) {

    if (write)
      current_object_->code_[write_index] = Atom::This();
    return true;
  }
  if (local_reference(index, t)) {

    if (write)
      current_object_->code_[write_index] = Atom::CodeVLPointer(index); // local references are always pointing to the value array
    return true;
  }
  if (global_reference(index, t)) { // index is the index held by a reference pointer

    if (write)
      current_object_->code_[write_index] = Atom::RPointer(index);
    return true;
  }
  vector<int16> v;
  if (this_indirection(v, t)) {

    if (write) {

      current_object_->code_[write_index] = Atom::IPointer(extent_index);
      current_object_->code_[extent_index++] = Atom::CPointer(v.size() + 1);
      current_object_->code_[extent_index++] = Atom::This();
      for (uint16 i = 0; i < v.size(); ++i) {

        switch (v[i]) {
        case -1:
          current_object_->code_[extent_index++] = Atom::View();
          break;
        case -2:
          current_object_->code_[extent_index++] = Atom::Mks();
          break;
        case -3:
          current_object_->code_[extent_index++] = Atom::Vws();
          break;
        default:
          current_object_->code_[extent_index++] = Atom::IPointer(v[i]);
          break;
        }
      }
    }
    return true;
  }
  uint16 cast_opcode;
  if (local_indirection(v, t, cast_opcode)) {

    if (write) {

      current_object_->code_[write_index] = Atom::IPointer(extent_index);
      current_object_->code_[extent_index++] = Atom::CPointer(v.size());
      current_object_->code_[extent_index++] = Atom::CodeVLPointer(v[0], cast_opcode);
      for (uint16 i = 1; i < v.size(); ++i) {

        switch (v[i]) {
        case -1:
          current_object_->code_[extent_index++] = Atom::View();
          break;
        case -2:
          current_object_->code_[extent_index++] = Atom::Mks();
          break;
        case -3:
          current_object_->code_[extent_index++] = Atom::Vws();
          break;
        default:
          current_object_->code_[extent_index++] = Atom::IPointer(v[i]);
          break;
        }
      }
    }
    return true;
  }
  if (global_indirection(v, t)) { // v[0] is the index held by a reference pointer

    if (write) {

      current_object_->code_[write_index] = Atom::IPointer(extent_index);
      current_object_->code_[extent_index++] = Atom::CPointer(v.size());
      current_object_->code_[extent_index++] = Atom::RPointer(v[0]);
      for (uint16 i = 1; i < v.size(); ++i) {

        switch (v[i]) {
        case -2:
          current_object_->code_[extent_index++] = Atom::Mks();
          break;
        case -3:
          current_object_->code_[extent_index++] = Atom::Vws();
          break;
        default:
          current_object_->code_[extent_index++] = Atom::IPointer(v[i]);
          break;
        }
      }
    }
    return true;
  }
  if (hlp_reference(index)) {

    if (write)
      current_object_->code_[write_index] = Atom::VLPointer(index);
    return true;
  }
  return false;
}

bool Compiler::read_wildcard(uint16 write_index, uint16 &extent_index, bool write) {

  if (wildcard()) {

    if (state_.pattern_lvl || allow_variables_and_wildcards_outside_pattern_skeleton_) {

      if (write)
        current_object_->code_[write_index] = Atom::Wildcard();
      return true;
    } else {

      set_error(" error: no wildcards allowed outside a pattern skeleton");
      return false;
    }
  }
  return false;
}

bool Compiler::read_tail_wildcard(uint16 write_index, uint16 &extent_index, bool write) {

  if (tail_wildcard()) {

    if (state_.pattern_lvl) {

      if (write)
        current_object_->code_[write_index] = Atom::TailWildcard();
      return true;
    } else {

      set_error(" error: no wildcards allowed outside a pattern skeleton");
      return false;
    }
  }
  return false;
}

std::string Compiler::getObjectName(const uint16 index) const {

  unordered_map<std::string, Reference>::const_iterator r;
  for (r = global_references_.begin(); r != global_references_.end(); ++r) {

    if (r->second.index_ == index)
      return r->first;
  }
  std::string s;
  return s;
}
}
