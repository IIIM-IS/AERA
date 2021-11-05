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

#include <algorithm>
#include <regex>
#include "decompiler.h"
#include "../submodules/CoreLibrary/CoreLibrary/utils.h"

using namespace std::chrono;
using namespace r_code;

namespace r_comp {

// Get a string for the value such as "a", "b" ... "z", "aa", "ab" ....
static std::string make_suffix(uint32 value) {
  string result;
  do {
    // Get the lowest digit as base 26 and prepend a char from 'a' to 'z'.
    uint32 lowest = value % 26;
    result = (char)('a' + lowest) + result;

    // Shift.
    value /= 26;
  } while (value != 0);

  return result;
}

Decompiler::Decompiler() : out_stream_(NULL), current_object_(NULL), metadata_(NULL), image_(NULL), in_hlp_(false) {
  // Make hlp_postfix_ true by default to put ':' after variables, unless it is set false for guards in cst and mdl.
  hlp_postfix_ = true;
}

Decompiler::~Decompiler() {

  if (out_stream_)
    delete out_stream_;
}

std::string Decompiler::get_variable_name(uint16 index, bool postfix) {

  unordered_map<uint16, std::string>::iterator it = variable_names_.find(index);
  if (it == variable_names_.end()) {

    std::string s = "v" + std::to_string(last_variable_id_++);
    variable_names_[index] = s;
    if (postfix)
      s += ':';
    out_stream_->insert(index, s);
    if (postfix)
      return s.substr(0, s.length() - 1);
    return s;
  }
  return it->second;
}

std::string Decompiler::get_hlp_variable_name(uint16 index) {

  std::string s = "v" + std::to_string(index);
  if (hlp_postfix_)
    s += ':';
  return s;
}

std::string Decompiler::get_object_name(uint16 index) {

  std::string s;
  unordered_map<uint16, std::string>::iterator it = object_names_.find(index);
  if (it == object_names_.end()) {

    s = "unknown-object";
    return s;
  }
  return it->second;
}

void Decompiler::init(r_comp::Metadata *metadata) {

  metadata_ = metadata;
  time_offset_ = microseconds(0);

  partial_decompilation_ = false;
  ignore_named_objects_ = false;

  // Load the renderers;
  for (uint16 i = 0; i < metadata->classes_by_opcodes_.size(); ++i) {

    Class *c = metadata->get_class(i);
    std::string class_name = c->str_opcode;
    size_t p = class_name.find("mk.");
    if (p != std::string::npos)
      renderers_[i] = &Decompiler::write_marker;
    else if (class_name == "grp")
      renderers_[i] = &Decompiler::write_group;
    else if (class_name == "ipgm" || class_name == "icpp_pgm")
      renderers_[i] = &Decompiler::write_ipgm;
    else if (class_name == "pgm" || class_name == "|pgm")
      renderers_[i] = &Decompiler::write_pgm;
    else if (class_name == "icmd")
      renderers_[i] = &Decompiler::write_icmd;
    else if (class_name == "cmd")
      renderers_[i] = &Decompiler::write_cmd;
    else if (class_name == "fact" || class_name == "|fact")
      renderers_[i] = &Decompiler::write_fact;
    else if (class_name == "cst" || class_name == "mdl")
      renderers_[i] = &Decompiler::write_hlp;
    else if (class_name == "icst" || class_name == "imdl")
      renderers_[i] = &Decompiler::write_ihlp;
    else if (class_name == "sim")
      renderers_[i] = &Decompiler::write_sim;
    else
      renderers_[i] = &Decompiler::write_expression;
  }
}

uint32 Decompiler::decompile(r_comp::Image *image, std::ostringstream *stream, Timestamp::duration time_offset, bool ignore_named_objects) {

  ignore_named_objects_ = ignore_named_objects;

  uint32 object_count = decompile_references(image);

  for (uint16 i = 0; i < image->code_segment_.objects_.size(); ++i)
    decompile_object(i, stream, time_offset);

  return object_count;
}

uint32 Decompiler::decompile(r_comp::Image *image, std::ostringstream *stream, Timestamp::duration time_offset, std::vector<SysObject *> &imported_objects,
  bool include_oid, bool include_label, bool include_views) {

  partial_decompilation_ = true;
  ignore_named_objects_ = true;
  imported_objects_ = imported_objects;

  uint32 object_count = decompile_references(image);

  for (uint16 i = 0; i < image->code_segment_.objects_.size(); ++i)
    decompile_object(i, stream, time_offset, include_oid, include_label, include_views);

  return object_count;
}

uint32 Decompiler::decompile_references(r_comp::Image *image, unordered_map<uint16, std::string>* object_names) {

  if (object_names) {
    // Pre-populate object_names_ and object_indices_.
    for (auto entry = object_names->begin(); entry != object_names->end(); ++entry) {
      object_names_[entry->first] = entry->second;
      object_indices_[entry->second] = entry->first;
    }
  }

  unordered_map<const Class *, uint16> object_ID_per_class;
  unordered_map<std::string, Class>::const_iterator it;
  for (it = metadata_->sys_classes_.begin(); it != metadata_->sys_classes_.end(); ++it)
    object_ID_per_class[&(it->second)] = 0;

  std::string s;

  image_ = image;

  // populate object names first so they can be referenced in any order.
  // First pass: Add the user-defined names to object_names_ and object_indices_.
  for (uint16 i = 0; i < image->code_segment_.objects_.size(); ++i) {

    SysObject *sys_object = (SysObject *)image->code_segment_.objects_[i];
    unordered_map<uint32, std::string>::const_iterator n = image->object_names_.symbols_.find(sys_object->oid_);
    if (n != image->object_names_.symbols_.end()) {

      s = n->second;
      named_objects_.insert(sys_object->oid_);

      if (object_names_[i] != "" && object_names_[i] != s)
        // The image->object_names_.symbols_ for the OID is different than the name in the provided object_names.
        // (We don't expect this to happen.)
        return 0;

      object_names_[i] = s;
      object_indices_[s] = i;
    }
  }

  regex verticalBarRegex("\\|");
  // Second pass: Create names for the remaining objects, making sure they are unique.
  for (uint16 i = 0; i < image->code_segment_.objects_.size(); ++i) {
    SysObject *sys_object = (SysObject *)image->code_segment_.objects_[i];

    unordered_map<uint32, std::string>::const_iterator n = image->object_names_.symbols_.find(sys_object->oid_);
    if (n != image->object_names_.symbols_.end())
      // Already set the user-defined name in the first pass.
      continue;

    Class *c = metadata_->get_class(sys_object->code_[0].asOpcode());
    string className = c->str_opcode;
    // A class name like mk.val has a dot, but this isn't allowed as an identifier.
    replace(className.begin(), className.end(), '.', '_');
    // A class name like |fact has a bar, but this isn't allowed as an identifier.
    // (Use regex_replace because it can handle multi-character strings.)
    className = regex_replace(className, verticalBarRegex, "anti_");

    if (sys_object->oid_ != UNDEFINED_OID)
      // Use the object's OID.
      s = className + "_" + std::to_string(sys_object->oid_);
    else {
      // Create a name with a unique ID.
      uint16 last_object_ID = object_ID_per_class[c];
      object_ID_per_class[c] = last_object_ID + 1;
      s = className + std::to_string(last_object_ID);
    }

    if (object_indices_.find(s) != object_indices_.end()) {
      // The created name matches an existing name. Keep trying an added
      // suffix until it is unique.
      for (uint32 value = 1; true; ++value) {
        std::string new_s = s + make_suffix(value);
        if (object_indices_.find(new_s) == object_indices_.end()) {
          s = new_s;
          break;
        }
      }
    }

    object_names_[i] = s;
    object_indices_[s] = i;
  }

  closing_set_ = false;

  return image->code_segment_.objects_.size();
}

void Decompiler::decompile_object(
  uint16 object_index, std::ostringstream *stream, Timestamp::duration time_offset, bool include_oid,
  bool include_label, bool include_views) {

  if (!out_stream_)
    out_stream_ = new OutStream(stream);
  else if (out_stream_->stream_ != stream) {

    delete out_stream_;
    out_stream_ = new OutStream(stream);
  }

  time_offset_ = duration_cast<microseconds>(time_offset);

  variable_names_.clear();
  last_variable_id_ = 0;

  current_object_ = image_->code_segment_.objects_[object_index];
  SysObject *sys_object = (SysObject *)current_object_;
  uint16 read_index = 0;
  bool after_tail_wildcard = false;
  indents_ = 0;

  if (!partial_decompilation_ && ignore_named_objects_) { // decompilation of the entire memory.

    if (named_objects_.find(sys_object->oid_) != named_objects_.end())
      return;
  }
  else { // decompiling on-the-fly: ignore named objects only if imported.

    bool imported = false;
    for (uint32 i = 0; i < imported_objects_.size(); ++i) {

      if (sys_object == imported_objects_[i]) {

        imported = true;
        break;
      }
    }

    if (imported) {

      if (named_objects_.find(sys_object->oid_) != named_objects_.end())
        return;
      else
        *out_stream_ << "imported";
    }
    else {
      if (include_oid) {
        if (sys_object->oid_ != UNDEFINED_OID)
          *out_stream_ << sys_object->oid_;
#ifdef WITH_DETAIL_OID
        *out_stream_ << "(" << sys_object->detail_oid_ << ") ";
#else
        if (sys_object->oid_ != UNDEFINED_OID)
          *out_stream_ << " ";
#endif
      }
    }
  }

  if (include_label) {
    std::string s = object_names_[object_index];
    s += ":";
    *out_stream_ << s;
  }

  horizontal_set_ = false;

  (this->*renderers_[current_object_->code_[read_index].asOpcode()])(read_index);

  if (include_views) {
    uint16 view_count = sys_object->views_.size();
    if (view_count) { // write the set of views

      *out_stream_ << " []";
      for (uint16 i = 0; i < view_count; ++i) {

        write_indent(3);
        current_object_ = sys_object->views_[i];
        write_view(0, current_object_->code_[0].getAtomCount());
      }
    }
    else
      *out_stream_ << " |[]";
  }
  write_indent(0);
  write_indent(0);
}

void Decompiler::decompile_object(const std::string object_name, std::ostringstream *stream, Timestamp::duration time_offset) {

  decompile_object(object_indices_[object_name], stream, time_offset);
}

void Decompiler::write_indent(uint16 i) {

  *out_stream_ << NEWLINE;
  indents_ = i;
  for (uint16 j = 0; j < indents_; j++)
    *out_stream_ << ' ';
}

void Decompiler::write_expression_head(uint16 read_index) {

  switch (current_object_->code_[read_index].getDescriptor()) {
  case Atom::OPERATOR:
    *out_stream_ << metadata_->operator_names_[current_object_->code_[read_index].asOpcode()];
    break;
  case Atom::OBJECT:
  case Atom::MARKER:
    *out_stream_ << metadata_->class_names_[current_object_->code_[read_index].asOpcode()];
    break;
  case Atom::INSTANTIATED_PROGRAM:
  case Atom::INSTANTIATED_INPUT_LESS_PROGRAM:
  case Atom::INSTANTIATED_ANTI_PROGRAM:
    *out_stream_ << "ipgm";
    break;
  case Atom::INSTANTIATED_CPP_PROGRAM:
    *out_stream_ << "icpp_pgm";
    break;
  case Atom::COMPOSITE_STATE:
    *out_stream_ << "cst";
    break;
  case Atom::MODEL:
    *out_stream_ << "mdl";
    break;
  case Atom::GROUP:
    *out_stream_ << "grp";
    break;
  default:
    *out_stream_ << "undefined-class";
    break;
  }
}

void Decompiler::write_expression_tail(uint16 read_index, bool apply_time_offset, bool vertical) { // read_index points initially to the head.

  uint16 arity = current_object_->code_[read_index].getAtomCount();
  bool after_tail_wildcard = false;

  for (uint16 i = 0; i < arity; ++i) {

    if (after_tail_wildcard)
      write_any(++read_index, after_tail_wildcard, apply_time_offset);
    else {

      if (closing_set_) {

        closing_set_ = false;
        if (!horizontal_set_)
          write_indent(indents_);
        else
          *out_stream_ << ' ';
      } else if (!vertical)
        *out_stream_ << ' ';

      write_any(++read_index, after_tail_wildcard, apply_time_offset);

      if (!closing_set_ && vertical)
        *out_stream_ << NEWLINE;
    }
  }
}

void Decompiler::write_expression(uint16 read_index) {

  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  out_stream_->push('(', read_index);
  write_expression_head(read_index);
  write_expression_tail(read_index, true);
  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  *out_stream_ << ')';
}

void Decompiler::write_group(uint16 read_index) {

  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  out_stream_->push('(', read_index);
  write_expression_head(read_index);
  write_expression_tail(read_index, false);
  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  *out_stream_ << ')';
}

void Decompiler::write_marker(uint16 read_index) {

  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  out_stream_->push('(', read_index);
  write_expression_head(read_index);
  write_expression_tail(read_index, false);
  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  *out_stream_ << ')';
}

void Decompiler::write_pgm(uint16 read_index) {

  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  out_stream_->push('(', read_index);
  write_expression_head(read_index);
  write_expression_tail(read_index, true);
  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  *out_stream_ << ')';
}


void Decompiler::write_ipgm(uint16 read_index) {

  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  out_stream_->push('(', read_index);
  write_expression_head(read_index);
  write_expression_tail(read_index, false);
  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  *out_stream_ << ')';
}

void Decompiler::write_hlp(uint16 read_index) {

  in_hlp_ = true;
  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  out_stream_->push('(', read_index);
  write_expression_head(read_index);
  *out_stream_ << " ";

  uint16 arity = current_object_->code_[read_index].getAtomCount();
  bool after_tail_wildcard = false;

  for (uint16 i = 0; i < arity; ++i) {

    if (after_tail_wildcard)
      write_any(++read_index, after_tail_wildcard, false);
    else {

      if (closing_set_) {

        closing_set_ = false;
        write_indent(indents_);
      }

      if (!(i == 0 || i == 1))
        // Put a colon after variables for template arguments and the facts, but not after further
        // element such as guards.
        hlp_postfix_ = false;
      bool save_horizontal_set = horizontal_set_;
      if (i == 0 || i == 4)
        // Write the set of template arguments and set of output groups horizontally.
        horizontal_set_ = true;
      write_any(++read_index, after_tail_wildcard, false);
      hlp_postfix_ = true;
      horizontal_set_ = save_horizontal_set;

      if (!closing_set_)
        *out_stream_ << NEWLINE;
    }
  }

  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  *out_stream_ << ')';
  in_hlp_ = false;
}

void Decompiler::write_ihlp(uint16 read_index) {

  if (!in_hlp_)
    horizontal_set_ = true;
  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  out_stream_->push('(', read_index);
  write_expression_head(read_index);
  write_expression_tail(read_index, true);
  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  *out_stream_ << ')';
  if (!in_hlp_)
    horizontal_set_ = false;
}

void Decompiler::write_icmd(uint16 read_index) {

  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  out_stream_->push('(', read_index);
  write_expression_head(read_index);
  //write_expression_tail(read_index,true);

  uint16 write_as_view_index = 0;
  if (current_object_->code_[read_index + 1].asOpcode() == metadata_->classes_.find("_inj")->second.atom_.asOpcode()) {

    uint16 arg_set_index = current_object_->code_[read_index + 2].asIndex(); // 2 args for _inj; the view is the second.
    write_as_view_index = current_object_->code_[arg_set_index + 2].asIndex();
  }

  uint16 arity = current_object_->code_[read_index].getAtomCount();
  bool after_tail_wildcard = false;

  for (uint16 i = 0; i < arity; ++i) {

    if (after_tail_wildcard)
      write_any(++read_index, after_tail_wildcard, true);
    else {

      if (closing_set_) {

        closing_set_ = false;
        write_indent(indents_);
      } else
        *out_stream_ << ' ';

      write_any(++read_index, after_tail_wildcard, true, write_as_view_index);
    }
  }

  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  *out_stream_ << ')';
}

void Decompiler::write_cmd(uint16 read_index) {

  if (!in_hlp_)
    horizontal_set_ = true;
  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  out_stream_->push('(', read_index);
  write_expression_head(read_index);
  write_expression_tail(read_index, false);
  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  *out_stream_ << ')';
  if (!in_hlp_)
    horizontal_set_ = false;
}

void Decompiler::write_fact(uint16 read_index) {

  if (in_hlp_)
    horizontal_set_ = true;
  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  out_stream_->push('(', read_index);
  write_expression_head(read_index);
  write_expression_tail(read_index, true);
  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  *out_stream_ << ')';
  horizontal_set_ = false;
}

void Decompiler::write_view(uint16 read_index, uint16 arity) {

  if (arity > VIEW_CODE_MAX_SIZE || arity <= 1) {

    *out_stream_ << "nil";
    return;
  }

  bool after_tail_wildcard = false;

  *out_stream_ << "[";
  for (uint16 j = 1; j <= arity; ++j) {

    write_any(read_index + j, after_tail_wildcard, true);
    if (j < arity)
      *out_stream_ << " ";
  }
  *out_stream_ << "]";
}

void Decompiler::write_sim(uint16 read_index) {
  // Imitate write_expression, except set apply_time_offset false for the time horizon.

  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  out_stream_->push('(', read_index);
  write_expression_head(read_index);

  // Imitate write_expression_tail, except set apply_time_offset false for the time horizon.
  bool vertical = false;
  uint16 arity = current_object_->code_[read_index].getAtomCount();
  bool after_tail_wildcard = false;

  for (uint16 i = 0; i < arity; ++i) {
    // Don't use the time offset for the time horizon.
    bool apply_time_offset = (i != (SIM_THZ - 1));

    if (after_tail_wildcard)
      write_any(++read_index, after_tail_wildcard, apply_time_offset);
    else {

      if (closing_set_) {

        closing_set_ = false;
        if (!horizontal_set_)
          write_indent(indents_);
        else
          *out_stream_ << ' ';
      }
      else if (!vertical)
        *out_stream_ << ' ';

      write_any(++read_index, after_tail_wildcard, apply_time_offset);

      if (!closing_set_ && vertical)
        *out_stream_ << NEWLINE;
    }
  }

  if (closing_set_) {

    closing_set_ = false;
    write_indent(indents_);
  }
  *out_stream_ << ')';
}

void Decompiler::write_set(uint16 read_index, bool apply_time_offset, uint16 write_as_view_index) { // read_index points to a set atom.

  uint16 arity = current_object_->code_[read_index].getAtomCount();
  bool after_tail_wildcard = false;

  if (arity == 1) { // write [element]

    out_stream_->push('[', read_index);
    write_any(++read_index, after_tail_wildcard, apply_time_offset);
    *out_stream_ << ']';
  } else if (write_as_view_index > 0 && write_as_view_index == read_index)
    write_view(read_index, arity);
  else if (horizontal_set_) { // write [elements].

    out_stream_->push('[', read_index);
    for (uint16 i = 0; i < arity; ++i) {

      if (i > 0)
        *out_stream_ << ' ';
      if (after_tail_wildcard)
        write_any(++read_index, after_tail_wildcard, apply_time_offset);
      else
        write_any(++read_index, after_tail_wildcard, apply_time_offset, write_as_view_index);
    }
    *out_stream_ << ']';
    closing_set_ = true;
  } else { // write []+indented elements.

    out_stream_->push("[]", read_index);
    indents_ += 3;
    for (uint16 i = 0; i < arity; ++i) {

      if (after_tail_wildcard)
        write_any(++read_index, after_tail_wildcard, apply_time_offset);
      else {

        write_indent(indents_);
        write_any(++read_index, after_tail_wildcard, apply_time_offset, write_as_view_index);
      }
    }
    closing_set_ = true;
    indents_ -= 3; // don't call write_indents() here as the last set member can be a set.
  }
}

void Decompiler::write_any(uint16 read_index, bool &after_tail_wildcard, bool apply_time_offset, uint16 write_as_view_index) { // after_tail_wildcard meant to avoid printing ':' after "::".

  Atom a = current_object_->code_[read_index];

  if (a.isFloat()) {

    if (a.atom_ == 0x3FFFFFFF)
      out_stream_->push("|nb", read_index);
    else if (a == Atom::PlusInfinity())
      out_stream_->push("forever", read_index);
    else {

      *out_stream_ << std::dec;
      out_stream_->push(a.asFloat(), read_index);
    }
    return;
  }

  Atom atom;
  uint16 index;
  switch (a.getDescriptor()) {
  case Atom::VL_PTR:
    // JTNote: The opcode 0x0FFE doesn't seem to be used.
    if (a.asCastOpcode() == 0x0FFE)
      out_stream_->push(':', read_index);
    else {

      std::string s = get_hlp_variable_name(a.asIndex());
      out_stream_->push(s, read_index);
    }
    break;
  case Atom::ASSIGN_PTR:
    // Output the assignment index, then fall through to process like an I_PTR.
    out_stream_->push(get_hlp_variable_name(a.asAssignmentIndex()), read_index);
    *out_stream_ << ":";
  case Atom::CODE_VL_PTR:
    // Fall through to start processing like an I_PTR. Will print the get_variable_name() and break.
  case Atom::I_PTR:
    index = a.asIndex();
    atom = current_object_->code_[index];
    while (atom.getDescriptor() == Atom::I_PTR) {

      index = atom.asIndex();
      atom = current_object_->code_[index];
    }
    if (index < read_index) { // reference to a label or variable, including CODE_VL_PTR.

      std::string s = get_variable_name(index, atom.getDescriptor() != Atom::WILDCARD); // post-fix labels with ':' (no need for variables since they are inserted just before wildcards).
      out_stream_->push(s, read_index);
      break;
    }
    switch (atom.getDescriptor()) { // structures.
    case Atom::OBJECT:
    case Atom::MARKER:
    case Atom::GROUP:
    case Atom::INSTANTIATED_PROGRAM:
    case Atom::INSTANTIATED_INPUT_LESS_PROGRAM:
    case Atom::INSTANTIATED_ANTI_PROGRAM:
    case Atom::INSTANTIATED_CPP_PROGRAM:
    case Atom::COMPOSITE_STATE:
    case Atom::MODEL:
    case Atom::OPERATOR:
      (this->*renderers_[atom.asOpcode()])(index);
      break;
    case Atom::SET:
    case Atom::S_SET:
      if (atom.readsAsNil())
        out_stream_->push("|[]", read_index);
      else
        write_set(index, apply_time_offset, write_as_view_index);
      break;
    case Atom::STRING:
      if (atom.readsAsNil())
        out_stream_->push("|st", read_index);
      else {

        Atom first = current_object_->code_[index + 1];
        std::string s = Utils::GetString(&current_object_->code_[index]);
        *out_stream_ << '\"' << s << '\"';
      }
      break;
    case Atom::TIMESTAMP:
      if (atom.readsAsNil())
        out_stream_->push("|us", read_index);
      else {

        Atom first = current_object_->code_[index + 1];
        auto ts = Utils::GetMicrosecondsSinceEpoch(&current_object_->code_[index]);
        if (!in_hlp_ && ts.count() > 0 && apply_time_offset)
          ts -= time_offset_;
        out_stream_->push(Time::ToString_seconds(ts), read_index);
      }
      break;
    case Atom::C_PTR: {

      uint16 opcode;
      uint16 member_count = atom.getAtomCount();
      atom = current_object_->code_[index + 1]; // current_object_->code[index] is the cptr; lead atom is at index+1; iptrs start at index+2.
      switch (atom.getDescriptor()) {
      case Atom::THIS: // this always refers to an instantiated reactive object.
        out_stream_->push("this", read_index);
        opcode = metadata_->sys_classes_["ipgm"].atom_.asOpcode();
        break;
      case Atom::CODE_VL_PTR: {

        uint8 cast_opcode = atom.asCastOpcode();
        while (current_object_->code_[atom.asIndex()].getDescriptor() == Atom::I_PTR) // position to a structure or an atomic value_.
          atom = current_object_->code_[atom.asIndex()];
        out_stream_->push(get_variable_name(atom.asIndex(), current_object_->code_[atom.asIndex()].getDescriptor() != Atom::WILDCARD), read_index);
        if (cast_opcode == 0xFF) {

          if (current_object_->code_[atom.asIndex()].getDescriptor() == Atom::WILDCARD)
            opcode = current_object_->code_[atom.asIndex()].asOpcode();
          else
            opcode = current_object_->code_[atom.asIndex()].asOpcode();
        } else
          opcode = cast_opcode;
        break;
      }case Atom::R_PTR: {
        uint32 object_index = current_object_->references_[atom.asIndex()];
        out_stream_->push(get_object_name(object_index), read_index);
        opcode = image_->code_segment_.objects_[object_index]->code_[0].asOpcode();
        break;
      }default:
        out_stream_->push("unknown-cptr-lead-type", read_index);
        break;
      }

      Class embedding_class = metadata_->classes_by_opcodes_[opcode]; // class defining the members.
      for (uint16 i = 2; i <= member_count; ++i) { // get the class of the pointed structure and retrieve the member name from i.

        std::string member_name;
        atom = current_object_->code_[index + i]; // atom is an iptr appearing after the leading atom in the cptr.
        switch (atom.getDescriptor()) {
        case Atom::VIEW:
          member_name = "vw";
          break;
        case Atom::MKS:
          member_name = "mks";
          break;
        case Atom::VWS:
          member_name = "vws";
          break;
        default:
          member_name = embedding_class.get_member_name(atom.asIndex());
          if (i < member_count) // not the last member, get the next class.
            embedding_class = *embedding_class.get_member_class(metadata_, member_name);
          break;
        }
        *out_stream_ << '.' << member_name;

        if (member_name == "vw") { // special case: no view structure in the code, vw is just a place holder; vw is the second to last member of the cptr: write the last member and exit.

          atom = current_object_->code_[index + member_count]; // atom is the last internal pointer.
          Class view_class;
          if (embedding_class.str_opcode == "grp")
            view_class = metadata_->classes_.find("grp_view")->second;
          else if (embedding_class.str_opcode == "ipgm" ||
            embedding_class.str_opcode == "icpp_pgm" ||
            embedding_class.str_opcode == "mdl" ||
            embedding_class.str_opcode == "cst")
            view_class = metadata_->classes_.find("pgm_view")->second;
          else
            view_class = metadata_->classes_.find("view")->second;
          member_name = view_class.get_member_name(atom.asIndex());
          *out_stream_ << '.' << member_name;
          break;
        }
      }
      break;
    }default:
      out_stream_->push("undefined-structural-atom-or-reference", read_index);
      break;
    }
    break;
  case Atom::R_PTR:
    out_stream_->push(get_object_name(current_object_->references_[a.asIndex()]), read_index);
    break;
  case Atom::THIS:
    out_stream_->push("this", read_index);
    break;
  case Atom::NIL:
    out_stream_->push("nil", read_index);
    break;
  case Atom::BOOLEAN_:
    if (a.readsAsNil())
      out_stream_->push("|bl", read_index);
    else {

      *out_stream_ << std::boolalpha;
      out_stream_->push(a.asBoolean(), read_index);
    }
    break;
  case Atom::WILDCARD:
    if (after_tail_wildcard)
      out_stream_->push();
    else
      out_stream_->push(':', read_index);
    break;
  case Atom::T_WILDCARD:
    out_stream_->push("::", read_index);
    after_tail_wildcard = true;
    break;
  case Atom::NODE:
    if (a.readsAsNil())
      out_stream_->push("|sid", read_index);
    else {

      out_stream_->push("0x", read_index);
      *out_stream_ << std::hex;
      *out_stream_ << a.atom_;
    }
    break;
  case Atom::DEVICE:
    if (a.readsAsNil())
      out_stream_->push("|did", read_index);
    else {

      out_stream_->push("0x", read_index);
      *out_stream_ << std::hex;
      *out_stream_ << a.atom_;
    }
    break;
  case Atom::DEVICE_FUNCTION:
    if (a.readsAsNil())
      out_stream_->push("|fid", read_index);
    else
      out_stream_->push(metadata_->function_names_[a.asOpcode()], read_index);
    break;
  case Atom::VIEW:
    out_stream_->push("vw", read_index);
    break;
  case Atom::MKS:
    out_stream_->push("mks", read_index);
    break;
  case Atom::VWS:
    out_stream_->push("vws", read_index);
    break;
  default:
    // out_stream_->push("undefined-atom",read_index).
    out_stream_->push("0x", read_index);
    *out_stream_ << std::hex;
    *out_stream_ << a.atom_;
    break;
  }
}
}
