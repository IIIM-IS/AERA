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

#ifndef decompiler_h
#define decompiler_h

#include <fstream>
#include <sstream>

#include "out_stream.h"
#include "segments.h"


namespace r_comp {

class dll_export Decompiler {
private:
  OutStream *out_stream_;
  uint16 indents_; // in chars.
  bool closing_set_; // set after writing the last element of a set: any element in an expression finding closing_set_ will indent and set closing_set_ to false.
  bool in_hlp_;
  bool hlp_postfix_;
  bool horizontal_set_;

  r_code::ImageObject *current_object_;

  r_comp::Metadata *metadata_;
  r_comp::Image *image_;

  std::chrono::microseconds time_offset_; // 0 means no offset.

  UNORDERED_MAP<uint16, std::string> variable_names_; // in the form vxxx where xxx is an integer representing the order of referencing of the variable/label in the code.
  uint16 last_variable_id_;
  std::string get_variable_name(uint16 index, bool postfix); // associates iptr/vptr indexes to names; inserts them in out_stream_ if necessary; when postfix==true, a trailing ':' is added.
  std::string get_hlp_variable_name(uint16 index);

  UNORDERED_MAP<uint16, std::string> object_names_; // in the form class_namexxx where xxx is an integer representing the order of appearence of the object in the image; or: user-defined names when they are provided.
  UNORDERED_MAP<std::string, uint16> object_indices_; // inverted version of the object_names.
  std::string get_object_name(uint16 index); // retrieves the name of an object.

  void write_indent(uint16 i);
  void write_expression_head(uint16 read_index); // decodes the leading atom of an expression.
  void write_expression_tail(uint16 read_index, bool apply_time_offset, bool vertical = false); // decodes the elements of an expression following the head.
  void write_set(uint16 read_index, bool apply_time_offset, uint16 write_as_view_index = 0);
  void write_any(uint16 read_index, bool &after_tail_wildcard, bool apply_time_offset, uint16 write_as_view_index = 0); // decodes any element in an expression or a set.

  typedef void (Decompiler::*Renderer)(uint16);
  r_code::vector<Renderer> renderers_; // indexed by opcodes; when not there, write_expression() is used.

  // Renderers.
  void write_expression(uint16 read_index); // default renderer.
  void write_group(uint16 read_index);
  void write_marker(uint16 read_index);
  void write_pgm(uint16 read_index);
  void write_ipgm(uint16 read_index);
  void write_icmd(uint16 read_index);
  void write_cmd(uint16 read_index);
  void write_fact(uint16 read_index);
  void write_hlp(uint16 read_index);
  void write_ihlp(uint16 read_index);
  void write_sim(uint16 read_index);
  void write_view(uint16 read_index, uint16 arity);

  bool partial_decompilation_; // used when decompiling on-the-fly.
  bool ignore_named_objects_;
  UNORDERED_SET<uint16> named_objects_;
  std::vector<r_code::SysObject *> imported_objects_; // referenced objects added to the image that were not in the original list of objects to be decompiled.
public:
  Decompiler();
  ~Decompiler();

  void init(r_comp::Metadata *metadata);
  uint32 decompile(r_comp::Image *image,
    std::ostringstream *stream,
    Timestamp::duration time_offset,
    bool ignore_named_objects); // decompiles the whole image; returns the number of objects.
  uint32 decompile(r_comp::Image *image,
    std::ostringstream *stream,
    Timestamp::duration time_offset,
    std::vector<r_code::SysObject *> &imported_objects,
    bool include_oid = true, bool include_label = true, bool include_views = true); // idem, ignores named objects if in the imported object list.

  /**
   * initialize a reference table so that objects can be decompiled individually; returns the number of objects.
   * \param image The Image containing the objects.
   * \param object_names (optional) The initial mapping from the index of the object in image to its name. If 
   * image->object_names_.symbols_ already has a name for the OID, then it must match. This is necessary to supply
   * names of objects which don't have an OID if available. If omitted or NULL, then use names from 
   * image->object_names_.symbols_ and create names for objects that don't have an OID.
   * \return The number of objects, or 0 for error.
   */
  uint32 decompile_references(r_comp::Image *image, UNORDERED_MAP<uint16, std::string>* object_names = NULL);

  /**
   * Decompile a single object.
   * \param object_index The position of the object in image_->code_segment_.objects_.
   * \param stream The output stream.
   * \param time_offset The time since the start of the run for showing relative times.
   * \param include_oid (optional) If true, prepend with the OID (and debug OID if enabled). If omitted, include the OID.
   * \param include_label (optional) If true, prepend the label and ':'. If omitted, include the label.
   * \param include_views (optional) If true, include the set of view, or |[] if there are not views. If omitted, include the views.
   */
  void decompile_object(
    uint16 object_index, std::ostringstream *stream, Timestamp::duration time_offset, bool include_oid = true, 
    bool include_label = true, bool include_views = true);

  void decompile_object(const std::string object_name, std::ostringstream *stream, Timestamp::duration time_offset); // decompiles a single object given its name: use this function to follow references.
};
}


#endif
