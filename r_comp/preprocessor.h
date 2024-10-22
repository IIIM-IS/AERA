//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2023 Jeff Thompson
//_/_/ Copyright (c) 2018-2023 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2023 Icelandic Institute for Intelligent Machines
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

#ifndef preprocessor_h
#define preprocessor_h

#include "segments.h"
#include <istream>
#include <sstream>
#include <fstream>
#include <list>

namespace r_comp {

class RepliMacro;
class RepliCondition;
class RepliStruct {
public:
  static std::unordered_map<std::string, RepliMacro *> RepliMacros_;
  static std::unordered_map<std::string, int32> Counters_;
  static std::list<RepliCondition *> Conditions_;
  static uint32 GlobalLine_;
  static std::vector<std::string> LoadedFilePaths_;

  enum Type { Root, Structure, Set, Atom, Directive, Condition, Development };
  Type type_;
  std::string cmd_;
  std::string tail_;
  std::string label_;
  std::string error_;
  uint32 line_;
  std::list<RepliStruct *> args_;
  RepliStruct *parent_;
  std::string filePath_;

  RepliStruct(RepliStruct::Type type);
  ~RepliStruct();

  void reset(); // remove rags that are objects.

  uint32 getIndent(std::istream *stream);
  int32 parse(std::istream *stream, const std::string& file_path, uint32 &cur_indent, uint32 &prev_indent, int32 param_expect = 0);
  bool parseDirective(std::istream *stream, const std::string& file_path, uint32 &cur_indent, uint32 &prev_indent);
  int32 process();

  RepliStruct *findAtom(const std::string &name);

  /**
   * Load the Replicode file from the filename and call parse, which will store the
   * filename in any !load directives for later use.
   * \param filename The file path which is already combined with the directory of the code with 
   * the !load directive.
   * \return The parsed code.
   */
  RepliStruct *loadReplicodeFile(const std::string &filename);

  /**
   * Search RepliStruct::LoadedFilePaths_ to check if the filePath is already loaded. This checks
   * for equivalent file paths.So, for example, "Test/file.replicode" will match with
   * "/work/AERA/Test/file.replicode" and "Test/../Test/file.replicode" if they all
   * refer to the same file.
   * \param file_path The file path to check.
   * \return True if an equivalend file path is already loaded, otherwise false.
   */
  static bool isFileLoaded(const std::string& file_path);

  RepliStruct *clone() const;
  std::string print() const;
  std::string printError() const;

  friend std::ostream& operator<<(std::ostream &os, const RepliStruct &structure);
  friend std::ostream& operator<<(std::ostream &os, RepliStruct *structure);
};

class RepliMacro {
public:
  std::string name_;
  RepliStruct *src_;
  RepliStruct *dest_;
  std::string error_;

  RepliMacro(const std::string &name, RepliStruct *src, RepliStruct *dest);
  ~RepliMacro();

  uint32 argCount();
  RepliStruct *expandMacro(RepliStruct *old_struct);
};

class RepliCondition {
public:
  std::string name_;
  bool reversed_;

  RepliCondition(const std::string &name, bool reversed);
  ~RepliCondition();
  void reverse();
  bool isActive(std::unordered_map<std::string, RepliMacro*> &repli_macros, std::unordered_map<std::string, int32> &counters);
};

class dll_export Preprocessor {
private:
  typedef enum {
    T_CLASS = 0,
    T_SYS_CLASS = 1,
    T_SET = 2
  }ClassType;
  Metadata *metadata_;
  uint16 class_opcode_; // shared with sys_classes_
  std::unordered_map<std::string, RepliStruct *> template_classes_;
  void instantiateClass(RepliStruct *tpl_class, std::list<RepliStruct *> &tpl_args, std::string &instantiated_class_name);
  bool isSet(std::string class_name);
  bool isTemplateClass(RepliStruct *s);
  void getMember(std::vector<StructureMember> &members, RepliStruct *m, std::list<RepliStruct *> &tpl_args, bool instantiate);
  void getMembers(RepliStruct *s, std::vector<StructureMember> &members, std::list<RepliStruct *> &tpl_args, bool instantiate);
  ReturnType getReturnType(RepliStruct *s);
  void initialize(Metadata *metadata); // init definition_segment
public:
  RepliStruct *root_;

  Preprocessor();
  ~Preprocessor();
  bool process(std::istream *stream, // if an ifstream, stream must be open.
    const std::string& filePath, // the file path of the ifstream.
    std::ostringstream *outstream, // output stream=input stream where macros are expanded.
    std::string &error, // set when function fails, e.g. returns false.
    Metadata *metadata = NULL); // process will fill class_image, or use the exiting one if NULL.
};
}


#endif
