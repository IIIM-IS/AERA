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

#include <experimental/filesystem>
#include "preprocessor.h"
#include "compiler.h"
#include "../submodules/CoreLibrary/CoreLibrary/utils.h"
namespace fs = std::experimental::filesystem;


namespace r_comp {

UNORDERED_MAP<std::string, RepliMacro *> RepliStruct::RepliMacros_;
UNORDERED_MAP<std::string, int32> RepliStruct::Counters_;
std::list<RepliCondition *> RepliStruct::Conditions_;
uint32 RepliStruct::GlobalLine_ = 1;
std::vector<std::string> RepliStruct::LoadedFilePaths_;

RepliStruct::RepliStruct(RepliStruct::Type type) {
  this->type_ = type;
  line_ = GlobalLine_;
  parent_ = NULL;
}

RepliStruct::~RepliStruct() {
  parent_ = NULL;
}

void RepliStruct::reset() {

  std::list<RepliStruct*>::const_iterator arg;
  for (arg = args_.begin(); arg != args_.end();) {

    switch ((*arg)->type_) {
    case RepliStruct::Atom:
    case RepliStruct::Structure:
    case RepliStruct::Development:
    case RepliStruct::Set:
      arg = args_.erase(arg);
      break;
    default:
      ++arg;
    }
  }
}

uint32 RepliStruct::getIndent(std::istream *stream) {

  uint32 count = 0;
  while (!stream->eof()) {
    switch (stream->get()) {
    case 10:
      GlobalLine_++;
    case 13:
      stream->seekg(-1, std::ios_base::cur);
      return count / 3;
    case ' ':
      count++;
      break;
    default:
      stream->seekg(-1, std::ios_base::cur);
      return count / 3;
    }
  }
  return count / 3;
}

int32 RepliStruct::parse(std::istream *stream, const std::string& filePath, uint32 &curIndent, uint32 &prevIndent, int32 paramExpect) {

  char c = 0, lastc = 0, lastcc, tc;
  std::string str, label;
  RepliStruct* subStruct;

  int32 paramCount = 0;
  int32 returnIndent = 0;

  bool inComment = false;
  bool expectSet = false;

  while (!stream->eof()) {
    lastcc = lastc;
    lastc = c;
    c = stream->get();
    // printf("%c", c);
    switch (c) {
    case '\t':
      if (inComment) continue; // allow tabs in comments, does not matter anyway
      line_ = GlobalLine_;
      error_ += "Tabs chars are not permitted. ";
      return -1;
    case '!':
      if (inComment) continue;
      if (this->type_ == Root) {
        subStruct = new RepliStruct(Directive);
        subStruct->parent_ = this;
        args_.push_back(subStruct);
        if (!subStruct->parseDirective(stream, filePath, curIndent, prevIndent))
          return -1;
        if (subStruct->cmd_.compare("!load") == 0)
          // Save the filePath of the containing file for later.
          subStruct->filePath_ = filePath;
      }
      else {
        error_ += "Directive not allowed inside a structure. ";
        return -1;
      }
      break;
    case 10:
      GlobalLine_++;
    case 13:
      // remain inComment?
      if (inComment) {
        if ((lastc == ' ') || ((lastc == '.') && (lastcc == '.')))
          continue; // continue comment on next line
        else
          inComment = false; // end comment
      }
      // skip all CR and LF
      while ((!stream->eof()) && (stream->peek() < 32))
        if (stream->get() == 10)
          GlobalLine_++;
      // get indents
      prevIndent = curIndent;
      curIndent = getIndent(stream);
      // Are we in a parenthesis or set
      if (curIndent > prevIndent) {
        // Are we in a set
        if (expectSet) {
          // Add new sub structure
          subStruct = new RepliStruct(Set);
          subStruct->parent_ = this;
          subStruct->label_ = label;
          label = "";
          args_.push_back(subStruct);
          returnIndent = subStruct->parse(stream, filePath, curIndent, prevIndent);
          expectSet = false;
          if (returnIndent < 0)
            return -1;
          if ((paramExpect > 0) && (++paramCount == paramExpect))
            return 0;
          if (returnIndent > 0)
            return (returnIndent - 1);
        }
        // or a parenthesis
        else {
          subStruct = new RepliStruct(Structure);
          args_.push_back(subStruct);
          returnIndent = subStruct->parse(stream, filePath, curIndent, prevIndent);
          expectSet = false;
          if (returnIndent < 0)
            return -1;
          if ((paramExpect > 0) && (++paramCount == paramExpect))
            return 0;
          if (returnIndent > 0)
            return (returnIndent - 1);
        }
      }
      else if (curIndent < prevIndent) {
        if (str.size() > 0) {
          if ((cmd_.size() > 0) || (type_ == Set)) {
            subStruct = new RepliStruct(Atom);
            subStruct->parent_ = this;
            args_.push_back(subStruct);
            subStruct->cmd_ = str;
            str = "";
            if ((paramExpect > 0) && (++paramCount == paramExpect))
              return 0;
          }
          else {
            cmd_ = str;
            str = "";
          }
        }
        // current structure or set is complete
        return prevIndent - curIndent - 1;
      }
      else {
        // act as if we met a space
        if (str.size() > 0) {
          if ((cmd_.size() > 0) || (type_ == Set) || (type_ == Root)) {
            subStruct = new RepliStruct(Atom);
            subStruct->parent_ = this;
            args_.push_back(subStruct);
            subStruct->cmd_ = str;
            str = "";
            if ((paramExpect > 0) && (++paramCount == paramExpect))
              return 0;
          }
          else {
            cmd_ = str;
            str = "";
          }
        }
      }
      break;
    case ';':
      inComment = true;
      break;
    case ' ':
      if (inComment) continue;
      // next string is ready
      if (str.size() > 0) {
        if ((cmd_.size() > 0) || (type_ == Set) || (type_ == Development)) { // Modification from Eric to have the Development treat tpl vars as atoms instead of a Development.cmd
          subStruct = new RepliStruct(Atom);
          subStruct->parent_ = this;
          args_.push_back(subStruct);
          subStruct->cmd_ = str;
          str = "";
          if ((paramExpect > 0) && (++paramCount == paramExpect))
            return 0;
        }
        else {
          cmd_ = str;
          str = "";
        }
      }
      break;
    case '(':
      if (inComment) continue;
      // Check for scenario 'xxx('
      if (str.size() > 0) {
        if (lastc == ':') { // label:(xxx)
          label = str;
          str = "";
        }
        else if ((cmd_.size() > 0) || (type_ == Set)) {
          subStruct = new RepliStruct(Atom);
          subStruct->parent_ = this;
          args_.push_back(subStruct);
          subStruct->cmd_ = str;
          str = "";
          if ((paramExpect > 0) && (++paramCount == paramExpect))
            return 0;
        }
        else {
          cmd_ = str;
          str = "";
        }
      }
      subStruct = new RepliStruct(Structure);
      subStruct->label_ = label;
      label = "";
      subStruct->parent_ = this;
      args_.push_back(subStruct);
      returnIndent = subStruct->parse(stream, filePath, curIndent, prevIndent);
      if (returnIndent < 0)
        return -1;
      if ((paramExpect > 0) && (++paramCount == paramExpect))
        return 0;
      if (returnIndent > 0)
        return (returnIndent - 1);
      break;
    case ')':
      if (inComment) continue;
      // Check for directive use of xxx):xxx or xxx):
      if (stream->peek() == ':') {
        // expect ':' or ':xxx'
        while ((!stream->eof()) && ((c = stream->get()) > 32))
          tail_ += c;
      }
      // We met a boundary, act as ' '
      if (str.size() > 0) {
        if ((cmd_.size() > 0) || (type_ == Set)) {
          subStruct = new RepliStruct(Atom);
          subStruct->parent_ = this;
          args_.push_back(subStruct);
          subStruct->cmd_ = str;
          str = "";
          if ((paramExpect > 0) && (++paramCount == paramExpect))
            return 0;
        }
        else {
          cmd_ = str;
          str = "";
        }
      }
      return 0;
    case '{':
      if (inComment) continue;
      // Check for scenario 'xxx{'
      if (str.size() > 0) {
        if (lastc == ':') { // label:{xxx}
          label = str;
          str = "";
        }
        else if ((cmd_.size() > 0) || (type_ == Set)) {
          subStruct = new RepliStruct(Atom);
          subStruct->parent_ = this;
          args_.push_back(subStruct);
          subStruct->cmd_ = str;
          str = "";
          if ((paramExpect > 0) && (++paramCount == paramExpect))
            return 0;
        }
        else {
          cmd_ = str;
          str = "";
        }
      }
      subStruct = new RepliStruct(Development);
      subStruct->label_ = label;
      label = "";
      subStruct->parent_ = this;
      args_.push_back(subStruct);
      returnIndent = subStruct->parse(stream, filePath, curIndent, prevIndent);
      if (returnIndent < 0)
        return -1;
      if ((paramExpect > 0) && (++paramCount == paramExpect))
        return 0;
      if (returnIndent > 0)
        return (returnIndent - 1);
      break;
    case '}':
      if (inComment) continue;
      // Check for directive use of xxx):xxx or xxx):
      if (stream->peek() == ':') {
        // expect ':' or ':xxx'
        while ((!stream->eof()) && ((c = stream->get()) > 32))
          tail_ += c;
      }
      // We met a boundary, act as ' '
      if (str.size() > 0) {
        if ((cmd_.size() > 0) || (type_ == Set) || (type_ == Development)) { // Modification from Eric to have the Development treat tpl vars as atoms instead of a Development.cmd
          subStruct = new RepliStruct(Atom);
          subStruct->parent_ = this;
          args_.push_back(subStruct);
          subStruct->cmd_ = str;
          str = "";
          if ((paramExpect > 0) && (++paramCount == paramExpect))
            return 0;
        }
        else {
          cmd_ = str;
          str = "";
        }
      }
      return 0;
    case '|':
      if (inComment) continue;
      if (stream->peek() == '[') {
        stream->get(); // read the [
        stream->get(); // read the ]
        str += "|[]";
      }
      else
        str += c;
      break;
    case '[': // set start
      if (inComment) continue;
      if (lastc == ':') { // label:[xxx]
        label = str;
        str = "";
      }
      if (stream->peek() == ']') {
        stream->get(); // read the ]
        // if we have a <CR> or <LF> next we may have a line indent set
        if (((tc = stream->peek()) < 32) || (tc == ';'))
          expectSet = true;
        else {
          // this could be a xxx:[] or xxx[]
          if ((lastc != ':') && (lastc > 32)) { // label[]
              // act as if [] is part of the string and continue
            str += "[]";
            continue;
          }
          // create empty set
          subStruct = new RepliStruct(Set);
          subStruct->parent_ = this;
          subStruct->label_ = label;
          args_.push_back(subStruct);
        }
      }
      else {
        // Check for scenario 'xxx['
        if (str.size() > 0) {
          if ((cmd_.size() > 0) || (type_ == Set)) {
            subStruct = new RepliStruct(Atom);
            subStruct->parent_ = this;
            args_.push_back(subStruct);
            subStruct->cmd_ = str;
            str = "";
            if ((paramExpect > 0) && (++paramCount == paramExpect))
              return 0;
          }
          else {
            cmd_ = str;
            str = "";
          }
        }
        subStruct = new RepliStruct(Set);
        subStruct->parent_ = this;
        subStruct->label_ = label;
        label = "";
        args_.push_back(subStruct);
        returnIndent = subStruct->parse(stream, filePath, curIndent, prevIndent);
        if (returnIndent < 0)
          return -1;
        if ((paramExpect > 0) && (++paramCount == paramExpect))
          return 0;
        if (returnIndent > 0)
          return (returnIndent - 1);
      }
      break;
    case ']':
      if (inComment) continue;
      // We met a boundary, act as ' '
      if (str.size() > 0) {
        if ((cmd_.size() > 0) || (type_ == Set)) {
          subStruct = new RepliStruct(Atom);
          subStruct->parent_ = this;
          args_.push_back(subStruct);
          subStruct->cmd_ = str;
          str = "";
          if ((paramExpect > 0) && (++paramCount == paramExpect))
            return 0;
        }
        else {
          cmd_ = str;
          str = "";
        }
      }
      return 0;
    default:
      if (inComment) continue;
      str += c;
      break;
    }
  }
  return 0;
}


bool RepliStruct::parseDirective(std::istream *stream, const std::string& filePath, uint32 &curIndent, uint32 &prevIndent) {

  std::string str = "!";
  // RepliStruct* subStruct;
  char c;

  // We know that parent read a '!', so first find out which directive
  while ((!stream->eof()) && ((c = stream->peek()) > 32))
    str += stream->get();
  if (stream->eof()) {
    error_ += "Error in directive formatting, end of file reached unexpectedly. ";
    return false;
  }

  unsigned int paramCount = 0;

  if (str.compare("!def") == 0) // () ()
    paramCount = 2;
  else if (str.compare("!counter") == 0) // xxx val
    paramCount = 2;
  else if (str.compare("!undef") == 0) // xxx
    paramCount = 1;
  else if (str.compare("!ifdef") == 0) { // name
    this->type_ = Condition;
    paramCount = 1;
  }
  else if (str.compare("!ifundef") == 0) { // name
    this->type_ = Condition;
    paramCount = 1;
  }
  else if (str.compare("!else") == 0) { //
    this->type_ = Condition;
    paramCount = 0;
  }
  else if (str.compare("!endif") == 0) { //
    this->type_ = Condition;
    paramCount = 0;
  }
  else if (str.compare("!class") == 0) // ()
    paramCount = 1;
  else if (str.compare("!op") == 0) // ():xxx
    paramCount = 1;
  else if (str.compare("!dfn") == 0) // ()
    paramCount = 1;
  else if (str.compare("!load") == 0) // xxx
    paramCount = 1;
  else {
    error_ += "Unknown directive: '" + str + "'. ";
    return false;
  }
  cmd_ = str;

  if (paramCount == 0) {
    // read until end of line, including any comments
    while ((!stream->eof()) && (stream->peek() > 13))
      stream->get();
    // read the end of line too
    while ((!stream->eof()) && (stream->peek() < 32))
      if (stream->get() == 10)
        GlobalLine_++;
    return true;
  }

  if (parse(stream, filePath, curIndent, prevIndent, paramCount) != 0) {
    error_ += "Error parsing the arguments for directive '" + cmd_ + "'. ";
    return false;
  }
  else
    return true;
}


int32 RepliStruct::process() {

  int32 changes = 0, count;
  RepliStruct *structure, *newStruct, *tempStruct;
  RepliMacro* macro;
  RepliCondition* cond;
  std::string loadError;

  // expand Counters_ in all structures
  if (Counters_.find(cmd_) != Counters_.end()) {
    // expand the counter
    cmd_ = std::to_string(Counters_[cmd_]++);
    changes++;
  }
  // expand Macros in all structures
  if (RepliMacros_.find(cmd_) != RepliMacros_.end()) {
    // expand the macro
    macro = RepliMacros_[cmd_];
    newStruct = macro->expandMacro(this);
    if (newStruct != NULL) {
      *this = *newStruct;
      delete(newStruct);
      changes++;
    }
    else {
      error_ = macro->error_;
      macro->error_ = "";
      return -1;
    }
  }

  if (args_.size() == 0)
    return changes;

  for (std::list<RepliStruct*>::iterator iter(args_.begin()), iterEnd(args_.end()); iter != iterEnd; ++iter) {
    structure = (*iter);

    // printf("Processing %s with %d args...\n", structure->cmd.c_str(), structure->args.size());
    if (structure->type_ == Condition) {
      if (structure->cmd_.compare("!ifdef") == 0) {
        cond = new RepliCondition(structure->args_.front()->cmd_, false);
        Conditions_.push_back(cond);
      }
      else if (structure->cmd_.compare("!ifundef") == 0) {
        cond = new RepliCondition(structure->args_.front()->cmd_, true);
        Conditions_.push_back(cond);
      }
      else if (structure->cmd_.compare("!else") == 0) {
        // reverse the current condition
        Conditions_.back()->reverse();
      }
      else if (structure->cmd_.compare("!endif") == 0) {
        Conditions_.pop_back();
      }
      return 0;
    }

    // Check Conditions_ to see if we are active at the moment
    for (std::list<RepliCondition*>::const_iterator iCon(Conditions_.begin()), iConEnd(Conditions_.end()); iCon != iConEnd; ++iCon) {
      // if just one active condition is not active we will ignore the current line
      // until we get an !else or !endif
      if (!((*iCon)->isActive(RepliMacros_, Counters_)))
        return 0;
    }

    if (structure->type_ == Directive) {
      if (structure->cmd_.compare("!counter") == 0) {
        // register the counter
        if (structure->args_.size() > 1)
          Counters_[structure->args_.front()->cmd_] = atoi(structure->args_.back()->cmd_.c_str());
        else
          Counters_[structure->args_.front()->cmd_] = 0;
      }
      else if (structure->cmd_.compare("!def") == 0) {
        // check second sub structure only containing macros
        while ((count = structure->args_.back()->process()) > 0)
          changes += count;
        if (count < 0)
          return -1;
        // register the macro
        macro = new RepliMacro(structure->args_.front()->cmd_, structure->args_.front(), structure->args_.back());
        RepliMacros_[macro->name_] = macro;
      }
      else if (structure->cmd_.compare("!undef") == 0) {
        // remove the counter or macro
        RepliMacros_.erase(RepliMacros_.find(structure->args_.front()->cmd_));
        Counters_.erase(Counters_.find(structure->args_.front()->cmd_));
      }
      else if (structure->cmd_.compare("!load") == 0) {
        // Check for a load directive...
        fs::path loadPath = fs::path(structure->args_.front()->cmd_);
        if (loadPath.is_relative()) {
          // Combine the relative path with the directory of the containing file.
          fs::path containingDirectory = fs::path(structure->filePath_).parent_path();
          loadPath = containingDirectory / loadPath;
        }

        newStruct = loadReplicodeFile(loadPath.string());
        if (newStruct == NULL) {
          structure->error_ += "Load: File '" + loadPath.string() + "' cannot be read! ";
          return -1;
        }
        else if ((loadError = newStruct->printError()).size() > 0) {
          structure->error_ = loadError;
          delete(newStruct);
          return -1;
        }
        // Insert new data into current args
        // save location
        tempStruct = (*iter);
        // insert new structures
        args_.insert(++iter, newStruct->args_.begin(), newStruct->args_.end());
        // The args have been copied out of newStruct. We are finished with it.
        delete newStruct;
        // reinit iterator and find location again
        iter = args_.begin();
        iterEnd = args_.end();
        while ((*iter) != tempStruct) iter++;
        // we want to remove the !load line, so get the next line
        iter++;
        args_.remove(tempStruct);
        if (iter != iterEnd) {
          // and because we changed the list, repeat
          tempStruct = (*iter);
          iter = args_.begin();
          iterEnd = args_.end();
          while ((*iter) != tempStruct) iter++;
        }
        // now we have replaced the !load line with the loaded lines
        changes++;
      }
    }
    else { // a Structure, Set, Atom or Development
      if (RepliMacros_.find(structure->cmd_) != RepliMacros_.end()) {
        // expand the macro
        macro = RepliMacros_[structure->cmd_];
        newStruct = macro->expandMacro(structure);
        if (newStruct != NULL) {
          *structure = *newStruct;
          delete(newStruct);
          changes++;
        }
        else {
          structure->error_ = macro->error_;
          macro->error_ = "";
          return -1;
        }
      }

      // check for sub structures containing macros
      for (std::list<RepliStruct*>::iterator iter2(structure->args_.begin()), iter2End(structure->args_.end()); iter2 != iter2End; ++iter2) {
        if ((count = (*iter2)->process()) > 0)
          changes += count;
        else if (count < 0)
          return -1;
      }
    }

    // expand Counters_ in all structures
    if (Counters_.find(structure->cmd_) != Counters_.end()) {
      // expand the counter
      structure->cmd_ = std::to_string(Counters_[structure->cmd_]++);
      changes++;
    }

    // The iter may already be moved to the end, so check here before the for loop increments it.
    if (iter == iterEnd)
      break;
  }

  return changes;
}

RepliStruct *RepliStruct::loadReplicodeFile(const std::string &filename) {

  RepliStruct* newRoot = new RepliStruct(Root);
  if (isFileLoaded(filename)) {
    // This file was already loaded. Skip it by returning an empty newRoot.
    return newRoot;
  }

  // Mark this file as loaded.
  LoadedFilePaths_.push_back(filename);

  std::ifstream loadStream(filename.c_str(), std::ios::binary | ios::in);
  if (loadStream.bad() || loadStream.fail() || loadStream.eof()) {
    newRoot->error_ += "Load: File '" + filename + "' cannot be read! ";
    loadStream.close();
    return newRoot;
  }
  // create new Root structure
  uint32 a = 0, b = 0;
  if (newRoot->parse(&loadStream, filename, a, b) < 0) {
    // error is already recorded in newRoot
  }
  if (!loadStream.eof())
    newRoot->error_ = "Code structure error: Unmatched ) or ].\n";
  loadStream.close();
  return newRoot;
}

bool RepliStruct::isFileLoaded(const std::string& filePath) {
  for (auto loadedFilePath = LoadedFilePaths_.begin();
    loadedFilePath != LoadedFilePaths_.end();
    ++loadedFilePath) {
    if (fs::equivalent(filePath, *loadedFilePath))
      return true;
  }

  return false;
}

std::string RepliStruct::print() const {

  std::string str;
  switch (type_) {
  case Atom:
    return cmd_;
  case Structure:
  case Development:
  case Directive:
  case Condition:
    str = cmd_;
    for (std::list<RepliStruct*>::const_iterator iter(args_.begin()), iterEnd(args_.end()); iter != iterEnd; ++iter)
      str += " " + (*iter)->print();
    if (type_ == Structure)
      return label_ + "(" + str + ")" + tail_;
    else if (type_ == Development)
      return label_ + "{" + str + "}" + tail_;
    else
      return str;
  case Set:
    for (std::list<RepliStruct*>::const_iterator iter(args_.begin()), last(args_.end()), iterEnd(last--); iter != iterEnd; ++iter) {
      str += (*iter)->print();
      if (iter != last)
        str += " ";
    }
    return label_ + "[" + str + "]" + tail_;
  case Root:
    for (std::list<RepliStruct*>::const_iterator iter(args_.begin()), iterEnd(args_.end()); iter != iterEnd; ++iter)
      str += (*iter)->print() + "\n";
    return str;
  default:
    break;
  }
  return str;
}

std::ostream &operator<<(std::ostream &os, RepliStruct *structure) {

  return operator<<(os, *structure);
}

std::ostream &operator<<(std::ostream &os, const RepliStruct &structure) {

  switch (structure.type_) {
  case RepliStruct::Atom:
    return os << structure.cmd_;
  case RepliStruct::Directive:
  case RepliStruct::Condition:
    return os;
  case RepliStruct::Structure:
  case RepliStruct::Development:
    if (structure.type_ == RepliStruct::Structure)
      os << structure.label_ << "(";
    else if (structure.type_ == RepliStruct::Development)
      os << structure.label_ << "{";
    os << structure.cmd_;
    for (std::list<RepliStruct*>::const_iterator iter(structure.args_.begin()), iterEnd(structure.args_.end()); iter != iterEnd; ++iter)
      os << " " << (*iter);
    if (structure.type_ == RepliStruct::Structure)
      os << ")" << structure.tail_;
    else if (structure.type_ == RepliStruct::Development)
      os << "}" << structure.tail_;
    break;
  case RepliStruct::Set:
    os << structure.label_ << "[";
    if (structure.args_.size() > 0) {
      for (std::list<RepliStruct*>::const_iterator iter(structure.args_.begin()), last(structure.args_.end()), iterEnd(last--); iter != iterEnd; ++iter) {
        os << (*iter);
        if (iter != last)
          os << " ";
      }
    }
    os << "]" << structure.tail_;
    break;
  case RepliStruct::Root:
    for (std::list<RepliStruct*>::const_iterator iter(structure.args_.begin()), iterEnd(structure.args_.end()); iter != iterEnd; ++iter)
      os << (*iter) << std::endl;
    break;
  default:
    break;
  }
  return os;
}

RepliStruct *RepliStruct::clone() const {

  RepliStruct* newStruct = new RepliStruct(type_);
  newStruct->cmd_ = cmd_;
  newStruct->label_ = label_;
  newStruct->tail_ = tail_;
  newStruct->parent_ = parent_;
  newStruct->error_ = error_;
  newStruct->line_ = line_;
  for (std::list<RepliStruct*>::const_iterator iter(args_.begin()), iterEnd(args_.end()); iter != iterEnd; ++iter)
    newStruct->args_.push_back((*iter)->clone());
  return newStruct;
}

std::string RepliStruct::printError() const {

  std::stringstream strError;
  if (error_.size() > 0) {
    std::string com = cmd_;
    RepliStruct* structure = parent_;
    while ((cmd_.size() == 0) && (structure != NULL)) {
      com = structure->cmd_;
      structure = structure->parent_;
    }
    if (com.size() == 0)
      strError << "Error";
    else
      strError << "Error in structure '" << com << "'";
    if (line_ > 0)
      strError << " line " << line_;
    strError << ": " << error_ << std::endl;
  }
  for (std::list<RepliStruct*>::const_iterator iter(args_.begin()), iterEnd(args_.end()); iter != iterEnd; ++iter)
    strError << (*iter)->printError();
  return strError.str();
}

RepliMacro::RepliMacro(const std::string &name, RepliStruct *src, RepliStruct *dest) {

  this->name_ = name;
  this->src_ = src;
  this->dest_ = dest;
}

RepliMacro::~RepliMacro() {

  name_ = "";
  src_ = NULL;
  dest_ = NULL;
}

uint32 RepliMacro::argCount() {

  if (src_ == NULL)
    return 0;
  return src_->args_.size();
}

RepliStruct *RepliMacro::expandMacro(RepliStruct *oldStruct) {

  if (src_ == NULL) {
    error_ += "Macro '" + name_ + "' source not defined. ";
    return NULL;
  }
  if (dest_ == NULL) {
    error_ += "Macro '" + name_ + "' destination not defined. ";
    return NULL;
  }
  if (oldStruct == NULL) {
    error_ += "Macro '" + name_ + "' cannot expand empty structure. ";
    return NULL;
  }
  if (oldStruct->cmd_.compare(this->name_) != 0) {
    error_ += "Macro '" + name_ + "' cannot expand structure with different name '" + oldStruct->cmd_ + "'. ";
    return NULL;
  }

  if ((src_->args_.size() > 0) && (src_->args_.size() != oldStruct->args_.size())) {
    error_ += "Macro '" + name_ + "' requires " + std::to_string(src_->args_.size()) + " arguments, cannot expand structure with " + std::to_string(oldStruct->args_.size()) + " arguments. ";
    return NULL;
  }

  RepliStruct* newStruct;

  // Special case of macros without args, just copy in the args from oldStruct
  if ((src_->args_.size() == 0) && (oldStruct->args_.size() > 0)) {
    newStruct = oldStruct->clone();
    newStruct->cmd_ = dest_->cmd_;
    newStruct->label_ = oldStruct->label_;
    return newStruct;
  } else {

    newStruct = dest_->clone();
    newStruct->label_ = oldStruct->label_;
  }

  RepliStruct *findStruct;

  std::list<RepliStruct*>::const_iterator iOld(oldStruct->args_.begin());
  for (std::list<RepliStruct*>::const_iterator iSrc(src_->args_.begin()), iSrcEnd(src_->args_.end()); iSrc != iSrcEnd; ++iSrc, ++iOld) {
    // printf("looking for '%s'\n", (*iSrc)->cmd.c_str());
    // find the Atom inside newStruct with the name of iSrc->cmd
    findStruct = newStruct->findAtom((*iSrc)->cmd_);
    if (findStruct != NULL) {
      // overwrite data in findStruct with the matching one from old
      *findStruct = *(*iOld);
    }
  }

  return newStruct;
}

RepliStruct *RepliStruct::findAtom(const std::string &name) {

  RepliStruct* structure;
  for (std::list<RepliStruct*>::iterator iter(args_.begin()), iterEnd(args_.end()); iter != iterEnd; ++iter) {
    switch ((*iter)->type_) {
    case Atom:
      if ((*iter)->cmd_.compare(name) == 0)
        return (*iter);
      break;
    case Structure:
    case Set:
    case Development:
      structure = (*iter)->findAtom(name);
      if (structure != NULL)
        return structure;
      break;
    default:
      break;
    }
  }
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RepliCondition::RepliCondition(const std::string &name, bool reversed) {

  this->name_ = name;
  this->reversed_ = reversed;
}

RepliCondition::~RepliCondition() {
}

bool RepliCondition::reverse() {

  reversed_ = !reversed_;
  return true;
}

bool RepliCondition::isActive(UNORDERED_MAP<std::string, RepliMacro *> &repliMacros, UNORDERED_MAP<std::string, int32> &counters) {

  bool foundIt = (repliMacros.find(name_) != repliMacros.end());

  if (!foundIt)
    foundIt = (counters.find(name_) != counters.end());

  if (reversed_)
    return (!foundIt);
  else
    return foundIt;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Preprocessor::Preprocessor() {

  root_ = new RepliStruct(RepliStruct::Root);
}

Preprocessor::~Preprocessor() {

  delete root_;
}

bool Preprocessor::process(std::istream *stream,
  const std::string& filePath,
  std::ostringstream *outstream,
  std::string &error,
  Metadata *metadata) {

  // Mark this file as loaded. We don't check if it is already loaded because
  // this is called from the top level Init (not from a !load directive) and we
  // assume that it won't call this twice for the same file.
  RepliStruct::LoadedFilePaths_.push_back(filePath);

  root_->reset(); // trims root from previously preprocessed objects.

  uint32 a = 0, b = 0;
  if (root_->parse(stream, filePath, a, b) < 0) {

    error = root_->printError();
    return false;
  }
  if (!stream->eof()) {

    error = "Code structure error: Unmatched ) or ].\n";
    return false;
  }

  // printf("Replicode:\n\n%s\n",root->print().c_str());

  int32 pass = 0, total = 0, count;
  while ((count = root_->process()) > 0) {

    total += count;
    pass++;
    // printf("Pass %d, %d changes, %d total\n", pass, count, total);
  }
  if (count < 0) {

    error = root_->printError();
    return false;
  }
  // printf("Replicode:\n\n%s\n",root->print().c_str());

  *outstream << root_;

  if (metadata)
    initialize(metadata);

  error = root_->printError();
  return (error.size() == 0);
}

bool Preprocessor::isTemplateClass(RepliStruct *s) {

  for (std::list<RepliStruct *>::iterator j(s->args_.begin()); j != s->args_.end(); ++j) {

    std::string name;
    std::string type;
    switch ((*j)->type_) {
    case RepliStruct::Atom:
      if ((*j)->cmd_ == ":~")
        return true;
      break;
    case RepliStruct::Structure: // template instantiation; args are the actual parameters.
    case RepliStruct::Development: // actual template arg as a list of args.
    case RepliStruct::Set: // sets can contain tpl args.
      if (isTemplateClass(*j))
        return true;
      break;
    default:
      break;
    }
  }
  return false;
}

bool Preprocessor::isSet(std::string class_name) {

  for (std::list<RepliStruct *>::iterator i(root_->args_.begin()); i != root_->args_.end(); ++i) {

    if ((*i)->type_ != RepliStruct::Directive || (*i)->cmd_ != "!class")
      continue;
    RepliStruct *s = *(*i)->args_.begin();
    if (s->cmd_ == class_name) // set classes are written class_name[].
      return false;
    if (s->cmd_ == class_name + "[]")
      return true;
  }
  return false;
}

void Preprocessor::instantiateClass(RepliStruct *tpl_class, std::list<RepliStruct *> &tpl_args, std::string &instantiated_class_name) {

  static uint32 LastClassID = 0;
  // remove the trailing [].
  std::string sset = "[]";
  instantiated_class_name = tpl_class->cmd_;
  instantiated_class_name = instantiated_class_name.substr(0, instantiated_class_name.length() - sset.length());
  // append an ID to the tpl class name.
  instantiated_class_name += std::to_string(LastClassID++);

  std::vector<StructureMember> members;
  std::list<RepliStruct *> _tpl_args;
  for (std::list<RepliStruct *>::reverse_iterator i = tpl_args.rbegin(); i != tpl_args.rend(); ++i)
    _tpl_args.push_back(*i);
  getMembers(tpl_class, members, _tpl_args, true);

  metadata_->class_names_[class_opcode_] = instantiated_class_name;
  metadata_->classes_by_opcodes_[class_opcode_] = metadata_->classes_[instantiated_class_name] = Class(Atom::SSet(class_opcode_, members.size()), instantiated_class_name, members);
  ++class_opcode_;
}

void Preprocessor::getMember(std::vector<StructureMember> &members, RepliStruct *m, std::list<RepliStruct *> &tpl_args, bool instantiate) {

  size_t p;
  std::string name;
  std::string type;
  switch (m->type_) {
  case RepliStruct::Set:
    name = m->label_.substr(0, m->label_.length() - 1);
    if (m->args_.size() == 0) // anonymous set of anything.
      members.push_back(StructureMember(&Compiler::read_set, name));
    else { // structured set, arg[0].cmd is ::type.

      type = (*m->args_.begin())->cmd_.substr(2, m->cmd_.length() - 1);
      if (isSet(type))
        members.push_back(StructureMember(&Compiler::read_set, name, type, StructureMember::I_SET));
      else if (type == Class::Type)
        members.push_back(StructureMember(&Compiler::read_set, name, type, StructureMember::I_DCLASS));
      else
        members.push_back(StructureMember(&Compiler::read_set, name, type, StructureMember::I_EXPRESSION));
    }
    break;
  case RepliStruct::Atom:
    if (m->cmd_ == "nil")
      break;
    p = m->cmd_.find(':');
    name = m->cmd_.substr(0, p);
    type = m->cmd_.substr(p + 1, m->cmd_.length());
    if (type == "")
      members.push_back(StructureMember(&Compiler::read_any, name));
    else if (type == "nb")
      members.push_back(StructureMember(&Compiler::read_number, name));
    else if (type == "us")
      members.push_back(StructureMember(&Compiler::read_timestamp, name));
    else if (type == "bl")
      members.push_back(StructureMember(&Compiler::read_boolean, name));
    else if (type == "st")
      members.push_back(StructureMember(&Compiler::read_string, name));
    else if (type == "did")
      members.push_back(StructureMember(&Compiler::read_device, name));
    else if (type == "fid")
      members.push_back(StructureMember(&Compiler::read_function, name));
    else if (type == "nid")
      members.push_back(StructureMember(&Compiler::read_node, name));
    else if (type == Class::Expression)
      members.push_back(StructureMember(&Compiler::read_expression, name));
    else if (type == "~") {

      RepliStruct *_m = tpl_args.back();
      tpl_args.pop_back();
      switch (_m->type_) {
      case RepliStruct::Structure: { // the tpl arg is an instantiated tpl set class.
        std::string instantiated_class_name;
        instantiateClass(template_classes_.find(_m->cmd_)->second, _m->args_, instantiated_class_name);
        members.push_back(StructureMember(&Compiler::read_set, _m->label_.substr(0, _m->label_.length() - 1), instantiated_class_name, StructureMember::I_CLASS));
        break;
      }default:
        getMember(members, _m, tpl_args, true);
        break;
      }
    } else if (type == Class::Type)
      members.push_back(StructureMember(&Compiler::read_class, name));
    else // type is a class name.
      members.push_back(StructureMember(&Compiler::read_expression, name, type));
    break;
  case RepliStruct::Structure: { // template instantiation; (*m)->cmd is the template class, (*m)->args are the actual parameters.
    RepliStruct *template_class = template_classes_.find(m->cmd_)->second;
    if (instantiate) {

      std::string instantiated_class_name;
      instantiateClass(template_class, m->args_, instantiated_class_name);
      members.push_back(StructureMember(&Compiler::read_set, m->label_.substr(0, m->label_.length() - 1), instantiated_class_name, StructureMember::I_CLASS));
    } else {

      for (std::list<RepliStruct *>::reverse_iterator i = m->args_.rbegin(); i != m->args_.rend(); ++i) // append the passed args to the ones held by m.
        tpl_args.push_back(*i);
      getMembers(template_class, members, tpl_args, false);
    }
    break;
  }case RepliStruct::Development:
    getMembers(m, members, tpl_args, instantiate);
    break;
  default:
    break;
  }
}

void Preprocessor::getMembers(RepliStruct *s, std::vector<StructureMember> &members, std::list<RepliStruct *> &tpl_args, bool instantiate) {

  for (std::list<RepliStruct *>::iterator j(s->args_.begin()); j != s->args_.end(); ++j)
    getMember(members, *j, tpl_args, instantiate);
}

ReturnType Preprocessor::getReturnType(RepliStruct *s) {

  if (s->tail_ == ":nb")
    return NUMBER;
  else if (s->tail_ == ":us")
    return TIMESTAMP;
  else if (s->tail_ == ":bl")
    return BOOLEAN;
  else if (s->tail_ == ":st")
    return STRING;
  else if (s->tail_ == ":nid")
    return NODE_ID;
  else if (s->tail_ == ":did")
    return DEVICE_ID;
  else if (s->tail_ == ":fid")
    return FUNCTION_ID;
  else if (s->tail_ == ":[]")
    return SET;
  return ANY;
}

void Preprocessor::initialize(Metadata *metadata) {

  this->metadata_ = metadata;

  class_opcode_ = 0;
  uint16 function_opcode = 0;
  uint16 operator_opcode = 0;

  std::vector<StructureMember> r_xpr;
  metadata->classes_[std::string(Class::Expression)] = Class(Atom::Object(class_opcode_, 0), Class::Expression, r_xpr); // to read unspecified expressions in classes and sets.
  ++class_opcode_;
  std::vector<StructureMember> r_type;
  metadata->classes_[std::string(Class::Type)] = Class(Atom::Object(class_opcode_, 0), Class::Type, r_type); // to read object types in expressions and sets.
  ++class_opcode_;

  for (std::list<RepliStruct *>::iterator i(root_->args_.begin()); i != root_->args_.end(); ++i) {

    if ((*i)->type_ != RepliStruct::Directive)
      continue;

    RepliStruct *s = *(*i)->args_.begin();
    std::vector<StructureMember> members;
    if ((*i)->cmd_ == "!class") {

      std::string sset = "[]";
      std::string class_name = s->cmd_;
      size_t p = class_name.find(sset);
      ClassType class_type = (p == std::string::npos ? T_CLASS : T_SET);
      if (class_type == T_SET) // remove the trailing [] since the RepliStructs for instantiated classes do so.
        class_name = class_name.substr(0, class_name.length() - sset.length());

      if (isTemplateClass(s)) {

        template_classes_[class_name] = s;
        continue;
      }

      std::list<RepliStruct *> tpl_args;
      getMembers(s, members, tpl_args, false);

      Atom atom;
      if (class_name == "grp")
        atom = Atom::Group(class_opcode_, members.size());
      else if (class_name == "ipgm")
        atom = Atom::InstantiatedProgram(class_opcode_, members.size());
      else if (class_name == "icpp_pgm")
        atom = Atom::InstantiatedCPPProgram(class_opcode_, members.size());
      else if (class_name == "cst")
        atom = Atom::CompositeState(class_opcode_, members.size());
      else if (class_name == "mdl")
        atom = Atom::Model(class_opcode_, members.size());
      else if (class_name.find("mk.") != string::npos)
        atom = Atom::Marker(class_opcode_, members.size());
      else
        atom = Atom::Object(class_opcode_, members.size());

      if (class_type == T_CLASS) { // find out if the class is a sys class, i.e. is an instantiation of _obj, _grp or _fact.

        std::string base_class = s->args_.front()->cmd_;
        if (base_class == "_obj" || base_class == "_grp" || base_class == "_fact")
          class_type = T_SYS_CLASS;
      }

      metadata->class_names_[class_opcode_] = class_name;
      switch (class_type) {
      case T_SYS_CLASS:
        metadata->classes_by_opcodes_[class_opcode_] = metadata->classes_[class_name] = metadata->sys_classes_[class_name] = Class(atom, class_name, members);
        break;
      case T_CLASS:
        metadata->classes_by_opcodes_[class_opcode_] = metadata->classes_[class_name] = Class(atom, class_name, members);
        break;
      case T_SET:
        metadata->classes_by_opcodes_[class_opcode_] = metadata->classes_[class_name] = Class(Atom::SSet(class_opcode_, members.size()), class_name, members);
        break;
      default:
        break;
      }
      ++class_opcode_;
    } else if ((*i)->cmd_ == "!op") {

      std::list<RepliStruct *> tpl_args;
      getMembers(s, members, tpl_args, false);
      ReturnType return_type = getReturnType(s);

      std::string operator_name = s->cmd_;
      metadata->operator_names_.push_back(operator_name);
      metadata->classes_[operator_name] = Class(Atom::Operator(operator_opcode, s->args_.size()), operator_name, members, return_type);
      ++operator_opcode;
    } else if ((*i)->cmd_ == "!dfn") { // don't bother to read the members, it's always a set.

      std::vector<StructureMember> r_set;
      r_set.push_back(StructureMember(&Compiler::read_set, ""));

      std::string function_name = s->cmd_;
      metadata->function_names_.push_back(function_name);
      metadata->classes_[function_name] = Class(Atom::DeviceFunction(function_opcode), function_name, r_set);
      ++function_opcode;
    }
  }
}
}
