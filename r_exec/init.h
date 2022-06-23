//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2022 Jeff Thompson
//_/_/ Copyright (c) 2018-2022 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2022 Icelandic Institute for Intelligent Machines
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

#ifndef init_h
#define init_h

#include "../submodules/CoreLibrary/CoreLibrary/utils.h"

#include "../r_code/list.h"

#include "../r_comp/segments.h"
#include "../r_comp/compiler.h"
#include "../r_comp/preprocessor.h"

#include "dll.h"


namespace r_exec {

// Time base; either Time::Get or network-aware synced time.
extern r_exec_dll Timestamp (*Now)();

// Loaded once for all.
// Results from the compilation of user.classes.replicode.
// The latter contains all class definitions and all shared objects (e.g. ontology); does not contain any dynamic (res!=forever) objects.
extern r_exec_dll r_comp::Metadata Metadata;
extern r_exec_dll r_comp::Image Seed;

// A preprocessor and a compiler are maintained throughout the life of the dll to retain, respectively, macros and global references.
// Both functions add the compiled object to Seed.code_image.
// Source files: use ANSI encoding (not Unicode).
bool r_exec_dll Compile(const char *filename, std::string &error);
bool r_exec_dll Compile(std::istream &source_code, std::string &error);

// Threaded decompiler, for decompiling on the fly asynchronously.
// Named objects are referenced but not decompiled.
class r_exec_dll TDecompiler :
  public _Object {
private:
  static const uint32 ObjectsInitialSize = 16;
  static thread_ret thread_function_call Decompile(void *args);

  class _Thread :
    public Thread {
  };

  _Thread *thread_;
  volatile uint32 spawned_;

  r_code::list<P<r_code::Code> > objects_;

  uint32 ostream_id_; // 0 is std::cout.

  std::string header_;
public:
  TDecompiler(uint32 ostream_id, std::string header);
  ~TDecompiler();

  void add_object(r_code::Code *object);
  void add_objects(const r_code::list<P<r_code::Code> > &objects);
  void add_objects(const std::vector<P<r_code::Code> > &objects);
  void decompile();
};

// Spawns an instance of output_window.exe (see output_window project) and opens a pipe between the main process and output_window.
// Temporary solution:
// (a) not portable,
// (b) shall be defined in CoreLibrary instead of here,
// (c) the stream pool management (PipeOStream::Open(), PipeOStream::Close() and PipeOStream::Get()) shall be decoupled from this implementation (it's an IDE feature),
// (d) PipeOStream shall be based on std::ostringstream instead of std::ostream with a refined std::stringbuf (override sync() to write in the pipe).
class r_exec_dll PipeOStream :
  public std::ostream {
private:
  static std::vector<PipeOStream *> Streams_;
  static PipeOStream NullStream_;

  HANDLE pipe_read_;
  HANDLE pipe_write_;

  void init(); // create one child process and a pipe.
  PipeOStream();
public:
  static void Open(uint8 count); // open count streams.
  static void Close(); // close all streams.
  static PipeOStream &Get(uint8 id); // return NullStream if id is out of range.

  ~PipeOStream();

  PipeOStream &operator <<(std::string &s);
  PipeOStream &operator <<(const char *s);
};

/**
 * Use the given metadata (not r_exec::Metatdata) to initialize
 * r_exec::_Opcodes, r_code::OpcodeNames, View::ViewOpcode_,  and the
 * values in the Opcodes class such as Opcodes::Fact. Also call
 * Operator::Register to set up standard operators in Operator::Operators_.
 * \return True for success.
 */
bool r_exec_dll InitOpcodes(const r_comp::Metadata& metadata);

// Initialize Now, compile user.classes.replicode, builds the Seed and loads the user-defined operators.
// Return false in case of a problem (e.g. file not found, operator not found, etc.).
bool r_exec_dll Init(const char *user_operator_library_path,
  Timestamp (*time_base)(),
  const char *seed_path);

// Alternate taking a ready-made metadata and seed (will be copied into Metadata and Seed).
bool r_exec_dll Init(const char *user_operator_library_path,
  Timestamp (*time_base)(),
  const r_comp::Metadata &metadata,
  const r_comp::Image &seed);

uint16 r_exec_dll GetOpcode(const char *name); // classes, operators and functions.

std::string r_exec_dll GetAxiomName(const uint16 index); // for constant objects (ex: self, position, and other axioms).
}


#endif
