//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ HUMANOBS - Replicode r_exec
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
extern r_exec_dll uint64(*Now)();

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

  _Thread *_thread;
  volatile uint32 spawned;

  r_code::list<P<Code> > objects;

  uint32 ostream_id; // 0 is std::cout.

  std::string header;
public:
  TDecompiler(uint32 ostream_id, std::string header);
  ~TDecompiler();

  void add_object(Code *object);
  void add_objects(const r_code::list<P<Code> > &objects);
  void add_objects(const std::vector<P<Code> > &objects);
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
  static std::vector<PipeOStream *> Streams;
  static PipeOStream NullStream;

  HANDLE pipe_read;
  HANDLE pipe_write;

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

// Initialize Now, compile user.classes.replicode, builds the Seed and loads the user-defined operators.
// Return false in case of a problem (e.g. file not found, operator not found, etc.).
bool r_exec_dll Init(const char *user_operator_library_path,
  uint64(*time_base)(),
  const char *seed_path);

// Alternate taking a ready-made metadata and seed (will be copied into Metadata and Seed).
bool r_exec_dll Init(const char *user_operator_library_path,
  uint64(*time_base)(),
  const r_comp::Metadata &metadata,
  const r_comp::Image &seed);

uint16 r_exec_dll GetOpcode(const char *name); // classes, operators and functions.

std::string r_exec_dll GetAxiomName(const uint16 index); // for constant objects (ex: self, position, and other axioms).
}


#endif
