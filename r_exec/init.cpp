//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2025 Jeff Thompson
//_/_/ Copyright (c) 2018-2025 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2025 Icelandic Institute for Intelligent Machines
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

#include <algorithm>
#include "init.h"
#include "controller.h"
#include "object.h"
#include "operator.h"
#include "cpp_programs.h"
#include "callbacks.h"
#include "opcodes.h"
#include "overlay.h"
#include "mem.h"

#include "../r_comp/decompiler.h"
#include "../r_comp/preprocessor.h"

#ifdef WINDOWS
#include <process.h>
#endif

using namespace std;
using namespace std::chrono;
using namespace r_code;

namespace r_exec {

dll_export Timestamp (*Now)();

dll_export r_comp::Metadata Metadata;
dll_export r_comp::Image Seed;

unordered_map<std::string, uint16> _Opcodes;

dll_export r_comp::Compiler Compiler;
r_exec_dll r_comp::Preprocessor Preprocessor;

////////////////////////////////////////////////////////////////////////////////////////////////

bool Compile(std::istream &source_code, const std::string& filePath, std::string &error, bool compile_metadata) {

  std::ostringstream preprocessed_code_out;
  if (!r_exec::Preprocessor.process(&source_code, filePath, &preprocessed_code_out, error, compile_metadata ? &Metadata : NULL))
    return false;

  std::istringstream preprocessed_code_in(preprocessed_code_out.str());

  if (!r_exec::Compiler.compile(&preprocessed_code_in, &Seed, &Metadata, error, false)) {

    std::streampos i = preprocessed_code_in.tellg();
    std::cerr.write(preprocessed_code_in.str().c_str(), i);
    std::cerr << " <- " << error << std::endl;
    return false;
  }

  return true;
}

bool Compile(const char *filename, std::string &error, bool compile_metadata) {

  std::ifstream source_code(filename, std::ios::binary | ios::in);
  if (!source_code.good()) {

    error = "unable to load file ";
    error += filename;
    return false;
  }

  bool r = Compile(source_code, filename, error, compile_metadata);
  source_code.close();
  return r;
}

bool Compile(const char *filename, std::string &error) {

  return Compile(filename, error, false);
}

bool Compile(std::istream &source_code, const std::string& filePath, std::string &error) {

  return Compile(source_code, filePath, error, false);
}

////////////////////////////////////////////////////////////////////////////////////////////////

thread_ret TDecompiler::Decompile(void *args) {

  P<TDecompiler> _this = (TDecompiler *)args;
  _this->spawned_ = 1;

  r_comp::Decompiler decompiler;
  decompiler.init(&r_exec::Metadata);

  vector<SysObject *> imported_objects;

  r_comp::Image *image = new r_comp::Image();
  image->add_objects(_this->objects_, imported_objects);
  image->object_names_.symbols_ = r_exec::Seed.object_names_.symbols_;

  std::ostringstream decompiled_code;
  decompiler.decompile(image, &decompiled_code, Utils::GetTimeReference(), imported_objects);

  if (_this->ostream_id_ == 0) {

    std::cout << _this->header_;
    std::cout << decompiled_code.str();
  } else {

    PipeOStream::Get(_this->ostream_id_ - 1) << _this->header_;
    PipeOStream::Get(_this->ostream_id_ - 1) << decompiled_code.str();
  }

  if (!_this->thread_)
    // We're running in diagnostic time without a thread. Just return.
    return 0;
  thread_ret_val(0);
}

TDecompiler::TDecompiler(uint32 ostream_id, std::string header) : _Object(), ostream_id_(ostream_id), header_(header), thread_(NULL), spawned_(0) {

  objects_.reserve(ObjectsInitialSize);
}

TDecompiler::~TDecompiler() {

  if (thread_)
    delete thread_;
}

void TDecompiler::add_object(Code *object) {

  objects_.push_back(object);
}

void TDecompiler::add_objects(const r_code::list<P<Code> > &objects) {

  r_code::list<P<Code> >::const_iterator o;
  for (o = objects.begin(); o != objects.end(); ++o)
    objects_.push_back(*o);
}

void TDecompiler::add_objects(const vector<P<Code> > &objects) {

  vector<P<Code> >::const_iterator o;
  for (o = objects.begin(); o != objects.end(); ++o)
    objects_.push_back(*o);
}

void TDecompiler::decompile() {

  if (_Mem::Get()->get_reduction_core_count() == 0 && _Mem::Get()->get_time_core_count() == 0)
    // We are running in diagnostic time with deterministic execution. Call Decompile in the current thread.
    Decompile(this);
  else {
    thread_ = Thread::New<_Thread>(Decompile, this);
    while (spawned_ == 0);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////

vector<PipeOStream *> PipeOStream::Streams_;

PipeOStream PipeOStream::NullStream_;

void PipeOStream::Open(uint8 count) {

#ifdef WINDOWS
  for (uint8 i = 0; i < count; ++i) {

    PipeOStream *p = new PipeOStream();
    p->init();

    Streams_.push_back(p);
  }
#endif
}

void PipeOStream::Close() {

#ifdef WINDOWS
  for (uint8 i = 0; i < Streams_.size(); ++i)
    delete Streams_[i];
  Streams_.clear();
#endif
}

PipeOStream &PipeOStream::Get(uint8 id) {

#ifdef WINDOWS
  if (id < Streams_.size())
    return *Streams_[id];
#endif
  return NullStream_;
}

PipeOStream::PipeOStream() : std::ostream(NULL)
#ifdef WINDOWS
  , pipe_read_(0), pipe_write_(0)
#endif
{
}

#ifdef WINDOWS
void PipeOStream::init() {

  SECURITY_ATTRIBUTES saAttr;
  saAttr.nLength = sizeof(saAttr);
  saAttr.bInheritHandle = true;
  saAttr.lpSecurityDescriptor = NULL;

  CreatePipe(&pipe_read_, &pipe_write_, &saAttr, 0);

  STARTUPINFO si;
  PROCESS_INFORMATION pi;

  ZeroMemory(&si, sizeof(si));
  si.cb = sizeof(si);
  ZeroMemory(&pi, sizeof(pi));

  char handle[255];
  itoa((int)pipe_read_, handle, 10);
  std::string command("output_window.exe ");
  command += std::string(handle);

  CreateProcess(NULL, // no module name (use command line)
    const_cast<char *>(command.c_str()), // command line
    NULL, // process handle not inheritable
    NULL, // thread handle not inheritable
    true, // handle inheritance
    CREATE_NEW_CONSOLE, // creation flags
    NULL, // use parent's environment block
    NULL, // use parent's starting directory
    &si, // pointer to STARTUPINFO structure
    &pi); // pointer to PROCESS_INFORMATION structure
}
#endif

PipeOStream::~PipeOStream() {

#ifdef WINDOWS
  if (pipe_read_ == 0)
    return;

  std::string stop("close_output_window");
  *this << stop;
  CloseHandle(pipe_read_);
  CloseHandle(pipe_write_);
#endif
}

PipeOStream &PipeOStream::operator <<(std::string &s) {

#ifdef WINDOWS // TODO: Implement for non-WINDOWS
  if (pipe_read_ == 0)
    return *this;

  uint32 to_write = s.length();
  uint32 written;

  WriteFile(pipe_write_, s.c_str(), to_write, &written, NULL);
#endif

  return *this;
}

PipeOStream& PipeOStream::operator <<(const char *s) {

#ifdef WINDOWS
  if (pipe_read_ == 0)
    return *this;

  uint32 to_write = strlen(s);
  uint32 written;

  WriteFile(pipe_write_, s, to_write, &written, NULL);
#endif

  return *this;
}

////////////////////////////////////////////////////////////////////////////////////////////////

uint16 RetrieveOpcode(const char *name) {

  return _Opcodes.find(name)->second;
}

bool InitOpcodes(const r_comp::Metadata& metadata) {
  unordered_map<uint16, set<string>> opcode_names;
  unordered_map<std::string, r_comp::Class>::const_iterator it;
  for (it = metadata.classes_.begin(); it != metadata.classes_.end(); ++it) {

    _Opcodes[it->first] = it->second.atom_.asOpcode();
    auto opcode = it->second.atom_.asOpcode();
    if (opcode_names.find(opcode) == opcode_names.end())
      // No existing entry for the opcode.
      opcode_names[opcode] = set<string>();
    opcode_names[opcode].insert(it->first.c_str());
  }
  for (it = metadata.sys_classes_.begin(); it != metadata.sys_classes_.end(); ++it) {

    _Opcodes[it->first] = it->second.atom_.asOpcode();
    auto opcode = it->second.atom_.asOpcode();
    if (opcode_names.find(opcode) == opcode_names.end())
      // No existing entry for the opcode.
      opcode_names[opcode] = set<string>();
    opcode_names[opcode].insert(it->first.c_str());
  }
  if (!r_code::SetOpcodeNames(opcode_names))
    return false;

  // load class Opcodes.
  View::ViewOpcode_ = _Opcodes.find("view")->second;

  Opcodes::View = _Opcodes.find("view")->second;
  Opcodes::PgmView = _Opcodes.find("pgm_view")->second;
  Opcodes::GrpView = _Opcodes.find("grp_view")->second;

  Opcodes::TI = _Opcodes.find("ti")->second;

  Opcodes::Ent = _Opcodes.find("ent")->second;
  Opcodes::Ont = _Opcodes.find("ont")->second;
  Opcodes::MkVal = _Opcodes.find("mk.val")->second;

  Opcodes::Grp = _Opcodes.find("grp")->second;

  Opcodes::Ptn = _Opcodes.find("ptn")->second;
  Opcodes::AntiPtn = _Opcodes.find("|ptn")->second;

  Opcodes::IPgm = _Opcodes.find("ipgm")->second;
  Opcodes::ICppPgm = _Opcodes.find("icpp_pgm")->second;

  Opcodes::Pgm = _Opcodes.find("pgm")->second;
  Opcodes::AntiPgm = _Opcodes.find("|pgm")->second;

  Opcodes::ICmd = _Opcodes.find("icmd")->second;
  Opcodes::Cmd = _Opcodes.find("cmd")->second;

  Opcodes::Fact = _Opcodes.find("fact")->second;
  Opcodes::AntiFact = _Opcodes.find("|fact")->second;

  Opcodes::Cst = _Opcodes.find("cst")->second;
  Opcodes::Mdl = _Opcodes.find("mdl")->second;

  Opcodes::ICst = _Opcodes.find("icst")->second;
  Opcodes::IMdl = _Opcodes.find("imdl")->second;

  Opcodes::Pred = _Opcodes.find("pred")->second;
  Opcodes::Goal = _Opcodes.find("goal")->second;

  Opcodes::Success = _Opcodes.find("success")->second;

  Opcodes::MkGrpPair = _Opcodes.find("mk.grp_pair")->second;

  Opcodes::MkRdx = _Opcodes.find("mk.rdx")->second;
  Opcodes::Perf = _Opcodes.find("perf")->second;

  Opcodes::MkNew = _Opcodes.find("mk.new")->second;

  Opcodes::MkLowRes = _Opcodes.find("mk.low_res")->second;
  Opcodes::MkLowSln = _Opcodes.find("mk.low_sln")->second;
  Opcodes::MkHighSln = _Opcodes.find("mk.high_sln")->second;
  Opcodes::MkLowAct = _Opcodes.find("mk.low_act")->second;
  Opcodes::MkHighAct = _Opcodes.find("mk.high_act")->second;
  Opcodes::MkSlnChg = _Opcodes.find("mk.sln_chg")->second;
  Opcodes::MkActChg = _Opcodes.find("mk.act_chg")->second;

  Opcodes::Sim = _Opcodes.find("sim")->second;
  Opcodes::Id = _Opcodes.find("id")->second;

  // load executive function Opcodes.
  Opcodes::Inject = _Opcodes.find("_inj")->second;
  Opcodes::Eject = _Opcodes.find("_eje")->second;
  Opcodes::Mod = _Opcodes.find("_mod")->second;
  Opcodes::Set = _Opcodes.find("_set")->second;
  Opcodes::NewClass = _Opcodes.find("_new_class")->second;
  Opcodes::DelClass = _Opcodes.find("_del_class")->second;
  Opcodes::LDC = _Opcodes.find("_ldc")->second;
  Opcodes::Swap = _Opcodes.find("_swp")->second;
  Opcodes::Prb = _Opcodes.find("_prb")->second;
  Opcodes::Stop = _Opcodes.find("_stop")->second;

  Opcodes::Gtr = _Opcodes.find("gtr")->second;
  Opcodes::Lsr = _Opcodes.find("lsr")->second;
  Opcodes::Gte = _Opcodes.find("gte")->second;
  Opcodes::Lse = _Opcodes.find("lse")->second;
  Opcodes::Add = _Opcodes.find("add")->second;
  Opcodes::Sub = _Opcodes.find("sub")->second;
  Opcodes::Mul = _Opcodes.find("mul")->second;
  Opcodes::Div = _Opcodes.find("div")->second;

  Opcodes::Var = _Opcodes.find("var")->second;

  // load std operators.
  uint16 operator_opcode = 0;
  Operator::Register(operator_opcode++, now);
  Operator::Register(operator_opcode++, rnd);
  Operator::Register(operator_opcode++, equ);
  Operator::Register(operator_opcode++, neq);
  Operator::Register(operator_opcode++, gtr);
  Operator::Register(operator_opcode++, lsr);
  Operator::Register(operator_opcode++, gte);
  Operator::Register(operator_opcode++, lse);
  Operator::Register(operator_opcode++, add);
  Operator::Register(operator_opcode++, sub);
  Operator::Register(operator_opcode++, mul);
  Operator::Register(operator_opcode++, div);
  Operator::Register(operator_opcode++, dis);
  Operator::Register(operator_opcode++, ln);
  Operator::Register(operator_opcode++, exp);
  Operator::Register(operator_opcode++, log);
  Operator::Register(operator_opcode++, e10);
  Operator::Register(operator_opcode++, syn);
  Operator::Register(operator_opcode++, ins);
  Operator::Register(operator_opcode++, red);
  Operator::Register(operator_opcode++, fvw);
  Operator::Register(operator_opcode++, is_sim);
  Operator::Register(operator_opcode++, minimum);
  Operator::Register(operator_opcode++, maximum);
  Operator::Register(operator_opcode++, id);

  return true;
}

static bool Init(FunctionLibrary* userOperatorLibrary,
          Timestamp (*time_base)()) {

  Now = time_base;

  if (!InitOpcodes(Metadata))
    return false;

  if (!userOperatorLibrary) // when no rMem is used.
    return true;

  // load usr operators and c++ programs.

  // Operators.
  typedef uint16 (*OpcodeRetriever)(const char *);
  typedef r_code::resized_vector<uint16> (*UserInit)(OpcodeRetriever);
  auto _Init = (UserInit)userOperatorLibrary->getFunction("Init");
  if (!_Init)
    return false;

  typedef uint16 (*UserGetOperatorCount)();
  auto GetOperatorCount = (UserGetOperatorCount)userOperatorLibrary->getFunction("GetOperatorCount");
  if (!GetOperatorCount)
    return false;

  typedef void (*UserGetOperatorName)(char *);
  auto GetOperatorName = (UserGetOperatorName)userOperatorLibrary->getFunction("GetOperatorName");
  if (!GetOperatorName)
    return false;

  Metadata.usr_classes_ = _Init(RetrieveOpcode);

  typedef bool (*UserOperator)(const Context &);

  uint16 operatorCount = GetOperatorCount();
  for (uint16 i = 0; i < operatorCount; ++i) {

    char op_name[256];
    memset(op_name, 0, 256);
    GetOperatorName(op_name);

    unordered_map<std::string, uint16>::iterator it = _Opcodes.find(op_name);
    if (it == _Opcodes.end()) {

      std::cerr << "Operator " << op_name << " is undefined" << std::endl;
      exit(-1);
    }
    auto op = (UserOperator)userOperatorLibrary->getFunction(op_name);
    if (!op)
      return false;

    Operator::Register(it->second, op);
  }

  // C++ programs.
  typedef uint16 (*UserGetProgramCount)();
  auto GetProgramCount = (UserGetProgramCount)userOperatorLibrary->getFunction("GetProgramCount");
  if (!GetProgramCount)
    return false;

  typedef void (*UserGetProgramName)(char *);
  auto GetProgramName = (UserGetProgramName)userOperatorLibrary->getFunction("GetProgramName");
  if (!GetProgramName)
    return false;

  typedef Controller *(*UserProgram)(_View *);

  uint16 programCount = GetProgramCount();
  for (uint16 i = 0; i < programCount; ++i) {

    char pgm_name[256];
    memset(pgm_name, 0, 256);
    GetProgramName(pgm_name);

    std::string _pgm_name = pgm_name;

    auto pgm = (UserProgram)userOperatorLibrary->getFunction(pgm_name);
    if (!pgm)
      return false;

    CPPPrograms::Register(_pgm_name, pgm);
  }

  // Callbacks.
  typedef uint16(*UserGetCallbackCount)();
  auto GetCallbackCount = (UserGetCallbackCount)userOperatorLibrary->getFunction("GetCallbackCount");
  if (!GetCallbackCount)
    return false;

  typedef void(*UserGetCallbackName)(char *);
  auto GetCallbackName = (UserGetCallbackName)userOperatorLibrary->getFunction("GetCallbackName");
  if (!GetCallbackName)
    return false;

  typedef bool(*UserCallback)(microseconds, bool, const char *, uint8, Code **);

  uint16 callbackCount = GetCallbackCount();
  for (uint16 i = 0; i < callbackCount; ++i) {

    char callback_name[256];
    memset(callback_name, 0, 256);
    GetCallbackName(callback_name);

    std::string _callback_name = callback_name;

    auto callback = (UserCallback)userOperatorLibrary->getFunction(callback_name);
    if (!callback)
      return false;

    Callbacks::Register(_callback_name, callback);
  }

  std::cout << "> user-defined operator library loaded" << std::endl;

  return true;
}

bool Init(FunctionLibrary* userOperatorLibrary,
  Timestamp (*time_base)(),
  const char *seed_path) {

  std::string error;
  if (!Compile(seed_path, error, true)) {

    std::cerr << error << std::endl;
    return false;
  }

  return Init(userOperatorLibrary, time_base);
}

bool Init(FunctionLibrary* userOperatorLibrary,
  Timestamp (*time_base)(),
  const r_comp::Metadata &metadata,
  const r_comp::Image &seed) {

  Metadata = metadata;
  Seed = seed;

  return Init(userOperatorLibrary, time_base);
}

uint16 GetOpcode(const char *name) {

  unordered_map<std::string, uint16>::iterator it = _Opcodes.find(name);
  if (it == _Opcodes.end())
    return 0xFFFF;
  return it->second;
}

std::string GetAxiomName(const uint16 index) {

  return Compiler.getObjectName(index);
}

bool hasUserDefinedOperators(const uint16 opcode) {
  const std::vector<uint16> *usr_defined_opcodes = Metadata.usr_classes_.as_std();
  return std::find(usr_defined_opcodes->begin(), usr_defined_opcodes->end(), opcode) != usr_defined_opcodes->end();
}

bool hasUserDefinedOperators(const std::string class_name) {
  if (_Opcodes.find(class_name) == _Opcodes.end()) {
    return false;
  }
  return hasUserDefinedOperators(_Opcodes[class_name]);
}

}
