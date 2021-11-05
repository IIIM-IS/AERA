//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
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

#include "decompiler.h"
#include "IODevices\TCP\tcp_io_device.h"
#include "test_mem.h"
#include "init.h"
#include "image_impl.h"
#include "settings.h"


//#define DECOMPILE_ONE_BY_ONE

using namespace std::chrono;
using namespace r_code;
using namespace r_comp;

r_exec::View *build_view(Timestamp time, Code* rstdin) { // this is application dependent WRT view->sync.

  r_exec::View *view = new r_exec::View();
  const uint32 arity = VIEW_ARITY; // reminder: opcode not included in the arity.
  uint16 write_index = 0;
  uint16 extent_index = arity + 1;

  view->code(VIEW_OPCODE) = Atom::SSet(r_exec::View::ViewOpcode_, arity);
  view->code(VIEW_SYNC) = Atom::Float(_View::SYNC_ONCE); // sync on front.
  view->code(VIEW_IJT) = Atom::IPointer(extent_index); // iptr to injection time.
  view->code(VIEW_SLN) = Atom::Float(1.0); // sln.
  view->code(VIEW_RES) = Atom::Float(1); // res is set to 1 upr of the destination group.
  view->code(VIEW_HOST) = Atom::RPointer(0); // stdin/stdout is the only reference.
  view->code(VIEW_ORG) = Atom::Nil(); // org.

  Utils::SetTimestamp(&view->code(extent_index), time);

  view->references_[0] = rstdin;

  return view;
}

Code *make_object(r_exec::_Mem *mem, Code* rstdin, float32 i) {

  Code *object = new r_exec::LObject(mem);
  //object->code(0)=Atom::Marker(r_exec::GetOpcode("mk.val"),4); // Caveat: arity does not include the opcode.
  object->code(0) = Atom::Marker(r_exec::Opcodes::MkVal, 4); // Caveat: arity does not include the opcode.
  object->code(1) = Atom::RPointer(0);
  object->code(2) = Atom::RPointer(1);
  object->code(3) = Atom::Float(i);
  object->code(4) = Atom::Float(1); // psln_thr.

  object->set_reference(0, rstdin);
  object->set_reference(1, rstdin);

  return object;
}

void test_injection(r_exec::_Mem *mem, float32 n) {

  Code* rstdin = mem->get_stdin();

  // mem->timings_report.clear();

  // uint64 tt1 = 0;
  // uint64 tt2 = 0;
  // uint64 tt3 = 0;
  // uint64 tt4 = 0;

  // vector<uint64> v1, v2, v3, v4;
  // v1.reserve(n);
  // v2.reserve(n);
  // v3.reserve(n);
  // v4.reserve(n);

  auto t0 = r_exec::Now();

  for (float32 i = 0; i < n; ++i) {
    // tt1 = r_exec::Now();
    Code* object = make_object(mem, rstdin, i);

    auto now = r_exec::Now();
    // v1.push_back(now - tt1);

            // Build a fact.
    // tt2 = now;
    Code *fact = new r_exec::Fact(object, now, now, 1, 1);
    // uint64 tt = r_exec::Now();
    // v2.push_back(tt - tt2);

            // Build a default view for the fact.
    // tt3 = tt;
    r_exec::View *view = build_view(now, rstdin);
    // tt = r_exec::Now();
    // v3.push_back(tt - tt3);

            // Inject the view.
    // tt4 = tt;
    view->set_object(fact);
    mem->inject(view);
    // v4.push_back(r_exec::Now() - tt4);
  }

  auto t1 = r_exec::Now();
  auto t2 = t1 - t0;
  std::cout << "for-loop total time: " << duration_cast<microseconds>(t2).count() << std::endl;
  /* uint64 acc=0;
      for(uint32 i=0;i<n;++i){
          acc+=v1[i];
          std::cout<<v1[i]<<'\t';
      }
      std::cout<<"\nfor-loop accumulated time make_object: "<<acc<< std::endl;
      acc=0;
      for(uint32 i=0;i<n;++i){
          acc+=v2[i];
  // std::cout<<v2[i]<<'/'<<mem->timings_report[i]<<'\t';
          std::cout<<v2[i]<<'\t';
      }
      std::cout<<"\nfor-loop accumulated time new Fact   : "<<acc<< std::endl;
      acc=0;
      for(uint32 i=0;i<n;++i){
          acc+=v3[i];
          std::cout<<v3[i]<<'\t';
      }
      std::cout<<"\nfor-loop accumulated time build_view : "<<acc<< std::endl;
      acc=0;
      for(uint32 i=0;i<n;++i){
          acc+=v4[i];
          std::cout<<v4[i]<<'\t';
      }
      std::cout<<"\nfor-loop accumulated time mem->inject: "<<acc<< std::endl;
  */
}

void test_many_injections(r_exec::_Mem *mem, milliseconds sampling_period, uint32 nRuns, float32 nObjects) {
  for (; nRuns; --nRuns) {
    auto start = r_exec::Now();
    std::cout << nRuns << '\t';
    test_injection(mem, nObjects);
    auto taken = r_exec::Now() - start;
    if (taken > sampling_period)
      std::cout << "Good grief! I exceeded the sampling period!" << std::endl;
    else
      Thread::Sleep(sampling_period - taken);
  }
}

void decompile(Decompiler &decompiler, r_comp::Image *image, Timestamp::duration time_offset, bool ignore_named_objects) {

#ifdef DECOMPILE_ONE_BY_ONE
  uint32 object_count = decompiler.decompile_references(image);
  std::cout << object_count << " objects in the image\n";
  while (1) {

    std::cout << "> which object (-1 to exit)?\n";
    int32 index; std::cin >> index;
    if (index == -1)
      break;
    if (index >= object_count) {

      std::cout << "> there is only " << object_count << " objects\n";
      continue;
    }
    std::ostringstream decompiled_code;
    decompiler.decompile_object(index, &decompiled_code, time_offset);
    std::cout << "\n\n> DECOMPILATION. TimeReference " << Time::ToString_seconds(time_offset) << "\n\n" <<
      decompiled_code.str() << std::endl;
  }
#else
  std::ostringstream decompiled_code;
  uint32 object_count = decompiler.decompile(image, &decompiled_code, time_offset, ignore_named_objects);
  //uint32 object_count=image->code_segment_.objects.size();
  std::cout << "\n\n> DECOMPILATION. TimeReference " << Time::ToString_seconds(time_offset) << "\n\n" << 
    decompiled_code.str() << std::endl;
  std::cout << "> image taken at: " << Time::ToString_year(image->timestamp_) << std::endl;
  std::cout << "> " << object_count << " objects\n";
#endif
}

void write_to_file(r_comp::Image *image, std::string &image_path, Decompiler *decompiler, Timestamp::duration time_offset) {

  ofstream output(image_path.c_str(), ios::binary | ios::out);
  r_code::Image<r_code::ImageImpl> *serialized_image = image->serialize<r_code::Image<r_code::ImageImpl> >();
  r_code::Image<r_code::ImageImpl>::Write(serialized_image, output);
  output.close();
  delete serialized_image;

  if (decompiler) {

    ifstream input(image_path.c_str(), ios::binary | ios::in);
    if (!input.good())
      return;

    r_code::Image<r_code::ImageImpl> *read_image = (r_code::Image<r_code::ImageImpl> *)r_code::Image<r_code::ImageImpl>::Read(input);
    input.close();

    resized_vector<Code *> objects;
    r_comp::Image *temp_image = new r_comp::Image();
    temp_image->load(read_image);

    decompile(*decompiler, temp_image, time_offset, false);
    delete temp_image;

    delete read_image;
  }
}

int32 main(int argc, char **argv) {

  core::Time::Init(1000);

  Settings settings;
  const char* file_name = (argc >= 2 ? argv[1] : "settings.xml");
  if (!settings.load(file_name))
    return 1;

  ofstream runtime_output_stream;
  if (settings.runtime_output_file_path_ != "") {
    runtime_output_stream.open(settings.runtime_output_file_path_);
    if (!runtime_output_stream.is_open()) {
      std::cout << "Cannot open runtime_output_file_path \"" << settings.runtime_output_file_path_ << "\"" << std::endl;
      return 2;
    }
  }

  std::cout << "> compiling ...\n";
  if (settings.reduction_core_count_ == 0 && settings.time_core_count_ == 0) {
    // Below, we will use run_in_diagnostic_time.
    // Initialize the diagnostic time to the real now.
    r_exec::_Mem::diagnostic_time_now_ = Time::Get();
    if (!r_exec::Init
    (settings.usr_operator_path_.c_str(), r_exec::_Mem::get_diagnostic_time_now,
      settings.usr_class_path_.c_str()))
      return 2;
  }
  else {
    if (!r_exec::Init(settings.usr_operator_path_.c_str(), Time::Get, settings.usr_class_path_.c_str()))
      return 2;
  }

  srand(duration_cast<microseconds>(r_exec::Now().time_since_epoch()).count());
  Random::Init();

  std::string error;
  if (!r_exec::Compile(settings.source_file_name_.c_str(), error)) {

    std::cerr << " <- " << error << std::endl;
    return 3;
  } else {

    std::cout << "> ... done\n";

    r_exec::PipeOStream::Open(settings.debug_windows_);

    Decompiler decompiler;
    decompiler.init(&r_exec::Metadata);

    r_comp::Image *image;

    r_exec::_Mem* mem;
    if (settings.io_device_.compare("test_mem") == 0) {
      if (settings.get_objects_)
        mem = new TestMem<r_exec::LObject, r_exec::MemStatic>();
      else
        mem = new TestMem<r_exec::LObject, r_exec::MemVolatile>();
    }
    else if (settings.io_device_.compare("tcp_io_device") == 0) {
      string port = "8080";
      int err = 0;
      if (settings.get_objects_) {
        mem = new tcp_io_device::TcpIoDevice<r_exec::LObject, r_exec::MemStatic>();
        err = static_cast<tcp_io_device::TcpIoDevice<r_exec::LObject, r_exec::MemStatic>*>(mem)->initTCP(port);
      }
      else {
        mem = new tcp_io_device::TcpIoDevice<r_exec::LObject, r_exec::MemVolatile>();
        err = static_cast<tcp_io_device::TcpIoDevice<r_exec::LObject, r_exec::MemVolatile>*>(mem)->initTCP(port);
      }
      if (err != 0) {
        cout << "ERROR: Could not connect to a TCP client" << endl;
        delete mem;
        return err;
      }
    }

    if (runtime_output_stream.is_open())
      // Use the debug stream from settings.xml.
      mem->set_default_runtime_output_stream(&runtime_output_stream);

    resized_vector<r_code::Code *> ram_objects;
    r_exec::Seed.get_objects(mem, ram_objects);

    mem->init(microseconds(settings.base_period_),
      settings.reduction_core_count_,
      settings.time_core_count_,
      settings.mdl_inertia_sr_thr_,
      settings.mdl_inertia_cnt_thr_,
      settings.tpx_dsr_thr_,
      microseconds(settings.min_sim_time_horizon_),
      microseconds(settings.max_sim_time_horizon_),
      settings.sim_time_horizon_factor_,
      microseconds(settings.tpx_time_horizon_),
      microseconds(settings.perf_sampling_period_),
      settings.float_tolerance_,
      microseconds(settings.time_tolerance_),
      seconds(settings.primary_thz_),
      seconds(settings.secondary_thz_),
      settings.debug_,
      settings.ntf_mk_resilience_,
      settings.goal_pred_success_resilience_,
      settings.probe_level_,
      settings.trace_levels_,
      settings.enable_assumptions_,
      settings.keep_invalidated_objects_);

    uint32 stdin_oid;
    std::string stdin_symbol("stdin");
    uint32 stdout_oid;
    std::string stdout_symbol("stdout");
    uint32 self_oid;
    std::string self_symbol("self");
    unordered_map<uint32, std::string>::const_iterator n;
    for (n = r_exec::Seed.object_names_.symbols_.begin(); n != r_exec::Seed.object_names_.symbols_.end(); ++n) {

      if (n->second == stdin_symbol)
        stdin_oid = n->first;
      else if (n->second == stdout_symbol)
        stdout_oid = n->first;
      else if (n->second == self_symbol)
        self_oid = n->first;
    }

    if (!mem->load(ram_objects.as_std(), stdin_oid, stdout_oid, self_oid))
      return 4;
    auto starting_time = mem->start();

    if (settings.reduction_core_count_ == 0 && settings.time_core_count_ == 0) {
      std::cout << "> running for " << settings.run_time_ << " ms in diagnostic time\n\n";
      mem->run_in_diagnostic_time(milliseconds(settings.run_time_));
    }
    else {
      std::cout << "> running for " << settings.run_time_ << " ms\n\n";
      Thread::Sleep(milliseconds(settings.run_time_));
    }

    /*Thread::Sleep(settings.run_time/2);
    test_many_injections(mem,
        argc > 2 ? atoi(argv[2]) : 100, // sampling period in ms
        argc > 3 ? atoi(argv[3]) : 600, // number of batches
        argc > 4 ? atoi(argv[4]) : 66); // number of objects per batch
    Thread::Sleep(settings.run_time/2);*/

    std::cout << "\n> shutting rMem down...\n";
    mem->stop();

    if (settings.get_objects_) {
      //TimeProbe probe;
      //probe.set();
      image = mem->get_objects(settings.keep_invalidated_objects_);
      //probe.check();
      image->object_names_.symbols_ = r_exec::Seed.object_names_.symbols_;

      if (settings.write_objects_)
        write_to_file(image, settings.objects_path_, settings.test_objects_ ? &decompiler : NULL, starting_time.time_since_epoch());

      if (settings.decompile_objects_ && (!settings.write_objects_ || !settings.test_objects_)) {

        if (settings.decompile_to_file_) { // argv[2] is a file to redirect the decompiled code to.

          std::ofstream outfile;
          outfile.open(settings.decompilation_file_path_.c_str(), std::ios_base::trunc);
          std::streambuf *coutbuf = std::cout.rdbuf(outfile.rdbuf());

          decompile(decompiler, image, starting_time.time_since_epoch(), settings.ignore_named_objects_);

          std::cout.rdbuf(coutbuf);
          outfile.close();
        } else
          decompile(decompiler, image, starting_time.time_since_epoch(), settings.ignore_named_objects_);
      }
      delete image;
      //std::cout<<"get_image(): "<<probe.us()<<"us"<<std::endl;
    }

    if (settings.get_models_) {
      //TimeProbe probe;
      //probe.set();
      image = mem->get_models();
      //probe.check();
      image->object_names_.symbols_ = r_exec::Seed.object_names_.symbols_;

      if (settings.write_models_)
        write_to_file(image, settings.models_path_, settings.test_models_ ? &decompiler : NULL, starting_time.time_since_epoch());

      if (settings.decompile_models_ && (!settings.write_models_ || !settings.test_models_)) {

        if (argc > 2) { // argv[2] is a file to redirect the decompiled code to.

          std::ofstream outfile;
          outfile.open(argv[2], std::ios_base::trunc);
          std::streambuf *coutbuf = std::cout.rdbuf(outfile.rdbuf());

          decompile(decompiler, image, starting_time.time_since_epoch(), settings.ignore_named_models_);

          std::cout.rdbuf(coutbuf);
          outfile.close();
        } else
          decompile(decompiler, image, starting_time.time_since_epoch(), settings.ignore_named_models_);
      }
      delete image;
      //std::cout<<"get_models(): "<<probe.us()<<"us"<<std::endl;
    }
    delete mem;

    r_exec::PipeOStream::Close();
  }

  return 0;
}
