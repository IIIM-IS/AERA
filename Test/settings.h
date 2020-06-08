//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ HUMANOBS - Replicode Test
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

#ifndef settings_h
#define settings_h

#include "../submodules/CoreLibrary/CoreLibrary/xml_parser.h"


class Settings {
public:
  // Load.
  std::string usr_operator_path_;
  std::string usr_class_path_;
  std::string source_file_name_;

  // Init.
  core::uint32 base_period_;
  core::uint32 reduction_core_count_;
  core::uint32 time_core_count_;

  // System.
  core::float32 mdl_inertia_sr_thr_;
  core::uint32 mdl_inertia_cnt_thr_;
  core::float32 tpx_dsr_thr_;
  core::uint32 min_sim_time_horizon_;
  core::uint32 max_sim_time_horizon_;
  core::float32 sim_time_horizon_factor_;
  core::uint32 tpx_time_horizon_;
  core::uint32 perf_sampling_period_;
  core::float32 float_tolerance_;
  core::uint32 time_tolerance_;
  core::uint64 primary_thz_;
  core::uint64 secondary_thz_;

  // Debug.
  bool debug_;
  core::uint32 ntf_mk_resilience_;
  core::uint32 goal_pred_success_resilience_;
  core::uint32 debug_windows_;
  std::string debug_stream_file_path_;
  core::uint32 trace_levels_;
  bool get_objects_;
  bool keep_invalidated_objects_;
  bool decompile_objects_;
  bool decompile_to_file_;
  std::string decompilation_file_path_;
  bool ignore_named_objects_;
  bool write_objects_;
  std::string objects_path_;
  bool test_objects_;

  //Run.
  core::uint32 run_time_;
  core::uint32 probe_level_;
  bool enable_assumptions_;
  bool get_models_;
  bool decompile_models_;
  bool ignore_named_models_;
  bool write_models_;
  std::string models_path_;
  bool test_models_;

  bool load(const char *file_name) {

    core::XMLNode mainNode = core::XMLNode::openFileHelper(file_name, "AERAConfiguration");
    if (!mainNode) {

      std::cerr << "> Error: AERAConfiguration is unreadable" << std::endl;
      return false;
    }

    core::XMLNode load = mainNode.getChildNode("Load");
    if (!!load) {

      usr_operator_path_ = load.getAttribute("usr_operator_path");
      usr_class_path_ = load.getAttribute("usr_class_path");
      source_file_name_ = load.getAttribute("source_file_name");
    } else {

      std::cerr << "> Error: Load section is unreadable" << std::endl;
      return false;
    }

    core::XMLNode init = mainNode.getChildNode("Init");
    if (!!init) {

      const char *base_period = init.getAttribute("base_period");
      const char *reduction_core_count = init.getAttribute("reduction_core_count");
      const char *time_core_count = init.getAttribute("time_core_count");

      base_period_ = atoi(base_period);
      reduction_core_count_ = atoi(reduction_core_count);
      time_core_count_ = atoi(time_core_count);
    } else {

      std::cerr << "> Error: Init section is unreadable" << std::endl;
      return false;
    }

    core::XMLNode system = mainNode.getChildNode("System");
    if (!!system) {

      const char *mdl_inertia_sr_thr = system.getAttribute("mdl_inertia_sr_thr");
      const char *mdl_inertia_cnt_thr = system.getAttribute("mdl_inertia_cnt_thr");
      const char *tpx_dsr_thr = system.getAttribute("tpx_dsr_thr");
      const char *min_sim_time_horizon = system.getAttribute("min_sim_time_horizon");
      const char *max_sim_time_horizon = system.getAttribute("max_sim_time_horizon");
      const char *sim_time_horizon_factor = system.getAttribute("sim_time_horizon_factor");
      const char *tpx_time_horizon = system.getAttribute("tpx_time_horizon");
      const char *perf_sampling_period = system.getAttribute("perf_sampling_period");
      const char *float_tolerance = system.getAttribute("float_tolerance");
      const char *time_tolerance = system.getAttribute("time_tolerance");
      const char *primary_thz = system.getAttribute("primary_thz");
      const char *secondary_thz = system.getAttribute("secondary_thz");

      mdl_inertia_sr_thr_ = atof(mdl_inertia_sr_thr);
      mdl_inertia_cnt_thr_ = atoi(mdl_inertia_cnt_thr);
      tpx_dsr_thr_ = atof(tpx_dsr_thr);
      min_sim_time_horizon_ = atoi(min_sim_time_horizon);
      max_sim_time_horizon_ = atoi(max_sim_time_horizon);
      sim_time_horizon_factor_ = atof(sim_time_horizon_factor);
      tpx_time_horizon_ = atoi(tpx_time_horizon);
      perf_sampling_period_ = atoi(perf_sampling_period);
      float_tolerance_ = atof(float_tolerance);
      time_tolerance_ = atoi(time_tolerance);
      primary_thz_ = atoi(primary_thz);
      secondary_thz_ = atoi(secondary_thz);
    } else {

      std::cerr << "> Error: System section is unreadable" << std::endl;
      return false;
    }

    core::XMLNode debug = mainNode.getChildNode("Debug");
    if (!!debug) {

      const char *debug_string = debug.getAttribute("debug");
      const char *debug_windows = debug.getAttribute("debug_windows");
      const char *debug_stream_file_path = debug.getAttribute("debug_stream_file_path");
      const char *trace_levels = debug.getAttribute("trace_levels");

      debug_ = (strcmp(debug_string, "yes") == 0);
      debug_windows_ = atoi(debug_windows);
      debug_stream_file_path_ = debug_stream_file_path;
      sscanf(trace_levels, "%x", &trace_levels_);

      core::XMLNode resilience = debug.getChildNode("Resilience");
      if (!!resilience) {

        const char *ntf_mk_resilience = resilience.getAttribute("ntf_mk_resilience");
        const char *goal_pred_success_resilience = resilience.getAttribute("goal_pred_success_resilience");

        ntf_mk_resilience_ = atoi(ntf_mk_resilience);
        goal_pred_success_resilience_ = atoi(goal_pred_success_resilience);
      } else {

        std::cerr << "> Error: Debug/Resilience section is unreadable" << std::endl;
        return false;
      }
      core::XMLNode objects = debug.getChildNode("Objects");
      if (!!objects) {

        const char *get_objects = objects.getAttribute("get_objects");
        const char *keep_invalidated_objects = objects.getAttribute("keep_invalidated_objects");
        const char *decompile_objects = objects.getAttribute("decompile_objects");
        const char *decompile_to_file = objects.getAttribute("decompile_to_file");
        decompilation_file_path_ = objects.getAttribute("decompilation_file_path");
        const char *ignore_named_objects = objects.getAttribute("ignore_named_objects");
        const char *write_objects = objects.getAttribute("write_objects");
        const char *test_objects = objects.getAttribute("test_objects");

        get_objects_ = (strcmp(get_objects, "yes") == 0);
        keep_invalidated_objects_ = (strcmp(keep_invalidated_objects, "yes") == 0);
        decompile_objects_ = (strcmp(decompile_objects, "yes") == 0);
        decompile_to_file_ = (strcmp(decompile_to_file, "yes") == 0);
        ignore_named_objects_ = (strcmp(ignore_named_objects, "yes") == 0);
        write_objects_ = (strcmp(write_objects, "yes") == 0);
        if (write_objects_) {

          objects_path_ = objects.getAttribute("objects_path");
          test_objects_ = (strcmp(test_objects, "yes") == 0);
        }
      } else {

        std::cerr << "> Error: Debug/Objects section is unreadable" << std::endl;
        return false;
      }
    } else {

      std::cerr << "> Error: Debug section is unreadable" << std::endl;
      return false;
    }

    core::XMLNode run = mainNode.getChildNode("Run");
    if (!!run) {

      const char *run_time = run.getAttribute("run_time");
      const char *probe_level = run.getAttribute("probe_level");
      const char *enable_assumptions = run.getAttribute("enable_assumptions");

      run_time_ = atoi(run_time);
      probe_level_ = atoi(probe_level);
      enable_assumptions_ = (strcmp(enable_assumptions, "yes") == 0);

      core::XMLNode models = run.getChildNode("Models");
      if (!!models) {

        const char *get_models = models.getAttribute("get_models");
        const char *decompile_models = models.getAttribute("decompile_models");
        const char *ignore_named_models = models.getAttribute("ignore_named_models");
        const char *write_models = models.getAttribute("write_models");
        const char *test_models = models.getAttribute("test_models");

        get_models_ = (strcmp(get_models, "yes") == 0);
        decompile_models_ = (strcmp(decompile_models, "yes") == 0);
        ignore_named_models_ = (strcmp(ignore_named_models, "yes") == 0);
        write_models_ = (strcmp(write_models, "yes") == 0);
        if (write_models_) {

          models_path_ = models.getAttribute("models_path");
          test_models_ = (strcmp(test_models, "yes") == 0);
        }
      } else {

        std::cerr << "> Error: Run/Models section is unreadable" << std::endl;
        return false;
      }
    } else {

      std::cerr << "> Error: Run section is unreadable" << std::endl;
      return false;
    }

    return true;
  }
};


#endif
