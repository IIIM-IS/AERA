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

#include <set>

#include "decompiler.h"
#include "init.h"
#include "opcodes.h"
#include "../r_code/image_impl.h"

#include "settings.h"
#include "correlator.h"

using namespace std;
using namespace std::chrono;
using namespace r_code;

//#define DECOMPILE_ONE_BY_ONE

void decompile(r_comp::Decompiler &decompiler, r_comp::Image *image, Timestamp::duration time_offset) {

#ifdef DECOMPILE_ONE_BY_ONE
  uint32 object_count = decompiler.decompile_references(image);
  std::cout << object_count << " objects in the image\n";
  while (1) {

    std::cout << "Which object (-1 to exit)?\n";
    int32 index; std::cin >> index;
    if (index == -1)
      break;
    if (index >= object_count) {

      std::cout << "there is only " << object_count << " objects\n";
      continue;
    }
    std::ostringstream decompiled_code;
    decompiler.decompile_object(index, &decompiled_code, time_offset);
    std::cout << "... done\n";
    std::cout << "\n\nDECOMPILATION\n\n" << decompiled_code.str() << std::endl;
  }
#else
  std::cout << "\ndecompiling ...\n";
  std::ostringstream decompiled_code;
  uint32 object_count = decompiler.decompile(image, &decompiled_code, time_offset, false);
  std::cout << "... done\n";
  std::cout << "\n\nDECOMPILATION\n\n" << decompiled_code.str() << std::endl;
  std::cout << "Image taken at: " << Time::ToString_year(image->timestamp_) << std::endl << std::endl;
  std::cout << object_count << " objects\n";
#endif
}

int32 main(int argc, char **argv) {

  core::Time::Init(1000);

  CorrelatorTestSettings settings;
  if (!settings.load(argv[1]))
    return 1;

  std::cout << "compiling ...\n";
  r_exec::Init(settings.usr_operator_path_.c_str(), Time::Get, settings.usr_class_path_.c_str());
  std::cout << "... done\n";

  r_comp::Decompiler decompiler;
  decompiler.init(&r_exec::Metadata);

  Correlator correlator;

  // Feed the Correlator incrementally with episodes related to the pursuit of one goal.

  // First, get an image containing all the episodes.
  std::string image_path = settings.use_case_path_;
  image_path += "/";
  image_path += settings.image_name_;

  ifstream input(image_path.c_str(), ios::binary | ios::in);
  if (!input.good())
    return 1;

  r_code::Image<ImageImpl> *img = (r_code::Image<ImageImpl> *)r_code::Image<ImageImpl>::Read(input);
  input.close();

  resized_vector<Code *> objects;
  r_comp::Image *_i = new r_comp::Image();
  _i->load(img);
  _i->get_objects<LocalObject>(objects);

  decompile(decompiler, _i, seconds(0));
  delete _i;

  delete img;

  // Second, filter objects: retain only those which are actual inputs in stdin and store them in a time-ordered list.
  uint32 stdin_oid;
  std::string stdin_str("stdin");
  unordered_map<uint32, std::string>::const_iterator n;
  for (n = _i->object_names_.symbols_.begin(); n != _i->object_names_.symbols_.end(); ++n)
    if (n->second == stdin_str) {

      stdin_oid = n->first;
      break;
    }

  std::set<_View *, _View::Less> correlator_inputs;
  for (uint32 i = 0; i < objects.size(); ++i) {

    Code *object = objects[i];
    if (object->code(0).asOpcode() == r_exec::Opcodes::IPgm)
      continue;

    unordered_set<_View *, _View::Hash, _View::Equal>::const_iterator v;
    for (v = object->views_.begin(); v != object->views_.end(); ++v) {

      if (!(*v)->references_[0])
        continue;

      if ((*v)->references_[0]->get_oid() == stdin_oid) {

        correlator_inputs.insert(*v);
        break;
      }
    }
  }

  // Third, feed the Correlator with the episodes, one by one.
  // Episodes are delimited with an object of the class episode_end (See user.classes.replicode).
  std::string episode_end_class_name = "episode_end";
  uint16 episode_end_opcode = r_exec::Metadata.get_class(episode_end_class_name)->atom_.asOpcode();
  uint16 episode_count = 0;

  std::set<_View *, _View::Less>::const_iterator v;
  for (v = correlator_inputs.begin(); v != correlator_inputs.end(); ++v) {

    if ((*v)->object_->code(0).asOpcode() == episode_end_opcode) {

      // Get the result.
      CorrelatorOutput *output = correlator.get_output();
      output->trace();
      delete output;

      ++episode_count;
    } else
      correlator.take_input(*v);
  }

  std::cout << episode_count << " episodes\n";

  return 0;
}
