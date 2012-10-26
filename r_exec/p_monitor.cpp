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

#include	"p_monitor.h"
#include	"mem.h"
#include	"mdl_controller.h"


namespace	r_exec{

	PMonitor::PMonitor(	MDLController	*controller,
						BindingMap		*bindings,
						Fact			*prediction,
						bool			rate_failures):Monitor(controller,bindings,prediction),rate_failures(rate_failures){	// prediction is f0->pred->f1->obj; not simulated.

		prediction_target=prediction->get_pred()->get_target();	// f1.
		uint64	now=Now();

		bindings->reset_fwd_timings(prediction_target);

		MonitoringJob<PMonitor>	*j=new	MonitoringJob<PMonitor>(this,prediction_target->get_before()+Utils::GetTimeTolerance());
		_Mem::Get()->pushTimeJob(j);
	}

	PMonitor::~PMonitor(){
	}

	bool	PMonitor::reduce(_Fact	*input){	// input is always an actual fact.

		if(target->is_invalidated()){//std::cout<<Time::ToString_seconds(Now()-Utils::GetTimeReference())<<" "<<std::hex<<this<<std::dec<<" target has been invalidated\n";
		return	true;}

		if(target->get_pred()->grounds_invalidated(input)){	// input is a counter-evidence for one of the antecedents: abort.

			target->invalidate();
			return	true;
		}

		Pred	*prediction=input->get_pred();
		if(prediction){

			switch(prediction->get_target()->is_evidence(prediction_target)){
			case	MATCH_SUCCESS_POSITIVE:	// predicted confirmation, skip.
				return	false;
			case	MATCH_SUCCESS_NEGATIVE:
				if(prediction->get_target()->get_cfd()>prediction_target->get_cfd()){

					target->invalidate();	// a predicted counter evidence is stronger than the target, invalidate and abort: don't rate the model.
					return	true;
				}else
					return	false;
			case	MATCH_FAILURE:
				return	false;
			}
		}else{
			//uint32	oid=input->get_oid();
			switch(((Fact	*)input)->is_evidence(prediction_target)){
			case	MATCH_SUCCESS_POSITIVE:
				//std::cout<<Time::ToString_seconds(Now()-Utils::GetTimeReference())<<" "<<std::hex<<this<<std::dec<<" target: "<<prediction_target->get_reference(0)->code(MK_VAL_VALUE).asFloat()<<" reduced: "<<input->get_oid()<<" positive\n";
				controller->register_pred_outcome(target,true,input,input->get_cfd(),rate_failures);
				return	true;
			case	MATCH_SUCCESS_NEGATIVE:
				//std::cout<<Time::ToString_seconds(Now()-Utils::GetTimeReference())<<" "<<std::hex<<this<<std::dec<<" target: "<<prediction_target->get_reference(0)->code(MK_VAL_VALUE).asFloat()<<" reduced: "<<input->get_oid()<<" negative\n";
				if(rate_failures)
					controller->register_pred_outcome(target,false,input,input->get_cfd(),rate_failures);
				return	true;
			case	MATCH_FAILURE:
				//std::cout<<Time::ToString_seconds(Now()-Utils::GetTimeReference())<<" "<<std::hex<<this<<std::dec<<" target: "<<prediction_target->get_reference(0)->code(MK_VAL_VALUE).asFloat()<<" reduced: "<<input->get_oid()<<" failure\n";
				return	false;
			}
		}
	}

	void	PMonitor::update(uint64	&next_target){	// executed by a time core, upon reaching the expected time of occurrence of the target of the prediction.

		if(!target->is_invalidated()){	// received nothing matching the target's object so far (neither positively nor negatively).

			if(rate_failures)
				controller->register_pred_outcome(target,false,NULL,1,rate_failures);
		}
		controller->remove_monitor(this);
		next_target=0;
	}
}