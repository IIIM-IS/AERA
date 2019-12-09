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

#ifndef	mem_h
#define	mem_h

#include	"reduction_core.h"
#include	"time_core.h"
#include	"pgm_overlay.h"
#include	"binding_map.h"
#include	"dll.h"

#include	<list>
#include	<deque>

#include	"../r_code/list.h"
#include	"../r_comp/segments.h"

#include	"../submodules/CoreLibrary/CoreLibrary/pipe.h"


namespace	r_exec{

	// The rMem.
	// Maintains 2 pipes of jobs (injection, update, etc.). each job is processed asynchronously by instances of ReductionCore and TimeCore.
	// Pipes and threads are created at starting time and deleted at stopping time.
	// Groups and IPGMControllers are cleared up when only held by jobs;
	// 	- when a group is not projected anywhere anymore, it is invalidated (it releases all its views) and when a job attempts an update, the latter is cancelled.
	// 	- when a reduction core attempts to perform a reduction for an ipgm-controller that is not projected anywhere anymore, the reduction is cancelled.
	// In addition:
	// 	- when an object is scheduled for injection and the target group does not exist anymore (or is invalidated), the injection is cancelled.
	// 	- when an object is scheduled for propagation of sln changes and has no view anymore, the operation is cancelled.
	// Main processing in _Mem::update().
	class	r_exec_dll	_Mem:
	public	r_code::Mem{
	public:
		typedef	enum{
			NOT_STARTED=0,
			RUNNING=1,
			STOPPED=2
		}State;
	protected:
		// Parameters::Init.
		uint32	base_period;
		uint32	reduction_core_count;
		uint32	time_core_count;

		// Parameters::System.
		float32	mdl_inertia_sr_thr;
		uint32	mdl_inertia_cnt_thr;
		float32	tpx_dsr_thr;
		uint32	min_sim_time_horizon;
		uint32	max_sim_time_horizon;
		float32	sim_time_horizon;
		uint32	tpx_time_horizon;
		uint32	perf_sampling_period;
		float32	float_tolerance;
		uint32	time_tolerance;
		uint64	primary_thz;
		uint64	secondary_thz;

		// Parameters::Debug.
		bool	debug;
		uint32	ntf_mk_res;
		uint32	goal_pred_success_res;

		// Parameters::Run.
		uint32	probe_level;

		PipeNN<P<_ReductionJob>,1024>	*reduction_job_queue;
		PipeNN<P<TimeJob>,1024>			*time_job_queue;
		ReductionCore					**reduction_cores;
		TimeCore						**time_cores;
		// Use a deque so we can efficiently remove from the front.
		// Only used if reduction and time core count == 0.
		std::deque<P<TimeJob>> ordered_time_job_queue;
		TimeJob::Compare timeJobCompare_;

		// Performance stats.
		uint32	reduction_job_count;
		uint64	reduction_job_avg_latency;	// latency: popping time.-pushing time; the lower the better.
		uint64	_reduction_job_avg_latency;	// previous value.
		uint32	time_job_count;
		uint64	time_job_avg_latency;		// latency: deadline-the time the job is popped from the pipe; if <0, not registered (as it is too late for action); the higher the better.
		uint64	_time_job_avg_latency;		// previous value.

		CriticalSection	time_jobCS;
		CriticalSection	reduction_jobCS;

		uint32			core_count;
		CriticalSection	core_countCS;
		
		State			state;
		CriticalSection	stateCS;
		Semaphore		*stop_sem;	// blocks the rMem until all cores terminate.

		r_code::list<P<Code> >	objects;	// store objects in order of injection: holds the initial objects (and dynamically created ones if MemStatic is used).

		P<Group>	_root;	// holds everything.
		Code		*_stdin;
		Code		*_stdout;
		Code		*_self;

		void	reset();	// clear the content of the mem.

		std::vector<Group	*>	initial_groups;	// convenience; cleared after start();

		void	init_timings(uint64	now)	const;

		void	store(Code	*object);
		virtual	void	set_last_oid(int32	oid)=0;
		virtual	void	bind(View	*view)=0;

		bool	deleted;

		static	const	uint32	DebugStreamCount=8;
		ostream	*debug_streams[8];

		_Mem();

		void	_unpack_code(Code	*hlp,uint16	fact_object_index,Code	*fact_object,uint16	read_index)	const;
	public:
		static	_Mem	*Get(){	return	(_Mem	*)Mem::Get();	}

		typedef	enum{
			STDIN=0,
			STDOUT=1
		}STDGroupID;

		virtual	~_Mem();

		void	init(uint32	base_period,
					uint32	reduction_core_count,
					uint32	time_core_count,
					float32	mdl_inertia_sr_thr,
					uint32	mdl_inertia_cnt_thr,
					float32	tpx_dsr_thr,
					uint32	min_sim_time_horizon,
					uint32	max_sim_time_horizon,
					float32	sim_time_horizon,
					uint32	tpx_time_horizon,
					uint32	perf_sampling_period,
					float32	float_tolerance,
					uint32	time_tolerance,
					uint32	primary_thz,
					uint32	secondary_thz,
					bool	debug,
					uint32	ntf_mk_res,
					uint32	goal_pred_success_res,
					uint32	probe_level,
					uint32	traces);

		uint64	get_probe_level()						const{	return	probe_level;	}
		float32	get_mdl_inertia_sr_thr()				const{	return	mdl_inertia_sr_thr;	}
		uint32	get_mdl_inertia_cnt_thr()				const{	return	mdl_inertia_cnt_thr;	}
		float32	get_tpx_dsr_thr()						const{	return	tpx_dsr_thr;	}
		uint32	get_min_sim_time_horizon()				const{	return	min_sim_time_horizon;	}
		uint32	get_max_sim_time_horizon()				const{	return	max_sim_time_horizon;	}
		uint64	get_sim_time_horizon(uint64	horizon)	const{	return	horizon*sim_time_horizon;	}
		uint32	get_tpx_time_horizon()					const{	return	tpx_time_horizon;	}
		uint64	get_primary_thz()						const{	return	primary_thz;	}
		uint64	get_secondary_thz()						const{	return	secondary_thz;	}
		
		bool	get_debug()								const{	return	debug;	}
		uint32	get_ntf_mk_res()						const{	return	ntf_mk_res;	}
		uint32	get_goal_pred_success_res(Group	*host,uint64	now,uint64	time_to_live)	const{
			
			if(debug)
				return	goal_pred_success_res;
			if(time_to_live=0)
				return	1;
			return	Utils::GetResilience(now,time_to_live,host->get_upr());	
		}

		Code	*get_root()		const;
		Code	*get_stdin()	const;
		Code	*get_stdout()	const;
		Code	*get_self()		const;

		State	check_state();			// called by delegates after waiting in case stop() is called in the meantime.
		void	start_core();			// called upon creation of a delegate.
		void	shutdown_core();		// called upon completion of a delegate's task.

		virtual bool	load(std::vector<r_code::Code	*>	*objects,uint32	stdin_oid,uint32	stdout_oid,uint32	self_oid);	// call before start; no mod/set/eje will be executed (only inj);
																														// return false on error.
		uint64	start();	// return the starting time.
		/**
		 * When reduction and core count == 0, start() does not start
		 * any core threads, so call this instead of Thread::Sleep(runTimeMilliseconds) 
		 * to run in the current thread using "diagnostic time". As opposed to
		 * real time which uses Time::Get, diagnostic time uses getDiagnosticTimeNow()
		 * which simply returns DiagnosticTimeNow. (The main() function should call
		 * r_exec::Init where time_base is getDiagnosticTimeNow.) So, runInDiagnosticTime
		 * updates DiagnosticTimeNow based on the next time job (which always
		 * runs on time). This way, the return value of Now() does not move with real time, but moves
		 * step-by-step when DiagnosticTimeNow is updated, making it possible to set
		 * break points and diagnose the program.
		 * @param runTimeMilliseconds The number of milliseconds (in diagnostic time) to run for.
		 */
		void	runInDiagnosticTime(uint32 runTimeMilliseconds);
		static	uint64 DiagnosticTimeNow;
		static	uint64 getDiagnosticTimeNow();
		// Tell an inheriting class (with inject) when the time is changed.
		virtual void onDiagnosticTimeTick();
		void	stop();		// after stop() the content is cleared and one has to call load() and start() again.

		// Internal core processing	////////////////////////////////////////////////////////////////

		P<_ReductionJob>	popReductionJob(bool waitForItem = true);
		void			pushReductionJob(_ReductionJob	*j);
		P<TimeJob>		popTimeJob(bool waitForItem = true);
		void			pushTimeJob(TimeJob	*j);

		// Called upon successful reduction.
		void	inject(View	*view);
		void	inject_async(View	*view);
		void	inject_new_object(View	*view);
		void	inject_existing_object(View	*view,Code	*object,Group	*host);
		void	inject_null_program(Controller	*c,Group *group,uint64	time_to_live,bool	take_past_inputs);	// build a view v (ijt=now, act=1, sln=0, res according to time_to_live in the group), attach c to v, inject v in the group.
		void	inject_hlps(std::vector<View	*>	views,Group	*destination);
		void	inject_notification(View	*view,bool	lock);
		virtual	Code	*check_existence(Code	*object)=0;	// returns the existing object if any, or object otherwise: in the latter case, packing may occur.

		void	propagate_sln(Code	*object,float32	change,float32	source_sln_thr);

		// Called by groups.
		void	inject_copy(View	*view,Group	*destination);		// for cov; NB: no cov for groups, r-groups, models, pgm or notifications.

		// Called by cores.
		void	register_reduction_job_latency(uint64	latency);
		void	register_time_job_latency(uint64	latency);
		void	inject_perf_stats();

		// rMem to rMem.
		// The view must contain the destination group (either stdin or stdout) as its grp member.
		// To be redefined by object transport aware subcalsses.
		virtual	void	eject(View	*view,uint16	nodeID);

		// From rMem to I/O device.
		// To be redefined by object transport aware subcalsses.
		virtual	void	eject(Code	*command);

		virtual	r_code::Code	*_build_object(Atom	head)	const=0;
		virtual	r_code::Code	*build_object(Atom	head)	const=0;
		
		// unpacking of high-level patterns: upon loading or reception.
		void	unpack_hlp(Code	*hlp)	const;
		Code	*unpack_fact(Code	*hlp,uint16	fact_index)	const;
		Code	*unpack_fact_object(Code	*hlp,uint16	fact_object_index)	const;

		// packing of high-level patterns: upon dynamic generation or transmission.
		void	pack_hlp(Code	*hlp)	const;
		void	pack_fact(Code	*fact,Code	*hlp,uint16	&write_index,std::vector<P<Code>	>	*references)	const;
		void	pack_fact_object(Code	*fact_object,Code	*hlp,uint16	&write_index,std::vector<P<Code>	>	*references)	const;

		Code	*clone(Code	*original)	const;	// shallow copy.

		// External device I/O	////////////////////////////////////////////////////////////////
		virtual	r_comp::Image	*get_objects()=0;	// create an image; fill with all objects; call only when stopped.
		r_comp::Image	*get_models();				// create an image; fill with all models; call only when stopped.

		//std::vector<uint64>	timings_report;	// debug facility.
		typedef	enum{
			CST_IN=0,
			CST_OUT=1,
			MDL_IN=2,
			MDL_OUT=3,
			PRED_MON=4,
			GOAL_MON=5,
			MDL_REV=6,
			HLP_INJ=7
		}TraceLevel;
		static	std::ostream	&Output(TraceLevel	l);

		// Note: This should match the definition in user.classes.replicode.
		static const uint64 sampling_period_us = 100000;
	};


#define	OUTPUT(c)	_Mem::Output(_Mem::c)

	// _Mem that stores the objects as long as they are not invalidated.
	class	r_exec_dll	MemStatic:
	public	_Mem{
	private:
		CriticalSection	objectsCS;			// protects last_oid and objects.
		uint32	last_oid;
		void	bind(View	*view);			// assigns an oid, stores view->object in objects if needed.
		void	set_last_oid(int32	oid);
	protected:
		MemStatic();
	public:
		virtual	~MemStatic();

		void	delete_object(r_code::Code	*object);	// erase the object from objects if needed.

		r_comp::Image	*get_objects();		// return an image containing valid objects.
	};

	// _Mem that does not store objects.
	class	r_exec_dll	MemVolatile:
	public	_Mem{
	private:
		volatile	int32	last_oid;
		uint32	get_oid();
		void	bind(View	*view);			// assigns an oid (atomic operation).
		void	set_last_oid(int32	oid);
	protected:
		MemVolatile();
	public:
		virtual	~MemVolatile();

		void	delete_object(r_code::Code	*object){}

		r_comp::Image	*get_objects(){	return	NULL;	}
	};

	// O is the class of the objects held by the rMem (except groups and notifications):
	// 	r_exec::LObject if non distributed, or
	// 	RObject (see the integration project) when network-aware.
	// Notification objects and groups are instances of r_exec::LObject (they are not network-aware).
	// Objects are built at reduction time as r_exec:LObjects and packed into instances of O when O is network-aware.
	// S is the super-class.
	template<class	O,class	S>	class	Mem:
	public	S{
	public:
		Mem();
		virtual	~Mem();

		// Called at load time.
		r_code::Code	*build_object(r_code::SysObject	*source)	const;

		// Called at runtime.
		r_code::Code	*_build_object(Atom	head)	const;
		r_code::Code	*build_object(Atom	head)	const;

		// Executive device functions	////////////////////////////////////////////////////////
		
		Code	*check_existence(Code	*object);

		// Called by the communication device (I/O).
		void	inject(O	*object,View	*view);
	};

	/* DEPRECATED
	r_exec_dll r_exec::Mem<r_exec::LObject> *Run(const	char	*user_operator_library_path,
												uint64			(*time_base)(),
												const	char	*seed_path,
												const	char	*source_file_name);*/
}


#include	"mem.tpl.cpp"


#endif