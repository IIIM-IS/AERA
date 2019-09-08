//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ HUMANOBS - Replicode Correlator
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

#ifndef	correlator_h
#define	correlator_h

#include	"../types.h"
#include	"../../r_exec/overlay.h"

// switch to use WinEpi instead of LSTM for finding correlations
//#define USE_WINEPI -- jm


extern	"C"{
r_exec::Controller	dll_export	*correlator(r_code::View	*view);
}

////////////////////////////////////////////////////////////////////////////////

//#include	<iostream>
//#include	<fstream>
#include	<string>
#include	<map>
#include	<set>
#include	<algorithm>
#include	<iterator>
#include	<utility> // pair
#include	<math.h>
#include	<../../CoreLibrary/CoreLibrary/base.h>
#include	<../r_code/object.h>

#ifdef USE_WINEPI
#include	"winepi.h"
#else
#include	"CorrelatorCore.h"
#endif

class	State:
public	core::_Object{
public:
	float32 confidence;
	virtual	void	trace(std::ostream& out)=0;
};

//	Set of time-invariant objects.
//	reads as "if objects then states hold".
class	IPGMContext:
public	State{
public:
	std::vector<P<r_code::Code> >	objects;
	std::vector<P<State> >			states;

	void	trace(std::ostream& out){

		out<<"IPGMContext\n";
		out<<"Objects\n";
		for(uint32	i=0;i<objects.size();++i)
			objects[i]->trace(out);
		out<<"States\n";
		for(uint32	i=0;i<states.size();++i)
			states[i]->trace(out);
	}

	void	trace() { trace(std::cout); }
};

//	Pattern that hold under some context.
//	Read as "left implies right".
class	Pattern:
public	State{
public:
	std::vector<P<r_code::Code> >	left;
	std::vector<P<r_code::Code> >	right;

	void	trace(std::ostream& out){

		out<<"Pattern\n";
		out<<"Left\n";
		for(uint32	i=0;i<left.size();++i)
			left[i]->trace(out);
		out<<"Right\n";
		for(uint32	i=0;i<right.size();++i)
			right[i]->trace(out);
	}

	void	trace() { trace(std::cout); }
};

class	CorrelatorOutput{
public:
	std::vector<P<State> >	states; // changed from vector<P<IPGMContext>>

	void	trace(std::ostream& out){

		out<<"CorrelatorOutput: " << states.size() << " states" << std::endl;
		for(uint32	i=0;i<states.size();++i)
			states[i]->trace(out);
	}

	void	trace() { trace(std::cout); }
};


#ifdef USE_WINEPI

// uncommented --jm
typedef uint64 timestamp_t;
typedef P<r_code::Code> event_t;
typedef std::vector<std::pair<timestamp_t,event_t> > Episode;

class	Correlator{
private:
	Episode episode;
	size_t episode_start; // index of start of current episode
	WinEpi winepi;

public:
	Correlator();

	void take_input(r_code::View* input);
	CorrelatorOutput* get_output(bool useEntireHistory = false);

	void dump(std::ostream& out = std::cout, std::string (*oid2str)(uint32) = NULL) const;
};

#else // USE_WINEPI

// Lots of typedefs, not because I'm too lazy to type "std::" all the time,
// but because this makes it easier to change container types;
// also, it makes the code easier to follow
typedef std::vector<float64> LSTMState; // values of LSTM output nodes; length=32 always!
typedef std::vector<LSTMState> LSTMInput;
struct	Source{ // an antecedent of an event
	P<r_code::Code> source; // the Replicode source object
	uint16 deltaTime; // #timesteps this antecedent preceeds the event
};
typedef std::vector<Source>					Sources;
struct	JacobianRule{ // target <- [source_1,..,source_n]
	float64 confidence;
	P<r_code::Code> target; // begin of a container of (at least) 32 floating point number
	Sources sources; // a container of Source structs
};
typedef std::vector<JacobianRule>			JacobianRules;
typedef std::vector<LSTMState>				JacobianSlice; // N vectors of size 32
//typedef std::vector<JacobianSlice>			JacobianMatrix3D; // 3D Jacobian matrix
typedef std::list<P<r_code::Code> >			SmallWorld;
typedef std::vector<SmallWorld>				SmallWorlds;
typedef std::vector<P<State> >				Correlations;
typedef uint32								OID_t; // object identifier type
typedef uint32								enc_t; // binary encoding type
typedef std::vector<enc_t>					Episode;
typedef std::map<OID_t,enc_t>				Table_OID2Enc;
typedef std::map<enc_t,P<r_code::Code> >	Table_Enc2Obj;

class	Correlator{
private:
	Episode			episode; // chronological list of binary encodings of observed objects
	Table_Enc2Obj	enc2obj; // binary encoding => object (P<Code>)
	Table_OID2Enc	oid2enc; // object identifier => binary encoding
	uint32			episode_start; // index of start of current episode
	CorrelatorCore	corcor; // holds and maintains the learning core

	// finds a sparse binary encoding of the provided identifier
	// sets is_new to false iff an encoding already exists
	// currently, the encoding is generated randomly,
	// but in the future we may implement a better algorithm
	enc_t encode(OID_t id, bool& is_new);

	// extracts rules of the form Target <= {(Src_1,Dt_1),..,(Src_n,Dt_n)}
	// for deltaTimes Dt_1 < .. < Dt_n
	// Note: expensive function!
	void extract_rules(JacobianRules& rules, uint32 episode_size);

	// returns the object whose binary encoding best matches
	//   the contents of the container starting at "first"
	// in case of ties, returns an unspecified best match
	// returns NULL iff enc2obj is empty
	// NOTE: assumes there are (at least) 32 elements accessible from "first"
	template<class InputIterator>
	r_code::Code* findBestMatch(InputIterator first, float64& bestMatch) const;

public:
	Correlator();

	// stores the object pointed to by the provided View
	// runtime: O(log N) where N = size of episode so far
	void take_input(r_code::View* input);

	// trains the correlator on the episode so far and
	// returns the found correlations
	// useEntireHistory determines whether to use entire episode
	// or only from last call to get_output
	// Note: expensive function!
	CorrelatorOutput* get_output(bool useEntireHistory = false);

	// dump a listing of object IDs and their encodings to some output
	// the oid2str function can be used to provide a way of printing objects
	void dump(std::ostream& out = std::cout, std::string (*oid2str)(OID_t) = NULL) const;

	// magic numbers!!1
	static uint16	NUM_BLOCKS; // #hidden blocks in LSTM network
	static uint16	CELLS_PER_BLOCK; // #cells per block
	static uint32	NUM_EPOCHS; // max number of epochs to train
	static float64	TRAIN_TIME_SEC; // time-out for training in seconds
	static float64	MSE_THR; // time-out threshold for mean-squared error of training
	static float64	LEARNING_RATE;
	static float64	MOMENTUM;
	static uint32	SLICE_SIZE; // size of Jacobian matrix slices
	static float64	OBJECT_THR; // threshold for matching LSTM output to a Replicode object
	static float64	RULE_THR; // threshold for Jacobian rule confidence
};

#endif // USE_WINEPI

#endif