//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ HUMANOBS - Replicode Correlator
//_/_/
//_/_/ Jan Koutnik
//_/_/ Istituto Dalle Molle di Studi sull'Intelligenza Artificiale
//_/_/   University of Lugano and SUPSI, Switzerland
//_/_/   http://www.idsia.ch/
//_/_/ Copyright(c)2012
//_/_/
//_/_/ This software was developed by the above copyright holder as part of 
//_/_/ the HUMANOBS EU research project, in collaboration with the 
//_/_/ following parties:
//_/_/ 
//_/_/ Center for Analysis and Design of Intelligent Agents
//_/_/   Reykjavik University, Menntavegur 1, 101 Reykjavik, Iceland
//_/_/   http://cadia.ru.is
//_/_/
//_/_/ Autonomous Systems Laboratory
//_/_/   Technical University of Madrid, Spain
//_/_/   http://www.aslab.org/
//_/_/
//_/_/ Communicative Machines
//_/_/   Edinburgh, United Kingdom
//_/_/   http://www.cmlabs.com/
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

#ifndef	winepi_h
#define	winepi_h

// do not uncomment this switch; implementation incomplete!
//#define WINEPI_SERIAL

#include <map>
#include <set>
#include <list>
#include <vector>
#include <algorithm>
#include <string>
#include <sstream>
#include <utility> // pair
#include <fstream>
#include <iostream> // cout
#include <ctime> // clock_t

#include	<../../CoreLibrary/trunk/CoreLibrary/base.h>
#include	<../r_code/object.h>

//#include "correlator.h"
typedef uint64 timestamp_t;
typedef P<r_code::Code> event_t;

// Added to handle operator overloading and map references 
inline bool operator<(event_t lhs, event_t rhs) { return (r_code::Code*)lhs < (r_code::Code*)rhs; }
inline bool operator>(event_t lhs, event_t rhs) { return lhs < rhs; }


struct event_less
{
	bool operator()(const event_t lhs, const event_t rhs)  const { return lhs < rhs; }
};

struct event_pair
{
	bool operator()(std::pair<event_t, size_t> lhs, std::pair<event_t, size_t> rhs) const { return lhs.first < rhs.first; }
};

struct event_compare
{
	bool operator() (const event_t e1, const event_t e2) const
	{
		return e1->get_oid() < e2->get_oid();
	}
};


struct Candidate {
	std::map<int,event_t> G; // mapping from [1..] to event_types
#ifdef WINEPI_SERIAL
	std::multimap<int,int> R; // list of tuples of type (int,int)
#endif
	std::map<event_t,size_t,event_less> type_count;
	int block_start;
	int event_count;
	int freq_count;
	timestamp_t inwindow;


	Candidate() {
		init();
	}

	// -- jm made parameter const
	Candidate(const event_t& A) {
		init();
		set(1, A);
	}

	void init() {
		block_start = 0;
		event_count = 0;
		freq_count = -1;
		inwindow = 0;
	}

	Candidate& operator=(const Candidate& e) {
		G.clear();
		G.insert(e.G.begin(), e.G.end());
#ifdef WINEPI_SERIAL
		R.clear();
		R.insert(e.R.begin(), e.R.end());
#endif
		type_count.clear();
		type_count.insert(e.type_count.begin(), e.type_count.end());
		block_start = e.block_start;
		event_count = e.event_count;
		freq_count = e.freq_count;
		inwindow = e.inwindow;
		return *this;
	}

	bool operator==(const Candidate& e) const {
		return G == e.G;
	}

	bool operator<(const Candidate& e) const {
		std::map<int,event_t>::const_iterator it = G.begin(), it2 = e.G.begin();
		//jm added explicit comparison from correlator.cpp -presumably where this was 
		// being handled by default in previous c++ versions.
		for(; it != G.end() && it2 != e.G.end(); ++it, ++it2)
			if(it->second->get_oid() < it2->second->get_oid())
				return true;
			else if(it->second->get_oid() > it2->second->get_oid())
				return false;
		return (it2 != e.G.end());
	}

	size_t size() const {
		return G.size();
	}

	event_t& get(int i) {
		return G.find(i)->second;
	}


	void set(int i, const event_t& x) {
//			if(G.find(i) != G.end())
//				--type_count[G[i]];
		++type_count[x];
		G[i] = x;
	}
/*
	bool order(int x, int y) {
		std::pair<std::multimap<int,int>::iterator,std::multimap<int,int>::iterator> its = R.equal_range(x);
		for(std::multimap<int,int>::iterator it = its.first; it != its.second; ++it)
			if(it->second == y)
				return true;
		return false;
	}
*/
	std::string toString() const {
		std::stringstream ss;
		ss << "<";
		bool comma = false;
		for(std::map<int,event_t>::const_iterator it = G.begin(); it != G.end(); ++it) {
			if(comma)
				ss << ", ";
			else
				comma = true;
			ss << it->second->get_oid() /*<< it->first*/;
		}
#ifdef WINEPI_SERIAL
		for(std::multimap<int,int>::const_iterator it = R.begin(); it != R.end(); ++it)
			ss << ", " << G.find(it->first)->second /*<< it->first*/ << "->" << G.find(it->second)->second /*<< it->second*/;
#endif
		ss << ">";
		return ss.str();
	}
};

struct Rule {
	Candidate lhs;
	Candidate rhs;
	double conf;

	Rule(const Candidate& lhs_, const Candidate& rhs_, double conf_) : lhs(lhs_), rhs(rhs_), conf(conf_) {}

	Rule& operator=(const Rule& r) {
		lhs = r.lhs;
		rhs = r.rhs;
		conf = r.conf;
		return *this;
	}

	std::string toString() {
		std::stringstream ss;
		ss << lhs.toString() << " implies " << rhs.toString() << " with conf " << conf;
		return ss.str();
	}
};

// a sequence is a tuple (s, starttime, endtime) where s is a list of tuples (timestamp,event)
// a candidate is a tuple (G,R,block_start)
//   where G is a dict: int -> event_types
//         R is a list of (int,int) tuples
//         block_start is an index

struct Sequence {
	std::multimap<timestamp_t,event_t> seq;
	timestamp_t start;
	timestamp_t end;

	Sequence() {}

	Sequence(std::multimap<timestamp_t,event_t>& seq_)
		: seq(seq_.begin(), seq_.end())
		, start(seq.begin()->first)
		, end(seq.rbegin()->first + 1)
	{}

	template<class InputIterator>
	void init(InputIterator it, InputIterator last) {
		seq.clear();
		seq.insert(it, last);
		start = seq.begin()->first;
		end = seq.rbegin()->first + 1;
	}

	void addEvent(timestamp_t t, event_t ev) {
		seq.insert(std::make_pair(t, ev));  //jm make_pair conversion
	}

	std::string toString() {
		std::stringstream ss;
		ss << "<";
		bool comma = false;
		for(std::multimap<timestamp_t,event_t,event_compare>::iterator it = seq.begin(); it != seq.end(); ++it) {
			if(comma)
				ss << ", ";
			else
				comma = true;
			ss << it->first << ":" << it->second->get_oid();
		}
		ss << "}, s:" << start << ", e:" << end << ">";
		return ss.str();
	}
};




class WinEpi {
public:

	Sequence seq;
	int win;
	double min_fr;
	double min_conf;
	int max_size;

	std::set<event_t,event_compare> event_types;

	WinEpi(/*std::multimap<timestamp_t,event_t>& seq, int win, double min_fr, double min_conf, int max_size = -1*/);

	// initialize the sequence; iterator must return pairs
//	template<class InputIterator> void setSeq(InputIterator it, InputIterator end);
	template<class InputIterator>
	void setSeq(InputIterator it, InputIterator end) {
		seq.init(it, end);
		for(; it != end; ++it) {
			event_types.insert(it->second);
		}
	}

	void setParams(int win, double min_fr, double min_conf, int max_size = -1);

	void algorithm_1(std::vector<Rule>& out);
	void algorithm_2(std::vector<std::vector<Candidate> >& F);
	void algorithm_3(std::vector<Candidate>& F, int el, std::vector<Candidate>& C);
	void algorithm_4(std::vector<Candidate>& C, double min_fr, std::vector<Candidate>& F);
	double fr(Candidate& a);

	template<class K, class V>
	static void subsets(std::map<K,V>& in, std::vector<std::map<K,V> >& out) {
		size_t nIn = in.size();
		size_t nOut = 1 << nIn;
		out.resize(nOut);
		std::map<K,V>::iterator it = in.begin();
		for(size_t i = 0; i < nIn; ++i, ++it) {
			size_t chunk_size = 1 << (nIn - i - 1);
			for(size_t chunk_start = 0; chunk_start < nOut; chunk_start += 2 * chunk_size)
				for(size_t j = chunk_start; j < chunk_start + chunk_size; ++j)
					out[j].insert(*it);
		}
	}

#ifdef WINEPI_SERIAL
	static void restrict(std::multimap<int,int>& in, std::map<int,event_t>& seq, std::multimap<int,int>& out) {
		for(std::multimap<int,int>::iterator it = in.begin(); it != in.end(); ++it)
			if(seq.find(it->first) != seq.end() && seq.find(it->second) != seq.end())
				out.insert(*it);
	}
#endif

	static void strict_subcandidates(Candidate& a, std::vector<Candidate>& out) {

		if(a.size() < 2)
			return;

		out.resize((1 << a.size()) - 2);

		std::vector<std::map<int,event_t> > subseqv(1 << a.size());
		subsets(a.G, subseqv);
		std::vector<std::map<int,event_t> >::iterator it = subseqv.begin();
		std::vector<std::map<int,event_t> >::iterator end = subseqv.end();
		size_t i = 0;
		for(++it, --end; it != end; ++it, ++i) {
#ifdef WINEPI_SERIAL
			std::multimap<int,int> R;
			restrict(a.R, *it, R);
			out[i].R.insert(R.begin(), R.end());
#endif
			for(std::map<int,event_t>::iterator it2 = it->begin(); it2 != it->end(); ++it2)
				out[i].set(it2->first, it2->second);
		}
	}
};

#endif
