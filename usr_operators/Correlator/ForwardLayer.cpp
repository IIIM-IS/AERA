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

#include <numeric>
#include <iostream>
#include "ForwardLayer.h"
#include "VectorMath.h"

//#define DEBUG 1
//#define DEBUG_FWD 1

ForwardLayer::ForwardLayer(int nCells, int nOutputs, int nBlocks) {
	std::vector <double> tmp (nBlocks * nCells, 0.);
	wK.assign(nOutputs, tmp);
	wKd.assign(nOutputs, tmp);
	wKm.assign(nOutputs, tmp);

	biasK.assign(nOutputs, 0.);
	biasKd.assign(nOutputs, 0.);
	biasKm.assign(nOutputs, 0.);

}
void ForwardLayer::forwardPassStep(int t, std::vector<ForwardLayerState>& state, std::vector<double>& b){
	for(int i=0;i<(int)wK.size();i++){
		state[t].ak[i] = inner_product(wK[i].begin(), wK[i].end(), b.begin(), 0.) + biasK[i];
		state[t].bk[i] = fnO(state[t].ak[i]);
	}

	#ifdef DEBUG_FWD
		/*std::cout << "Debug [forward Layer, ak, t = "<< t <<" ]:" << std::endl;
		for(int i=0;i<state[t].ak.size();i++){
			std::cout << state[t].ak[i] << " ";
		} std::cout << std::endl;*/
		std::cout << "Debug [forward Layer, forward pass step, bk, t = "<< t <<" ]:" << std::endl;
		for(int i=0;i<state[t].bk.size();i++){
			std::cout << state[t].bk[i] << " ";
		} std::cout << std::endl;
	#endif
}
void ForwardLayer::backwardPassStep(int t, std::vector<ForwardLayerState>& state, std::vector<double>& b, std::vector<std::vector<double> >& er){
	// vector_difference(tr[t].begin(), er[t].end(), state[t].bk.begin(),state[t].ek.begin() );// output error

	for(int i=0;i<wK.size();i++)state[t].ek[i] = er[t][i];

	for(int i=0; i<(int)wK.size(); i++){
		state[t].dk[i] = fnOd(state[t].ak[i]) * state[t].ek[i];
		for(int j=0; j < (int) wK[0].size(); j++){
			wKd[i][j] += state[t].dk[i] * b[j];
		}
		// bias treatment
		biasKd[i] += state[t].dk[i];
	}

#ifdef DEBUG
	std::cout << "Debug [forward Layer, backward pass step, ek, t = "<< t <<" ]:" << std::endl;
	for(int i=0;i<state[t].ek.size();i++){
		std::cout << state[t].ek[i] << " ";
	} std::cout << std::endl;
	std::cout << "Debug [forward Layer, backward pass step, dk, t = "<< t <<" ]:" << std::endl;
	for(int i=0;i<state[t].dk.size();i++){
		std::cout << state[t].dk[i] << " ";
	} std::cout << std::endl;
	std::cout << "Debug [forward Layer, backward pass step, wKd, t = "<< t <<" ]:" << std::endl;
	for(int i=0;i<state[t].dk.size();i++){
		for(int j=0;j < (int) wK[0].size(); j++){
			std::cout << wKd[i][j] << " ";
		} std::cout << std::endl;
	} std::cout << std::endl;
#endif
}

void ForwardLayer::setConstantWeights(double w){
	std::vector<std::vector<double> > newwK (wK.size(), std::vector<double> (wK[0].size(), w));
	wK.assign(newwK.begin(), newwK.end());

	std::vector<double> newBiasK (biasK.size(), w);
	biasK.assign(newBiasK.begin(), newBiasK.end());

}

void ForwardLayer::setRandomWeights(double halfRange){
	for(int i=0;i<wK.size();i++){
		for(int j=0;j<wK[0].size();j++){
			wK[i][j] = randomW(halfRange);
		}
		biasK[i] = randomW(halfRange);
	}
}

void ForwardLayer::setwK(std::vector <std::vector<double> >& newwK){
	wK.assign(newwK.begin(), newwK.end());

	#ifdef DEBUG
		std::cout << "Debug [forward Layer, wK]:" << std::endl;
		for(int i=0;i<wK.size();i++){
			for(int j=0;j<wK[0].size();j++){
				std::cout << wK[i][j] << " ";
			} std::cout << std::endl;
		}
	#endif
}

void ForwardLayer::updateWeights(double eta, double alpha){
	for(int i=0;i<wK.size();i++){
		scalar_multiple(eta, wKd[i].begin(), wKd[i].end(), wKd[i].begin()); // d = eta * d
		scalar_multiple(alpha, wKm[i].begin(), wKm[i].end(), wKm[i].begin()); // m = alpha * m
		vector_add(wKm[i].begin(), wKm[i].end(), wKd[i].begin()); // m = m + d
		vector_add(wK[i].begin(), wK[i].end(), wKm[i].begin()); // w = w + m
	}

	// bias treatment
	scalar_multiple(eta, biasKd.begin(), biasKd.end() ,biasKd.begin());
	scalar_multiple(alpha, biasKm.begin(), biasKm.end() ,biasKm.begin());
	vector_add(biasKm.begin(), biasKm.end(), biasKd.begin());
	vector_add(biasK.begin(), biasK.end(), biasKm.begin());
}

void ForwardLayer::resetDerivs(){
	std::vector <double> tmp (wKd[0].size(), 0.);
	wKd.assign(wKd.size(), tmp);
	biasKd.assign(biasK.size(), 0.);
}

void ForwardLayer::printWeights(){
	std::cout << "Debug [forward Layer, wK ]:" << std::endl;
	for(int i=0;i<wK.size();i++){
		for(int j=0;j < (int) wK[0].size(); j++){
			std::cout << wK[i][j] << " ";
		} std::cout << std::endl;
	} std::cout << std::endl;
	std::cout << "Debug [forward Layer, wKd ]:" << std::endl;
	for(int i=0;i<wKd.size();i++){
		for(int j=0;j < (int) wKd[0].size(); j++){
			std::cout << wKd[i][j] << " ";
		} std::cout << std::endl;
	} std::cout << std::endl;
	std::cout << "Debug [forward Layer, wKm ]:" << std::endl;
	for(int i=0;i<wKm.size();i++){
		for(int j=0;j < (int) wKm[0].size(); j++){
			std::cout << wKm[i][j] << " ";
		} std::cout << std::endl;
	} std::cout << std::endl;
}

void ForwardLayer::getSerializedWeights(std::vector<double> & w){
	for(int i=0;i<wK.size();i++){
		for(int j=0;j < (int) wK[0].size(); j++){
			w.push_back(wK[i][j]);
		}
	}
}

void ForwardLayer::getSerializedDerivs(std::vector<double> & w){
	for(int i=0;i<wKd.size();i++){
		for(int j=0;j < (int) wKd[0].size(); j++){
			w.push_back(wKd[i][j]);
		}
	}
}

void ForwardLayer::updateOutputError(std::vector<ForwardLayerState>& state, std::vector<std::vector<double> >& tr,std::vector<std::vector<double> >& er){
	for(int i=0;i<tr.size();i++){
		//vector_difference(state[i].bk.begin(), state[i].bk.end(), tr[i].begin(), er[i].begin());
		vector_difference(tr[i].begin(), tr[i].end(), state[i].bk.begin(), er[i].begin());
	}
}

ForwardLayer::~ForwardLayer() {

}
