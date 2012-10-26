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

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <numeric>
#include <ctime>

#include "CorrelatorCore.h"
#include "LstmLayer.h"
#include "ForwardLayer.h"
#include "LstmNetwork.h"
#include "VectorMath.h"


CorrelatorCore::CorrelatorCore() {

}

CorrelatorCore::~CorrelatorCore() {

}

void CorrelatorCore::initializeOneStepPrediction(int nC, int nB, std::vector<std::vector <double> >& dataSequence){
	nCells = nC;
	nBlocks = nB;
	nInputs = dataSequence[0].size();
	nOutputs = nInputs;
	//dataSeqLength = dataSequence.size();
	buffersLength = dataSequence.size()-1;

	inputSequenceBuffer.assign(dataSequence.begin(), dataSequence.end() - 1);
	trainingSequenceBuffer.assign(dataSequence.begin() + 1, dataSequence.end()); // shifted copy of the input sequence
	outputErrorBuffer.assign(trainingSequenceBuffer.begin(), trainingSequenceBuffer.end());

	std::vector<LstmBlockState> lstmBlockState(buffersLength);
	lstmStateBuffer.assign(nBlocks, lstmBlockState);
	if(nCells > 1){ // cells need to be added
		for(int i=0;i<nBlocks;i++){
			for(int j=0;j<lstmStateBuffer.size();j++){
				for(int k=0;k<nCells-1;k++){
					lstmStateBuffer[i][j].addCell();
				}
			}
		}
	}

	ForwardLayerState outputState;
	outputState.ak.assign(nOutputs, 0.);
	outputState.bk.assign(nOutputs, 0.);
	outputState.ek.assign(nOutputs, 0.);
	outputState.dk.assign(nOutputs, 0.);

	outputLayerStateBuffer.assign(buffersLength, outputState); // initialize output layer state buffer

	std::vector<double> zeroVector(nInputs, 0.);
	inputLayerErrorBuffer.assign(buffersLength, zeroVector);

	lstmNetwork = new LstmNetwork(nCells,nInputs,nOutputs,nBlocks);

}

void CorrelatorCore::appendBuffers(std::vector<std::vector <double> >& dataSequence){
	buffersLength += dataSequence.size();
	inputSequenceBuffer.push_back(trainingSequenceBuffer.back()); // first element is taken from the back of the training buffer
	inputSequenceBuffer.insert(inputSequenceBuffer.end(), dataSequence.begin() + 1, dataSequence.end()); // append the rest of the data
	trainingSequenceBuffer.insert(trainingSequenceBuffer.end(), dataSequence.begin(), dataSequence.end()); // copy the whole new part to the training buffer
	outputErrorBuffer.insert(outputErrorBuffer.end(), dataSequence.begin(), dataSequence.end()); // does not matter, what's inside

	std::vector<LstmBlockState> lstmBlockState(dataSequence.size());
	if(nCells > 1){
		for(int j=0;j<lstmBlockState.size();j++){
			for(int k=0;k<nCells-1;k++){
				lstmBlockState[j].addCell();
			}
		}
	}	// it's easier to addCell just once and then copy the whole buffer nBlocks times.
	for(int i=0;i<nBlocks;i++){
		lstmStateBuffer[i].insert(lstmStateBuffer[i].end(), lstmBlockState.begin(), lstmBlockState.end());
	}

	ForwardLayerState outputState;
	outputState.ak.assign(nOutputs, 0.);
	outputState.bk.assign(nOutputs, 0.);
	outputState.ek.assign(nOutputs, 0.);
	outputState.dk.assign(nOutputs, 0.);

	std::vector<double> zeroVector(nInputs, 0.);

	for(int i=0;i<dataSequence.size();i++){
		outputLayerStateBuffer.push_back(outputState);
		inputLayerErrorBuffer.push_back(zeroVector);
	}
}

void CorrelatorCore::forwardPass(){
	lstmNetwork->forwardPass(0, buffersLength, lstmStateBuffer, outputLayerStateBuffer,
							 inputSequenceBuffer);
}

void CorrelatorCore::backwardPass(){
	lstmNetwork->backwardPass(0, buffersLength, lstmStateBuffer, outputLayerStateBuffer,
							  outputErrorBuffer, inputLayerErrorBuffer, inputSequenceBuffer);
}

double CorrelatorCore::trainingEpoch(double learningRate, double momentum){
	lstmNetwork->resetDerivs();

	forwardPass();

	lstmNetwork->outputLayer->updateOutputError(outputLayerStateBuffer, trainingSequenceBuffer, outputErrorBuffer);

	/*for(int i=0;i<outputErrorBuffer.size();i++){
		for(int j=0;j<outputErrorBuffer[0].size();j++){
			std::cout << outputErrorBuffer[i][j] << " ";
		}std::cout << std::endl;
	}*/

	double error = 0.;
	for(int i=0;i<outputErrorBuffer.size();i++){
		error += mse(outputErrorBuffer[i].begin(), outputErrorBuffer[i].end(), 0.);
	}
	backwardPass();
	lstmNetwork->updateWeights(learningRate /*/ trainingSequenceBuffer.size()*/, momentum);
	return error/(outputErrorBuffer.size() * outputErrorBuffer[0].size());
}

void CorrelatorCore::snapshot(int t1, int t2){
	std::vector<LstmBlockState> tmpState;

	inputSequenceBufferSnapshot.assign(this->inputSequenceBuffer.begin()+t1,this->inputSequenceBuffer.begin()+t2);
	lstmStateBufferSnapshot.clear();
	for(int i=0;i<lstmStateBuffer.size();i++){ // all buffers for all lstm blocks
		tmpState.assign(this->lstmStateBuffer[i].begin()+t1,this->lstmStateBuffer[i].begin()+t2);
		lstmStateBufferSnapshot.push_back(tmpState);
	}
	outputLayerStateBufferSnapshot.assign(this->outputLayerStateBuffer.begin()+t1,this->outputLayerStateBuffer.begin()+t2);
	trainingSequenceBufferSnapshot.assign(this->trainingSequenceBuffer.begin()+t1,this->trainingSequenceBuffer.begin()+t2);
	inputLayerErrorBufferSnapshot.assign(this->inputLayerErrorBuffer.begin()+t1,this->inputLayerErrorBuffer.begin()+t2);
	outputErrorBufferSnapshot.assign(this->outputErrorBuffer.begin()+t1,this->outputErrorBuffer.begin()+t2); // could be taken out, it is not useful in getJacobian
	// a copy of the lstm network is needed as well, because the weight derivatives get messed up with backpropagation (TODO)
	// input vector buffer is not needed, it's just used to update the weight derivatives

}

void CorrelatorCore::getJacobian(int t1, int t2, std::vector<std::vector <double> >& jacobian){
	// the forward pass should be executed before taking snapshots

	snapshot(t1, t2); // fill the snapshot buffers with the copy

	int sLength = t2 - t1; // length of the snapshot
	std::vector<double> deltaVector;
	deltaVector.assign(trainingSequenceBuffer[t2-1].begin(),trainingSequenceBuffer[t2-1].end()); // use the training vector to propagate back as delta (it's expected to be sparse vector of 0s and 1s)
	std::vector<double> zeroVector(deltaVector.size(), 0.);
	outputErrorBufferSnapshot.assign(sLength - 1, zeroVector); // t2-1
	outputErrorBufferSnapshot.push_back(deltaVector); // add the delta vector to the end

	lstmNetwork->backwardPass(0, sLength, lstmStateBufferSnapshot, outputLayerStateBufferSnapshot,
							  outputErrorBufferSnapshot, jacobian, inputSequenceBufferSnapshot);
	abs_matrix(jacobian);
	rescale_matrix(jacobian); // scale into (0, 1)
}
void readMatrix(std::vector<std::vector <double> >& mat, int lineLength){
   std::ifstream indata;
   double num;
   int cntr,lineCntr = 0;
   mat.clear();
   indata.open("/home/koutnij/d/shared_humanobs/ericdata/data.dat");
   if(!indata){
	   std::cout << "Error reading file." << "test" << std::endl;
	   exit(1);
   }
   std::vector <double> line;
   do{
	   cntr = 0;
	   line.clear();
	   do{
		   indata >> num;
		   line.push_back(num);
		   cntr++;
	   }while(!indata.eof() && cntr < lineLength);
	   if(line.size() == lineLength /*&& lineCntr++ < 650*/) {
		   std::cout << "adding : ";
		   for(int i=0;i<line.size();i++) std::cout << line[i] <<" ";
		   mat.push_back(line);
		   std::cout << std::endl;
	   }
   }while(!indata.eof());
}



int main(){
	std::cout.precision(8);

	int nCells=1, nBlocks=8, nInputs=32, nX=4, nEpochs=3; // for testing, the sequence has length of nInputs + 1 (for one step prediction)
	int nOutputs = nInputs;

	// let's have some data
	std::vector< std::vector<double> > data(nX, std::vector<double>(nInputs, 0.) );
	for(int i=0;i<nX;i++){
		for(int j=0;j<nInputs;j++){
			if(j == i%(nX-1)) data[i][j] = 1.; else data[i][j] = 0.;
		}
	}

	// load data from file
	data.clear();
	readMatrix(data, 32);

	// initialize the one step prediction experiment
	CorrelatorCore* ccore = new CorrelatorCore();
	ccore->initializeOneStepPrediction(nCells, nBlocks, data);

	//setup the weights to have some values (manually for testing)
	ccore->lstmNetwork->setConstantWeights(.5);

	//or randomize them
	srand(time(0));
	ccore->lstmNetwork->setRandomWeights(0.5);

	//ccore->lstmNetwork->printSerializedWeights(6);

	for(int i=0;i<30;i++){
		std::cout << i <<" MSE = "<< ccore->trainingEpoch(0.001,0.001) << std::endl;
		// ccore->lstmNetwork->printSerializedWeights(6);
	}
	//ccore->lstmNetwork->printSerializedWeights(6);
	// copy the data there once more
	/*
	ccore->appendBuffers(data);

	for(int i=0;i<1000;i++){
		std::cout << i <<" MSE = "<< ccore->trainingEpoch(0.1,0.01) << std::endl;
		// ccore->lstmNetwork->printSerializedWeights(6);
	}*/

	// sequential Jacobian
	int jacobianLength = 5;// should be a parameter
	ccore->forwardPass(); // do the forward pass first, to update the network state, should be changed to the network copy
	std::vector<double> zeroVector(nInputs, 0.);
	std::vector<std::vector<double> > jacobian(jacobianLength, zeroVector); // initialize the empty Jacobian matrix


	ccore->getJacobian(data.size()- 6, data.size()- 1, jacobian); // get the jacobian for the last vector

	for(int i=0;i<jacobian.size();i++){
		for(int j=0;j<jacobian[0].size();j++){
			std::cout << jacobian[i][j] << " ";
		}
		std::cout << std::endl;
	}


}
