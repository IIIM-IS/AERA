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
//_/_/ Copyright (c) 2012 Jan Koutnik
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

void CorrelatorCore::initializeOneStepPrediction(int nC, int nB, std::vector<std::vector <double> >& dataSequence) {
  nCells_ = nC;
  nBlocks_ = nB;
  nInputs_ = dataSequence[0].size();
  nOutputs_ = nInputs_;
  //dataSeqLength = dataSequence.size();
  buffersLength_ = dataSequence.size() - 1;

  inputSequenceBuffer_.assign(dataSequence.begin(), dataSequence.end() - 1);
  trainingSequenceBuffer_.assign(dataSequence.begin() + 1, dataSequence.end()); // shifted copy of the input sequence
  outputErrorBuffer_.assign(trainingSequenceBuffer_.begin(), trainingSequenceBuffer_.end());

  std::vector<LstmBlockState> lstmBlockState(buffersLength_);
  lstmStateBuffer_.assign(nBlocks_, lstmBlockState);
  if (nCells_ > 1) { // cells need to be added
    for (int i = 0; i < nBlocks_; i++) {
      for (int j = 0; j < lstmStateBuffer_.size(); j++) {
        for (int k = 0; k < nCells_ - 1; k++) {
          lstmStateBuffer_[i][j].addCell();
        }
      }
    }
  }

  ForwardLayerState outputState;
  outputState.ak.assign(nOutputs_, 0.);
  outputState.bk.assign(nOutputs_, 0.);
  outputState.ek.assign(nOutputs_, 0.);
  outputState.dk.assign(nOutputs_, 0.);

  outputLayerStateBuffer_.assign(buffersLength_, outputState); // initialize output layer state buffer

  std::vector<double> zeroVector(nInputs_, 0.);
  inputLayerErrorBuffer_.assign(buffersLength_, zeroVector);

  lstmNetwork_ = new LstmNetwork(nCells_, nInputs_, nOutputs_, nBlocks_);

}

void CorrelatorCore::appendBuffers(std::vector<std::vector <double> >& dataSequence) {
  buffersLength_ += dataSequence.size();
  inputSequenceBuffer_.push_back(trainingSequenceBuffer_.back()); // first element is taken from the back of the training buffer
  inputSequenceBuffer_.insert(inputSequenceBuffer_.end(), dataSequence.begin() + 1, dataSequence.end()); // append the rest of the data
  trainingSequenceBuffer_.insert(trainingSequenceBuffer_.end(), dataSequence.begin(), dataSequence.end()); // copy the whole new part to the training buffer
  outputErrorBuffer_.insert(outputErrorBuffer_.end(), dataSequence.begin(), dataSequence.end()); // does not matter, what's inside

  std::vector<LstmBlockState> lstmBlockState(dataSequence.size());
  if (nCells_ > 1) {
    for (int j = 0; j < lstmBlockState.size(); j++) {
      for (int k = 0; k < nCells_ - 1; k++) {
        lstmBlockState[j].addCell();
      }
    }
  } // it's easier to addCell just once and then copy the whole buffer nBlocks times.
  for (int i = 0; i < nBlocks_; i++) {
    lstmStateBuffer_[i].insert(lstmStateBuffer_[i].end(), lstmBlockState.begin(), lstmBlockState.end());
  }

  ForwardLayerState outputState;
  outputState.ak.assign(nOutputs_, 0.);
  outputState.bk.assign(nOutputs_, 0.);
  outputState.ek.assign(nOutputs_, 0.);
  outputState.dk.assign(nOutputs_, 0.);

  std::vector<double> zeroVector(nInputs_, 0.);

  for (int i = 0; i < dataSequence.size(); i++) {
    outputLayerStateBuffer_.push_back(outputState);
    inputLayerErrorBuffer_.push_back(zeroVector);
  }
}

void CorrelatorCore::forwardPass() {
  lstmNetwork_->forwardPass(0, buffersLength_, lstmStateBuffer_, outputLayerStateBuffer_,
    inputSequenceBuffer_);
}

void CorrelatorCore::backwardPass() {
  lstmNetwork_->backwardPass(0, buffersLength_, lstmStateBuffer_, outputLayerStateBuffer_,
    outputErrorBuffer_, inputLayerErrorBuffer_, inputSequenceBuffer_);
}

double CorrelatorCore::trainingEpoch(double learningRate, double momentum) {
  lstmNetwork_->resetDerivs();

  forwardPass();

  lstmNetwork_->outputLayer_->updateOutputError(outputLayerStateBuffer_, trainingSequenceBuffer_, outputErrorBuffer_);

  /*for(int i=0;i<outputErrorBuffer_.size();i++){
      for(int j=0;j<outputErrorBuffer_[0].size();j++){
          std::cout << outputErrorBuffer_[i][j] << " ";
      }std::cout << std::endl;
  }*/

  double error = 0.;
  for (int i = 0; i < outputErrorBuffer_.size(); i++) {
    error += mse(outputErrorBuffer_[i].begin(), outputErrorBuffer_[i].end(), 0.);
  }
  backwardPass();
  lstmNetwork_->updateWeights(learningRate /*/ trainingSequenceBuffer_.size()*/, momentum);
  return error / (outputErrorBuffer_.size() * outputErrorBuffer_[0].size());
}

void CorrelatorCore::snapshot(int t1, int t2) {
  std::vector<LstmBlockState> tmpState;

  inputSequenceBufferSnapshot_.assign(inputSequenceBuffer_.begin() + t1, inputSequenceBuffer_.begin() + t2);
  lstmStateBufferSnapshot_.clear();
  for (int i = 0; i < lstmStateBuffer_.size(); i++) { // all buffers for all lstm blocks
    tmpState.assign(lstmStateBuffer_[i].begin() + t1, lstmStateBuffer_[i].begin() + t2);
    lstmStateBufferSnapshot_.push_back(tmpState);
  }
  outputLayerStateBufferSnapshot_.assign(outputLayerStateBuffer_.begin() + t1, outputLayerStateBuffer_.begin() + t2);
  trainingSequenceBufferSnapshot_.assign(trainingSequenceBuffer_.begin() + t1, trainingSequenceBuffer_.begin() + t2);
  inputLayerErrorBufferSnapshot_.assign(inputLayerErrorBuffer_.begin() + t1, inputLayerErrorBuffer_.begin() + t2);
  outputErrorBufferSnapshot_.assign(outputErrorBuffer_.begin() + t1, outputErrorBuffer_.begin() + t2); // could be taken out, it is not useful in getJacobian
  // a copy of the lstm network is needed as well, because the weight derivatives get messed up with backpropagation (TODO)
  // input vector buffer is not needed, it's just used to update the weight derivatives

}

void CorrelatorCore::getJacobian(int t1, int t2, std::vector<std::vector <double> >& jacobian) {
  // the forward pass should be executed before taking snapshots

  snapshot(t1, t2); // fill the snapshot buffers with the copy

  int sLength = t2 - t1; // length of the snapshot
  std::vector<double> deltaVector;
  deltaVector.assign(trainingSequenceBuffer_[t2 - 1].begin(), trainingSequenceBuffer_[t2 - 1].end()); // use the training vector to propagate back as delta (it's expected to be sparse vector of 0s and 1s)
  std::vector<double> zeroVector(deltaVector.size(), 0.);
  outputErrorBufferSnapshot_.assign(sLength - 1, zeroVector); // t2-1
  outputErrorBufferSnapshot_.push_back(deltaVector); // add the delta vector to the end

  lstmNetwork_->backwardPass(0, sLength, lstmStateBufferSnapshot_, outputLayerStateBufferSnapshot_,
    outputErrorBufferSnapshot_, jacobian, inputSequenceBufferSnapshot_);
  abs_matrix(jacobian);
  rescale_matrix(jacobian); // scale into (0, 1)
}
void readMatrix(std::vector<std::vector <double> >& mat, int lineLength) {
  std::ifstream indata;
  double num;
  int cntr, lineCntr = 0;
  mat.clear();
  indata.open("/home/koutnij/d/shared_humanobs/ericdata/data.dat");
  if (!indata) {
    std::cout << "Error reading file." << "test" << std::endl;
    exit(1);
  }
  std::vector <double> line;
  do {
    cntr = 0;
    line.clear();
    do {
      indata >> num;
      line.push_back(num);
      cntr++;
    } while (!indata.eof() && cntr < lineLength);
    if (line.size() == lineLength /*&& lineCntr++ < 650*/) {
      std::cout << "adding : ";
      for (int i = 0; i < line.size(); i++) std::cout << line[i] << " ";
      mat.push_back(line);
      std::cout << std::endl;
    }
  } while (!indata.eof());
}



int main() {
  std::cout.precision(8);

  int nCells = 1, nBlocks = 8, nInputs = 32, nX = 4, nEpochs = 3; // for testing, the sequence has length of nInputs + 1 (for one step prediction)
  int nOutputs = nInputs;

  // let's have some data
  std::vector< std::vector<double> > data(nX, std::vector<double>(nInputs, 0.));
  for (int i = 0; i < nX; i++) {
    for (int j = 0; j < nInputs; j++) {
      if (j == i % (nX - 1)) data[i][j] = 1.; else data[i][j] = 0.;
    }
  }

  // load data from file
  data.clear();
  readMatrix(data, 32);

  // initialize the one step prediction experiment
  CorrelatorCore* ccore = new CorrelatorCore();
  ccore->initializeOneStepPrediction(nCells, nBlocks, data);

  //setup the weights to have some values (manually for testing)
  ccore->lstmNetwork_->setConstantWeights(.5);

  //or randomize them
  srand(time(0));
  ccore->lstmNetwork_->setRandomWeights(0.5);

  //ccore->lstmNetwork_->printSerializedWeights(6);

  for (int i = 0; i < 30; i++) {
    std::cout << i << " MSE = " << ccore->trainingEpoch(0.001, 0.001) << std::endl;
    // ccore->lstmNetwork_->printSerializedWeights(6);
  }
  //ccore->lstmNetwork_->printSerializedWeights(6);
  // copy the data there once more
  /*
  ccore->appendBuffers(data);

  for(int i=0;i<1000;i++){
      std::cout << i <<" MSE = "<< ccore->trainingEpoch(0.1,0.01) << std::endl;
      // ccore->lstmNetwork_->printSerializedWeights(6);
  }*/

  // sequential Jacobian
  int jacobianLength = 5;// should be a parameter
  ccore->forwardPass(); // do the forward pass first, to update the network state, should be changed to the network copy
  std::vector<double> zeroVector(nInputs, 0.);
  std::vector<std::vector<double> > jacobian(jacobianLength, zeroVector); // initialize the empty Jacobian matrix


  ccore->getJacobian(data.size() - 6, data.size() - 1, jacobian); // get the jacobian for the last vector

  for (int i = 0; i < jacobian.size(); i++) {
    for (int j = 0; j < jacobian[0].size(); j++) {
      std::cout << jacobian[i][j] << " ";
    }
    std::cout << std::endl;
  }


}
