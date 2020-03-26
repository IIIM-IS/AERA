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

#ifndef VECTORMATH_H_
#define VECTORMATH_H_

#include <cstdlib>

// outer product
template<class InputIterator1, class InputIterator2, class OutputIterator>
static OutputIterator
outer_product(InputIterator1 first1, InputIterator1 last1,
  InputIterator2 first2, InputIterator2 last2,
  OutputIterator result) {

  for (; first1 != last1; ++first1)
    for (InputIterator2 it2 = first2; it2 != last2; ++it2)
      *result++ = (*first1) * (*it2);
  return result;
}

// vector difference
template <class InputIterator1, class InputIterator2, class OutputIterator>
static OutputIterator vector_difference(InputIterator1 first1, InputIterator1 last1,
  InputIterator2 first2, OutputIterator result) {
  while (first1 != last1) {
    *result++ = (*first1++) - (*first2++);
  }
  return result;
}

// vector sum
template <class InputIterator1, class InputIterator2, class OutputIterator>
static OutputIterator vector_sum(InputIterator1 first1, InputIterator1 last1,
  InputIterator2 first2, OutputIterator result) {
  while (first1 != last1) {
    *result++ = (*first1++) + (*first2++);
  }
  return result;
}

// vector add
template <class InputIterator1, class InputIterator2>
static InputIterator2 vector_add(InputIterator1 first1, InputIterator1 last1,
  InputIterator2 first2) {
  while (first1 != last1) {
    *first1++ += (*first2++);
  }
  return first1;
}

// vector subtract
template <class InputIterator1, class InputIterator2>
static InputIterator2 vector_sub(InputIterator1 first1, InputIterator1 last1,
  InputIterator2 first2) {
  while (first1 != last1) {
    *first1++ -= (*first2++);
  }
  return first1;
}

// scalar multiple
template <class T, class InputIterator, class OutputIterator>
static OutputIterator scalar_multiple(T k, InputIterator first1, InputIterator last1,
  OutputIterator first2) {
  while (first1 != last1) {
    *first2++ = k * (*first1++);
  }
  return first2;
}

// total 1D
template <class InputIterator, class T>
static T total(InputIterator first1, InputIterator last1, T t) {
  //T t = 0;
  while (first1 != last1) {
    t += (*first1++);
  }
  return t;
}

// mean square error
template <class InputIterator, class T>
static T mse(InputIterator first1, InputIterator last1, T t) {
  //T t = 0;
  while (first1 != last1) {
    t += (*first1)*(*first1++);
  }
  return t / 2.;
}

static double randomW(double halfRange) {
  return halfRange * (2 * (rand() / (double)RAND_MAX) - 1);
}

static void rescale_matrix(std::vector<std::vector <double> >& mat) {
  double min = mat[0][0];
  double max = mat[0][0];
  double d;

  for (int i = 0; i < mat.size(); i++) {
    for (int j = 0; j < mat[0].size(); j++) {
      if (mat[i][j] < min) min = mat[i][j];
      if (mat[i][j] > max) max = mat[i][j];
    }
  }
  if (min != max) {
    d = max - min;
    for (int i = 0; i < mat.size(); i++) {
      for (int j = 0; j < mat[0].size(); j++) {
        mat[i][j] = (mat[i][j] - min) / d;
      }
    }
  }

}

static void abs_matrix(std::vector<std::vector <double> >& mat) {
  for (int i = 0; i < mat.size(); i++) {
    for (int j = 0; j < mat[0].size(); j++) {
      mat[i][j] = fabs(mat[i][j]);

    }

  }
}

#endif /* VECTORMATH_H_ */
