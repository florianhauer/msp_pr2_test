/*
 * State.h
 *
 *  Created on: Feb 12, 2015
 *      Author: florian
 */

#ifndef STATE_H_
#define STATE_H_

#include "Params.h"
#include <array>
#include <numeric>
#include <algorithm>
#include <iterator>
#include <iostream>     // std::cout

template <unsigned int DIM> class State{
public:
	typedef std::array<float,DIM> VectorND;

private:
	VectorND vec_; //n-dim vector

public:
	State():vec_(){}
	State(float n):vec_(){std::fill(vec_.begin(),vec_.end(),n);}
	State(VectorND vec):vec_(vec){}
	State(const State& s):vec_(s.vec_){}

	//[] operator to access the array elements
	inline float& operator [] (int i){
		return vec_[i];
	}

	//difference operation
	inline State operator - ( const State& b) const{
		State c;
		std::transform(vec_.begin(), vec_.end(), b.vec_.begin(), c.vec_.begin(),
				[](float a, float b){return a - b;});
		return c;
	}

	//addition operation
	inline State operator + ( const State& b) const{
		State c;
		std::transform(vec_.begin(), vec_.end(), b.vec_.begin(), c.vec_.begin(),
				[](float a, float b){return a + b;});
		return c;
	}

	//multiplication by a scalar
	inline State operator * ( const float s) const{
		//std::cout << "mult " << *this << " by " << s << std::endl;
		State c;
		std::transform(vec_.begin(), vec_.end(), c.vec_.begin(),
				[s](float a)->float {return s*a;});
		return c;
	}

	//squared norm of the vector
	inline float normSq () const{
		return std::inner_product(vec_.begin(), vec_.end(), vec_.begin(), 0.0f);
	}

	//norm of the vector
	inline float norm () const{
		return sqrt(normSq());
	}

	//abs of each component of the vector
	inline State abs () const{
		State s=State(*this);
		std::for_each(s.vec_.begin(),s.vec_.end(),[](float &a){a=fabs(a);});
		return s;
	}

	//1-norm of the vector
	inline float normInf() const{
		State s=abs();
		return *std::max_element(s.vec_.begin(),s.vec_.end());
	}

	//Sort the components of the vector in ascending order
	inline void sort(){
		std::sort(vec_.begin(),vec_.end());
	}

	//Sort the components of the vector in ascending order
	inline double min(){
		return *std::min_element(vec_.begin(),vec_.end());
	}

	//Sort the components of the vector in ascending order
	inline double max(){
		return *std::max_element(vec_.begin(),vec_.end());
	}

	// check if all components are smaller than the components of bound
	inline bool isWithin(const State bound) const{
		State test=*this-bound;
		return std::all_of(test.vec_.begin(),test.vec_.end(),[](float a){return a<0;});
	}

	// < operator (full ordering)
	inline bool operator < (const State b) const{
		return vec_ < b.vec_;
	}

	//print operator
	friend std::ostream& operator<< (std::ostream& stream, const State& st) {
		stream << "(";
		for(int i=0;i<DIM-1;++i)
			stream << ((int)16*st.vec_[i]) << ", ";
		stream << ((int)16*st.vec_[DIM-1]) << ")";
		return stream;
	}
};

#endif /* STATE_H_ */
