#include "optimization/leastsquares.h"



	LeastSquaresOptimizer::LeastSquaresOptimizer(unsigned int window_length,double sampletime) :Optimizer(sampletime),
		window_length_(window_length), index_(window_length-1)
	{

		ticksPosVec_.setZero(window_length_ * 2);
		theta_pos_vec_.setZero(window_length_);

	}


	inline void LeastSquaresOptimizer::setSampletime(double sampletime)
	{
		sampletime_ = sampletime;
		init();
	}
	
	inline void LeastSquaresOptimizer::reset()
	{
		Optimizer::reset();
		ticksPosVec_.setZero(window_length_ * 2);
		theta_pos_vec_.setZero(window_length_);
	}
