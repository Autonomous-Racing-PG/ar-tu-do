#pragma once

#include "optimization/leastsquares.h"



	class LeastSquares1 : public LeastSquaresOptimizer
	{
	public:
		LeastSquares1(unsigned int window_length, double sampletime);
		virtual ~LeastSquares1() = default;
		virtual double optimize(double value);
	protected:
		virtual void init() override;
		double time_end_;
	};
