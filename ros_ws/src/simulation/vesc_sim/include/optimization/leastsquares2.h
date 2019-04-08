#pragma once

#include "optimization/leastsquares.h"

namespace tudora_optimization
{
	class LeastSquares2 : public LeastSquaresOptimizer
	{
	public:
		LeastSquares2(unsigned int window_length, double sampletime);
		virtual ~LeastSquares2() = default;
		virtual double optimize(double value);
	protected:
		virtual void init() override;
		double time_end_;
		double time_end2_;
	};
}