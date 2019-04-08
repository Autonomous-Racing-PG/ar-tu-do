#pragma once

#include "optimization/leastsquares.h"

namespace tudora_optimization
{
	class LeastSquares3 : public LeastSquaresOptimizer
	{
	public:
		LeastSquares3(unsigned int window_length, double sampletime);
		virtual ~LeastSquares3() = default;
		virtual double optimize(double value);
	protected:
		virtual void init() override;

		double time_end_;
		double time_end2_;
		double time_end3_;
	};
}