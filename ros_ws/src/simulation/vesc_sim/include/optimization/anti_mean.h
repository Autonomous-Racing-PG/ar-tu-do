#pragma once
#include <deque>
#include "optimization/mean.h"

namespace tudora_optimization
{
    class AntiMean : public Mean
    {
	public:
		AntiMean(double window_size, double sampletime) : Mean(window_size,sampletime) {}
		inline virtual double optimize(double value)
		{
			return value - Mean::optimize(value);
		}	
    };
}