#pragma once
#include <deque>
#include "optimization/optimizer.h"

namespace tudora_optimization
{
	class BSplineOptimizer :public Optimizer
	{
	public:
		BSplineOptimizer(double sampletime);
		virtual double optimize(double value) override;
		virtual ~BSplineOptimizer() {}
		void setWeights(std::deque<double>& weights);
		inline virtual void reset() override;
	private:
		std::deque<double> points_;
		std::deque<double> weights_;
		unsigned int num_points_;
	};
}