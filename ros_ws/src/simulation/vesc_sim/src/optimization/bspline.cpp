#include "optimization/bspline.h"



	BSplineOptimizer::BSplineOptimizer(double sampletime) : Optimizer(sampletime),
		num_points_(0)
	{

	}

	double BSplineOptimizer::optimize(double value)
	{
		
		double derivate = (value - pos_) / sampletime_;
		pos_=value;
		points_.push_front(derivate);
		while (points_.size() > num_points_)
		{
			points_.pop_back();
		}

		auto it_p = points_.begin();
		auto it_w = weights_.begin();
		double val = 0;
		double sum = 0;
		for (; it_p != points_.end();++it_p)
		{
			val += (*it_p)*(*it_w);
			sum += (*it_w);
			++it_w;
		}

		vel_ = (val / sum);
		return vel_;
	}

	void BSplineOptimizer::setWeights(std::deque<double>& weights)
	{
		num_points_ = weights.size();
		weights_ = weights;
	}

	inline void BSplineOptimizer::reset()
	{
		Optimizer::reset();
		points_.clear();
	}

