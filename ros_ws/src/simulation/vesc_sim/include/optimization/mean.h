#pragma once
#include <deque>
#include "optimization/optimizer.h"



    class Mean : public Optimizer
    {
	public:
		Mean(double window_size, double sampletime) : Optimizer(sampletime),
			window_size_(window_size), mean_(0)
		{}
		inline virtual double optimize(double value)
		{
			window_.push_front(value);
			while (window_.size()>window_size_)
			{
				window_.pop_back();
			}
			double sum = 0;
			for (auto it = window_.begin(); it != window_.end(); ++it)
			{
				sum += (*it);
			}
			mean_ = sum / window_size_;
			return mean_;
		}
		inline virtual double getMean() { return mean_; }
		inline virtual void reset() { window_.clear(); mean_ = 0; }
	protected:
		double window_size_;
		std::deque<double> window_;
		double mean_;
    };
