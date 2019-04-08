#pragma once

#include "optimization/optimizer.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


	class LeastSquaresOptimizer : public Optimizer
	{
	public:
		LeastSquaresOptimizer(unsigned int window_length, double sampletime);
		virtual ~LeastSquaresOptimizer() = default;
		virtual double optimize(double value)=0;
		virtual void setSampletime(double sampletime) override;
		inline virtual void reset() override;

	protected:
		virtual void init() =0;

		const unsigned int window_length_;
		unsigned int index_;
		Eigen::RowVectorXd time_vec_;
		Eigen::VectorXd ticksPosVec_;
		Eigen::MatrixXd help_1_;
		Eigen::MatrixXd help_2_;
		Eigen::VectorXd theta_pos_vec_;
	};
