#include "optimization/leastsquares1.h"



	LeastSquares1::LeastSquares1(unsigned int window_length, double sampletime) :LeastSquaresOptimizer(window_length, sampletime)
	{
		init();
	}

	double LeastSquares1::optimize(double value)
	{
		ticksPosVec_(index_) = value;
		//rechenen
		unsigned int startIndex = index_ - (window_length_ - 1);
		theta_pos_vec_ = help_1_*(help_2_*ticksPosVec_.segment(startIndex, window_length_));

		pos_ = theta_pos_vec_(0)*time_end_ + theta_pos_vec_(1);
		vel_ = theta_pos_vec_(0);
		//umkopieren, wenn am Ende
		++index_;
		if (index_ >= (window_length_ * 2))
		{
			for (unsigned int i = 0; i < window_length_ - 1; ++i)
			{
				ticksPosVec_[i] = ticksPosVec_[i + window_length_ + 1];
			}
			index_ = window_length_ - 1;
		}
		return pos_;
	}

	void LeastSquares1::init()
	{
		Eigen::MatrixXd w_mat_;
		Eigen::MatrixXd psi_mat_;

		time_vec_.setZero(window_length_);

		w_mat_.setIdentity(window_length_, window_length_);
		psi_mat_.setOnes(window_length_, 2);

		for (unsigned int i = 0; i < window_length_; ++i)
		{
			time_end_ = sampletime_ * i;
			time_vec_[i] = time_end_;
			psi_mat_(i, 0) = time_end_;
			//psi_mat_(i,1)=1 ist bereits durch setOnes() geschehen.
		}

		help_2_ = (psi_mat_.transpose()*w_mat_);
		help_1_ = help_2_*psi_mat_;
		help_1_ = help_1_.inverse();
	}
