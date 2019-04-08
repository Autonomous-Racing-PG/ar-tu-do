#pragma once
#include "optimization/optimizer.h"



	/**
	* @brief
	* DE:
	* EN:
	*
	*/
	class LinearOptimizer : public Optimizer
	{
	public:
		/**
		* DE: Konstruktor, initialisert die Abtastzeit und alle anderen Attribute mit 0
		* EN: Constructor, initializes sampletime and all other attributes with 0
		* @param[in] sampletime in seconds
		*/
		LinearOptimizer(double sampletime) : Optimizer(sampletime), old_value_(0) {}

		/**
		* DE: Mit dem Parameterwert und dem letzen Parameterwert (vorheriger Aufruf) wird die Ableitung mittels
		*		finiter Differenzen ermittelt. Falls es keinen vorherigen Aufruf gab, so wird die Differenz mit 0 gebildet.
		* EN: Derivation, calculated by differences. Parameter minus old darameter (initial 0) divided by sampletime
		* param[in] value current value
		* @return calculated value (e.g. filtered, or median,...)
		*/
		inline virtual double optimize(double value) override
		{
			pos_= value;
			vel_ = (value - old_value_) / sampletime_;
			old_value_ = value;
			return vel_;
		}

		/**
		* DE: virtueller Destruktor, wichtig bei Verbung, falls eine Kind-Klasse einen Destruktor verwendet
		* EN: virtual destructor for polymorphy, important for child classes
		*/
		virtual ~LinearOptimizer() = default;

		/**
		*
		*
		*/
		inline virtual void reset() override { old_value_ = 0; Optimizer::reset(); }
	private:

		/**
		* DE: aktuelle/letzte Eingabe von optimize zum Loggen
		* EN: current/last input of optimize to log this data
		*/
		double old_value_;
	};
