#pragma once


	/**
	* @brief
	* DE:
	* EN:
	* 
	*/
	class Optimizer
	{
	public:
		/**
		* DE: Konstruktor, initialisert die Abtastzeit
		* EN: Constructor, initializes sampletime
		* @param[in] sampletime in seconds
		*/
		Optimizer(double sampletime) : sampletime_(sampletime), pos_(0), vel_(0){}

		/**
		* DE: virtueller Destruktor, wichtig bei Verbung, falls eine Kind-Klasse einen Destruktor verwendet
		* EN: virtual destructor for polymorphy, important for child classes
		*/
		virtual ~Optimizer() = default;

		/**
		* DE: Der aktuelle Wert wird als Parameter �bertragen. Diser wird verabeitet und zur�ckgegeben
		*		Beispiele: Filter, Median, Ableitung, Least Squared, B-Splines
		* EN: Examples to calculate: filter, median, derivation, least Squared, B-Splines
		* param[in] value current value
		* @return calculated value (e.g. filtered, or median,...)
		*/
		inline virtual double optimize(double value) = 0;

		/**
		* DE: Funktion um die Abtastzeit zur Laufzeit zur �ndern
		* EN: Function to modify the sample time during runtime
		* @param[in] sampletime in seconds
		*/
		inline virtual void setSampletime(double sampletime) { sampletime_ = sampletime; }

		/**
		* DE: wird bei der Nullung der Encoder aufgerufen
		*
		*/
		inline virtual void reset(){ pos_=0; vel_=0; }

		inline double getPosition(){return pos_;}
		inline double getVelocity(){return vel_;}

	protected:

		/**
		* DE: Abtastzeit, feste Zeit zur effizienten Berechnung
		* EN: Sampletime, constant time for efficient calculation
		* UNTI: seconds
		*/
		double sampletime_;

		/**
		* DE: aktuelle/letzte Position von optimize zum Loggen
		* EN: current/last position of optimize to log this data
		*/
		double pos_;
		/**
		* DE: aktuelle/letzte Geschwindigkeit von optimize zum Loggen
		* EN: current/last velocity of optimize to log this data
		*/
		double vel_;
	};
