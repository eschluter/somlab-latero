// Erik Schluter
// 12/9/2015

#pragma once

#include "IndependentStimulator.h"

extern "C" {
#include "latero_core.h"
#include "latero_io.h"
#include "latero_testpattern.h"
}

namespace libStimulator {

class Latero : public IndependentStimulator {
	
	private:
		const int			SlipSteps;
		const int			RollSteps;
		const int			PressSteps;

		latero_t			latero;
		latero_t*			platero;
		latero_pkt_t		resp;
		latero_dst_device	destination;
		string				latero_ip;
		double				bladegap;
		double				t_distance;

		string				patternName;
		int					patternID;
		double				velocity;
		int					row;
		int					col;
		double				rampOffDur;

		LARGE_INTEGER		liPatternStart;
		LARGE_INTEGER		liRampStart;
		LARGE_INTEGER		liCurrent;
		LARGE_INTEGER		liFrequency;

	public:
		Latero(void);
		~Latero(void);

	private:
		bool			isActionFinished(double actionTime, ControlState &controlState);

	public:
		bool			configure(vector<TrialBlockEventFile::StimulatorBlock>::iterator stimulatorSpecIt, 
								vector<TrialBlockEventFile::StimulatorBlock>::iterator stimulatorSpecEndIt,
								bool shouldUseFullConfigure = true);

		bool			cleanup();
		bool			zero();
		bool			turnOn();
		bool			turnOff();
		void			go(bool shouldForceAction = false);

	private:
		void			setVelocity(const double& vel);
		void			setRow(const int& row);
		void			setColumn(const int& col);
		void			setPatternID(const __int64& id);

		void			runTestPatterns();
		void			runAtSpeedX(int nsteps = (LATERO_NB_PINS_X - 1));

		void			setPins(const int &step);
		void			recordLastZeroPins();
		void			setPinsZero();
		void			recordLastPins();
		void			setDiffPins();
		void			setPinsSlip(const int &step);
		void			setPinsRoll(const int &step);
		void			setPinsPress(const int &step);
		void			setRampOff(const double &rampOff);
		void			zeroAllPins();
		void			rampToZero();

	public:
		virtual void	createPatternApplication(TrialBlockEventFile::ActionBlock *actionBlock, TrialBlockEventFile::ChannelBlock *channelBlock);

	protected:
		virtual bool	setupAction(bool forceCompleteSetup = false);
};

}