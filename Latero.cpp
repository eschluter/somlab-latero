// Erik Schluter
// 12/9/2015

#include "Latero.h"

namespace libStimulator {

static double		pattern[LATERO_NB_PINS];
static double		last[LATERO_NB_PINS];
static double		diff[LATERO_NB_PINS];

Latero::Latero()
:
SlipSteps(7),
RollSteps(6),
PressSteps(4)
{
	latero_ip	= "192.168.87.98";
	platero		= &latero;
	patternID	= -1;
	velocity	= 0.0;
	rampOffDur	= 0.0;
	row			= 0;
	col			= 0;
	bladegap	= 1.2; //in millimeters
	t_distance	= bladegap * 7; //times number of spaces across grid(7)

	QueryPerformanceFrequency ( &liFrequency );
}

Latero::~Latero() {
}

bool Latero::configure(vector<TrialBlockEventFile::StimulatorBlock>::iterator stimulatorSpecIt,
							vector<TrialBlockEventFile::StimulatorBlock>::iterator stimulatorSpecEndIt,
							bool shouldUseFullConfigure) {
	shouldUseFullConfigure;
	
	if (stimulatorSpecIt != stimulatorSpecEndIt && stimulatorSpecIt->isInitialized("eve_stimulator_ip_address")) {
		latero_ip	= (string)stimulatorSpecIt->at("eve_stimulator_ip_address").getValue();
	}

	return true;
}

bool Latero::turnOn() {
	int errorCode;

	errorCode = latero_open(platero, latero_ip.c_str());
	if ( errorCode < 0 ) {
		cerr << "Could not connect to Latero device at address: " << latero_ip << '\n';
		cerr << "Returned error code: " << errorCode << endl;
		return false;
	} else {
		cout << "Latero device connected at address: " << latero_ip << endl;
	}
	return true;
}

bool Latero::turnOff() {
	cout << "Latero disconnected" << endl;
	if ( latero_close(platero) < 0 )
		return false;
	return true;
}

bool Latero::zero() {
	latero_test_connection(platero);
	setPinsZero();
	return true;
}

bool Latero::cleanup() {
	return true;
}

void Latero::createPatternApplication(TrialBlockEventFile::ActionBlock *actionBlock, TrialBlockEventFile::ChannelBlock *channelBlock) {
	Action			*patternAction		= new PinPatternAction();
	double			duration			= 0.0;
	string			pattern				= "";
	__int64			patternID			= -1;
	__int64			row					= 0;
	__int64			col					= 0;
	double			vel					= 0.0;
	double			rampOff				= 0.0;
	
	if (channelBlock->isInitialized("eve_stim_dur")) {
		duration	= (double)channelBlock->at("eve_stim_dur").getValue();
	}

	if (channelBlock->isInitialized("eve_stim_pattern_type")) {
		pattern		= (string)channelBlock->at("eve_stim_pattern_type").getValue();
	}

	if (channelBlock->isInitialized("eve_stim_pattern_id")) {
		patternID	= (__int64)channelBlock->at("eve_stim_pattern_id").getValue();
	}

	if (channelBlock->isInitialized("eve_stim_loc_row")) {
		row			= (__int64)channelBlock->at("eve_stim_loc_row").getValue() - 1;
	}

	if (channelBlock->isInitialized("eve_stim_loc_col")) {
		col			= (__int64)channelBlock->at("eve_stim_loc_col").getValue() - 1;
	}

	if (channelBlock->isInitialized("eve_vel")) {
		vel			= (double)channelBlock->at("eve_vel").getValue();
	}

	if (channelBlock->isInitialized("eve_stim_ramp_off_dur")) {
		rampOff		= (double)channelBlock->at("eve_stim_ramp_off_dur").getValue();
	}

	patternAction->create(pattern, patternID, duration, row, col, vel, rampOff);
	patternAction->setPreTrial(actionBlock->isPreTrial());
	patternAction->setPostTrial(actionBlock->isPostTrial());
	patternAction->setStimulus(actionBlock->isStimulus());
	patternAction->setAbort(actionBlock->isAbort());
	addAnalogAction(patternAction, (unsigned int)0);
}

void Latero::go(bool shouldForceAction) {

	shouldForceAction;
	// Verify that action exists and is prepared
	if (actionIt[PrimaryChannel] < analogActions[actionActivePtr][PrimaryChannel].end()) {

		PinPatternAction	*action	= (PinPatternAction*)(*actionIt[PrimaryChannel]);
		action->setStarted();

		switch (action->getPatternMode()) {
			case PinPatternAction::TEST:
				runTestPatterns();
				break;
			case PinPatternAction::SLIP:
				cout << "Slip" << endl;
				runAtSpeedX(SlipSteps);
				break;
			case PinPatternAction::ROLL:
				cout << "Roll" << endl;
				runAtSpeedX(RollSteps);
				break;
			case PinPatternAction::PRESSURE:
				cout << "Pressure" << endl;
				runAtSpeedX(PressSteps);
				break;
			default:
				break;
		}
	}
}

bool Latero::isActionFinished(double actionTime, ControlState &controlState) {
	controlState;

	if (actionIt[PrimaryChannel] >= analogActions[actionActivePtr][PrimaryChannel].end()) {
		return true;
	} else if (!(*actionIt[PrimaryChannel])->isStarted()) {
		return false;
	} else if ((*actionIt[PrimaryChannel])->isFinished()) {
		rampToZero();
		return true;
	} else if (stimDuration > 0.0) {
		if ((actionTime - actionStartTimes[PrimaryChannel]) >= stimDuration) {
			rampToZero();
			(*actionIt[PrimaryChannel])->setFinished();
			return true;
		} else {
			return false;
		}
	} else {
		rampToZero();
		(*actionIt[PrimaryChannel])->setFinished();
	}
	return true;
}

void Latero::runTestPatterns() {

	TestSplit1(platero);
	Sleep(1000);
	TestSplit2(platero);
	Sleep(1000);
	TestAllpin(platero);
	Sleep(1000);
	TestRow(platero);
	Sleep(1000);
	TestCol(platero);
	Sleep(1000);
	TestFirstlast(platero);
	cout << "\nTest patterns completed.\n" << endl;
}

void Latero::runAtSpeedX(int nsteps) {
	double	sec;
	double	current		= 0.0;
	double	target		= 0.0;
	double	ramp_sec	= 0.0;
	int		it			= 0;
	int		direction	= 0;
	double	speed		= 0.0;
	double	inter_time	= 0.0;
	double	frame[LATERO_NB_PINS];
	QueryPerformanceCounter( &liPatternStart );

	// Determine stimulus direction (negative speed - left, positive speed - right)
	if (velocity < 0) {
		direction	= -1;
		speed		= -1 * velocity;
	} else {
		direction	= 1;
		speed		= velocity;
	}

	// Determine total time and interval time
	inter_time	= bladegap / speed;
	ramp_sec	= 0.95 * inter_time; // leave a tiny gap to make sure we can get to our destination

	// run the pattern at specified speed
	for (int i = 0; i < nsteps; i++) {
		it = (direction == -1) ? (LATERO_NB_PINS_X-2)-i : i;
		setPins(it);
		// move to target position in ramp_sec seconds
		sec = 0.0;
		setDiffPins();
		QueryPerformanceCounter( &liRampStart );
		while (sec < ramp_sec) {
			// interpolate and write
			for (int j = 0; j < LATERO_NB_PINS; j++)
				frame[j] = last[j] + (sec/ramp_sec * diff[j]);
			latero_write_ether(platero, frame);
			// get time since start in seconds - this order avoids overshooting ramp_sec
			QueryPerformanceCounter( &liCurrent );
			sec = (double)( (liCurrent.QuadPart - liRampStart.QuadPart)* (double)1.0/(double)liFrequency.QuadPart );
		}
		latero_write_ether(platero, pattern);
		QueryPerformanceCounter( &liCurrent );
		target	= (i+1)*inter_time;
		current = (double)( (liCurrent.QuadPart - liPatternStart.QuadPart)* (double)1.0/(double)liFrequency.QuadPart );
		if ( current < target )
			Sleep((target - current)*1000);
	}
}

void Latero::setPins(const int &step) {
	// record last positions and zero the array
	recordLastZeroPins();

	switch (patternID) {
		case 0:
			setPinsSlip(step);
			break;
		case 1:
			setPinsRoll(step);
			break;
		case 2:
			setPinsPress(step);
			break;
		default:
			cerr << "Invalid Pattern ID: " << patternID << endl;
	}
}

void Latero::setPinsSlip(const int &step) {
	int start	= row;
	int numRows = row+1;
	// run over all rows simultaneously if specified (row = -1)
	if (row < 0) {
		numRows = LATERO_NB_PINS_Y;
		start = 0;
	}
	// set pattern
	for (int i = start; i < numRows; i++) {
		for (int j = 0; j < LATERO_NB_PINS_X; j++) {
			pattern[i*LATERO_NB_PINS_X+j] = (j <= step) ? -1 : 1;
		}
	}
}

void Latero::setPinsRoll(const int &step) {
	int start	= row;
	int numRows = row+1;
	// run over all rows simultaneously if specified (row = -1)
	if (row < 0) {
		numRows = LATERO_NB_PINS_Y;
		start = 0;
	}
	// set pattern
	for (int i = start; i < numRows; i++) {
		for (int j = 0; j < LATERO_NB_PINS_X; j++) {
			if (j < step+1) {
				pattern[i*LATERO_NB_PINS_X+j] = -1;
			} else if (j > step+1) {
				pattern[i*LATERO_NB_PINS_X+j] = 1;
			} //leave at 0 if (j == step+1)
		}
	}
}

void Latero::setPinsPress(const int &step) {
	int l,r,f;
	int start		= row;
	int numRows		= row+1;
	double maxVal	= 0.0;
	double fillVal	= 0.0;
	double interval = 0.0;
	double fill[LATERO_NB_PINS_X];
	// run over all rows simultaneously if specified (row = -1)
	if (row < 0) {
		numRows = LATERO_NB_PINS_Y;
		start = 0;
	}
	//specify fill values
	interval	= (double)(1.0/PressSteps); 
	maxVal		= (step+1)*interval;
	for (f = 0; f < PressSteps; f++) {
		fillVal += interval;
		fill[f] = (fillVal < maxVal) ? fillVal : maxVal;
	}
	//set pattern
	for (int i = start; i < numRows; i++) {
		l = PressSteps-1;
		r = PressSteps;
		f = 0;
		while (r < LATERO_NB_PINS_X) {
			pattern[i*LATERO_NB_PINS_X+r] = fill[f];
			pattern[i*LATERO_NB_PINS_X+l] = -fill[f];
			r++;l--;f++;
		}
	}
}

void Latero::recordLastZeroPins() {
	for (int i = 0; i < LATERO_NB_PINS; i++) {
		last[i]		= pattern[i];
		pattern[i]	= 0;
	}
}

void Latero::setDiffPins() {
	for (int i = 0; i < LATERO_NB_PINS; i++)
		diff[i] = pattern[i] - last[i];
}

void Latero::setPinsZero() {
	for (int i = 0; i < LATERO_NB_PINS; i++) {
		pattern[i]	= 0;
		last[i]		= 0;
		diff[i]		= 0;
	}
}

void Latero::recordLastPins() {
	for (int i = 0; i < LATERO_NB_PINS; i++)
		last[i]		= pattern[i];
}

//usually used to zero pin array when pattern application complete
void Latero::zeroAllPins() {
	setPinsZero();
	latero_write_ether(platero, pattern);
}

void Latero::rampToZero() {
	double sec = 0.0;
	double frame[LATERO_NB_PINS];

	if (rampOffDur > 0) {
		recordLastZeroPins();
		setDiffPins();
		QueryPerformanceCounter( &liRampStart );
		while (sec < rampOffDur) {
			// interpolate and write
			for (int j = 0; j < LATERO_NB_PINS; j++)
				frame[j] = last[j] + (sec/rampOffDur * diff[j]);
			latero_write_ether(platero, frame);
			// get time since start in seconds - this order avoids overshooting ramp_sec
			QueryPerformanceCounter( &liCurrent );
			sec = (double)( (liCurrent.QuadPart - liRampStart.QuadPart)* (double)1.0/(double)liFrequency.QuadPart );
		}
		latero_write_ether(platero, pattern);
	} else {
		zeroAllPins();
	}
}


void Latero::setVelocity(const double& vel) {
	this->velocity = vel;
}

void Latero::setRow(const int& row) {
	if (row > LATERO_NB_PINS_Y - 1) {
		cout << "Row exceeds latero max: " << LATERO_NB_PINS_Y << endl;
		cout << "Setting to max." << endl;
		this->row = LATERO_NB_PINS_Y - 1;
	} else {
		this->row = row;
	}
}

void Latero::setColumn(const int& col) {
	if (col > LATERO_NB_PINS_X - 1) {
		cout << "Column exceeds latero max: " << LATERO_NB_PINS_X << endl;
		cout << "Setting to max." << endl;
		this->col = LATERO_NB_PINS_X - 1;
	} else {
		this->col = col;
	}
}

void Latero::setPatternID(const __int64& id) {
	this->patternID = (int)id;
}

void Latero::setRampOff(const double& rampOff) {
	this->rampOffDur = rampOff;
}

bool Latero::setupAction(bool forceCompleteSetup) {
	forceCompleteSetup;

	PinPatternAction	*action	= (PinPatternAction*)(*actionIt[PrimaryChannel]);

	setPatternID(action->getPatternID());
	switch (action->getPatternMode()) {
		case PinPatternAction::SLIP:
			setVelocity(action->getVelocity());
			setRow(action->getRow());
			setColumn(action->getColumn());
			setRampOff(action->getRampOff());
			break;
		case PinPatternAction::ROLL:
			setVelocity(action->getVelocity());
			setRow(action->getRow());
			setRampOff(action->getRampOff());
			break;
		case PinPatternAction::PRESSURE:
			setVelocity(action->getVelocity());
			setRow(action->getRow());
			setRampOff(action->getRampOff());
			break;
		default:
			break;
	}
	return true;
}

}