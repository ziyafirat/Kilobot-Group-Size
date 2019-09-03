/* Include the controller definition */
#include "kilobot_aggregation.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <vector>
#include <algorithm>

/****************************************/
/****************************************/
// run
//mkdir build
//cd build/
//cmake -DCMAKE_BUILD_TYPE=Release ..
//make
// argos3 -c src/aggregation/experiments/kilobot_aggregation.argos

#define PIN_FORWARD 1.0f
#define PIN_TURN    1.57f
#define PIN_STOP    0.0f
#define COMPLETE_TURN 50 // = pi/(omega delta_t) = pi/(v*delta_t/l) = (pi*l)/(v*delta_t)
#define MAX_WALK 100

#define STATE_WALK 0
#define STATE_STAY 1
#define SPOT_WHITE 2
#define SPOT_BLACK 3

CKilobotAggregation::CKilobotAggregation() :
		m_pcMotors(NULL), m_pcRABA(NULL), m_pcRABS(NULL), m_pcRNG(NULL), lastMove(
				0), avoidTurns(0), walkTurns(50), minDist(1), state(STATE_WALK), a(
				1.7), b(3.9), numOfNeighboursWhileJoining(-1), numOfNeighboursWhileOnSite(
				-1), spotInfoSite(-1), computeProbVal(-1), spotInf(-1), numInformedRobot(
				10), k(15) {
	//a(1.7), b(3.9),values that best fit Correl & Martinolli
}

/****************************************/
/****************************************/

void CKilobotAggregation::Init(TConfigurationNode &t_node) {
	/*
	 * Get sensor/actuator handles
	 *
	 * The passed string (ex. "differential_steering") corresponds to the
	 * XML tag of the device whose handle we want to have. For a list of
	 * allowed values, type at the command prompt:
	 *
	 * $ argos3 -q actuators
	 *
	 * to have a list of all the possible actuators, or
	 *
	 * $ argos3 -q sensors
	 *
	 * to have a list of all the possible sensors.
	 *
	 * NOTE: ARGoS creates and initializes actuators and sensors
	 * internally, on the basis of the lists provided the configuration
	 * file at the <controllers><footbot_aggregation><actuators> and
	 * <controllers><footbot_aggregation><sensors> sections. If you forgot to
	 * list a device in the XML and then you request it here, an error
	 * occurs.
	 */
	m_pcMotors = GetActuator<CCI_DifferentialSteeringActuator>(
			"differential_steering");
	m_pcRABA = GetActuator<CCI_KilobotCommunicationActuator>(
			"kilobot_communication");
	m_pcRABS = GetSensor<CCI_KilobotCommunicationSensor>(
			"kilobot_communication");
	/*
	 * Parse the configuration file
	 */
	GetNodeAttributeOrDefault(t_node, "minDist", minDist, minDist);
	GetNodeAttributeOrDefault(t_node, "aParam", a, a);
	GetNodeAttributeOrDefault(t_node, "bParam", b, b);

	/* Create a random number generator. We use the 'argos' category so
	 that creation, reset, seeding and cleanup are managed by ARGoS. */
	m_pcRNG = CRandom::CreateRNG("argos");
	Reset();
}

/****************************************/
/****************************************/

void CKilobotAggregation::Reset() {
	message_t msg;
	msg.type = NORMAL;
	for (int i = 0; i < 9; ++i)
		msg.data[i] = 0;
	msg.crc = 0; //not implemented yet, I reckon
	m_pcRABA->SetMessage(&msg);
	walkTurns = MAX_WALK;
	ChangeState(STATE_WALK);
}

/****************************************/
/****************************************/

void CKilobotAggregation::ControlStep() {
	++lastMove;

	//send state
	message_t msg;
	msg.type = NORMAL;
	for (int i = 0; i < 9; ++i)
		msg.data[i] = 0;
	msg.data[0] = 1;
	msg.data[3] = atoi(GetId().c_str());
	msg.crc = 0; //not implemented yet, I reckon
	m_pcRABA->SetMessage(&msg);
	if (GetId() == "101" || GetId() == "102") {
		ChangeState(STATE_STAY);
	} else {
		switch (state) {
		case STATE_WALK:
			Walk();
			lastMove = 0;
			break;
		case STATE_STAY:
			Stay();
			break;
		}
	}
}

/****************************************/
/****************************************/

void CKilobotAggregation::Move() {
	if (--walkTurns == 0) {
		UInt32 direction = m_pcRNG->Uniform(CRange<UInt32>(0, 2));
		if (direction == 0)
			m_pcMotors->SetLinearVelocity(PIN_TURN, PIN_STOP);
		else
			m_pcMotors->SetLinearVelocity(PIN_STOP, PIN_TURN);

		avoidTurns = m_pcRNG->Uniform(CRange<UInt32>(1, COMPLETE_TURN));

	} else if (--avoidTurns == 0) {
		m_pcMotors->SetLinearVelocity(PIN_FORWARD, PIN_FORWARD);
		walkTurns = MAX_WALK;
	}
}

void CKilobotAggregation::ChangeState(unsigned short int newState) {
	state = newState;

	switch (state) {
	case STATE_WALK:
		m_pcMotors->SetLinearVelocity(PIN_FORWARD, PIN_FORWARD);
		numOfNeighboursWhileJoining = -1;
		numOfNeighboursWhileOnSite = -1;
		spotInfoSite = -1;
		break;
	case STATE_STAY:
		m_pcMotors->SetLinearVelocity(PIN_STOP, PIN_STOP);
		break;
	}
}

unsigned int CKilobotAggregation::CountNeighbours() {
	unsigned int counter = 0;
	if (m_pcRABS->MessageSent()) {
		const CCI_KilobotCommunicationSensor::TPackets &tPackets =
				m_pcRABS->GetPackets();

		for (size_t i = 0; i < tPackets.size(); ++i) {
			if (tPackets[i].Message->data[0] == STATE_STAY) { //and tPackets[i].Distance.high_gain < minDist) {

				++counter;
				LOGERR << "	tPackets[i].Message->data[0]:" << tPackets[i].Message->data[0] << " spotInf:" << tPackets[i].Message->data[3] << " robot:"
								<< GetId() << std::endl;
			}
		}
	}

	if (counter > 1)
		cout << counter << endl;
	return counter;
}

float CKilobotAggregation::ComputeProba(unsigned int n) {
	float res = 0;
	switch (state) {
	case STATE_WALK: //P_join

		res = 1; // = 0.48 * (1 - exp(-a * n)) + 0.03;

//		LOGERR << "	joinneigh:" << n << " spotInf:" << state << " robot:"
//				<< GetId() << std::endl;
		break;
	case STATE_STAY: //1-P_leave
		res = 0; // = 1 - exp(-b * n);

//		LOGERR << "	leavneigh:" << n << " spotInf:" << state << " robot:"
//				<< GetId() << std::endl;
		break;

//	case STATE_WALK: //P_join
//
//		if (numOfNeighboursWhileJoining < 0) { //while non informed robot join, calculate number of neighbors
//			numOfNeighboursWhileJoining = n;  // n is counted neighbors
//			numOfNeighboursWhileOnSite = n;
//			spotInfoSite = spotInf;
//		}
//		//if (n > 0) {
//		return 1;
//		//			} else {
//		//				return 0;
//		//			}
//		break;
//	case STATE_STAY: //1-P_leave
//
//		numOfNeighboursWhileOnSite = n;
//		int value = atoi(GetId().c_str());       // get robot ID
//
//		//			clockCounter++;
//		//0 is black and 1 is white.
//		//m_cOutFile << clockCounter << "	" << n << "	" << spotInf << endl;
//
//					LOGERR << "	neigh:" << n
//							<< " spotInf:" << spotInf << " robot:" << GetId()
//							<< " numInfR:" << numInformedRobot << std::endl;
//
//		//if (value >= numInformedRobot && numInformedRobot > 0) { // if robot non informed robot and total informed > 0 use new method
//
//		float Res1 = std::numeric_limits<float>::infinity();
//		Res1 = Exp(-a * (k - Abs(n - numOfNeighboursWhileJoining)));
//		computeProbVal = Res1;
//
//		float one = 1.0f;
//
//		if (value >= numInformedRobot) { // if robot non informed robotE
//
//			if (n > 0) {
//
//				return Min(one, Res1);
//			} else {
//				return 1;
//			}
//		} else {
//
//			return Min(one, Res1);
//		}
//
//		break;

	}
	return res;
}

void CKilobotAggregation::Walk() {
	float p = ComputeProba(CountNeighbours());
	if (m_pcRNG->Uniform(CRange<Real>(0.0f, 1.0f)) < p) {
		//if (GetId() == "kb1" || GetId() == "kb2") {

			ChangeState(STATE_STAY);
		//}
	} else
		Move();
}

void CKilobotAggregation::Stay() {
	m_pcMotors->SetLinearVelocity(PIN_STOP, PIN_STOP);
	float p = ComputeProba(CountNeighbours());
	if (m_pcRNG->Uniform(CRange<Real>(0.0f, 1.0f)) >= p){
		ChangeState(STATE_WALK);
	}
}

string CKilobotAggregation::GetState() {
	string res = "";
	switch (state) {
	case STATE_WALK:
		res = "WALK";
		break;
	case STATE_STAY:
		res = "STAY";
		break;
	}
	return res;
}

int CKilobotAggregation::LastMove() {
	return lastMove;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.  The string is then usable in the configuration
 * file to refer to this controller.  When ARGoS reads that string in
 * the configuration file, it knows which controller class to
 * instantiate.  See also the configuration files for an example of
 * how this is used.
 */
REGISTER_CONTROLLER(CKilobotAggregation, "kilobot_aggregation_controller")
