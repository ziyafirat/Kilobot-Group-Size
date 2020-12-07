#include "aggregation.h"
//#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <algorithm>
#include <cstring>
#include <cerrno>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_controller.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_measures.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
//#include <aggregation/behaviors/aggregation.c>
/****************************************/
/****************************************/
using namespace std;

static const Real BOT_RADIUS = 0.033f;
static const UInt32 MAX_PLACE_TRIALS = 20;

/****************************************/
/****************************************/

CAggregation::CAggregation() {
}

/****************************************/
/****************************************/

CAggregation::~CAggregation() {
	/* Nothing to do */
}

/****************************************/
/****************************************/

void CAggregation::Init(TConfigurationNode &t_tree) {
	/* Get output file name from XML tree */
	GetNodeAttribute(t_tree, "output", m_strOutFile);
	GetNodeAttributeOrDefault(t_tree, "timeStopCond", timeStopCond,
			timeStopCond);

	int nBots;
	GetNodeAttributeOrDefault(t_tree, "nBots", nBots, nBots);

	GetNodeAttributeOrDefault(t_tree, "aParam", a, a);
	GetNodeAttributeOrDefault(t_tree, "commRange", commRange, commRange);

	GetNodeAttributeOrDefault(t_tree, "link", link, link);

	GetNodeAttributeOrDefault(t_tree, "mutation", m, m);

	GetNodeAttribute(t_tree, "totalSpots", totalSpots);
	GetNodeAttribute(t_tree, "radiusSpot", m_fRadius);
	GetNodeAttribute(t_tree, "blackSpotSize", blackSpotVector);
	GetNodeAttribute(t_tree, "whiteSpotSize", whiteSpotVector);
	GetNodeAttributeOrDefault(t_tree, "minDist", minDist, minDist);
	GetNodeAttributeOrDefault(t_tree, "timeStopCond", timeStopCond,
			timeStopCond);

	m_cCoordBlackSpot = CVector2(-blackSpotVector, 0);
	m_cCoordWhiteSpot = CVector2(whiteSpotVector, 0);

	/* Open the file for text writing */
	m_cOutFile.open(m_strOutFile.c_str(),
			std::ofstream::out | std::ofstream::trunc);
	if (m_cOutFile.fail()) {
		THROW_ARGOSEXCEPTION(
				"Error opening file \"" << m_strOutFile << "\": " << ::strerror(errno));
	}

	beacon_blue_count = 0;
	beacon_red_count = 0;

//	informed_size_blue_beacon = 5;
//	informed_size_red_beacon = 5;
//	total_informed_size = 10;

////////////////////////////////////////////////////////////////////////////////// CREATION AND POSITIONING OF THE ARENA WALLS////////////////////////////////////////////////////////////////////////////////
	CVector3 arena_size = GetSpace().GetArenaSize();
	float m_fArenaRadius = Min(arena_size[0], arena_size[1]) / 2;
	/*switch(nBots) {
	 case 25:
	 m_fArenaRadius = 0.35;
	 break;
	 case 50:
	 m_fArenaRadius = 0.5;
	 break;
	 case 100:
	 m_fArenaRadius = 0.7;
	 break;
	 }*/
	unsigned int m_unNumArenaWalls = 20;

	CRadians wall_angle = CRadians::TWO_PI / m_unNumArenaWalls;
	CVector3 wall_size(0.01,
			2.0 * m_fArenaRadius * Tan(CRadians::PI / m_unNumArenaWalls), 0.05);
	ostringstream entity_id;
	for (UInt32 i = 0; i < m_unNumArenaWalls; i++) {
		entity_id.str("");
		entity_id << "wall_" << i;
		CRadians wall_rotation = wall_angle * i;

		CVector3 wall_position(m_fArenaRadius * Cos(wall_rotation),
				m_fArenaRadius * Sin(wall_rotation), 0);
		CQuaternion wall_orientation;
		wall_orientation.FromEulerAngles(wall_rotation, CRadians::ZERO,
				CRadians::ZERO);

		CBoxEntity *box = new CBoxEntity(entity_id.str(), wall_position,
				wall_orientation, false, wall_size, (Real) 1.0);
		AddEntity(*box);
	}

	CKilobotEntity *pcKB;
	for (int i = 0; i < nBots; ++i) {
		pcKB = new CKilobotEntity("" + ToString(i), "kbc", CVector3(0, 0, 0),
				CQuaternion(0, 0, 0, 0), commRange);
		bots.push_back(pcKB);
		AddEntity(*pcKB);
	}

	PlaceBots(m_fArenaRadius);

}

void CAggregation::PlaceBots(float m_fArenaRadius) {

	CVector3 cPosition;
	CQuaternion cOrientation;
	cPosition.SetZ(0.0);
	CRandom::CRNG *m_pcRNG = CRandom::CreateRNG("argos");
	unsigned int unTrials;
	CKilobotEntity *pcKB;
	for (unsigned int i = 0; i < bots.size(); ++i) {
		bool bDone = false;
		unTrials = 0;

		pcKB = bots[i];
		do {
			CRadians cRandomAngle = CRadians(
					m_pcRNG->Uniform(
							CRange<Real>(-CRadians::PI.GetValue(),
									CRadians::PI.GetValue())));
			Real cRandomRadius = m_pcRNG->Uniform(
					CRange<Real>(-m_fArenaRadius, m_fArenaRadius));

			cPosition.SetX(cRandomRadius * Cos(cRandomAngle));
			cPosition.SetY(cRandomRadius * Sin(cRandomAngle));

			CRadians cRandomOrientation = CRadians(
					m_pcRNG->Uniform(
							CRange<Real>(-CRadians::PI.GetValue(),
									CRadians::PI.GetValue())));
			cOrientation.FromEulerAngles(cRandomOrientation, CRadians::ZERO,
					CRadians::ZERO);

			bDone = MoveEntity(pcKB->GetEmbodiedEntity(), cPosition,
					cOrientation);
			++unTrials;
		} while (!bDone && unTrials <= MAX_PLACE_TRIALS);
		if (!bDone) {
			THROW_ARGOSEXCEPTION("Can't place " << "kb_" + ToString(i));
		}
	}

}

/****************************************/
/****************************************/

void CAggregation::Reset() {
	//PlaceBots();
	/* Close the output file */
	m_cOutFile.close();
	if (m_cOutFile.fail()) {
		THROW_ARGOSEXCEPTION(
				"Error closing file \"" << m_strOutFile << "\": " << ::strerror(errno));
	}

	/* Open the file for text writing */
	m_cWordFile.open(m_strWordFile.c_str(),
			std::ofstream::out | std::ofstream::trunc);
	if (m_cWordFile.fail()) {
		THROW_ARGOSEXCEPTION(
				"Error opening file \"" << m_strWordFile << "\": " << ::strerror(errno));
	}
}

/****************************************/
/****************************************/

void CAggregation::Destroy() {
	/* Close the output file */
	m_cOutFile.close();
	if (m_cOutFile.fail()) {
		THROW_ARGOSEXCEPTION(
				"Error closing file \"" << m_strOutFile << "\": " << ::strerror(errno));
	}
}

/****************************************/
/****************************************/

void CAggregation::PreStep() {
	//int clock = GetSpace().GetSimulationClock();
	/* Nothing to do */
}

//void CAggregation::PostStep() {
//
//	  int recordSteps = 100;
//	    int clock = GetSpace().GetSimulationClock();
//	    if(clock%recordSteps==0)
//	        m_cOutFile << clock << " ";
//
//	    for(unsigned int i=0; i<bots.size(); ++i) {
//	        CKilobotEntity& kbEntity = *any_cast<CKilobotEntity*>(bots[i]);
//	        CKilobotCommunicationEntity kilocomm = kbEntity.GetKilobotCommunicationEntity();
//	        if(kilocomm.GetTxStatus()==CKilobotCommunicationEntity::TX_SUCCESS)
//	            stayArray[i] = kilocomm.GetTxMessage()->data[2]!=0;
//
//	        int clock = GetSpace().GetSimulationClock();
//	        if(clock%recordSteps==0) {
//	            Real Robot_X = kbEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
//	            Real Robot_Y = kbEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
//	            m_cOutFile << "( " << Robot_X << " , " << Robot_Y << " , " << stayArray[i] << ") ";
//	        }
//	    }
//
//	    if(clock%recordSteps==0)
//	        m_cOutFile<<endl;
//
////	int clock = GetSpace().GetSimulationClock();
////	if (clock % 100 == 0) {
////
//////		 /* Go through the kilobots */
//////		   for(size_t i = 0; i < m_tKBs.size(); ++i) {
//////		      /* Create a pointer to the kilobot state */
//////		      kilobot_state_t* ptState = m_tKBs[i]->CCI_KilobotController;
//////		      /* Print current state internal robot state */
//////		      LOG << ptState->tx_state << ": "                       << std::endl
//////		          << "\ttx_state: "           << ptState->tx_state          << std::endl
//////		          << "\trx_state: "           << ptState->rx_state          << std::endl
//////		          << "\tambientlight: "       << ptState->ambientlight      << std::endl
//////		          << "\tleft_motor: "         << ptState->left_motor        << std::endl
//////		          << "\tright_motor: "        << ptState->right_motor       << std::endl
//////		          << "\tcolor: "              << ptState->color             << std::endl
//////		          << "\tgradient: "           << m_tKBs[i].second->gradient << std::endl;
//////		   }
////
////
//////		   for(size_t i = 0; i < m_tKBs.size(); ++i) {
//////		      /* Create a pointer to the kilobot state */
//////		      kilobot_state_t* ptState = m_tKBs[i];
//////		      /* Print current state internal robot state */
//////		      m_cOutFile << ptState->tx_state  << ": "                       << std::endl
//////		          << "\ttx_state: "           << ptState->tx_state          << std::endl
//////		          << "\trx_state: "           << ptState->rx_state          << std::endl
//////		          << "\tambientlight: "       << ptState->ambientlight      << std::endl
//////		          << "\tleft_motor: "         << ptState->left_motor        << std::endl
//////		          << "\tright_motor: "        << ptState->right_motor       << std::endl
//////		          << "\tcolor: "              << ptState->color             << std::endl
//////		          << "\tgradient: "           << std::endl;
//////		   }
////
////		beacon_blue_count = 0;
////		beacon_red_count = 0;
////		m_cOutFile << clock << "	";
////		for (unsigned int i = 0; i < bots.size(); ++i) {
////			CKilobotEntity &kbEntity = *any_cast<CKilobotEntity*>(bots[i]);
////
////			CCI_KilobotController &controller =
////								static_cast<CCI_KilobotController>(kbEntity.GetControllableEntity().GetController());
////						//string state = controller.;
////
//////			  //CKilobotEntity &c_kilobot = *any_cast<CKilobotEntity *>(it->second);
//////					   CCI_KilobotController &c_controller = dynamic_cast<CCI_KilobotController &>(kbEntity.GetControllableEntity().GetController());
//////					            int sharedMemFD = c_controller.CCI_KilobotController();
//////
//////					            kilobot_state_t *robotState;
//////					            /* Resize shared memory area to contain the robot state, filling it with zeros */
//////					            ftruncate(sharedMemFD, sizeof(kilobot_state_t));
//////					            /* Get pointer to shared memory area */
////////					            robotState =
////////					                (kilobot_state_t *)mmap(NULL,
////////					                                        sizeof(kilobot_state_t),
////////					                                        PROT_READ,
////////					                                        MAP_SHARED,
////////					                                        sharedMemFD,
////////					                                        0);
////////					            if (robotState == MAP_FAILED)
////////					            {
////////					                  close(sharedMemFD);
////////					                  exit(1);
////////					            }
//////
//////					            message_t message = robotState->tx_message;
//////
////////			Real Robot_X =
////////					kbEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
////////			Real Robot_Y =
////////					kbEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
//////
////
////
////		     CKilobotCommunicationEntity kilocomm = kbEntity.GetKilobotCommunicationEntity();
////
////
////			int state = 2;
//////			if (kilocomm.GetTxStatus()
//////					== CKilobotCommunicationEntity::TX_SUCCESS) {
//////				state = kilocomm.GetTxMessage()->data[2];
//////			}
//////			message_t* message = kilocomm.GetTxMessage();
//////			state=message->data[2];
////			state = kilocomm.GetTxMessage()->data[2];
////			if (state == 0) {
////				beacon_blue_count += 1;
////			} else if (state == 1) {
////				beacon_red_count += 1;
////			}
////
////			m_cOutFile << i << "id	" <<  state << "state	"
////							<< kilocomm.GetTxMessage() << "	";
////			m_cOutFile << endl;
////
//////			m_cOutFile << "( " << Robot_X << " , " << Robot_Y << " , "
//////					<< ((unsigned int) word) << ") ";
////		}
////
////		m_cOutFile << beacon_blue_count / bots.size() << "	" <<  bots.size()<< "	"
////				<< beacon_red_count / bots.size() << "	";
////		m_cOutFile << endl;
////	}
//}

void CAggregation::PostStep() {
	int clock = GetSpace().GetSimulationClock();
	if (clock % 10 == 0) {

		beacon_blue_count = 0;
		beacon_red_count = 0;
		//m_cOutFile << clock << "	";
		for (unsigned int i = 0; i < bots.size(); ++i) {
			CKilobotEntity &kbEntity = *any_cast<CKilobotEntity*>(bots[i]);
			CKilobotCommunicationEntity kilocomm =
					kbEntity.GetKilobotCommunicationEntity();

			CColor color = bots[i]->GetLEDEquippedEntity().GetLED(0).GetColor();

			//CColor cl;

			CVector2 c_kilobot_xy_position(
					kbEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
					kbEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
			m_cKilobotOriginalPositions.push_back(c_kilobot_xy_position);

			Real fDistanceSpotBlack =
					(m_cCoordBlackSpot - c_kilobot_xy_position).Length();
			Real fDistanceSpotWhite =
					(m_cCoordWhiteSpot - c_kilobot_xy_position).Length();
			if (fDistanceSpotBlack <= m_fRadius && totalSpots >= 2) {

				//color.SetBlue(1);
				LOG << i << "- bluespotJoin:" << c_kilobot_xy_position
						<< std::endl;

				message_t msg = *(new message_t());
				msg.type = NORMAL;
				msg.data[0] = 198;
				msg.data[1] = 198;
				msg.data[2] = 0;
				msg.data[5] = 30;
				msg.crc = 0;				//message_crc(&msg);
				GetSimulator().GetMedium<CKilobotCommunicationMedium>(
						"kilocomm").SendOHCMessageTo(kbEntity, &msg);
//				if (clock % 100 == 0) {
//					beacon_blue_count += 1;
//				}
			} else if (fDistanceSpotWhite <= m_fRadius && totalSpots >= 2) {
				//color.SetBlue(1);
				LOG << i << "- redspotJoin:" << c_kilobot_xy_position
						<< std::endl;

				message_t msg = *(new message_t());
				msg.type = NORMAL;
				msg.data[0] = 199;
				msg.data[1] = 199;
				msg.data[2] = 1;
				msg.data[5] = 60;
				msg.crc = 0;				//message_crc(&msg);
				GetSimulator().GetMedium<CKilobotCommunicationMedium>(
						"kilocomm").SendOHCMessageTo(kbEntity, &msg);
//				if (clock % 100 == 0) {
//					beacon_red_count += 1;
//				}

			} else {
				//LOG << i << "- spotleave:" << c_kilobot_xy_position<< std::endl;
				message_t msg = *(new message_t());
				msg.type = NORMAL;
				msg.data[0] = 197;
				msg.data[1] = 197;
				msg.data[2] = 2;
				msg.data[5] = 70;
				msg.crc = 0;				//message_crc(&msg);
				GetSimulator().GetMedium<CKilobotCommunicationMedium>(
						"kilocomm").SendOHCMessageTo(kbEntity, &msg);

			}

//			Real fDistanceSpotWhite =
//					(m_cCoordWhiteSpot - c_kilobot_xy_position).Length();
//			if (fDistanceSpotWhite <= m_fRadius && totalSpots >= 2) {
//				//color.SetBlue(1);
//				LOG << i << "- redspot:" << c_kilobot_xy_position << std::endl;
//
//				message_t msg = *(new message_t());
//				msg.type = NORMAL;
//				msg.data[0] = 199;
//				msg.data[1] = 199;
//				msg.data[2] = 1;
//				msg.data[5] = 60;
//				msg.crc = 0;				//message_crc(&msg);
//				GetSimulator().GetMedium<CKilobotCommunicationMedium>(
//						"kilocomm").SendOHCMessageTo(kbEntity, &msg);
//
//			}
//			else {
//				LOG << i << "- redspotleave:" << c_kilobot_xy_position << std::endl;
//				message_t msg = *(new message_t());
//				msg.type = NORMAL;
//				msg.data[0] = 199;
//				msg.data[1] = 199;
//				msg.data[2] = 2;
//				msg.data[5] = 60;
//				msg.crc = 0;				//message_crc(&msg);
//				GetSimulator().GetMedium<CKilobotCommunicationMedium>(
//						"kilocomm").SendOHCMessageTo(kbEntity, &msg);
//
//			}
//
//
			if (clock % 100 == 0) {
				LOG << i << "- color:" << color << std::endl;
				if (color.GetGreen() > 0) {

				} else {
					if (color.GetBlue() > 0) {
						beacon_blue_count += 1;
						LOG << "blue:" << beacon_blue_count << std::endl;
					} else if (color.GetRed() > 0) {
						beacon_red_count += 1;
						LOG << "red:" << beacon_red_count << std::endl;
					}
				}
			}
			//kilobot_state_t* ptState = kbEntity->GetRobotState();
			//m_vecKilobotColors
//			GetKilobotLedColor(kbEntity);
//			int state = 2;
//			if (kilocomm.GetTxStatus()
//					== CKilobotCommunicationEntity::TX_SUCCESS) {
//				state = kilocomm.GetTxMessage()->data[2];
//			}
//
//			if (state == 0) {
//				beacon_blue_count += 1;
//			} else if (state == 1) {
//				beacon_red_count += 1;
//			}

		}

		if (clock % 100 == 0) {
			m_cOutFile << clock
			<< "	" << beacon_blue_count / bots.size() << "	"
			<< beacon_red_count / bots.size() << "	";
			m_cOutFile << endl;
		}
	}

}

//void CAggregation::PostStep() {
//   /* Go through the kilobots */
//   for(size_t i = 0; i < m_tKBs.size(); ++i) {
//      /* Create a pointer to the kilobot state */
//      kilobot_state_t* ptState = m_tKBs[i].first->GetRobotState();
//      /* Print current state internal robot state */
//      LOG << m_tKBs[i].first->GetId() << ": "                       << std::endl
//          << "\ttx_state: "           << ptState->tx_state          << std::endl
//          << "\trx_state: "           << ptState->rx_state          << std::endl
//          << "\tambientlight: "       << ptState->ambientlight      << std::endl
//          << "\tleft_motor: "         << ptState->left_motor        << std::endl
//          << "\tright_motor: "        << ptState->right_motor       << std::endl
//          << "\tcolor: "              << ptState->color             << std::endl
//          << "\tgradient: "           << m_tKBs[i].second->gradient << std::endl;
//   }
//}

//void CAggregation::PostStep() {
//    //int recordSteps = 100;
//    int clock = GetSpace().GetSimulationClock();
//    if (clock % 100 == 0)
//        m_cOutFile << clock << " ";
//
//    for(unsigned int i=0; i<bots.size(); ++i) {
//    	CKilobotEntity &kbEntity = *any_cast<CKilobotEntity*>(bots[i]);
//
//    	//int sd=commit_state;
//
//    			CKilobotCommunicationEntity kilocomm =
//    					kbEntity.GetKilobotCommunicationEntity();
//
//    			int state = commit_state;
////    			if (kilocomm.GetTxStatus()
////    					== CKilobotCommunicationEntity::TX_SUCCESS) {
////    				state = kilocomm.GetTxMessage()->data[2];
////    			}
//
////    			if (state == 0) {
////    				beacon_blue_count += 1;
////    			} else if (state == 1) {
////    				beacon_red_count += 1;
////    			}
//
//        int clock = GetSpace().GetSimulationClock();
//        if(clock % 100 == 0) {
//        			m_cOutFile << i << "i	" << state << "	" << beacon_blue_count / bots.size() << "	"
//        					<< beacon_red_count / bots.size() << "	";
//        }
//    }
//
//    if (clock % 100 == 0)
//        m_cOutFile<<endl;
//}

//void CAggregation::PostStep() {
//    int recordSteps = 100;
//    int clock = GetSpace().GetSimulationClock();
//    if(clock%recordSteps==0)
//        m_cOutFile << clock << " ";
//
//    for(unsigned int i=0; i<bots.size(); ++i) {
//        CKilobotEntity& kbEntity = *any_cast<CKilobotEntity*>(bots[i]);
//        CKilobotCommunicationEntity kilocomm = kbEntity.GetKilobotCommunicationEntity();
//        if(kilocomm.GetTxStatus()==CKilobotCommunicationEntity::TX_SUCCESS)
//            stayArray[i] = kilocomm.GetTxMessage()->data[0]!=0;
//
//        int clock = GetSpace().GetSimulationClock();
//        if(clock%recordSteps==0) {
//            Real Robot_X = kbEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
//            Real Robot_Y = kbEntity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
//            m_cOutFile << "( " << Robot_X << " , " << Robot_Y << " , " << stayArray[i] << ") ";
//        }
//    }
//
//    if(clock%recordSteps==0)
//        m_cOutFile<<endl;
//}

/****************************************/
/****************************************/

argos::CColor CAggregation::GetFloorColor(
		const argos::CVector2 &c_position_on_plane) {

	if (totalSpots == 2) {
//2 spots
		CVector2 vCurrentPoint(c_position_on_plane.GetX(),
				c_position_on_plane.GetY());
		Real d = (m_cCoordBlackSpot - vCurrentPoint).Length();
		if (d <= m_fRadius) {
			return CColor::BLACK;
		}
		d = (vCurrentPoint - m_cCoordWhiteSpot).Length();
		if (d <= m_fRadius) {
			return CColor::WHITE;
		}

	} else if (totalSpots == 3) {
//3 spots
		CVector2 vCurrentPoint(c_position_on_plane.GetX(),
				c_position_on_plane.GetY());
		Real d = (m_cCoordBlackSpot - vCurrentPoint).Length();
		if (d <= m_fRadius) {
			return CColor::BLACK;
		}
		d = (vCurrentPoint - m_cCoordWhiteSpot).Length();
		if (d <= m_fRadius) {
			return CColor::WHITE;
		}
		d = (vCurrentPoint - m_cCoordWhiteSpot2).Length();
		if (d <= (m_fRadius)) {
			return CColor::WHITE;
		}

	} else if (totalSpots == 4) {

//// 4 spots
		CVector2 vCurrentPoint(c_position_on_plane.GetX(),
				c_position_on_plane.GetY());
		Real d = (m_cCoordBlackSpot - vCurrentPoint).Length();
		if (d <= m_fRadius) {
			return CColor::BLACK;
		}
		d = (m_cCoordBlackSpot2 - vCurrentPoint).Length();
		if (d <= m_fRadius) {
			return CColor::BLACK;
		}
		d = (vCurrentPoint - m_cCoordWhiteSpot).Length();
		if (d <= m_fRadius) {
			return CColor::WHITE;
		}
		d = (vCurrentPoint - m_cCoordWhiteSpot2).Length();
		if (d <= (m_fRadius)) {
			return CColor::WHITE;
		}
	}

	return CColor::WHITE;
}

/* Register this loop functions into the ARGoS plugin system */
REGISTER_LOOP_FUNCTIONS(CAggregation, "aggregation_loop_functions");
