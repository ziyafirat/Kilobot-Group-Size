#include <kilolib.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
//#include "aggregation.h"
//#include <debug.h>

// run
//mkdir build
//cd build/
//cmake -DCMAKE_BUILD_TYPE=Release ..
//make
// argos3 -c aggregation/experiments/kilobot_aggregation.argos

#define PI_TURN 26
#define STRAIGHT 500
#define RHO 0.6
#define PI 3.14159265358979323846
#define TWO_PI 2.0*PI

#define COMPLETE_TURN 70
#define TOO_CLOSE_DISTANCE 500
#define DESIRED_DISTANCE 60
#define PROB_DISTANCE 60

#define BEACON_BLUE 30
#define BEACON_RED 60
#define BEACON_WALL 90
#define NON_INFORMED 0
#define LEAVE_TIMESTEP 32
#define STAY_TIMESTEP 32


//debug_info_t dddsds;
// Track the number of nearest neighbors
int informed_size_blue_beacon = 2 ;
int informed_size_red_beacon = 18;
int total_informed_size = 20;
int goNearBeacon = 2000;
int N_neighbors = 0;

int next_turn, turn_turn;
char walk;

message_t message;
unsigned char sending_turn;

uint8_t ids[100];
uint8_t msgs[100];
int msgs_size;
int agg_site;
int walls_flag;
unsigned char mes_from;
unsigned char mes_payload;
unsigned char rec_quality[2];


char motion_state;
int new_message;
int commit_state;
char flag = 0;
int neighbours_size_while_joining;

int distance;
int distance_beacon;
int neighbor_commit_state;
int distance_flag = 0;
int message_sent = 0;
int leave_flag = 0;
int stay_flag = 0;
int flag_join_n = 0;
int flag_leave_n = 0;
int flag_wall_count = 0;
int t;
long int cur_time, motion_prev_time = -401;

int turn_side;
int turn_time;
char motion_state;
int count_timer_log;




// Declare a structure to represent each neighbor
typedef struct {
	int id;
	int distance;
	uint32_t timestamp;
} neighbor_t;

// Create list of neighbors
neighbor_t neighbors[30];

enum commitment {
	Cb, Cr, uncommited
};
/* Enum for different motion types */
typedef enum {
	FORWARD = 0, TURN_LEFT = 1, TURN_RIGHT = 2, STOP = 3,
} motion_t;

/* Enum for boolean flags */
typedef enum {
	false = 0, true = 1,
} bool;

// /* Enum for the robot states */
// typedef enum {
//     OUTSIDE_CLUSTERING_HUB = 0,
//     INSIDE_CLUSTERING_HUB = 1,
// } action_t;

/* current motion type */
motion_t current_motion_type = STOP;

/* current state */
//action_t current_state = OUTSIDE_CLUSTERING_HUB;
/* counters for motion, turning and random_walk */
uint32_t last_turn_ticks = 0;
uint32_t turn_ticks = 60;

unsigned int turning_ticks = 0;
const uint8_t max_turning_ticks = 250; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint16_t max_straight_ticks = 500; /* set the \tau_m period to 2.5 s: n_m = \tau_m/\delta_t = 2.5/(1/32) */
uint32_t last_motion_ticks = 0;
uint32_t turn_into_random_walker_ticks = 160; /* timestep to wait without any direction message before turning into random_walker */
uint32_t last_direction_msg = 0;
uint32_t last_state_ticks = 0;
// Removes neighbors who are no longer in contact
void purge(void) {
	int8_t i;

	for (i = N_neighbors; i >= 0; i--) {

		if (kilo_ticks - neighbors[i].timestamp > 128) //32 ticks = 1 s
				{
			//this one is too old.
			neighbors[i] = neighbors[N_neighbors - 1];
			//replace it by the last entry
			N_neighbors--;
		}
	}
}

//void write_coords(char *header, int kilo_uid, int id, int N_neighbors,
//		int neighbours_size_while_joining, double exp1, double rl,
//		double Res1, int idx, int commit_state, int walk, int distance,
//		int kilo_ticks) {
//
//	printf(
//			"%s kilo_uid:%i  id:%i  Neighbors:%i N_join:%i  exp1:%f  rl:%f  Res1:%f idx:%i state:%i walk:%i  distance:%i  ticks:%i \n",
//			header, kilo_uid, id, N_neighbors, neighbours_size_while_joining,
//			exp1, rl, Res1, idx, commit_state, walk, distance, kilo_ticks);
//	FILE *fPointer_GOD;
//	char str_GOD[100];
//	sprintf(str_GOD, "datalog/robotid_%i.csv", kilo_uid);
//	fPointer_GOD = fopen(str_GOD, "a");
//	fprintf(fPointer_GOD,
//			"%s kilo_uid:%i  id:%i  Neighbors:%i N_join:%i  exp1:%f  rl:%f  Res1:%f idx:%i state:%i walk:%i  distance:%i  ticks:%i \n",
//			header, kilo_uid, id, N_neighbors, neighbours_size_while_joining,
//			exp1, rl, Res1, idx, commit_state, walk, distance, kilo_ticks);
//
//	fclose(fPointer_GOD);
//}
//
//void write_coords_timer(char *header, int kilo_uid,  int count, int commit_state, int kilo_ticks) {
//
////	printf(
////			"%s kilo_uid:%i  id:%i  Neighbors:%i N_join:%i  exp1:%f  rl:%f  Res1:%f idx:%i state:%i walk:%i  distance:%i  ticks:%i \n",
////			header, kilo_uid, id, N_neighbors, neighbours_size_while_joining,
////			exp1, rl, Res1, idx, commit_state, walk, distance, kilo_ticks);
//	FILE *fPointer_GOD;
//	char str_GOD[100];
//	sprintf(str_GOD, "data/robotid_%i_%i_%i.csv", informed_size_blue_beacon,informed_size_red_beacon,kilo_uid);
//	fPointer_GOD = fopen(str_GOD, "a");
//	fprintf(fPointer_GOD,
//			"%i	%i	%i	%i \n",
//			kilo_uid, count, commit_state, kilo_ticks);
//
//	fclose(fPointer_GOD);
//}


/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion(motion_t new_motion_type) {
	bool calibrated = true;
	if (current_motion_type != new_motion_type) {
		switch (new_motion_type) {
		case FORWARD:
			spinup_motors();
			if (calibrated)
				set_motors(kilo_straight_left, kilo_straight_right);
			else
				set_motors(70, 70);
			break;
		case TURN_LEFT:
			spinup_motors();
			if (calibrated)
				set_motors(kilo_turn_left, 0);
			else
				set_motors(70, 0);
			break;
		case TURN_RIGHT:
			spinup_motors();
			if (calibrated)
				set_motors(0, kilo_turn_right);
			else
				set_motors(0, 70);
			break;
		case STOP:
		default:
			set_motors(0, 0);
		}
		current_motion_type = new_motion_type;
	}
}

void setup() {

	srand(rand_hard());
	sending_turn = 0;
	commit_state = uncommited;
	mes_payload = 0;
	agg_site = 0;
	motion_state = 1;
	msgs_size = 0;
	N_neighbors = 0;

	count_timer_log=0;
	message.type = NORMAL;
// 	if (kilo_uid < informed_size_blue_beacon) { // blue informed

// 		message.data[0] = kilo_uid;
// 		message.data[1] = kilo_uid;
// 		message.data[2] = commit_state; // 0;
// 		message.data[5] = BEACON_BLUE;
// 		message.data[6] = Cb;
// 	} else if (kilo_uid >= informed_size_blue_beacon
// 			&& kilo_uid < total_informed_size) {  // red informed

// 		message.data[0] = kilo_uid;
// 		message.data[1] = kilo_uid;
// 		message.data[2] = commit_state; // 0;
// 		message.data[5] = BEACON_RED;
// 		message.data[6] = Cr;
// 	} else {    // non-informed

	message.data[0] = kilo_uid;
	message.data[1] = kilo_uid;
	message.data[2] = commit_state; // 0;

	message.data[5] = NON_INFORMED;
	//message.data[6] = uncommited;

	//}
	message.crc = message_crc(&message);

	rand_seed(rand_hard());

	walk = 1;
	next_turn = STRAIGHT;
	turn_turn = COMPLETE_TURN; //flag value
	//t = rand_soft() % TIMESTEP;
	/* Initialise motion variables */
	last_motion_ticks = rand() % max_straight_ticks;
	last_state_ticks = 0;
	set_motion(FORWARD);

}

double arccos(double x) {
	const double pi = 3.141592653;
	return pi / 2
			- (.5689111419 - .2644381021 * x
					- .4212611542 * (2 * x - 1) * (2 * x - 1)
					+ .1475622352 * (2 * x - 1) * (2 * x - 1) * (2 * x - 1))
					/ (2.006022274 - 2.343685222 * x
							+ .3316406750 * (2 * x - 1) * (2 * x - 1)
							+ .02607135626 * (2 * x - 1) * (2 * x - 1)
									* (2 * x - 1));
}

int random_turn2(double ran) {
	double c = ((2.0 * RHO) / (1.0 + ( RHO * RHO)));
	double V = cos(ran * TWO_PI);
	double Vc = V + c;
	double cV = 1.0 + (c * V);
	double VccV = Vc / cV;
	double x = VccV; //1.0;//(V + c) / (1.0 + (c * V));
	double sigma = arccos(x);

	int tt = (int) (max_turning_ticks * sigma) / PI;
	return tt;
}

void led_colour(char x) {

	if (x == 0) //Cb
		set_color(RGB(0, 0, 1)); //blue
	else if (x == 1) //Cr
		set_color(RGB(1, 0, 0)); //red
	else if (x == 2) //uncommited
			{
		//set_color(RGB(0, 1, 0)); //green

		if (kilo_uid < informed_size_blue_beacon) {
			set_color(RGB(1, 1, 1)); // White
		} else if (kilo_uid >= informed_size_blue_beacon
				&& kilo_uid < total_informed_size) {
			set_color(RGB(1, 1, 0)); // Yellow
		} else {
			set_color(RGB(0, 1, 0)); //green
		}
	}

}

void stay() {
	if (kilo_uid < total_informed_size) {
		message.data[2] = commit_state;
		message.crc = message_crc(&message);
	}
	float p = 1;
	float r = (float) rand_soft() / (float) 255;
	if (r < p) {
		walk = 0;
		neighbours_size_while_joining = N_neighbors;
		leave_flag = 0;
		flag_join_n = 0;
		stay_flag = goNearBeacon;
	}
}

void leave() {
	if (flag_join_n < 10) { // physical kilobot 10 seconds
		neighbours_size_while_joining = N_neighbors;
		flag_join_n++;

// 		write_coords("Joined ", kilo_uid, 0, N_neighbors,
// 				neighbours_size_while_joining, 0, 0, 0, 0, commit_state, walk,
// 				distance, kilo_ticks);
	} else {

		if (flag_join_n < 20) { //physical kilobot 20 seconds
			flag_join_n++;

		} else {
			flag_join_n = 10; // physical kilobot 10 seconds

			double p = 0;
			int abs1 = abs(N_neighbors - neighbours_size_while_joining);
			int k = 18;
			double exp1 = -3.6 * (k - abs1);
			double Res1 = (double) exp(exp1);
			//if (Res1 < 1 && Res1 > 0) {
			if (kilo_uid >= total_informed_size) { // if robot non informed robot

				if (N_neighbors > 0) {

					p = Res1; // Min(one, Res1);
				} else {
					p = 1;
				}

			} else {

				p = Res1; // Min(one, Res1);
			}

// 		double rl = ((double) ((rand_soft() << 8) + rand_soft()))
// 				/ (double) 65535; //generates random number on 16 bits for accuracy
			double rl = (double) rand_soft() / (double) 255;

//			write_coords("c_leave ", kilo_uid, 0, N_neighbors,
//					neighbours_size_while_joining, exp1, rl, p, 0, commit_state,
//					walk, distance, kilo_ticks);

			if (rl < p) { //leave

				walk = 1;

				commit_state = uncommited;
				mes_payload = 0;
				agg_site = 0;
				if (kilo_uid < total_informed_size) {
					message.data[2] = commit_state;
					message.crc = message_crc(&message);
				}
				set_motion(FORWARD);
				leave_flag = 100;
				msgs_size = 0;
				N_neighbors = 0;
				flag_join_n = 0;

//				write_coords("leaving.. ", kilo_uid, 0, N_neighbors,
//								neighbours_size_while_joining, exp1, rl, p, 0, commit_state,
//								walk, distance, kilo_ticks);

			}

//			write_coords("c_leave2 ", kilo_uid, 0, N_neighbors,
//							neighbours_size_while_joining, exp1, rl, p, 0, commit_state,
//							walk, distance, kilo_ticks);

			msgs_size = 0;
			N_neighbors = 0;
		}
	}

}

/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk() {
	switch (current_motion_type) {
	case TURN_LEFT:
		if (kilo_ticks > last_motion_ticks + turning_ticks) {
			/* start moving forward */
			last_motion_ticks = kilo_ticks;  // fixed time FORWARD
			//	last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
			set_motion(FORWARD);
		}
		break;
	case TURN_RIGHT:
		if (kilo_ticks > last_motion_ticks + turning_ticks) {
			/* start moving forward */
			last_motion_ticks = kilo_ticks;  // fixed time FORWARD
			//	last_motion_ticks = rand() % max_straight_ticks + 1;  // random time FORWARD
			set_motion(FORWARD);
		}
		break;
	case FORWARD:

		if (walls_flag == 90 && flag_wall_count < 1) {
			last_motion_ticks = kilo_ticks;
			turn_time = 160;
			set_motion(TURN_LEFT);

			walls_flag = 0;
			flag_wall_count = 200;
			turning_ticks = turn_time;

		} else if (kilo_ticks > last_motion_ticks + max_straight_ticks) {
			/* perform a radnom turn */
			last_motion_ticks = kilo_ticks;

			double ran = rand() / (double) RAND_MAX;
			turn_time = random_turn2(ran);
			if (ran > 0.5) {
				set_motion(TURN_LEFT);
			} else {
				set_motion(TURN_RIGHT);
			}
			turning_ticks = turn_time;		// rand() % max_turning_ticks + 1;
		} else if (distance > 0 && distance < DESIRED_DISTANCE) {

			/* perform a radnom turn */
//			write_coords("turn ", kilo_uid, 0, N_neighbors,
//					neighbours_size_while_joining, 0, 0, 0, 0, commit_state,
//					walk, distance, kilo_ticks);
			last_motion_ticks = kilo_ticks;
			set_motors(0, 0);
			double ran = rand() / (double) RAND_MAX;
			turn_time = random_turn2(ran);
			if (ran > 0.5) {
				set_motion(TURN_LEFT);
			} else {
				set_motion(TURN_RIGHT);
			}
			turning_ticks = turn_time;

		}
		break;
	case STOP:
	default:
		set_motion(STOP);
	}
}
void loop() {

	if (kilo_ticks % 100 == 0)
	{

		count_timer_log=count_timer_log+1;
		//message.data[2] = commit_state;
		//write_coords_timer("timer ", kilo_uid, count_timer_log, commit_state, kilo_ticks);
	}
	if (flag_wall_count > 0) { //wall flag
		--flag_wall_count;
		//set_color(RGB(0, 0, 1));
		// delay(100);
		//set_color(RGB(0, 0, 0));
		set_color(RGB(1, 1, 1));
		random_walk();

	} else {
		if (stay_flag > 0) {

//			write_coords("stay_flag1 ", kilo_uid, 0, N_neighbors, stay_flag, 0,
//					0, 0, 0, commit_state, walk, distance, kilo_ticks);
			random_walk();
			--stay_flag;
			if (new_message) {
				new_message = 0;

				if (distance>0 && distance < DESIRED_DISTANCE
						&& neighbor_commit_state < uncommited) {
//					write_coords("stay_flag2 ", kilo_uid, 0, N_neighbors,
//							stay_flag, 0, 0, 0, 0, commit_state, walk, distance,
//							kilo_ticks);
					stay_flag = 0;
				}

//				write_coords("stay_flag ", kilo_uid, 0, N_neighbors, stay_flag,
//						0, 0, 0, 0, commit_state, walk, distance, kilo_ticks);

				if (stay_flag <= 2) {
					if (distance_beacon>0 && distance_beacon < TOO_CLOSE_DISTANCE) {
						stay_flag = 0;
					} else {
						stay_flag = 0;
						walk = 1;

						commit_state = uncommited;
						mes_payload = 0;
						agg_site = 0;
						if (kilo_uid < total_informed_size) {
							message.data[2] = commit_state;
							message.crc = message_crc(&message);
						}
						//set_motion(FORWARD);
						random_walk();
						msgs_size = 0;
						//N_neighbors = 0;
						flag_join_n = 0;
					}
				}
			}
		} else if (walk) {
			//RANDOM WALK

			random_walk();

			if (leave_flag <= 0) {
				if (new_message) {
					new_message = 0;
					//rec_quality[mes_from] = mes_payload;
					flag = 1;
				}

				/*-----When uncommitted------*/
				if (commit_state == uncommited) {

					if (kilo_ticks > last_state_ticks + STAY_TIMESTEP) {
						last_state_ticks = kilo_ticks;
						if (flag) {
							//	if (distance < TOO_CLOSE_DISTANCE) {

							flag = 0;
							if (agg_site == 30) { //rec_quality[mes_from] == 30) { // blue
								if (kilo_uid < informed_size_blue_beacon
										|| kilo_uid >= total_informed_size) {
									commit_state = Cb; // Stay in blue beacon

//									write_coords("BlueBeacon ", kilo_uid, 0,
//											N_neighbors,
//											neighbours_size_while_joining, 0, 0,
//											0, 0, commit_state, walk, distance,
//											kilo_ticks);
									stay();
								} else {
									walk = 1;
								}
							} else if (agg_site == 60) {
								//rec_quality[mes_from] == 60) { // red
								if (kilo_uid >= informed_size_blue_beacon) {
									commit_state = Cr; // Stay in red beacon
//									write_coords("RedBeacon ", kilo_uid, 0,
//											N_neighbors,
//											neighbours_size_while_joining, 0, 0,
//											0, 0, commit_state, walk, distance,
//											kilo_ticks);
									stay();
								} else {
									walk = 1;
								}
							} else {
								walk = 1;
							}
						}
					}

				}

			} else {
				--leave_flag;
			}
		} else //STAY
		{
			set_motion(STOP);
			if (kilo_ticks > last_state_ticks + LEAVE_TIMESTEP) { //32 clocktick= 1 second
				last_state_ticks = kilo_ticks;
				leave();
			}
		}
		led_colour(commit_state);
		//delay(100);
	}

}


message_t* message_tx() {
	return &message;
}

short int idxOf(uint8_t *ids, uint8_t id, int ids_size) {
	short int i;
	for (i = 0; i < ids_size; ++i) {
		if (ids[i] == id) {
			break;
		}
	}
	return i;
}

void message_rx(message_t *m, distance_measurement_t *d) {

	distance = estimate_distance(d);
	int id = m->data[1];
	walls_flag = m->data[5];
	neighbor_commit_state = m->data[2];
//	if (m->data[5] == BEACON_BLUE || m->data[5] == BEACON_RED) {
	if (id < total_informed_size) {
		if (commit_state < uncommited && distance < PROB_DISTANCE) {

			if (m->data[2] == commit_state) {

				short int idx = idxOf(ids, id, msgs_size);
				ids[idx] = id;
				if (idx >= msgs_size) {
					++msgs_size;
					N_neighbors = msgs_size;

				}
//				write_coords("++ Neigh ", kilo_uid, id, N_neighbors, 0, 0, 0, 0,
//						idx, commit_state, walk, distance, kilo_ticks);

			}
		}
	}
//	else {
//		write_coords("NO Neigh ", kilo_uid, id, N_neighbors, 0, 0, 0, 0, 0,
//				commit_state, walk, distance, kilo_ticks);
//	}
	if (m->data[5] == BEACON_BLUE || m->data[5] == BEACON_RED) {

		if (m->data[2] < uncommited) {
			if (distance < TOO_CLOSE_DISTANCE) {
				mes_from = m->data[6];
				agg_site = m->data[5];
				mes_payload = m->data[5];
				new_message = 1;
				distance_beacon = estimate_distance(d);

			}
		}

//		write_coords("data5 ", kilo_uid, id, N_neighbors, 0, 0, 0, 0,
//				distance_beacon, commit_state, walk, distance, kilo_ticks);

	}

}

void message_tx_succes() {
	message_sent = 1;
}

int main() {
	kilo_init();
	kilo_message_rx = message_rx;
	kilo_message_tx = message_tx;
	kilo_message_tx_success = message_tx_succes;
	kilo_start(setup, loop);

	return 0;
}
