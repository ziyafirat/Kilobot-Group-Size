#include <kilolib.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// run
//mkdir build
//cd build/
//cmake -DCMAKE_BUILD_TYPE=Release ..
//make
// argos3 -c aggregation/experiments/kilobot_aggregation.argos

#define HEAR_TIME 8

#define PI_TURN 26
#define STRAIGHT 200
#define RHO 0.6
#define PI 3.14159265358979323846
#define TWO_PI 2.0*PI

#define COMPLETE_TURN 70
#define TOO_CLOSE_DISTANCE 50
#define DESIRED_DISTANCE 50

// Track the number of nearest neighbors
int N_neighbors = 0;

int next_turn, turn_turn;
char walk;

message_t message;
unsigned char sending_turn;



uint8_t ids[100];
uint8_t msgs[100];
int  msgs_size;

unsigned char mes_from;
unsigned char mes_payload;
unsigned char rec_quality[2];

int new_message;
int commit_state;
char flag = 0;
int neighbours_size_while_joining;
int informed_size_blue_beacon = 4;
int informed_size_red_beacon = 4;
int total_informed_size = 8;
int distance;
int distance_flag = 0;
int message_sent = 0;
int leave_flag = 0;
int flag_join_n = 0;

int turn_side;
int turn_time;
char motion_state;

enum commitment {
	Cb, Cr, uncommited
};


void setup() {

	srand(rand_hard());
	sending_turn = 0;
	commit_state = uncommited;

	msgs_size = 0;
	N_neighbors = 0;

	message.data[0] = kilo_uid;
	message.data[1] = kilo_uid;
	message.data[2] = commit_state; // 0;

	message.data[7] = kilo_uid;
	message.data[8] = commit_state;
	message.crc = message_crc(&message);
	rand_seed(rand_hard());

	walk = 1;
	next_turn = STRAIGHT;
	turn_turn = COMPLETE_TURN; //flag value

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
//	double c = (double) ((2.0 * RHO) / (1.0 + (RHO * RHO)));
//	double V = (double) cos(ran * TWO_PI);
//	double x = (V + c) / (1 + (c * V));
//	double sigma = acos(x); //% [0, PI];
//	printf("sigma sigma %f  x %f kilo_ticks %i robid %i \n", sigma, x,
//			kilo_ticks, kilo_uid);
//	double sigma3 = arccos(x); //% [0, PI];
//	printf("sigma3 sigma3 %f  x %f kilo_ticks %i robid %i \n", sigma3, x,
//			kilo_ticks, kilo_uid);
//	return (int) (COMPLETE_TURN * sigma) / PI;

	double c = ((2.0 * RHO) / (1.0 + ( RHO * RHO)));
	double V = cos(ran * TWO_PI);
	double Vc = V + c;
	double cV = 1.0 + (c * V);
	double VccV = Vc / cV;
	double x = VccV; //1.0;//(V + c) / (1.0 + (c * V));
	double sigma = arccos(x);

	int tt = (int) (COMPLETE_TURN * sigma) / PI;
	printf("sigma3 sigma3 %f  x %f tt %i kilo_ticks %i robid %i \n", sigma, x,
			tt, kilo_ticks, kilo_uid);
	return tt;
}

void led_colour(char x) {
	if (x == 0) //Cb
		set_color(RGB(0, 0, 1)); //blue
	else if (x == 1) //Cr
		set_color(RGB(1, 0, 0)); //red
	else if (x == 2) //uncommited
		set_color(RGB(0, 1, 0)); //green
}

void stay() {
	message.data[2] = commit_state;
	message.crc = message_crc(&message);

	float p = 1;
	float r = (float) rand_soft() / (float) 255;
	if (r < p) {
		walk = 0;
		neighbours_size_while_joining = N_neighbors;
		leave_flag = 0;
		flag_join_n = 0;
		printf("stay Neighbor %i  walk %i kilo_ticks %i robid %i \n",
				N_neighbors, walk, kilo_ticks, kilo_uid);
	}
}

void leave() {
	if (flag_join_n < 10) {
		neighbours_size_while_joining = N_neighbors;
		flag_join_n++;
		printf("Joined Neighbor %i  walk %i kilo_ticks %i robid %i \n",
				N_neighbors, walk, kilo_ticks, kilo_uid);
	} else {

		double p = 0; // exp(-b * n);

		double Res1 = exp(
				-2.2 * (10 - abs(N_neighbors - neighbours_size_while_joining)));
		if (kilo_uid >= total_informed_size) { // if robot non informed robot

			if (N_neighbors > 0) {

				p = Res1; // Min(one, Res1);
			} else {
				p = 1;
			}
			double rl = (double) rand_soft() / (double) 255;
			if (rl < p) { //leave
				walk = 1;
				commit_state = uncommited;
				message.data[2] = commit_state;
				message.crc = message_crc(&message);
				leave_flag = 10;
				msgs_size = 0;
				N_neighbors = 0;
				flag_join_n = 0;
				printf(
						"leave Neighbor %i n_join %i walk %i p %f  rl %f robid %i \n",
						N_neighbors, neighbours_size_while_joining, walk, p, rl,
						kilo_uid);
			}

		} else {

			p = Res1; // Min(one, Res1);
		}
	}
}

void turn_k() {
	if (turn_turn == COMPLETE_TURN) {
		double ran = rand() / (double) RAND_MAX;
		turn_turn = random_turn2(ran);
		printf("turn_turn turn_turn %i robid %i \n", turn_turn, kilo_uid);
		if (ran < 0.5)
			turn_side = 0;// turn right
		else
			turn_side = 1;// turn left
	} else if (turn_turn <= 0) {
		turn_turn = COMPLETE_TURN; //reset
		next_turn = STRAIGHT;
	}

	if (turn_turn < COMPLETE_TURN) {
		spinup_motors(); // Spinup the motors to overcome friction.
		if (turn_side > 0) {
			set_motors(0, kilo_turn_right);
		} else {
			set_motors(kilo_turn_left, 0);
		}

		--turn_turn;
	}
	set_color(RGB(3, 0, 0));
}
void loop() {
	if (walk) {
		//RANDOM WALK

		set_color(RGB(0, 0, 3));

		--next_turn;

		if (distance > 0 && distance < DESIRED_DISTANCE && distance_flag == 0) {

			distance_flag = 1;
			turn_turn = COMPLETE_TURN;
			next_turn = 0;
			turn_k();

		} else if (next_turn <= 0) {

			turn_k();

		} else {
			distance_flag = 0;
			spinup_motors(); // Spinup the motors to overcome friction.
			set_motors(kilo_straight_left, kilo_straight_right);

		}

		if (new_message) {
			new_message = 0;
			rec_quality[mes_from] = mes_payload;
			flag = 1;
		}

		/*-----When uncommitted------*/
		if (commit_state == uncommited) {
			if (kilo_ticks % HEAR_TIME == 0) {

				if (flag) {
					if (distance < TOO_CLOSE_DISTANCE) {
						if (leave_flag <= 0) {

							flag = 0;
							if (rec_quality[mes_from] == 30) { // blue
								if (kilo_uid < informed_size_blue_beacon
										|| kilo_uid >= total_informed_size) {
									commit_state = Cb; // Stay in blue beacon
									stay();
								} else {
									walk = 1;
								}
							} else if (rec_quality[mes_from] == 60) { // red
								if (kilo_uid >= informed_size_blue_beacon) {
									commit_state = Cr; // Stay in red beacon
									stay();
								} else {
									walk = 1;
								}
							} else {
								walk = 1;
							}
						}
					} else {
						--leave_flag;
					}
				}
			}
		}

	} else //STAY
	{
		set_motors(0, 0);
		if (kilo_ticks % HEAR_TIME == 0) {
			leave();
		}
	}
	led_colour(commit_state);
	message.data[2] = commit_state;
	message.crc = message_crc(&message);
}

message_t* message_tx() {
	return &message;
}

short int idxOf(uint8_t *ids, uint8_t id, int ids_size) {
	short int i;
	for (i = 0; i < ids_size; ++i) {
		if (ids[i] == id)
			break;
	}
	return i;
}

void message_rx(message_t *m, distance_measurement_t *d) {

	distance = estimate_distance(d);

	if (m->data[2] == commit_state && commit_state < uncommited) {

		short int idx = idxOf(ids, m->data[1], msgs_size);
		msgs[idx] = m->data[2];
		ids[idx] = m->data[1];
		if (idx >= msgs_size) {
			++msgs_size;
			N_neighbors = msgs_size;
			printf(
					"msgs_size %i idx %i commit_state %i  distance %i kilo_ticks %i robid %i \n",
					msgs_size, idx, commit_state, distance, kilo_ticks,
					kilo_uid);
		}
	} else if (m->data[2] == uncommited) {
		short int idx = idxOf(ids, m->data[1], msgs_size);
		msgs[idx] = m->data[2];
		ids[idx] = m->data[1];
		if (idx < msgs_size) {
			--msgs_size;
			N_neighbors = msgs_size;
			printf(
					"msgs_size %i idx %i commit_state %i  distance %i kilo_ticks %i robid %i \n",
					msgs_size, idx, commit_state, distance, kilo_ticks,
					kilo_uid);
		}
	}

	if (m->data[5] == 30 || m->data[5] == 60) {
		mes_from = m->data[6];
		mes_payload = m->data[5];
		new_message = 1;
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
