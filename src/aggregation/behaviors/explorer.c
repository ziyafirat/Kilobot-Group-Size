#include <kilolib.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

// run
//mkdir build
//cd build/
//cmake -DCMAKE_BUILD_TYPE=Release ..
//make
// argos3 -c aggregation/experiments/exp_aggregation.argos


#define pa1 5
#define pa2 5

#define PI_TURN 26
#define STRAIGHT 10
#define RHO 0.6
#define PI 3.14159265358979323846
#define TWO_PI 2.0*PI
#define COMPLETE_TURN 200

#define TOO_CLOSE_DISTANCE 40
#define DESIRED_DISTANCE 80

// Declare a structure to represent each neighbor
typedef struct {
	int id;
	int distance;
	uint32_t timestamp;
} neighbor_t;

long int cur_time, motion_prev_time = -401, commit_prev_time, send_prev_time;
char motion_state;
int commit_state;

unsigned char mes_from;
unsigned char mes_payload;
unsigned char rec_quality[2];
char flag = 0;
int randvar;
int new_message;
int turn_time;
int x;
int message_sent = 0;
char com[2] = { 30, 60 };
int spot_flag = 0;

char blue = 0;
char red = 1;

int next_turn, turn_turn;
char walk;

int numOfTimeStepTurning;

int distance;
unsigned spot_state;

message_t message;

// Create list of neighbors
neighbor_t neighbors[30];

// Track the number of nearest neighbors
int N_neighbors = 0;

int random_turn(double);

enum commitment{Cb, Cr ,uncommited};

// Removes neighbors who are no longer in contact
void purge(void) {
	int8_t i;

	for (i = N_neighbors - 1; i >= 0; i--) {

		if (kilo_ticks - neighbors[i].timestamp > 64) //32 ticks = 1 s
				{
			//this one is too old.
			neighbors[i] = neighbors[N_neighbors - 1];
			//replace it by the last entry
			N_neighbors--;
		}
	}
}

void led_colour(char x) {
	if (x == 0) //Ci
		set_color(RGB(0, 0, 1)); //blue
	else if (x == 1) //Cj
		set_color(RGB(1, 0, 0)); //red
	else if (x == 2) //uncommited
		set_color(RGB(0, 1, 0)); //green
}

//uint8_t noise(uint8_t w)
//{
//    uint8_t i;
//    for(i=0; i<8; ++i)
//    {
//        double r = (double) rand_soft()/(double) 256;
//        if(r < m)
//            w = w ^ (uint8_t) pow(2,i);
//    }
//    return w;
//}

//void hear() {
//	n = 0;
//	int i;
//
//	/*int scores[100];
//	 for(i=0; i<100; ++i)
//	 scores[i] = 0;*/
//
//	for (i = 0; i < msgs_size; ++i) {
//		uint8_t inside = 0;
//		uint8_t w = msgs[i];
//		if (link == EVO_LINK)
//			w = noise(w);
//
//		int j = 0;
//		while (j < inventory_size && !inside) {
//			if (inventory[j] == w) {
//				//++scores[j];
//				inside = 1;
//			}
//			++j;
//		}
//
//		if (inside) {
//			n += 1;
//			inventory[0] = w;
//			inventory_size = 1;
//		} else {
//			n = 0;
//			inventory[inventory_size] = w;
//			++inventory_size;
//		}
//	}
//	/*int best = 0;
//	 int best_idx;
//	 for(i=0; i<inventory_size; ++i) {
//	 if(scores[i] > best) {
//	 best = scores[i];
//	 best_idx = i;
//	 }
//	 }
//	 if(best>0) {
//	 inventory[0] = inventory[best_idx];
//	 inventory_size = 1;
//	 }
//	 n = best;*/
//
//	//if(link==NO_LINK || 1)
//	n = msgs_size; //this line erases previous operation in order to perform regular aggregation
//	msgs_size = 0; //delete messages;
//}

// receive message callback
void message_rx(message_t *message,
		distance_measurement_t *distance_measurement) {
	// Set flag on message reception.
	new_message = 1;

	mes_from = message->data[0];
	mes_payload = message->data[1];
	int id = message->data[0];
	distance = estimate_distance(distance_measurement);

	if (!walk) {
		int i = 0;
		// Check to see if the id is in the list of neighbors
		for (i = 0; i < N_neighbors; i++) {
			printf("Neighbor Count %i id %i time %i  ival %i robid %i \n",
					N_neighbors, neighbors[i].id, neighbors[i].timestamp, i,
					kilo_uid);
			if (neighbors[i].id == id) { // found it
				neighbors[i].distance = distance;
				neighbors[i].timestamp = kilo_ticks;
				N_neighbors++;
				break;
			}
		}

//		if (N_neighbors < 50) { // if we have too many neighbors,
//			N_neighbors++;
//			// i now points to where this message should be stored
//			neighbors[i].id = id;
//			neighbors[i].distance = distance;
//			neighbors[i].timestamp = kilo_ticks;
//			printf("many Neighbor Count %i id %i time %i  ival %i robid %i \n",
//					N_neighbors, neighbors[i].id, neighbors[i].timestamp, i,
//					kilo_uid);
//		}
	}

}

void setup() {
	//printf("Hello World\n");
	//printf("%d %d %d\n", Ci, Cj, uncommited);
	srand(rand_hard());

	motion_state = 1;
	commit_state = uncommited;
	led_colour(commit_state);
	message.data[2] = kilo_uid;
	message.crc = message_crc(&message);

	walk = 1;
	next_turn = 10;
	turn_turn = 100; //flag value
}

void loop() {

	if (walk) {
		//MOVES
		set_color(RGB(0, 0, 3));

		cur_time = kilo_ticks;

		/*----------------motion state decision--------------------*/
		if (motion_state == 1 && kilo_ticks - motion_prev_time > 100) {
			motion_state = 2;
			motion_prev_time = kilo_ticks;
			turn_time = rand() % 100;
			spinup_motors();
			double ran = rand() / (double) RAND_MAX;
			int turn = random_turn(ran);
			if (ran < 0.5)
				set_motors(0, turn);
			else
				set_motors(turn, 0);

		} else if (motion_state == 2
				&& kilo_ticks - motion_prev_time > turn_time) {
			motion_state = 1;
			motion_prev_time = kilo_ticks;
			spinup_motors();
			set_motors(255, 255);
		}
		/*--------------------Recording messages-----------------------*/
		if (new_message) {
			new_message = 0;
			rec_quality[mes_from] = mes_payload;
			flag = 1;
		}

		/*-----When uncommitted------*/
		if (commit_state == uncommited) {
			if (flag) {
				flag = 0;
				//if (distance < DESIRED_DISTANCE) {

				if (rec_quality[mes_from] == 30) {
					commit_state = Cb;
					walk = 0;
					printf(
							"Ci:kilo_uid: %d. mes_payload: %d. mes_from: %d distance: %d. commit_state: %d\n",
							kilo_uid, mes_payload, mes_from, distance,
							commit_state);
				} else if (rec_quality[mes_from] == 60) {
					commit_state = Cr;
					walk = 0;
					printf(
							"Cj:kilo_uid: %d. mes_payload: %d. mes_from: %d distance: %d. commit_state: %d\n",
							kilo_uid, mes_payload, mes_from, distance,
							commit_state);
				} else {
					walk = 1;
				}

			}
		}

	} else //STAY
	{
		set_color(RGB(0, 3, 0));
		set_motors(0, 0);
		//MOVES

		if (N_neighbors == 0)
			set_color(RGB(0, 0, 1));
		else if (N_neighbors > 0)
			set_color(RGB(1, 1, 0));

//		float p = ComputeProba(N_neighbors);
//		float randUniform = rand() / (double) RAND_MAX;
//
//		if (randUniform < p) {
//
//			walk = 1;
//
//		} else {
//			walk = 0;
//
//		}

	}

	led_colour(commit_state);
	/*----------------Broadcasting state--------------------*/
//	if (commit_state != uncommited) {
//		send_prev_time = cur_time;
//		message.type = NORMAL;
//		message.data[0] = commit_state;
//		message.data[1] = com[commit_state];
//		message.crc = message_crc(&message);
//	}
}

int random_turn(double ran) {
	double c = ((2.0 * RHO) / (1.0 + (RHO * RHO)));
	double V = cos(ran * TWO_PI);
	double sigma = acos((V + c) / (1 + (c * V))); //% [0, PI];
	return (int) (rint)(COMPLETE_TURN * sigma) / PI;
}

//int random_turn(double ran) {
//	double val, theta, u, q;
//	double c = RHO;
//	q = 0.5;
//	u = (double) rand_soft() / (double) 256;
//	val = (1.0 - c) / (1.0 + c);
//	theta = 2 * atan(val * tan(PI * (u - q)));
//
//	return theta * 8;
//}

message_t* message_tx() {
	return &message;
}

void message_tx_succes() {
	message_sent = 1;
	purge();
}

int main() {
	kilo_init();
	kilo_message_rx = message_rx;
	kilo_message_tx = message_tx;
	kilo_message_tx_success = message_tx_succes;
	kilo_start(setup, loop);
	return 0;
}

///////////////NOTES  /////////////////////////

//// Set the LED color based on the gradient.
//if (own_gradient == 0)
//{
//   set_color(RGB(1, 1, 1)); // White
//}
//else if (own_gradient == 1)
//{
//   set_color(RGB(1, 0, 0)); // Red
//}
//else if (own_gradient == 2)
//{
//   set_color(RGB(0, 1, 0)); // Green
//}
//else if (own_gradient == 3)
//{
//   set_color(RGB(0, 0, 1)); // Blue
//}
//else if (own_gradient == 4)
//{
//   set_color(RGB(1, 0, 1)); // Magenta
//}
//else if (own_gradient >= 5)
//{
//   set_color(RGB(1, 1, 0)); // Yellow
//}

//void read_neighbours(){
//    FILE *fPointer;
//    char str[100];
//    sprintf(str, "src/examples/behaviors/neighbours/neighbours%i.csv",kilo_uid);
//    fPointer = fopen(str, "r");
//
//    char neighbours_front_s[10];
//    char neighbours_back_s[10];
//    char neighbours_left_s[10];
//    char neighbours_right_s[10];
//    char neighbours_frontleft_s[10];
//    char neighbours_frontright_s[10];
//    char neighbours_backleft_s[10];
//    char neighbours_backright_s[10];
//
//    fscanf(fPointer, "%s", neighbours_front_s);
//    fscanf(fPointer, "%s", neighbours_back_s);
//    fscanf(fPointer, "%s", neighbours_left_s);
//    fscanf(fPointer, "%s", neighbours_right_s);
//
//    fscanf(fPointer, "%s", neighbours_frontleft_s);
//    fscanf(fPointer, "%s", neighbours_frontright_s);
//    fscanf(fPointer, "%s", neighbours_backleft_s);
//    fscanf(fPointer, "%s", neighbours_backright_s);
//
//    char *p;
//
//    neighbours[FRONT]       = strtof(neighbours_front_s, &p);
//    neighbours[BACK]        = strtof(neighbours_back_s, &p);
//    neighbours[LEFT]        = strtof(neighbours_left_s, &p);
//    neighbours[RIGHT]       = strtof(neighbours_right_s, &p);
//    neighbours[FRONTLEFT]   = strtof(neighbours_frontleft_s, &p);
//    neighbours[FRONTRIGHT]  = strtof(neighbours_frontright_s, &p);
//    neighbours[BACKLEFT]    = strtof(neighbours_backleft_s, &p);
//    neighbours[BACKRIGHT]   = strtof(neighbours_backright_s, &p);
//}

