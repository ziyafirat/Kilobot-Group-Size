#include <kilolib.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define pa1 5
#define pa2 5

long unsigned int cur_time, motion_prev_time, commit_prev_time, send_prev_time;
char motion_state;
char mes_type;
char mes_payload;
char rec_quality[2];
int rand;
int new_message;
int turn_time;
int x;
bool flag=0;

enum commitment{Ci, Cj ,uncommited};

void led_colour(char x)
{
    if(x==1)
        set_color(RGB(1,0,0));
    else if(x==2)
        set_color(RGB(0,1,0));
    else if(x==3)
        set_color(RGB(0,0,1));
}

// receive message callback
void message_rx(message_t *message/*, distance_measurement_t *distance_measurement*/)
{
    // Set flag on message reception.
    new_message = 1;

    mes_from = message->data[0];
    mes_payload = message->data[1];
}


void setup()
{
    printf("Hello World\n");
    srand(rand_hard());

    motion_state=1;
    commit_state = uncommited;
    led_colour(commit_state);
}

void loop()
{
    cur_time = kilo_ticks;

    /*----------------motion state decision--------------------*/
    if(motion_state == 1 && cur_time-motion_prev_time >600)
    {
        motion_state = 2;
        turn_time = rand()%200;
        motion_prev_time = cur_time;
        spinup_motors();
        set_motors(180, 180);
    }
    else if(motion_state == 2 && cur_time-motion_prev_time > turn_time)
	{
        motion_state = 1;
        rand =  rand()%2;
        printf("rand: %d\n",rand);
        motion_prev_time = cur_time;
        spinup_motors();
        if(rand)
        {
            // spinup_motors();
            set_motors(0, kilo_turn_right);
        }
        else
        {
            //spinup_motors();
            set_motors(kilo_turn_left, 0);
        }
    }
    /*--------------------Recording messages-----------------------*/
    if(new_message)
    {
        new_message = 0;
        rec_quality[mes_from]=mes_payload;
        flag=1;
    }

    /*----------------commitment state decision--------------------*/
    if(cur_time-commit_prev_time > 400)
    {
        if(flag)
        {
            flag=0;
            x=rand()%255;
            if(x<rec_quality[Ci])
                commit_state = Ci;
            else if(x>=rec_quality[Ci]  && x<(rec_quality[Ci]+rec_quality[Cj]))
                commit_state = Cj;
        }

        else if(commit_state == Ci && rand()%100 > pa1)
            commit_state = uncommited;

        else if(commit_state == Cj && rand()%100 > pa2)
            commit_state = uncommited;

        led_colour(commit_state);
        commit_prev_time = cur_time;
    }

    /*----------------Broadcasting state--------------------*/
    if(commit_state!=uncommited && cur_time - send_prev_time > 100)
    {
        send_prev_time = cur_time;
        message.type = NORMAL;
        message.data[0] = commit_state;
        message.data[0] = rand()%255;
        message.crc = message_crc(&message);
    }
}

int main()
{
    kilo_init();
    kilo_message_rx = rx_message;
    kilo_message_tx = message_tx;
    kilo_message_tx_success = message_tx_succes;
    kilo_start(setup, loop);
    return 0;
}
