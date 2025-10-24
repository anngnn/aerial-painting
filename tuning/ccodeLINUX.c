//for linux, compile with -lm after ccodeLINUX.c before -o

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
// #define _USE_MATH_DEFINES //This may be windows only.
#include <math.h>
#include <time.h>
//linux imports
#include <unistd.h> // POSIX API for close()
#include <arpa/inet.h>   // Functions for internet operations (inet_addr, htons, etc.)
#include <sys/socket.h>  // Socket functions
#include <netinet/in.h>  // Internet address structures and protocols
#include <fcntl.h>
#include <sys/mman.h> //used for memory management (shared memory)
#include <termios.h> //used for serail stuff

#include "control_helper.h"
#include "myserial.h"


#define BUFFER_SIZE 1024
#define TRUE 1
#define FALSE 0
#define UDELTA 200 //MAX control limit +/- 1500 for x, y, and yaw
#define MAX_Z_U 2000
#define MIN_Z_U 900
#define MINI_DELTA 50 //limit on how different a control value can be from the previou value
#define SHARED_MEMORY_NAME "MySharedMemory"
#define DATA_SIZE 150  // Total size of shared memory (in bytes)
#define MAX_VELO 0.125 //meters per second
#define MASS 0.120
#define ARM_DELAY 300000000 // nanoseconds ==> 300ms ==> 0.3s
#define BAD_MSG_THRSHLD 10

//Definitions for the serial connection.
#define MSP_API_VERSION 1
#define MSP_FC_VARIANT 2
#define MSP_FC_VERSION 3
#define MSP_BUILD_INFO 5
#define MSP_BOARD_INFO 4
#define MSP_NAME 10
#define MSP_BATTERY_CONFIG 32
#define MSP_SET_REBOOT 68
#define MSP_STATUS 101
#define MSP_ANALOG 110
#define MSP_BATTERY_STATE 130
#define MSP_STATUS_EX 150
#define MSP_UID 160
#define MSP_SET_RAW_RC 200
#define MSP_ACC_TRIM 240
#define CTRL_LOOP_TIME 10000000 //nano seconds (should be 0.01s or 10 million nano seconds)
#define SLOW_MSGS_LOOP_TIME 200000000 //nano seconds (should be 0.2s or 200 million nano seconds)
#define FC_WARM_UP_TIME 5 //seconds


//DEVELOPERS NOTE: IF you need to reset the board, send this: send_msp_command(fserial, MSP_SET_REBOOT, NULL, 0);

// Send RC command (Throttle, Roll, Pitch, Yaw, AUX channels)
//This is directly from ChatGPT
void send_rc_command(int fserial, uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw, uint16_t aux1, uint16_t aux2) {
  /*
    Receives commands in the form A E T R Aux1, Aux2. These are values between 1000 and 2000
    IT then automaticall makes them into uint8_t (little endian I think??) to be compatible with the expected serial comm protocol on the betaflight fc
    Finally, it sends a MSP_SET_RAW_RC command to the FLIGHT CONTROLLER
  */
    uint8_t rc_values[12] = {
        roll & 0xFF, roll >> 8,
        pitch & 0xFF, pitch >> 8,
        throttle & 0xFF, throttle >> 8,
        yaw & 0xFF, yaw >> 8,
        aux1 & 0xFF, aux1 >> 8,  // AUX1 (Arming switch)
        aux2 & 0xFF, aux2 >> 8,  // AUX2 (Modes)
    };

    // printf("CMD: %d, %d, %d, %d, %d, %d\r\n", roll, pitch, throttle, yaw, aux1, aux2);
    send_msp_command(fserial, MSP_SET_RAW_RC, rc_values, 12);  // 200 = MSP_SET_RAW_RC
}

void update_KF(float *x, float *y, float *z, float *roll, float *pitch, float *yaw, \
                float *dx, float *dy, float *dz, float *droll, float *dpitch, float *dyaw, \
                float *eststate, float *P, float *R){
  //1st, compute measurement residual (Y)
  float Y[12];
  Y[0] = *x - eststate[0];
  Y[1] = *y - eststate[1];
  Y[2] = *z - eststate[2];
  Y[3] = *dx - eststate[3];
  Y[4] = *dy - eststate[4];
  Y[5] = *dz - eststate[5];
  Y[6] = *pitch - eststate[6];
  Y[7] = *roll - eststate[7];
  Y[8] = *yaw - eststate[8];
  Y[9] = *dpitch - eststate[9];
  Y[10] = *droll - eststate[10];
  Y[11] = *dyaw - eststate[11];

  // printf("FINDING Y:\r\n");
  // for (int i = 0; i < 12; i ++){
  //   printf("%0.3f, ", Y[i]);
  // }

  // printf("\r\n");

  
  float S[12];
  float K[12];
  for (int i = 0; i < 12; i ++){

    // printf("FINDING THE REST FOR ITEM: %d\r\n", i);

    // printf("R: %f\r\n", R[i]);
    // printf("P: %f\r\n", P[i]);

    //2nd, compute residual covariance (S)
    S[i] = P[i] + R[i];

    // printf("S: %f\r\n", S[i]);

    //3rd, compute the Kalman Gain (K)
    K[i] = P[i] * 1/(S[i]);

    // printf("K: %f\r\n", K[i]);

    //4th, Update the state estimate
    eststate[i] = eststate[i] + (K[i] * Y[i]);

    // printf("Est state: %f\r\n", eststate[i]);

    //5th, Update covariance estimate (P)
    P[i] = (1-K[i]) * P[i];

    // printf("P: %f\r\n", P[i]);

  }

  return;
}

void predict_KF(float *eststate, float *uroll, float *upitch, float *uthrottle, float *uyaw, float dt, float *P, float *Q){
  
  //Predict the next state x
  //x = Ax + Bu
  eststate[0] = eststate[0] + eststate[3]*dt;
  eststate[1] = eststate[1] + eststate[4]*dt;
  eststate[2] = eststate[2] + eststate[5]*dt;
  eststate[3] = eststate[3];
  eststate[4] = eststate[4];
  eststate[5] = (*uthrottle)/MASS;
  eststate[6] = eststate[6] + eststate[9]*dt;
  eststate[7] = eststate[7] + eststate[10]*dt;
  eststate[8] = eststate[8] + eststate[11]*dt;
  eststate[9] = eststate[9] + *upitch*dt;
  eststate[10] = eststate[10] + *uroll*dt;
  eststate[11] = eststate[11] + *uyaw*dt;

  //Update P
  //P = A@P@A.T + Q
  for (int i = 0; i < 12; i ++){
    //Do this for x,y,z and roll, pitch, yaw only.
    if (i < 3 || (i >= 6 && i <9)){
      P[i] = dt*dt*P[i] + Q[i];
    }
    // printf("%f\r\n", eststate[i]);
  }


}


int controlLoop(uint8_t *p_id, char *plocalizer_ip, uint16_t *plocalizer_port, uint16_t *plocalizer_timeout_sec, uint32_t *plocalizer_timeout_nsec,\
                         uint8_t *standalone){
  /*
  gets a unique robot id (from bootloader when the localizer object is created) and stores it

  Also gets ip address of multicast group, port for the socket, and a socket timeout time..

  Creates a udp client socket to listen for information from the optitrack computer. 
  via a multicast network
  DEVELOPERS NOTE: Port and ip address defined in params.json
  The ip address is likely 224.1.1.1, which is the multicast group that this robot will join.

  Gets location data from optitrack, runs the control loop, and writes control values to the flight controller.p_id

  Interfaces with the python code via shared memory.

  If standalone, the controlloop assumes it is running on its own (without the rest of the operating system)
  */

  FILE *clog_fptr; //
  char filename[] = "data.csv"; //This name will change below.
  time_t file_name_time = time(NULL);

  //create a log for administrative c things (i.e., we won't be printing our control data here, but we will be printing info about c operations/setup)
  struct tm tm = *localtime(&file_name_time);
  sprintf(filename, "clogs/%d-%02d-%02d_%02d%02d%02d.csv", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  clog_fptr = fopen(filename, "w");

  //if standalone, we will print out what we are doing since we assume this is being run in a debug environment
  //if NOT standalone, then we will have a full bootloader environment to interface with, and we likely won't be able to see the prints anyway
  if (*standalone){ 
    printf("my id is: %d\r\n", *p_id);
    printf("Setting up a socket at %s on %d\r\n", plocalizer_ip, *plocalizer_port);
  }
  else{
    fprintf(clog_fptr, "my id is: %d\r\n", *p_id);
    fprintf(clog_fptr, "Setting up a socket at %s on %d\r\n", plocalizer_ip, *plocalizer_port);
    fflush(clog_fptr);
  }

  //Set up a socket - LINUX
  int sock;                         // Socket file descriptor
  struct sockaddr_in addr;          // Address struct for binding
  struct ip_mreq mreq;              // Multicast request struct
  char buffer[BUFFER_SIZE];         // Buffer to store received data
  uint8_t reconnect_tried = FALSE;

  // Create a UDP socket - LINUX
  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      fprintf(clog_fptr,"socket creation failed.");
      fflush(clog_fptr);
      fclose(clog_fptr);
      perror("Socket creation failed");
      exit(EXIT_FAILURE);
  }

  // Allow multiple processes to bind to the same address (optional but useful) - LINUX
  int reuse = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
      fprintf(clog_fptr, "Setting SO_REUSEADDR failed");
      fflush(clog_fptr);
      fclose(clog_fptr);
      perror("Setting SO_REUSEADDR failed");
      exit(EXIT_FAILURE);
  }

  // Allow multiple processes to bind to the same port (optional but useful) - LINUX
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) < 0) {
      fprintf(clog_fptr, "Setting SO_REUSEPORT failed");
      fflush(clog_fptr);
      fclose(clog_fptr);
      perror("Setting SO_REUSEPORT failed");
      exit(EXIT_FAILURE);
  }

  // Set up the address struct for binding - LINUX
  memset(&addr, 0, sizeof(addr));      // Zero out the struct
  addr.sin_family = AF_INET;           // Internet address family
  addr.sin_addr.s_addr = INADDR_ANY;   // Listen on all network interfaces
  addr.sin_port = htons(*plocalizer_port);         // Convert port to network byte order

  // Bind the socket to the multicast port - LINUX
  if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      fprintf(clog_fptr, "multicast socket binding failed.");
      fflush(clog_fptr);
      fclose(clog_fptr);
      perror("Binding failed");
      exit(EXIT_FAILURE);
    }


  // Join the multicast group - LINUX
  mreq.imr_multiaddr.s_addr = inet_addr(plocalizer_ip);
  // mreq.imr_interface.s_addr = INADDR_ANY; //TODO TRY MAKING THIS THE ROBOT'S IP ADDRESS
  char my_ip[] = "192.168.18.100";
  char my_ip_end[2];
  if (*p_id < 10){
    sprintf(my_ip_end, "0%d", *p_id);
  }
  else
  {
    fprintf(clog_fptr, "my id: %d\r\n", *p_id);
    sprintf(my_ip_end, "%d", *p_id);
  }
  fprintf(clog_fptr, "my ip end = %s\r\n", my_ip_end);
  my_ip[12] = my_ip_end[0];
  my_ip[13] = my_ip_end[1];

  fprintf(clog_fptr, "mreq.s_addr = %s\r\n", my_ip);

  mreq.imr_interface.s_addr = inet_addr(my_ip);

  if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
      fprintf(clog_fptr, "joining multicast group failed");
      fflush(clog_fptr);
      fclose(clog_fptr);
      perror("Joining multicast group failed");
      exit(EXIT_FAILURE);
  }


  // //Set socket to non blocking mode - LINUX
  // int flags = fcntl(sock, F_GETFL, 0);
  // fcntl(sock, F_SETFL, flags | O_NONBLOCK);

  // set the timeout value for the receivcer socket
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 20000; //this should cause the socket to block for a maximum of 2x the control loop time or until a message is received from optitrack (whichever comes first.)
  float kf_dt = (float)tv.tv_usec/1000000;

  //Set all of the socket options that we just elected above.
  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv) < 0) {
      perror("setsockopt");
      return 1;
  }

  //if standalone, let the user know we are set up to listen for multicast messages from the optitrack computer
  if(*standalone){
    printf("Listening for multicast messages on %s:%d\n", plocalizer_ip, *plocalizer_port);
  }
  else
  {
    fprintf(clog_fptr, "Listening for multicast messages on %s:%d\n", plocalizer_ip, *plocalizer_port);
    fflush(clog_fptr);
  }

  //Now we have to set up to read and write to the shared memory between the c code and the python code.
  // Open the shared file
  int fd = open(SHARED_MEMORY_NAME, O_RDWR);
  if (fd == -1) {
      perror("Error opening file");
      return 1;
  }

  // Map the file into memory
  void *map = mmap(NULL, DATA_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (map == MAP_FAILED) {
      perror("Error mapping memory");
      close(fd);
      return 1;
    }

  //Initialize pointers to the different areas of the shared data.
  //Floats go first to make sure all of the floats have word-aligned (mod 4) addresses.
  float *shared_state = (float *)map; //the state starts at the beginning of the map.
  float *shared_desired_state = (float *)(shared_state + 12); //desired state starts 12 floats (48 bytes) after state (this pattern follows below)
  float *shared_KS = (float *)(shared_desired_state + 4); 
  float *user_code_start_time = (float *)(shared_KS + 12); 
  float *shared_global_time = (float *)(user_code_start_time + 1); //the user code start time is 1 float, so we only move up 1 float (4 bytes) in memory  
  uint16_t *shared_battery_power = (uint16_t *)(shared_global_time + 1); // 1 float (4 bytes) after shared global time
  uint16_t *shared_battery_voltage = (uint16_t *)(shared_battery_power + 1); //Since we have casted to uint16_ts, when we add 1 to the address, we now move 2 bytes in memory.
  uint16_t *shared_armed_status = (uint16_t *)(shared_battery_voltage + 1); 
  uint16_t *shared_flight_mode = (uint16_t *)(shared_armed_status + 1);
  uint8_t *lock_state = (uint8_t *)(shared_flight_mode + 1); //since we cast AFTER the add, this is still moving 2 bytes in memory
  uint8_t *lock_desired_state = (uint8_t *)(lock_state + 1); //now we cast to uint8_ts. each add to address is 1 byte in memory.
  uint8_t *shared_ds_counter = (uint8_t *)(lock_desired_state + 1); 
  uint8_t *safety_code = (uint8_t *)(shared_ds_counter + 1); //1 uint_8t (byte) after shared_ds_counter
  uint8_t *user_code_running = (uint8_t *)(safety_code + 1); //1 uint_8t (byte) after safety_code
  uint8_t *controller_type = (uint8_t *)(user_code_running + 1); //1 uint_8t (byte) after user_code_running
  uint8_t *shared_controller_status = (uint8_t *)(controller_type + 1); //1 uint_8t (byte) after controller_type

  // Set safety to 5 to let teh rest of the processes know we are trying to connect to the FC board.
  *safety_code = 5; //DEVELOPERS NOTE: bevause safety_code is an array of length 1, I can use *safety_code OR safety_code[0]. They do the same thing.

  uint8_t armed = FALSE;
  struct timespec time_of_armed;
  struct timespec time_passed_since_arm;
  clock_gettime(CLOCK_MONOTONIC, &time_of_armed);

  
  
  
  
  //OPEN A SERIAL PORT FOR FC COMMS
  int fserial = open_serial("/dev/serial0");

  if (fserial == -1) {
    printf("The pi DID NOT connect to the fc board.\r\n");
    fprintf(clog_fptr, "The pi DID NOT connect to the fc board.\r\n");
    fflush(clog_fptr);
    fclose(clog_fptr);
    return -1;
  }
  else{
    printf("The pi successfully connected to the FC board.\r\n");
    fprintf(clog_fptr, "The pi CONNECTED to the fc board.\r\n");
    fflush(clog_fptr);
  }

  


  // send initial commands to prevent the RX failsafe from activating
  uint8_t initial_cmds[13] = {MSP_API_VERSION, MSP_FC_VARIANT, MSP_FC_VERSION, MSP_BUILD_INFO,\
                              MSP_BOARD_INFO, MSP_UID, MSP_ACC_TRIM, MSP_NAME, MSP_STATUS, MSP_STATUS_EX,\
                              MSP_BATTERY_CONFIG, MSP_BATTERY_STATE, MSP_BOXNAMES};
  
  for (uint8_t i = 0; i < sizeof(initial_cmds); i++){
    printf("Sending initial command number: %d\r\n", i);
    fprintf(clog_fptr, "Sending initial command number: %d\r\n", i);
    fflush(clog_fptr);
    send_msp_command(fserial, initial_cmds[i], NULL, 0);
    usleep(CTRL_LOOP_TIME/1000);
    uint8_t payload_buffer[256]; //max of 256 bytes
    int result = read_msp_response(fserial, payload_buffer);
    if (result < 0) //the serial read encountered an error.
    {
      return -1;
    }
  }


  if (*standalone){printf("INITIAL SEND COMPLETE\r\n");}
  
  if (*standalone){printf("INITIAL SEND COMPLETE\r\n");}

  printf("INITIAL SEND COMPLETE\r\n");
  fprintf(clog_fptr, "INITIAL SEND COMPLETE\r\n");
  fflush(clog_fptr);
  // return 0;

  uint8_t slow_msgs[2] = {MSP_ANALOG, MSP_STATUS_EX};
  uint8_t slow_msg_pointer = 0;
  uint8_t board_armed_status = 0;


  //INITIALLIZE TIMERS, CLOCKS, and COUNTERS
  uint16_t last_sequence_number = 0;
  uint16_t received_sequence_number;
  float received_time, last_received_time;
  float latency = 0;
  struct timespec loop_start_time, loop_end_time, loop_duration, raw_local_time, local_time_of_user_code_start, elapsed_user_code_time;
  struct timespec c_start_time, time_of_last_succesful_parse, current_time, elapsed_time, time_of_last_estimate;
  struct timespec time_of_last_control, time_since_last_control;
  struct timespec time_of_last_slow_control, time_since_last_slow_control;
  struct timespec actual_time_between_messages;
  clock_gettime(CLOCK_MONOTONIC, &c_start_time); //initialize the c start time
  clock_gettime(CLOCK_MONOTONIC, &time_of_last_succesful_parse); //initialize this as the time of the last successful parse.
  float frequency = 42;
  uint16_t number_of_no_data = 0;
  uint16_t number_of_incorrect_messages = 0;
  uint16_t number_of_scrambled_messages = 0;

  //INITIALIZE STATE AND SETPOINT INFO
  float x,y,z,dx,dy,dz;
  float lastx,lasty,lastz,lastroll,lastpitch,lastyaw;
  float roll,pitch,yaw,droll,dpitch,dyaw;
  float desiredx = 0, desiredy = 0, desiredz = 0, desiredw = 0;
  float lastdesiredx = 0, lastdesiredy = 0, lastdesiredz = 0, lastdesiredw = 0;
  float local_setx = 0, local_sety = 0, local_setz = 0;
  uint8_t desired_state_local_counter = 0;

  //INITIALIZE KALMAN FILTER VARIABLES - START
  //KF VARS

  //state estimate values
  float eststate[12];

  //Estimate Covariance (P) (uncertainty in the real-world initial state. Higher = more uncertain)
  float kfP[12];
  float kfpval = 0.00035;
  for(int i=0; i < 12; i++){kfP[i] = kfpval;}

  //Measurement Covariance (R) 
  // If R is too small, the filter assumes localization data is perfect, which might cause oscillations
  // IF R is too big, the filter ignores measurements too much, leading to drift
  float kfR[12]; //measurement noise
  float kfrval = 6.25;
  for(int i=0; i < 12; i++){kfR[i] = kfrval;}

  //noise covatiance (Q)
  // If Q is too small, the filter trusts the model too much and doesn't correct itself with measurements enough.
  // If Q is too big, the filter will be noisy and unstanble
  float kfQ[12]; //Process noise
  float kfqval = 0.285;
  for(int i=0; i < 12; i++){kfQ[i] = kfqval;}

  uint8_t filter_initialized = FALSE;








  //INITIALIZE KALMAN FILTER VARIABLES - END

  //Iniitialize velocity filters
  float velo_alpha = 0.25;
  // float fx, fy, fz;

  //Create a file to write data.
  FILE *fptr; //pointer to a file
  uint8_t file_initialized = FALSE;
  uint8_t write_to_file = FALSE;

  
  ///////////////////////////////////////////
  /// DEFINE VARIABLES FOR THE CONTROL LOOP
  ///////////////////////////////////////////

  // K matrix for LQR controller is pre-solved.
  // float Ksx1_P_x = shared_KS[0]; //3.000000;
  // float Ksx2_D_x = shared_KS[1]; //2.547266;
  // float Ksx3_P_pitch = (-1)*shared_KS[2]; //0.798802;
  // float Ksx4_D_pitch = (-1)*shared_KS[3]; //0.012890;
  // float Ksy1_P_y = shared_KS[4]; //-3.000000;
  // float Ksy2_D_y = shared_KS[5]; //-2.547266;
  // float Ksy3_P_roll = (-1)*shared_KS[6]; //0.798802;
  // float Ksy4_D_roll = (-1)*shared_KS[7]; //0.012890;
  // float Ksz1 = shared_KS[8]; //6.546537;
  // float Ksz2 = shared_KS[9]; //3.184744;
  // float Ksw1 = shared_KS[10]; //1.414214;
  // float Ksw2 = shared_KS[11]; //0.015033;

  float Ksx1_P_x = 1.885;//3.82, i term of 0.001
  float Ksx2_D_x = 1.885; //2.547266;
  float Ksy1_P_y = -1.885;//3.82, i term of 0.001
  float Ksy2_D_y = -1.885; //-2.547266;
  float Ksz1 = 11.8554;//5.8554;
  float Ksz2 = 3.4813189;
  float Ksw1 = shared_KS[10]; //1.414214;
  float Ksw2 = shared_KS[11]; //0.015033;

  float filtered_desired_pitch = 0.0;
  float filtered_desired_roll = 0.0;
  float alpha = 1.0; //closer to 1 -> more weight on NEW data

  // NEW**
  // Initialize gains for POSITION controller
  float Kp_pos_x = 2.0;
  float Ki_pos_x = 0;
  float Kd_pos_x = 0;
  float P_term_pos_x = 0.0; 
  float I_term_pos_x = 0.0;
  float D_term_pos_x = 0.0;
  float desired_velocity_x = 0.0;
  float error_pos_x = 0.0;
  float error_pos_y = 0.0;
  float error_pos_z = 0.0;
  
  // Initialize gains for VELOCITY controller
  
  // NEW** END

  
  //Integral terms
  float integral_x = 0.0, integral_y = 0.0, integral_z = 0.0;
  float KI_z = 1.25;
  float KI_x = 0.0010;
  float KI_y = -0.0010;
  uint16_t u_hover = 250;

  //Commands
  uint8_t we_have_data_to_run_controller = FALSE;
  int16_t roll_u = 1500;
  int16_t pitch_u = 1500;
  int16_t z_u = 900;
  int16_t yaw_u = 1500;
  int16_t temp_u = 1500;
  uint16_t counter = 0;
  uint8_t up_down = 0;
  int16_t commanded_u = 1500;
  int16_t lastroll_u = 1500;
  int16_t lastpitch_u = 1500;
  int16_t lastz_u = 900;
  int16_t lastyaw_u = 1500;
  float ux, uy, uz, uw; //these are the RAW commands (not between 1000 and 2000)
  // float acc_roll = 0; float acc_pitch = 0; float acc_yaw = 0; float acc_z = 0;


  //Initialize the ARM and MODE commands to be 1000 and 1000
  *shared_armed_status = 1000;
  *shared_flight_mode = 1000;

  uint8_t first_data_point = TRUE;
  if (*standalone){printf("TRYING TO SEND 5s of disarm commands\r\n");}
  else{
    fprintf(clog_fptr, "TRYING TO SEND 5s of disarm commands\r\n");
    fflush(clog_fptr);
  }
  

  //send a disarm command for Xs as the drone gets warmed up.
  //To do this, we need 1 timer tracking Xs of time (that's elapsed time)
  //And one timer tracking the time between commands sent to the fc (that's time_since_last control.)
  //We initialize those timers here.
  clock_gettime(CLOCK_MONOTONIC, &time_of_last_control);
  clock_gettime(CLOCK_MONOTONIC, &raw_local_time);
  sub_timespec(c_start_time, raw_local_time, &elapsed_time);
  sub_timespec(time_of_last_control, raw_local_time, &time_since_last_control);

  //HERE WE SEND THE XS of WARMUP MESSAGES
  while (elapsed_time.tv_sec < FC_WARM_UP_TIME){

    //we want to send a command every CTRL_LOOP_TIME (which is in nano seconds), 
    //BUT, if for some reason we did not update time_of_last_control within 1s, then it may have been OVER 0seconds since our last send,
    //Which should trigger a resent too.
    if (time_since_last_control.tv_sec > 0 || time_since_last_control.tv_nsec >= CTRL_LOOP_TIME){

      send_rc_command(fserial, 1500, 1500, 900, 1500, 1000, 1000);
      uint8_t payload_buffer[256]; //max of 256 bytes
      int result = read_msp_response(fserial, payload_buffer);
      // if (result < 0) //the serial read encountered an error.
      // {
      //   return -1;
      // }
      //update time of last control sent
      clock_gettime(CLOCK_MONOTONIC, &time_of_last_control);

      // break;

    }

    //update elapsed time and time SINCE last control.
    clock_gettime(CLOCK_MONOTONIC, &raw_local_time);
    sub_timespec(c_start_time, raw_local_time, &elapsed_time);
    sub_timespec(time_of_last_control, raw_local_time, &time_since_last_control);
  }
  clock_gettime(CLOCK_MONOTONIC, &raw_local_time);
  sub_timespec(c_start_time, raw_local_time, &elapsed_time);


  if (*standalone){printf("DONE SENDING.\r\n");}
  else
  {
    fprintf(clog_fptr, "DONE SENDING.\r\n");
    fflush(clog_fptr);
  }

  //just double check that no one else overwrote the safety value before we set it back to zero.
  if (*safety_code == 5){
    *safety_code = 0;
  }

  uint8_t data_found = FALSE; //tracks if we found data that is relevant to us.
  uint16_t count_delta = 0;

  srand(time(NULL));

  // Generate a random number between 10 and 20
  int randomNumber = (rand() % 11) + 10;
  int printed = FALSE;
  int break_counter = 0;

  uint8_t check_slow = FALSE;

  clock_gettime(CLOCK_MONOTONIC, &loop_start_time);
  fprintf(clog_fptr,"%d.%.3ld: C while loop reached.\r\n", (int)loop_start_time.tv_sec, loop_start_time.tv_nsec/1000000);
  fflush(clog_fptr);

  uint8_t lost_while_stationary = FALSE;
  uint8_t stationary_lost_counter = 0;
  uint8_t printed_heartbeat = FALSE;
  clock_gettime(CLOCK_MONOTONIC, &time_of_last_succesful_parse); //initialize this as the time of the last successful parse.

  // Infinite loop to receive messages and run the controller - LINUX
  while (1) {
    clock_gettime(CLOCK_MONOTONIC, &loop_start_time);
    
    data_found = FALSE;

    // printf("loop start.\r\n");

    //check for estop or regular stop
    if (*safety_code == 1 || *safety_code ==2){
      fprintf(clog_fptr, "C IS BREAKING DUE TO: %d\r\n", *safety_code);
      fflush(clog_fptr);
      break;
    }
    //check if the bootlaoder is doing something with the logs. In which case, just ignore it (on our side)
    //DEVELOPERS NOTE: code on the python side will ensure this is only reset to 0 when it needs to be.
    //by keeping it 0 on the C side, we make sure it doesn't immediately go back to 6 or 7 when bootloader is done.
    else if (*safety_code > 5)
    {
      *safety_code = 0;
    }

    //update the clog file periodically so we know its still running.
    if (loop_start_time.tv_sec % 10 == 0)
    {
      if (printed_heartbeat == FALSE)
      {
        fprintf(clog_fptr, "%d.%.3ld: still running.\r\n", (int)loop_start_time.tv_sec, loop_start_time.tv_nsec/1000000);
        fflush(clog_fptr);
        printed_heartbeat = TRUE;
      }
    }
    else if (lost_while_stationary == TRUE && stationary_lost_counter < 50)
    {
      fprintf(clog_fptr, "%d.%.3ld: still lost.\r\n", (int)loop_start_time.tv_sec, loop_start_time.tv_nsec/1000000);
      fflush(clog_fptr);
      stationary_lost_counter += 1;
    }
    else
    {
      printed_heartbeat = FALSE;
    }
    

    //if we haven't initialized a file yet, and the user code is running, we should initialize a file.
    if (file_initialized == FALSE && *user_code_running == TRUE){
      file_initialized = TRUE;
      write_to_file = TRUE;
      //no need to rewrite over tm, as we did it in the beginning of the c code and we'll only have one control log per c code run.
      // struct tm tm = *localtime(&file_name_time);
      sprintf(filename, "ctrl_logs/%d-%02d-%02d_%02d%02d%02d.csv", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
      fptr = fopen(filename, "w");
      //WRITE HERE
      fprintf(fptr, "New control log.\n");
      fprintf(fptr, "Raw c time,robot_local_time,global time,data size,frequency,received_sequence_number,latency,");
      fprintf(fptr, "x,y,z,dx,dy,dz,pitch,roll,yaw,dpitch,droll,dyaw,");
      fprintf(fptr, "setx,sety,setz,u_roll,u_pitch,u_z,u_yaw,");

      // NEW**
      fprintf(fptr, "int_x,int_y,int_z,count_delta,voltage,error,");
      fprintf(fptr, "P_term_pos_x,I_term_pos_x,D_term_pos_x,desired_velocity_x,");
      fprintf(fptr, "error_pos_x,error_pos_y,error_pos_z\n"); 

      clock_gettime(CLOCK_MONOTONIC, &local_time_of_user_code_start);
      fflush(fptr);
      fprintf(clog_fptr, "KICKED OFF A NEW FILE.");
      fprintf(clog_fptr, " %d\r\n", *user_code_running);
      fflush(clog_fptr);
    }
    else if (file_initialized && *user_code_running == FALSE)
    {
      //In this case, we have initialized a file, but the user code is no longer running, so we stop writing to the file.
      write_to_file = FALSE;
    }
    
    socklen_t addrlen = sizeof(addr);
      
    //RECEIVED DATA FROM THE SOCKET
    int n = recvfrom(sock, buffer, BUFFER_SIZE - 1, 0, (struct sockaddr*)&addr, &addrlen);

    //This is the case where no data was available, or it has been some number of messages since we got a good one.
    if (n < 0 || number_of_incorrect_messages > BAD_MSG_THRSHLD || number_of_scrambled_messages > BAD_MSG_THRSHLD) {
      //TODO Hanlde timeout here;
      clock_gettime(CLOCK_MONOTONIC, &current_time);
      struct timespec temp_time;
      sub_timespec(time_of_last_succesful_parse, current_time, &temp_time); //time since last successful parse is stored in temp_time
      //basically we check if the timeout has occurred AND (either the user code is running OR the controller is running)
      if ((temp_time.tv_sec > *plocalizer_timeout_sec || temp_time.tv_nsec >= *plocalizer_timeout_nsec) && (*shared_controller_status || *shared_armed_status == 1800) && (reconnect_tried == FALSE))
      {
        
        
        reconnect_tried = TRUE;
        setsockopt(sock, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq)); //leave the multicast group
        if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) { //join the multicast group.
          fprintf(clog_fptr, "%d.%.9ld: re-joining multicast group failed\r\n", (int)loop_start_time.tv_sec, loop_start_time.tv_nsec);
          fflush(clog_fptr);
          break;
        }
        else
        {
          fprintf(clog_fptr, "%d.%.9ld: rejoined the multicast group because ", (int)loop_start_time.tv_sec, loop_start_time.tv_nsec);
          fprintf(clog_fptr, "%d.%ds since last message.\r\n", temp_time.tv_sec, temp_time.tv_nsec);
          fflush(clog_fptr);
        }
      
        
      }
      // there's a hard cutout at half a second.
      else if ((temp_time.tv_sec > 0 || temp_time.tv_nsec >= 500000000) && (*shared_controller_status || *shared_armed_status == 1800))
      {
          *safety_code = 1; //initiate a safety code 1 for estop.

          fprintf(clog_fptr, "Data loss issue.\r\n");
          fprintf(clog_fptr, "%d.%ds since last message.\r\n", temp_time.tv_sec, temp_time.tv_nsec);
          fprintf(clog_fptr, "%d socket queries w. no data.\r\n", number_of_no_data);
          fprintf(clog_fptr, "%d wrong messages.\r\n", number_of_incorrect_messages);
          fprintf(clog_fptr, "%d scrambled messages.\r\n", number_of_scrambled_messages);
          fflush(clog_fptr);
          break; //This will automatically reset the drone as reset happens after break from while
      }
      else if (temp_time.tv_sec > 0 || temp_time.tv_nsec >= 500000000)
      {
        *safety_code = 8;
        if (lost_while_stationary == FALSE)
        {
          lost_while_stationary = TRUE;
          fprintf(clog_fptr, "%d.%.3d: LOST WHILE STATIONARY.\r\n", (int)loop_start_time.tv_sec, loop_start_time.tv_nsec/1000000);
          fflush(clog_fptr);
          stationary_lost_counter = 0;
        }
        
        setsockopt(sock, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq));
        setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));

      }
      

      sub_timespec(time_of_last_estimate, current_time, &temp_time);
      kf_dt = (float)temp_time.tv_nsec/NS_PER_SECOND;

      //ESTIMATE THE STATE 
      if (filter_initialized){
        predict_KF(eststate, &uy, &ux, &uz, &uw, kf_dt, kfP, kfQ);

        //Use the estimate as the state
        // x = eststate[0];
        // y = eststate[1];
        // z = eststate[2];
        // dx = eststate[3];
        // dy = eststate[4];
        // dz = eststate[5];
        // pitch = eststate[6];
        // roll = eststate[7];
        // yaw = eststate[8];
        // dpitch = eststate[9];
        // droll = eststate[10];
        // dyaw = eststate[11];

      }
      clock_gettime(CLOCK_MONOTONIC, &time_of_last_estimate);
      
      //only increment no data if there was no data :)
      if (n<0){
        number_of_no_data ++;
      }

        
    }
    //This is the case where data WAS available and needs to be parsed.
    //DEVELOPERS NOTE: This is not an else if to n < 0 so we can evaluate the safety check even if we have data (but the data has been bad for some time.)
    if (n > 0)
    {
      uint8_t ours_to_parse = FALSE;
      char first_char = buffer[0];
      char last_char = buffer[4];
      //check if it is a message for us or not.
      // 'opti1' is for IDs 5-29 and 'opti2' is for IDs 30 and up
      if ((first_char == 'o') && (last_char == '1') && (*p_id < 30)){
        ours_to_parse = TRUE;
      }
      else if ((first_char == 'o') && (last_char == '2') && (*p_id >= 30))
      {
        ours_to_parse = TRUE;
      }
      else{
        number_of_incorrect_messages ++;
        continue;
      }

      

      //Parse the rest of the message
      if (ours_to_parse == TRUE){
        // printf("ours to parse\r\n");
        
        //The robot messages start 14 bytes in from the start of 'opti' (I.e., it takes 14 bytes to capture the preamble: optiX, timestamp, sequence number,)
        uint8_t offset = 14; 
        uint8_t modulus = 29; //there are 29 bytes for each robot (7 floats * 4bytes/float + 1byte for ID), so we only need to check every 29th byte for a robot id
        
        for (uint16_t i = offset; i < n; i++)
        {
          if((i-offset) % modulus > 0){
            continue;
          }
          else
          {
            //unpack the first byte as the rec_id
            uint8_t rec_id = buffer[i];
            // printf("rec id %d vs. %d\r\n", rec_id, *p_id);
            //check if it is intended for us (the ID is our ID), and store the data if it is.
            if (rec_id == *p_id){

              //unpack the rest of the preamble.
              //DEVELOPERS NOTE: theoretically, the received time should start at the 5th position of the buffer, but
              //when I implemented that it was buggy. Instead, I use the last 2 bytes for the received sequence no and
              //the 4 bytes BEFORE The last 2 (i.e., starting at the last 6) to get the time.
              received_time = parse_a_float(buffer, offset-6); //the time starts in the 6th position (after the 5 bytes of 'optiX')
              *shared_global_time = received_time; // set the shared_global_time to received time
              // printf("C global time: %f \n\r", received_time);
              // printf("rec time: %f\r\n", received_time);
              
              //Use the last 2 bytes of the preamble to get a uint_16 for the count number.
              //These are sent little endian, so the second to last byte of the preamble is the little part of the number
              // printf("%02x    %02x\r\n", buffer[offset-2], buffer[offset-1]);
              // received_sequence_number = buffer[offset-2];
              uint8_t lower_byte = buffer[offset-2];
              uint8_t higher_byte = buffer[offset-1];
              received_sequence_number = (uint16_t)higher_byte << 8 | lower_byte;
              // printf("rec sequ no: %d\r\n", received_sequence_number);
              
              /*#Make sure the sequence number is not an old one.
                #and check the roll over at 65535 
                #to do this, see if the number we received is between 0 and 100, and our previous number was between 65435 and 65535 
                #This is a range of 200 messages, incase we actually missed a message or 2 during the course of the roll over.
                #The 200 message rangge is arbitrary (if we miss more than 200 messages, we are screwed anyway)
              */
              uint8_t roll_over_flag = FALSE;
              if ((received_sequence_number < 100) && (last_sequence_number >= 65435)){
                roll_over_flag = TRUE;
              }

              //if we didn't encounter a roll over and our received sequence number is less than (or equal to) the last one we received, then we are looking at an old message.
              if ((received_sequence_number < last_sequence_number) && (roll_over_flag == FALSE)){
                data_found = FALSE;
                fprintf(clog_fptr, "Sequence number roll over issue. %d received. %d last", received_sequence_number, last_sequence_number);
                fflush(clog_fptr);
                break; //break from parsing this message
              }

              //assuming this isn't an old message, move on to unpack the rest of the state information
              //unpack state information.
              //Each item is 4 bytes long
              uint16_t j = i + 1; //index where x starts
              x = parse_a_float(buffer, j);
              j += 4;
              y = parse_a_float(buffer, j);
              j += 4;
              z = parse_a_float(buffer, j);
              j += 4;
              float q1 = parse_a_float(buffer, j);
              j += 4;
              float q2 = parse_a_float(buffer, j);
              j += 4;
              float q3 = parse_a_float(buffer, j);
              j += 4;
              float q4 = parse_a_float(buffer, j);
              //calculate roll, pitch, and yaw from quaternion
              quaternion_to_euler(&q1, &q2, &q3, &q4, &roll, &pitch, &yaw); 

              //Finally, set the data found flag to be true.
              data_found = TRUE;
            }
          }
          
        }
        
      }

      float dt = 0;

      if (data_found == TRUE){

        if (lost_while_stationary){
          lost_while_stationary=FALSE;
          *safety_code = 0;
          fprintf(clog_fptr, "%d.%.3ld: found data again.\r\n", (int)loop_start_time.tv_sec, loop_start_time.tv_nsec/1000000);
          fflush(clog_fptr);
        }

        // printf("Data found.\r\n");
        reconnect_tried = FALSE; //make it so the socket will try to reconnect again next time there is packet loss
        //DEVELOPERS NOTE: Since we are obviously connected now, we are effectively resetting the reconnect try (only one try per 0.Xs timeout)

        //If this is the first data point, we should make velocities zero.
        if (first_data_point == TRUE){
          first_data_point = FALSE;
          dx = 0.0; dy = 0.0; dz = 0.0;
          droll = 0.0; dpitch = 0.0; dyaw = 0.0;
          // acc_roll = 0.0; acc_pitch = 0.0; acc_yaw = 0.0; acc_z = 0.0;

          //intitialzie our last sequence number (since the optitrack may not start at sequence no 1)
          last_sequence_number = received_sequence_number -1;

          //assume a perfect 100Hz and initialize the last received time
          last_received_time = received_time - 0.01;

          //Initialize our desired state to our current state (in case the user code doesn't set one.)
          desiredx = x; desiredy = y; desiredz = z; desiredw = yaw;

          //Initialize our last known setpoint as our current position.
          lastdesiredx = x, lastdesiredy = y, lastdesiredz = z, lastdesiredw = yaw;

          dt = 0.01; //assume 100 Hz
          // printf("First data set.\r\n");

          //initialize the kalman filter
          eststate[0] = x; eststate[1] = y; eststate[2] = z;
          eststate[3] = dx; eststate[4] = dy; eststate[5] = dz;
          eststate[6] = pitch; eststate[7] = roll; eststate[8] = yaw;
          eststate[9] = dpitch; eststate[10] = droll; eststate[11] = dyaw;

          filter_initialized = TRUE;
          // for (int boob=0; boob < 12; boob ++){
          //   printf("%f, ", eststate[boob]);

          // }
          // printf(" INITALIZED \r\n");

          // printf("%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f\r\n", x, y, z, dx, dy, dz, pitch, roll, yaw, dpitch, droll, dyaw);

        }
        //if this is NOT the first data point, calculate velocities
        else
        {
          if (received_time - last_received_time == 0){
            continue; //avoid the divide by zero case
          }

          //velocities are calculated using the time sent with the data from the optitrack machine, 
          //so transmission latency and processing latency are not incorporated in the denomenator of the velocity calculations
          dt = received_time - last_received_time;
          // dx = (x-lastx)/dt;
          // dy = (y-lasty)/dt;
          // dz = (z-lastz)/dt;
          // droll = (roll-lastroll)/dt;
          // dpitch = (pitch-lastpitch)/dt;
          // dyaw = (yaw-lastyaw)/dt;

          dx = (1-velo_alpha)*dx + velo_alpha*((x-lastx)/dt);
          dy = (1-velo_alpha)*dy + velo_alpha*((y-lasty)/dt);
          dz = (1-velo_alpha)*dz + velo_alpha*((z-lastz)/dt);
          droll = (1-velo_alpha)*droll + velo_alpha*((roll-lastroll)/dt);
          dpitch = (1-velo_alpha)*dpitch + velo_alpha*((pitch-lastpitch)/dt);
          dyaw = (1-velo_alpha)*dyaw + velo_alpha*((yaw-lastyaw)/dt);

          // //update the filtered velocity
          // fx = (1-velo_alpha)*fx + velo_alpha*dx;
          // fy = (1-velo_alpha)*fy + velo_alpha*dy;
          // fz = (1-velo_alpha)*fz + velo_alpha*dz;

          // printf("Velocities calcultaed.\r\n");

          //UPDATE THE KALMAN FILTER
          if (filter_initialized){

            predict_KF(eststate, &uy, &ux, &uz, &uw, dt, kfP, kfQ);
            // printf("UPDATING\r\n");
            // printf("%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f\r\n", x, y, z, dx, dy, dz, pitch, roll, yaw, dpitch, droll, dyaw);
            update_KF(&x, &y, &z, &roll, &pitch, &yaw, &dx, &dy, &dz, &droll, &dpitch, &dyaw, eststate, kfP, kfR);

            // for (int boob=0; boob < 12; boob ++){
            //   printf("%0.3f, ", eststate[boob]);

            // } 
            // printf("\r\n"); 

            //ESTIMATE THE STATE (for printing only)
            // printf("%f\r\n", kf_dt);
            

            // for (int boob=0; boob < 12; boob ++){
            //   printf("%0.3f, ", eststate[boob]);

            // } 
            // printf("\r\n");

            // if (break_counter > 10){
            //   return 0;
            // }
            // break_counter++;



          }
          
 
        }

        
        // uint32_t local_time_seconds = raw_local_time.tv_sec - c_start_time.tv_sec;
        // uint32_t local_time_nanosec = raw_local_time.tv_nsec - c_start_time.tv_nsec;
        clock_gettime(CLOCK_MONOTONIC, &raw_local_time);
        sub_timespec(c_start_time, raw_local_time, &elapsed_time);
        sub_timespec(local_time_of_user_code_start, raw_local_time, &elapsed_user_code_time);

        count_delta = received_sequence_number - last_sequence_number; //this is c, so it will automatically be between 0 and 65535
        

        //Write the state information (if it is available for locking.)
        if (*lock_state == 0){
          *lock_state = 1; //set the lock to c code having it (1), then write all of the variables.
          shared_state[0] = x;
          shared_state[1] = y;
          shared_state[2] = z;
          shared_state[3] = dx;
          shared_state[4] = dy;
          shared_state[5] = dz;
          shared_state[6] = pitch;
          shared_state[7] = roll;
          shared_state[8] = yaw;
          shared_state[9] = dpitch;
          shared_state[10] = droll;
          shared_state[11] = dyaw;

          // printf("State set.\r\n");
          // printf("%f, %f", received_time, 42);

          //set the global time too.
          // *shared_global_time = received_time - *user_code_start_time;

          // printf("time set.\r\n");

          *lock_state = 0; //unlock it so python can access if it wants/needs to.
        }
        

        //before looping back, store data so we can calculate velocities next time
        lastx = x; lasty = y; lastz = z;
        lastroll = roll; lastpitch = pitch; lastyaw = yaw;

        //calculate latency
        float time_between_messages = received_time - last_received_time;
        sub_timespec(time_of_last_succesful_parse, raw_local_time, &actual_time_between_messages);
        float temp_sec = (float) actual_time_between_messages.tv_sec;
        float temp_nsec = ((float) actual_time_between_messages.tv_nsec)/NS_PER_SECOND;
        float temp_combo = temp_sec + temp_nsec;

        

        latency = temp_combo - time_between_messages;

        // printf("%0.5f, %0.5f, %0.5f\r\n", temp_combo, time_between_messages, latency);
        





        last_sequence_number = received_sequence_number;
        last_received_time = received_time;

        clock_gettime(CLOCK_MONOTONIC, &time_of_last_succesful_parse);
        clock_gettime(CLOCK_MONOTONIC, &time_of_last_estimate);
        // printf("%d.%d\r\n", time_of_last_succesful_parse.tv_sec, time_of_last_succesful_parse.tv_nsec)
        number_of_no_data = 0;
        number_of_incorrect_messages = 0;
        number_of_scrambled_messages = 0;

        we_have_data_to_run_controller = TRUE;

      } //end data found if
      else{
        number_of_scrambled_messages ++;
      }
    } //end if n>0
        // if (time_of_last_succesful_parse.tv_sec % 10 == 0 && time_of_last_succesful_parse.tv_nsec < 20000000){
        //   printf("pos: %0.3f, %0.3f, %0.3f, %0.3f\r\n", x, y, z, received_time);
        //   printf("%d \r\n", *user_code_running);
        //   printf("%d\r\n", *shared_armed_status);
        // }


    if (we_have_data_to_run_controller){

      we_have_data_to_run_controller = FALSE;

      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ////RUN CONTROL LOOP NOW ////
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      //get voltage and power
      //TODO


      //If the user wants the controller turned on, execute the control loop.
      if (*shared_controller_status == 1)
      {

        // printf("The system is armed. Getting commands.\r\n");
        
        //Get the desired state, IFF the lock is available
        if (*lock_desired_state == 0){
          *lock_desired_state = 1; //let python know that we have the lock.

          //if the counter set in shared data is greater than ours, we should pull new data.
          //This includes roll over logic inherently, as 0-255 = 1, which is greater than 0.  
          if ((uint8_t) (*shared_ds_counter - desired_state_local_counter) > 0){
              desired_state_local_counter = *shared_ds_counter;
              // Get the setpoint
              desiredx = shared_desired_state[0];
              desiredy = shared_desired_state[1];
              desiredz = shared_desired_state[2];
              desiredw = shared_desired_state[3];
            }

          *lock_desired_state = 0; //unlock it.

        }

        //double check we didn't just set all of our desired states to zero. (sometimes theres an error on the python side)
        //IF we did, revert to our last desiredx, etc.
        if (desiredx == 0 && desiredy == 0 && desiredz == 0 && desiredw == 0){
          desiredx = lastdesiredx;
          desiredy = lastdesiredy;
          desiredz = lastdesiredz;
          desiredw = lastdesiredw;
          fprintf(clog_fptr, "THE CODE ALMOST SET A DESIRED STATE TO ALL ZEROS.\r\n");
          fflush(clog_fptr);
        }
        

        

        // float max_distance_we_could_move = 0.577*MAX_VELO*dt; //root(3)/3 * Velocity * dt (assuming max velocity is Xm/s in the [1,1,1] direction (close enough probably))
        // float dist_x = abs(local_setx - lastdesiredx);  // x distance between last setpoint and new setpoint
        // float dist_y = abs(local_sety - lastdesiredy);  // y distance between last setpoint and new setpoint
        // float dist_z = abs(local_setz - lastdesiredz);  // z distance between last setpoint and new setpoint
        // printf("Max distances: %0.3f, %0.3f, %0.3f, %0.3f\r\n", dist_x, dist_y, dist_z, max_distance_we_could_move);

        // if(max_distance_we_could_move < dist_x){
        //   desiredx = lastdesiredx + max_distance_we_could_move;
        //   printf("changing x setpoint from %0.3f to %0.3f\r\n", local_setx, desiredx);
        // }
        // else
        // {
        //   desiredx = local_setx;
        //   printf("maintaining a x setpoint of %0.3f\r\n", desiredx);
        // }
        
        // if(max_distance_we_could_move < dist_y){
        //   desiredy = lastdesiredy + max_distance_we_could_move;
        //   printf("changing y setpoint from %0.3f to %0.3f\r\n", local_sety, desiredy);
        // }
        // else
        // {
        //   desiredy = local_sety;
        //   printf("maintaining a y setpoint of %0.3f\r\n", desiredy);
        // }
        
        // if(max_distance_we_could_move < dist_z){
        //   desiredz = lastdesiredz + max_distance_we_could_move;
        //   printf("changing z setpoint from %0.3f to %0.3f\r\n", local_setz, desiredz);
        // }
        // else
        // {
        //   desiredz = local_setz;
        //   printf("maintaining a z setpoint of %0.3f\r\n", desiredz);
        // }
        

        // printf("desired state: %0.3f, %0.3f, %0.3f, %0.3f\r\n", desiredx, desiredy, desiredz, desiredw);
        

        //check bounding box and roll over errors.
        float box_dim = 1.05; //#meters #TODO make this a parameter we read in from somewhere.

        //Buid the box around the current setpoint
        float x_min = desiredx - box_dim;
        float x_max = desiredx + box_dim;
        float y_min = desiredy - box_dim;
        float y_max = desiredy + box_dim;
        float z_min = 0.0;
        float z_max = desiredz + box_dim;

        //check if roll or pitch are way out of wack.
        if (roll > 1.2 || roll < -1.2){
          //roll over error.
          *safety_code = 1; //initiate a safety code 1 for estop.
          fprintf(clog_fptr,"ROLL roll over error %f.\r\n" , roll);
          fflush(clog_fptr);
          break; //break from while loop. Will cause board to reset.
        }
        else if (pitch > 1.2 || pitch < -1.2)
        {
          //roll over error
          *safety_code = 1;  //initiate a safety code 1 for estop.
          fprintf(clog_fptr, "PITCH roll over error.\r\n");
          fflush(clog_fptr);
          break; //break from while loop. will cause board to reset.
        }
        else if (z < z_min || z > z_max || y < y_min || y > y_max || x < x_min || x > x_max){
          //bounding box error
          *safety_code = 1;  //initiate a safety code 1 for estop.
          fprintf(clog_fptr, "Bounding box error.\r\n");
          fprintf(clog_fptr, "x= %f, setx= %f, y= %f, sety= %f, z= %f, setz= %f", x, desiredx, y, desiredy, z, desiredz);
          fflush(clog_fptr);
          break; //break from while loop. will cause board to reset.
        }
        else{

          //no errors. Go ahead and run the controller.
          //first, get the percentage of each that should be added based on yaw
          // double x_comp = cos((double)yaw);
          // double y_comp = sin((double)yaw);
          double x_comp = 1;
          double y_comp = 0;

          // Get the error in x, y, z
          error_pos_x = desiredx - x; // NEW**
          error_pos_y = desiredy - y;
          error_pos_z = desiredz - z;

          //HYBRID - set up the integral commands (to compensate wind, dying batteries)
          // integral_x += KI_x*(error_pos_x);
          // integral_y += KI_y*(error_pos_y);
          integral_x += error_pos_x; // NEW**
          integral_y += error_pos_y;
          integral_z += KI_z*(error_pos_z);

          // NEW**
          P_term_pos_x = Kp_pos_x*error_pos_x ; 
          I_term_pos_x = Ki_pos_x*integral_x;
          D_term_pos_x = -Kd_pos_x*dx;       // D term should always oppose velocity to provide damping

          desired_velocity_x = P_term_pos_x + I_term_pos_x + D_term_pos_x;
        
          // float desired_pitch_angle = x_comp*(Ksx1_P_x*error_pos_x - Ksx2_D_x*dx + KI_x*integral_x) + y_comp*(-1)*(Ksy1_P_y*error_pos_y - Ksy2_D_y*dy + KI_y*integral_y);
          float desired_roll_angle = y_comp*(Ksx1_P_x*error_pos_x - Ksx2_D_x*dx + KI_x*integral_x) + x_comp*(Ksy1_P_y*error_pos_y - Ksy2_D_y*dy + KI_y*integral_y);

          // filtered_desired_pitch = (1-alpha)*filtered_desired_pitch + alpha*desired_pitch_angle;
          filtered_desired_roll = (1-alpha)*filtered_desired_roll + alpha*desired_roll_angle;

          float angle_scaler = 100;



          //THis is essentially matrix multiplication of the error vector times the K vecor, just written out ahead of time
          //with the values that would compute to zero as zero, assuming a desired 0 velocity and 0 roll, pitch
          // ux = x_comp*(Ksx1_P_x*(error_pos_x) - Ksx2_D_x*(dx) - Ksx3_P_pitch*(pitch) - Ksx4_D_pitch*(dpitch)) + y_comp*(-1)*(Ksy1_P_y*(error_pos_y) - Ksy2_D_y*(dy) - Ksy3_P_roll*(roll) - Ksy4_D_roll*(droll));
          // uy = y_comp*(Ksx1_P_x*(error_pos_x) - Ksx2_D_x*(dx) - Ksx3_P_pitch*(pitch) - Ksx4_D_pitch*(dpitch)) + x_comp*(Ksy1_P_y*(error_pos_y) - Ksy2_D_y*(dy) - Ksy3_P_roll*(roll) - Ksy4_D_roll*(droll));
          uz = Ksz1*(error_pos_z) - Ksz2*(dz);
          uw = Ksw1*(desiredw - yaw) - Ksw2*(dyaw);

          //YAW SEEMS TO BE REVERSED FOR SOME REASON.
          uw = -1*uw;

          //Convert the LQR commands into integer values from 1000 to 2000 around 1500
          pitch_u = (int16_t) (angle_scaler*filtered_desired_pitch + 1500);
          roll_u = (int16_t) (angle_scaler*filtered_desired_roll + 1500);
          z_u = (int16_t) (100*uz + 1500);
          yaw_u = (int16_t) (100*uw + 1500);

          //Add in the integral terms and any hover terms
          // pitch_u = pitch_u + (int16_t)(integral_x*x_comp) + (uint16_t)((-1)*integral_y*y_comp);
          // roll_u = roll_u + (int16_t)(integral_x*y_comp) + (uint16_t)(integral_y*x_comp);
          z_u = z_u + (int16_t) integral_z + u_hover;         

          // z_u = 1500;
          // yaw_u = 1500;
          // pitch_u = 1500;


          // //quick step
          // if (received_time - *user_code_start_time > 10.25)
          // {
          //   temp_u = 1500;
          // }
          // else if (received_time - *user_code_start_time > 10)
          // {
          //   temp_u = 1700;
          // }
          // else
          // {
          //   temp_u = 1500;
          // }

          

          // ////simple ramp
          // if (up_down){
          //   counter ++;
          //   if (counter % 1 == 0){
          //     temp_u += 2;
          //   }
          //   if (temp_u > 1650){
          //     up_down = 0;
          //   }

          // }
          // else
          // {
          //   counter ++;
          //   if (counter % 1 == 0){
          //     temp_u -= 2;
          //   }
          //   if (temp_u < 1350){
          //     up_down = 1;
          //   }
          // }

          //////time based stepping
          // if (received_time - *user_code_start_time > 60)
          // {
          //   temp_u = 1700;
          // }
          // else if (received_time - *user_code_start_time > 50)
          // {
          //   temp_u = 1500;
          // }
          // else if (received_time - *user_code_start_time > 40)
          // {
          //   temp_u = 1600;
          // }
          // else if (received_time - *user_code_start_time > 30)
          // {
          //   temp_u = 1300;
          // }
          // else if (received_time - *user_code_start_time > 20)
          // {
          //   temp_u = 1500;
          // }
          // else if (received_time - *user_code_start_time > 10)
          // {
          //   temp_u = 1400;
          // }
          // else
          // {
          //   temp_u = 1300;
          // }

          // pitch_u = temp_u;
          // roll_u = 1500;
          


          //Compare the new control value with the old one. Make sure its within max diff/min absolutes
          //basically make sure the jump between values isn't too big that it will make the flight controller unhappy
          if (roll_u > lastroll_u + MINI_DELTA){
            roll_u = lastroll_u + MINI_DELTA;
          }
          else if (roll_u < lastroll_u - MINI_DELTA)
          {
            roll_u = lastroll_u - MINI_DELTA;
          }

          if (pitch_u > lastpitch_u + MINI_DELTA){
            pitch_u = lastpitch_u + MINI_DELTA;
          }
          else if (pitch_u < lastpitch_u - MINI_DELTA)
          {
            pitch_u = lastpitch_u - MINI_DELTA;
          }

          if (yaw_u > lastyaw_u + MINI_DELTA){
            yaw_u = lastyaw_u + MINI_DELTA;
          }
          else if (yaw_u < lastyaw_u - MINI_DELTA)
          {
            yaw_u = lastyaw_u - MINI_DELTA;
          }

          if (z_u > lastz_u + MINI_DELTA){
            z_u = lastz_u + MINI_DELTA;
          }
          else if (z_u < lastz_u - MINI_DELTA)
          {
            z_u = lastz_u - MINI_DELTA;
          }




          //Enforce Control Limits (MAX and MINs)
          uint16_t upper_limit = 1500+UDELTA;
          uint16_t lower_limit = 1500-UDELTA;

          if (roll_u > upper_limit){
            roll_u = upper_limit;
          }
          else if (roll_u < lower_limit)
          {
            roll_u = lower_limit;
          }

          if (pitch_u > upper_limit){
            pitch_u = upper_limit;
          }
          else if (pitch_u < lower_limit)
          {
            pitch_u = lower_limit;
          }

          if (yaw_u > upper_limit){
            yaw_u = upper_limit;
          }
          else if (yaw_u < lower_limit)
          {
            yaw_u = lower_limit;
          }

          if (z_u > MAX_Z_U){
            z_u = MAX_Z_U;
            //sacrifice lateral control to stay up in z
            // yaw_u = 1500;
            // roll_u = 1500;
            // pitch_u = 1500;
          }
          else if (z_u < MIN_Z_U)
          {
            z_u = MIN_Z_U;
          }


        
          //save the previous desired x, y, and z in case we need them next loop.
          lastdesiredx = desiredx;
          lastdesiredy = desiredy;
          lastdesiredz = desiredz;
          lastdesiredw = desiredw;

          //save the raw commands so we can get a better KF estimate
          ux = 0.01*(((float)pitch_u - 1500)/100.0);
          uy = 0.01*((float)roll_u - 1500)/100.0;
          uz = 0.01*((float)z_u - 1500)/100.0;
          uw = 0.01*((float)yaw_u-1500)/100.0;

          // printf("%d, %d, %d, %d, %d\r\n", roll_u, pitch_u, z_u, yaw_u, *shared_armed_status);


        }
        
      

        //print commands IFF a file has been initialized.
        if (write_to_file){
                      //WRITE HERE
          // fprintf(fptr, "Raw c time,robot_local_time,global time,frequency,received_sequence_number,count_delta,");
          // fprintf(fptr, "x,y,z,dx,dy,dz,phi,theta,psi,r,p,y");
          // fprintf(fptr, "setx,sety,setz,u_roll,u_pitch,u_z,u_yaw");
          // fprintf(fptr, "int_x,int_y,int_z,power,voltage,error\n");
          fprintf(fptr, "%d.%.9ld,%d.%.9ld,%f,%d,%f,%d,%0.4f,", (int)elapsed_time.tv_sec, elapsed_time.tv_nsec, (int)elapsed_user_code_time.tv_sec, elapsed_user_code_time.tv_nsec, received_time - *user_code_start_time, n, frequency, received_sequence_number, latency);
          fprintf(fptr, "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,", x, y, z, dx, dy, dz, pitch, roll, yaw, dpitch, droll, dyaw);
          fprintf(fptr, "%0.3f,%0.3f,%0.3f,%d,%d,%d,%d,", desiredx, desiredy, desiredz,roll_u,pitch_u,z_u,yaw_u);
          fprintf(fptr, "%0.3f,%0.3f,%0.3f,%d,%d,%d,", integral_x, integral_y, integral_z, count_delta, *shared_battery_voltage, 0);
          fprintf(fptr, "%0.3f,%0.3f,%0.3f,%0.3f,", P_term_pos_x, I_term_pos_x, D_term_pos_x, desired_velocity_x);
          fprintf(fptr, "%0.3f,%0.3f,%0.3f\n", error_pos_x, error_pos_y, error_pos_z); 
          // for (int i=0; i < 12; i++){
          //   fprintf(fptr, "%0.3f,", eststate[i]);
          // }
          // fprintf(fptr, "\n");
          fflush(fptr);
        }
      
    
      } 
      else //case where the controller is off.
      {
        //The controller is off/not running 
        //We still want to print data though, and we need to re-initialize things in case the controlelr is armed again.
        roll_u = 1500; pitch_u = 1500; z_u = 900; yaw_u = 1500;
        integral_x = 0; integral_y = 0; integral_z = 0;

        // NEW**
        // Position PID Controller x
        P_term_pos_x = 0.0;
        I_term_pos_x = 0.0;
        D_term_pos_x = 0.0;
        desired_velocity_x = 0.0;

        if (write_to_file){
                      //WRITE HERE
          fprintf(fptr, "%d.%.9ld,%d.%.9ld,%f,%d,%f,%d,%0.4f,", (int)elapsed_time.tv_sec, elapsed_time.tv_nsec, (int)elapsed_user_code_time.tv_sec, elapsed_user_code_time.tv_nsec, received_time - *user_code_start_time, n, frequency, received_sequence_number, latency);
          fprintf(fptr, "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,", x, y, z, dx, dy, dz, pitch, roll, yaw, dpitch, droll, dyaw);
          fprintf(fptr, "%0.3f,%0.3f,%0.3f,%d,%d,%d,%d,", desiredx, desiredy, desiredz,roll_u,pitch_u,z_u,yaw_u);
          fprintf(fptr, "%0.3f,%0.3f,%0.3f,%d,%d,%d,", integral_x, integral_y, integral_z, count_delta, *shared_battery_voltage, 1);
          fprintf(fptr, "%0.3f,%0.3f,%0.3f,%0.3f,", P_term_pos_x, I_term_pos_x, D_term_pos_x, desired_velocity_x);
          fprintf(fptr, "%0.3f,%0.3f,%0.3f\n", error_pos_x, error_pos_y, error_pos_z);

          // for (int i=0; i < 12; i++){
          //   fprintf(fptr, "%0.3f,", eststate[i]);
          // }
          // fprintf(fptr, "\n");
          fflush(fptr);
        }

      }

      // printf("looping.\r\n");


      //Record the last control values so we are ready for the next loop.
      lastroll_u = roll_u; lastpitch_u = pitch_u; lastz_u = z_u; lastyaw_u = yaw_u;
      
    }//end if run c
    else // case where flyer is not getting data to run controller.
    {
      // printf("HERE\r\n");

      if (write_to_file){
        clock_gettime(CLOCK_MONOTONIC, &raw_local_time);
        sub_timespec(c_start_time, raw_local_time, &elapsed_time);
        sub_timespec(local_time_of_user_code_start, raw_local_time, &elapsed_user_code_time);
                    //WRITE HERE
        fprintf(fptr, "%d.%.9ld,%d.%.9ld,%f,%d,%f,%d,%d,", (int)elapsed_time.tv_sec, elapsed_time.tv_nsec, (int)elapsed_user_code_time.tv_sec, elapsed_user_code_time.tv_nsec, received_time - *user_code_start_time, 0, frequency, 0, 0);
        fprintf(fptr, "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,", x, y, z, dx, dy, dz, pitch, roll, yaw, dpitch, droll, dyaw);
        fprintf(fptr, "%0.3f,%0.3f,%0.3f,%d,%d,%d,%d,", 0.0, 0.0, 0.0,roll_u,pitch_u,z_u,yaw_u);
        fprintf(fptr, "%0.3f,%0.3f,%0.3f,%d,%d,%d,", 0.0, 0.0, 0.0, 0, *shared_battery_voltage, 2);
        fprintf(fptr, "%0.3f,%0.3f,%0.3f,%0.3f,", P_term_pos_x, I_term_pos_x, D_term_pos_x, desired_velocity_x);
        fprintf(fptr, "%0.3f,%0.3f,%0.3f\n", error_pos_x, error_pos_y, error_pos_z);
        // for (int i=0; i < 12; i++){
        //   fprintf(fptr, "%0.3f,", eststate[i]);
        // }
        // fprintf(fptr, "\n");
        fflush(fptr);
      }
    }

    // roll_u = 1500; pitch_u = 1500; z_u = 900; yaw_u = 1500;

    
    //see if we should be armed or not (based on changes to the shared armed status)
    if (*shared_armed_status == 1800 && armed == FALSE){
      armed=TRUE;
      fprintf(clog_fptr, "ARMING!!\r\n");
      fflush(clog_fptr);
      check_slow = TRUE;
      clock_gettime(CLOCK_MONOTONIC, &time_of_armed); //record the time that we saw we should arm.
    }
    else if (*shared_armed_status == 1000 && armed == TRUE)
    {
      armed=FALSE;
      fprintf(clog_fptr,"disarming..\r\n");
      fflush(clog_fptr);
      check_slow = TRUE;
    }

    //update elapsed time and time SINCE last control.
    clock_gettime(CLOCK_MONOTONIC, &raw_local_time);
    sub_timespec(time_of_last_control, raw_local_time, &time_since_last_control);
    sub_timespec(time_of_armed, raw_local_time, &time_passed_since_arm);

    //we want to send a command every CTRL_LOOP_TIME (which is in nano seconds), 
    //BUT, if for some reason we did not update time_of_last_control within 1s, then it may have been OVER 0seconds since our last send,
    //Which should trigger a resent too.
    if (time_since_last_control.tv_sec > 0 || time_since_last_control.tv_nsec >= CTRL_LOOP_TIME){

      commanded_u = pitch_u;

      if (time_passed_since_arm.tv_sec > 0 || time_passed_since_arm.tv_nsec >= ARM_DELAY)
      //here we are free to send whatever the actual commands are.
      {
        send_rc_command(fserial, roll_u, pitch_u, z_u, yaw_u, *shared_armed_status, *shared_flight_mode);
      }
      else
      //Here we must wait for the flyer to arm correctly. Sending neutral values for approx 0.3s after arming.
      {
        send_rc_command(fserial, 1500, 1500, 900, 1500, *shared_armed_status, *shared_flight_mode);
        // printf("Controlled send.\r\n");
      }
      // printf("%d\r\n", *shared_flight_mode);


      
      uint8_t payload_buffer[256]; //max of 256 bytes
      int result = read_msp_response(fserial, payload_buffer);
      // printf("%0.06f\r\n", (float)(time_since_last_control.tv_nsec)/NS_PER_SECOND);

      //DEVELOPERS NOTE:
      //There seems to be a backlog of messages sometimes, so we may get a response to our MSP_ANALOG
      //query after we send the rc command. Instead of dealing with the queue, I just elected to parse the message here.
      if (result == MSP_ANALOG){
        uint8_t battery_voltage = payload_buffer[0];  // Battery voltage in 0.1V increments
        int16_t amperage = payload_buffer[1] | (payload_buffer[2] << 8);  // Signed 16-bit current (0.1A)
        uint16_t mAh_drawn = payload_buffer[3] | (payload_buffer[4] << 8); // Unsigned 16-bit mAh draw

        // printf("MSP_ANALOG:\n");
        // printf("  Battery Voltage: %dV\n", battery_voltage);
        // printf("  Current Draw: %fA\n", amperage / 10.0);
        // printf("  mAh Drawn: %d mAh\n", mAh_drawn);
        // printf("broke from the fast call.\r\n");

        //store the voltage and power in shared data
        if (battery_voltage > 1 && battery_voltage < 100){
          *shared_battery_voltage = battery_voltage;
          *shared_battery_power = battery_voltage*amperage;
        }

        // printf("SHARED V: %d\r\n", *shared_battery_voltage);

        // break;

      }
      else if (result == MSP_STATUS_EX)
      {
        board_armed_status = decode_msp_status_ex(payload_buffer);
        // printf("arm status: %d\r\n", board_armed_status);
      }

      //Update the time of the last control signal sent.
      clock_gettime(CLOCK_MONOTONIC, &time_of_last_control);
    
    } //end fast control update if

    //update the time since last slow control
    sub_timespec(time_of_last_slow_control, raw_local_time, &time_since_last_slow_control);

    if (time_since_last_slow_control.tv_sec > 0 || time_since_last_slow_control.tv_nsec >= SLOW_MSGS_LOOP_TIME || check_slow){

      //send the slow message pointed to by our pointer
      if (check_slow)
      {
        send_msp_command(fserial, MSP_STATUS_EX, NULL, 0);
        check_slow = FALSE;
      }
      else
      {
        send_msp_command(fserial, slow_msgs[slow_msg_pointer], NULL, 0);
        slow_msg_pointer ++;
        if (slow_msg_pointer >= sizeof(slow_msgs)) // should be 2
        {
          slow_msg_pointer = 0;
        }
      }
      uint8_t payload_buffer[256]; //max of 256 bytes
      int result = read_msp_response(fserial, payload_buffer);
      

      if (result == MSP_ANALOG){

        uint8_t battery_voltage = payload_buffer[0];  // Battery voltage in 0.1V increments
        int16_t amperage = payload_buffer[1] | (payload_buffer[2] << 8);  // Signed 16-bit current (0.1A)
        uint16_t mAh_drawn = payload_buffer[3] | (payload_buffer[4] << 8); // Unsigned 16-bit mAh draw

        // printf("MSP_ANALOG:\n");
        // printf("  Battery Voltage: %dV\n", battery_voltage);
        // printf("  Current Draw: %fA\n", amperage / 10.0);
        // printf("  mAh Drawn: %d mAh\n", mAh_drawn);
        // printf("broke from the slow call.\r\n");

        //store the voltage and power in shared data
        if (battery_voltage > 1 && battery_voltage < 100){
          *shared_battery_voltage = battery_voltage;
          *shared_battery_power = battery_voltage*amperage;
        }
      }
      else if (result == MSP_STATUS_EX)
      {
        board_armed_status = decode_msp_status_ex(payload_buffer);
        // printf("arm status: %d, %d\r\n", board_armed_status, *shared_armed_status);
      }
              
  
      //update the time of the last slow control signal

      clock_gettime(CLOCK_MONOTONIC, &time_of_last_slow_control);
      // break;

    }//end slow control update if

    if (data_found){
        clock_gettime(CLOCK_MONOTONIC, &loop_end_time);
        sub_timespec(loop_start_time, loop_end_time, &loop_duration); 
        // uint32_t loop_duration_seconds = loop_end_time.tv_sec - loop_start_time.tv_sec;
        if (loop_duration.tv_sec > 0 || actual_time_between_messages.tv_nsec == 0)
        {
          frequency = -10; //this is so in the data we can tell when the frequency is SUPER slow (instead of 0 when no data)
        }
        else
        {
          //CHANGED TO BE TIME BETWEEN MESSAGES ONLY FOR CLEANER DATA/EASIER TO DIAGNOSE
          // uint32_t loop_duration_nanoseconds = loop_duration.tv_nsec;
          
          // frequency = (float)1000000000/(loop_duration_nanoseconds);

          frequency = (float)1000000000/(float)(actual_time_between_messages.tv_nsec);
        }

    }//end if data found for updating frequency
      
    
    
  }// end while loop

  *safety_code = 5; //set this so hopefully it shows up on the display and goes away once the board is ready to go again.

  if (file_initialized){
    fclose(fptr);
  }

  //reset the board, and then close the serial port to the fc board
  send_msp_command(fserial, MSP_SET_REBOOT, NULL, 0);
  fprintf(clog_fptr, "RESET SENT.\r\n");
  
  
  // Leave the multicast group before exiting
  setsockopt(sock, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq));

  // Close the socket
  close(sock);

  
  usleep(500000);//1/2 second sleep before we close the serial connection
  close(fserial);

  fprintf(clog_fptr, "CLOSING SERIAL PORT. ENDING C.\r\n");
  fflush(clog_fptr);
  fclose(clog_fptr);

  return 0;

}



int main(int argc, char *argv[]) {
  //This expects to get a bunch of things...

  //Parse the variables that were provided when this executable is called
  uint8_t id = atoi(argv[1]);   //ID of the robot. atoi converts a string to an integer
  char *localizer_ip = argv[2]; // pointer to the ip address of the multicast group
  uint16_t localizer_port = atoi(argv[3]);  // port to listen for multicast info from optitrack
  uint16_t localizer_timeout = atoi(argv[4]); //time in seconds * 1000 that the code will wait for before flagging a localizer error
  uint32_t localizer_timeout_nsec = atoi(argv[5]);
  uint8_t standalone = atoi(argv[6]); // whether or not this is operating as a standalone localization tracker (1) OR as a part of the robot system (0)


  printf("hello world 2.0\r\n");

  // start the localizer/control function
  int ret_val = controlLoop(&id, localizer_ip, &localizer_port, &localizer_timeout, &localizer_timeout_nsec, &standalone);
  return ret_val;
}