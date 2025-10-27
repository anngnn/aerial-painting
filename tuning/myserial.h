#ifndef MYSERIAL_H
#define MYSERIAL_H
#include <stdint.h>




#define BAUD_RATE B115200
#define PREAMBLE_LENGTH 5
#define MSP_BOXNAMES 116
#define BLOCK 1
#define NONBLOCK 0
#define BOXNAMES_PAYLOAD_SIZE 281
#define MSP_STATUS_EX_PAYLOAD_SIZE 22

// Function declarations
int open_serial(const char *port);
void send_msp_command(int fserial, uint8_t command, uint8_t *payload, uint8_t size);
int read_serial_response(int fd, uint8_t *buffer, int expected_len, uint8_t block);
int read_msp_response(int fserial, uint8_t *payload_buffer);
void print_flight_modes(uint32_t flags);
uint8_t decode_msp_status_ex(uint8_t* payload);





#endif