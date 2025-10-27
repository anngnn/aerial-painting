#include <stdio.h>
#include <fcntl.h>
#include <unistd.h> // POSIX API for close()
#include <termios.h> //this is for the serial stuff
#include "myserial.h"


int open_serial(const char *port) {
  /*
    // Open serial port for communication with the flight controller board.
    //This is directly from ChatGPT
  */

  //Here is the raw opening of the serial port.
  int fserial = open(port, O_RDWR | O_NOCTTY | O_SYNC);
  if (fserial == -1) {
      perror("Unable to open serial port");
      return -1;
  }
  
  //Here we specify parameters of the serial connection
  struct termios tty;
  tcgetattr(fserial, &tty); //This gets us a pointer to the attributes of our serial connection.
  cfsetispeed(&tty, BAUD_RATE); //sets the baudrate for INPUT to 115200 which is what Betaflight expects
  cfsetospeed(&tty, BAUD_RATE); //sets the baudrate for OUTPUT to 115200 which is what Betaflight expects
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
  tty.c_cflag &= ~(PARENB | PARODD); // no parity
  tty.c_cflag &= ~CSTOPB; // 1 stop bit
  tty.c_cflag &= ~CRTSCTS; //no hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; //enable read, ignore model control lines

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); //disable software flow control
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); //raw input

  tty.c_oflag &= ~OPOST; //raw output (no newline translation)

  tty.c_lflag &= ~ICANON; //non-canonical mode
  tty.c_lflag &= ~ECHO; //no echo
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0; //X00 ms read timeout. 0 = no timeout (non blocking)

  //This sets all of the serial attributes we just defined above.  
  if (tcsetattr(fserial, TCSANOW, &tty) != 0) {
      perror("Error setting serial attributes");
      close(fserial);
      return -1;
  } 

  return fserial;
}

// Send MSP command
//This is directly from ChatGPT
void send_msp_command(int fserial, uint8_t command, uint8_t *payload, uint8_t size) {
  /*
    Send a command and payload to the flight controller 
    It automatically generates the MSP message header for pi to fc messages: $M<
    It also automatically creates a checksum for the given payload.
  */
    uint8_t checksum = size ^ command;
    uint8_t buffer[10 + size]; // Adjust buffer size as needed
    int index = 0;

    // MSP header
    buffer[index++] = '$';
    buffer[index++] = 'M';
    buffer[index++] = '<';
    buffer[index++] = size;
    buffer[index++] = command;

    // Payload
    for (int i = 0; i < size; i++) {
        buffer[index++] = payload[i];
        checksum ^= payload[i];
    }

    // Checksum
    buffer[index++] = checksum;

    // Print bytes being sent
    // if (command == 150){
    //   printf("Sending: ");
    //   for (int i = 0; i < index; i++) {
    //       printf("%02X ", buffer[i]);
    //   }
    //   printf("\n");
    // }
    

    write(fserial, buffer, index);
    // // printf("written.\r\n");
}



int read_serial_response(int fd, uint8_t *buffer, int expected_len, uint8_t block) {
  /*
    Tries to read from the serail buffer for a specified number of bytes (expected len)
    Stores the data in buffer if read correctly and returns the number of bytes read.
    Returns -1 if the read is unsuccessful (no data to read)
  */
    int bytes_read = 0;
    if(block == 1)
    {
      while (bytes_read < expected_len) {
        int result = read(fd, buffer + bytes_read, expected_len - bytes_read);
        if (result < 0) {
            perror("Serial read error");
            return -1;
        }
        // printf(" - res: %d \r\n", result);
        bytes_read += result;
      }
    }
    else{
      int result = read(fd, buffer, expected_len);
      if (result < 0) {
          perror("Serial read error");
          return -1;
      }
      // printf(" - res: %d \r\n", result);
      bytes_read = result;
    }
    
    return bytes_read;
}

int read_msp_response(int fserial, uint8_t *payload_buffer) {
  /*
    Reads an msp message.
    First, reads the preamble, then verifies the header before extracting the payload, command being responded to, and checksum.
    Verifies the checksum against the payload data.
    Stores the payload into the payload buffer if successful.
    Returns command if successful, -1 otherwise.
  */
    uint8_t preamble[PREAMBLE_LENGTH]; //preamble is 5 bytes long
    
    // Step 1: Read the first 5 bytes (stores in preamble)
    if (read_serial_response(fserial, preamble, PREAMBLE_LENGTH, BLOCK) != PREAMBLE_LENGTH) {
        printf("Error: Incomplete MSP header received.\n");
        return -1;
    }

    // Step 2: Verify header
    if (preamble[0] != '$' || preamble[1] != 'M' || preamble[2] != '>') {
        printf("Error: Invalid MSP header: %02X %02X %02X\n", preamble[0], preamble[1], preamble[2]);
        return -1;
    }

    // Step 3: Extract payload length and command ID
    uint8_t payload_size = preamble[3];
    uint8_t command = preamble[4];

    // Step 4: Read the payload + checksum
    int total_bytes = payload_size + 1; // Payload + checksum
    if (command == MSP_BOXNAMES){
      /*
        The MSP_BOXNAMES returns the ASCII text of all of the flight modes and special AUX functions.
        Specifically, it returns the below. I don't NEED this info, I just need to parse all 281 bytes of it 
        so it doesn't interfere with the other times I try to read from the I2C port.command

        ARM;ANGLE;HORIZON;HEADFREE;FAILSAFE;HEADADJ;BEEPER;LEDLOW;
        OSD DISABLE;BLACKBOX;FPV ANGLE MIX;BLACKBOX ERASE (>30s);
        CAMERA CONTROL 1;CAMERA CONTROL 2;CAMERA CONTROL 3;
        FLIP OVER AFTER CRASH;PREARM;VTX PIT MODE;PARALYZE;
        VTX CONTROL DISABLE;STICK COMMANDS DISABLE;
        BEEPER MUTE;READY;
      */
      printf("Received ASCII %d bytes: ", BOXNAMES_PAYLOAD_SIZE);
      for (int i = 0; i < PREAMBLE_LENGTH; i++){
        printf("%02X ",preamble[i]);
      }
      int ret_val = 0;
      int total = 0;
      int j = 0;
      int zero_counter = 0;
      uint8_t temp_buffer[BOXNAMES_PAYLOAD_SIZE];
      // printf(" - AND - ");
      ret_val = read_serial_response(fserial, temp_buffer, BOXNAMES_PAYLOAD_SIZE, BLOCK);
      for (int i = 0; i < ret_val; i++) {
        printf("%02X ", temp_buffer[i]);
      }
      printf("--fin \n");

    }
    else
    {
      if (read_serial_response(fserial, payload_buffer, total_bytes, BLOCK) != total_bytes) {
          printf("Error: Incomplete MSP payload received.\n");
          return -1;
      }
    }
    

    //Don't bother verifying the checksum for boxnames, the command is going to send ASCII text back which will ruin the checksum.
    //DEVELOPERS NOTE: I don't care about the response for BOXNAMES, so it is okay that I skip it.
    if (command != MSP_BOXNAMES){
      // Step 5: Verify checksum
      uint8_t checksum = 0;
      checksum ^= payload_size;
      checksum ^= command;
      for (int i = 0; i < payload_size; i++) {
          checksum ^= payload_buffer[i];
      }

      
      if (checksum != payload_buffer[payload_size]) {
          printf("Error: Checksum mismatch! Expected: %02X, Received: %02X\n", checksum, payload_buffer[payload_size]);
          return -1;
      }
    }


    

    // if (command == 150){
    //   printf("Received %d bytes: ", total_bytes);
    //   for (int i = 0; i < PREAMBLE_LENGTH; i++){
    //     printf("%02X ",preamble[i]);
    //   }
    //   for (int i = 0; i < total_bytes; i++) {
    //     printf("%02X ", payload_buffer[i]);
    //   }
    //   printf("\n");
    // }

    return command;
}

// Helper: Print flight mode flags
void print_flight_modes(uint32_t flags) {
    if (flags & (1 << 0))  printf("ANGLE ");
    if (flags & (1 << 1))  printf("HORIZON ");
    if (flags & (1 << 2))  printf("MAG ");
    if (flags & (1 << 3))  printf("HEADFREE ");
    if (flags & (1 << 4))  printf("FAILSAFE ");
    if (flags & (1 << 5))  printf("HEADADJ ");
    if (flags & (1 << 6))  printf("GPS_HOLD ");
    if (flags & (1 << 7))  printf("GPS_HOME ");
    printf("\n");
}

// Decode the payload of MSP_STATUS_EX
uint8_t decode_msp_status_ex(uint8_t* payload) {

  //This is the extended version of MSP_STATUS — added in Betaflight 4.x. It includes everything from MSP_STATUS, plus extra mode info.

  if (!payload) return 42;

  // Parse fields
  uint16_t cycleTime         = payload[0] | (payload[1] << 8);
  uint16_t i2cErrorCounter   = payload[2] | (payload[3] << 8);
  uint32_t sensorPresent     = payload[4] | (payload[5] << 8) | (payload[6] << 16) | (payload[7] << 24);
  uint32_t flightModeFlags   = payload[11] | (payload[10] << 8) | (payload[9] << 16) | (payload[8] << 24);
  uint8_t currentProfile     = payload[12];

  uint16_t boxActivation[4];
  for (int i = 0; i < 4; i++) {
      boxActivation[i] = payload[13 + i * 2] | (payload[14 + i * 2] << 8);
  }

  uint32_t armed_bit_filter = 0x02; //0b0010

  uint32_t filtered_flags = flightModeFlags & armed_bit_filter; //This will give 0b0010 if NOT armed, 0b0000 if armed.
  // Print decoded values
  // printf("MSP_STATUS_EX:\n");
  // printf("  Cycle Time         : %u µs\n", cycleTime);
  // printf("  I2C Errors         : %u\n", i2cErrorCounter);
  // printf("  Sensor Present     : 0x%08X\n", sensorPresent);
  // printf("  Flight Modes       : 0x%08X → ", flightModeFlags);
  // print_flight_modes(flightModeFlags);
  // printf("  SecondLast bit      : %d\n ", flightModeFlags & foo);
  // printf("  Active Profile     : %u\n", currentProfile);
  // printf("  Box Activation Bits:\n");
  // for (int i = 0; i < 4; i++) {
  //     printf("    [%d]: 0x%04X\n", i, boxActivation[i]);
  // }

  /*
  Bit and its meaning for box activation bits
  0	ARM
  1	ANGLE
  2	HORIZON
  3	HEADFREE
  4	FAILSAFE
  9	BLACKBOX
  10	FPV ANGLE MIX
  */
  // printf("  ARMED: ");
  // if(boxActivation[0] & (1 << 0)) printf("TRUE\r\n"); else printf ("FALSE\r\n");
  // printf("  MODE: ");
  // if(boxActivation[0] & (1 << 1)) printf("ANGLE\r\n");
  // else if (boxActivation[0] & (1 << 2)) printf("HORIZON\r\n");
  // printf("  HEADFREE: ");
  // if(boxActivation[0] & (1 << 3)) printf("TRUE\r\n"); else printf ("FALSE\r\n");
  // printf("  FAILSAFE: ");
  // if(boxActivation[0] & (1 << 4)) printf("TRUE\r\n"); else printf ("FALSE\r\n");
  // printf("  BLACKBOX: ");
  // if(boxActivation[1] & (1 << 1)) printf("TRUE\r\n"); else printf ("FALSE\r\n");
  // printf("  FPV ANGLE MIX: ");
  // if(boxActivation[1] & (1 << 2)) printf("TRUE\r\n"); else printf ("FALSE\r\n");

  if(filtered_flags == 0){
    return 1; //return ARMED
  }
  else
  {
    return 0; //return DISARMED
  }
  
  
}