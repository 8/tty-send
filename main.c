/* test tool to send data to tty devices */

#include <stdio.h>
#include <iostream>
#include <termios.h> // needed for implementing set_inferface_attribs()
#include <fcntl.h>   /* needed for O_RDWR, O_NOCTTY, O_NDELAY */
#include <string.h>
#include <getopt.h>  /* for getopt_long*/
#include <stdlib.h>  /* for atoi() */
#include <errno.h>

/* function to configure the serial port */
int set_interface_attribs(int fd, int speed, int parity, int stopbits, int databits)
{
  struct termios tty = {};
  //memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
  {
    //error_message ("error %d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  //tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  tty.c_cflag = (tty.c_cflag & ~CSIZE);
  tty.c_cflag |= databits;

  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  //tty.c_cflag |= CS8;
  tty.c_iflag &= ~IGNBRK;         // ignore break signal
  tty.c_lflag = 0;                // no signaling chars, no echo,
                                  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag |= stopbits;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
  {
    //error_message ("error %d from tcsetattr", errno);
    return -1;
  }
  return 0;
}

typedef enum { action_print_help, action_send_file, action_send_pattern } action_t;

#define DATA_LENGTH 1

/* variables */
typedef struct {
  char device[255];
  char filename[255];
  int non_blocking_write;
  int repeat_count;
  char data[DATA_LENGTH];
  int data_length;
  action_t action;
} internals_t;

/* forward declarations */
static void ttys_printMsg(char* msg, int count);
static void ttys_printMsgHex(char* msg);
static void ttys_handle_parameters(int argc, char** argv, internals_t *internals);
static int  ttys_open_device(void);

static void init_internals(internals_t *internals)
{
  memset(internals, 0, sizeof(internals_t));
  internals->data_length = DATA_LENGTH;
  internals->repeat_count = 1;
  internals->data[0] = 0xAA;
}

static int ttys_open_device(char* device, int non_blocking_write)
{
  int fd;

  /* opening device */
  printf("opening device %s\n", device );
  fd = open(device, O_RDWR | O_NOCTTY  | (non_blocking_write ? O_NDELAY : 0));
  if (fd < 0) {
    printf("open failed!\n");
  }
  return fd;
}

static void ttys_print_usage()
{
  printf("tty-send tool by mdk\n");
  printf("example usage:\n");
  printf("\twith a file:    ./tty-send --device=/dev/ttyRPC+0 --file=test.txt --block=no\n");
  printf("\twith a pattern: ./tty-send --device=/dev/ttyRPC+0 --pattern=1 --repeat=10\n");
}

static void write_pattern(int fd, internals_t *internals)
{
  int i, remaining_bytes_to_write, bytes_written;
  char *pwrite;

  for (i = 0; i < internals->repeat_count || internals->repeat_count == -1; i++)
  {
    pwrite = internals->data;
    remaining_bytes_to_write = internals->data_length;

    while (remaining_bytes_to_write)
    {
      bytes_written = write(fd, internals->data, remaining_bytes_to_write);
      if (bytes_written > 0)
      {
        remaining_bytes_to_write -= bytes_written;
        pwrite += bytes_written;
      }
    }
  }
}

static void write_file(int fd, internals_t *internals)
{
  int file, readBytes, remaining_bytes_to_write, bytesWritten;
  char buffer[8];
  char* pwrite;
  int write_calls = 0, incomplete_writes = 0, total_bytes_written = 0;

  /* open the file */
  if ( (file = open(internals->filename, O_RDONLY)) < 0)
  {
    printf("could not open file: %s!\n", internals->filename);
  }
  else
  {
    /* transmit the data by reading the file and writing it to the device */
    while ((readBytes = read(file, buffer, sizeof(buffer))) > 0)
    {
      pwrite = buffer;
      remaining_bytes_to_write = readBytes;

      /* resend only the bytes that did not get transmitted! */
      while (remaining_bytes_to_write > 0)
      {
        bytesWritten = write(fd, pwrite, remaining_bytes_to_write);
        if (bytesWritten > 0)
        {
          remaining_bytes_to_write -= bytesWritten;
          pwrite += bytesWritten;
          total_bytes_written += bytesWritten;
        }

        if (bytesWritten != readBytes)
          incomplete_writes++;

        write_calls++;
      }
    }

    printf("total bytes written: %i\n", total_bytes_written);
    printf("write calls: %i\n", write_calls);
    printf("incomplete writes: %i\n", incomplete_writes);

    /* close the file */
    close(file);
  }
}

/* main entry point */
int main(int argc, char *argv[])
{
  int fd, bytesRead, file;

  internals_t internals;
  init_internals(&internals);

  /* handle the params */
  ttys_handle_parameters(argc, argv, &internals);

  if (internals.action == action_print_help)
    ttys_print_usage();
  else
  {
    /* open the device */
    if ((fd = ttys_open_device(internals.device, internals.non_blocking_write)) != -1)
    {
      if (internals.action == action_send_file)
        write_file(fd, &internals);
      else if (internals.action == action_send_pattern)
        write_pattern(fd, &internals);
    
      /* closing device */
      printf("closing device %i\n", fd);
      close(fd);
    }
  }
}

typedef enum
{
  ttys_command_set_device,
  ttys_command_set_file,
  ttys_command_set_blocking_mode,
  ttys_command_set_repeat,
  ttys_command_send_pattern
} test_command_t;

static void ttys_execute_command(test_command_t cmd, char* arg, internals_t *internals)
{
  switch (cmd)
  {
    case ttys_command_set_device:
      printf("device: %s\n", arg);
      strncpy(internals->device, arg, 255);
      break;

    case ttys_command_set_file:
      printf("file: %s\n", arg);
      strncpy(internals->filename, arg, 255);
      internals->action = action_send_file;
      break;

    case ttys_command_set_blocking_mode:
      printf("block on write: %s\n", arg);
      if (strcmp("no", arg) == 0 || strcmp("n", arg) == 0)
        internals->non_blocking_write = 1;
      break;

    case ttys_command_set_repeat:
      printf("repeat: %s\n", arg);
      internals->repeat_count = atoi(arg);
      break;

    case ttys_command_send_pattern:
      printf("sending pattern instead of file\n");
      if (arg != NULL)
        internals->data[0] = atoi(arg);
      internals->action = action_send_pattern;
      break;
  }
}

static void ttys_handle_parameters(int argc, char** argv, internals_t *internals)
{
  int c = -1;

  struct option long_options[] = {
    { "device", required_argument, 0, 0 },
    { "file",   required_argument, 0, 0 },
    { "block",  required_argument, 0, 0 },
    { "repeat", required_argument, 0, 0 },
    { "pattern", optional_argument,0, 0 },
    { 0,        0,                 0, 0 }
  };

  /* enable error messages for arguments */
  opterr = 1;

  /* getopt_long stores the option index here */
  int option_index = 0;

  do
  {
    c = getopt_long(argc, argv, "", long_options, &option_index);

    if (c != -1)
      ttys_execute_command((test_command_t)option_index, optarg, internals);

  } while (c != -1);

}

static void ttys_printMsg(char* msg, int count)
{
  int i;
  printf("(%i) [", count);
  for (i = 0; i < count; i++)
  {
    printf( (i + 1 == count) ? "0x%02X" : "0x%02X ", msg[i]);
  }
  printf("]\n");
}

static void ttys_printMsgHex(char* msg)
{
  int i, length = 0;
  while (msg[length++] != 0);
  length--;

  printf("[");
  for (i = 0; i < length; i++)
  {
    switch (msg[i])
    {
      case '\r': printf("  \\r"); break;
      case '\n': printf("  \\n"); break;
      default:
        printf ("%4c", msg[i]);
        break;
    }
    if (i + 1 < length)
      printf(" ");
  }
  printf("]");

  printf("\n");
  
  printf("[");
  for (i = 0; i < length; i++)
  {
    printf ("0x%02X", msg[i]);
    if (i + 1 < length)
      printf(" ");
  }
  printf("]");

  printf("\n");
}

//char cmd_echo_off[] = { "ATE0\r\n" };
//int cmd_echo_off_length = sizeof(cmd_echo_off)-1;
//
//char msg[] = { "AT+CGMR\r\n" };
//int msg_length = sizeof(msg)-1;
//
//char cmd_verbose_errors[] = { "AT+CMEE=2\r\n" };
//int cmd_verbose_errors_length = sizeof(cmd_verbose_errors)-1;

//void read_response(int fd) 
//{
//  int bytesRead;
//
//  printf("reading from the device:\n");
//  memset(buffer, 0, sizeof(buffer));
//  bytesRead = read(fd, buffer, sizeof(buffer)-1);
//  printMsgHex(buffer);
//  printf("\n");
//}

//void send_command(int fd, char* cmd, int cmd_length)
//{
//  printf("writing to the device:\n");
//  test_printMsgHex(cmd);
//  printf("\n");
//  write(fd, cmd, cmd_length);
//}

