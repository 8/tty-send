/* test tool to send data to tty devices */

#include <stdio.h>
#include <unistd.h>  /* needed for read/write */
#include <termios.h> // needed for implementing set_inferface_attribs()
#include <fcntl.h>   /* needed for O_RDWR, O_NOCTTY, O_NDELAY */
#include <string.h>
#include <getopt.h>  /* for getopt_long*/
#include <stdlib.h>  /* for atoi() */
#include <errno.h>
#include <pthread.h> /* used for multithreading */
#include <time.h>    /* used for time measurement */

#include <linux/serial.h> /* used for seria_struct */
#include <sys/ioctl.h> /* used for ioctl */

/* function to configure the serial port */
/* custom_baudrate: if this value is different from 0 specify a custom baudrate instead of the baudrate supplied as the speed parameter */
int set_interface_attribs(int fd, speed_t speed, int parity, int stopbits, int databits, int custom_baudrate)
{
  int closest_speed;
  struct serial_struct ss ={};
  struct termios tty = {};

  /* return with an error if tcgetattr fails*/
  if (tcgetattr (fd, &tty) != 0)
    return -1;

  if (custom_baudrate != 0)
  {
    /* configure the port to use custom speed instead of 38400 */
    ioctl(fd, TIOCGSERIAL, &ss);
    ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
    printf("custom_baudrate: %i, ss.baud_base: %i\n", custom_baudrate, ss.baud_base);
    ss.custom_divisor = (ss.baud_base + (custom_baudrate / 2)) / custom_baudrate;
    printf("setting custom divisor to: %i\n", ss.custom_divisor);
    //closest_speed = ss.baud_base / ss.custom_divisor;
    ioctl(fd, TIOCSSERIAL, &ss);
    cfsetospeed(&tty, B38400);
    cfsetispeed(&tty, B38400);
  }
  else
  {
    /* set the baudrate */
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
  }

  //tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  tty.c_cflag = (tty.c_cflag & ~CSIZE);
  tty.c_cflag |= databits;

  // disable IGNBRK for mismatched speed tests; otherwise receive break as \000 chars
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

  /* return with an error if tcsetattr fails */
  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    return -1;

  return 0;
}

typedef enum
{
  ttys_command_set_device,
  ttys_command_set_file,
  ttys_command_set_blocking_mode,
  ttys_command_set_count,
  ttys_command_send_pattern,
  ttys_command_set_echo,
  ttys_command_set_baudrate,
  ttys_command_set_parity,
  ttys_command_set_stopbits,
  ttys_command_set_databits,
  ttys_command_set_receive,
  ttys_command_set_local_echo,
  ttys_command_set_output_file,
  ttys_command_set_send_and_receive
} ttys_command_t;
typedef enum { action_print_help, action_send_file, action_send_pattern, action_echo, action_receive, action_send_and_receive } action_t;

typedef enum { local_echo_hex, local_echo_char, local_echo_none } local_echo_t;

#define DATA_LENGTH 1
#define FILE_NAME_LENGTH 255

/* variables */
typedef struct {
  char device[255];
  char filename[255];
  int non_blocking_write;
  int count;
  char data[DATA_LENGTH];
  int data_length;
  action_t action;
  local_echo_t local_echo;
  speed_t baudrate;
  int custom_baudrate;
  int parity;
  int stopbits;
  int databits;
  char output_file[FILE_NAME_LENGTH];
  volatile int total_bytes_received;
  int total_bytes_written;
} internals_t;

/* forward declarations */
static void ttys_printMsg(char* msg, int count);
static void ttys_printMsgHex(char* msg);
static void ttys_handle_parameters(int argc, char** argv, internals_t *internals);
static int  ttys_open_device(char* device, int non_blocking_write);

static void init_internals(internals_t *internals)
{
  memset(internals, 0, sizeof(internals_t));
  internals->data_length = DATA_LENGTH;
  internals->count = 0;
  internals->data[0] = 0xAA;
  internals->baudrate = B9600;
  internals->parity = 0;
  internals->stopbits = 0;
  internals->databits = CS8;
  internals->custom_baudrate = 0;
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
  printf("\twith a pattern: ./tty-send --device=/dev/ttyRPC+0 --pattern=1 --count=10\n");
  printf("\twith echo:      ./tty-send --device=/dev/ttyRPC+0 --echo\n");
}

static void send_pattern(int fd, internals_t *internals)
{
  int i, remaining_bytes_to_write, bytes_written;
  char *pwrite;

  for (i = 0; i < internals->count || internals->count == 0; i++)
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
      else if (bytes_written == -1)
      {
        printf("error occurred while writing to device\n");
        exit(-1);
      }
    }
  }
}

static void send_file(int fd, internals_t *internals)
{
  int file, readBytes, remaining_bytes_to_write, bytesWritten;
  char buffer[8];
  char* pwrite;
  int write_calls = 0, incomplete_writes = 0;

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
          internals->total_bytes_written += bytesWritten;
        }

        if (bytesWritten != readBytes)
          incomplete_writes++;

        write_calls++;
      }
    }

    printf("total bytes written: %i\n", internals->total_bytes_written);
    printf("write calls: %i\n", write_calls);
    printf("incomplete writes: %i\n", incomplete_writes);

    /* close the file */
    close(file);
  }
}

static void print_local_echo(local_echo_t local_echo, char c)
{
    /* echo the read data back to the console */
    switch (local_echo)
    {
      case local_echo_hex: printf("0x%02X - %c\n", c, c); break;
      case local_echo_char: printf("%c", c); break;
      case local_echo_none: break;
    }
}

static void echo_loop(int fd, internals_t *internals)
{
  char buffer[1];
  int bytes_read;

  printf("entering echo loop - waiting to receive data\n");
  while(1)
  {
    /* wait for reception of data */
    bytes_read = read(fd, buffer, sizeof(buffer));

    if (bytes_read > 0)
    {
      print_local_echo(internals->local_echo, buffer[0]);

      /* echo the read bytes back to device */
      write(fd, buffer, bytes_read);
    }
  }
}

static void receive_loop(int fd, internals_t *internals)
{
  char buffer[1];
  int bytes_read;
  int fd_output = -1;

  /* try to open the output file */
  if (internals->output_file[0] != 0)
  {
    printf("opening output file: '%s'\n", internals->output_file);
    fd_output = open(internals->output_file, O_CREAT | O_RDWR | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
    if (fd_output == -1)
      printf("failed opening output file\n");
  }

  printf("entering receive loop - waiting to receive data\n");
  while(1)
  {
    bytes_read = read(fd, buffer, sizeof(buffer));

    if (bytes_read > 0)
    {
      /* print a local echo */
      print_local_echo(internals->local_echo, buffer[0]);

      /* write the received bytes to the output file */
      if (fd_output != -1)
        write(fd_output, buffer, bytes_read);

      /* stop after reading 'count' bytes */
      internals->total_bytes_received += bytes_read;
      if (internals->count != 0 && internals->total_bytes_received >= internals->count)
        break;
    }
  }
  printf("\n");

  /* close the output file */
  if (fd_output != -1)
    close(fd_output);

}

typedef struct {
  int fd;
  internals_t *internals;
} thread_params_t;

static void *send_file_thread_entry(void *data)
{
  thread_params_t *p = (thread_params_t*)data;
  if (p)
    send_file(p->fd, p->internals);
}

static void *receive_thread_entry(void *data)
{
  thread_params_t *p = (thread_params_t*)data;
  if (p)
    receive_loop(p->fd, p->internals);
}

static int64_t get_elapsed_time_ns(const struct timespec *before, const struct timespec *after)
{
  // printf("before - secs: %li, nsecs: %li\n", before->tv_sec, before->tv_nsec);
  // printf("after - secs: %li, nsecs: %li\n", after->tv_sec, after->tv_nsec);

  return ((after->tv_sec - before->tv_sec) * 1000000000LL) + after->tv_nsec - before->tv_nsec;
}

static void send_and_receive_loop(int fd, internals_t *internals)
{
  pthread_t thread;
  thread_params_t p;
  struct timespec time_before, time_after;
  int64_t elapsed_time_ns;

  printf("will wait with sending file until at least 1 byte is received...\n");

  /* offload the receiving to a different thread */
  p.fd = fd;
  p.internals = internals;
  pthread_create(&thread, NULL, receive_thread_entry, &p);

  /* wait until we received at least 1 byte */
  while (internals->total_bytes_received == 0)
    usleep(10000);

  /* get the time before we start sending the file */
  clock_gettime(CLOCK_MONOTONIC_RAW, &time_before);

  /* start the file sending on the main thread */
  send_file(fd, internals);

  /* get the time after we stopped sending the file */
  clock_gettime(CLOCK_MONOTONIC_RAW, &time_after);

  /* calculate the elapsed time in ns */
  elapsed_time_ns = get_elapsed_time_ns(&time_before, &time_after);

  printf("sending file took: %lli ns\n", elapsed_time_ns);

  /* print a summary */
  printf("sending %i bytes in %lli ms is ~%f bytes per second or ~%f baud\n",
    internals->total_bytes_written,
    elapsed_time_ns / 1000000,
    internals->total_bytes_written / (double)elapsed_time_ns * 1000000000L,
    internals->total_bytes_written / (double)elapsed_time_ns * 1000000000L * 8);

  /* wait for the sending thread */
  pthread_join(thread, NULL);
}

/* main entry point */
int main(int argc, char *argv[])
{
  int fd, bytesRead, file;

  internals_t internals;
  init_internals(&internals);

  /* handle the params */
  ttys_handle_parameters(argc, argv, &internals);

  /* disable buffering for stdout */
  setbuf(stdout, NULL);

  if (internals.action == action_print_help)
    ttys_print_usage();
  else
  {
    /* open the device */
    if ((fd = ttys_open_device(internals.device, internals.non_blocking_write)) != -1)
    {
      /* configure the tty device */
      set_interface_attribs(fd, internals.baudrate, internals.parity, internals.stopbits, internals.databits, internals.custom_baudrate);

      if (internals.action == action_send_file)
        send_file(fd, &internals);
      else if (internals.action == action_send_pattern)
        send_pattern(fd, &internals);
      else if (internals.action == action_echo)
        echo_loop(fd, &internals);
      else if (internals.action == action_receive)
        receive_loop(fd, &internals);
      else if (internals.action == action_send_and_receive)
        send_and_receive_loop(fd, &internals);

      /* closing device */
      printf("closing device %i\n", fd);
      close(fd);
    }
  }
}

static void ttys_set_baudrate(char* arg, internals_t *internals)
{
  int i;
  printf("set baudrate: %s\n", arg);

  i = atoi(arg);
  switch (i)
  {
    case 9600: internals->baudrate = B9600; break;
    case 19200: internals->baudrate = B19200; break;
    case 38400: internals->baudrate = B38400; break;
    case 57600: internals->baudrate = B57600; break;
    case 115200: internals->baudrate = B115200; break;
    default:
      printf("using custom baudrate!\n");
      internals->custom_baudrate = i;
      break;
  }
}

static void ttys_set_parity(char* arg, internals_t *internals)
{
  printf("set parity: %s\n", arg);
  if (strncmp(arg, "none", 4)==0)
    internals->parity = 0;
  else if (strncmp(arg, "even", 4) == 0)
    internals->parity = PARENB;
  else if (strncmp(arg, "odd", 3) == 0)
    internals->parity = (PARENB | PARODD);
  else
    printf("unknown parity value, allowed values are: none, even, odd\n");
}

static void ttys_set_stopbits(char* arg, internals_t *internals)
{
  int i;
  printf("set stopbits: %s\n", arg);

  i = atoi(arg);

  if (i == 1)
    internals->stopbits = 0;
  else if (i == 2)
    internals->stopbits = CSTOPB;
  else
    printf("valid values: 0, 1\n");
}

static void ttys_set_databits(char* arg, internals_t *internals)
{
  int i;
  printf("set databits: %s\n", arg);
  i = atoi(arg);

  switch (i)
  {
    case 5: internals->databits = CS5; break;
    case 6: internals->databits = CS6; break;
    case 7: internals->databits = CS7; break;
    case 8: internals->databits = CS8; break;
    default: printf("valid values: 5, 6, 7, 8\n"); break;
  }
}

static void ttys_execute_command(ttys_command_t cmd, char* arg, internals_t *internals)
{
  int i;
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

    case ttys_command_set_count:
      printf("count: %s\n", arg);
      internals->count = atoi(arg);
      break;

    case ttys_command_send_pattern:
      printf("sending pattern instead of file\n");
      if (arg != NULL)
        internals->data[0] = atoi(arg);
      internals->action = action_send_pattern;
      break;

    case ttys_command_set_echo:
      internals->action = action_echo;
      break;

    case ttys_command_set_baudrate:
      ttys_set_baudrate(arg, internals);
      break;

    case ttys_command_set_parity:
      ttys_set_parity(arg, internals);
      break;

    case ttys_command_set_stopbits:
      ttys_set_stopbits(arg, internals);
      break;

    case ttys_command_set_databits:
      ttys_set_databits(arg, internals);
      break;

    case ttys_command_set_receive:
      internals->action = action_receive;
      break;

    case ttys_command_set_local_echo:
      internals->local_echo = local_echo_hex;
      if (arg != NULL)
      {
        if (strcmp("no", arg) == 0 || strcmp("n", arg) == 0 || strcmp("none", arg) == 0)
          internals->local_echo = local_echo_none;
        else if (strcmp("char", arg) == 0 || strcmp("c", arg) == 0)
          internals->local_echo = local_echo_char;
      }
      break;

    case ttys_command_set_output_file:
      strncpy(internals->output_file, arg, FILE_NAME_LENGTH-1); /* store the file name */
      printf("setting output file to: %s\n", internals->output_file);
      break;

    case ttys_command_set_send_and_receive:
      internals->action = action_send_and_receive;
      break;
  }
}

static void ttys_handle_parameters(int argc, char** argv, internals_t *internals)
{
  int c = -1;

  /* must match the definition of ttys_command_t */
  struct option long_options[] = {
    { "device",     required_argument, 0, 0 },
    { "file",       required_argument, 0, 0 },
    { "block",      required_argument, 0, 0 },
    { "count",      required_argument, 0, 0 },
    { "pattern",    optional_argument, 0, 0 },
    { "echo",       no_argument,       0, 0 },
    { "baudrate",   required_argument, 0, 0 },
    { "parity",     required_argument, 0, 0 },
    { "stopbits",   required_argument, 0, 0 },
    { "databits",   required_argument, 0, 0 },
    { "receive",    no_argument,       0, 0 },
    { "local-echo", required_argument, 0, 0 },
    { "output",     required_argument, 0, 0 },
    { "send_and_receive", no_argument, 0, 0 },
    { 0,            0,                 0, 0 }
  };

  /* enable error messages for arguments */
  opterr = 1;

  /* getopt_long stores the option index here */
  int option_index = 0;

  do
  {
    c = getopt_long(argc, argv, "", long_options, &option_index);

    if (c != -1)
      ttys_execute_command((ttys_command_t)option_index, optarg, internals);

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

