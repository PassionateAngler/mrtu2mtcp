#include <stdio.h>
#include <ctype.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <semaphore.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <modbus.h>

#define MODBUS_RTU_SLAVE 0x01
#define BAUDRATE 9600 
#define SERIAL_BITS 8
#define PARITY 'E'
#define STOP_BITS 1
#define IP "127.0.0.1"
#define PORT 1502
#define USB_DEV "/dev/ttyUSB0"

/* Function codes */
#define _FC_READ_COILS 0x01
#define _FC_READ_DISCRETE_INPUTS 0x02
#define _FC_READ_HOLDING_REGISTERS 0x03
#define _FC_READ_INPUT_REGISTERS 0x04
#define _FC_WRITE_SINGLE_COIL 0x05
#define _FC_WRITE_SINGLE_REGISTER 0x06
#define _FC_READ_EXCEPTION_STATUS 0x07
#define _FC_WRITE_MULTIPLE_COILS 0x0F
#define _FC_WRITE_MULTIPLE_REGISTERS 0x10
#define _FC_REPORT_SLAVE_ID 0x11
#define _FC_READ_AND_WRITE_REGISTERS 0x17

/*
#define DEBUG_TCP 0
#define DEBUG_RTU 0
*/

/*Number of child server porcess, we need only one*/
#define NB_CON 1

typedef struct{
	/*RTU properties*/
	unsigned int slave_addr;
	char *usb_dev;	
	unsigned int baudrate;
	char parity;
	unsigned int bits;
	unsigned int stop_bits;
	/*TCP properies*/
	char *ip;
	unsigned int port;
	/*daemon properties*/
	unsigned int debug;
	char *log_file;	
	unsigned int daemonize;
} config_t;

int recive_data_from_rtu(modbus_t *rtu, uint8_t *req, 
												int offset, modbus_mapping_t *mb_mapping);

int process_tcp_request(modbus_t *mb_tcp, modbus_t *mb_rtu, 
												modbus_mapping_t *mb_mapping, sem_t *rtu_lock,int offset);

int parseopts(int argc, char *argv[], config_t *opts);

int main(int argc, char *argv[])
{
	/*Semaphore for blocking access to RTU witch is simple thread*/
 	sem_t *rtu_lock;  

	modbus_t *mb_tcp;
	modbus_t *mb_rtu;
	int mb_tcp_socket;
	modbus_mapping_t *mb_mapping;
	int rc;
	int offset;

	/*place for command line opts*/
	config_t opts;

	/*set default values of command lines and prarse them*/
	opts.slave_addr = MODBUS_RTU_SLAVE;
	opts.usb_dev = USB_DEV;
	opts.baudrate = BAUDRATE;
	opts.parity = PARITY;
	opts.bits = SERIAL_BITS;
	opts.stop_bits = STOP_BITS;
	opts.ip = IP;
	opts.port = PORT;	
	opts.debug = 0;
	opts.log_file = "mrtu2mtcp.log";
	opts.daemonize = 0;

	if(parseopts(argc,argv,&opts)<0){
		exit(EXIT_FAILURE);
	}

	/* create, initialize semaphore before everything else */
	rtu_lock = mmap(NULL, sizeof(sem_t),PROT_READ | PROT_WRITE,
									MAP_ANONYMOUS | MAP_SHARED, 0, 0);

	if (!rtu_lock) {
		perror("out of memory\n");
		exit(1);
	}
	if (sem_init(rtu_lock, 1, 1) < 0) {
    perror("semaphore initialization");
    exit(1);
  }
	
	mb_tcp = modbus_new_tcp( opts.ip, opts.port);
	mb_rtu = modbus_new_rtu( opts.usb_dev,opts.baudrate,
													 opts.parity,opts.bits,opts.stop_bits);

	if(opts.debug){
		modbus_set_debug(mb_tcp,TRUE);
	}

#ifdef DEBUG_RTU
	modbus_set_debug(mb_rtu,TRUE);
#endif

	modbus_set_slave(mb_tcp,opts.slave_addr);
	modbus_set_slave(mb_rtu,opts.slave_addr);

	offset = modbus_get_header_length(mb_tcp);

  mb_mapping = modbus_mapping_new(MODBUS_MAX_READ_BITS, MODBUS_MAX_WRITE_BITS,
                             MODBUS_MAX_READ_REGISTERS, MODBUS_MAX_WRITE_REGISTERS );

  if (mb_mapping == NULL) {
 		fprintf(stderr, "Failed to allocate the mapping: %s\n",
   					 modbus_strerror(errno));
    modbus_free(mb_tcp);
    modbus_free(mb_rtu);
		exit(EXIT_FAILURE);
  }

	if( modbus_connect(mb_rtu) == -1){
		fprintf(stderr,"Connection to device '%s': FAILED\n",opts.usb_dev);
		exit(EXIT_FAILURE);
	}

	printf("Connection to device '%s': SUCCESS\n",opts.usb_dev);

	mb_tcp_socket = modbus_tcp_listen(mb_tcp,NB_CON);

	if( mb_tcp_socket == -1){
		fprintf(stderr,"Binding to address %s on port %d: FAILED\n",opts.ip,opts.port);
		exit(EXIT_FAILURE);
	}

	printf("Binding to address %s on port %d: SUCCESS\n",opts.ip,opts.port);
	
	printf("Serial connection params:\n");
	printf("RTU slave address =  %d\n", opts.slave_addr );
	printf("Baudrate = %d\n", opts.baudrate );
	printf("Number of transport bits = %d\n",opts.bits);
	printf("Parity = %c\n",opts.parity);

	for(;;){
		printf("Waiting for TCP connection...\n");
		rc = modbus_tcp_accept(mb_tcp,&mb_tcp_socket);		
  	if (rc == -1) {
 			fprintf(stderr, "Modbus TCP error: %s\n",modbus_strerror(errno));
			continue;
  	}
		//printf("End of Accept\n");
		switch(fork()){
		case 0:
			//printf("Child\n");
			printf("Connection accepted and used by child PID: %d\n",getpid());
 			rc = process_tcp_request(mb_tcp, mb_rtu, mb_mapping, rtu_lock, offset);
			close(mb_tcp_socket);
			printf("Server child PID %d died\n",getpid());
			/*DIE! DIE! DIE! my sweet child:)*/
			return rc;
			break;

		default:
			//printf("Parent\n");	
			break;

		case -1:
			perror("Error during fork\n");
			break;
		}
	}

  sem_close(rtu_lock);
	modbus_mapping_free(mb_mapping);

  modbus_close(mb_rtu);
  modbus_free(mb_rtu);

  modbus_close(mb_tcp);
  modbus_free(mb_tcp);

	return EXIT_SUCCESS;
}

int parseopts(int argc, char *argv[], config_t *opts)
{
	/*
		TODO
		Write options validation 
	*/
	int c;	

	while ((c = getopt (argc, argv, "hvdS:b:y:B:n:A:p:s:l:")) != -1){
		switch(c){
		case 's':
			opts->slave_addr=atoi(optarg);
		break;
		case 'S':
			opts->usb_dev=optarg;
		break;
		case 'b':
			opts->baudrate=atoi(optarg);
		break;
		case 'y':
			opts->parity=optarg[0];
		break;
		case 'n':
			opts->bits=atoi(optarg);
		break;
		case 'B':
			opts->stop_bits=atoi(optarg);
		break;
		case 'A':
			opts->ip=optarg;
		break;
		case 'p':
			opts->port=atoi(optarg);
		break;
		case 'v':
			opts->debug=1;
		break;
		case 'l':
			opts->log_file=optarg;
		break;
		case 'd':
			opts->daemonize=1;
		break;
		case 'h':
    default:
	    fprintf(stderr, "Usage: %s [options]\n",argv[0] );
	    fprintf(stderr, "Possible options:\n" );
	    fprintf(stderr, "-s rtu_slave_address (default %d)\n",MODBUS_RTU_SLAVE );
	    fprintf(stderr, "-S usb_device (default %s)\n",USB_DEV );
	    fprintf(stderr, "-b baudrate (default %d)\n",BAUDRATE );
	    fprintf(stderr, "-y parity posible values: N,E,O (default %c)\n",PARITY);
	    fprintf(stderr, "-n number_of_transport_bits (default %d)\n",SERIAL_BITS);
	    fprintf(stderr, "-B number_of_stop_bits (default %d)\n",STOP_BITS);
	    fprintf(stderr, "-A ip_address to bind (default %s)\n",IP);
	    fprintf(stderr, "-p port to bind (default %d)\n",PORT);
	    fprintf(stderr, "-v be verbose if present\n");
	    fprintf(stderr, "-l log_file (NOT IMPLEMENTED)\n");
	    fprintf(stderr, "-d demonize if present (NOT IMPLEMENTED)\n");
			return -1;
		}
	}

	return 0;
}

int recive_data_from_rtu(modbus_t *rtu, uint8_t *req, 
																	 int offset, modbus_mapping_t *mb_mapping)
{
	int function;
  uint16_t address;	
	int nb_data;
	int status;
	int i, j;

	function = req[offset];
  address = (req[offset + 1] << 8) + req[offset + 2];	

	/*
		for *_READ functions nb_data => number of registers/colis to read
		for *_WRITE_SINGLE functions nb_data => data to write to register/coil
		for *_WRITE_MULTIPLE functions nb_data => number of registers/coils to write
	*/ 
	nb_data = (req[offset + 3] << 8) + req[offset + 4];

	switch(function){
	case _FC_READ_COILS: {
		status = modbus_read_bits(rtu,address,nb_data,&mb_mapping->tab_bits[address]);
	}
		break;
 	case _FC_READ_DISCRETE_INPUTS: {
		status = modbus_read_input_bits(rtu,address,nb_data,
							&mb_mapping->tab_input_bits[address]);
	}
		break;
	case _FC_READ_HOLDING_REGISTERS: {
		status = modbus_read_registers(rtu,address,nb_data,
								&mb_mapping->tab_registers[address]);
	}	
		break;
	case _FC_READ_INPUT_REGISTERS: {
		status = modbus_read_input_registers(rtu,address,nb_data,
							&mb_mapping->tab_input_registers[address]);
	}
		break;
	case _FC_WRITE_SINGLE_COIL: {
		status = modbus_write_bit(rtu,address,nb_data);	
		mb_mapping->tab_bits[address] = nb_data;
	}
		break;
	case _FC_WRITE_SINGLE_REGISTER: {
		status = modbus_write_register(rtu,address,nb_data);	
		mb_mapping->tab_bits[address] = nb_data;
	}
		break;
	case _FC_WRITE_MULTIPLE_COILS: {
		status = modbus_write_bits(rtu,address,nb_data,&req[offset+6]);
		modbus_set_bits_from_bytes(mb_mapping->tab_bits, address, nb_data, &req[offset + 6]);
	}
		break;
	case _FC_WRITE_MULTIPLE_REGISTERS: {
		for (i = address, j = 6; i < address + nb_data; i++, j += 2) {
    	/* 6 and 7 = first value */
      mb_mapping->tab_registers[i] =
   		 (req[offset + j] << 8) + req[offset + j + 1];
  	}		
		status = modbus_write_registers(rtu,address,nb_data,
								&mb_mapping->tab_registers[address]);
	}
		break;
	case _FC_READ_AND_WRITE_REGISTERS: {
		uint16_t address_write = (req[offset + 5] << 8) + req[offset + 6];
 		int nb_write = (req[offset + 7] << 8) + req[offset + 8];
		
		/* Write first 10 and 11 are the offset of the first values to write */
    for (i = address_write, j = 10; i < address_write + nb_write; i++, j += 2) {
  			mb_mapping->tab_registers[i] = (req[offset + j] << 8) + req[offset + j + 1];
    }

		status = modbus_read_and_write_registers(rtu,address,nb_data,
											&mb_mapping->tab_registers[address],
											address_write,nb_write,
											&mb_mapping->tab_registers[address_write]);
	}
		break;

	default:
		/*we don`t have to worry about not implemented functions, modbus_reply
			will do it for us*/
		status = 1;
	}

	return status;
}

int process_tcp_request(modbus_t *mb_tcp, modbus_t *mb_rtu, 
												modbus_mapping_t *mb_mapping, sem_t *rtu_lock,int offset)
{
	int pid;
	int query_size;
	uint8_t *req;
	req = malloc(MODBUS_TCP_MAX_ADU_LENGTH);
	pid = getpid();

	if(req == NULL){
 		fprintf(stderr, "process_tcp_request : Failed to allocate memory.\n");
		return -2;
	}

  for (;;) {
		query_size = modbus_receive(mb_tcp, -1, req);

		if (query_size != -1) {

			/*
				communicate with RTU unit
				most of the serial lines device cannot serv for multiple clients
				we use semaphore to lock it and wait for responce
			*/
			sem_wait(rtu_lock);
			//fprintf(stderr, "Child PID:%d enters critical\n ", pid);
			query_size = recive_data_from_rtu(mb_rtu, req, offset, mb_mapping);
			sem_post(rtu_lock);
 			//fprintf(stderr, "Child PID:%d exits critical\n ", pid);

			if( query_size > 0){
				modbus_reply(mb_tcp,req,query_size,mb_mapping);			
			}else{
 				//fprintf(stderr, "RTU Slave error PID:%d: %s\n", pid,
 				fprintf(stderr, "RTU Slave error: %s\n",
   					 modbus_strerror(errno));
				modbus_reply_exception(mb_tcp, req, errno);
			}
		} else {
			if (errno == ECONNRESET) {
				//printf("Connection closed by child PID: %d\n",pid);
				printf("Connection closed by client or child\n");
				/* Connection closed by the client, end of CHILD server */
				break;
			} else {
				fprintf(stderr,"Modbus TCP error: %s\n" ,modbus_strerror(errno));
			}
		}
	}

	free(req);
	return query_size;
}
