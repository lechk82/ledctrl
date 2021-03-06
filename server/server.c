/*
 * server.c
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <glob.h>
#include "jsonrpc-c.h"


#define ROWS 16
#define MIN_cols 1
#define MAX_cols 255

#define JSON_LEVEL_ROWS	2
#define JSON_LEVEL_COLS	3
#define JSON_LEVEL_RGB	4

#define JSON_OK_STR "OK"
#define JSON_ERR_STR "ERROR"

#define DRVNAME "/dev/stm32led"

typedef int bool;
enum { false, true };

typedef union {
  uint32_t rgba32;
  struct rgba_t {
    unsigned b : 8;
    unsigned g : 8;
    unsigned r : 8;
    unsigned a : 8;
  } rgba;
} rgba32_t;

uint32_t GpioBase0 = 0;
uint32_t GpioBase1 = 0;

const uint8_t gamma8[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

uint32_t port = 0;

rgba32_t *rgbbuf = NULL;
rgba32_t *rgbbufbase = NULL;
uint32_t cols = 0;
uint32_t col = 0;
uint32_t row = 0;

struct jrpc_server my_server;
bool initLEDdone = false;
bool gammaSet = false;
bool busy = true;

/*** Prototypes ***/
uint32_t getGpioBase(uint8_t reg);
uint32_t exportGpios(uint32_t base, uint32_t n);
uint32_t unexportGpios(uint32_t base, uint32_t n);
uint32_t setGpio(uint32_t base, uint32_t gpio, bool val);

cJSON * setEnable(jrpc_context * ctx, cJSON * params, cJSON *id);
cJSON * initLED(jrpc_context * ctx, cJSON * params, cJSON *id);
cJSON * setLED(jrpc_context * ctx, cJSON * params, cJSON *id);
cJSON * setGamma(jrpc_context * ctx, cJSON * params, cJSON *id);
cJSON * exit_server(jrpc_context * ctx, cJSON * params, cJSON *id);
void callback(cJSON *item, uint8_t level, uint16_t iter);
void parse_and_callback(cJSON *item, uint8_t level, uint16_t iter);
int fast_atoi( const char * str );

/*** Main ***/

int main(int argc, char* argv[]){
	
	if(argc != 3){
		printf("Usage: %s <numCols,1:255> <port>\n",argv[0]);
		exit(1);
	}
	else{
		cols=strtol(argv[1], NULL, 10);
		port=strtol(argv[2], NULL, 10);
	    if (!(cols >= MIN_cols && cols <= MAX_cols)) {
    		printf("INIT: Argument 'cols' illegal value: %d. Valid: %d - %d.\n",cols,MIN_cols,MAX_cols);
    		exit(1);
    	}
	}
	
    /*
    // Gpio test
    uint32_t base = getGpioBase(21);
    exportGpios(base,8);
    setGpio(base,0,false);
    unexportGpios(base,8);
    */
    
    
    
	//server stuff
	jrpc_server_init(&my_server, port);
    jrpc_register_procedure(&my_server, setEnable, "e", NULL );
	jrpc_register_procedure(&my_server, initLED, "i", NULL );
	jrpc_register_procedure(&my_server, setLED, "s", NULL );
	jrpc_register_procedure(&my_server, setGamma, "y", NULL );
	jrpc_register_procedure(&my_server, exit_server, "exit", NULL );
	printf("Listening on port %d\n",port); fflush(stdout);
	jrpc_server_run(&my_server);
	jrpc_server_destroy(&my_server);
    
	free(rgbbuf);
	return 0;
}

/*** Functions ***/

uint32_t getGpioBase(uint8_t reg){
glob_t globbuf;
FILE * fp = NULL;
char buf[255];
uint32_t base=0;
    
    sprintf(buf, "/sys/class/i2c-dev/i2c-1/device/1-00%2d/gpio/gpiochip*/base", reg);
    glob(buf, 0, NULL, &globbuf);
    if(globbuf.gl_pathc == 1){
        fp = fopen (globbuf.gl_pathv[0],"r");
        if(fp){
            fscanf(fp, "%s", buf);
            base = atoi(buf);
            fclose (fp);
        }
    }
    else{
        globfree(&globbuf);
        printf("Error: No  gpiochip base found for reg 0x%d\n",reg); fflush(stdout);
        exit(1);
    }
    globfree(&globbuf);
    return base;
}

uint32_t exportGpios(uint32_t base, uint32_t n){
FILE * fp = NULL;
char buf1[16];
char buf2[255];
char buf3[255];
uint32_t err = 0;
    for(int i=0;i<n;i++){
        sprintf(buf2, "/sys/class/gpio/gpio%d", base+i);
        sprintf(buf3, "/sys/class/gpio/gpio%d/direction", base+i);
        if( access(buf2, F_OK ) == -1 ) {
            printf("export: %s\n",buf2); fflush(stdout);
            fp = fopen("/sys/class/gpio/export", "w");
            if(fp){
                sprintf(buf1, "%d", base+i); 
                fwrite(buf1, 1 , sizeof(buf1) , fp );
                fclose (fp);
            }
            else err++;
            while(access(buf3, F_OK ) == -1);
            fp = NULL;
            fp = fopen(buf3, "w");
            if(fp){
                sprintf(buf1, "high"); 
                fwrite(buf1, 1 , sizeof(buf1) , fp );
                fclose (fp);
            }
            else err++;
        }
    }
    return err;
}

uint32_t unexportGpios(uint32_t base, uint32_t n){
FILE * fp = NULL;
char buf1[16];
char buf2[255];
char buf3[255];
uint32_t err = 0;
    for(int i=0;i<n;i++){
        sprintf(buf2, "/sys/class/gpio/gpio%d", base+i);
        sprintf(buf3, "/sys/class/gpio/gpio%d/value", base+i);
        if( access(buf2, W_OK ) != -1 ) {
            fp = fopen(buf3, "w");
            if(fp){
                sprintf(buf1, "%d", 1); 
                fwrite(buf1, 1 , sizeof(buf1) , fp );
                fclose (fp);
                printf("unexport: %s\n",buf2); fflush(stdout);
            }
            else err++;
            if( access(buf3, W_OK ) != -1 ) {
                fp = NULL;
                fp = fopen("/sys/class/gpio/unexport", "w");
                if(fp){
                    sprintf(buf1, "%d", base+i); 
                    fwrite(buf1, 1 , sizeof(buf1) , fp );
                    fclose (fp);
                }
                else err++;
            }
            else err++;
        }
        else err++;
    }
    return err;
}

uint32_t setGpio(uint32_t base, uint32_t gpio, bool val){
FILE * fp = NULL;
char buf1[16];
char buf2[255];
uint32_t err = 0;
    sprintf(buf2, "/sys/class/gpio/gpio%d/value", base+gpio);
    if( access(buf2, W_OK ) != -1 ) {
        if(val == true){
            sprintf(buf1, "%d", 1); 
        }
        if(val == false){
            sprintf(buf1, "%d", 0); 
        }
        fp = fopen(buf2, "w");
        if(fp){
            fwrite(buf1, 1 , sizeof(buf1) , fp );
            fclose (fp);
        }
        else err++;
    }
    else err++;
    return err;
}

cJSON * setEnable(jrpc_context * ctx, cJSON * params, cJSON *id) {
    setGpio(GpioBase0, fast_atoi(params->child->string), !(bool) params->child->valueint);
    return cJSON_CreateNumber(true);
}

void callback(cJSON *item, uint8_t level, uint16_t iter){
uint32_t i;
char str[8];
	switch(level){
		case JSON_LEVEL_ROWS:
			for(i=0;i<ROWS;i++){
				if(fast_atoi(item->string)==i) {row=i; break;}
			}
		break;
		case JSON_LEVEL_COLS:
			for(i=0;i<cols;i++){
				if(fast_atoi(item->string)==i) {col=i; break;}
			}
		break;
		case JSON_LEVEL_RGB:
			rgbbuf=rgbbufbase+row*cols+col;
			if(!strcmp(item->string, "r")){
				if(gammaSet)
					rgbbuf->rgba.r=(uint8_t)gamma8[item->valueint];
				else
					rgbbuf->rgba.r=(uint8_t)item->valueint;
			}
			else if(!strcmp(item->string, "g")){
				if(gammaSet)
					rgbbuf->rgba.g=(uint8_t)gamma8[item->valueint];
				else
					rgbbuf->rgba.g=(uint8_t)item->valueint;
			}
			else if(!strcmp(item->string, "b")){
				if(gammaSet)
					rgbbuf->rgba.b=(uint8_t)gamma8[item->valueint];
				else
					rgbbuf->rgba.b=(uint8_t)item->valueint;
			}
		break;
	}
}

void parse_and_callback(cJSON *item, uint8_t level, uint16_t iter){
uint8_t local_level=level+1;
uint16_t local_iter = iter;
    while(item){
        callback(item,local_level,local_iter);
        if (item->child){
            parse_and_callback(item->child,local_level,iter);
        }
        item = item->next;
        local_iter++;
    }
}

cJSON * initLED(jrpc_context * ctx, cJSON * params, cJSON *id) {
uint32_t ret,fd;
uint8_t err = 0;

	//init buffer
	rgbbuf = (rgba32_t*) malloc(sizeof(rgba32_t*)*cols*ROWS);
	if (rgbbuf==NULL) exit(1);
	rgbbufbase=rgbbuf;
	memset(rgbbuf,0,sizeof(rgba32_t)*cols*ROWS);
	
	//open device and clear leds
	fd = open(DRVNAME, O_RDWR);  
	if (fd < 0){
		printf("INIT: Failed to open %s\n",DRVNAME); fflush(stdout);
		err++;
   	}
   	if(!err){
   		ret = write(fd,rgbbufbase,cols*ROWS);
   		
		if(ret != cols*ROWS){
			printf("INIT: Write Error on %s\n",DRVNAME); fflush(stdout);
			err++;
		}
		close(fd);
		
	}
    
    GpioBase0 = getGpioBase(21);
    GpioBase1 = getGpioBase(22);
    ret += exportGpios(GpioBase0,8);
    ret += exportGpios(GpioBase1,8);
    
	if(err){
        printf("INIT: FAILED\n"); fflush(stdout);
		initLEDdone = false;
		return cJSON_CreateNumber(false);
	}
	else{
		printf("INIT: DONE\n"); fflush(stdout);
		initLEDdone = true;
		return cJSON_CreateNumber(true);
	}
}

cJSON * exit_server(jrpc_context * ctx, cJSON * params, cJSON *id){
	jrpc_server_stop(&my_server);
    unexportGpios(GpioBase0,8);
    unexportGpios(GpioBase1,8);
	printf("SERVER: EXIT\n"); fflush(stdout);
	return cJSON_CreateNumber(true);
}

cJSON * setLED(jrpc_context * ctx, cJSON * params, cJSON *id) {
uint32_t ret,fd;
uint8_t err = 0;

	if(initLEDdone){
		parse_and_callback(params,0,0);
		fd = open(DRVNAME, O_RDWR);  
		if (fd < 0){
			printf("SETLED: Failed to open %s\n",DRVNAME); fflush(stdout);
			err++;
		}
		if(!err){
			ret = write(fd,rgbbufbase,cols*ROWS);
			if(ret != cols*ROWS){
				printf("SETLED: Write Error on %s\n",DRVNAME); fflush(stdout);
				err++;
			}
			close(fd);
		}
	}
	
	if(err | !initLEDdone)
		return cJSON_CreateNumber(false);
	else
		return cJSON_CreateNumber(true);
}

cJSON * setGamma(jrpc_context * ctx, cJSON * params, cJSON *id) {
	if(params->child->valueint){
		printf("GAMMA: ENABLED\n"); fflush(stdout);
		gammaSet = true;
	}
	else{
		printf("GAMMA: DISABLED\n"); fflush(stdout);
		gammaSet = false;
	}
	return cJSON_CreateNumber(true);
}

/*** Helper functions ***/

int fast_atoi( const char * str ){
    int val = 0;
    while( *str ) {
        val = val*10 + (*str++ - '0');
    }
    return val;
}