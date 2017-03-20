#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>        
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>          // Required for the copy to user function  
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#define DRVNAME "stm32led"
#define CLASS_NAME "stm32led"
 
#define MY_BUS_NUM 0
static struct spi_device *spi_device;
#define DRVNAME "stm32led"

#define MIN_COLS 1
#define MAX_COLS 255
#define ROWS 16
#define HANDSHAKE 0xaa

typedef union {
  uint16_t val16;
  struct bytes_t {
    unsigned msb : 8;
    unsigned lsb : 8;
  } bytes;
} val16_t;


static bool silent = false;
static val16_t cols16;
static unsigned cols;
static unsigned reset_pin = 26;
module_param(silent, bool, 0);
module_param(cols, uint, 0444);
module_param(reset_pin, uint, 0);
MODULE_PARM_DESC(cols, " Number of columns (0-255)");
MODULE_PARM_DESC(silent, " Disables led wakeup sequence (1,0), default: 0");
MODULE_PARM_DESC(reset_pin, " stm32 gpio reset pin, default: 26");

static struct gpio pins[1];

static uint8_t initdata[3];
static uint8_t initdata_ret[3];

static int32_t led_bufsize = 0;
static uint16_t *ledbuf;

static int majorNumber;

static struct class* stm32ledCharClass = NULL;
static struct device* stm32ledCharDevice =NULL;

// The prototype functions for the character driver -- must come before the struct definition
static int dev_open(struct inode *, struct file *);
static int dev_release(struct inode *, struct file *);

static ssize_t dev_write_all(struct file *, const uint32_t *, size_t, loff_t *);

static void device_spi_delete(struct spi_master *master, unsigned cs);
static void ledSetPixel(uint8_t row, uint16_t column, uint8_t red, uint8_t green, uint8_t blue, uint16_t *pbuf);
static int ledClearAllPixel(void);
static int ledWakeUpSequence(void);

static struct file_operations fopsall = {
	.open = dev_open, .write = (void *)dev_write_all, .release = dev_release
};

struct spi_board_info spi_device_info = {
	.modalias = DRVNAME,
	.max_speed_hz = 20000000, //speed your device (slave) can handle
	.bus_num = MY_BUS_NUM,
	.chip_select = 0,
	.mode = 0,
};
 
static int __init stm32led_init(void){
    int ret;
    struct spi_master *master;
    cols16.val16 = cols;
    
    if (!(cols16.val16 >= MIN_COLS && cols16.val16 <= MAX_COLS)) {
    	pr_err(DRVNAME": Argument 'cols16' illegal value: %d. Valid: %d - %d.\n",cols16.val16,MIN_COLS,MAX_COLS);
    	return -EINVAL;
    }
    
    led_bufsize=cols16.val16*8*3;
    ledbuf = kmalloc(sizeof(uint16_t)*led_bufsize, GFP_KERNEL);
    
    if (!ledbuf){
     	pr_err(DRVNAME": Error allocating %d Bytes for memory buffer\n",led_bufsize);
     	return -EINVAL;
	}
     	
    //Register information about your slave device:
    master = spi_busnum_to_master( spi_device_info.bus_num );
    if( !master ){
    	kfree(ledbuf);
        pr_err(DRVNAME": SPI Master not found.\n");
        return -ENODEV;
    }
    
    device_spi_delete(master, spi_device_info.chip_select);
     
    // create a new slave device, given the master and device info
    spi_device = spi_new_device( master, &spi_device_info );
 
    if( !spi_device ) {
    	kfree(ledbuf);
        pr_err(DRVNAME": FAILED to create SPI slave.\n");
        return -ENODEV;
    }
     
    spi_device->bits_per_word = 8;
 
    ret = spi_setup( spi_device );
     
    if(ret){
    	kfree(ledbuf);
        pr_err(DRVNAME": FAILED to setup SPI slave.\n");
        spi_unregister_device( spi_device );
        return -ENODEV;
    }
    
    // Register sm32led reset pin as gpio
    pins[0].gpio=reset_pin;
    pins[0].flags=GPIOF_OUT_INIT_HIGH;
    pins[0].label = "STM32RESET";
    ret = gpio_request_array(pins, ARRAY_SIZE(pins));
    if (ret) {
		pr_err(DRVNAME": Unable to request GPIO Pin: %d\n", ret);
		kfree(ledbuf);
		gpio_free_array(pins, ARRAY_SIZE(pins));
		if(spi_device){spi_unregister_device( spi_device );}
		return -ENODEV;
	}
	
	// reset stm32
	gpio_set_value(pins[0].gpio, 0);
	msleep(50);
	gpio_set_value(pins[0].gpio, 1);
	msleep(200);
    
    // init stm32
    initdata[0] = HANDSHAKE;
    initdata[1] = cols16.bytes.msb;
    initdata[2] = cols16.bytes.lsb;
    ret = spi_write(spi_device, &initdata, 3);
    msleep(50);
  	ret = spi_read(spi_device, &initdata_ret[0],1);
	ret = spi_read(spi_device, &initdata_ret[1],1);
	ret = spi_read(spi_device, &initdata_ret[2],1);
	if (ret){
		pr_err(DRVNAME": Unable to init stm32\n");
		kfree(ledbuf);
		gpio_free_array(pins, ARRAY_SIZE(pins));
		if(spi_device){spi_unregister_device( spi_device );}
		return -ENODEV;
	}
	if(initdata_ret[0] != HANDSHAKE){
		pr_err(DRVNAME": Unable to init stm32 (no handshake: %d)\n",initdata_ret[0]);
		kfree(ledbuf);
		gpio_free_array(pins, ARRAY_SIZE(pins));
		if(spi_device){spi_unregister_device( spi_device );}
		return -ENODEV;
	}
	if(initdata_ret[1] != cols16.bytes.msb){
		pr_err(DRVNAME": Unable to init stm32 (cols16.msb=%d received, must be cols16.msb=%d)\n",initdata_ret[1],cols16.bytes.msb);
		kfree(ledbuf);
		gpio_free_array(pins, ARRAY_SIZE(pins));
		if(spi_device){spi_unregister_device( spi_device );}
		return -ENODEV;
	}
	
	if(initdata_ret[2] != cols16.bytes.lsb){
		pr_err(DRVNAME": Unable to init stm32 (cols16.lsb=%d received, must be cols16.lsb=%d)\n",initdata_ret[2],cols16.bytes.lsb);
		kfree(ledbuf);
		gpio_free_array(pins, ARRAY_SIZE(pins));
		if(spi_device){spi_unregister_device( spi_device );}
		return -ENODEV;
	}
	
    msleep(50);
    
    ret = ledClearAllPixel();
    if(!ret){
    	if(!silent){
    		ret = ledWakeUpSequence();
    	}
    }
    if (ret){
		pr_err(DRVNAME": Unable to init stm32\n");
		kfree(ledbuf);
		gpio_free_array(pins, ARRAY_SIZE(pins));
		if(spi_device){spi_unregister_device( spi_device );}
		return -ENODEV;
	}
    

	// Register all device
	majorNumber = register_chrdev(0, DRVNAME, &fopsall);
	if (majorNumber<0){
		pr_err(DRVNAME": FAILED to register a major number\n");
		kfree(ledbuf);
		gpio_free_array(pins, ARRAY_SIZE(pins));
		spi_unregister_device( spi_device );
		return majorNumber;
	}
	stm32ledCharClass = class_create(THIS_MODULE, DRVNAME);
	if (IS_ERR(stm32ledCharClass)){                // Check for error and clean up if there is
		unregister_chrdev(majorNumber, DRVNAME);
		pr_err(DRVNAME": FAILED to register device class\n");
		kfree(ledbuf);
		gpio_free_array(pins, ARRAY_SIZE(pins));
		spi_unregister_device( spi_device );
		return PTR_ERR(stm32ledCharClass);          // Correct way to return an error on a pointer
	}
	stm32ledCharDevice = device_create(stm32ledCharClass, NULL, MKDEV(majorNumber, 0), NULL, DRVNAME);
	if (IS_ERR(stm32ledCharDevice)){               // Clean up if there is an error
		class_destroy(stm32ledCharClass);           // Repeated code but the alternative is goto statements
		unregister_chrdev(majorNumber, DRVNAME);
		pr_err(DRVNAME": FAILED to create the device\n");
		kfree(ledbuf);
		gpio_free_array(pins, ARRAY_SIZE(pins));
		spi_unregister_device( spi_device );
		return PTR_ERR(stm32ledCharDevice);
	}
	pr_info(DRVNAME": %s registered. cols16=%d\n",DRVNAME,cols16.val16);
    
	return 0;
}
 
 
static void __exit stm32led_exit(void){
	ledClearAllPixel();
    if(spi_device){
        spi_unregister_device( spi_device );
    }
    kfree(ledbuf);
    
	device_destroy(stm32ledCharClass, MKDEV(majorNumber, 0));     // remove the device
	class_unregister(stm32ledCharClass);                          // unregister the device class
	class_destroy(stm32ledCharClass);                             // remove the device class
	unregister_chrdev(majorNumber, DRVNAME);             // unregister the major number
	gpio_free_array(pins, ARRAY_SIZE(pins));
	pr_info(DRVNAME": Unregistered\n");
}

static void device_spi_delete(struct spi_master *master, unsigned cs){
struct device *dev;
char str[32];

	snprintf(str, sizeof(str), "%s.%u", dev_name(&master->dev), cs);
	dev = bus_find_device_by_name(&spi_bus_type, NULL, str);
	if (dev) {
		pr_info(DRVNAME": Deleting %s\n", str);
		device_del(dev);
	}
}

/*
	Dev open
*/


static int dev_open(struct inode *inodep, struct file *filep){
   return 0;
}


/*
	Dev write
*/

static ssize_t dev_write_all(struct file *filep, const uint32_t *buffer, size_t len, loff_t *offset){
int col, row;
int ret;
uint8_t r;
uint8_t g;
uint8_t b;
	
	if(len != ROWS*cols16.val16){
		return -1;
	}
	
	for(row=0;row<ROWS;row++){
		for(col=0;col<cols16.val16;col++){
			r=(uint8_t)((buffer[row*cols16.val16+col]>>16) & 0xff);
			g=(uint8_t)((buffer[row*cols16.val16+col]>>8) & 0xff);
			b=(uint8_t)(buffer[row*cols16.val16+col] & 0xff);

			ledSetPixel(row, col, r, g, b, ledbuf);
		}
	}
	
	ret = spi_write(spi_device, (uint8_t *)ledbuf, led_bufsize*2);
	if(ret)
		return ret;
	return len;
		
}

/*
	Dev Release
*/

static int dev_release(struct inode *inodep, struct file *filep){
   return 0;
}


/*
	Low-Level Functions
*/

static void ledSetPixel(uint8_t row, uint16_t column, uint8_t red, uint8_t green, uint8_t blue, uint16_t *pbuf){
uint8_t i;
uint32_t offset=column*3*8;
uint32_t pos_g;
uint32_t pos_r;
uint32_t pos_b;
uint16_t delmask = ~(0x0001<<row);

  for (i = 0; i < 8; i++){
    pos_g=offset+i;
    pos_r=pos_g+8;
    pos_b=pos_r+8;
    // clear the data for pixel
    pbuf[pos_g] &= delmask;
    pbuf[pos_r] &= delmask;
    pbuf[pos_b] &= delmask;
    // write new data for pixel
    pbuf[pos_g] |= (((((uint16_t)green<<(i+(ROWS-8))) & (0x0001<<(ROWS-1)))>>(ROWS-1))<<row);
    pbuf[pos_r] |= (((((uint16_t)red<<(i+(ROWS-8))) & (0x0001<<(ROWS-1)))>>(ROWS-1))<<row);
    pbuf[pos_b] |= (((((uint16_t)blue<<(i+(ROWS-8))) & (0x0001<<(ROWS-1)))>>(ROWS-1))<<row);
  }
}

static int ledClearAllPixel(void){
uint32_t row;
uint32_t col;
int ret;
	// clear leds
 	for(row=0;row<ROWS;row++){
 		for(col=0;col<cols16.val16;col++){
			ledSetPixel(row, col, 0, 0, 0, ledbuf);
		}
    }
   	ret = spi_write(spi_device, (uint8_t *)ledbuf, led_bufsize*2);
   	return ret;
}

static int ledWakeUpSequence(void){
uint32_t row;
uint32_t col;
uint8_t val;
int ret;
	// fade in
	val=0;
    while(val!=255){
		for(row=0;row<ROWS;row++){
			for(col=0;col<cols16.val16;col++){
				ledSetPixel(row, col, val, val, val, ledbuf);
			}
		}
		val++;
		ret = spi_write(spi_device, (uint8_t *)ledbuf, led_bufsize*2);
    }
    
    // fade out
	val=255;
    while(val!=0){
    	val--;
		for(row=0;row<ROWS;row++){
			for(col=0;col<cols16.val16;col++){
				ledSetPixel(row, col, val, val, val, ledbuf);
			}
		}
		ret = spi_write(spi_device, (uint8_t *)ledbuf, led_bufsize*2);
    }
    return ret;
}


module_init(stm32led_init);
module_exit(stm32led_exit);
 
MODULE_LICENSE("GPL");
MODULE_AUTHOR("LK");
MODULE_DESCRIPTION("stm32led module");
MODULE_VERSION("0.1");