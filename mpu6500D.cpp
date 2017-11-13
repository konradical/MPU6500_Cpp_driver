#include <iostream>
using namespace std;

/* main.cpp
 *
 *  Created on: Aug 24, 2015
 *      Author: konradantoniuk
 */

#include <stdio.h>
#include <stdexcept>
#include <sys/types.h>
#include <stdlib.h>
#include "dmpKey.h"
#include "inv_mpu_dmp_motion_driver1.hpp"
#include "inv_mpu1.hpp"
#include "inv_mpu_dmp_motion_driver2.hpp"
#include "inv_mpu2.hpp"
#include "mraa.hpp"
#include "I2Cme.hpp"

/** additional headers from getopt example
http://www.gnu.org/software/libc/manual/html_node/Example-of-Getopt.html#Example-of-Getopt
*/
#include <ctype.h>
#include <unistd.h>
//add in the math for time calculation from string
#include <math.h>

#define uint8_t unsigned char
//#define delay_ms(a)    usleep((uint8_t)(a*1000))
#define DIM 3
#define int32_t long
#define int16_t short
// MPU control/status vars
	uint8_t devStatus;      // return status after each device operation
	//(0 = success, !0 = error)
	uint8_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	//int16_t a[3];              // [x, y, z]            accel vector
	short a[3];
	//int16_t g[3];              // [x, y, z]            gyro vector
	short g[3];
	int32_t _q[4];
	int32_t t;
	int16_t c[3];

	//VectorFloat gravity;    // [x, y, z]            gravity vector

	int r;
	int initialized = 0;
	int dmpReady = 0;
	float lastval[3];
	unsigned char sensors;

	float ypr[3];
	//Quaternion q;
	float temp;
	float gyro[3];
	float accel[3];
	float compass[3];

	uint8_t rate = 100;


	static signed char gyro_orientation[9] = {-1, 0, 0,
	                                           0,-1, 0,
	                                           0, 0, 1};

	/* These next two functions converts the orientation matrix (see
	 * gyro_orientation) to a scalar representation for use by the DMP.
	 * NOTE: These functions are borrowed from Invensense's MPL.
	 */
	static inline unsigned short inv_row_2_scale(const signed char *row)
	{
	    unsigned short b;

	    if (row[0] > 0)
	        b = 0;
	    else if (row[0] < 0)
	        b = 4;
	    else if (row[1] > 0)
	        b = 1;
	    else if (row[1] < 0)
	        b = 5;
	    else if (row[2] > 0)
	        b = 2;
	    else if (row[2] < 0)
	        b = 6;
	    else
	        b = 7;      // error
	    return b;
	}

	static inline unsigned short inv_orientation_matrix_to_scalar(
	    const signed char *mtx)
	{
	    unsigned short scalar;

	    /*
	       XYZ  010_001_000 Identity Matrix
	       XZY  001_010_000
	       YXZ  010_000_001
	       YZX  000_010_001
	       ZXY  001_000_010
	       ZYX  000_001_010
	     */

	    scalar = inv_row_2_scale(mtx);
	    scalar |= inv_row_2_scale(mtx + 3) << 3;
	    scalar |= inv_row_2_scale(mtx + 6) << 6;


	    return scalar;
	}

	void led_setup(mraa::I2c* i2c){
			i2c = new mraa::I2c(0);
			i2c->address(0x09);
			i2c->frequency(MRAA_I2C_STD);
	}

	void led_change(mraa::I2c* i2c,uint8_t r, uint8_t g, uint8_t b){
		uint8_t buffer[4]  = {'n',r,g,b};
			i2c->write(buffer, 4);

	}

int main(int argc, char** argv) {

	/**
	 * implmenting getopt function to allow input for sampling time
	 *
	 * */
	  int t_flag = 0;
	  char *cvalue = NULL;
	  int index;
	  int c;

	  opterr = 0;

	  //only option is going to be -t for time followed by specified sampling period
	  while ((c = getopt (argc, argv, "t:")) != -1)
	    switch (c)
	      {
	      case 't':
	    	t_flag = 1;
	        cvalue = optarg;
	        break;
	      case '?':
	        if (optopt == 't')
	          fprintf (stderr, "Option -%c requires an argument.\n", optopt);
	        else if (isprint (optopt))
	          fprintf (stderr, "Unknown option `-%c'.\n", optopt);
	        else
	          fprintf (stderr,
	                   "Unknown option character `\\x%x'.\n",
	                   optopt);
	        return 1;
	      default:
	        abort ();
	      }
	  printf ("period value = %s\n", cvalue);

	  for (index = optind; index < argc; index++)
	    printf ("Non-option argument %s\n", argv[index]);

	  //interpret passed in value here, calculate # of samples
	  //reusing free variable index
	  index = 0;
	  int number = 0;
	  int num_samples = 500; //default value if time isn't specified
	  if (t_flag == 1)
	  {
		  while (cvalue[index] != '\0' && index < 5)
			  index++;
		  if (index == 5)
			  return 1;
		  int tmp = --index;
		  for(;index > 0; index--)
			  number += ((cvalue[index] % 48)*pow(10,tmp-index));
		  printf("number of seconds = %d\n",number);
		  int seconds = number; //of seconds
		  num_samples = seconds * rate;
		  printf("number of samples = %d\n",num_samples);
	  }
	/**
	 * samples goes on to be substituted into loop for sensor sampling
	 * */


	/**
	 * original code from mpu6500C follows
	 * */

		FILE * fp;

	   fp = fopen ("/home/root/data.txt", "w+");
	   if (fp == NULL)
		   printf("whoops\n");
	   //fprintf(fp,"working");//%d",rate);

		mraa::I2c* i2c_led;
		i2c_led = new mraa::I2c(0);
		i2c_led->address(0x09);
		i2c_led->frequency(MRAA_I2C_STD);
		led_change(i2c_led,0,0,50);
		usleep(1000000);
		led_change(i2c_led,0,50,0);
		dmpReady=1;

		i2c_sensor_init();

		unsigned char POWER;
/*		int reslt = mpu_run_self_test(g, a);

		int ff = 0b010;
		if(ff)
			printf("my test works\n");
		if (reslt & 0b001)
			printf("gyro works\n");
		if (reslt & 0b010)
			printf("accel works\n");
		if (reslt & 0b100)
			printf("compass tests passed\n");
*/

		short unsigned rate;
		short unsigned int dmp_features;
		int samples;

{
	using namespace inv2;

				if (mpu_init())
				return -1;

				  /* Get/set hardware configuration. Start gyro. */
				  /* Wake up all sensors. */
				  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

				  /* Push both gyro and accel data into the FIFO. */
				  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
				  mpu_set_sample_rate(100); //DEFAULT_MPU_HZ);

			//	  dmp_load_motion_driver_firmware();
				  dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
				  dmp_register_tap_cb(NULL);

				  dmp_features = (DMP_FEATURE_6X_LP_QUAT |
												 DMP_FEATURE_SEND_RAW_ACCEL |
												 DMP_FEATURE_SEND_CAL_GYRO);
				  dmp_enable_feature(dmp_features);

				  //dmp_set_fifo_rate(100); //DEFAULT_MPU_HZ);
				  //mpu_set_dmp_state(1);


				if (mpu_get_power_state(&POWER))
					return -1;
				if(POWER)
					printf("POWER IS ON: %c\n", POWER);

				//reading data starts to take place here
				if (!dmpReady) {
						printf("Error: DMP not ready!!\n");
						return -1;
					}

				samples = 0;
				mpu_get_sample_rate(&rate);
				//int more = 0;
				printf("rate: %d\n",rate);
}

{
	using namespace inv1;

		if (mpu_init())
			return -1;

		  /* Get/set hardware configuration. Start gyro. */
		  /* Wake up all sensors. */
		  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

		  /* Push both gyro and accel data into the FIFO. */
		  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
		  mpu_set_sample_rate(100); //DEFAULT_MPU_HZ);

	//	  dmp_load_motion_driver_firmware();
		  dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
		  dmp_register_tap_cb(NULL);

		  dmp_features = (DMP_FEATURE_6X_LP_QUAT |
		                                 DMP_FEATURE_SEND_RAW_ACCEL |
		                                 DMP_FEATURE_SEND_CAL_GYRO);
		  dmp_enable_feature(dmp_features);

		  //dmp_set_fifo_rate(100); //DEFAULT_MPU_HZ);
		  //mpu_set_dmp_state(1);


		if (mpu_get_power_state(&POWER))
			return -1;
		if(POWER)
			printf("POWER IS ON: %c\n", POWER);

		//reading data starts to take place here
		if (!dmpReady) {
				printf("Error: DMP not ready!!\n");
				return -1;
			}

		samples = 0;
		mpu_get_sample_rate(&rate);
		//int more = 0;
		printf("rate: %d\n",rate);
}
		sleep(2);
		int wait = 1000/rate;
		led_change(i2c_led,50,0,0);

		inv1::mpu_read_fifo(g,a,&sensors,&fifoCount);
		inv2::mpu_read_fifo(g,a,&sensors,&fifoCount);
		for (int i = 0; i < 8; i++)
			inv1::mpu_read_fifo(g,a,&sensors,&fifoCount);
		while (samples < num_samples) {
			//usleep(200);
			inv1::delay_ms(wait-2);

			inv1::mpu_read_fifo(g,a,&sensors,&fifoCount);
			fprintf(fp,"accel1:\t %6d\t %6d\t %6d\tgyro1:\t %6d\t %6d\t %6d\t", a[0], a[1], a[2], g[0], g[1], g[2]);
			inv2::mpu_read_fifo(g,a,&sensors,&fifoCount);
			fprintf(fp,"accel2:\t %6d\t %6d\t %6d\tgyro2:\t %6d\t %6d\t %6d\n", a[0], a[1], a[2], g[0], g[1], g[2]);

			//more = fifoCount;
		//printf("samples: %d\n",samples);

			//mpu_read_fifo(g,a,&sensors,&fifoCount);
			samples++;
		}
		led_change(i2c_led,50,0,50);
		//printf("more: %d\nsamples: %d\n",more,samples);
		printf("exit\n");
		fclose(fp);

		return 0;
}
