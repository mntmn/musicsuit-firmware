#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

static int file;

/*
 * https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.cpp
 * http://www.i2cdevlib.com/docs/html/class_i2_cdev.html#aa68890af87de5471d32e583ebbd91acb
 * https://github.com/Harinadha/STM32_MPU6050lib/blob/master/MPU6050.h
 */

#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_FIFO_EN          0x23

#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C

void select_reg_i2c(char reg) {
    if (write(file,&reg,1) != 1) {
        printf("Failed to write to the i2c bus.\n");
        printf(strerror(errno));
        printf("\n\n");
	exit(1);
    }
}

void read_i2c(char* buf, char reg, int len) {
    select_reg_i2c(reg);

    if (read(file,buf,len) != len) {
	printf("Failed to read from the i2c bus.\n");
	printf(strerror(errno));
	printf("\n\n");
	exit(1);
    }
}

void write_i2c_byte(char reg, char content) {
    char buf[2] = {reg,content};

    if (write(file,&buf,2) != 2) {
        printf("Failed to write to the i2c bus.\n");
        printf(strerror(errno));
        printf("\n\n");
	exit(1);
    }
}

void get_motion_6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    char buffer[14] = {0};
    read_i2c(buffer, MPU6050_RA_ACCEL_XOUT_H, 14);

    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

int
main(void) {
    char filename[40];
    int addr = 0x68;

    sprintf(filename,"/dev/i2c-2");
    if ((file = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.");
        exit(1);
    }

    if (ioctl(file,I2C_SLAVE,addr) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        exit(1);
    }

    int16_t ax,ay,az;
    int16_t gx,gy,gz;

    // bit 4+5 = 00 for FS_250
    write_i2c_byte(MPU6050_RA_SMPLRT_DIV,0x07);
    write_i2c_byte(MPU6050_RA_CONFIG,0);
    write_i2c_byte(MPU6050_RA_GYRO_CONFIG,0b00001000);
    write_i2c_byte(MPU6050_RA_ACCEL_CONFIG,0);
    write_i2c_byte(MPU6050_RA_FF_THR,0);
    write_i2c_byte(MPU6050_RA_FF_DUR,0);
    write_i2c_byte(MPU6050_RA_MOT_THR,0);
    write_i2c_byte(MPU6050_RA_MOT_DUR,0);
    write_i2c_byte(MPU6050_RA_ZRMOT_THR,0);
    write_i2c_byte(MPU6050_RA_ZRMOT_DUR,0);
    write_i2c_byte(MPU6050_RA_FIFO_EN,0);
    write_i2c_byte(MPU6050_RA_SIGNAL_PATH_RESET,0);
    write_i2c_byte(MPU6050_RA_MOT_DETECT_CTRL,0);
    write_i2c_byte(MPU6050_RA_USER_CTRL,0);
    write_i2c_byte(MPU6050_RA_PWR_MGMT_1,0b00000010); 
    write_i2c_byte(MPU6050_RA_PWR_MGMT_2,0b00000000); 

    while (1) {
	get_motion_6(&ax, &ay, &az, &gx, &gy, &gz);
	printf("%d %d %d | %d %d %d\n",ax,ay,az, gx,gy,gz);
    }

    return 0;
}
