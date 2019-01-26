#include "Accelometer.h"
#define GyroSens 				 131
#include <cmath>
#define M_PI        3.14159265358979323846


int16_t mpu_accel_x, mpu_accel_y, mpu_accel_z;
int16_t mpu_resultx, mpu_resulty, mpu_resultz;


/**
@brief Function reads any adress from MPU9562
@author Jakub Brzezowski
@version 1.0 2018-12-03
@param addr is addres of reading register
@retval result is data reads from register
*/
uint8_t read_register(uint8_t addr){
	  	uint8_t result = 0;

    // Generate START signal
    I2C_Start(); 

    // Send slave address followed by an Write bit
    I2C_Write(MPU9250_ADDRESS  | I2C_WRITE); // Uwaga! Mozna tez spróbowac podmienic adres urzadzenia na I2C_SLV0_ADDR.
	
    //  Send register address from we will be read
	  I2C_Wait();
		I2C_GetAck();
	
    I2C_Write(addr);
	  I2C_Wait();
		I2C_GetAck();
	
    // Repeat START signal in order to communicate in a 
    // reveice mode without releasing the bus
    I2C_RepeatedStart();
    // Send slave address followed by an Read bit
    I2C_Write(MPU9250_ADDRESS  | I2C_READ);
		 I2C_Wait();
		I2C_GetAck();
    // Now we can read from the previously selected register

    // Change transmit mode to Receive
    I2C_SetRXmode();
		I2C_SendNack();

    // Clean data register
    result = I2C_ReadByte();
    I2C_Wait();

		// Send NACK to indicate to the slave
    // that one last byte is required
		I2C_SendNack();
		// Read value from slave device register
    result = I2C_ReadByte();
    I2C_Wait();

		// Generate STOP signal
    I2C_Stop();
		result = I2C_ReadByte();
		//Wait for STOP signal to propagate
		//pause();

		return result;
}
/**
@brief Function write any adress from MPU9562
@author Jakub Brzezowski
@version 1.0 2018-12-03
@param addr is addres of writing register
@param data is data write toregister
@retval none
*/
void write_register(uint8_t addr, uint8_t data){
	I2C_Start();
	
	I2C_WriteByte(MPU9250_ADDRESS|I2C_WRITE);
	I2C_Wait();
	I2C_GetAck();
	
	I2C_WriteByte(addr);
	I2C_Wait();
	I2C_GetAck();
	
	I2C_WriteByte(data);
	I2C_Wait();
	I2C_GetAck();
	
	I2C_Stop();
	//pause();
}
/**
@brief Function reads X axis of accelometer
@author Jakub Brzezowski
@version 1.0 2018-12-06
@param data is variable, which store data.
@retval none
*/
float X_acc_read(void){
	
	//Virtual variable to read 8 byte registers.
	uint16_t data1, data2;
	float data;
	
	data1 = hal_dev_mma8451_read_reg(ACCEL_XOUT_H);
	data2 = hal_dev_mma8451_read_reg(ACCEL_XOUT_L);
	
	//Moving bits for high nible
	data1= data1<<8;
	
	return data = (data1+data2)* (8.0 / 65536.0) * 9.81;
	
}

/**
@brief Function reads Y axis of accelometer
@author Jakub Brzezowski
@version 1.0 2018-12-06
@param data is variable, which store data.
@retval none
*/
float Y_acc_read(void){
	
	//Virtual variable to read 8 byte registers.
	uint16_t data1, data2;
	float data;
	
	data1 = hal_dev_mma8451_read_reg(ACCEL_YOUT_H);
	data2 = hal_dev_mma8451_read_reg(ACCEL_YOUT_L);
	
	//Moving bits for high nible
	data1= data1<<8;
	
	return data = (data1+data2)* (8.0 / 65536.0) * 9.81;
	
}

/**
@brief Function reads Z axis of accelometer
@author Jakub Brzezowski
@version 1.0 2018-12-06
@param data is variable, which store data.
@retval none
*/
float Z_acc_read(void){
	
	//Virtual variable to read 8 byte registers.
	uint16_t data1, data2;
	float data;
	
	data1 = hal_dev_mma8451_read_reg(ACCEL_ZOUT_H);
	data2 = hal_dev_mma8451_read_reg(ACCEL_ZOUT_L);
	
	//Moving bits for high nible
	data1= data1<<8;
	
	return data = (data1+data2)* (8.0 / 65536.0) * 9.81;
	
}

/**
@brief Function reads X axis of gyroskop
@author Jakub Brzezowski
@version 1.0 2018-12-06
@param data is variable, which store data.
@retval none
*/
float X_gyro_read(void){
	
	//Virtual variable to read 8 byte registers.
	uint16_t data1, data2;
	float data;
	
	data1 = hal_dev_mma8451_read_reg(GYRO_XOUT_H);
	data2 = hal_dev_mma8451_read_reg(GYRO_XOUT_L);
	
	//Moving bits for high nible
	data1= data1<<8;
	
	return data = (data1+data2) * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
	
}

/**
@brief Function reads Y axis of gyroskop
@author Jakub Brzezowski
@version 1.0 2018-12-06
@param data is variable, which store data.
@retval none
*/
float Y_gyro_read(void){
	
	//Virtual variable to read 8 byte registers.
	uint16_t data1, data2;
	float data;
	
	data1 = hal_dev_mma8451_read_reg(GYRO_YOUT_H);
	data2 = hal_dev_mma8451_read_reg(GYRO_YOUT_L);
	
	//Moving bits for high nible
	data1= data1<<8;
	
	return data = (data1+data2)*(4000.0/65536.0) * (M_PI/180.0) * 25.0;
	
}

/**
@brief Function reads Z axis of gyroskop
@author Jakub Brzezowski
@version 1.0 2018-12-06
@param data is variable, which store data.
@retval none
*/
float Z_gyro_read(void){
	
	//Virtual variable to read 8 byte registers.
	uint16_t data1, data2;
	float data;
	
	data1 = hal_dev_mma8451_read_reg(GYRO_ZOUT_H);
	data2 = hal_dev_mma8451_read_reg(GYRO_ZOUT_L);
	
	//Moving bits for high nible
	data1= data1<<8;
	
	return data = (data1+data2)* (4000.0/65536.0) * (M_PI/180.0) * 25.0;
	
}
/**
@brief Function reads Z axis of gyroskop
@author Jakub Brzezowski
@version 1.0 2018-12-06
@param data is variable, which store data.
@retval none
*/
float psi_x(void){
	
	float xdata;
	float zdata;

	float psix;

	xdata=X_gyro_read();
	zdata=Z_gyro_read();
	
	return psix =atan2(xdata,zdata);
	
}

float psi_y(void){
	
	float ydata;
	float zdata;
	
	float psiy;
	
	ydata =Y_gyro_read();
	zdata =Z_gyro_read();
	
	return psiy = atan2(ydata,zdata);
	
}

float omega_x(void){
	float psi1, psi2, omega;
	psi1 = psi_x();
	time_delay_ms(1); //~1ms delay
	psi2 = psi_x();
	return (psi2-psi1);
	//0.001;
}

float omega_y(void){
	float psi1, psi2, omega;
	psi1 = psi_y();
	time_delay_ms(1); //~1ms delay
	psi2 = psi_y();
	return (psi2-psi1);
	//0.001;
}

float mpu_results[3];

void Gyro_R_Read(){
	
		float data[3];
		float accel[3];
		float accel_normal[3];
		float gyro_normal[3];
		float omegax;
		float omegay;
		float Omega_Gyro;
	
		float AcceVector;
		float GyroVector;
	
		data[0]=X_gyro_read();
		data[1]=Y_gyro_read();
		data[2]=Z_gyro_read();
	
		accel[0]=X_acc_read();
		accel[1]=Y_acc_read();
		accel[2]=Z_acc_read();
	
		AcceVector=sqrt(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
		GyroVector=sqrt(data[0]*data[0]+data[1]*data[1]+data[2]*data[2]);
	
		for(uint8_t i =0;i<3;i++){accel_normal[i]=accel[i]/AcceVector;}
		for(uint8_t i =0;i<3;i++){gyro_normal[i]=data[i]/GyroVector;}
		
		omegax = omega_x();
		omegay = omega_y();
		Omega_Gyro = omegay/omegax;
		
		mpu_results[0]= (accel_normal[0]+gyro_normal[0]*Omega_Gyro)/(1+Omega_Gyro);
		mpu_results[1]= (accel_normal[1]+gyro_normal[1]*Omega_Gyro)/(1+Omega_Gyro);
		mpu_results[2]= (accel_normal[2]+gyro_normal[2]*Omega_Gyro)/(1+Omega_Gyro);
		
}


/////////////////////////////////////
extern signed short mpu_resultx, mpu_resulty, mpu_resultz;

/*Acceleration  RAM */
signed int mpu_X_acc;
signed int mpu_Y_acc;
signed int mpu_Z_acc;

unsigned int mpu_xy_mag;
unsigned int mpu_xz_mag;
unsigned int mpu_yz_mag;

signed  int mpu_xy_angle;
signed  int mpu_xz_angle;
signed  int mpu_yz_angle;


struct tipo_mediana mpu_arr_medianas[3];

unsigned int mpu_cat;
unsigned int mpu_offset;



void mpu_angle_calc(void)
 {
   signed int nv, x2, y2, z2;

 /*  if (accel_count != 0) return;
   accel_count = 60; //60 msec
   */
	 
   
   nv = (signed char)(mpu_resultx);
   X_acc = median(nv, &arr_medianas[0]);
   
   
   nv = (signed char)(mpu_resulty);
   Y_acc = median(nv, &arr_medianas[1]);
   
   nv = (signed char)(mpu_resultz);  
   Z_acc = median(nv, &arr_medianas[2]);
	 
	 

   x2        = X_acc*X_acc;
   y2        = Y_acc*Y_acc;

   xy_mag   = sqrt_16(x2 + y2);
  
   if (Y_acc<0) mpu_cat = -Y_acc; else mpu_cat = Y_acc;
   
   mpu_offset = (unsigned int)(mpu_cat<<7)/(unsigned int)xy_mag;
   if (mpu_offset>127) mpu_offset = 127;    
   xy_angle = asin(mpu_offset);
   
   if (Y_acc>0)  xy_angle = -xy_angle;
   
   
   z2        = Z_acc*Z_acc;   
   xz_mag    = sqrt_16(x2 + z2);
   if (X_acc<0) mpu_cat = -X_acc; else mpu_cat = X_acc;
   mpu_offset = (unsigned int)(mpu_cat<<7)/(unsigned int)xz_mag;
   if (mpu_offset>127) mpu_offset = 127;    
   xz_angle = asin(mpu_offset);
   
   if (X_acc>0)  xz_angle = -xz_angle;
   

   yz_mag    = sqrt_16(y2 + z2);
   if (Y_acc<0) mpu_cat = -Y_acc; else mpu_cat = Y_acc;
   mpu_offset = (unsigned int)(mpu_cat<<7)/(unsigned int)yz_mag;
   if (mpu_offset>127) mpu_offset = 127;    
   yz_angle = asin(mpu_offset);
   if (Y_acc>0)  yz_angle = -yz_angle;
    
 }
 
void mpu_accel_read(void)
{
        mpu_accel_x   = X_acc_read();       
				mpu_accel_x >>= 2;
				
        mpu_accel_y  = Y_acc_read();
        mpu_accel_y >>= 2;
				
        mpu_accel_z   = Z_acc_read();        
        mpu_accel_z >>= 2;
        
        mpu_angle_calc(); //-900  to  900        
}












