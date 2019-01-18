#include "Accelometer.h"
#define GyroSens 				 131
#include <cmath>
#define M_PI        3.14159265358979323846

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
// this delay is very important, it may cause w-r operation failure.
static void pause(void)
{
    int n;
    for(n=0; n<40; n++)
        asm("nop");
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
	pause();
}
/**
@brief Function reads X axis of accelometer
@author Jakub Brzezowski
@version 1.0 2018-12-06
@param data is variable, which store data.
@retval none
*/
double X_acc_read(void){
	
	//Virtual variable to read 8 byte registers.
	uint16_t data1, data2;
	double data;
	
	data1 = read_register(ACCEL_XOUT_H);
	data2 = read_register(ACCEL_XOUT_L);
	
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
double Y_acc_read(void){
	
	//Virtual variable to read 8 byte registers.
	uint16_t data1, data2;
	double data;
	
	data1 = read_register(ACCEL_YOUT_H);
	data2 = read_register(ACCEL_YOUT_L);
	
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
double Z_acc_read(void){
	
	//Virtual variable to read 8 byte registers.
	uint16_t data1, data2;
	double data;
	
	data1 = read_register(ACCEL_ZOUT_H);
	data2 = read_register(ACCEL_ZOUT_L);
	
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
double X_gyro_read(void){
	
	//Virtual variable to read 8 byte registers.
	uint16_t data1, data2;
	double data;
	
	data1 = read_register(GYRO_XOUT_H);
	data2 = read_register(GYRO_XOUT_L);
	
	//Moving bits for high nible
	data1= data1<<8;
	
	return data = (data1+data2)* (4000.0/65536.0) * (M_PI/180.0) * 25.0;
	
}

/**
@brief Function reads Y axis of gyroskop
@author Jakub Brzezowski
@version 1.0 2018-12-06
@param data is variable, which store data.
@retval none
*/
double Y_gyro_read(void){
	
	//Virtual variable to read 8 byte registers.
	uint16_t data1, data2;
	double data;
	
	data1 = read_register(GYRO_YOUT_H);
	data2 = read_register(GYRO_YOUT_L);
	
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
double Z_gyro_read(void){
	
	//Virtual variable to read 8 byte registers.
	uint16_t data1, data2;
	double data;
	
	data1 = read_register(GYRO_ZOUT_H);
	data2 = read_register(GYRO_ZOUT_L);
	
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
double psi_x(void){
	
	double data[3];

	double psix;

	data[0]=X_gyro_read();
	data[1]=Y_gyro_read();
	data[2]=Z_gyro_read();
	
	return psix =atan2(data[0],data[2]);
	
}

double psi_y(void){
	
	double data[3];
	
	double psiy;
	
	data[0]=X_gyro_read();
	data[1]=Y_gyro_read();
	data[2]=Z_gyro_read();
	
	return psiy = atan2(data[1],data[2]);
	
}

double omega_x(void){
	double psi1, psi2, omega;
	psi1 = psi_x();
	delay_mc(21000); //~1ms delay
	psi2 = psi_x();
	return (psi2-psi1)/0.001;
}

double omega_y(void){
	double psi1, psi2, omega;
	psi1 = psi_y();
	delay_mc(21000); //~1ms delay
	psi2 = psi_y();
	return (psi2-psi1)/0.001;
}

double Gyro_R_Read(double* input){
	
		double data[3];
		double accel[3];
		double accel_normal[3];
		double gyro_normal[3];
		double omegax;
		double omegay;
		double Omega_Gyro;
	
		double AcceVector;
		double GyroVector;
	
		data[0]=X_gyro_read();
		data[1]=Y_gyro_read();
		data[2]=Z_gyro_read();
	
		accel[0]=X_acc_read();
		accel[1]=Y_acc_read();
		accel[2]=Z_acc_read();
	
		AcceVector=sqrt(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
		GyroVector=sqrt(data[0]*data[0]+data[1]*data[1]+data[2]*data[2]);
	
		for(uint8_t i =0;i<0;i++){accel_normal[i]=accel[i]/AcceVector;}
		for(uint8_t i =0;i<0;i++){gyro_normal[i]=data[i]/GyroVector;}
		
		omegax = omega_x();
		omegay = omega_y();
		Omega_Gyro = omegay/omegax;
		
		input[0]= (accel_normal[0]+gyro_normal[0]*Omega_Gyro)/(1+Omega_Gyro);
		input[1]= (accel_normal[1]+gyro_normal[1]*Omega_Gyro)/(1+Omega_Gyro);
		input[2]= (accel_normal[2]+gyro_normal[2]*Omega_Gyro)/(1+Omega_Gyro);
		
}


