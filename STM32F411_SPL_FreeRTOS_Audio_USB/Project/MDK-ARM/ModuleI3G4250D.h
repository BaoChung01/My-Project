/*
* Filename: ModuleI3G4250D.h
* Content: ModuleI3G4250D library of the program
*/
#ifndef _MODULEI3G4250D_H_
#define _MODULEI3G4250D_H_

#include "stm32f4xx.h"

#define I3G4250D_LOW_CS 				GPIO_ResetBits(GPIOE,GPIO_Pin_3)
#define I3G4250D_HIGH_CS 				GPIO_SetBits(GPIOE,GPIO_Pin_3)

/* Identification number */
#define I3G4250D_WHO_AM_I				0xD3  /* The Value of WHO_AM_I register */

/* Registers addresses */
#define I3G4250D_WHO_AM_I_ADDR          0x0F  /* device identification register */
#define I3G4250D_CTRL_REG1_ADDR         0x20  /* Control register 1 */
#define I3G4250D_CTRL_REG2_ADDR         0x21  /* Control register 2 */
#define I3G4250D_CTRL_REG3_ADDR         0x22  /* Control register 3 */
#define I3G4250D_CTRL_REG4_ADDR         0x23  /* Control register 4 */
#define I3G4250D_CTRL_REG5_ADDR         0x24  /* Control register 5 */
#define I3G4250D_REFERENCE_REG_ADDR     0x25  /* Reference register */
#define I3G4250D_OUT_TEMP_ADDR          0x26  /* Out temp register */
#define I3G4250D_STATUS_REG_ADDR        0x27  /* Status register */
#define I3G4250D_OUT_X_L_ADDR           0x28  /* Output Register X */
#define I3G4250D_OUT_X_H_ADDR           0x29  /* Output Register X */
#define I3G4250D_OUT_Y_L_ADDR           0x2A  /* Output Register Y */
#define I3G4250D_OUT_Y_H_ADDR           0x2B  /* Output Register Y */
#define I3G4250D_OUT_Z_L_ADDR           0x2C  /* Output Register Z */
#define I3G4250D_OUT_Z_H_ADDR           0x2D  /* Output Register Z */
#define I3G4250D_FIFO_CTRL_REG_ADDR     0x2E  /* Fifo control Register */
#define I3G4250D_FIFO_SRC_REG_ADDR      0x2F  /* Fifo src Register */

/* I3G4250D main output structure */
typedef struct {
	int8_t X; 								  /*!< X axis rotation */
	int8_t Y; 								  /*!< Y axis rotation */
	int8_t Z; 								  /*!< Z axis rotation */
} I3G4250D_Output_uint8_t;
extern I3G4250D_Output_uint8_t I3G4250D_Data;

/* I3G4250D Result enumerations */
typedef enum {
	I3G4250D_Result_Ok,   					  /*!< Everything OK */
	I3G4250D_Result_Error 					  /*!< Error occurred */
} I3G4250D_Result_t;

/* I3G4250D Scale enumerations */
typedef enum {
	I3G4250D_Scale_245,					 	  /*!< Set full scale to 245 mdps */ // Reset state
	I3G4250D_Scale_500, 					  /*!< Set full scale to 500 mdps */
	I3G4250D_Scale_2000 					  /*!< Set full scale to 2000 mdps */
} I3G4250D_Scale_t;

/* Public function */
/** 
* This function using SPI1 (PA5 = SCK, PA6 = MISO, PA7 = MOSI, PE3 = CS) 
* @brief Init I3G4250D module
* @retval I3G4250D_Result_Ok: the process is successful
* @retval I3G4250D_Result_Error:  the process fails. pls reset processor and try again.
*/
I3G4250D_Result_t I3G4250D_Init(void);

/** 
* This function using SPI1 (PA5 = SCK, PA6 = MISO, PA7 = MOSI, PE3 = CS) 
* @brief Init I3G4250D module
* @param p_I3G4250D_Data: Value X Y Z axis (uint8)
* @retval I3G4250D_Result_Ok: the process is successful
* @retval I3G4250D_Result_Error:  the process fails. pls reset processor and try again.
*/
I3G4250D_Result_t I3G4250D_Read(I3G4250D_Output_uint8_t* p_I3G4250D_Data);

#endif



