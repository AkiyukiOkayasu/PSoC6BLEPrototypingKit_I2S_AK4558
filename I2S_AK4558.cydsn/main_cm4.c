/*****************************************************************************
* File Name: main_cm4.c
*
* Description:  This file contains the implementation of the main function and
* the I2S interrupt handler.
*
* Related Document: Code example CE218636
*
* Hardware Dependency: See code example CE218636
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*****************************************************************************/
#include "project.h"
#include "wave.h"
#include "ak4558.h"

/* Combine master error statuses in single mask */
#define MASTER_ERROR_MASK  (CY_SCB_I2C_MASTER_DATA_NAK | CY_SCB_I2C_MASTER_ADDR_NAK    | \
                            CY_SCB_I2C_MASTER_ARB_LOST | CY_SCB_I2C_MASTER_ABORT_START | \
                            CY_SCB_I2C_MASTER_BUS_ERR)    

/* I2C slave address to communicate with */
#define I2C_SLAVE_ADDR  (0b0010000)

/* I2C Buffer and packet size */
#define TX_PACKET_SIZE  (2UL)
#define RX_PACKET_SIZE  (2UL)
uint8_t  buffer[TX_PACKET_SIZE];

/* I2C Structure for master transfer configuration */
cy_stc_scb_i2c_master_xfer_config_t masterTransferCfg =
{
    .slaveAddress = I2C_SLAVE_ADDR,
    .buffer       = NULL,
    .bufferSize   = 0U,
    .xferPending  = false
};

/* I2C Command valid status */
#define TRANSFER_CMPLT    (0x00UL)
#define TRANSFER_ERROR    (0xFFUL)
#define READ_CMPLT        (TRANSFER_CMPLT)
#define READ_ERROR        (TRANSFER_ERROR)

/* Global Variables for the I2S */
bool isPlaying = false;
uint32 waveDataSampleIndex = 0;

/******************************************************************************
* Function Name: WaitOneUnit
****************************************************************************//**
*
* Waits for one unit before unblock code execution.
* Note If a timeout value is 0, this function does nothing and returns 0.
*
* \param timeout
* The pointer to a timeout value.
*
* \return
* Returns 0 if a timeout does not expire or the timeout mask.
*
*******************************************************************************/
static bool WaitOneUnit(uint32_t *timeout)
{
    uint32_t status = 0UL;

    /* If the timeout equal to 0. Ignore the timeout. */
    if (*timeout > 0UL)
    {
        Cy_SysLib_DelayUs(CY_SCB_WAIT_1_UNIT);
        --(*timeout);

        if (0UL == *timeout)
        {
            status = true;
        }
    }
    return (status);
}

/****************************************************************************
* Function Name: I2C_Write
****************************************************************************//**
*
* I2C write high level api
*
*******************************************************************************/
static uint8_t I2C_Write()
{
    uint8_t status = TRANSFER_ERROR;
    cy_en_scb_i2c_status_t errorStatus;
    
    masterTransferCfg.buffer = buffer;
    masterTransferCfg.bufferSize = TX_PACKET_SIZE;
    
    errorStatus = Cy_SCB_I2C_MasterWrite(I2C_HW, &masterTransferCfg, &I2C_context);
    if(errorStatus != CY_SCB_I2C_SUCCESS) return status;
        
    uint32_t masterStatus;
    bool timeOutStatus;
        
    //1sec(one unit is us)
    uint32_t timeout = 1000000UL;
        
    do
    {
        masterStatus = Cy_SCB_I2C_MasterGetStatus(I2C_HW, &I2C_context);
        timeOutStatus = WaitOneUnit(&timeout);
    } while ((0UL != (masterStatus & CY_SCB_I2C_MASTER_BUSY)) && (timeOutStatus == false));
        
    if (timeOutStatus)
    {
        //Timeout recovery
        Cy_SCB_I2C_Disable(I2C_HW, &I2C_context);
        Cy_SCB_I2C_Enable(I2C_HW);
    }
    else 
    {
        if((0u == (MASTER_ERROR_MASK & masterStatus)) && (TX_PACKET_SIZE == Cy_SCB_I2C_MasterGetTransferCount(I2C_HW, &I2C_context)))
        {
            status = TRANSFER_CMPLT;
        }
    }            
    return status;
}

/****************************************************************************
* Function Name: I2C_Read
****************************************************************************//**
*
* I2C read high level api
*
*******************************************************************************/
static uint8_t I2C_Read(void)
{
    uint8 status = TRANSFER_ERROR;
    cy_en_scb_i2c_status_t errorStatus;
    uint8 rxBuffer  [RX_PACKET_SIZE];
    
    masterTransferCfg.buffer = rxBuffer;
    masterTransferCfg.bufferSize = RX_PACKET_SIZE;
    
    errorStatus = Cy_SCB_I2C_MasterRead(I2C_HW, &masterTransferCfg, &I2C_context);
    if (errorStatus != CY_SCB_I2C_SUCCESS) return status;
    
    uint32 masterStatus;
    bool timeOutStatus;
        
    //1sec(one unit is us)
    uint32_t timeout = 1000000UL;
        
    do
    {
        masterStatus = Cy_SCB_I2C_MasterGetStatus(I2C_HW, &I2C_context);
        timeOutStatus = WaitOneUnit(&timeout);                        
    }while((0UL != (masterStatus & CY_SCB_I2C_MASTER_BUSY)) && (timeOutStatus == false));
        
    if(timeOutStatus)
    {
        Cy_SCB_I2C_Disable(I2C_HW, &I2C_context);
        Cy_SCB_I2C_Enable(I2C_HW);
    }
    else
    {
        if(0u == (MASTER_ERROR_MASK & masterStatus))
        {
            // do something
        }
    }   
    
    return status;        
}

/******************************************************************************
* Function Name: ak4558Config
****************************************************************************//**
*
* AK4558 I2C config
* PLL slave
* 16bitI2S stereo
* 44.1kHz or 48KHz
* DAC only(NO ADC)
*
*******************************************************************************/
void ak4558Config(void) 
{                
    // PLL PowerDown
    // BICK FS: 32FS
    buffer[0] = AK4558_I2C_PLL;//0x01
    buffer[1] = (BICK32FS << 1) | EXT_POWERDOWN;    
    I2C_Write();         
    
    // Audio format: 16bit I2S
    // Ch: Stereo
    // SoftMute: OFF
    buffer[0] = AK4558_I2C_CONTROL1;
    buffer[1] = (STEREO << 6) | (I2S_16BIT << 3) | (ATTENUATIONTIME << 1) | SOFTMUTE_OFF;    
    I2C_Write();        
    
    // LPF filter: Short delay sharp rolloff
    buffer[0] = AK4558_I2C_FILTER;
    buffer[1] = FILTER_SHORTDELAY_SHARPROLLOFF;
    I2C_Write();      
    
    // LOUT volume set to 0dB
    buffer[0] = AK4558_I2C_LOUT;
    buffer[1] = DACVOLUME_0dB;
    I2C_Write();    
    
    // ROUT volume set to 0dB
    buffer[0] = AK4558_I2C_ROUT;
    buffer[1] = DACVOLUME_0dB;
    I2C_Write();    
    
    // Power save
    // Sample rate: 48kHz or 44.1kHz
    buffer[0] = AK4558_I2C_MODE;
    buffer[1] = (FS48_441 << 3) | LOPS_POWERSAVE;
    I2C_Write();    
    
    // PLL Powerup
    //BICK FS: 32FS
    buffer[0] = AK4558_I2C_PLL;
    buffer[1] = (BICK32FS << 1) | PLL_POWERUP;
    I2C_Write();
    
    CyDelay(10);
    
    // DAC ON
    buffer[0] = AK4558_I2C_POWER;
    buffer[1] = ADC_OFF_DAC_ON;
    I2C_Write();
    
    CyDelay(300);    
    
    // Start Normal operation
    buffer[0] = AK4558_I2C_MODE;//0x05
    buffer[1] = (FS48_441 << 3) | LOPS_NORMALOPERATION;
    I2C_Write();        
}

/*******************************************************************************
* Function Name: I2S_isr_Handler
****************************************************************************//**
*
* I2S Interrupt Handler Implementation. Feed the I2S internal FIFO with audio
* data.
*  
*******************************************************************************/
void I2S_isr_Handler(void)
{   
    uint32_t I2SDataToSend = 0;
    
    if(isPlaying) 
    {
        I2SDataToSend += waveData[waveDataSampleIndex];
        waveDataSampleIndex++;
        if(waveDataSampleIndex >= NUMSAMPLES) 
        {
            isPlaying = false;
            waveDataSampleIndex = 0;
        }
    }
    
    /* Write data for the left side */
    I2S_WriteTxData(I2SDataToSend);    
    /* Write data for the right side */
    I2S_WriteTxData(I2SDataToSend);    
    
    //uint32 n = I2S_GetNumInRxFifo();
    //uint32 s = I2S_ReadRxData();

    /* Clear I2S Interrupt */
    I2S_ClearInterrupt(I2S_INTR_TX_TRIGGER_Msk); 
    //I2S_ClearInterrupt(I2S_INTR_RX_TRIGGER_Msk);
}

/*******************************************************************************
* Function Name: main
****************************************************************************//**
*
* The main function for the Cortex-M4 CPU does the following:
*  Initialization:
*  - Initializes all the hardware blocks
*  Do forever loop:
*  - Check if the SW2 button was pressed. If yes, plays the audio stream. 
*  
*******************************************************************************/
int main(void)
{
    /* Initialize the I2S interrupt */
    Cy_SysInt_Init(&I2S_isr_cfg, I2S_isr_Handler);
    NVIC_EnableIRQ(I2S_isr_cfg.intrSrc);        
 
    /* Enable global interrupts. */
    __enable_irq();                
    
    I2C_Start();    
        
    /* AK4558 Reset(PDN) pin L->H */
    CyDelay(1);
    Cy_GPIO_Write(RESET_PORT, RESET_NUM, 0u);    
    CyDelay(1);    
    Cy_GPIO_Write(RESET_PORT, RESET_NUM, 1u);    
    
    /*Wait for AK4558 internal LDO wake up */
    CyDelay(15);
    
    /* Start the I2S interface */
    I2S_Start();
    
    /* AK4558 codec config */
    ak4558Config();           
        
    for(;;)
    {
        if(Cy_GPIO_Read(SW2_PORT, SW2_NUM) == 0)
        {
            if(isPlaying == false) {
                isPlaying = true;
            }            
        }                             
        CyDelay(1);//1ms delay
    }
}

/* [] END OF FILE */
