/*
 *  spiaccess - Helper functions for accessing the DVIplugin FPGA registers 
 *              via SPI bus.
 *
 *  Copyright (c) 2018 Exor Int.
 *  Written by: Giovanni Pavoni, Exor Int.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 or
 *  later as publishhed by the Free Software Foundation.
 *
 */

#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>

//------------------------------------------------------------------------------
// Basic Defines 
//------------------------------------------------------------------------------
//SPI slave Instruction set
#define	SPI_CMD_RD_BYTE         0x00
#define	SPI_CMD_WR_BYTE         0x08
#define	SPI_CMD_RD_WORD         0x10
#define	SPI_CMD_WR_WORD         0x18
#define	SPI_CMD_RD_DWORD        0x20
#define	SPI_CMD_WR_DWORD        0x28
#define	SPI_CMD_RD_BLOCK        0x40
#define	SPI_CMD_WR_BLOCK        0x48
#define	SPI_CMD_SETP_B0         0x80
#define	SPI_CMD_SETP_B1         0x88
#define	SPI_CMD_SETP_B2         0x90
#define	SPI_CMD_SETP_B3         0x98
#define	SPI_CMD_GET_PTR         0xC0
#define	SPI_CMD_SET_PTR         0xC8
#define	SPI_CMD_SET_BLOCK_SIZE  0xD0
#define	SPI_CMD_NOP             0xE0
#define	SPI_CMD_RST_ERR         0xF8

//SPI slave Response code
#define	SPI_RESP_READY          0xAA
#define	SPI_RESP_WAIT           0xBB
#define	SPI_RESP_OK             0xCC
#define	SPI_RESP_ERROR          0xDD

//Available pointers
#define  SPI_PTR_0              0x00     
#define  SPI_PTR_1              0x01     
#define  SPI_PTR_2              0x02     
#define  SPI_PTR_3              0x03     
#define  SPI_PTR_4              0x04     
#define  SPI_PTR_5              0x05     
#define  SPI_PTR_6              0x06     
#define  SPI_PTR_7              0x07     
#define  SPI_PTR_MAX_NUM        0x08

//Reset SPI errors constants
#define  SPI_MAXNUM_RTRCMD          3
#define  SPI_MAXNUM_RSTERR          5
#define  SPI_MAXNUM_WAITOK          5
#define  SPI_RSTERR_OK              0x00
#define  SPI_RSTERR_NOK             0xFF
#define  SPI_WAITOK_OK              0x00
#define  SPI_WAITOK_NOK             0xFF

//Variable lenght constants
#define  BYTE_VAL_LEN            (0x01)
#define  WORD_VAL_LEN            (0x02)
#define  DWORD_VAL_LEN           (0x04)
#define  SPI_MAX_BLOCK_SIZE      64

//******************************************************************************
//******************************************************************************
// Static helper functions (used only locally by the current module)
//******************************************************************************
//******************************************************************************

/* Do a SPI I/O R/W access
 */
static int spi_rw(struct spi_device *spi, u8* buf, u8 len)
{
  struct spi_transfer t;
  struct spi_message  m;
  u8 rx_buf[SPI_MAX_BLOCK_SIZE];
  
  spi_message_init(&m);
  memset(&t, 0, sizeof t);
  t.tx_buf = buf;
  t.rx_buf = rx_buf;
  t.len = len;
  spi_message_add_tail(&t, &m);
  spi_sync(spi, &m);
  
  memcpy(buf, t.rx_buf, len);
  return 0;
}


//Reset error on SPI interface
static unsigned char SpiResetErrors(struct spi_device *spi)
{
   u8 ucData[8], idx;

   // Reset spi
   for( idx=0; idx<SPI_MAXNUM_RSTERR; idx++)
   {
     ucData[0] = SPI_CMD_RST_ERR;
     spi_rw(spi, ucData, 1);
     
     if( ucData[0] == SPI_RESP_READY)
       break;
   }
   if( idx>=SPI_MAXNUM_RSTERR)
      return SPI_RSTERR_NOK;
   else
      return SPI_RSTERR_OK;
}


// Sends command to the SPI slave core
static int RW_SPI(struct spi_device *spi, char spiPtr, unsigned char cmd, unsigned char *pVal, unsigned char lenVal)
{
   u8                ucData[SPI_MAX_BLOCK_SIZE];
   u8                numDataOut;
   int               RetryCnt;
   int               i;

   //Check input parameter consistence
   if (spiPtr  >= 0 && spiPtr  < SPI_PTR_MAX_NUM)
   {
      // Send command
      for (RetryCnt = 0; RetryCnt < SPI_MAXNUM_RSTERR; RetryCnt++)
      {
         ucData[0] = cmd | spiPtr;
         if (cmd == SPI_CMD_WR_BYTE  ||
             cmd == SPI_CMD_WR_WORD  ||
             cmd == SPI_CMD_WR_DWORD ||
             cmd == SPI_CMD_WR_BLOCK ||
             cmd == SPI_CMD_SETP_B0  ||
             cmd == SPI_CMD_SETP_B1  ||
             cmd == SPI_CMD_SETP_B2  ||
             cmd == SPI_CMD_SETP_B3  ||
             cmd == SPI_CMD_SET_PTR  ||
             cmd == SPI_CMD_SET_BLOCK_SIZE)	// if write command
         {
            for (i = 0; i < lenVal; i++)
               ucData[i + 1] = pVal[i];
            ucData[lenVal + 1] = SPI_CMD_NOP;
            ucData[lenVal + 2] = SPI_CMD_NOP;
         }
         else //read command
         {
            for (i = 1; i < lenVal + 4; i++)
               ucData[i] = SPI_CMD_NOP;
         }
         numDataOut = lenVal + 1;      
         if (cmd == SPI_CMD_RD_BYTE  ||
             cmd == SPI_CMD_WR_BYTE  ||
             cmd == SPI_CMD_RD_WORD  ||
             cmd == SPI_CMD_WR_WORD  ||
             cmd == SPI_CMD_RD_DWORD ||
             cmd == SPI_CMD_WR_DWORD ||
             cmd == SPI_CMD_WR_BLOCK)	// If access to memory
         {
            numDataOut += 2;
         }
         else if (cmd == SPI_CMD_RD_BLOCK)   // Another NOP at the end for checking success
            numDataOut += 3;
  
	 spi_rw(spi, ucData, numDataOut);

         if (cmd == SPI_CMD_WR_BYTE  ||
             cmd == SPI_CMD_WR_WORD  ||
             cmd == SPI_CMD_WR_DWORD ||
             cmd == SPI_CMD_WR_BLOCK)  // if write to memory
         {
            // Correct reply is READY, followed by WAIT and OK at the end
            if (ucData[0] == SPI_RESP_READY)
            {
               for (i = 1; i <= lenVal; i++)
                  if (ucData[i] != SPI_RESP_WAIT)
                     break;
               if (i == lenVal + 1)
                  if ((ucData[lenVal + 1] == SPI_RESP_OK   && ucData[lenVal + 2] == SPI_RESP_READY) ||
                      (ucData[lenVal + 1] == SPI_RESP_WAIT && ucData[lenVal + 2] == SPI_RESP_OK) )
                  {
                     break;
                  }
            }
            if (ucData[lenVal + 1] == SPI_RESP_ERROR || ucData[lenVal + 2] == SPI_RESP_ERROR)
	      SpiResetErrors(spi);
            if (RetryCnt + 1 == SPI_MAXNUM_RSTERR)
            {
               return -3; //Write Data fault
            }
         }
         else if (cmd == SPI_CMD_SETP_B0  ||
                  cmd == SPI_CMD_SETP_B1  ||
                  cmd == SPI_CMD_SETP_B2  ||
                  cmd == SPI_CMD_SETP_B3  ||
                  cmd == SPI_CMD_SET_PTR  ||
                  cmd == SPI_CMD_SET_BLOCK_SIZE)	// if write to SPI register
         {
            // Correct reply is READY for all the transmitted bytes
            for (i = 0; i <= lenVal; i++)
               if (ucData[i] != SPI_RESP_READY)
                  break;
            if (i == lenVal + 1)
            {
               break;
            }
            if (ucData[lenVal]     == SPI_RESP_ERROR || ucData[lenVal + 1] == SPI_RESP_ERROR)
	      SpiResetErrors(spi);
            if (RetryCnt + 1 == SPI_MAXNUM_RSTERR)
            {
               return -5; //Write Data fault
            }
         }
         else //read command
         {
            int   DataOffset=0;
            if (ucData[0] == SPI_RESP_READY)
            {
               if (cmd == SPI_CMD_GET_PTR)
               {
                  for (i = 0; i < lenVal; i++)
                     pVal[i] = ucData[1 + i];
                  break;
               }
               else  // For all read memory, then find beginning of data
               {
                  for (i = 1; i <= 2; i++)
                     if (ucData[i] != SPI_RESP_WAIT)
                        break;
                  if (ucData[i] == SPI_RESP_OK)
                  {
                     DataOffset = i + 1;
                     if (cmd != SPI_CMD_RD_BLOCK ||
                         ucData[DataOffset + lenVal] == SPI_RESP_READY)
                     {
                        for (i = 0; i < lenVal; i++)
                           pVal[i] = ucData[DataOffset + i];
                        break;
                     }
                  }
               }
            }
            if (ucData[0] == SPI_RESP_ERROR ||
                ucData[1] == SPI_RESP_ERROR ||
                ucData[2] == SPI_RESP_ERROR ||
                (cmd == SPI_CMD_RD_BLOCK && ucData[DataOffset + lenVal] == SPI_RESP_ERROR))
            {
               SpiResetErrors(spi);
               if (RetryCnt + 1 == SPI_MAXNUM_RSTERR)
               {
                  return -6; //Read Data fault
               }
            }
         }
      }
   }
   else //Check input parameter consistence
   {
      printk("%s : Invalid parameters\n", __func__);
      return -1;
   }
   return 0;
}

//******************************************************************************
//******************************************************************************
// Exported functions (available for application)
//******************************************************************************
//******************************************************************************

/* DVIwriteReg()
 * Write a 32 bit register at the specified (32 bit) address
 *
 * Input: Handle hspi    =SPI handle
 *        DWORD  regaddr =Reg address (32bit)
 *        DWORD  value   =Value to be written
 *
 */
int DVIwriteReg(struct spi_device *spi, u32 regaddr, u32 value)
{
	int res;

	//Write address pointer
	res = RW_SPI(spi, SPI_PTR_0, SPI_CMD_SET_PTR, (unsigned char*)&regaddr, DWORD_VAL_LEN);
	if(res)
	{
		printk("%s ERROR 1 regaddr=0x%x value=0x%x\n", __func__, regaddr, value);
		return -1;
	}
	
	// Write register value
	res = RW_SPI(spi, SPI_PTR_0, SPI_CMD_WR_DWORD, (unsigned char*)&value, DWORD_VAL_LEN);
	if(res)
	{
		printk("%s ERROR 2 regaddr=0x%x value=0x%x\n", __func__, regaddr, value);
		return -1;
	}
	//printk("%s OK regaddr=0x%x value=0x%x\n", __func__, regaddr, value);
	
	return 0;
}


/* DVIreadReg()
 * Read a 32 bit register from the specified (32 bit) address
 *
 * Input: Handle hspi    =SPI handle
 *        DWORD  regaddr =Reg address (32bit)
 *
 * Return: DWORD reg value
 */
u32 DVIreadReg(struct spi_device *spi, u32 regaddr)
{
	int res;
	u32 tmp=0;

	//Write address pointer
	res = RW_SPI(spi, SPI_PTR_0, SPI_CMD_SET_PTR, (unsigned char*)&regaddr, DWORD_VAL_LEN);
	if(res)
	{
		printk("%s ERROR 1 regaddr=0x%x\n", __func__, regaddr);
		return 0;
	}
	
	//Read the register
	res = RW_SPI(spi, SPI_PTR_0, SPI_CMD_RD_DWORD, (unsigned char*)&tmp, DWORD_VAL_LEN);
	if(res)
	{
		printk("%s ERROR 2 regaddr=0x%x\n", __func__, regaddr);
		return 0;
	}
	//printk("%s OK regaddr=0x%x val=0x%x\n", __func__, regaddr, tmp);

	return tmp;
}
