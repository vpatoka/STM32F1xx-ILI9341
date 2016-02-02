/**
  ******************************************************************************
  * @file    ili9341.c
  * @author  V. Patoka
  * @version V0.0.1 alpha
  * @date    31-January-2016
  * @brief   This file includes the LCD driver for ILI9341 LCD.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 Patoka Consulting Inc</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "ili9341.h"
#include <stdlib.h>


/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
  
/** @addtogroup ILI9341
  * @brief This file provides a set of functions needed to drive the 
  *        ILI9341 LCD.
  * @{
  */

/** @defgroup ILI9341_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup ILI9341_Private_Defines
  * @{
  */
/**
  * @}
  */ 
  
/** @defgroup ILI9341_Private_Macros
  * @{
  */
/**
  * @}
  */  

/** @defgroup ILI9341_Private_Variables
  * @{
  */ 


LCD_DrvTypeDef   ili9341_drv = 
{
  ili9341_Init,
  ili9341_ReadID,
  ili9341_DisplayOn,
  ili9341_DisplayOff,
  ili9341_SetCursor,
  ili9341_WritePixel,
  0,
  ili9341_SetDisplayWindow,
  ili9341_DrawHLine,
  ili9341_DrawVLine,
  ili9341_GetLcdPixelWidth,
  ili9341_GetLcdPixelHeight,
  ili9341_DrawBitmap,
  ili9341_DrawRGBImage,    
};




// #################### LCD_SPIx ##################################
#define LCD_SPIx                               SPI1
#define LCD_SPIx_CLK_ENABLE()                  __HAL_RCC_SPI1_CLK_ENABLE()

#define LCD_SPIx_SCK_GPIO_PORT                 GPIOA             // PA.05
#define LCD_SPIx_SCK_PIN                       GPIO_PIN_5
#define LCD_SPIx_SCK_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD_SPIx_SCK_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()

#define LCD_SPIx_MISO_MOSI_GPIO_PORT           GPIOA
#define LCD_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOC_CLK_ENABLE()
#define LCD_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOC_CLK_DISABLE()
#define LCD_SPIx_MISO_PIN                      GPIO_PIN_6       /* PA.06*/
#define LCD_SPIx_MOSI_PIN                      GPIO_PIN_7       /* PA.07*/
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define LCD_SPIx_TIMEOUT_MAX                   1000

#define BUFFERSIZE                             8

/**
 * @brief BUS variables
 */
uint32_t SpixTimeout = LCD_SPIx_TIMEOUT_MAX;        /*<! Value of Timeout when SPI communication fails */

enum {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};


/* Buffer used for transmission */
uint8_t aTxBuffer[BUFFERSIZE];

/* Buffer used for reception */
uint8_t aRxBuffer[BUFFERSIZE];

/* transfer state */
__IO uint32_t wTransferState = TRANSFER_WAIT;
__IO uint32_t rTransferState = TRANSFER_WAIT;

// -------------- START

/**
  * @}
  */ 
  
/** @defgroup ILI9341_Private_FunctionPrototypes
  * @{
  */

extern SPI_HandleTypeDef hspi1;

/**
  * @}
  */ 
  
/** @defgroup ILI9341_Private_Functions
  * @{
  */   

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
void ili9341_Init(void)
{
    /* Initialize ILI9341 low level bus layer ----------------------------------*/
    ili9341_IO_Init();

    ili9341_WriteReg(0xEF);
    ili9341_WriteData(0x03);
    ili9341_WriteData(0x80);
    ili9341_WriteData(0x02);

    ili9341_WriteReg(0xCF);  
    ili9341_WriteData(0x00); 
    ili9341_WriteData(0XC1); 
    ili9341_WriteData(0X30); 

    ili9341_WriteReg(0xED);  
    ili9341_WriteData(0x64); 
    ili9341_WriteData(0x03); 
    ili9341_WriteData(0X12); 
    ili9341_WriteData(0X81); 
 
    ili9341_WriteReg(0xE8);  
    ili9341_WriteData(0x85); 
    ili9341_WriteData(0x00); 
    ili9341_WriteData(0x78); 

    ili9341_WriteReg(0xCB);  
    ili9341_WriteData(0x39); 
    ili9341_WriteData(0x2C); 
    ili9341_WriteData(0x00); 
    ili9341_WriteData(0x34); 
    ili9341_WriteData(0x02); 
 
    ili9341_WriteReg(0xF7);  
    ili9341_WriteData(0x20); 

    ili9341_WriteReg(0xEA);  
    ili9341_WriteData(0x00); 
    ili9341_WriteData(0x00); 

    ili9341_WriteReg(LCD_POWER1);    //Power control 
    ili9341_WriteData(0x23);   //VRH[5:0] 
 
    ili9341_WriteReg(LCD_POWER1);    //Power control 
    ili9341_WriteData(0x10);   //SAP[2:0];BT[3:0] 
 
    ili9341_WriteReg(LCD_VCOM1);    //VCM control 
    ili9341_WriteData(0x3e); //对比度调节
    ili9341_WriteData(0x28); 
  
    ili9341_WriteReg(LCD_VCOM2);    //VCM control2 
    ili9341_WriteData(0x86);  //--
 
    ili9341_WriteReg(LCD_MAC);    // Memory Access Control 
    ili9341_WriteData(0x48);

    ili9341_WriteReg(LCD_PIXEL_FORMAT);    
    ili9341_WriteData(0x55); 
  
    ili9341_WriteReg(LCD_FRMCTR1);    
    ili9341_WriteData(0x00);  
    ili9341_WriteData(0x18); 
 
    ili9341_WriteReg(LCD_DFC);    // Display Function Control 
    ili9341_WriteData(0x08); 
    ili9341_WriteData(0x82);
    ili9341_WriteData(0x27);  
 
    ili9341_WriteReg(0xF2);    // 3Gamma Function Disable 
    ili9341_WriteData(0x00); 
 
    ili9341_WriteReg(LCD_GAMMA);    //Gamma curve selected 
    ili9341_WriteData(0x01); 
 
    ili9341_WriteReg(LCD_PGAMMA);    //Set Gamma 
    ili9341_WriteData(0x0F); 
    ili9341_WriteData(0x31); 
    ili9341_WriteData(0x2B); 
    ili9341_WriteData(0x0C); 
    ili9341_WriteData(0x0E); 
    ili9341_WriteData(0x08); 
    ili9341_WriteData(0x4E); 
    ili9341_WriteData(0xF1); 
    ili9341_WriteData(0x37); 
    ili9341_WriteData(0x07); 
    ili9341_WriteData(0x10); 
    ili9341_WriteData(0x03); 
    ili9341_WriteData(0x0E); 
    ili9341_WriteData(0x09); 
    ili9341_WriteData(0x00); 
  
    ili9341_WriteReg(LCD_NGAMMA);    //Set Gamma 
    ili9341_WriteData(0x00); 
    ili9341_WriteData(0x0E); 
    ili9341_WriteData(0x14); 
    ili9341_WriteData(0x03); 
    ili9341_WriteData(0x11); 
    ili9341_WriteData(0x07); 
    ili9341_WriteData(0x31); 
    ili9341_WriteData(0xC1); 
    ili9341_WriteData(0x48); 
    ili9341_WriteData(0x08); 
    ili9341_WriteData(0x0F); 
    ili9341_WriteData(0x0C); 
    ili9341_WriteData(0x31); 
    ili9341_WriteData(0x36); 
    ili9341_WriteData(0x0F); 

    ili9341_WriteReg(LCD_SLEEP_OUT);    //Exit Sleep 
    ili9341_Delay(200); 		
    ili9341_WriteReg(LCD_DISPLAY_ON);    //Display on 
}    



/**
  * @brief  Writes to the selected LCD register.
  * @param  LCDReg:      address of the selected register.
  * @retval None
  */
void ili9341_WriteReg(uint8_t LCDReg)
{
    // Set D/C to send command
    LCD_DC_LOW();
    // Deselect : Chip Select high 
    LCD_CS_LOW();

    wTransferState = TRANSFER_WAIT;
    HAL_SPI_Transmit_IT(&hspi1, &LCDReg, 1);
    while (wTransferState == TRANSFER_WAIT);
    if(wTransferState != TRANSFER_COMPLETE) {
        ili9341_Error();
    }
    LCD_CS_HIGH();
}

/**
  * @brief  Writes data to the selected LCD register.
  * @param  RegValue: address of the selected register.
  * @retval None
  */
void ili9341_WriteData(uint8_t RegValue)
{
    // Set D/C to send data
    LCD_DC_HIGH();
    // Deselect : Chip Select high 
    LCD_CS_LOW();

    wTransferState = TRANSFER_WAIT;
    HAL_SPI_Transmit_IT(&hspi1, &RegValue, 1);
    while (wTransferState == TRANSFER_WAIT);
    if(wTransferState != TRANSFER_COMPLETE) {
        ili9341_Error();
    }
    LCD_CS_HIGH();
}

/**
  * @brief  Writes Byte to SPI
  * @param  RegValue: address of the selected register.
  * @retval None
  */
void ili9341_WriteByte(uint8_t Value)
{
    wTransferState = TRANSFER_WAIT;
    HAL_SPI_Transmit_IT(&hspi1, &Value, 1);
    while (wTransferState == TRANSFER_WAIT);
    if(wTransferState != TRANSFER_COMPLETE) {
        ili9341_Error();
    }
}

/**
  * @brief  Writes Word to SPI
  * @param  RegValue: address of the selected register.
  * @retval None
  */
void ili9341_WriteWord(uint16_t Value)
{
    aTxBuffer[0] = Value>>8;
    aTxBuffer[1] = Value&0xff;
    wTransferState = TRANSFER_WAIT;
    HAL_SPI_Transmit_IT(&hspi1, (uint8_t *)aTxBuffer, 2);
    while (wTransferState == TRANSFER_WAIT);
    if(wTransferState != TRANSFER_COMPLETE) {
        ili9341_Error();
    }
}


/**
  * @brief  Read output from the certain command
  * @param  LCDReg:      address of the selected register.
  * @retval None
  */
uint8_t ili9341_ReadCommand8(uint8_t c, uint8_t index) 
{
    uint8_t esc = 0xD9;	// V.P. "magic" code

    LCD_DC_LOW();
    LCD_CS_LOW();

    wTransferState = TRANSFER_WAIT;
    HAL_SPI_Transmit_IT(&hspi1, &esc, 1);
    while (wTransferState == TRANSFER_WAIT);
    if(wTransferState != TRANSFER_COMPLETE) {
        ili9341_Error();
    }

    LCD_DC_HIGH();
    esc = 0x10 + index;
    wTransferState = TRANSFER_WAIT;
    HAL_SPI_Transmit_IT(&hspi1, &esc, 1);
    while (wTransferState == TRANSFER_WAIT);
    if(wTransferState != TRANSFER_COMPLETE) {
        ili9341_Error();
    }
    LCD_CS_HIGH();

    LCD_DC_LOW();
    // CLCK LOW needed
    LCD_CS_LOW();
    wTransferState = TRANSFER_WAIT;
    HAL_SPI_Transmit_IT(&hspi1, &c, 1);
    while (wTransferState == TRANSFER_WAIT);
    if(wTransferState != TRANSFER_COMPLETE) {
        ili9341_Error();
    }
    LCD_DC_HIGH();
    rTransferState = TRANSFER_WAIT;
    HAL_SPI_Receive_IT(&hspi1, &esc, 1);
    while (rTransferState == TRANSFER_WAIT);
    if(rTransferState != TRANSFER_COMPLETE) {
        ili9341_Error();
    }
    LCD_CS_HIGH();

    return(esc);
}



/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
uint16_t ili9341_ReadID(void)
{
    uint16_t ToF = ILI9341_ID;	// TBD: Use ili9341_ReadCommand8 instead (ind=2/ind=3)
    return ToF;
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOn(void)
{
  /* Display On */
  ili9341_WriteReg(LCD_DISPLAY_ON);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOff(void)
{
  /* Display Off */
  ili9341_WriteReg(LCD_DISPLAY_OFF);
}


/**
  * @brief  Get LCD PIXEL WIDTH.
  * @param  None
  * @retval LCD PIXEL WIDTH.
  */
uint16_t ili9341_GetLcdPixelWidth(void)
{
  /* Return LCD PIXEL WIDTH */
  return ILI9341_LCD_PIXEL_WIDTH;
}

/**
  * @brief  Get LCD PIXEL HEIGHT.
  * @param  None
  * @retval LCD PIXEL HEIGHT.
  */
uint16_t ili9341_GetLcdPixelHeight(void)
{
  /* Return LCD PIXEL HEIGHT */
  return ILI9341_LCD_PIXEL_HEIGHT;
}

/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */ 

/**
  * @}
  */

/**
  * SPI =======================================================================================
  */


/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of Interrupt TxRx transfer
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // Turn LED on: Transfer in transmission/reception process is correct 
    HAL_GPIO_WritePin(GPIOB, LED_A_Pin, GPIO_PIN_SET);
    wTransferState = TRANSFER_COMPLETE;
}

/**
  * @brief  Tx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of Interrupt Tx transfer
  * @retval None
  */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // Turn LED on: Transfer in transmission/reception process is correct 
    HAL_GPIO_WritePin(GPIOB, LED_A_Pin, GPIO_PIN_SET);
    wTransferState = TRANSFER_COMPLETE;
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of Interrupt Tx transfer
  * @retval None
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // Turn LED on: Transfer in transmission/reception process is correct 
    HAL_GPIO_WritePin(GPIOB, LED_A_Pin, GPIO_PIN_SET);
    rTransferState = TRANSFER_COMPLETE;
}


/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    rTransferState = wTransferState = TRANSFER_ERROR;
}



/**
  * @brief  SPIx error treatment function.
  * @param  None
  * @retval None
  */
void ili9341_Error(void)
{
    printf("# SPI ERROR\n\r");
    /* De-initialize the SPI communication BUS */
    HAL_SPI_DeInit(&hspi1);
  
    /* Re- Initialize the SPI communication BUS */
    ili9341_IO_Init();
}

/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */ 

/**
  * @}
  */

/**
  * LCD =======================================================================================
  */


/**
  * @brief  Configures the LCD_SPI interface.
  * @retval None
  */
void ili9341_IO_Init(void)
{

    // HW Reset for LCD
    LCD_RST_LOW();
    ili9341_Delay(200);
    LCD_RST_HIGH();

    // Set or Reset the control line 
    LCD_CS_LOW();
    LCD_CS_HIGH();

    hspi1.Instance = LCD_SPIx;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;    // ~ 1Mbit/s for ILI9341
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi1.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi1);

    if(HAL_SPI_Init(&hspi1) != HAL_OK) {
        /* Initialization Error */
        ili9341_Error();
    }

    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

    ili9341_LED(1);

}


/**
  * @brief  Wait for loop in ms.
  * @param  Delay in ms.
  * @retval None
  */
void ili9341_Delay (uint32_t Delay)
{
  HAL_Delay(Delay);
}

/**
  * @brief  ON/OFF LCD LED
  * @param  None
  * @retval None
  */
void ili9341_LED(uint8_t state)
{
        if (state)
                HAL_GPIO_WritePin(LCD_PORT, LCD_LED_PIN, GPIO_PIN_SET);
        else
                HAL_GPIO_WritePin(LCD_PORT, LCD_LED_PIN, GPIO_PIN_RESET);
}

// Invert Display
void ili9341_invertDisplay(uint8_t i) 
{
	ili9341_WriteReg(i ? LCD_DINVON : LCD_DINVOFF);
}



/**
  * DRWAING and COLORING =======================================================================================
  */

/*! Utility to reverse binary bit.   */
uint32_t ili9341_revbin(uint32_t in)
{
    uint8_t i;
    uint32_t out;

    // byte = (byte * 0x0202020202ULL & 0x010884422010ULL) % 1023;

    for(i=0; i<32; i++) {
        out  <<= 1;
        out  |= in & 1;
        in >>= 1;
    } // end for

    return (out);
}

/*! Utility function to convert a 24-bit RGB value into a 16-bit RGB value. */
uint16_t LCD_rgb24b(uint32_t c) 
{
    uint8_t r = c >> 16;
    uint8_t g = c >> 8;
    uint8_t b = c;

    r = r >> 3;
    g = g >> 2;
    b = b >> 3;
    return ((r << 11) | (g << 5) | b);
}

/*! Utility function to convert three component colour values (R, G, B) into a 16-bit RGB value.*/
uint16_t LCD_rgb16b(uint8_t r, uint8_t g, uint8_t b) 
{
    r = r >> 3;
    g = g >> 2;
    b = b >> 3;
    return ((r << 11) | (g << 5) | b);
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t LCD_color565(uint8_t r, uint8_t g, uint8_t b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


/**
  * @brief  Set Cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @retval None
  */
void ili9341_SetCursor(uint16_t poX, uint16_t poY)
{
    uint16_t esc;

    esc = ILI9341_LCD_PIXEL_WIDTH - 1 - poX;
    ili9341_WriteReg(LCD_COLUMN_ADDR); // Column addr set
    ili9341_WriteData(esc >> 8);
    ili9341_WriteData(esc & 0xFF); // XSTART

    ili9341_WriteReg(LCD_PAGE_ADDR); // Row addr set
    ili9341_WriteData(poY>>8);
    ili9341_WriteData(poY); // YSTART
}


void ili9341_SetDisplayWindow(uint16_t x0, uint16_t Width, uint16_t Height) 
{
    uint16_t start, end;

    start = ILI9341_LCD_PIXEL_WIDTH - x0 - Width;
    end = ILI9341_LCD_PIXEL_WIDTH - x0 - 1;
    ili9341_WriteReg(LCD_COLUMN_ADDR); // Column addr set
    ili9341_WriteData(start >> 8);
    ili9341_WriteData(start & 0xFF); // XSTART
    ili9341_WriteData(end >> 8);
    ili9341_WriteData(end & 0xFF); // XEND

    start = y0;
    end = y0 + Height -1;
    ili9341_WriteReg(LCD_PAGE_ADDR); // Row addr set
    ili9341_WriteData(start>>8);
    ili9341_WriteData(start); // YSTART
    ili9341_WriteData(end>>8);
    ili9341_WriteData(end); // YEND

    ili9341_WriteReg(LCD_GRAM); // write to RAM
}


void ili9341_pushColor(uint16_t color) 
{
    ili9341_WriteData((0x00ff & color));
    ili9341_WriteData((color>>8));
}


void ili9341_WritePixel(uint16_t x, uint16_t y, uint16_t color) 
{

    if((x < 0) ||(x >= ILI9341_LCD_PIXEL_WIDTH) || (y < 0) || (y >= ILI9341_LCD_PIXEL_HEIGHT)) return;

    ili9341_SetCursor(x, y);
    ili9341_WriteReg(LCD_GRAM);
    ili9341_pushColor(color);

}


void ili9341_DrawVLine(uint16_t x, uint16_t y, uint16_t h, uint16_t color) 
{
    ili9341_DrawLine(x, y, x, y+h-1, color);
}


void ili9341_DrawHLine(uint16_t x, uint16_t y, uint16_t w, uint16_t color) 
{
    ili9341_DrawLine(x, y, x+w-1, y, color);
}




/*! Draw a straight line
 *  ====================
 *  This function uses Bresenham's algorithm to draw a straight line.  The line
 *  starts at coordinates (x0, y0) and extends to coordinates (x1, y1).  The line
 *  is drawn in color (color).
 */
void ili9341_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) 
{
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1) {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = (dx >> 1);
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++) {
    if (steep) {
      ili9341_WritePixel(y0, x0, color);
    } else {
      ili9341_WritePixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}


/**
  * @brief  Draws a bitmap picture loaded in the STM32 MCU internal memory.
  * @param  Xpos: Bmp X position in the LCD
  * @param  Ypos: Bmp Y position in the LCD
  * @param  pBmp: Pointer to Bmp picture address
  * @retval None
  */
void LCD_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp)
{
    uint32_t height = 0, width  = 0;
    uint32_t index, size;
    uint16_t color;
  
    // Read bitmap size 
    size = *(volatile uint16_t *) (pbmp + 2);
    size |= (*(volatile uint16_t *) (pbmp + 4)) << 16;

    // Read bitmap width 
    width = *(volatile uint16_t *) (pbmp + 18);
    width |= (*(volatile uint16_t *) (pbmp + 20)) << 16;
  
    // Read bitmap height 
    height = *(volatile uint16_t *) (pbmp + 22);
    height |= (*(volatile uint16_t *) (pbmp + 24)) << 16; 

    // Get bitmap data address offset 
    index = *(volatile uint16_t *) (pbmp + 10);
    index |= (*(volatile uint16_t *) (pbmp + 12)) << 16;
    //size = (size - index)>>1;
    size = height * width;
    pbmp += index;

    ili9341_SetDisplayWindow(Xpos, Ypos, width, height);

    LCD_DC_HIGH();
    LCD_CS_LOW();
    for(index = 0; index < size; index++) {
        // Write 16-bit GRAM Reg
        ili9341_WriteWord(*(volatile uint16_t *)pbmp);
        pbmp += 2;
    }
    LCD_DC_LOW(); LCD_DC_HIGH();
    LCD_CS_HIGH();

    ili9341_SetDisplayWindow(0, 0, ili9341_GetLcdPixelWidth(), ili9341_GetLcdPixelHeight());
    
}


/**
  * @brief  Displays picture.
  * @param  pdata: picture address.
  * @param  Xpos: Image X position in the LCD
  * @param  Ypos: Image Y position in the LCD
  * @param  Xsize: Image X size in the LCD
  * @param  Ysize: Image Y size in the LCD
  * @retval None
  */
void LCD_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata)
{
  uint32_t index = 0, size = 0;

  size = (Xsize * Ysize);

  /* Set Cursor */
  ili9341_SetCursor(Xpos, Ypos);  
  
  for(index = 0; index < size; index++)
  {
    /* Write 16-bit GRAM Reg */
    ili9341_WriteData(*(volatile uint16_t *)pdata); // 2 bits - coordinates + color
    pdata += 2;
  }
}


/************************ (C) COPYRIGHT Patoka Consulting Inc. *****END OF FILE****/
