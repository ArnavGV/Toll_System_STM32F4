#include "stm32f4xx.h"
#include "rfid.h"
#include <stdbool.h>

void rc522_init(void) {
    // Initialize SPI1 pins and SPI1 peripheral
    SPI1_Pins_Init();
    SPI1_Init();

    // Enable clock for GPIOA (for RST pin)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure PA8 as output (RST pin for RC522)
    GPIOA->MODER |= GPIO_MODER_MODER8_0;
    GPIOA->MODER &= ~GPIO_MODER_MODER8_1;

    // Enable clock for GPIOB (for CS pin)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Configure PB0 as output (CS pin for RC522)
    GPIOB->MODER |= GPIO_MODER_MODER0_0;
    GPIOB->MODER &= ~GPIO_MODER_MODER0_1;

    // Reset RC522 module
    GPIOA->BSRR = GPIO_BSRR_BR8;
    for (volatile int i = 0; i < 100000; i++);
    GPIOA->BSRR = GPIO_BSRR_BS8;
    for (volatile int i = 0; i < 100000; i++);

    rc522_reset();

    // Configure RC522
    rc522_regWrite8(MFRC522_REG_T_MODE, 0x80);
    rc522_regWrite8(MFRC522_REG_T_PRESCALER, 0xA9);
    rc522_regWrite8(MFRC522_REG_T_RELOAD_L, 0xE8);
    rc522_regWrite8(MFRC522_REG_T_RELOAD_H, 0x03);
    rc522_regWrite8(MFRC522_REG_TX_AUTO, 0x40);
    rc522_regWrite8(MFRC522_REG_MODE, 0x3D);

    rc522_antennaON();  // Open the antenna
}

void spi_cs_rfid_write(bool state) {
    if (state) {
        GPIOB->BSRR = GPIO_BSRR_BS0;  // Set PB0 high
    } else {
        GPIOB->BSRR = GPIO_BSRR_BR0;  // Set PB0 low
    }
}

uint8_t rc522_regRead8(uint8_t reg) {
    spi_cs_rfid_write(0);
    reg = ((reg << 1) & 0x7E) | 0x80;
    spi1_transmit(&reg, 1);
    uint8_t dataRd = 0;
    spi1_receive(&dataRd, 1);
    spi_cs_rfid_write(1);
    return dataRd;
}

void rc522_regWrite8(uint8_t reg, uint8_t data8) {
    spi_cs_rfid_write(0);
    uint8_t txData[2] = {0x7E & (reg << 1), data8};
    spi1_transmit(txData, 2);
    spi_cs_rfid_write(1);
}


void rc522_setBit(uint8_t reg, uint8_t mask) {
    rc522_regWrite8(reg, rc522_regRead8(reg) | mask);
}

void rc522_clearBit(uint8_t reg, uint8_t mask) {
    rc522_regWrite8(reg, rc522_regRead8(reg) & (~mask));
}

void rc522_reset(void) {
    rc522_regWrite8(0x01, 0x0F);
}

void rc522_antennaON(void) {
    uint8_t temp;
    temp = rc522_regRead8(MFRC522_REG_TX_CONTROL);
    if (!(temp & 0x03)) {
        rc522_setBit(MFRC522_REG_TX_CONTROL, 0x03);
    }
}

bool rc522_checkCard(uint8_t *id) {
    bool status = false;
    // Find cards, return card type
    status = rc522_request(PICC_REQIDL, id);
    if (status == true) {
        // Card detected, perform anti-collision to get card serial number
        status = rc522_antiColl(id);
    }
    rc522_halt();  // Put the card into hibernation
    return status;
}

bool rc522_request(uint8_t reqMode, uint8_t *tagType) {
    bool status = false;
    uint16_t backBits;
    rc522_regWrite8(MFRC522_REG_BIT_FRAMING, 0x07);
    tagType[0] = reqMode;
    status = rc522_toCard(PCD_TRANSCEIVE, tagType, 1, tagType, &backBits);
    if ((status != true) || (backBits != 0x10)) {
        status = false;
    }
    return status;
}

bool rc522_toCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen) {
    bool status = false;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint16_t i;

    switch (command) {
        case PCD_AUTHENT:
            irqEn = 0x12;
            waitIRq = 0x10;
            break;
        case PCD_TRANSCEIVE:
            irqEn = 0x77;
            waitIRq = 0x30;
            break;
        default:
            break;
    }

    rc522_regWrite8(MFRC522_REG_COMM_IE_N, irqEn | 0x80);
    rc522_clearBit(MFRC522_REG_COMM_IRQ, 0x80);
    rc522_setBit(MFRC522_REG_FIFO_LEVEL, 0x80);

    rc522_regWrite8(MFRC522_REG_COMMAND, PCD_IDLE);

    // Write data to the FIFO
    for (i = 0; i < sendLen; i++) {
        rc522_regWrite8(MFRC522_REG_FIFO_DATA, sendData[i]);
    }

    // Execute the command
    rc522_regWrite8(MFRC522_REG_COMMAND, command);
    if (command == PCD_TRANSCEIVE) {
        rc522_setBit(MFRC522_REG_BIT_FRAMING, 0x80);  // StartSend=1, transmission of data starts
    }

    // Waiting to receive data to complete
    i = 1000;
    do {
        n = rc522_regRead8(MFRC522_REG_COMM_IRQ);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    rc522_clearBit(MFRC522_REG_BIT_FRAMING, 0x80);  // StartSend=0

    if (i != 0) {
        if (!(rc522_regRead8(MFRC522_REG_ERROR) & 0x1B)) {
            status = true;
            if (n & irqEn & 0x01) {
                status = false;
            }

            if (command == PCD_TRANSCEIVE) {
                n = rc522_regRead8(MFRC522_REG_FIFO_LEVEL);
                lastBits = rc522_regRead8(MFRC522_REG_CONTROL) & 0x07;
                if (lastBits) {
                    *backLen = (n - 1) * 8 + lastBits;
                } else {
                    *backLen = n * 8;
                }

                if (n == 0) {
                    n = 1;
                }
                if (n > MFRC522_MAX_LEN) {
                    n = MFRC522_MAX_LEN;
                }

                // Reading the received data in FIFO
                for (i = 0; i < n; i++) {
                    backData[i] = rc522_regRead8(MFRC522_REG_FIFO_DATA);
                }
            }
        } else {
            status = false;
        }
    }

    return status;
}

bool rc522_antiColl(uint8_t* serNum) {
    bool status;
    uint8_t i;
    uint8_t serNumCheck = 0;
    uint16_t unLen;

    rc522_regWrite8(MFRC522_REG_BIT_FRAMING, 0x00);  // TxLastBits = BitFramingReg[2..0]
    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = rc522_toCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == true) {
        // Check card serial number
        for (i = 0; i < 4; i++) {
            serNumCheck ^= serNum[i];
        }
        if (serNumCheck != serNum[i]) {
            status = false;
        }
    }
    return status;
}

void rc522_halt(void) {
    uint16_t unLen;
    uint8_t buff[4];

    buff[0] = PICC_HALT;
    buff[1] = 0;
    rc522_calculateCRC(buff, 2, &buff[2]);

    rc522_toCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}

void rc522_calculateCRC(uint8_t* pIndata, uint8_t len, uint8_t* pOutData) {
    uint8_t i, n;

    rc522_clearBit(MFRC522_REG_DIV_IRQ, 0x04);  // CRCIrq = 0
    rc522_setBit(MFRC522_REG_FIFO_LEVEL, 0x80);  // Clear the FIFO pointer

    // Writing data to the FIFO
    for (i = 0; i < len; i++) {
        rc522_regWrite8(MFRC522_REG_FIFO_DATA, *(pIndata + i));
    }
    rc522_regWrite8(MFRC522_REG_COMMAND, PCD_CALCCRC);

    // Wait for CRC calculation to complete
    i = 0xFF;
    do {
        n = rc522_regRead8(MFRC522_REG_DIV_IRQ);
        i--;
    } while ((i != 0) && !(n & 0x04));  // CRCIrq = 1

    // Read CRC calculation result
    pOutData[0] = rc522_regRead8(MFRC522_REG_CRC_RESULT_L);
    pOutData[1] = rc522_regRead8(MFRC522_REG_CRC_RESULT_M);
}

bool rc522_compareIds(uint8_t *idCurrent, uint8_t *idReference) {
    uint8_t i;
    for (i = 0; i < 4; i++) {
        if (idCurrent[i] != idReference[i]) {
            return false;
        }
    }
    return true;
}
