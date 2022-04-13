/*
 * spi.h
 *
 * Created: 13.04.2022 14:10:10
 *  Author: qfj
 */ 


#ifndef SPI_H_
#define SPI_H_

void SPI_MasterInit(void);
uint8_t SPI_MasterTransmit(uint8_t cData);
void SPI_SlaveInit(void);
char SPI_SlaveReceive(void);


#endif /* SPI_H_ */