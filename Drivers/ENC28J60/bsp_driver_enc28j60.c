/* Includes ------------------------------------------------------------------*/
#include "bsp_driver_enc28j60.h"
#include "hw_config.h"
#include "time.h"

/* Extern variables ----------------------------------------------------------*/ 
  
extern SPI_HandleTypeDef hethspi;

/* Private variables ---------------------------------------------------------*/
uint8_t ENC28J60_CurBank;
uint16_t gNextPacketPtr;
uint8_t erxfcon;

/* Private function prototypes -----------------------------------------------*/
uint8_t BSP_ENC28J60_ReadOp(uint8_t op, uint8_t address);
void BSP_ENC28J60_WriteOp(uint8_t op, uint8_t address, uint8_t data);
void BSP_ENC28J60_SetBank(uint8_t address);
uint8_t BSP_ENC28J60_Read(uint8_t address);
void BSP_ENC28J60_Write(uint8_t address, uint8_t data);
void BSP_ENC28J60_WriteWord(uint8_t address, uint16_t data);
void BSP_ENC28J60_PhyWrite(uint8_t address, uint16_t data);
void BSP_ENC28J60_WriteBuffer(uint16_t len, uint8_t* data);
void BSP_ENC28J60_ReadBuffer(uint16_t len, uint8_t* data);
uint16_t BSP_ENC28J60_ReadBufferWord();

/**
  * @brief  Initializes the ENC28J60 device.
  * @param  None
  * @retval ENC28J60 status
  */
uint8_t BSP_ENC28J60_Init(uint8_t* macaddr)
{
uint8_t eth_status = ETH_OK;
  HAL_SPI_Init(&hethspi);

  BSP_ENC28J60_WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);

  // check CLKRDY bit to see if reset is complete
  // The CLKRDY does not work. See Rev. B4 Silicon Errata point. Just wait.
  //while(!(enc28j60Read(ESTAT) & ESTAT_CLKRDY));
  HAL_Delay(50);
  // do bank 0 stuff
  // initialize receive buffer
  // 16-bit transfers, must write low byte first
  // set receive buffer start address
  gNextPacketPtr = RXSTART_INIT;                                              //TODO
  // Rx start
  BSP_ENC28J60_WriteWord(ERXSTL, RXSTART_INIT);
  // set receive pointer address
  BSP_ENC28J60_WriteWord(ERXRDPTL, RXSTART_INIT);
  // RX end
  BSP_ENC28J60_WriteWord(ERXNDL, RXSTOP_INIT);
  // TX start
  BSP_ENC28J60_WriteWord(ETXSTL, TXSTART_INIT);
  // TX end
  BSP_ENC28J60_WriteWord(ETXNDL, TXSTOP_INIT);
  // do bank 1 stuff, packet filter:
  // For broadcast packets we allow only ARP packtets
  // All other packets should be unicast only for our mac (MAADR)
  //
  // The pattern to match on is therefore
  // Type     ETH.DST
  // ARP      BROADCAST
  // 06 08 -- ff ff ff ff ff ff -> ip checksum for theses bytes=f7f9
  // in binary these poitions are:11 0000 0011 1111
  // This is hex 303F->EPMM0=0x3f,EPMM1=0x30
  
  //enc28j60Write(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN);
  //Change to add ERXFCON_BCEN recommended by epam
  //enc28j60Write(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN|ERXFCON_BCEN);
  erxfcon = ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_PMEN | ERXFCON_BCEN;
  BSP_ENC28J60_Write(ERXFCON, erxfcon);
  BSP_ENC28J60_WriteWord(EPMM0, 0x303f);
  BSP_ENC28J60_WriteWord(EPMCSL, 0xf7f9);
  //
  // do bank 2 stuff
  // enable MAC receive
  BSP_ENC28J60_Write(MACON1, MACON1_MARXEN | MACON1_TXPAUS | MACON1_RXPAUS);
  // bring MAC out of reset
  BSP_ENC28J60_Write(MACON2, 0x00);
  // enable automatic padding to 60bytes and CRC operations
  BSP_ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, MACON3,
                       MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FRMLNEN); //|MACON3_FULDPX);
  // set inter-frame gap (non-back-to-back)
  BSP_ENC28J60_WriteWord(MAIPGL, 0x0C12);
  // set inter-frame gap (back-to-back)
  BSP_ENC28J60_Write(MABBIPG, 0x12);
  // Set the maximum packet size which the controller will accept
  // Do not send packets longer than MAX_FRAMELEN:
  BSP_ENC28J60_WriteWord(MAMXFLL, MAX_FRAMELEN);
  // do bank 3 stuff
  // write MAC address
  // NOTE: MAC address in ENC28J60 is byte-backward
  BSP_ENC28J60_Write(MAADR5, macaddr[0]);
  BSP_ENC28J60_Write(MAADR4, macaddr[1]);
  BSP_ENC28J60_Write(MAADR3, macaddr[2]);
  BSP_ENC28J60_Write(MAADR2, macaddr[3]);
  BSP_ENC28J60_Write(MAADR1, macaddr[4]);
  BSP_ENC28J60_Write(MAADR0, macaddr[5]);
  
  eth_status |= BSP_ENC28J60_Read(MAADR5) != macaddr[0];
  eth_status |= BSP_ENC28J60_Read(MAADR4) != macaddr[1];
  eth_status |= BSP_ENC28J60_Read(MAADR3) != macaddr[2];
  eth_status |= BSP_ENC28J60_Read(MAADR2) != macaddr[3];
  eth_status |= BSP_ENC28J60_Read(MAADR1) != macaddr[4];
  eth_status |= BSP_ENC28J60_Read(MAADR0) != macaddr[5];
  
  // no loopback of transmitted frames
  BSP_ENC28J60_PhyWrite(PHCON2, PHCON2_HDLDIS);
  // switch to bank 0
  BSP_ENC28J60_SetBank(ECON1);
  // enable interrutps
  BSP_ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE | EIE_PKTIE);
  // enable packet reception
  BSP_ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
  
  return eth_status;
}

void BSP_ENC28J60_Blink(uint8_t n)
{
uint8_t i;
  for(i = 0; i < n; i++) 
  {
    // 0x880 is PHLCON LEDB=on, LEDA=on
    BSP_ENC28J60_PhyWrite(PHLCON, 0x3880);
    HAL_Delay(500);
    
    // 0x990 is PHLCON LEDB=off, LEDA=off
    BSP_ENC28J60_PhyWrite(PHLCON, 0x3990);
    HAL_Delay(500);
  }
  // 0x476 is PHLCON LEDA=links status, LEDB=receive/transmit
  BSP_ENC28J60_PhyWrite(PHLCON, 0x3476);
  HAL_Delay(100);
}

void BSP_ENC28J60_clkout(uint8_t clk)
{
  //setup clkout: 2 is 12.5MHz:
  BSP_ENC28J60_Write(ECOCON, clk & 0x7);
}

uint8_t BSP_ENC28J60_ReadOp(uint8_t op, uint8_t address) 
{
  uint8_t temp;
  BSP_ENC28J60_EnableCS();
  // issue read command
  temp = op | (address & ADDR_MASK);
  HAL_SPI_Transmit(&hethspi, &temp, 1, 100);
  HAL_SPI_Receive(&hethspi, &temp, 1, 100);
  if (address & 0x80)
    HAL_SPI_Receive(&hethspi, &temp, 1, 100);
  
  // release CS
  BSP_ENC28J60_DisableCS();
  return temp;
}

void BSP_ENC28J60_WriteOp(uint8_t op, uint8_t address, uint8_t data)
{
  uint8_t buf[2] = {op | (address & ADDR_MASK), data};
  BSP_ENC28J60_EnableCS();
  HAL_SPI_Transmit(&hethspi, buf, 2, 100);
  BSP_ENC28J60_DisableCS();
}

void BSP_ENC28J60_SetBank(uint8_t address) 
{
  if ((address & BANK_MASK) != ENC28J60_CurBank) 
  {
    BSP_ENC28J60_WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1,
                    ECON1_BSEL1 | ECON1_BSEL0);
    ENC28J60_CurBank = address & BANK_MASK;
    BSP_ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ENC28J60_CurBank >> 5);
  }
}


uint8_t BSP_ENC28J60_Read(uint8_t address) 
{
  // set the bank
  BSP_ENC28J60_SetBank(address);
  // do the read
  return BSP_ENC28J60_ReadOp(ENC28J60_READ_CTRL_REG, address);
}

void BSP_ENC28J60_Write(uint8_t address, uint8_t data)
{
  // set the bank
  BSP_ENC28J60_SetBank(address);
  // do the write
  BSP_ENC28J60_WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
}

void BSP_ENC28J60_WriteWord(uint8_t address, uint16_t data)
{
  BSP_ENC28J60_Write(address, data & 0xff);
  BSP_ENC28J60_Write(address + 1, data >> 8);
}

void BSP_ENC28J60_PhyWrite(uint8_t address, uint16_t data)
{
  // set the PHY register address
  BSP_ENC28J60_Write(MIREGADR, address);
  // write the PHY data
  BSP_ENC28J60_Write(MIWRL, data);
  BSP_ENC28J60_Write(MIWRH, data >> 8);
  // wait until the PHY write completes
  while (BSP_ENC28J60_Read(MISTAT) & MISTAT_BUSY)
    usleep(15);
}

void BSP_ENC28J60_ReadBuffer(uint16_t len, uint8_t* data)
{
uint8_t temp;                                                                   //TODO Remove 1 byte buffer
  BSP_ENC28J60_EnableCS();
  temp = ENC28J60_READ_BUF_MEM;
  HAL_SPI_Transmit(&hethspi, &temp, 1, 100);
  HAL_SPI_Receive(&hethspi, data, len, 100);

  BSP_ENC28J60_DisableCS();
}

static uint16_t BSP_ENC28J60_ReadBufferWord()
{
uint16_t result;
  BSP_ENC28J60_ReadBuffer(2, (uint8_t*) &result);
  return result;
}

extern uint16_t XXX;            //TODO
void BSP_ENC28J60_WriteBuffer(uint16_t len, uint8_t* data)
{
uint8_t temp;
//TODO Remove 1 byte buffer
  BSP_ENC28J60_EnableCS();
  temp = ENC28J60_WRITE_BUF_MEM;
  HAL_SPI_Transmit(&hethspi, &temp, 1, 100);
  HAL_SPI_Transmit(&hethspi, data, len, 100);
  BSP_ENC28J60_DisableCS();
}

uint32_t prevSeq = 0;
void BSP_ENC28J60_PacketSend(uint8_t* packet, uint16_t len) 
{
uint32_t Seq;
  Seq = (*(packet + 0x26)<<24) + (*(packet + 0x27)<<16) + (*(packet + 0x28)<<8) + (*(packet + 0x29));
  if (Seq - prevSeq == 2920)
    /*XXX++*/;
  prevSeq = Seq;
  // Check no transmit in progress
  while (BSP_ENC28J60_ReadOp(ENC28J60_READ_CTRL_REG, ECON1) & ECON1_TXRTS) 
  {
    // Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
    if ((BSP_ENC28J60_Read(EIR) & EIR_TXERIF)) {
      BSP_ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
      BSP_ENC28J60_WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
    }
  }
  
  // Set the write pointer to start of transmit buffer area
  BSP_ENC28J60_WriteWord(EWRPTL, TXSTART_INIT);
  // Set the TXND pointer to correspond to the packet size given
  BSP_ENC28J60_WriteWord(ETXNDL, (TXSTART_INIT + len));
  // write per-packet control byte (0x00 means use macon3 settings)
  BSP_ENC28J60_WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
  // copy the packet into the transmit buffer
  BSP_ENC28J60_WriteBuffer(len, packet);
  // send the contents of the transmit buffer onto the network
  BSP_ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
  // Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
  
  //while (BSP_ENC28J60_ReadOp(ENC28J60_READ_CTRL_REG, ECON1) & ECON1_TXRTS);
  //  while((BSP_ENC28J60_Read(EIR)&EIR_TXIF) == 0)
  //if((BSP_ENC28J60_Read(ESTAT)&ESTAT_TXABRT))
}

// Check if a packet is available in receive buffer.
// Returns: 1 a packet was retrieved, 0 otherwise.
uint8_t BSP_ENC28J60_PacketAvailable() 
{
  // check if a packet has been received and buffered
  //if( !(enc28j60Read(EIR) & EIR_PKTIF) ){
  // The above does not work. See Rev. B4 Silicon Errata point 6.
  if (BSP_ENC28J60_Read(EPKTCNT) == 0) 
    return (0);
  else
    return (1);
}
// Gets a packet from the network receive buffer, if one is available.
// The packet will be headed by an ethernet header.
//      maxlen  The maximum acceptable length of a retrieved packet.
//      packet  Pointer where packet data should be stored.
// Returns: Packet length in bytes if a packet was retrieved, zero otherwise.
uint16_t BSP_ENC28J60_PacketReceive(uint8_t* packet, uint16_t maxlen) 
{
uint16_t rxstat;
uint16_t len;
  // check if a packet has been received and buffered
  //if( !(enc28j60Read(EIR) & EIR_PKTIF) ){
  // The above does not work. See Rev. B4 Silicon Errata point 6.
  if (BSP_ENC28J60_Read(EPKTCNT) == 0) 
    return (0);
  
  //XXX++;
    
  // Set the read pointer to the start of the received packet
  BSP_ENC28J60_WriteWord(ERDPTL, gNextPacketPtr);
  // read the next packet pointer
  gNextPacketPtr = BSP_ENC28J60_ReadBufferWord();
  // read the packet length (see datasheet page 43)
  len = BSP_ENC28J60_ReadBufferWord() - 4;
  // read the receive status (see datasheet page 43)
  rxstat = BSP_ENC28J60_ReadBufferWord();
  // limit retrieve length
  if (len > maxlen - 1)
    len = maxlen - 1;
  
  // check CRC and symbol errors (see datasheet page 44, table 7-3):
  // The ERXFCON.CRCEN is set by default. Normally we should not
  // need to check this.
  if ((rxstat & 0x80) == 0)
    // invalid
    len = 0;
  else
    // copy the packet from the receive buffer
    BSP_ENC28J60_ReadBuffer(len, packet);

  // Move the RX read pointer to the start of the next received packet
  // This frees the memory we just read out
  BSP_ENC28J60_WriteWord(ERXRDPTL, gNextPacketPtr);
  // However, compensate for the errata point 13, rev B4: enver write an even address!
  if ((gNextPacketPtr - 1 < RXSTART_INIT) || (gNextPacketPtr - 1 > RXSTOP_INIT))
    BSP_ENC28J60_WriteWord(ERXRDPTL, RXSTOP_INIT);
  else
    BSP_ENC28J60_WriteWord(ERXRDPTL, (gNextPacketPtr - 1));
  
  // decrement the packet counter indicate we are done with this packet
  BSP_ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
  return len;  
}

void BSP_ENC28J60_PowerDown()
{
  BSP_ENC28J60_WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_RXEN);
  while (BSP_ENC28J60_Read(ESTAT) & ESTAT_RXBUSY)
    ;
  while (BSP_ENC28J60_Read(ECON1) & ECON1_TXRTS)
    ;
  BSP_ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PWRSV);
}

void BSP_ENC28J60_PowerUp()
{
  BSP_ENC28J60_WriteOp(ENC28J60_BIT_FIELD_CLR, ECON2, ECON2_PWRSV);
  while (!BSP_ENC28J60_Read(ESTAT) & ESTAT_CLKRDY)
    ;
}

/************************ (C) COPYRIGHT LXltd. *****************END OF FILE****/
