#ifndef ENC28J60_INCLUDED
#define ENC28J60_INCLUDED

enum bank_number {
	BANK_0 = 0,
	BANK_1 = 1,
	BANK_2 = 2,
	BANK_3 = 3,
	BANK_COMMON = 255,
};

struct control_register {
	enum bank_number bank;
	uint8_t address;
};

#define CONTROL_REGISTER(banknum, addr) \
	{ \
		.bank = (banknum), \
		.address = (addr) \
	}

/* Common registers */
const struct control_register EIE	= CONTROL_REGISTER(BANK_COMMON, 0x1b);
const struct control_register EIR	= CONTROL_REGISTER(BANK_COMMON, 0x1c);
const struct control_register ESTAT	= CONTROL_REGISTER(BANK_COMMON, 0x1d);
const struct control_register ECON2	= CONTROL_REGISTER(BANK_COMMON, 0x1e);
const struct control_register ECON1	= CONTROL_REGISTER(BANK_COMMON, 0x1f);

/* Bank 0 */
const struct control_register ERDPTL	= CONTROL_REGISTER(BANK_0, 0x00);
const struct control_register ERDPTH	= CONTROL_REGISTER(BANK_0, 0x01);
const struct control_register EWRPTL	= CONTROL_REGISTER(BANK_0, 0x02);
const struct control_register EWRPTH	= CONTROL_REGISTER(BANK_0, 0x03);
const struct control_register ETXSTL	= CONTROL_REGISTER(BANK_0, 0x04);
const struct control_register ETXSTH	= CONTROL_REGISTER(BANK_0, 0x05);
const struct control_register ETXNDL	= CONTROL_REGISTER(BANK_0, 0x06);
const struct control_register ETXNDH	= CONTROL_REGISTER(BANK_0, 0x07);
const struct control_register ERXSTL	= CONTROL_REGISTER(BANK_0, 0x08);
const struct control_register ERXSTH	= CONTROL_REGISTER(BANK_0, 0x09);
const struct control_register ERXNDL	= CONTROL_REGISTER(BANK_0, 0x0a);
const struct control_register ERXNDH	= CONTROL_REGISTER(BANK_0, 0x0b);
const struct control_register ERXRDPTL	= CONTROL_REGISTER(BANK_0, 0x0c);
const struct control_register ERXRDPTH	= CONTROL_REGISTER(BANK_0, 0x0d);
const struct control_register ERXWRPTL	= CONTROL_REGISTER(BANK_0, 0x0e);
const struct control_register ERXWRPTH	= CONTROL_REGISTER(BANK_0, 0x0f);
/* DMA related registers go on*/

/* Bank 1 */
const struct control_register ERXFCON	= CONTROL_REGISTER(BANK_1, 0x18);

/* Bank 2 */
const struct control_register MACON1	= CONTROL_REGISTER(BANK_2, 0x00);
const struct control_register MACON3	= CONTROL_REGISTER(BANK_2, 0x02);
const struct control_register MACON4	= CONTROL_REGISTER(BANK_2, 0x03);
const struct control_register MABBIPG	= CONTROL_REGISTER(BANK_2, 0x04);
const struct control_register MAIPGL	= CONTROL_REGISTER(BANK_2, 0x06);
const struct control_register MAMXFLL	= CONTROL_REGISTER(BANK_2, 0x0a);
const struct control_register MAMXFLH	= CONTROL_REGISTER(BANK_2, 0x0b);

/* Bank 3 */
const struct control_register MAADR5 = CONTROL_REGISTER(BANK_3, 0x00);
const struct control_register MAADR6 = CONTROL_REGISTER(BANK_3, 0x01);
const struct control_register MAADR3 = CONTROL_REGISTER(BANK_3, 0x02);
const struct control_register MAADR4 = CONTROL_REGISTER(BANK_3, 0x03);
const struct control_register MAADR1 = CONTROL_REGISTER(BANK_3, 0x04);
const struct control_register MAADR2 = CONTROL_REGISTER(BANK_3, 0x05);
const struct control_register EREVID = CONTROL_REGISTER(BANK_3, 0x12);

#define ECON1_BSEL0		0b00000001
#define ECON1_BSEL1		0b00000010

#define ERXFCON_CRCEN		0b00010000

#define ESTAT_CLKRDY		0b00000001

#define MACON1_MARXEN		0b00000001
#define MACON1_PASSALL		0b00000010
#define MACON1_RXPAUS		0b00000100
#define MACON1_TXPAUS		0b00001000

#define MACON3_FULLDPX		0b00000001
#define MACON3_FRMLNEN		0b00000010
#define MACON3_HFRMLEN		0b00000100
#define MACON3_PHDREN		0b00001000
#define MACON3_TXCRCEN		0b00010000
#define MACON3_PADCFG0		0b00100000
#define MACON3_PADCFG1		0b01000000
#define MACON3_PADCFG2		0b10000000
#define PADCFG_60		0b00100000

#define MACON4_DEFER		0b01000000


const uint16_t ENC_TX_BUF_SIZE = 1024 * 3 - 1;
const uint16_t ENC_RX_START_ADDR = ENC_TX_BUF_SIZE + 1;
const uint16_t ENC_RX_END_ADDR = 0x1fff;

#define INT16_L(n) (uint8_t)((n) & 0xff)
#define INT16_H(n) (uint8_t)((n) >> 8)

enum spi_command {
	SPI_COM_RCR = 0x0,
	SPI_COM_RBM = 0x1,
	SPI_COM_WCR = 0x2,
	SPI_COM_WBM = 0x3,
	SPI_COM_BFS = 0x4,
	SPI_COM_BFC = 0x5,
	SPI_COM_SRC = 0xff,
};

union spi_operation {
	uint8_t raw8;
	struct __attribute__ ((packed)) {
		uint8_t arg:5;
		uint8_t opcode:3;
	} op;
};

#endif	/* ENC28J60_INCLUDED */
