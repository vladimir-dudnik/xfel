/*
 * sys-spi.c
 *
 * Copyright(c) 2007-2021 Jianjun Jiang <8192542@qq.com>
 * Official site: http://xboot.org
 * Mobile phone: +86-18665388956
 * QQ: 8192542
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include <xboot.h>

enum {
	SPI_GCR	= 0x04,
	SPI_TCR	= 0x08,
	SPI_IER	= 0x10,
	SPI_ISR	= 0x14,
	SPI_FCR	= 0x18,
	SPI_FSR	= 0x1c,
	SPI_WCR	= 0x20,
	SPI_CCR	= 0x24,
	SPI_MBC	= 0x30,
	SPI_MTC	= 0x34,
	SPI_BCC	= 0x38,
	SPI_TXD	= 0x200,
	SPI_RXD	= 0x300,
};

//#define ORIGINAL_CODE 1
#if ORIGINAL_CODE
static void sys_spi_init(void)
{
	virtual_addr_t addr;
	u32_t val;

	/* Config GPIOC2, GPIOC3, GPIOC4 and GPIOC5 */
	addr = 0x02000060 + 0x00;
	val = read32(addr);
	val &= ~(0xf << ((2 & 0x7) << 2));
	val |= ((0x2 & 0xf) << ((2 & 0x7) << 2));
	write32(addr, val);

	val = read32(addr);
	val &= ~(0xf << ((3 & 0x7) << 2));
	val |= ((0x2 & 0xf) << ((3 & 0x7) << 2));
	write32(addr, val);

	val = read32(addr);
	val &= ~(0xf << ((4 & 0x7) << 2));
	val |= ((0x2 & 0xf) << ((4 & 0x7) << 2));
	write32(addr, val);

	val = read32(addr);
	val &= ~(0xf << ((5 & 0x7) << 2));
	val |= ((0x2 & 0xf) << ((5 & 0x7) << 2));
	write32(addr, val);

	/* Deassert spi0 reset */
	addr = 0x0200196c;
	val = read32(addr);
	val |= (1 << 16);
	write32(addr, val);

	/* Open the spi0 gate */
	addr = 0x02001940;
	val = read32(addr);
	val |= (1 << 31);
	write32(addr, val);

	/* Open the spi0 bus gate */
	addr = 0x0200196c;
	val = read32(addr);
	val |= (1 << 0);
	write32(addr, val);

	/* Select pll-periph0 for spi0 clk */
	addr = 0x02001940;
	val = read32(addr);
	val &= ~(0x3 << 24);
	val |= 0x1 << 24;
	write32(addr, val);

	/* Set clock pre divide ratio, divided by 1 */
	addr = 0x02001940;
	val = read32(addr);
	val &= ~(0x3 << 8);
	val |= 0x0 << 8;
	write32(addr, val);

	/* Set clock divide ratio, divided by 6 */
	addr = 0x02001940;
	val = read32(addr);
	val &= ~(0xf << 0);
	val |= (6 - 1) << 0;
	write32(addr, val);

	/* Set spi clock rate control register, divided by 2 */
	addr = 0x04025000;
	write32(addr + SPI_CCR, 0x1000);

	/* Enable spi0 and do a soft reset */
	addr = 0x04025000;
	val = read32(addr + SPI_GCR);
	val |= (1 << 31) | (1 << 7) | (1 << 1) | (1 << 0);
	write32(addr + SPI_GCR, val);
	while(read32(addr + SPI_GCR) & (1 << 31));

	val = read32(addr + SPI_TCR);
	val &= ~(0x3 << 0);
	val |= (1 << 6) | (1 << 2);
	write32(addr + SPI_TCR, val);

	val = read32(addr + SPI_FCR);
	val |= (1 << 31) | (1 << 15);
	write32(addr + SPI_FCR, val);
}
#else
static void sys_spi_init(void)
{
	virtual_addr_t addr;
	u32_t val;

	/* Config GPIOD10, GPIOD11, GPIOD12 and GPIOD13 */
	/* PD10 - CS,  PD11 - CLK, PD12 - MOSI, PD13 - MISO */
	/* 0x02000000 - GPIO base addr */
	/* 0x0090     - PD_CFG0 */
	/* 0x0094     - PD_CFG1 */
	/* 0x0098     - PD_CFG2 */
	/* 0x00a0     - PD_DAT */
	/* 0x00a4     - PD_DRV0 */
	/* 0x00a8     - PD_DRV1 */
	/* 0x00ac     - PD_DRV2 */
	/* 0x00b4     - PD_PULL0 */
	/* 0x00b8     - PD_PULL1 */

	/* 0x0090     - PD_CFG0 */
	/* +-------------------------------------------------------------------------------------------------------------+ */
	/* | 31 30 29 28 | 27 26 25 24 | 23 22 21 20 | 19 18 17 16 | 15 14 13 12 | 11 10  9  8 | 7  6  5  4 | 3  2  1  0 | */
	/* +-------------------------------------------------------------------------------------------------------------+ */
	/* |    PD7      |     PD6     |     PD5     |     PD4     |     PD3     |     PD2     |    PD1     |    PD0     | */
	/* +-------------------------------------------------------------------------------------------------------------+ */
	/*                                                                                                        |        */
	/*                                                                                                        +- 0000 Input      */
	/*                                                                                                        +- 0001 Output     */
	/*                                                                                                        +- 0010 LCD0-D2    */
	/*                                                                                                        +- 0011 LVDS0-V0P  */
	/*                                                                                                        +- 0100 DSI-D0P    */
	/*                                                                                                        +- 0101 TWI0-SCK   */
	/*                                                                                                        +- 0110 Reserved   */
	/*                                                                                                        +- 0111 Reserved   */
	/*                                                                                                        +- 1000 Reserved   */
	/*                                                                                                        +- 1001 Reserved   */
	/*                                                                                                        +- 1110 PD-EINT0   */
	/*                                                                                                        +- 1111 IO Disable */

	/* 0x0094 - PD_CFG1                                                                                                                 */
	/* +-------------------------------------------------------------------------------------------------------------+                  */
	/* | 31 30 29 28 | 27 26 25 24 | 23 22 21 20 | 19 18 17 16 | 15 14 13 12 | 11 10  9  8 | 7  6  5  4 | 3  2  1  0 |                  */
	/* +-------------------------------------------------------------------------------------------------------------+                  */
	/* |    PD15     |     PD14    |     PD13    |     PD12    |     PD11    |     PD10    |    PD9     |    PD8     |                  */
	/* +-------------------------------------------------------------------------------------------------------------+                  */
	/*                                                                               |                        |                         */
	/*                                                                               |                        +- 0000 Input             */
	/*                                                                               |                        +- 0001 Output            */
	/*                                                                               |                        +- 0010 LCD0-D12          */
	/*                                                                               |                        +- 0011 LVDS0-V3P         */
	/*                                                                               |                        +- 0100 DSI-D3P           */
	/*                                                                               |                        +- 0101 UART4-RX          */
	/*                                                                               |                        +- 0110..1001  Reserved   */
	/*                                                                               |                        +- 1110 PD-EINT8          */
	/*                                                                               |                        +- 1111 IO Disable        */
	/*                                                                               |                                                  */
	/*                                                                               +- 0000 Input                                      */
	/*                                                                               +- 0001 Output                                     */
	/*                                                                               +- 0010 LCD0-D14                                   */
	/*                                                                               +- 0011 LVDS1-V0P                                  */
	/*                                                                               +- 0100 SPI1-CS/DBI-CSX                            */
	/*                                                                               +- 0101 UART3-TX                                   */
	/*                                                                               +- 0110..1001 Reserved                             */
	/*                                                                               +- 1110 PD-EINT10                                  */
	/*                                                                               +- 1111 IO Disable                                 */

	addr = 0x02000000 + 0x0094;
	val = read32(addr);
//	val &= ~(0xf << ((2 & 0x7) << 2));
//	val |= ((0x2 & 0xf) << ((2 & 0x7) << 2));
	/*               PD13PD12PD11PD10         */
	val &= 0b11111111000000000000000011111111;
	val |= 0b00000000010001000100010000000000;
	write32(addr, val);

//	val = read32(addr);
//	val &= ~(0xf << ((3 & 0x7) << 2));
//	val |= ((0x2 & 0xf) << ((3 & 0x7) << 2));
//	write32(addr, val);

//	val = read32(addr);
//	val &= ~(0xf << ((4 & 0x7) << 2));
//	val |= ((0x2 & 0xf) << ((4 & 0x7) << 2));
//	write32(addr, val);

//	val = read32(addr);
//	val &= ~(0xf << ((5 & 0x7) << 2));
//	val |= ((0x2 & 0xf) << ((5 & 0x7) << 2));
//	write32(addr, val);

        /* 0x04026000 - SPI_DBI base address */
	/* 0x0004     - SPI_GCR */
//	addr = 0x04026000 + SPI_CGR;
//	val = read32(addr);
//	val |= 0x7;
//	write32(addr, val);

	/* 0x0008     - SPI_TCR */
//	addr = 0x04026000 + SPI_TCR;
//	val = read32(addr);

	/* 0x0010     - SPI_IER */
//	addr = 0x04026000 + SPI_IER;
//	val = read32(addr);

	/* 0x0014     - SPI_ISR */
//	addr = 0x04026000 + SPI_ISR;
//	val = read32(addr);

        /* 0x0018     - SPI_FCR */
//	addr = 0x04026000 + SPI_FCR;
//	val = read32(addr);

        /* 0x001c     - SPI_FSR */
//	addr = 0x04026000 + SPI_FSR;
//	val = read32(addr);

        /* 0x0020     - SPI_WCR */
//	addr = 0x04026000 + SPI_WCR;
//	val = read32(addr);

        /* 0x0028     - SPI_SAMP_DL */
//	addr = 0x04026000 + 0x0028;
//	val = read32(addr);

        /* 0x0030     - SPI_MBC */
//	addr = 0x04026000 + SPI_MBC;
//	val = read32(addr);

        /* 0x0034     - SPI_MTC */
//	addr = 0x04026000 + SPI_MTC;
//	val = read32(addr);

        /* 0x0038     - SPI_BCC */
//	addr = 0x04026000 + SPI_BCC;
//	val = read32(addr);

        /* 0x0040     - SPI_BATC */
//	addr = 0x04026000 + 0x0040;
//	val = read32(addr);

        /* 0x0044     - SPI_BA_CCR */
//	addr = 0x04026000 + 0x0044;
//	val = read32(addr);

        /* 0x0048     - SPI_TBR (TX data) */
//	addr = 0x04026000 + 0x0048;
//	val = read32(addr);

        /* 0x004c     - SPI_RBR (RX data) */
//	addr = 0x04026000 + 0x004c;
//	val = read32(addr);

        /* 0x0088     - SPI_NDMA_MODE_CTL */
//	addr = 0x04026000 + 0x0088;
//	val = read32(addr);

        /* 0x0200     - SPI_TXD */
//	addr = 0x04026000 + SPI_TXD;
//	val = read32(addr);

        /* 0x0300     - SPI_RXD */
//	addr = 0x04026000 + SPI_RXD;
//	val = read32(addr);

    /* 0x02001000 - CCU base address */
	/* 0x096c - SPI bus gating reset register */
	addr = 0x02001000 + 0x096c;
	val = read32(addr);
	
	/* Deassert spi1 reset, open SPI1 bus gate */
	// bit 17 - SPI1_RST, 1 - deasseert
	// bit 1 - SPI1_GATING, 1 - pass clock to SPI1
	val |= (1 << 17) + (1 << 1);
	write32(addr, val);

    /* 0x0944 - SPI1 clock regirster */
	/* Open the SPI1 gate */
	addr = 0x02001000 + 0x0944;
	val = read32(addr);
	// bit 31 - SCLK_GATING, 1 - clock is ON
	val |= (1 << 31);
	write32(addr, val);

	/* Select pll-periph0 for SPI1 clk */
	addr = 0x02001000 + 0x0944;
	val = read32(addr);
	val &= ~(0x3 << 24);
	// 26..24 - CLK_SRC_SEL, 001 - PLL_PERI(1x)
	val |= (1 << 24);
	write32(addr, val);

	/* Set SPI1 clock pre divide ratio, divided by 1 */
	addr = 0x02001000 + 0x0944;
	val = read32(addr);
	val &= ~(0x3 << 8);
	// bit 9..8 - FACTOR_N, 00 - 1
	val |= 0x0 << 8;
	write32(addr, val);

	/* Set SPI1 clock divide ratio, divided by 6 */
	addr = 0x02001000 + 0x0944;
	val = read32(addr);
	val &= ~(0xf << 0);
	// bits 3..0 - FACTOR_M, 0101 - FACTOR_M = 6
	val |= (6 - 1) << 0;
	write32(addr, val);

    /* 0x04026000 - SPI_DBI base address */
	/* Set SPI1 clock rate control register, divided by 2 */
	addr = 0x04026000;
	write32(addr + SPI_CCR, 0x1000);

	/* Enable SPI1 and do a soft reset */
	addr = 0x04026000;
	val = read32(addr + SPI_GCR);
	// bit 31 - SRST (software reset SPI controller)
	// bit 7  - TP_EN, 1 - stop transmit data when RXFIFO full
	// bit 1  - MODE, 0 - slave, 1 - master
	// bit 0  - EN, 1 - enable spi module control
	val |= (1 << 31) | (1 << 7) | (1 << 1) | (1 << 0);
	write32(addr + SPI_GCR, val);
	while(read32(addr + SPI_GCR) & (1 << 31));

    // bit 6 - SS owner, 1 - software
	// bit 2 - SPOL, CS active low (1 - idle)
	val = read32(addr + SPI_TCR);
	val &= ~(0x3 << 0);
	val |= (1 << 6) | (1 << 2);
	write32(addr + SPI_TCR, val);

    // bit 31 - TF_FIFO_RST, 1 - reset TXFIFO
	// bit 15 - RF_RST, 1 - reset RXFIFO
	val = read32(addr + SPI_FCR);
	val |= (1 << 31) | (1 << 15);
	write32(addr + SPI_FCR, val);
}
#endif
#if ORIGINAL_CODE
static void sys_spi_select(void)
{
	virtual_addr_t addr = 0x04025000;
	u32_t val;

	val = read32(addr + SPI_TCR);
	val &= ~((0x3 << 4) | (0x1 << 7));
	val |= ((0 & 0x3) << 4) | (0x0 << 7);
	write32(addr + SPI_TCR, val);
}
#else
static void sys_spi_select(void)
{
	virtual_addr_t addr = 0x04026000;
	u32_t val;

	val = read32(addr + SPI_TCR);
	val &= ~((0x3 << 4) | (0x1 << 7));
	// bits 5,4 - 00 spi0, 01 spi1, 10 spi2, 11 spi3
	// bit 6 - SS owner, 0 - spi controller, 1 - software
	// bit 7 - software SS level, 0 - SS low, 1 - SS high
	val |= ((1 & 0x3) << 4) | (0x0 << 7);
	write32(addr + SPI_TCR, val);
}
#endif
#if ORGINAL_CODE
static void sys_spi_deselect(void)
{
	virtual_addr_t addr = 0x04025000;
	u32_t val;

	val = read32(addr + SPI_TCR);
	val &= ~((0x3 << 4) | (0x1 << 7));
	val |= ((0 & 0x3) << 4) | (0x1 << 7);
	write32(addr + SPI_TCR, val);
}
#else
static void sys_spi_deselect(void)
{
	virtual_addr_t addr = 0x04026000;
	u32_t val;

	val = read32(addr + SPI_TCR);
	val &= ~((0x3 << 4) | (0x1 << 7));
	// bits 5,4 - 00 spi0, 01 spi1, 10 spi2, 11 spi3
	// bit 6 - SS owner, 0 - spi controller, 1 - software
	// bit 7 - software SS level, 0 - SS low, 1 - SS high
	val |= ((1 & 0x3) << 4) | (0x1 << 7);
	write32(addr + SPI_TCR, val);
}
#endif
#if ORIGINAL_CODE
static inline void sys_spi_write_txbuf(u8_t * buf, int len)
{
	virtual_addr_t addr = 0x04025000;
	int i;

	write32(addr + SPI_MTC, len & 0xffffff);
	write32(addr + SPI_BCC, len & 0xffffff);
	if(buf)
	{
		for(i = 0; i < len; i++)
			write8(addr + SPI_TXD, *buf++);
	}
	else
	{
		for(i = 0; i < len; i++)
			write8(addr + SPI_TXD, 0xff);
	}
}
#else
static inline void sys_spi_write_txbuf(u8_t* buf, int len)
{
	virtual_addr_t addr = 0x04026000;
	int i;

    // bits 23..0 - MWTC, master write transmit counter
	write32(addr + SPI_MTC, len & 0xffffff);
	// bits 23..0 - STC, master single mode transmit counter
	write32(addr + SPI_BCC, len & 0xffffff);
	if(buf)
	{
		for(i = 0; i < len; i++)
			write8(addr + SPI_TXD, *buf++);
	}
	else
	{
		for(i = 0; i < len; i++)
			write8(addr + SPI_TXD, 0xff);
	}
}
#endif
#if ORIGINAL_CODE
static void sys_spi_transfer(void * txbuf, void * rxbuf, u32_t len)
{
	virtual_addr_t addr = 0x04025000;
	u8_t * tx = txbuf;
	u8_t * rx = rxbuf;
	u8_t val;
	int n, i;

	while(len > 0)
	{
		n = (len <= 64) ? len : 64;
		write32(addr + SPI_MBC, n);
		sys_spi_write_txbuf(tx, n);
		write32(addr + SPI_TCR, read32(addr + SPI_TCR) | (1 << 31));
		while((read32(addr + SPI_FSR) & 0xff) < n);
		for(i = 0; i < n; i++)
		{
			val = read8(addr + SPI_RXD);
			if(rx)
				*rx++ = val;
		}
		if(tx)
			tx += n;
		len -= n;
	}
}
#else
static void sys_spi_transfer(void* txbuf, void* rxbuf, u32_t len)
{
	virtual_addr_t addr = 0x04026000;
	u8_t* tx = txbuf;
	u8_t* rx = rxbuf;
	u8_t val;
	int n, i;

	while(len > 0)
	{
		n = (len <= 64) ? len : 64;
		write32(addr + SPI_MBC, n);
		sys_spi_write_txbuf(tx, n);
		write32(addr + SPI_TCR, read32(addr + SPI_TCR) | (1 << 31));
		while((read32(addr + SPI_FSR) & 0xff) < n);
		for(i = 0; i < n; i++)
		{
			val = read8(addr + SPI_RXD);
			if(rx)
				*rx++ = val;
		}
		if(tx)
			tx += n;
		len -= n;
	}
}
#endif

enum {
	SPI_CMD_END	         = 0x00,
	SPI_CMD_INIT         = 0x01,
	SPI_CMD_SELECT       = 0x02,
	SPI_CMD_DESELECT     = 0x03,
	SPI_CMD_FAST         = 0x04,
	SPI_CMD_TXBUF        = 0x05,
	SPI_CMD_RXBUF        = 0x06,
	SPI_CMD_SPINOR_WAIT  = 0x07,
	SPI_CMD_SPINAND_WAIT = 0x08,
};

void sys_spi_run(void* cbuf)
{
	uint8_t tx[8], rx[8];
	u8_t c;
	u8_t* p = cbuf;
	u32_t addr, len;

	while(1)
	{
		c = *p++;
		if(c == SPI_CMD_INIT)
		{
			sys_spi_init();
		}
		else if(c == SPI_CMD_SELECT)
		{
			sys_spi_select();
		}
		else if(c == SPI_CMD_DESELECT)
		{
			sys_spi_deselect();
		}
		else if(c == SPI_CMD_FAST)
		{
			len  = p[0];
			sys_spi_transfer((void*)&p[1], NULL, len);
			p += (len + 1);
		}
		else if(c == SPI_CMD_TXBUF)
		{
			addr = (p[0] << 0) | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
			len  = (p[4] << 0) | (p[5] << 8) | (p[6] << 16) | (p[7] << 24);
			sys_spi_transfer((void*)((unsigned long)addr), NULL, len);
			p += 8;
		}
		else if(c == SPI_CMD_RXBUF)
		{
			addr = (p[0] << 0) | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
			len  = (p[4] << 0) | (p[5] << 8) | (p[6] << 16) | (p[7] << 24);
			sys_spi_transfer(NULL, (void*)((unsigned long)addr), len);
			p += 8;
		}
		else if(c == SPI_CMD_SPINOR_WAIT)
		{
			tx[0] = 0x05;
			do {
				sys_spi_transfer((void*)&tx[0], NULL, 1);
				sys_spi_transfer(NULL, (void*)&rx[0], 1);
			} while((rx[0] & 0x1) == 0x1);
		}
		else if(c == SPI_CMD_SPINAND_WAIT)
		{
			tx[0] = 0x0f;
			tx[1] = 0xc0;
			do {
				sys_spi_transfer((void*)&tx[0], NULL, 2);
				sys_spi_transfer(NULL, (void*)&rx[0], 1);
			} while((rx[0] & 0x1) == 0x1);
		}
		else
		{
			return;
		}
	}
}

void sys_spi_test(void)
{
    uint32_t clen = 0;
    uint8_t cbuf[256];

    cbuf[clen++] = 0x01;
    cbuf[clen++] = 0x33;
    cbuf[clen++] = 0x02;
    cbuf[clen++] = 0x5b;

    sys_spi_init();
    sys_spi_select();
    sys_spi_transfer(&cbuf[0], NULL, 4);
    sys_spi_deselect();

    return;
}
