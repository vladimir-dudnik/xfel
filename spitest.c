#include <spitest.h>

struct spi_pdata_t {
	uint32_t swapbuf;
	uint32_t swaplen;
	uint32_t cmdlen;
};


static int spi_helper_init(struct xfel_ctx_t* ctx, struct spi_pdata_t* pdat)
{
    return fel_spi_init(ctx, &pdat->swapbuf, &pdat->swaplen, &pdat->cmdlen);
}


int spi_test(struct xfel_ctx_t* ctx)
{
	struct spi_pdata_t pdat;

	pdat.swapbuf = 0x00022000;
	pdat.swaplen = 65536;
	pdat.cmdlen = 4096;

    if(spi_helper_init(ctx, &pdat))
        return 1;

    return 0;
}