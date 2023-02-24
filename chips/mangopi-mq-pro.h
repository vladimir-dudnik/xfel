
#define REG_ADDR(BASE, OFFSET) (BASE) + (OFFSET)

#define GPIO_BASE      0x02000000

// PB 13 pins
#define GPIO_PB_CFG0   REG_ADDR(GPIO_BASE, 0x0030)
#define GPIO_PB_CFG1   0x0034
// 38, 3c
#define GPIO_PB_DAT    0x0040
#define GPIO_PB_DRV0   0x0044
#define GPIO_PB_DRV1   0x0048
// 4c, 50
#define GPIO_PB_PULL0  0x0054

// PC 8 pins
#define GPIO_PC_CFG0   0x0060
// 64, 68, 6c
#define GPIO_PC_DAT    0x0070
#define GPIO_PC_DRV0   0x0074
// 78, 7c, 80
#define GPIO_PC_PULL0  0x0084

// PD 23 pins
#define GPIO_PD_CFG0   0x0090
#define GPIO_PD_CFG1   0x0094
#define GPIO_PD_CFG2   0x0098
#define GPIO_PD_DAT    0x00a0
#define GPIO_PD_DRV0   0x00a4
#define GPIO_PD_DRV1   0x00a8
#define GPIO_PD_DRV2   0x00ac
#define GPIO_PD_PULL0  0x00b4
#define GPIO_PD_PULL1  0x00b8

// PE 18 pins
#define GPIO_PE_CFG0   0x00c0
#define GPIO_PE_CFG1   0x00c4
#define GPIO_PE_DAT    0x00d0
#define GPIO_PE_DRV0   0x00d4
#define GPIO_PE_DRV1   0x00d8
#define GPIO_PE_PULL0  0x00e4

// PF 7 pins
#define GPIO_PF_CFG0   0x00f0
#define GPIO_PF_DAT    0x0100
#define GPIO_PF_DRV0   0x0104
#define GPIO_PF_PULL0  0x0114

// PG 19 pins
#define GPIO_PG_CFG0   0x0120
#define GPIO_PG_CFG1   0x0124
#define GPIO_PG_DAT    0x0130
#define GPIO_PG_DRV0   0x0134
#define GPIO_PG_DRV1   0x0138
#define GPIO_PG_DRV3   0x0140
#define GPIO_PG_PULL0  0x0144

#define CCU_BASE  0x02001000
