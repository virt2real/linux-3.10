#ifdef CONFIG_V2R_PARSE_CMDLINE
static void v2r_parse_cmdline(char * string);
#endif

bool w1_run; // 1-wire init flag
bool lan_run; // LAN init flag
bool wlan_run; // WLAN init flag
bool spi0_run; // SPI0 init flag
bool led_run; // LED-triggers init flag
bool camera_run; // camera init flag
bool uart1_run; // UART1 init flag
bool ghid_k_run; //HID gadget keybord init flag
bool ghid_m_run; //HID gadget mouse init flag 
/*  PRTCSS functionality */

#define PRTCSS_BASE 0x01C69000

/* PRTC interface registers */

#define DAVINCI_PRTCIF_PID				0x00
#define DAVINCI_PRTCIF_CTLR				0x04
#define DAVINCI_PRTCIF_LDATA			0x08
#define DAVINCI_PRTCIF_UDATA			0x0C
#define DAVINCI_PRTCIF_INTEN			0x10
#define DAVINCI_PRTCIF_INTFLG			0x14

/* DAVINCI_PRTCIF_CTLR bit fields */

#define DAVINCI_PRTCIF_CTLR_BUSY		BIT(31)
#define DAVINCI_PRTCIF_CTLR_SIZE		BIT(25)
#define DAVINCI_PRTCIF_CTLR_DIR			BIT(24)
#define DAVINCI_PRTCIF_CTLR_BENU_MSB		BIT(23)
#define DAVINCI_PRTCIF_CTLR_BENU_3RD_BYTE	BIT(22)
#define DAVINCI_PRTCIF_CTLR_BENU_2ND_BYTE	BIT(21)
#define DAVINCI_PRTCIF_CTLR_BENU_LSB		BIT(20)
#define DAVINCI_PRTCIF_CTLR_BENU_MASK		(0x00F00000)
#define DAVINCI_PRTCIF_CTLR_BENL_MSB		BIT(19)
#define DAVINCI_PRTCIF_CTLR_BENL_3RD_BYTE	BIT(18)
#define DAVINCI_PRTCIF_CTLR_BENL_2ND_BYTE	BIT(17)
#define DAVINCI_PRTCIF_CTLR_BENL_LSB		BIT(16)
#define DAVINCI_PRTCIF_CTLR_BENL_MASK		(0x000F0000)

/* DAVINCI_PRTCIF_INTEN bit fields */

#define DAVINCI_PRTCIF_INTEN_RTCSS		BIT(1)
#define DAVINCI_PRTCIF_INTEN_RTCIF		BIT(0)
#define DAVINCI_PRTCIF_INTEN_MASK		(DAVINCI_PRTCIF_INTEN_RTCSS | DAVINCI_PRTCIF_INTEN_RTCIF)

/* DAVINCI_PRTCIF_INTFLG bit fields */

#define DAVINCI_PRTCIF_INTFLG_RTCSS		BIT(1)
#define DAVINCI_PRTCIF_INTFLG_RTCIF		BIT(0)
#define DAVINCI_PRTCIF_INTFLG_MASK		(DAVINCI_PRTCIF_INTFLG_RTCSS | DAVINCI_PRTCIF_INTFLG_RTCIF)

/* PRTC subsystem registers */

#define DAVINCI_PRTCSS_RTC_INTC_EXTENA1	(0x0C)
#define DAVINCI_PRTCSS_RTC_CTRL			(0x10)
#define DAVINCI_PRTCSS_RTC_WDT			(0x11)
#define DAVINCI_PRTCSS_RTC_TMR0			(0x12)
#define DAVINCI_PRTCSS_RTC_TMR1			(0x13)
#define DAVINCI_PRTCSS_RTC_CCTRL		(0x14)
#define DAVINCI_PRTCSS_RTC_SEC			(0x15)
#define DAVINCI_PRTCSS_RTC_MIN			(0x16)
#define DAVINCI_PRTCSS_RTC_HOUR			(0x17)
#define DAVINCI_PRTCSS_RTC_DAY0			(0x18)
#define DAVINCI_PRTCSS_RTC_DAY1			(0x19)
#define DAVINCI_PRTCSS_RTC_AMIN			(0x1A)
#define DAVINCI_PRTCSS_RTC_AHOUR		(0x1B)
#define DAVINCI_PRTCSS_RTC_ADAY0		(0x1C)
#define DAVINCI_PRTCSS_RTC_ADAY1		(0x1D)
#define DAVINCI_PRTCSS_RTC_CLKC_CNT		(0x20)

/* DAVINCI_PRTCSS_RTC_INTC_EXTENA1 */

#define DAVINCI_PRTCSS_RTC_INTC_EXTENA1_MASK	(0x07)

/* DAVINCI_PRTCSS_RTC_CTRL bit fields */

#define DAVINCI_PRTCSS_RTC_CTRL_WDTBUS		BIT(7)
#define DAVINCI_PRTCSS_RTC_CTRL_WEN			BIT(6)
#define DAVINCI_PRTCSS_RTC_CTRL_WDRT		BIT(5)
#define DAVINCI_PRTCSS_RTC_CTRL_WDTFLG		BIT(4)
#define DAVINCI_PRTCSS_RTC_CTRL_TE			BIT(3)
#define DAVINCI_PRTCSS_RTC_CTRL_TIEN		BIT(2)
#define DAVINCI_PRTCSS_RTC_CTRL_TMRFLG		BIT(1)
#define DAVINCI_PRTCSS_RTC_CTRL_TMMD		BIT(0)

/* DAVINCI_PRTCSS_RTC_CCTRL bit fields */

#define DAVINCI_PRTCSS_RTC_CCTRL_CALBUSY	BIT(7)
#define DAVINCI_PRTCSS_RTC_CCTRL_DAEN		BIT(5)
#define DAVINCI_PRTCSS_RTC_CCTRL_HAEN		BIT(4)
#define DAVINCI_PRTCSS_RTC_CCTRL_MAEN		BIT(3)
#define DAVINCI_PRTCSS_RTC_CCTRL_ALMFLG		BIT(2)
#define DAVINCI_PRTCSS_RTC_CCTRL_AIEN		BIT(1)
#define DAVINCI_PRTCSS_RTC_CCTRL_CAEN		BIT(0)



static inline void davinci_rtcif_write(u32 val, u32 addr){
    writel(val, IO_ADDRESS(PRTCSS_BASE) + addr);
}

static inline u32 davinci_rtcif_read(u32 addr){
    return readl(IO_ADDRESS(PRTCSS_BASE) + addr);
}

static inline void davinci_rtcif_wait(void){
    while (davinci_rtcif_read(DAVINCI_PRTCIF_CTLR) & DAVINCI_PRTCIF_CTLR_BUSY) cpu_relax();
}

static inline void davinci_rtcss_write(unsigned long val, u8 addr){
    davinci_rtcif_write(DAVINCI_PRTCIF_CTLR_BENL_LSB | addr, DAVINCI_PRTCIF_CTLR);
    davinci_rtcif_write(val, DAVINCI_PRTCIF_LDATA);
    davinci_rtcif_wait();
}

static inline u8 davinci_rtcss_read(u8 addr){
    davinci_rtcif_wait();
    davinci_rtcif_write(DAVINCI_PRTCIF_CTLR_DIR|DAVINCI_PRTCIF_CTLR_BENL_LSB|addr, DAVINCI_PRTCIF_CTLR);
    davinci_rtcif_wait();
    return davinci_rtcif_read(DAVINCI_PRTCIF_LDATA);
}

