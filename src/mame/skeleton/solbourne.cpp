// license:BSD-3-Clause
// copyright-holders: ajfa
/***************************************************************************

    Solbourne Series 5E workstation (KBus family)

    Cypress CY7C601 SPARC v7 IU at 40.1 MHz, Weitek 3171 FPU, Solbourne KBus
    interconnect and a Solbourne-specific, software-loaded MMU.

    Register layout from the OS/MP 4.1C kernel headers (sys/kbus/mmu.h,
    kbus.h, sysboard.h):

      ASI 0x80  MMCR   memory management control register
      ASI 0x81  FCR    fault cause register
      ASI 0x82  FVAR   fault virtual address register
      ASI 0x83  PDBA   page directory base register
      ASI 0x85/86/87   GTLB / FTLB / both invalidate
      ASI 0xa8/0xaa    load instruction / data TLB entry
      ASI 0xb1/0xb3    read instruction / data TLB entry
      ASI 0xc0  BID    board id register
      ASI 0xc1  LED    front panel LED register
      ASI 0xc8..0xcd   interrupt registers
      ASI 0xd0..0xd4   fault address / syndrome registers
      ASI 0xe0         2 KB diagnostic RAM

    Two translation windows always work, MMU enabled or not: 0xfe000000
    reaches I/O space 0, the boot PROM, and 0xff000000 reaches the first
    16 MB of physical memory.

    Physical I/O addresses are (space << 28) | (slot << 24) | offset, with
    space 0 the boot PROM, space 1 the ID space, and the system board in
    slot 7: the I/O ASIC at 0x90000000, the WD33C93A at 0x90000800 and the
    LANCE at 0x90001000.

    The four PROM dumps are the byte lanes of a 32-bit wide PROM and carry
    their address lines inverted, so the usable image is the word sequence
    reversed with lane _03 as the most significant byte. init_sols5e()
    undoes both; the reset vector at offset 0 then reads

        sethi   0x30, %l0
        or      %l0, 0xc0, %l0
        stha    %l0, [%g0] 0xc1     ! 0xc0c0 to the LEDs
        lda     [%g0] 0x80, %l0     ! MMCR, tests the cold start bit

    Work in progress.

***************************************************************************/

#include "emu.h"

#include "bus/rs232/rs232.h"
#include "cpu/sparc/sparc.h"
#include "bus/nscsi/hd.h"
#include "machine/msm58321.h"
#include "machine/nscsi_bus.h"
#include "machine/wd33c9x.h"
#include "machine/ram.h"
#include "machine/z80scc.h"

#include <utility>


#define LOG_MMU     (1U << 1)
#define LOG_CTRL    (1U << 2)
#define LOG_IO      (1U << 3)
#define LOG_LED     (1U << 4)
#define LOG_TLB     (1U << 5)
#define LOG_TRACE   (1U << 6)

#define VERBOSE 0
#include "logmacro.h"


namespace {

// The board registers all live at address 0 of their own ASI and take byte,
// halfword or word accesses: the value travels in the high part of the word,
// as the mask says.
static uint32_t reg_in(uint32_t data, uint32_t mem_mask)
{
	if (mem_mask == 0xff000000) return (data >> 24) & 0xff;
	if (mem_mask == 0xffff0000) return (data >> 16) & 0xffff;
	return data;
}

static uint32_t reg_out(uint32_t value, uint32_t mem_mask)
{
	if (mem_mask == 0xff000000) return (value & 0xff) << 24;
	if (mem_mask == 0xffff0000) return (value & 0xffff) << 16;
	return value;
}

// mmu.h
static constexpr uint32_t MMCR_ME   = 0x00000001;  // mmu enable
static constexpr uint32_t MMCR_CS   = 0x00000002;  // cold start
static constexpr uint32_t MMCR_FIO  = 0x00000004;  // fast I/O
static constexpr uint32_t MMCR_PIO  = 0x00000008;  // fast I/O pending
static constexpr uint32_t MMCR_WDEN = 0x00000010;  // watchdog
static constexpr uint32_t MMCR_WTAG = 0x00000020;  // clears the bus watcher tags
static constexpr uint32_t MMCR_KCB  = 0x00000040;  // KCB supplies the check bits
static constexpr uint32_t MMCR_EE   = 0x00000080;  // ECC checking
static constexpr uint32_t FCR_ECCM  = 0x00000004;  // multi-bit ECC error
static constexpr uint32_t FCR_TOF   = 0x00000001;  // KBus timeout
static constexpr uint32_t FCR_WDOG  = 0x00000002;  // watchdog reset
static constexpr uint32_t FCR_DTRAP = 0x00000008;  // double trap
static constexpr uint32_t FCR_POF   = 0x00000010;  // page out
static constexpr uint32_t FCR_WPF   = 0x00000020;  // write protection
static constexpr uint32_t FCR_UPF   = 0x00000040;  // user protection
static constexpr uint32_t FCR_TMISS = 0x00000080;  // TLB miss
static constexpr uint32_t BID_VALUE = 0x0003;      // the CPU sits in slot 3
static constexpr uint32_t SYS_SLOT  = 7;           // the system board, in slot 7
static constexpr uint32_t MEM_SLOT  = 5;           // one memory board, in slot 5

// memory board registers inside its own slot (memboard.h)
static constexpr uint32_t MEM_SIZE_REG   = 0x10000;
static constexpr uint32_t MEM_BASE_REG   = 0x20000;
static constexpr uint32_t MEM_ENABLE_REG = 0x30000;

// system board interrupt registers (sysboard.h)
static constexpr uint32_t SBIR_REG   = 0x30000;   // reading it also turns them off
static constexpr uint32_t SBIEN_REG  = 0x31000;   // reading it turns them on
static constexpr uint8_t  SBIR_DI    = 0x40;      // directed interrupt
static constexpr uint8_t  SBIR_DDID  = 0x3f;      // destination id
static constexpr uint8_t  SBIR_INFO  = 0x3f;      // undirected information field

// The system board takes sixteen interrupt lines, one per device, and sends
// them out as vector 0x80 plus the line number (sic.h, IOINTBASE, and table
// 5-1 of the theory of operation manual).
static constexpr int DMA_SETTLE_US = 0;   // how long the drive takes to answer

static constexpr uint32_t IOINT_BASE   = 0x80;
static constexpr uint32_t IOINT_SCSI   = 0x01;   // the WD33C93A
static constexpr uint32_t IOINT_ASIC   = 0x03;   // DMA page exhausted
static constexpr uint32_t IOINT_ENET   = 0x07;   // the LANCE
static constexpr uint32_t IOINT_SERIAL  = 0x09;   // serial ports A and B
static constexpr uint32_t IOINT_KBD  = 0x0b;   // keyboard and mouse
static constexpr uint32_t IOINT_CLOCK  = 0x0d;   // system timer
static constexpr uint32_t IOINT_PROFILE = 0x0f;   // profiling timer

// register that maps the frame buffer into a KBus space
static constexpr uint32_t VIDMAP_REG = 0x200000;
static constexpr uint8_t  VIDMAP_EN  = 0x80;      // frame buffer mapped
static constexpr uint32_t VRAM_SIZE  = 0x40000;   // 256 KB

// the two serial controllers, inside the board slot
static constexpr uint32_t SCC_KM_BASE  = 0x011000;  // keyboard and mouse
static constexpr uint32_t SCC_TTY_BASE = 0x012000;  // serial ports A and B
static constexpr uint32_t SCC_SIZE     = 0x000040;

// the real time clock, at a single address of the slot (clock.h)
static constexpr uint32_t RTC_REG      = 0x020000;

// The board I/O page is space 9, slot 0: the ASIC at the start and the SCSI
// controller at 0x800, address and data sixteen bytes apart just like the
// serial chips (sysboard.h).
static constexpr uint32_t OBIO_SPACE   = 9;
static constexpr uint32_t SCSI_ADDR    = 0x800;
static constexpr uint32_t SCSI_DATA    = 0x810;
static constexpr uint32_t ASIC_IR      = 0x000;   // interrupt register
static constexpr uint32_t ASIC_IR_PAGE = 0x0002;  // DMA ran past the page
static constexpr uint32_t ASIC_DMA_PAGE = 0x010;  // DMA physical page
static constexpr uint32_t ASIC_DMA_OFF  = 0x014;  // offset and direction
static constexpr uint32_t ASIC_DMA_NEXT  = 0x030;  // preloaded page (ioasic.h)
static constexpr uint8_t  MEM_READ_ENAB  = 0x01;
static constexpr uint8_t  MEM_WRITE_ENAB = 0x02;

// Memory ECC protects each doubleword with one check byte. The matrix comes
// from the table the POST itself writes to the KCB register for each of the
// 64 data bits: a Hsiao code, all 64 columns distinct, none zero and all of
// odd weight. The check byte of an all-zero doubleword is 0x0c, not zero.
static constexpr uint8_t ECC_ZERO = 0x0c;
static const uint8_t ECC_COL[64] =
{
	0xce, 0xcb, 0xd3, 0xd5, 0xd6, 0xd9, 0xda, 0xdc,
	0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x31, 0x34,
	0x0e, 0x0b, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c,
	0xe3, 0xe5, 0xe6, 0xe9, 0xea, 0xec, 0xf1, 0xf4,
	0x4f, 0x4a, 0x52, 0x54, 0x57, 0x58, 0x5b, 0x5d,
	0xa2, 0xa4, 0xa7, 0xa8, 0xab, 0xad, 0xb0, 0xb5,
	0x8f, 0x8a, 0x92, 0x94, 0x97, 0x98, 0x9b, 0x9d,
	0x62, 0x64, 0x67, 0x68, 0x6b, 0x6d, 0x70, 0x75,
};

// The check byte is the XOR of the columns of the bits that are set, so it
// can be taken a byte at a time from a table: eight lookups instead of
// sixty-four, and this runs on every memory read.
struct ecc_table
{
	uint8_t t[8][256];
	ecc_table()
	{
		for (int b = 0; b < 8; b++)
			for (int v = 0; v < 256; v++)
			{
				uint8_t c = 0;
				for (int i = 0; i < 8; i++)
					if (BIT(v, i))
						c ^= ECC_COL[b * 8 + i];
				t[b][v] = c;
			}
	}
};
static const ecc_table ECC_TAB;


static void solbourne_scsi_devices(device_slot_interface &device)
{
	device.option_add("harddisk", NSCSI_HARDDISK);
}

// The PROM leaves both ports at 9600 8N1; whatever is attached to port A has
// to match or characters are lost.
static DEVICE_INPUT_DEFAULTS_START(tty_9600)
	DEVICE_INPUT_DEFAULTS("RS232_RXBAUD", 0xff, RS232_BAUD_9600)
	DEVICE_INPUT_DEFAULTS("RS232_TXBAUD", 0xff, RS232_BAUD_9600)
	DEVICE_INPUT_DEFAULTS("RS232_DATABITS", 0xff, RS232_DATABITS_8)
	DEVICE_INPUT_DEFAULTS("RS232_PARITY", 0xff, RS232_PARITY_NONE)
	DEVICE_INPUT_DEFAULTS("RS232_STOPBITS", 0xff, RS232_STOPBITS_1)
DEVICE_INPUT_DEFAULTS_END

class solbourne_state : public driver_device
{
public:
	solbourne_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_ram(*this, "ram")
		, m_prom(*this, "maincpu")
		, m_scc_km(*this, "scc_km")
		, m_scc_tty(*this, "scc_tty")
		, m_rtc(*this, "rtc")
		, m_scsi(*this, "wd33c93a")
	{ }

	void sols5e(machine_config &config);
	void init_sols5e();

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	required_device<sparcv7_device> m_maincpu;
	required_device<ram_device> m_ram;
	required_memory_region m_prom;
	required_device<scc8530_device> m_scc_km;
	required_device<scc8530_device> m_scc_tty;
	required_device<msm58321_device> m_rtc;
	required_device<wd33c93a_device> m_scsi;
	uint32_t m_asic_ir = 0;      // ASIC interrupt register
	uint32_t m_dma_page = 0;     // physical page the DMA is in
	uint32_t m_dma_off = 0;      // offset shifted left two, plus direction
	bool m_drq = false;          // the controller has a byte waiting
	uint8_t m_wd_reg = 0;        // WD register selected through the indirect port
	uint8_t m_cdb[12]{};         // the command being assembled for the WD
	uint8_t m_wd_ptr = 0;
	uint8_t m_scsi_target = 0;
	bool m_dma_settle = false;   // fresh command: the drive needs time to answer
	void scsi_drq(int state);
	void dma_step();
	void bus_watch(uint32_t phys);
	void dma_flush();
	uint8_t m_dma_buf[32]{};        // the ASIC gathers a whole cache block
	uint32_t m_dma_block = 0xffffffff;
	uint32_t m_dma_valid = 0;    // which bytes of the block actually arrived
	uint32_t m_last_block = 0xffffffff;
	// The offset is thirteen bits in the high part of the register; the bottom
	// one says whether data goes from memory to the device.
	uint32_t dma_addr() const { return (m_dma_page << 13) | ((m_dma_off >> 2) & 0x1fff); }
	bool dma_to_disk() const { return BIT(m_dma_off, 0); }
	// the boot code learns that the controller is done by reading the chip's own
	// auxiliary status, not through the ASIC
	void scsi_irq(int state)
	{
		if (state && !m_scsi_irq_prev)
			io_intr(IOINT_SCSI, 1);
		m_scsi_irq_prev = state != 0;
	}
	uint8_t m_rtc_data = 0;      // nibble the clock drives on its pins
	void rtc_bit(int bit, int state)
	{
		m_rtc_data = (m_rtc_data & ~(1 << bit)) | (state ? (1 << bit) : 0);
	}
	void rtc_w(uint8_t data);
	uint8_t scc_r(scc8530_device &scc, uint32_t off);
	void scc_w(scc8530_device &scc, uint32_t off, uint8_t data);

	// The GTLB holds 2048 entries per side, indexed by bits 23:13 of the
	// address; the tag is bits 31:24, which the TIR compares in two halves (GM0
	// for 27:24 and GM1 for 31:28). Both translation windows use the same
	// entries without looking at the tag.
	static constexpr int TLB_ENTRIES = 0x800;
	static uint32_t tlb_index(uint32_t va) { return (va >> 13) & (TLB_ENTRIES - 1); }
	static uint32_t tlb_tag(uint32_t va) { return va >> 24; }

	struct tlb_entry
	{
		uint32_t vpn = 0;
		uint32_t tag = 0;       // full address the entry was loaded with
		uint32_t pte = 0;
		bool valid = false;
	};

	template <uint8_t Asi> uint32_t asi_r(offs_t offset, uint32_t mem_mask);
	template <uint8_t Asi> void asi_w(offs_t offset, uint32_t data, uint32_t mem_mask);
	template <uint8_t Asi> void asi_map(address_map &map) ATTR_COLD;
	template <std::size_t... I> void install_asis(std::index_sequence<I...>);

	TIMER_CALLBACK_MEMBER(system_tick);
	TIMER_CALLBACK_MEMBER(dma_tick);
	TIMER_CALLBACK_MEMBER(watchdog);
	TIMER_CALLBACK_MEMBER(wdog_expired);
	TIMER_CALLBACK_MEMBER(fio_timeout);
	TIMER_CALLBACK_MEMBER(io_deliver);
	uint32_t m_dma_next_page = 0;
	bool m_in_dma = false;    // lock against DMA re-entry
	bool m_next_page_set = false;
	emu_timer *m_sbi_timer = nullptr;
	TIMER_CALLBACK_MEMBER(sys_reset);
	// The watchdog period comes from test 18 itself: it waits 0.75 of the period
	// and demands no reset, then waits 1.1 and demands one; measured on the
	// emulated machine that puts the period between 63 and 92 ms.
	static constexpr int WDOG_MS = 84;
	void wdog_restart() { m_wdog->adjust((m_mmcr & MMCR_WDEN) ? attotime::from_msec(WDOG_MS) : attotime::never); }
	emu_timer *m_tick = nullptr;   // system timer, line d
	emu_timer *m_dma_timer = nullptr;
	emu_timer *m_wd = nullptr;
	emu_timer *m_wdog = nullptr;
	emu_timer *m_fio = nullptr;
	emu_timer *m_sysreset = nullptr;

	bool translate(uint8_t asi, uint32_t va, uint32_t mem_mask, bool write, int &space, uint32_t &phys);
	// MAME's CPU aligns the address on byte and halfword accesses; the exact
	// byte comes back from the mask, which is what the fault registers must
	// record
	static uint32_t low_bits(uint32_t mem_mask)
	{
		if (mem_mask == 0x00ff0000) return 1;
		if (mem_mask == 0x0000ff00 || mem_mask == 0x0000ffff) return 2;
		if (mem_mask == 0x000000ff) return 3;
		return 0;
	}
	void fault(uint32_t cause, uint32_t va, uint32_t mem_mask);

	// page absent, user protected or write protected
	bool protection_ok(uint32_t pte, uint8_t asi, bool write, uint32_t va, uint32_t mem_mask)
	{
		if (!BIT(pte, 0))                                { fault(FCR_POF, va, mem_mask); return false; }
		if ((asi == 0x08 || asi == 0x0a) && BIT(pte, 4)) { fault(FCR_UPF, va, mem_mask); return false; }
		// an ldstub is a single KBus transaction: write protection already
		// applies on the read half
		if ((write || m_maincpu->atomic_access()) && BIT(pte, 3))
			{ fault(FCR_WPF, va, mem_mask); return false; }
		return true;
	}
	// On the KBus only the PROM, memory, the ID space of the boards present and
	// the system board answer. Nothing replies for anything else and the
	// transaction ends in a timeout. The memory board answers only addresses
	// inside its window, and only if the enable register allows it: the base
	// register is the high bits of the physical address and the board size says
	// how many of them are compared (memboard.h).
	bool memory_answers(uint32_t addr, bool write) const
	{
		if (!(m_mem_enable & (write ? MEM_WRITE_ENAB : MEM_READ_ENAB)))
			return false;
		const uint32_t mask = ~(m_ram_size - 1);
		return (addr & mask) == ((uint32_t(m_mem_base) << 24) & mask);
	}
	bool responds(int space, uint32_t addr) const
	{
		if (space < 0)
			return memory_answers(addr, m_cur_write);
		// an I/O address carries the space in its top four bits: the board
		// decoders only see the bottom 28
		addr &= 0x0fffffff;
		if (space == 0)
			return true;
		if (space == vram_space())
			return addr < VRAM_SIZE;
		if (space == 1)
		{
			// ID space: every occupied slot answers in its ID PROM window, and
			// the system board also in its registers; nobody answers the rest
			const uint32_t slot = (addr >> 24) & 0x0f;
			const uint32_t off = addr & 0x00ffffff;
			if (slot != SYS_SLOT && slot != MEM_SLOT && slot != BID_VALUE)
				return false;
			// the memory board also has the size, base address and enable
			// registers (memboard.h)
			if (slot == MEM_SLOT && off >= MEM_SIZE_REG && off < MEM_ENABLE_REG + 0x10000)
				return true;
			if (slot == SYS_SLOT && off == VIDMAP_REG)
				return true;
			return (off < 0x5000) || (slot == SYS_SLOT && off < 0x40000);
		}
		// The system board also has its I/O register page: the ASIC, the SCSI
		// controller and the LANCE (sysboard.h). Like any I/O device it does not
		// take doubleword transfers, which is what test 19 checks. The LANCE, at
		// 0x1000, is not implemented yet: with nobody there the kernel does not
		// attach it and does not wait for it when bringing up the network.
		if (space == int(OBIO_SPACE) && addr < 0x1000)
			return !m_maincpu->dword_access();
		// nothing is attached to the remaining spaces yet
		return false;
	}
	void timeout(int space, uint32_t addr);

	// details of the access in flight, to fill in the fault registers
	uint32_t m_cur_va = 0;
	uint32_t m_cur_mask = 0;
	bool m_cur_write = false;
	uint32_t m_ftsr = 0;      // space and type of the transaction that timed out
	uint32_t m_ftor = 0;      // physical address of that transaction
	bool m_fault_armed = true;  // the fault registers keep the first fault
	uint32_t phys_r(int space, uint32_t addr, uint32_t mem_mask);
	void phys_w(int space, uint32_t addr, uint32_t data, uint32_t mem_mask);

	uint32_t m_mmcr = 0;
	uint32_t m_fcr = 0;
	uint32_t m_fvar = 0;
	uint32_t m_pdba = 0;
	uint32_t m_tir = 0;
	uint32_t m_ftir = 0;
	uint32_t m_dir = 0;
	uint32_t m_ixr = 0;
	uint32_t m_itxc = 0;
	uint32_t m_ipr = 0;
	uint32_t m_ipv = 0;
	uint32_t m_irxc = 0;
	uint32_t m_led = 0;
	uint8_t m_dgram[0x2000]{};

	tlb_entry m_itlb[TLB_ENTRIES];
	tlb_entry m_dtlb[TLB_ENTRIES];

	// The GTLB RAM has an area of its own for the two translation windows,
	// picked by the high address bits. The tag is not compared there, so an
	// entry loaded from 0xff serves 0xfe too, and the whole area is supervisor
	// only.
	static bool is_window(uint32_t va) { return (va & 0xfe000000) == 0xfe000000; }
	tlb_entry m_witlb[TLB_ENTRIES];
	tlb_entry m_wdtlb[TLB_ENTRIES];

	// The FTLB is the fast table the cache consults. It is loaded by the same
	// write as the GTLB (ASI 0xa8 or 0xaa) and read back by forcing an access
	// through ASI 0x70 or 0x72; the TIR returns the tag comparison in three
	// pieces (FM0 for 23:20, FM1 for 27:24 and FM3 for 31:28).
	static constexpr int FTLB_ENTRIES = 0x80;   // indexed by bits 19:13
	static uint32_t ftlb_index(uint32_t va) { return (va >> 13) & (FTLB_ENTRIES - 1); }
	tlb_entry m_iftlb[FTLB_ENTRIES];
	tlb_entry m_dftlb[FTLB_ENTRIES];

	// Cache tags: 128 KB in 32-byte blocks, 4096 entries indexed by bits 16:5
	// of the address. The tag holds the physical address the translation gave,
	// and the TIR reports the comparison in pieces (CM0 for 20:17, CM1 for
	// 24:21, CM2 for 27:25 and CM3 for 31:29, the last two also carrying the
	// valid bit) plus the block ownership bit.
	static constexpr int CACHE_BLOCKS = 0x1000;
	static uint32_t cache_index(uint32_t va) { return (va >> 5) & (CACHE_BLOCKS - 1); }

	struct cache_tag
	{
		uint32_t phys = 0;
		bool valid = false;
		bool own = false;
		bool dirty = false;     // dirty RAM: one bit per block
	};
	cache_tag m_ctag[CACHE_BLOCKS];

	// The bus watcher has a tag RAM of its own, one entry per cache block,
	// loaded and read through KBus diagnostic transactions (ASI 0xf4). The
	// comparison result comes out of the physical diagnostic register.
	struct watcher_tag
	{
		uint32_t tag = 0;
		bool own = false;
		bool valid = false;
	};
	watcher_tag m_wtag[CACHE_BLOCKS];
	uint32_t m_pdr = 0;      // state of the last diagnostic transaction
	uint32_t m_kcb = 0;      // check byte forced by the diagnostic
	uint32_t m_fpar = 0;     // physical address of the ECC fault
	uint32_t m_fes = 0;      // syndrome of the ECC fault
	bool m_ecc_armed = true; // the registers keep the first fault
	std::vector<uint8_t> m_eccb;   // one check byte per doubleword
	uint32_t ram_word(uint32_t a) const
	{
		const uint8_t *p = m_ram_ptr + a;
		return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16)
			 | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
	}
	static uint8_t ecc_check(uint32_t hi, uint32_t lo)
	{
		return uint8_t(ECC_ZERO
			^ ECC_TAB.t[0][lo & 0xff] ^ ECC_TAB.t[1][(lo >> 8) & 0xff]
			^ ECC_TAB.t[2][(lo >> 16) & 0xff] ^ ECC_TAB.t[3][(lo >> 24) & 0xff]
			^ ECC_TAB.t[4][hi & 0xff] ^ ECC_TAB.t[5][(hi >> 8) & 0xff]
			^ ECC_TAB.t[6][(hi >> 16) & 0xff] ^ ECC_TAB.t[7][(hi >> 24) & 0xff]);
	}
	void ecc_store(uint32_t addr);
	uint32_t ecc_load(uint32_t addr, uint32_t value);
	uint32_t m_ett = 0;      // encoded type of the last KBus transaction
	int m_force_ttype = -1;  // type forced by a diagnostic transaction

	// The physical diagnostic register does not carry the KBus transaction type
	// as is, but encoded in four bits (diag.h): cache ones are 8 to 11 and 14,
	// I/O ones 12 and 13, the doubleword 5 and the probe 7.
	static uint32_t ett_from_ttype(uint32_t tt)
	{
		switch (tt)
		{
		case 1:  return 0xa0;                    // IOB
		case 2:  return 0xb0;                    // write and invalidate
		case 3:  return 0x80;                    // read and invalidate
		case 7:  return 0x90;                    // cacheable read
		case 8: case 10: case 12: return 0xd0;   // I/O write
		case 9: case 11: case 13: case 17: return 0xc0;  // I/O read
		case 14: case 15: return 0x50;           // I/O doubleword
		case 31: return 0x70;                    // probe
		default: return 0x00;
		}
	}
	// The bus watcher serves an ordinary cacheable memory access with a read,
	// or a read and invalidate when it is a write; every other space gets an
	// I/O transaction the size of the access.
	void bus_ett(int space, bool write, uint32_t mem_mask)
	{
		if (space < 0)
		{
			m_ett = write ? 0x80 : 0x90;
			return;
		}
		if (m_maincpu->dword_access())
			m_ett = 0x50;
		else
			m_ett = write ? 0xd0 : 0xc0;
	}
	uint8_t m_cdata[CACHE_BLOCKS * 32]{};     // cache data RAM
	uint32_t m_flush[CACHE_BLOCKS]{};         // flush RAM: physical bits 28:17, in place
	bool m_corrupt[CACHE_BLOCKS]{};           // corrupt block RAM, one bit per block

	// ASIs 0x40 to 0x7f are cache accesses with flags (diag.h): 0x20 updates
	// the TIR, 0x10 takes no exception on a TLB miss, 0x08 forces a hit, 0x02
	// picks data space and 0x01 supervisor.
	static bool is_cache_asi(uint8_t asi) { return asi >= 0x40 && asi < 0x80; }
	uint32_t cache_data_r(uint32_t va, uint32_t mem_mask) const;
	void cache_data_w(uint32_t va, uint32_t data, uint32_t mem_mask, bool set_dirty);

	// physical address the FTLB gives for an address, valid bit ignored
	uint32_t ftlb_phys(bool insn, uint32_t va) const
	{
		const tlb_entry &e = (insn ? m_iftlb : m_dftlb)[ftlb_index(va)];
		return (e.pte >> 13) << 13;
	}

	void cache_probe(bool insn, uint32_t va);

	uint32_t m_ram_size = 0;
	uint8_t *m_ram_ptr = nullptr;
	uint32_t *m_prom_ptr = nullptr;
	uint32_t m_prom_size = 0;
	const uint8_t *m_sysid = nullptr;
	uint8_t m_memid[0x800]{};   // synthetic ID PROM of the memory board
	uint8_t m_slotcfg[16][32]{}; // configuration registers of each slot
	uint8_t m_vidmap = 0;       // KBus space the frame buffer shows up in
	std::vector<uint8_t> m_vram;   // frame buffer memory
	int vram_space() const
	{
		return (m_vidmap & VIDMAP_EN) ? int((m_vidmap >> 1) & 0x0f) : -2;
	}
	uint8_t m_sbir = 0;         // system board interrupt register
	bool m_sbi_enabled = false; // interrupt transmission allowed
	int m_sbi_left = 16;        // vectors left from the reset queue
	uint16_t m_io_pend = 0;     // device lines pending on the board
	bool m_scsi_irq_prev = false;
	bool m_scc_tty_irq_prev = false;
	bool m_scc_km_irq_prev = false;
	// copy of what leaves the serial console, so the machine can be answered
	// when it asks instead of typing blind against the clock
	FILE *m_console_copy = nullptr;
	void io_intr(uint32_t linea, int state);
	void io_poll();
	bool send_vector(uint32_t dest, uint32_t vector);
	bool recv_vector(uint32_t dest, uint32_t vector);
	void sbi_next();
	uint8_t m_mem_base = 0;     // memory board base address
	uint8_t m_mem_enable = 0;   // read and write enable
	uint8_t m_cpuid[0x800]{};   // synthetic ID PROM of the CPU board
	uint8_t m_eeprom[0x400]{};
	bool m_kernel_started = false;
	uint32_t m_last_iopte = 0;
};


void solbourne_state::machine_start()
{
	m_ram_ptr = m_ram->pointer();
	m_ram_size = m_ram->size();
	if (const char *v = getenv("SOLB_CONSOLE"))
		m_console_copy = fopen(v, "w");
	m_eccb.assign(m_ram_size / 8, ecc_check(0, 0));

	// The EEPROM comes up blank. The boot code accepts it when the sum of all
	// its contents equals the structure version, 0x13 (eeprom.h), which zeros
	// do not satisfy, so the PROM writes its own factory values: manual boot
	// and both serial ports at 9600 with eight bits.
	std::fill(std::begin(m_eeprom), std::end(m_eeprom), 0);

	m_vram.assign(VRAM_SIZE, 0);
	m_prom_ptr = reinterpret_cast<uint32_t *>(m_prom->base());
	m_prom_size = m_prom->bytes();
	m_sysid = memregion("sysid")->base();


	// the board system timer, at a hundred ticks per second, which is SunOS 4's
	// HZ; the manual says it needs no service, so each tick only raises its line
	m_tick = timer_alloc(FUNC(solbourne_state::system_tick), this);
	m_tick->adjust(attotime::from_hz(100), 0, attotime::from_hz(100));

	// The drive does not answer the instant the command is issued: selection,
	// command phase and the drive's own latency add up to hundreds of
	// microseconds. The OS/MP driver counts on that: it fills its buffer with
	// 0xFECECACA AFTER issuing the command and then waits for the DMA to
	// overwrite it. Without this delay the data arrives before the poison, the
	// poison wipes it out and the driver hangs.
	m_dma_timer = timer_alloc(FUNC(solbourne_state::dma_tick), this);
	m_wd = timer_alloc(FUNC(solbourne_state::watchdog), this);
	m_wd->adjust(attotime::from_usec(20), 0, attotime::from_usec(20));
	m_wdog = timer_alloc(FUNC(solbourne_state::wdog_expired), this);
	m_fio = timer_alloc(FUNC(solbourne_state::fio_timeout), this);
	m_sbi_timer = timer_alloc(FUNC(solbourne_state::io_deliver), this);
	m_sysreset = timer_alloc(FUNC(solbourne_state::sys_reset), this);

	// The system board ID PROM is the dump of a real machine. The processor and
	// memory ones are built here with the same structure: the board type letter,
	// the header length at indices 8 to 11, a byte that makes the checksum come
	// out, and the description from 13 on, zero terminated. On the memory one
	// the byte after that string is the board size in megabytes, which has to be
	// a multiple of 16.
	auto build_id = [] (uint8_t *id, char type, uint8_t minor, const char *desc, uint8_t megabytes)
	{
		std::fill_n(id, 0x800, 0);
		id[0] = uint8_t(type);
		id[1] = minor;                   // minor type
		id[2] = 'A';                     // revision level
		id[3] = 'A';
		id[11] = 22;                     // header length
		const size_t len = std::strlen(desc);
		std::copy_n(desc, len, id + 13);
		id[13 + len] = 0;
		if (megabytes)
			id[14 + len] = megabytes;
		uint8_t sum = 0;
		for (int i = 0; i < id[11]; i++)
			sum += id[i];
		id[12] = uint8_t(-sum);
	};
	build_id(m_cpuid, 'P', 2, "Kbus Processor Board", 0);
	build_id(m_memid, 'M', 2, "Kbus Memory Board", 32);

	save_item(NAME(m_eeprom));
	// the clock starts from the host time: without setting it the counters come
	// up at a zero that is not a valid date
	system_time now;
	machine().current_datetime(now);
	m_rtc->set_current_time(now);

	save_item(NAME(m_rtc_data));
	save_item(NAME(m_asic_ir));
	save_item(NAME(m_dma_page));
	save_item(NAME(m_dma_off));
	save_item(NAME(m_in_dma));
	save_item(NAME(m_drq));
	save_item(NAME(m_mmcr));
	save_item(NAME(m_fcr));
	save_item(NAME(m_fvar));
	save_item(NAME(m_pdba));
	save_item(NAME(m_led));
	save_item(NAME(m_dgram));
	save_item(NAME(m_eccb));
	save_item(NAME(m_cdata));
	save_item(NAME(m_flush));
	save_item(NAME(m_kernel_started));
	save_item(NAME(m_cdb));
	save_item(NAME(m_corrupt));
	save_item(NAME(m_cur_mask));
	save_item(NAME(m_cur_va));
	save_item(NAME(m_cur_write));
	save_item(NAME(m_dir));
	save_item(NAME(m_dma_block));
	save_item(NAME(m_dma_buf));
	save_item(NAME(m_dma_settle));
	save_item(NAME(m_dma_valid));
	save_item(NAME(m_ecc_armed));
	save_item(NAME(m_ett));
	save_item(NAME(m_fault_armed));
	save_item(NAME(m_fes));
	save_item(NAME(m_force_ttype));
	save_item(NAME(m_fpar));
	save_item(NAME(m_ftir));
	save_item(NAME(m_ftor));
	save_item(NAME(m_ftsr));
	save_item(NAME(m_io_pend));
	save_item(NAME(m_ipr));
	save_item(NAME(m_ipv));
	save_item(NAME(m_irxc));
	save_item(NAME(m_itxc));
	save_item(NAME(m_ixr));
	save_item(NAME(m_kcb));
	save_item(NAME(m_mem_base));
	save_item(NAME(m_mem_enable));
	save_item(NAME(m_pdr));
	save_item(NAME(m_sbi_enabled));
	save_item(NAME(m_sbi_left));
	save_item(NAME(m_sbir));
	save_item(NAME(m_scc_km_irq_prev));
	save_item(NAME(m_scc_tty_irq_prev));
	save_item(NAME(m_scsi_target));
	save_item(NAME(m_scsi_irq_prev));
	save_item(NAME(m_slotcfg));
	save_item(NAME(m_tir));
	save_item(NAME(m_last_block));
	save_item(NAME(m_last_iopte));
	save_item(NAME(m_vidmap));
	save_item(NAME(m_wd_ptr));
	save_item(NAME(m_wd_reg));

	// the TLB entries and the cache tags are structures: they go field by field
	for (int i = 0; i < TLB_ENTRIES; i++)
	{
		save_item(NAME(m_itlb[i].vpn), i);
		save_item(NAME(m_itlb[i].tag), i);
		save_item(NAME(m_itlb[i].pte), i);
		save_item(NAME(m_itlb[i].valid), i);
		save_item(NAME(m_dtlb[i].vpn), i);
		save_item(NAME(m_dtlb[i].tag), i);
		save_item(NAME(m_dtlb[i].pte), i);
		save_item(NAME(m_dtlb[i].valid), i);
		save_item(NAME(m_witlb[i].vpn), i);
		save_item(NAME(m_witlb[i].tag), i);
		save_item(NAME(m_witlb[i].pte), i);
		save_item(NAME(m_witlb[i].valid), i);
		save_item(NAME(m_wdtlb[i].vpn), i);
		save_item(NAME(m_wdtlb[i].tag), i);
		save_item(NAME(m_wdtlb[i].pte), i);
		save_item(NAME(m_wdtlb[i].valid), i);
	}
	for (int i = 0; i < FTLB_ENTRIES; i++)
	{
		save_item(NAME(m_iftlb[i].vpn), i);
		save_item(NAME(m_iftlb[i].tag), i);
		save_item(NAME(m_iftlb[i].pte), i);
		save_item(NAME(m_iftlb[i].valid), i);
		save_item(NAME(m_dftlb[i].vpn), i);
		save_item(NAME(m_dftlb[i].tag), i);
		save_item(NAME(m_dftlb[i].pte), i);
		save_item(NAME(m_dftlb[i].valid), i);
	}
	for (int i = 0; i < CACHE_BLOCKS; i++)
	{
		save_item(NAME(m_ctag[i].phys), i);
		save_item(NAME(m_ctag[i].valid), i);
		save_item(NAME(m_ctag[i].own), i);
		save_item(NAME(m_ctag[i].dirty), i);
		save_item(NAME(m_wtag[i].tag), i);
		save_item(NAME(m_wtag[i].own), i);
		save_item(NAME(m_wtag[i].valid), i);
	}
}

void solbourne_state::machine_reset()
{
	m_mmcr = MMCR_CS;
	m_fcr = 0;
	m_fvar = 0;
	m_pdba = 0;
	m_led = 0;
	for (auto &e : m_itlb) e.valid = false;
	for (auto &e : m_dtlb) e.valid = false;
	for (auto &e : m_witlb) e.valid = false;
	for (auto &e : m_wdtlb) e.valid = false;
	for (auto &e : m_iftlb) e.valid = false;
	for (auto &e : m_dftlb) e.valid = false;
}


// The reset comes through the system board and does not touch the diagnostic
// RAM: the boot code leaves the reset type there, which is why the next one is
// warm and does not repeat the tests.
TIMER_CALLBACK_MEMBER(solbourne_state::sys_reset)
{
	LOGMASKED(LOG_CTRL, "system reset\n");
	m_maincpu->reset();
	m_mmcr = 0;                    // no longer a cold start
	m_fcr = 0;
}

TIMER_CALLBACK_MEMBER(solbourne_state::fio_timeout)
{
	LOGMASKED(LOG_IO, "fast I/O: level 8 interrupt\n");
	m_maincpu->set_input_line(SPARC_IRQ8, ASSERT_LINE);
}

TIMER_CALLBACK_MEMBER(solbourne_state::wdog_expired)
{
	// the watchdog resets the machine and says so in the cause register; the
	// counter clears when the MMCR is read
	LOGMASKED(LOG_MMU, "watchdog: CPU reset\n");
	m_maincpu->reset();
	m_mmcr = 0;
	m_fcr = FCR_WDOG;
}

TIMER_CALLBACK_MEMBER(solbourne_state::watchdog)
{
	// a trap taken with traps disabled leaves the CPU in error mode; on this
	// machine that resets the system and records it in the cause register, which
	// is how the PROM finds out
	if (!m_maincpu->in_error_mode())
		return;

	LOGMASKED(LOG_MMU, "double trap: CPU reset\n");
	m_maincpu->reset();
	m_mmcr = 0;                    // no longer a cold start
	m_fcr = FCR_DTRAP;
}

TIMER_CALLBACK_MEMBER(solbourne_state::system_tick)
{
	// the power-on test does not expect system board interrupts: the timer does
	// not start asking until the kernel is running
	if (!m_kernel_started)
		return;
	io_intr(IOINT_CLOCK, 1);
}


uint32_t solbourne_state::cache_data_r(uint32_t va, uint32_t mem_mask) const
{
	const uint32_t off = (cache_index(va) << 5) | (va & 0x1c);
	return (m_cdata[off] << 24) | (m_cdata[off + 1] << 16)
		 | (m_cdata[off + 2] << 8) | m_cdata[off + 3];
}

void solbourne_state::cache_data_w(uint32_t va, uint32_t data, uint32_t mem_mask, bool set_dirty)
{
	m_ctag[cache_index(va)].dirty = set_dirty;

	const uint32_t off = (cache_index(va) << 5) | (va & 0x1c);
	uint32_t cur = (m_cdata[off] << 24) | (m_cdata[off + 1] << 16)
				 | (m_cdata[off + 2] << 8) | m_cdata[off + 3];
	cur = (cur & ~mem_mask) | (data & mem_mask);
	m_cdata[off]     = cur >> 24;
	m_cdata[off + 1] = cur >> 16;
	m_cdata[off + 2] = cur >> 8;
	m_cdata[off + 3] = cur;
}

void solbourne_state::cache_probe(bool insn, uint32_t va)
{
	const cache_tag &t = m_ctag[cache_index(va)];
	const tlb_entry &e = (insn ? m_iftlb : m_dftlb)[ftlb_index(va)];
	const uint32_t p = (e.pte >> 13) << 13;

	// the translation looked up lands in the FTIR and its state in the TIR; the
	// top three bits of the physical page travel in the TAGADD field
	m_ftir = e.pte << 3;
	m_tir = ((e.pte >> 29) & 0x07) << 20;                                 // TAGADD
	if (((e.tag >> 20) & 0x0f) == ((va >> 20) & 0x0f)) m_tir |= 0x0010;   // FM0
	if (((e.tag >> 24) & 0x0f) == ((va >> 24) & 0x0f)) m_tir |= 0x0020;   // FM1
	if (((e.tag >> 28) & 0x0f) == ((va >> 28) & 0x0f)) m_tir |= 0x40000;  // FM3
	// an I/O page never stays in the FTLB
	if (e.valid && BIT(e.pte, 0) && !BIT(e.pte, 5)) m_tir |= 0x0040;      // FM2
	if (BIT(e.pte, 3)) m_tir |= 0x0080;                                   // FWP
	if (BIT(e.pte, 4)) m_tir |= 0x0100;                                   // FUP

	if (m_corrupt[cache_index(va)]) m_tir |= 0x00020000;                  // corrupt block
	if (t.own) m_tir |= 0x00000001;                                       // POWN
	if (t.dirty) m_tir |= 0x00010000;                                     // dirty block
	if (((t.phys >> 17) & 0x0f) == ((p >> 17) & 0x0f)) m_tir |= 0x02;     // CM0
	if (((t.phys >> 21) & 0x0f) == ((p >> 21) & 0x0f)) m_tir |= 0x04;     // CM1
	if (((t.phys >> 25) & 0x0f) == ((p >> 25) & 0x0f)) m_tir |= 0x08;                   // CM2 (28:25)
	if (t.valid && ((t.phys >> 29) & 0x07) == ((p >> 29) & 0x07)) m_tir |= 0x80000;     // CM3
}


//**************************************************************************
//  address translation
//**************************************************************************

void solbourne_state::fault(uint32_t cause, uint32_t va, uint32_t mem_mask)
{
	// one instruction can fault twice (a doubleword is two accesses): the
	// registers keep the first and do not rearm until the software reads the
	// cause register
	if (!m_fault_armed)
		return;
	m_fault_armed = false;

	// MAME's CPU aligns the address on byte and halfword accesses: the exact
	// byte comes back from the mask, which is what the fault address register
	// must record
	m_fcr = cause;
	m_fvar = va | low_bits(mem_mask);
	LOGMASKED(LOG_MMU, "%s: fault cause=%02x fvar=%08x\n", machine().describe_context(), cause, m_fvar);
}

bool solbourne_state::translate(uint8_t asi, uint32_t va, uint32_t mem_mask, bool write, int &space, uint32_t &phys)
{
	const bool insn_asi = (asi == 0x08 || asi == 0x09);

	if ((va & 0xfe000000) == 0xfe000000)
	{
		// Both translation windows work with the MMU off or on, but translate
		// through the fe/ff half of the GTLB (mmu.h); until an entry is loaded
		// the fixed split applies: 0xfe... to the PROM and 0xff... to physical
		// memory. The windows are supervisor only: any user access faults on
		// protection without looking at the entry.
		if (asi == 0x08 || asi == 0x0a)
		{
			fault(FCR_UPF, va, mem_mask);
			return false;
		}

		// the window fixes the space, the GTLB entry supplies the address
		space = (va & 0xff000000) == 0xfe000000 ? 0 : -1;

		const tlb_entry &w = (insn_asi ? m_witlb : m_wdtlb)[tlb_index(va)];
		if (w.valid)
		{
			phys = ((w.pte >> 13) << 13) | (va & 0x1fff);
		}
		else
		{
			phys = va & 0x00ffffff;
		}
		return true;
	}

	if (!(m_mmcr & MMCR_ME))
	{
		// with the MMU off the address is a KBus physical address: the top
		// four bits are the I/O space, so the reset vector at 0 lands in
		// space 0, the boot PROM
		space = int(va >> 28);
		phys = va & 0x0fffffff;
		return true;
	}

	const bool insn = (asi == 0x08 || asi == 0x09);
	const tlb_entry *tlb = insn ? m_itlb : m_dtlb;
	const tlb_entry &e = tlb[tlb_index(va)];
	if (!e.valid || tlb_tag(e.tag) != tlb_tag(va))
	{
		fault(FCR_TMISS, va, mem_mask);
		LOGMASKED(LOG_MMU, "%s: TLB miss %c va=%08x fvar=%08x\n",
			machine().describe_context(), insn ? 'I' : 'D', va, m_fvar);
		return false;
	}

	if (!protection_ok(e.pte, asi, write, va, mem_mask))
		return false;

	// pte.h: 19-bit physical page at the top, I/O bit at 5 and four space bits
	// at 9 to 12 (PG_SPACE). On the Series 5 the page already carries the space
	// in its top four bits, which is what pte_io_ok wants, and the PG_SPACE bits
	// add to them: the boot code uses them to ask for space 1 with a zero page.
	const uint32_t pte = e.pte;
	phys = ((pte >> 13) << 13) | (va & 0x1fff);
	const uint32_t sp = (pte >> 9) & 0x0f;
	space = BIT(pte, 5) ? int(sp ? sp : (phys >> 28)) : -1;
	if (BIT(pte, 5))
		m_last_iopte = pte;
	return true;
}


//**************************************************************************
//  physical bus
//**************************************************************************

// On a write the check byte is generated from the data left in the whole
// doubleword.
void solbourne_state::ecc_store(uint32_t addr)
{
	const uint32_t dw = addr & ~uint32_t(7);
	if (dw + 7 < m_ram_size)
		m_eccb[dw >> 3] = ecc_check(ram_word(dw), ram_word(dw + 4));
}

// On a read with ECC on, the stored check byte, or the KCB register when that
// one rules, is compared with the one the data calls for. A syndrome matching
// the column of a data bit is a single-bit error: it is corrected on the fly
// and reported through the level 14 interrupt, leaving the physical address
// and the syndrome in their registers. If the syndrome is a power of two the
// error is in the check byte itself and the data comes out intact; any other
// syndrome is multi-bit, and that is a data exception.
uint32_t solbourne_state::ecc_load(uint32_t addr, uint32_t value)
{
	// the checker sees the whole cache block, not the word asked for: all four
	// doublewords go through it, and the fault reported is that of the first one
	// that does not check out, even if the CPU was reading another
	const uint32_t block = addr & ~uint32_t(31);
	if (block + 31 >= m_ram_size)
		return value;

	for (int line = 0; line < 4; line++)
	{
		const uint32_t dw = block + line * 8;
		const uint8_t stored = (m_mmcr & MMCR_KCB) ? uint8_t(m_kcb) : m_eccb[dw >> 3];
		const uint8_t syn = stored ^ ecc_check(ram_word(dw), ram_word(dw + 4));
		if (!syn)
			continue;

		int bit = -1;
		for (int i = 0; i < 64; i++)
			if (ECC_COL[i] == syn) { bit = i; break; }
		const bool single = (bit >= 0) || ((syn & (syn - 1)) == 0);

		// bits 0 to 31 are the low word, which lives at the higher address; the
		// correction only shows in the word being read
		if (bit >= 0 && (addr & ~uint32_t(7)) == dw
			&& ((bit >= 32) == ((addr & 4) == 0)))
			value ^= uint32_t(1) << (bit & 31);

		if (m_ecc_armed)
		{
			m_ecc_armed = false;
			m_fpar = dw;
			m_fes = syn;
			LOGMASKED(LOG_IO, "%s: ECC %s at %08x syndrome %02x bit %d\n",
				machine().describe_context(), single ? "single-bit" : "multi-bit",
				dw, syn, bit);
		}
		if (single)
		{
			m_maincpu->set_input_line(SPARC_IRQ14, ASSERT_LINE);
		}
		else
		{
			// a multi-bit error cannot be corrected: it is a data exception, with
			// the logical address in the FVAR and the physical address of the
			// doubleword in the transaction register
			if (m_fault_armed)
			{
				m_fault_armed = false;
				m_fcr = FCR_ECCM;
				m_fvar = m_cur_va | low_bits(m_cur_mask);
				m_ftor = dw;
			}
			m_maincpu->set_mae();
			break;
		}
	}
	return value;
}

void solbourne_state::timeout(int space, uint32_t addr)
{
	// with fast I/O the transaction does not stall the CPU: the failure arrives
	// later as a level 8 interrupt and is flagged in the MMCR
	const bool fast = (m_mmcr & MMCR_FIO) != 0;
	if (fast)
	{
		// the pending flag is set right away, but the report takes as long as
		// the bus takes to give up
		m_mmcr |= MMCR_PIO;
		m_fio->adjust(attotime::from_usec(5));
		LOGMASKED(LOG_IO, "fast I/O: pending at %08x\n", addr);
	}
	else
	{
		m_maincpu->set_mae();
	}
	// the KBus transaction type follows from the size and direction of the
	// access (kbus.h): 8 and 9 for byte, 10 and 11 for halfword, 12 and 13 for
	// word; the odd ones are reads
	uint32_t size = 2;
	if (m_cur_mask == 0xff000000 || m_cur_mask == 0x00ff0000
		|| m_cur_mask == 0x0000ff00 || m_cur_mask == 0x000000ff)
		size = 0;
	else if (m_cur_mask == 0xffff0000 || m_cur_mask == 0x0000ffff)
		size = 1;
	else if (m_maincpu->dword_access())
		size = 3;                                // ldd and std are eight bytes
	// the doubleword comes out as 0x0e in both directions, which is what test
	// 19 checks
	uint32_t ttype = (size == 3) ? 0x0e : (8 + size * 2 + (m_cur_write ? 0 : 1));
	// a memory transaction is not an I/O one: a write goes out as read and
	// invalidate and a read as a cacheable read
	if (space < 0)
		ttype = m_cur_write ? 3 : 7;
	if (m_force_ttype >= 0)
	{
		ttype = uint32_t(m_force_ttype);
		m_force_ttype = -1;
	}
	m_ett = ett_from_ttype(ttype);

	// a cache transaction that never completes leaves the block marked corrupt:
	// nobody knows what is inside it
	if (space < 0)
	{
		m_corrupt[cache_index(m_cur_va)] = true;
	}

	// the fault registers keep the instruction's first fault
	if (!m_fault_armed)
		return;
	m_fault_armed = false;

	if (!fast)
	{
		m_fcr = FCR_TOF;
		m_fvar = m_cur_va | low_bits(m_cur_mask);
	}
	// the low byte of the 16-bit register carries the type and the space; the
	// high one is the physical diagnostic register, assembled when read
	m_ftsr = (ttype << 4) | ((space < 0) ? 0 : (uint32_t(space) & 0x0f));
	// the space goes in the FTSR, so only the KBus address belongs here
	m_ftor = addr | low_bits(m_cur_mask);
	LOGMASKED(LOG_IO, "bus timeout space=%x addr=%08x va=%08x ftor=%08x ftsr=%04x pte=%08x pc=%08x\n",
		space, addr, m_fvar, m_ftor, m_ftsr, m_last_iopte, m_maincpu->pc());
}

uint32_t solbourne_state::phys_r(int space, uint32_t addr, uint32_t mem_mask)
{
	if (!responds(space, addr))
	{
		if (!machine().side_effects_disabled())
			timeout(space, addr);
		return 0;
	}

	// the space is already separate: only 28 bits reach the decoder
	if (space >= 0)
		addr &= 0x0fffffff;

	if (space == 0)
		return m_prom_ptr[(addr & (m_prom_size - 1)) >> 2];

	if (space < 0)
	{
		addr &= m_ram_size - 1;                  // the board base already matches
		if (addr + 3 < m_ram_size)
		{
			const uint32_t v = ram_word(addr);
			if ((m_mmcr & MMCR_EE) && !machine().side_effects_disabled())
				return ecc_load(addr, v);
			return v;
		}
		if (!machine().side_effects_disabled())
			LOGMASKED(LOG_IO, "%s: read past end of RAM %08x\n",
				machine().describe_context(), addr);
		return 0;
	}

	if (space == vram_space() && addr + 3 < VRAM_SIZE)
	{
		const uint8_t *p = &m_vram[addr];
		return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16)
			 | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
	}

	if (space == int(OBIO_SPACE) && (addr >> 24) == 0)
	{
		const uint32_t off = addr & 0xffffff;
		if (off == ASIC_IR)
			return m_asic_ir;
		if (off == ASIC_DMA_PAGE)
			return m_dma_page;
		if (off == ASIC_DMA_OFF)
			return m_dma_off;
		if (off == ASIC_DMA_NEXT)
			return m_dma_next_page;
		if (off == SCSI_ADDR)
		{
			// MAME's indir_addr_r() hides the interrupt bit while the buffer
			// ready bit is set, a workaround for another machine; here the
			// driver polls exactly that bit, so the auxiliary status register
			// has to be read as it stands
			const uint8_t v = m_scsi->status_r();
			return uint32_t(v) << 24;
		}
		if (off == SCSI_DATA)
		{
			const uint8_t v = m_scsi->indir_reg_r();
			return uint32_t(v) << 24;
		}
		if (!machine().side_effects_disabled())
			LOGMASKED(LOG_IO, "%s: obio read %04x (mask %08x)\n",
				machine().describe_context(), off, mem_mask);
		return 0;
	}

	if (space == 1)     // ID space: slot in bits 27..24
	{
		const int slot = (addr >> 24) & 0x0f;
		const uint32_t off = addr & 0x00ffffff;


		// each board's ID PROM sits at the start of its slot, one useful byte
		// every eight addresses; on the system board it only takes the first
		// half, because the second one is the EEPROM
		if (off < (slot == int(SYS_SLOT) ? 0x2000 : 0x4000))
		{
			const uint32_t i = (off >> 3) & 0x7ff;   // 2048 useful bytes
			if (!machine().side_effects_disabled() && i == 0)
				LOGMASKED(LOG_IO, "probing slot %d\n", slot);
			if (slot == SYS_SLOT)
				return uint32_t(m_sysid[i]) << 24;
			if (slot == MEM_SLOT)
				return uint32_t(m_memid[i]) << 24;
			if (slot == BID_VALUE)
				return uint32_t(m_cpuid[i]) << 24;
		}

		// each board also has configuration registers right after its ID PROM,
		// which the boot code uses while probing
		if (off >= 0x4000 && off < 0x5000)
			return uint32_t(m_slotcfg[slot][((off - 0x4000) >> 3) & 0x1f]) << 24;

		// memory board registers: size in megabytes, base address (physical
		// bits 31 to 24) and read and write enable
		if (slot == MEM_SLOT && off >= MEM_SIZE_REG)
		{
			switch (off & 0x00ff0000)
			{
			case MEM_SIZE_REG:   return uint32_t(m_ram_size >> 20) << 24;
			case MEM_BASE_REG:   return uint32_t(m_mem_base) << 24;
			case MEM_ENABLE_REG: return uint32_t(m_mem_enable) << 24;
			}
		}

		if (slot == SYS_SLOT)                                 // system board obio
		{
			if (off >= 0x2000 && off < 0x4000)                // EEPROM, one byte per 8
				return uint32_t(m_eeprom[((off - 0x2000) >> 3) & 0x3ff]) << 24;
			if ((off & 0xff0000) == RTC_REG)                  // real time clock
			{
				return uint32_t(m_rtc_data & 0x0f) << 24;
			}
			// reading the register turns interrupt transmission off and reading
			// the next one turns it on (sysboard.h)
			if (off == SBIR_REG)
			{
				if (!machine().side_effects_disabled())
				{
					m_sbi_enabled = false;
					LOGMASKED(LOG_CTRL, "sbir read: transmit disabled\n");
				}
				return uint32_t(m_sbir) << 24;
			}
			if (off == VIDMAP_REG)
				return uint32_t(m_vidmap) << 24;
			if (off >= SCC_KM_BASE && off < SCC_KM_BASE + SCC_SIZE)
				return uint32_t(scc_r(*m_scc_km, off - SCC_KM_BASE)) << 24;
			if (off >= SCC_TTY_BASE && off < SCC_TTY_BASE + SCC_SIZE)
				return uint32_t(scc_r(*m_scc_tty, off - SCC_TTY_BASE)) << 24;
			if (off == SBIEN_REG)
			{
				if (!machine().side_effects_disabled())
				{
					m_sbi_enabled = true;
					LOGMASKED(LOG_CTRL, "sbien read: transmit enabled\n");
					sbi_next();
					io_poll();
				}
				return 0;
			}
		}
	}

	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_IO, "%s: I/O read space %x addr %08x (mask %08x)\n",
			machine().describe_context(), space, addr, mem_mask);
	return 0;
}

void solbourne_state::phys_w(int space, uint32_t addr, uint32_t data, uint32_t mem_mask)
{
	if (!responds(space, addr))
	{
		timeout(space, addr);
		return;
	}

	if (space >= 0)
		addr &= 0x0fffffff;

	if (space < 0)
	{
		addr &= m_ram_size - 1;
		if (addr + 3 < m_ram_size)
		{
			uint8_t *p = m_ram_ptr + addr;
			uint32_t cur = (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
			cur = (cur & ~mem_mask) | (data & mem_mask);
			p[0] = cur >> 24; p[1] = cur >> 16; p[2] = cur >> 8; p[3] = cur;
			ecc_store(addr);
			return;
		}
		LOGMASKED(LOG_IO, "%s: write past end of RAM %08x = %08x\n",
			machine().describe_context(), addr, data);
		return;
	}

	if (space == 0)
	{
		LOGMASKED(LOG_IO, "%s: write to PROM space %08x = %08x\n",
			machine().describe_context(), addr, data);
		return;
	}

	if (space == vram_space() && addr + 3 < VRAM_SIZE)
	{
		uint8_t *p = &m_vram[addr];
		uint32_t cur = (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16)
					 | (uint32_t(p[2]) << 8) | uint32_t(p[3]);
		cur = (cur & ~mem_mask) | (data & mem_mask);
		p[0] = cur >> 24; p[1] = cur >> 16; p[2] = cur >> 8; p[3] = cur;
		return;
	}

	if (space == int(OBIO_SPACE) && (addr >> 24) == 0)
	{
		const uint32_t off = addr & 0xffffff;
		if (off == ASIC_IR)
		{
			{
				const bool was_stopped = (m_asic_ir & ASIC_IR_PAGE) != 0;
				m_asic_ir = 0;                   // writing it acknowledges the interrupt
				io_intr(IOINT_ASIC, 0);
				if (was_stopped && m_drq)
					dma_step();
			}
			return;
		}
		if (off == ASIC_DMA_PAGE)
		{
			// reprogramming the address makes the ASIC release the block it had
			// half filled: that is how the driver learns its header is already in
			// memory
			dma_flush();
			m_dma_page = data;
			dma_step();                          // it can go on now
			return;
		}
		if (off == ASIC_DMA_NEXT)
		{
			m_dma_next_page = data;
			m_next_page_set = true;
			LOGMASKED(LOG_IO, "asic: next page = %08x\n", data);
			return;
		}
		if (off == ASIC_DMA_OFF)
		{
			// reprogramming it acknowledges the overflow
			m_dma_off = data;
			m_asic_ir &= ~ASIC_IR_PAGE;
			io_intr(IOINT_ASIC, 0);
			return;   // the page is written next; the transfer resumes with it
		}
		if (off == SCSI_ADDR)
		{
			m_wd_reg = data >> 24;
			m_wd_ptr = m_wd_reg;
			m_scsi->indir_addr_w(data >> 24);
			return;
		}
		if (off == SCSI_DATA)
		{
			// the twelve command registers are filled one after another, the WD
			// advances the pointer by itself
			if (m_wd_ptr >= 0x03 && m_wd_ptr <= 0x0e)
				m_cdb[m_wd_ptr - 3] = data >> 24;
			if (m_wd_ptr < 0x18)
				m_wd_ptr++;
			// writing the command register starts a fresh bus transaction: the
			// next DMA request is the one that pays for the delay
			if (m_wd_reg == 0x18)
			{
				m_dma_settle = true;
			}
			if (m_wd_reg == 0x15)
				m_scsi_target = (data >> 24) & 7;
			m_scsi->indir_reg_w(data >> 24);
			return;
		}
		LOGMASKED(LOG_IO, "%s: obio write %04x = %08x (mask %08x)\n",
			machine().describe_context(), off, data, mem_mask);
		return;
	}

	if (space == 1)
	{
		const uint32_t slot = (addr >> 24) & 0x0f;
		const uint32_t off2 = addr & 0x00ffffff;
		if (off2 >= 0x4000 && off2 < 0x5000)
		{
			m_slotcfg[slot][((off2 - 0x4000) >> 3) & 0x1f] = data >> 24;
			return;
		}
		if (slot == MEM_SLOT && off2 >= MEM_SIZE_REG)
		{
			switch (off2 & 0x00ff0000)
			{
			case MEM_BASE_REG:
				m_mem_base = data >> 24;
				LOGMASKED(LOG_IO, "%s: memory board base = %02x000000\n",
					machine().describe_context(), m_mem_base);
				return;
			case MEM_ENABLE_REG:
				m_mem_enable = data >> 24;
				LOGMASKED(LOG_IO, "%s: memory board enable = %02x\n",
					machine().describe_context(), m_mem_enable);
				return;
			case MEM_SIZE_REG:
				return;
			}
		}
	}

	if (space == 1 && ((addr >> 24) & 0x0f) == SYS_SLOT)
	{
		const uint32_t off = addr & 0x00ffffff;
		if (off >= 0x2000 && off < 0x4000)
		{
			m_eeprom[((off - 0x2000) >> 3) & 0x3ff] = data >> 24;
			return;
		}
		if (off == SBIR_REG)
		{
			m_sbir = data >> 24;
			// Once the reset interrupt queue is drained, programming the mode
			// leaves the board ready to transmit: that is how the kernel turns
			// its own on, because after this write it never reads the enable
			// register again. While the queue is still full the read-to-disable
			// and read-to-enable pair rules, which is what the POST uses.
			if (m_sbi_left <= 0)
				m_sbi_enabled = true;
			io_poll();
			return;
		}
		if ((off & 0xff0000) == RTC_REG)
		{
			rtc_w(data >> 24);
			return;
		}
		if (off >= SCC_KM_BASE && off < SCC_KM_BASE + SCC_SIZE)
		{
			scc_w(*m_scc_km, off - SCC_KM_BASE, data >> 24);
			return;
		}
		if (off >= SCC_TTY_BASE && off < SCC_TTY_BASE + SCC_SIZE)
		{
			scc_w(*m_scc_tty, off - SCC_TTY_BASE, data >> 24);
			return;
		}
		// the first address of the board: with the top bit set, a reset
		if (off < 0x10 && BIT(data >> 24, 7))
		{
			m_sysreset->adjust(attotime::zero);
			return;
		}
		if (off == VIDMAP_REG)
		{
			m_vidmap = data >> 24;
			LOGMASKED(LOG_IO, "%s: frame buffer in space %x\n",
				machine().describe_context(), (m_vidmap >> 1) & 0x0f);
			return;
		}
	}

	LOGMASKED(LOG_IO, "%s: I/O write space %x addr %08x = %08x (mask %08x)\n",
		machine().describe_context(), space, addr, data, mem_mask);
}


// Each channel takes thirty-two addresses: the control register at the start
// and the data one sixteen further on; channel B comes first.
uint8_t solbourne_state::scc_r(scc8530_device &scc, uint32_t off)
{
	switch ((off >> 4) & 3)
	{
	case 0: return scc.cb_r(0);
	case 1: return scc.db_r(0);
	case 2: return scc.ca_r(0);
	default:
		return scc.da_r(0);
	}
}

void solbourne_state::scc_w(scc8530_device &scc, uint32_t off, uint8_t data)
{
	switch ((off >> 4) & 3)
	{
	case 0: scc.cb_w(0, data); return;
	case 1:
		LOGMASKED(LOG_CTRL, "t=%.6f serial B: %02x %c\n", machine().time().as_double(), data,
			(data >= 32 && data < 127) ? data : '.');
		scc.db_w(0, data);
		return;
	case 2: scc.ca_w(0, data); return;
	default:
		LOGMASKED(LOG_CTRL, "t=%.6f serial A: %02x %c\n", machine().time().as_double(), data,
			(data >= 32 && data < 127) ? data : '.');
		if (m_console_copy && &scc == m_scc_tty.target())
		{
			fputc(char(data), m_console_copy);
			fflush(m_console_copy);
		}
		scc.da_w(0, data);
		return;
	}
}

// The clock is driven with a single byte: the low four bits are the data and
// the high four the address load, read, write and stop lines (clock.h). The
// chip is always selected at that address, and what it drives on its pins is
// collected in m_rtc_data.
void solbourne_state::rtc_w(uint8_t data)
{
	m_rtc->cs1_w(1);
	m_rtc->cs2_w(1);
	m_rtc->d0_w(BIT(data, 0));
	m_rtc->d1_w(BIT(data, 1));
	m_rtc->d2_w(BIT(data, 2));
	m_rtc->d3_w(BIT(data, 3));
	m_rtc->address_write_w(BIT(data, 4));
	m_rtc->read_w(BIT(data, 5));
	m_rtc->write_w(BIT(data, 6));
	m_rtc->stop_w(BIT(data, 7));
}

// The ASIC moves one byte each time the controller asks and advances the
// offset, which is what the boot code reads to see how far it has got.
void solbourne_state::scsi_drq(int state)
{
	m_drq = state != 0;
	if (!m_drq)
		return;
	if (m_dma_settle)
	{
		// first request of a fresh command: the drive needs the time it takes
		// to answer
		m_dma_settle = false;
		m_dma_timer->adjust(attotime::from_usec(DMA_SETTLE_US));
		return;
	}
	dma_step();
}

TIMER_CALLBACK_MEMBER(solbourne_state::dma_tick)
{
	dma_step();
}

// The bus watcher on the CPU board snoops KBus writes and throws out of the
// cache the block somebody else has just changed. Without this the CPU keeps
// reading what it had cached and never sees what the DMA just left in memory.
// The cache is virtually indexed, but index bits 12:5 are the same in the
// virtual and the physical address, so a physical block can only be in
// sixteen places.
void solbourne_state::bus_watch(uint32_t phys)
{
	const uint32_t block = phys & ~uint32_t(31);
	if (block == m_last_block)
		return;
	m_last_block = block;

	// the tag holds the physical page of the block, not the whole block: the
	// rest of the address comes from the index itself
	const uint32_t page = block & ~uint32_t(0x1fff);
	const uint32_t low = (block >> 5) & 0xff;
	for (int high = 0; high < 16; high++)
	{
		cache_tag &t = m_ctag[(high << 8) | low];
		if (t.valid && t.phys == page)
		{
			t.valid = false;
			t.own = false;
			t.dirty = false;
		}
	}
}

// The ASIC does not write byte by byte: it gathers a 32-byte cache block in a
// buffer of its own and flushes the whole thing. That is why the OS/MP driver
// rounds transfers up to 32, uses a bounce buffer for the header and detects
// the end by putting a sentinel at the start of the block and then asking for
// a single byte at the end: flushing the block wipes the sentinel out.
void solbourne_state::dma_flush()
{
	if (m_dma_block == 0xffffffff)
		return;
	for (int i = 0; i < 32; i++)
	{
		const uint32_t a = m_dma_block + i;
		// only the bytes that arrived go out: what never came is left alone
		if (BIT(m_dma_valid, i) && a < m_ram_size)
		{
			bus_watch(a);
			m_ram_ptr[a] = m_dma_buf[i];
		}
	}
	// the check byte covers the whole doubleword: once per doubleword is enough
	for (int dw = 0; dw < 32; dw += 8)
		if ((m_dma_valid >> dw) & 0xff)
		{
			const uint32_t a = m_dma_block + dw;
			if (a < m_ram_size)
				ecc_store(a);
		}
}

void solbourne_state::dma_step()
{
	// The controller asks for data again from inside dma_r and dma_w, so this
	// calls itself. Without the lock the inner call advances the offset,
	// returns, and the outer one advances it again: a byte gets skipped. The
	// outer loop already checks m_drq after every byte, so nothing is lost by
	// not entering.
	if (m_in_dma)
		return;
	m_in_dma = true;
	bool moved = false;
	while (m_drq && !(m_asic_ir & ASIC_IR_PAGE))
	{
		const uint32_t a = dma_addr();
		if (dma_to_disk())
		{
			m_scsi->dma_w(a < m_ram_size ? m_ram_ptr[a] : 0xff);
		}
		else
		{
			const uint8_t d = m_scsi->dma_r();
			if ((a & ~uint32_t(31)) != m_dma_block)
			{
				dma_flush();
				m_dma_block = a & ~uint32_t(31);
				m_dma_valid = 0;
			}
			m_dma_buf[a & 31] = d;
			m_dma_valid |= 1u << (a & 31);
			moved = true;
		}

		if ((m_dma_off & 0x7ffc) == 0x7ffc)
		{
			// with the next page already loaded the transfer carries on into it
			// without a word: that is what the preload register is for
			if (m_next_page_set)
			{
				m_next_page_set = false;
				m_dma_page = m_dma_next_page;
				m_dma_off &= ~uint32_t(0x7ffc);
			}
			else
			{
				// the page register is just the high address bits: when the page
				// runs out the counter carries on into the next one
				m_dma_page++;
				m_dma_off &= ~uint32_t(0x7ffc);
			}
		}
		else
			m_dma_off = (m_dma_off & ~uint32_t(0x7ffc)) | ((m_dma_off + 4) & 0x7ffc);
	}
	if (moved)
		dma_flush();
	m_in_dma = false;
}

// A directed interrupt only lands if it is meant for this board and its
// vector is above the receiver's level.
bool solbourne_state::send_vector(uint32_t dest, uint32_t vector)
{
	// a directed one is only taken by its addressee
	if (dest != (0x40u | (BID_VALUE & 0x0f)))
		return false;
	return recv_vector(dest, vector);
}

bool solbourne_state::recv_vector(uint32_t dest, uint32_t vector)
{
	// the receiver has to be on and idle: while the P bit stands no other one
	// gets in, and the sender does not count it as delivered
	if (vector <= m_ipr)
	{
		return false;
	}
	if (!(m_irxc & 0x01) || (m_irxc & 0x02))
	{
		return false;
	}
	m_ipv = (vector << 8) | dest;
	m_irxc |= 0x02;                              // P: interrupt pending
	m_maincpu->set_input_line(SPARC_IRQ12, ASSERT_LINE);
	LOGMASKED(LOG_CTRL, "%s: directed interrupt, vector %02x\n",
		machine().describe_context(), vector);
	return true;
}

// The system board leaves sixteen interrupts queued on every reset and lets
// them go, highest priority first, the first time it is allowed to transmit;
// each one waits for the previous to be acknowledged. Every line goes into a
// flip-flop; the PAL scans from high to low, takes the highest and clears its
// flip-flop. While the CPU receiver holds an unacknowledged interrupt, or its
// priority is higher, the line waits.
void solbourne_state::io_intr(uint32_t line, int state)
{
	if (state)
		m_io_pend |= 1 << line;
	else
		m_io_pend &= ~(1 << line);
	io_poll();
}

// Delivery is not instantaneous: the system board has to win the bus and send
// the transaction, and that is microseconds. Delivering it inside the very
// write that caused it makes the kernel take the trap halfway through a
// routine.
void solbourne_state::io_poll()
{
	io_deliver(0);
}

TIMER_CALLBACK_MEMBER(solbourne_state::io_deliver)
{
	if (!m_sbi_enabled || !m_io_pend)
		return;
	for (int i = 15; i >= 0; i--)
	{
		if (!BIT(m_io_pend, i))
			continue;
		const uint32_t vector = IOINT_BASE + i;
		const uint32_t dest = (m_sbir & SBIR_DI)
			? (0x40u | (m_sbir & SBIR_DDID)) : uint32_t(m_sbir & SBIR_INFO);
		const bool ida = recv_vector(dest, vector);
		if (ida)
		{
			m_io_pend &= ~(1 << i);
			if (m_io_pend)
				m_sbi_timer->adjust(attotime::from_usec(2));
		}
		return;
	}
}

void solbourne_state::sbi_next()
{
	if (!m_sbi_enabled || m_sbi_left <= 0 || !(m_sbir & SBIR_DI))
		return;
	if (send_vector(0x40u | (m_sbir & SBIR_DDID), 0x80 + m_sbi_left - 1))
		m_sbi_left--;
}

//**************************************************************************
//  per-ASI dispatch
//**************************************************************************

template <uint8_t Asi>
uint32_t solbourne_state::asi_r(offs_t offset, uint32_t mem_mask)
{
	const uint32_t va = offset << 2;

	if (is_cache_asi(Asi))
	{
		if (!machine().side_effects_disabled() && BIT(Asi, 5))
			cache_probe(!BIT(Asi, 1), va);
		return cache_data_r(va, mem_mask);
	}

	if (Asi >= 0x08 && Asi <= 0x0b)
	{
		// the kernel is loaded at this address: until it jumps there the
		// power-on test wants no system board interrupts
		if (Asi == 0x09 && va == 0xff060000 && !machine().side_effects_disabled())
			m_kernel_started = true;
		if (!machine().side_effects_disabled())
		{
			m_cur_va = va;
			m_cur_mask = mem_mask;
			m_cur_write = false;
		}

		int space;
		uint32_t phys;
		if (!translate(Asi, va, mem_mask, false, space, phys))
		{
			if (!machine().side_effects_disabled())
			{
				m_maincpu->set_mae();
			}
			return 0;
		}
		// a block marked corrupt is never touched again: the cache transaction
		// ends in a timeout
		if (space < 0 && m_corrupt[cache_index(va)])
		{
			if (!machine().side_effects_disabled())
			{
				// the bus tries to pull the block out with an IOB transaction,
				// which is the one left unanswered; the physical diagnostic
				// register still reports the access the CPU asked for
				m_force_ttype = 1;
				timeout(space, phys);
				bus_ett(space, false, mem_mask);
			}
			return 0;
		}
		// only data accesses leave their type in the physical diagnostic
		// register: instruction fetches are served by the cache
		if ((Asi == 0x0a || Asi == 0x0b) && !machine().side_effects_disabled())
			bus_ett(space, false, mem_mask);
		return phys_r(space, phys, mem_mask);
	}

	switch (Asi)
	{
	case 0x80:
		if (!machine().side_effects_disabled())
			wdog_restart();                      // reading it zeroes the counter
		return reg_out(m_mmcr, mem_mask);
	case 0x81:
		if (!machine().side_effects_disabled())
			m_fault_armed = true;                // reading it rearms the latch
		return reg_out(m_fcr, mem_mask);
	case 0x82: return reg_out(m_fvar, mem_mask);
	case 0x83: return reg_out(m_pdba, mem_mask);
	// The board id register carries the slot in the low four bits and above it
	// the state of the diagnostic switch, the KBus NMI and SYSFAIL signals and
	// the burn-in jumper, all high when nothing is pulling them down
	// (cpuboard.h). With the jumper out the boot code does not sit in the
	// burn-in loop.
	case 0xc0: return reg_out(BID_VALUE | 0x0080, mem_mask);  // board id
	case 0xc8: return reg_out(m_dir, mem_mask);             // device id
	case 0xc9: return reg_out(m_ixr, mem_mask);             // interrupt transmit
	case 0xca: return reg_out(m_itxc, mem_mask);            // transmit control
	case 0xcb: return reg_out(m_ipr, mem_mask);             // interrupt priority
	case 0xec: return reg_out(m_ipv, mem_mask);             // pending vector, no ack
	case 0xcc:                                              // pending vector, with ack
	{
		// the acknowledge drops the request to the CPU, but the IRXC P bit
		// stands until the software writes the register
		if (!machine().side_effects_disabled())
			m_maincpu->set_input_line(SPARC_IRQ12, CLEAR_LINE);
		return reg_out(m_ipv, mem_mask);
	}
	case 0xcd: return reg_out(m_irxc, mem_mask);            // receiver control
	case 0xcf: return 0xffffffff;                // CRDAT: bus idle
	case 0xd3:                                   // address of the timeout fault
		// reading it rearms fast I/O and the fault register latch, and drops
		// the interrupt
		if (!machine().side_effects_disabled())
		{
			m_fault_armed = true;
			m_mmcr &= ~MMCR_PIO;
			m_maincpu->set_input_line(SPARC_IRQ8, CLEAR_LINE);
		}
		return reg_out(m_ftor, mem_mask);
	case 0xf4:                                   // KBus diagnostic transaction
	{
		// The low three address bits pick what the transaction does: 7 loads
		// the tag and 0 reads it back, and MAME's CPU aligns them away on byte
		// accesses, so the exact address matters. An access that is not a
		// single byte is not a valid diagnostic transaction: the watcher puts
		// a write and invalidate on the bus, nobody completes it, and the
		// access ends in a data exception.
		if (mem_mask != 0xff000000 && mem_mask != 0x00ff0000
			&& mem_mask != 0x0000ff00 && mem_mask != 0x000000ff)
		{
			if (!machine().side_effects_disabled())
			{
				m_cur_va = va;
				m_cur_mask = mem_mask;
				m_cur_write = true;
				m_force_ttype = 2;               // write and invalidate
				timeout(int(va >> 28), va);
			}
			return 0;
		}

		const uint32_t a = va | low_bits(mem_mask);
		const uint32_t index = (a >> 5) & (CACHE_BLOCKS - 1);
		watcher_tag &w = m_wtag[index];
		if (!machine().side_effects_disabled())
			m_ett = (7 - (a & 7)) << 4;          // the subtype gives the PDR type
		if (BIT(a, 2))
		{
			// subtypes 4 to 7 load the tag with address bits 28:17; the low two
			// bits say whether the block is owned and whether the entry ends up
			// valid (diag.h)
			w.tag = (a >> 17) & 0x7fff;
			w.own = BIT(a, 1);
			w.valid = BIT(a, 0);
		}
		else
		{
			// the state comes out of the physical diagnostic register: the first
			// comparator looks at the low eight tag bits, the second at the top
			// three and also demands a valid entry, and the third bit says the
			// block is not owned
			m_pdr = 0;
			if ((w.tag & 0xff) == ((a >> 17) & 0xff)) m_pdr |= 0x01;
			if (w.valid && ((w.tag >> 8) & 0x7f) == ((a >> 25) & 0x7f)) m_pdr |= 0x02;
			if (!w.own) m_pdr |= 0x04;
		}
		return 0;
	}
	case 0xd4:                                   // fault space and type
		// 16-bit register: read as a byte at address 0 it returns the high
		// half, which is the physical diagnostic register
		if (mem_mask == 0xff000000)
		{
			return (((m_pdr & 0x0f) | m_ett) & 0xff) << 24;
		}
		return reg_out((((m_pdr & 0x0f) | m_ett) << 8) | (m_ftsr & 0xff), mem_mask);
	case 0xd0: case 0xd1:                        // ECC address and syndrome
		// reading either one drops the interrupt and rearms the latch
		if (!machine().side_effects_disabled())
		{
			m_ecc_armed = true;
			m_maincpu->set_input_line(SPARC_IRQ14, CLEAR_LINE);
		}
		return reg_out(Asi == 0xd0 ? m_fpar : m_fes, mem_mask);
	case 0xc1: return reg_out(m_led, mem_mask);
	case 0xe0:                                   // diagnostic RAM
	{
		const uint32_t a = va & 0x1ffc;
		return (m_dgram[a] << 24) | (m_dgram[a + 1] << 16)
			 | (m_dgram[a + 2] << 8) | m_dgram[a + 3];
	}
	case 0xb5: return reg_out(m_flush[cache_index(va)], mem_mask);   // flush RAM
	case 0x84: return reg_out(m_tir, mem_mask);             // test information register
	case 0x85: return reg_out(m_ftir, mem_mask);            // FTLB test information register
	case 0xb1: case 0xb3:                        // read TLB entry
	{
		const tlb_entry &e = is_window(va)
			? (Asi == 0xb1 ? m_witlb : m_wdtlb)[tlb_index(va)]
			: (Asi == 0xb1 ? m_itlb : m_dtlb)[tlb_index(va)];
		// reading the entry is what loads the TIR; the bits come from the test
		// table of the PROM itself, checked against tir.h
		m_tir = 0;
		// GM0 and GM1 are the stored tag compared with the address used for
		// the access (tir.h: 27:24 and 31:28)
		if (((e.tag >> 24) & 0x0f) == ((va >> 24) & 0x0f)) m_tir |= 0x0200;
		if (((e.tag >> 28) & 0x0f) == ((va >> 28) & 0x0f)) m_tir |= 0x0400;
		if (e.valid) m_tir |= 0x0800;                         // GM2: entry valid
		if (BIT(e.pte, 0)) m_tir |= 0x8000;                   // GPV: page valid
		if (BIT(e.pte, 3)) m_tir |= 0x4000;                   // GWP: write protected
		if (BIT(e.pte, 4)) m_tir |= 0x2000;                   // GUP: user protected
		if (BIT(e.pte, 5)) m_tir |= 0x1000;                   // GIO: I/O page
		return e.pte;   // the whole entry reads back even when invalid
	}
	default:
		if (!machine().side_effects_disabled())
			LOGMASKED(LOG_CTRL, "%s: read ASI %02x addr %08x (mask %08x)\n",
				machine().describe_context(), Asi, va, mem_mask);
		return 0;
	}
}

template <uint8_t Asi>
void solbourne_state::asi_w(offs_t offset, uint32_t data, uint32_t mem_mask)
{
	const uint32_t va = offset << 2;

	if (is_cache_asi(Asi))
	{
		if (BIT(Asi, 5))
			cache_probe(!BIT(Asi, 1), va);
		cache_data_w(va, data, mem_mask, BIT(Asi, 3));
		return;
	}

	if (Asi >= 0x08 && Asi <= 0x0b)
	{
		m_cur_va = va;
		m_cur_mask = mem_mask;
		m_cur_write = true;

		int space;
		uint32_t phys;
		if (!translate(Asi, va, mem_mask, true, space, phys))
		{
			m_maincpu->set_mae();
			return;
		}
		if (space < 0 && m_corrupt[cache_index(va)])
		{
			m_force_ttype = 1;
			timeout(space, phys);
			bus_ett(space, true, mem_mask);
			return;
		}
		if (Asi == 0x0a || Asi == 0x0b)
			bus_ett(space, true, mem_mask);
		phys_w(space, phys, data, mem_mask);
		return;
	}

	switch (Asi)
	{
	case 0x80:
		LOGMASKED(LOG_MMU, "%s: MMCR = %08x\n", machine().describe_context(), data);
		m_mmcr = reg_in(data, mem_mask);
		if (m_mmcr & MMCR_WTAG)                  // clears the watcher tags
			for (auto &w : m_wtag) { w.own = false; w.valid = false; w.tag = 0; }
		wdog_restart();                          // enabling it starts the count
		return;
	case 0x81: m_fcr = reg_in(data, mem_mask); m_fault_armed = true; return;
	// interrupt registers, with the widths from cpuboard.h
	case 0xc8: m_dir = reg_in(data, mem_mask) & 0x000000ff; return;
	case 0xc9: m_ixr = reg_in(data, mem_mask) & 0x0000ffff; return;
	case 0xca:                                   // transmit control
	{
		const uint32_t v = reg_in(data, mem_mask) & 0x00000003;
		m_itxc = v;
		if (!BIT(v, 0))
			return;

		// bit 0 starts the transmission of whatever is in the IXR, and the
		// "gone" bit is only set if the receiver takes the interrupt: if its
		// priority level refuses it, the transmission does not complete
		m_itxc = send_vector(m_ixr & 0xff, (m_ixr >> 8) & 0xff) ? 0x02 : 0x00;
		return;
	}
	case 0xcb:
		m_ipr = reg_in(data, mem_mask) & 0x000000ff;
		io_poll();
		return;
	case 0xcc: m_ipv = reg_in(data, mem_mask) & 0x0000ffff; return;
	case 0xcd:
		// clearing the pending bit makes room for the next one the system board
		// has queued
		m_irxc = reg_in(data, mem_mask) & 0x00000003;
		if (!(m_irxc & 0x02))
		{
			sbi_next();
			io_poll();
		}
		return;
	case 0x83:
		// the register lives at address 0; the error handler's register dump
		// uses this same ASI with offsets
		if (va == 0)
		{
			LOGMASKED(LOG_MMU, "%s: PDBA = %08x\n", machine().describe_context(), data);
			m_pdba = data;
		}
		return;
	case 0xc1:
		// 16-bit register at address 0: it arrives in the high half
		m_led = (m_led & ~mem_mask) | (data & mem_mask);
		LOGMASKED(LOG_LED, "%s: LED = %04x\n", machine().describe_context(), m_led >> 16);
		return;
	case 0xe0:
	{
		const uint32_t a = va & 0x1ffc;
		uint32_t cur = (m_dgram[a] << 24) | (m_dgram[a + 1] << 16)
					 | (m_dgram[a + 2] << 8) | m_dgram[a + 3];
		cur = (cur & ~mem_mask) | (data & mem_mask);
		m_dgram[a] = cur >> 24; m_dgram[a + 1] = cur >> 16;
		m_dgram[a + 2] = cur >> 8; m_dgram[a + 3] = cur;
		return;
	}
	case 0x84: m_tir = reg_in(data, mem_mask); return;      // the test zeroes it before reading
	case 0xd2: m_kcb = reg_in(data, mem_mask) & 0x1ff; return;   // test check byte
	// corrupt block RAM: one bit per cache block, set by writing ASI 0xf9 and
	// cleared by writing 0xf8 (diag.h)
	case 0xf8: m_corrupt[cache_index(va)] = false; return;
	case 0xf9: m_corrupt[cache_index(va)] = true; return;
	case 0x94:                                   // clear the cache tags
		LOGMASKED(LOG_TLB, "%s: cache tags cleared\n", machine().describe_context());
		for (auto &t : m_ctag) { t.phys = 0; t.valid = false; t.own = false; }
		return;
	case 0x90: case 0x92: case 0x98: case 0x9a:  // write cache tag
	{
		// on the instruction side the block is only read; on the data side it
		// ends up owned, which is what the OWN bit tells apart
		cache_tag &t = m_ctag[cache_index(va)];
		t.phys = ftlb_phys(!BIT(Asi, 1), va);
		t.valid = true;
		t.own = BIT(Asi, 1);
		m_flush[cache_index(va)] = t.phys & 0xfffe0000;   // the 5E stores 31:17
		return;
	}
	case 0xa8: case 0xaa:                        // load TLB entry
	{
		tlb_entry &e = is_window(va)
			? (Asi == 0xa8 ? m_witlb : m_wdtlb)[tlb_index(va)]
			: (Asi == 0xa8 ? m_itlb : m_dtlb)[tlb_index(va)];
		// the same write loads the FTLB
		tlb_entry &f = (Asi == 0xa8 ? m_iftlb : m_dftlb)[ftlb_index(va)];
		f.tag = va;
		f.pte = data;
		f.valid = !BIT(data, 7);

		e.vpn = va >> 13;
		e.tag = va;
		e.pte = data;
		e.valid = !BIT(data, 7);                 // pte.h: pg_tlbinv invalidates the entry
		if (va >= 0xffff8000 && !machine().side_effects_disabled())
			LOGMASKED(LOG_CTRL, "uarea: %cTLB va=%08x pte=%08x pc=%08x\n",
				Asi == 0xa8 ? 0x49 : 0x44, va, data, m_maincpu->pc());
		LOGMASKED(LOG_TLB, "%s: %cTLB va=%08x pte=%08x\n",
			machine().describe_context(), Asi == 0xa8 ? 'I' : 'D', va, data);
		return;
	}
	case 0x85: case 0x86: case 0x87:             // invalidate
		// 0x85 clears the GTLB, 0x86 the FTLB and 0x87 both; the invalidate
		// wipes the tags as well, which is what cases 5 and 6 of test 0b check
		LOGMASKED(LOG_TLB, "%s: TLB invalidate (ASI %02x)\n", machine().describe_context(), Asi);
		if (Asi != 0x86)
		{
			for (auto &e : m_itlb) { e.valid = false; e.tag = 0; }
			for (auto &e : m_dtlb) { e.valid = false; e.tag = 0; }
			// the window area is not cleared: the kernel maps the process user
			// area there and counts on it surviving between the GTLB flush and
			// the context switch
			}
		if (Asi != 0x85)
		{
			for (auto &e : m_iftlb) { e.valid = false; e.tag = 0; }
			for (auto &e : m_dftlb) { e.valid = false; e.tag = 0; }
		}
		return;
	default:
		LOGMASKED(LOG_CTRL, "%s: write ASI %02x addr %08x = %08x (mask %08x)\n",
			machine().describe_context(), Asi, va, data, mem_mask);
		return;
	}
}

template <uint8_t Asi>
void solbourne_state::asi_map(address_map &map)
{
	map(0x00000000, 0xffffffff).rw(FUNC(solbourne_state::asi_r<Asi>), FUNC(solbourne_state::asi_w<Asi>));
}

template <std::size_t... I>
void solbourne_state::install_asis(std::index_sequence<I...>)
{
	(m_maincpu->set_addrmap(0x10 + int(I), &solbourne_state::asi_map<uint8_t(I)>), ...);
}


//**************************************************************************
//  machine
//**************************************************************************

void solbourne_state::init_sols5e()
{
	// the dumps have their address lines inverted
	uint32_t *rom = reinterpret_cast<uint32_t *>(memregion("maincpu")->base());
	const uint32_t words = memregion("maincpu")->bytes() / 4;
	for (uint32_t i = 0; i < words / 2; i++)
		std::swap(rom[i], rom[words - 1 - i]);
}

static INPUT_PORTS_START( sols5e )
INPUT_PORTS_END

void solbourne_state::sols5e(machine_config &config)
{
	SPARCV7(config, m_maincpu, 40'100'000);
	// The CY7C601 identifies itself in the status register as implementation 1
	// version 1, which is what the OS/MP kernel demands in order to boot.
	m_maincpu->set_nwindows(8);
	m_maincpu->set_impl_ver(1, 1);
	// the machine has a floating point unit: without one the kernel has nowhere
	// to save its state when forking and ends up in a trap
	m_maincpu->set_fpu_present(true);
	install_asis(std::make_index_sequence<0x100>{});

	// the board SCSI controller, at id seven, and a disk at zero, which is
	// where the system boots from
	auto &scsi(NSCSI_BUS(config, "scsi"));
	NSCSI_CONNECTOR(config, "scsi:0", solbourne_scsi_devices, "harddisk", false);   // system disk
	NSCSI_CONNECTOR(config, "scsi:1", solbourne_scsi_devices, "harddisk", false);   // installation medium
	NSCSI_CONNECTOR(config, "scsi:2", solbourne_scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsi:3", solbourne_scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsi:4", solbourne_scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsi:5", solbourne_scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsi:6", solbourne_scsi_devices, nullptr, false);
	WD33C93A(config, m_scsi, 20'000'000);
	scsi.set_external_device(7, m_scsi);
	m_scsi->irq_cb().set(FUNC(solbourne_state::scsi_irq));
	m_scsi->drq_cb().set(FUNC(solbourne_state::scsi_drq));

	// the battery backed real time clock, on the system board; the chip's year
	// zero is 1968 (clock.h)
	MSM58321(config, m_rtc, 32.768_kHz_XTAL);
	m_rtc->set_year0(1968);
	m_rtc->set_default_24h(true);
	m_rtc->d0_handler().set([this] (int state) { rtc_bit(0, state); });
	m_rtc->d1_handler().set([this] (int state) { rtc_bit(1, state); });
	m_rtc->d2_handler().set([this] (int state) { rtc_bit(2, state); });
	m_rtc->d3_handler().set([this] (int state) { rtc_bit(3, state); });

	// The two Z8530s on the system board run at 3.6864 MHz: the boot code
	// programs the divisor to 10 in times-sixteen mode, which at that clock
	// gives the 9600 baud of the console.
	SCC8530(config, m_scc_km, 3'686'400);
	SCC8530(config, m_scc_tty, 3'686'400);
	// table 5-1: both serial ports come in on line 9 and the keyboard with the
	// mouse on line b
	m_scc_tty->out_int_callback().set(
		[this] (int state)
		{
			if (state && !m_scc_tty_irq_prev)
				io_intr(IOINT_SERIAL, 1);
			m_scc_tty_irq_prev = state != 0;
		});
	m_scc_km->out_int_callback().set(
		[this] (int state)
		{
			if (state && !m_scc_km_irq_prev)
				io_intr(IOINT_KBD, 1);
			m_scc_km_irq_prev = state != 0;
		});

	rs232_port_device &ttya(RS232_PORT(config, "ttya", default_rs232_devices, "terminal"));
	ttya.rxd_handler().set(m_scc_tty, FUNC(scc8530_device::rxa_w));
	ttya.cts_handler().set(m_scc_tty, FUNC(scc8530_device::ctsa_w));
	ttya.dcd_handler().set(m_scc_tty, FUNC(scc8530_device::dcda_w));
	m_scc_tty->out_txda_callback().set(ttya, FUNC(rs232_port_device::write_txd));
	m_scc_tty->out_rtsa_callback().set(ttya, FUNC(rs232_port_device::write_rts));
	ttya.set_option_device_input_defaults("terminal", DEVICE_INPUT_DEFAULTS_NAME(tty_9600));
	ttya.set_option_device_input_defaults("pty", DEVICE_INPUT_DEFAULTS_NAME(tty_9600));
	ttya.set_option_device_input_defaults("null_modem", DEVICE_INPUT_DEFAULTS_NAME(tty_9600));

	rs232_port_device &ttyb(RS232_PORT(config, "ttyb", default_rs232_devices, nullptr));
	ttyb.rxd_handler().set(m_scc_tty, FUNC(scc8530_device::rxb_w));
	m_scc_tty->out_txdb_callback().set(ttyb, FUNC(rs232_port_device::write_txd));

	RAM(config, m_ram).set_default_size("32M");
}

ROM_START( sols5e )
	ROM_REGION32_BE( 0x80000, "maincpu", 0 )
	ROM_LOAD32_BYTE( "s5e_3.6b_d732_03.bin", 0, 0x20000, CRC(bb6ce8d6) SHA1(6d83e28f9ebac794ad05f47bdd578ef9c1e07de8) )
	ROM_LOAD32_BYTE( "s5e_3.6b_623a_02.bin", 1, 0x20000, CRC(56c582fb) SHA1(367b2a9bedf5e5fc7a2aa1f7a05e8c1101b09dd7) )
	ROM_LOAD32_BYTE( "s5e_3.6b_fdd0_01.bin", 2, 0x20000, CRC(454e8e62) SHA1(dba27acd4abd5e9873dacfaff982c59175de34f6) )
	ROM_LOAD32_BYTE( "s5e_3.6b_45aa_00.bin", 3, 0x20000, CRC(52315d71) SHA1(4e6cdfadd1c8a1dd39f6e1b9e09a90f1d9e6b54c) )

	ROM_REGION( 0x800, "sysid", 0 )
	ROM_LOAD( "30341id.bin", 0x000, 0x800, CRC(cdf8cffe) SHA1(725279d995100a37c8a01edfdb3af71332a55c89) )
ROM_END

} // anonymous namespace


COMP( 1991, sols5e, 0, 0, sols5e, sols5e, solbourne_state, init_sols5e, "Solbourne Computer Inc", "Series 5E Computer Workstation", MACHINE_NO_SOUND | MACHINE_SUPPORTS_SAVE )
