// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Clement Burin des Roziers                       *
 *   clement.burin-des-roziers@hikob.com                                   *
 *                                                                         *
 *   Copyright (C) 2023 by LuYao                                           *
 *   themaluyao@outlook.com                                                *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>

/* air001 flash register locations */

#define FLASH_ACR		    	0x00
// uint32_t FLASH_RESERVED1 	0x04
#define FLASH_KEYR		    	0x08
#define FLASH_OPTKEYR       	0x0C
#define FLASH_SR		    	0x10
#define FLASH_CR		    	0x14
// uint32_t FLASH_RESERVED2 	0x1C	/* 0x18 - 0x1C */
#define FLASH_OPTR		    	0x20
#define FLASH_SDKR		    	0x24
// uint32_t FLASH_RESERVED3		0x28
#define FLASH_WRPR	        	0x2C
// uint32_t FLASH_RESERVED4		0x2C	/* 0x2C - 0x90 */
#define FLASH_STCR	        	0x90
// uint32_t FLASH_RESERVED5		0x2C	/* 0x90 - 0x100 */
#define FLASH_TS0	        	0x100
#define FLASH_TS1	        	0x104
#define FLASH_TS2P	        	0x108
#define FLASH_TPS3	        	0x10C
#define FLASH_TS3	        	0x110
#define FLASH_PERTPE	    	0x114
#define FLASH_SMERTPE	    	0x118
#define FLASH_PRGTPE	    	0x11C
#define FLASH_PRETPE	    	0x120


/* FLASH_ACR bites */
#define FLASH_ACR__LATENCY		(1<<0) 		// Read flash wait status
                                       		// AIR001 Reserved(31:1)

/* FLASH_CR bits */
#define FLASH_CR__PG		    (1<<0) 		// Program
#define FLASH_CR__PER   		(1<<1) 		// Page erase
#define FLASH_CR__MER   		(1<<2) 		// Mass erase
                                       		// AIR001 Reserved(bit 10:3)
#define FLASH_CR__SER		    (1<<11)		// Sector erase
                                       		// AIR001 Reserved(bit 16:12)
#define FLASH_CR__OPTSTRT		(1<<17)		// Options bytes Start to alter
                                       		// AIR001 Reserved(bit 18)
#define FLASH_CR__PGSTRT	    (1<<19)		// Program Start flag, software set, hardware clear.
                                       		// AIR001 Reserved(bit 23:20)
#define FLASH_CR__EOPIE		    (1<<24)		// When operate end(EOP set), enable interrupt
#define FLASH_CR__ERRIE		    (1<<25)		// Enable error interrupt
                                       		// AIR001 Reserved(bit 26)
#define FLASH_CR__OBL_LAUNCH	(1<<27)		// Force options bytes reload
                                       		// AIR001 Reserved(bit 29:28)
#define FLASH_CR__OPTLOCK		(1<<30)		// Options bytes Lock bit
#define FLASH_CR__LOCK		    (1<<31)		// FLASH_CR register lock bit

/* FLASH_SR bits */ 
#define FLASH_SR__EOP		    (1<<0) 		// Flash read/write complete, hardware set. first EOPIE bit need set. write 1 to clear.
                                       		// AIR001 Reserved(bit 3:1)
#define FLASH_SR__WRPERR	    (1<<4) 		// Write protection error
                                       		// AIR001 Reserved(bit 14:5)
#define FLASH_SR__OPTVERR	    (1<<15)		// Option and trimming bits loading validity error
#define FLASH_SR__BSY		    (1<<16)		// (1<<0), busy flag, hardware set, software clear when operate complete or occur error
                                       		// AIR001 Reserved(bit 31:17)

/* Unlock keys */
#define KEY1			        0x45670123  // Write KEY1 and KEY2 to FLASH_KEYR can unlock FLASH_CR
#define KEY2			        0xCDEF89AB 
#define OPTKEY1			        0x08192A3B  // Write OPTKEY1 and OPTKEY2 to FLASH_OPTKEYR can unlock Options bytes
#define OPTKEY2			        0x4C5D6E7F

/* other registers */
#define DBGMCU_IDCODE		    0xE0042000  // DBGMCU_IDCODE register address
#define DBGMCU_IDCODE_L0	    0x40015800

/* Constants */
#define FLASH_SECTOR_SIZE       4096
#define FLASH_BANK0_ADDRESS     0x08000000  // flash base address

/* option bytes */
#define OPTION_BYTES_ADDRESS    0x1FFF0E80  // Options bytes base address

#define OPTION_BYTE_0_PR1       0x41FFBE00  // Level 1 default value
#define OPTION_BYTE_0_PR0       0x4155BEAA  // Level 0 default value

#define IWDG_BASE				0x40003000  // IWDG BASE

#define MAX_ERASE_SECTORS_ONCE	7


static int air001_mass_erase(struct flash_bank *bank);
static int air001_lock(struct flash_bank *bank);
static int air001_unlock(struct flash_bank *bank);

static int air001_erase_sectors(struct flash_bank *bank, unsigned int first, unsigned int last);

static int air001_do_write(struct flash_bank* bank, const uint8_t* buffer, uint32_t offset, uint32_t count);
static int air001_write_block_async(struct flash_bank* bank, const uint8_t* buffer, uint32_t offset, uint32_t words_count);
static int air001_write_block_sync(struct flash_bank* bank, const uint8_t* buffer, uint32_t offset, uint32_t count);
static int air001_write_page(struct flash_bank* bank, const uint8_t* buffer, uint32_t offset);
static int air001_write_single(struct flash_bank* bank, const uint8_t* buffer, uint32_t offset, uint32_t count);

static int air001_unlock_options_bytes(struct flash_bank* bank);
static int air001_unlock_program_memory(struct flash_bank* bank);
static int air001_lock_program_memory(struct flash_bank* bank);
static int air001_wait_until_bsy_clear(struct flash_bank *bank);
static int air001_wait_until_bsy_clear_timeout(struct flash_bank *bank, int timeout);

struct air001_rev {
	uint16_t rev;
	const char *str;
};

struct air001_part_info {
	uint16_t		id;
	const char		*device_str;
	const struct air001_rev *revs;
	size_t			num_revs;
	unsigned int 	page_size;
	unsigned int 	pages_per_sector;
	uint16_t 		max_flash_size_kb;
	uint16_t 		first_bank_size_kb;
	bool			has_dual_banks;		/* used when has_dual_banks is true		*/

	uint32_t 		flash_reg_base;		/* Flash controller registers location	*/
	uint32_t 		fsize_base;			/* Location of FSIZE register			*/
};

struct air001_flash_bank {
	bool		probed;
	uint32_t 	idcode;
	uint32_t 	user_bank_size;
	uint32_t 	flash_reg_base;

	struct air001_part_info part_info;
};

static const struct air001_rev air_467_revs[] = {
    /* MCU revision code define, device id high 16 bit.(This data is not offical.) */
	{ 0x6000, "A" }, { 0x6008, "I" },
};

static const struct air001_part_info air001_parts[] = {
	{
		.id					= 0x467,        /* = device id & 0xFFF */
		.revs				= air_467_revs,
		.num_revs			= ARRAY_SIZE(air_467_revs),
		.device_str			= "AIR001 (Cat.1 - Low/Medium Density)",
		.page_size			= 128,
		.pages_per_sector	= 32,
		.max_flash_size_kb	= 32,
		.has_dual_banks		= false,
		.flash_reg_base		= 0x40022000,
		.fsize_base			= 0x1FF8007C,   /* flash size register base address. air001 not support. */
	},
};

/* choose async/sync to write image */
static bool async = 0;


/* flash bank air001 <base> <size> 0 0 <target#>
 * int air001_flash_bank_command(struct command_invocation *cmd, struct flash_bank *bank)
 */
FLASH_BANK_COMMAND_HANDLER(air001_flash_bank_command)
{
	struct air001_flash_bank* air001_flash_bank;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Create the bank structure */
	air001_flash_bank = calloc(1, sizeof(*air001_flash_bank));

	/* Check allocation */
	if (!air001_flash_bank) {
		LOG_ERROR("failed to allocate bank structure");
		return ERROR_FAIL;
	}

	bank->driver_priv = air001_flash_bank;
	bank->default_padded_value = bank->erased_value = 0xFF;

	air001_flash_bank->probed = false;

	return ERROR_OK;
}

/* int air001_handle_mass_erase_command(struct command_invocation *cmd)
 */
COMMAND_HANDLER(air001_handle_mass_erase_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank* bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = air001_mass_erase(bank);
	if (retval == ERROR_OK)
		command_print(CMD, "air001 mass erase complete");
	else
		command_print(CMD, "air001 mass erase failed");

	return retval;
}

/* int air001_handle_lock_command(struct command_invocation *cmd)
 */
COMMAND_HANDLER(air001_handle_lock_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank* bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = air001_lock(bank);

	if (retval == ERROR_OK)
		command_print(CMD, "air001 locked, takes effect after power cycle.");
	else
		command_print(CMD, "air001 lock failed");

	return retval;
}

/* int air001_handle_unlock_command(struct command_invocation *cmd)
 */
COMMAND_HANDLER(air001_handle_unlock_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank* bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = air001_unlock(bank);

	if (retval == ERROR_OK)
		command_print(CMD, "air001 unlocked, takes effect after power cycle.");
	else
		command_print(CMD, "air001 unlock failed");

	return retval;
}

COMMAND_HANDLER(air001_handle_write_async_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], async);

	LOG_INFO("air001 write algorithm: async = %s", CMD_ARGV[0]);
	return ERROR_OK;
}


static int air001_mass_erase(struct flash_bank* bank)
{
	const struct air001_flash_bank* air001_flash_bank = bank->driver_priv;
	struct target* target = bank->target;
	uint32_t reg32;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Wait for last operation to be completed */
	retval = air001_wait_until_bsy_clear(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Authorize the FLASH Registers access */
	retval = air001_unlock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Mass erase */
	retval = target_read_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, reg32 | FLASH_CR__MER);
	if (retval != ERROR_OK)
		return retval;

	/* Write any word to any main flash. */
	retval = target_write_u32(target, bank->base, 0x12344321);
	if (retval != ERROR_OK)
		return retval;

	/* Wait for last operation to be completed */
	retval = air001_wait_until_bsy_clear(bank);
	if (retval != ERROR_OK)
		return retval;

	/* If the erase operation is completed, disable the MER Bit */
	retval = target_read_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, reg32 & ~FLASH_CR__MER);

	return retval;
}

static int air001_lock(struct flash_bank* bank)
{
	struct target* target = bank->target;
	const struct air001_flash_bank* air001_bank = bank->driver_priv;
	const struct air001_part_info air001_info = air001_bank->part_info;
	uint32_t reg32;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* read FLASH_CR */
	retval = target_read_u32(target, air001_info.flash_reg_base + FLASH_CR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	/* lock the option bytes through OPTLOCK(bit30) */
	if ((reg32 & FLASH_CR__OPTLOCK) != 0) {
		retval = target_write_u32(target, air001_info.flash_reg_base + FLASH_CR, reg32 | FLASH_CR__OPTLOCK);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = air001_wait_until_bsy_clear_timeout(bank, 1000);
	if (retval != ERROR_OK)
		return retval;

	/* lock the FLASH_CR through LOCK(bit31) */
	if ((reg32 & FLASH_CR__LOCK) != 0) {
		retval = target_write_u32(target, air001_info.flash_reg_base + FLASH_CR, reg32 | FLASH_CR__LOCK);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int air001_unlock(struct flash_bank* bank)
{
	int retval;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* unlock option bytes */
	retval = air001_unlock_options_bytes(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = air001_wait_until_bsy_clear_timeout(bank, 1000);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}


static int air001_erase(struct flash_bank* bank, const unsigned int first, const unsigned int last)
{
	int32_t num_sectors = last - first;
	unsigned int erase_first = first, erase_last;
	int error_times = 0;
	int retval;

	struct duration bench;
	duration_start(&bench);

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* erase sectors with specified sectors. */
	while (num_sectors >= 0) {
		erase_last = erase_first + MIN(num_sectors, MAX_ERASE_SECTORS_ONCE);

		/* Loop over the selected sectors and erase them. */
		retval = air001_erase_sectors(bank, erase_first, erase_last);
		if (retval != ERROR_OK) {
			LOG_WARNING("air001 erase sector from %d to %d error(retry):%d", erase_first, erase_last, retval);
			if (error_times++ >= 2) {
				return retval;
			}
			alive_sleep(1);
			continue;
		}

		erase_first = erase_last;
		num_sectors -= MAX_ERASE_SECTORS_ONCE;
	}

	/* marked the sectors as erased */
	for (unsigned int i = first; i <= last; i++) {
		bank->sectors[i].is_erased = 1;
	}

	if (duration_measure(&bench) == ERROR_OK)
		LOG_INFO("air001 erase %d sectors in %fs", last - first + 1, duration_elapsed(&bench));

	return ERROR_OK;
}

static int air001_erase_sectors(struct flash_bank* bank, unsigned int first, unsigned int last)
{
	struct target* target = bank->target;
	const struct air001_flash_bank* air001_bank = bank->driver_priv;
	struct armv7m_algorithm armv7m_info;
	struct working_area* erase_algorithm;
	struct reg_param reg_params[4];
	int retval;


	static const uint8_t air001_erase_sector_code[] = {
#include "../../../contrib/loaders/flash/airm2m/air001_erase.inc"
	};

	LOG_DEBUG("air001 erase sectors from %d(0x%08llX) to %d(0x%08llX)",
	          first, bank->base + first * FLASH_SECTOR_SIZE,
	          last, bank->base + last * FLASH_SECTOR_SIZE);

	/* unlock FLASH_CR */
	retval = air001_unlock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;

	/* allocate working area to run algorithm. */
	retval = target_alloc_working_area(target, sizeof(air001_erase_sector_code), &erase_algorithm);
	if (retval != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* write the algorithm code to the working area. */
	retval = target_write_buffer(target, erase_algorithm->address, sizeof(air001_erase_sector_code),
	                             air001_erase_sector_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, erase_algorithm);
		return retval;
	}
	LOG_WARNING("code base addr %llx", erase_algorithm->address);
	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT); /* return value, flash base */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);    /* IWDG base  				 */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);    /* sector begin address 	 */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);    /* sector end address   	 */

	buf_set_u32(reg_params[0].value, 0, 32, air001_bank->flash_reg_base);
	buf_set_u32(reg_params[1].value, 0, 32, IWDG_BASE);
	buf_set_u32(reg_params[2].value, 0, 32, bank->base + first * FLASH_SECTOR_SIZE);
	buf_set_u32(reg_params[3].value, 0, 32, bank->base + last * FLASH_SECTOR_SIZE);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_algorithm(target, 0, NULL,
	                              ARRAY_SIZE(reg_params), reg_params,
	                              erase_algorithm->address, 0,
	                              2000, &armv7m_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("air001 erase error: r0 = 0x%08X, retval = %d", buf_get_u32(reg_params[0].value, 0, 32), retval);
	}

	target_free_working_area(target, erase_algorithm);
	for (uint8_t i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

	return retval;
}


static int air001_write(struct flash_bank* bank, const uint8_t* buffer, uint32_t offset, uint32_t count)
{
	const struct air001_flash_bank* air001_flash_bank = bank->driver_priv;
	const unsigned int page_size = air001_flash_bank->part_info.page_size;
	int retval;

	struct duration bench;
	duration_start(&bench);

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x3) {
		LOG_ERROR("air001 offset 0x%" PRIx32 " breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* the data is written from one boundary of a page to another. */
	if ((bank->base + offset) % page_size == 0 && (bank->base + offset + count) % page_size == 0) {
		LOG_DEBUG("air001 write: base = 0x%08llX, offset = 0x%X, count = %d", bank->base, offset, count);
		retval = air001_do_write(bank, buffer, offset, count);
		if (retval != ERROR_OK)
			return retval;
	} else if (offset / page_size == (offset + count) / page_size) {
		/* the data that need to write is in a page. */
		const uint32_t page_head = (bank->base + offset) % page_size;
		uint8_t buf[page_size];

		/* read a page_size data from this page's boundary. */
		retval = target_read_buffer(bank->target, bank->base + offset - page_head, page_size, buf);
		if (retval != ERROR_OK)
			return retval;
		memcpy(buf + page_head, buffer, count);

		offset -= page_head;

		LOG_DEBUG("air001 write: base = 0x%08llX, offset = 0x%X, count = %d", bank->base, offset, page_size);
		retval = air001_do_write(bank, buf, offset, page_size);
		if (retval != ERROR_OK)
			return retval;
	} else {
		/* the data count more than one page size. */
		const uint32_t page_first_head = (bank->base + offset) % page_size;
		const uint32_t page_last_tail = (bank->base + offset + count) % page_size;
		const uint32_t page_last_sub = page_last_tail == 0 ? 0 : page_size - page_last_tail;
		uint8_t buffer_padding[page_first_head + count + page_last_sub];

		/* section 1 of buffer: the data that the first page remain. */
		retval = target_read_buffer(bank->target, bank->base + offset - page_first_head,
		                            page_first_head, buffer_padding);
		if (retval != ERROR_OK)
			return retval;

		/* section 2 of buffer: the data that prepare to write. */
		memcpy(buffer_padding + page_first_head, buffer, count);

		/* section 3 of buffer: the data of the last page remain. */
		retval = target_read_buffer(bank->target, bank->base + offset + count,
		                            page_last_sub, buffer_padding + page_first_head + count);
		if (retval != ERROR_OK)
			return retval;

		offset -= page_first_head;
		count = sizeof(buffer_padding);

		LOG_DEBUG("air001 write: base = 0x%08llX, offset = 0x%X, count = %d", bank->base, offset, count);
		retval = air001_do_write(bank, buffer_padding, offset, count);
		if (retval != ERROR_OK)
			return retval;
	}

	if (duration_measure(&bench) == ERROR_OK)
		LOG_INFO("air001 wrote %" PRIu32 " bytes in %fs (%0.3f KiB/s)", count,
	         duration_elapsed(&bench), duration_kbps(&bench, count));

	return retval;
}

static int air001_do_write(struct flash_bank* bank, const uint8_t* buffer, uint32_t offset, uint32_t count)
{
	int retval;

	/* try using a block write */
	if (async) {
		retval = air001_write_block_async(bank, buffer, offset, count / 4);
	} else {
		retval = air001_write_block_sync(bank, buffer, offset, count);
	}

	if (retval != ERROR_OK) {
		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
			/* if block write failed (no sufficient working area), use normal (slow) SWD/JTAG method */
			LOG_WARNING("air001 couldn't use block writes, falling back to single memory accesses");

			retval = air001_write_single(bank, buffer, offset, count);
			if (retval != ERROR_OK) {
				LOG_ERROR("air001 slow write failed");
				return ERROR_FLASH_OPERATION_FAILED;
			}
		}
	}

	return retval;
}

static int air001_write_block_async(struct flash_bank* bank, const uint8_t* buffer,
                                    uint32_t offset, const uint32_t words_count)
{
	const struct air001_flash_bank* air001_flash_bank = bank->driver_priv;
	struct target* target = bank->target;
	struct working_area* write_algorithm; /* a working area information for the write algorithm	*/
	struct working_area* source;          /* a working area information for the data transfer	*/
	struct armv7m_algorithm armv7m_info;
	uint32_t buffer_size;
	int retval;

	static const uint8_t air001_flash_write_code[] = {
#include "../../../contrib/loaders/flash/airm2m/air001_write_async.inc"
	};

	/* unlock */
	retval = air001_unlock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;

	/* flash alloc for write code */
	if (target_alloc_working_area(target, sizeof(air001_flash_write_code),
	                              &write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	/* write code and config write_algorithm */
	retval = target_write_buffer(target, write_algorithm->address,
	                             sizeof(air001_flash_write_code), air001_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* memory buffer */
	buffer_size = target_get_working_area_avail(target);
	buffer_size = MIN(words_count * 4 + 8, MAX(buffer_size, 256));
	retval = target_alloc_working_area(target, buffer_size, &source);
	if (retval != ERROR_OK) {
		LOG_WARNING("no large enough working area available, can't do block memory writes");
		target_free_working_area(target, source);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	struct reg_param reg_params[5];

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT); /* flash base (out), status (in)	*/
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);    /* count (word-32bit)				*/
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);    /* buffer start					*/
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);    /* buffer end						*/
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT); /* target address					*/

	buf_set_u32(reg_params[0].value, 0, 32, air001_flash_bank->flash_reg_base);
	buf_set_u32(reg_params[1].value, 0, 32, words_count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, bank->base + offset);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	retval = target_run_flash_async_algorithm(target, buffer, words_count, 4,
	                                          0, NULL,
	                                          ARRAY_SIZE(reg_params), reg_params,
	                                          source->address, source->size,
	                                          write_algorithm->address, 0,
	                                          &armv7m_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("air001 write async failed just before address 0x%08X, retval = %d",
		          buf_get_u32(reg_params[4].value, 0, 32), retval);
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	return retval;
}

static int air001_write_block_sync(struct flash_bank* bank, const uint8_t* buffer,
                                   uint32_t offset, uint32_t count)
{
	struct target* target = bank->target;
	const struct air001_flash_bank* air001_flash_bank = bank->driver_priv;
	const unsigned int page_size = air001_flash_bank->part_info.page_size;
	uint32_t buffer_size = 4096 / page_size * page_size;
	uint32_t address = bank->base + offset;

	struct armv7m_algorithm armv7m_info;
	struct working_area* write_algorithm;
	struct working_area* source;
	struct reg_param reg_params[5];
	int retval;

	static const uint8_t air001_write_sync_code[] = {
#include "../../../contrib/loaders/flash/airm2m/air001_write_sync.inc"
	};

	/* unlock */
	retval = air001_unlock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;

	/* allocate working area to run algorithm. */
	retval = target_alloc_working_area(target, sizeof(air001_write_sync_code), &write_algorithm);
	if (retval != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* write the algorithm code to the working area. */
	retval = target_write_buffer(target, write_algorithm->address, sizeof(air001_write_sync_code),
	                             air001_write_sync_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* Allocate pages memory for working area of source. */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		if (buffer_size > 1024)
			buffer_size -= 1024;
		else
			buffer_size /= 2;

		if (buffer_size <= page_size) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		/* Make sure we're still asking for an integral number of pages */
		buffer_size -= buffer_size % page_size;
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;
	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT); /* return value, flash base */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);    /* IWDG base    			 */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);    /* target address			 */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);    /* source address			 */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);    /* word count    			 */

	while (count > 0) {
		uint32_t this_count = count > buffer_size ? buffer_size : count;

		/* write the source code to the working area. */
		retval = target_write_buffer(target, source->address, this_count, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, air001_flash_bank->flash_reg_base);
		buf_set_u32(reg_params[1].value, 0, 32, IWDG_BASE);
		buf_set_u32(reg_params[2].value, 0, 32, address);
		buf_set_u32(reg_params[3].value, 0, 32, source->address);
		buf_set_u32(reg_params[4].value, 0, 32, this_count / 4);

		retval = target_run_algorithm(target, 0, NULL,
		                              ARRAY_SIZE(reg_params), reg_params,
		                              write_algorithm->address, 0,
		                              5000, &armv7m_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("air001 write sync error: %d", retval);
			break;
		}

		buffer += this_count;
		address += this_count;
		count -= this_count;
	}

	target_free_working_area(target, write_algorithm);
	target_free_working_area(target, source);

	for (uint8_t i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

	return retval;
}

static int air001_write_page(struct flash_bank* bank, const uint8_t* buffer, uint32_t offset)
{
	struct target* target = bank->target;
	struct air001_flash_bank* air001_flash_bank = bank->driver_priv;
	int retval;
	uint32_t reg32;
	uint8_t index = 0, count = 0;

	/* The offset address must be the head address of page, and the buffer length must
	 * large than page size.
	 */

	retval = air001_wait_until_bsy_clear(bank);
	if (retval != ERROR_OK)
		return retval;

	/* set PG bit of FLASH_CR. */
	retval = target_read_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, &reg32);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, reg32 | FLASH_CR__PG);
	if (retval != ERROR_OK)
		return retval;

	/* air001 has 32 words per page. */
	while (count < 32U) {
		uint32_t word = (buffer[index + 0] << 0) |
						(buffer[index + 1] << 8) |
						(buffer[index + 2] << 16) |
						(buffer[index + 3] << 24);
		retval = target_write_u32(target, bank->base + offset, word);
		if (retval != ERROR_OK)
			return retval;

		offset += 4;
		index += 4;
		count++;

		/* After writing 31 words, set PGSTART bit of FLASH_CR, then write the last word. */
		if (count == 31) {
			retval = target_read_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, &reg32);
			if (retval != ERROR_OK)
				return retval;
			retval = target_write_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, reg32 | FLASH_CR__PGSTRT);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	retval = air001_wait_until_bsy_clear(bank);
	if (retval != ERROR_OK)
		return retval;

	/* clear PGSTART bit of FLASH_CR. */
	retval = target_read_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, &reg32);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, reg32 & (~FLASH_CR__PG));
	if (retval != ERROR_OK)
		return retval;
	return ERROR_OK;
}

static int air001_write_single(struct flash_bank* bank, const uint8_t* buffer,
                               const uint32_t offset, const uint32_t count)
{
	const struct air001_flash_bank* air001_flash_bank = bank->driver_priv;
	const unsigned int page_size = air001_flash_bank->part_info.page_size;
	int retval = ERROR_OK;

	/* unlock */
	retval = air001_unlock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;

	/* write every page. */
	for (uint32_t i = 0; i < count / page_size; i++) {
		retval = air001_write_page(bank, buffer + i * page_size, offset  + i * page_size);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = air001_lock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}


static int air001_read_id_code(struct target* target, uint32_t* id)
{
	/* air001 is Cortex-M0+, belong to Armv6-M. However, the DBG_IDCODE register
	 * in air001 is reserved.
	 */
	struct armv7m_common* armv7m = target_to_armv7m(target);
	int retval;
	if (armv7m->arm.arch == ARM_ARCH_V6M)
		retval = target_read_u32(target, DBGMCU_IDCODE_L0, id);
	else
		/* read air001 device id register */
		retval = target_read_u32(target, DBGMCU_IDCODE, id);
	return retval;
}

static int air001_probe(struct flash_bank* bank)
{
	struct air001_flash_bank* air001_flash_bank = bank->driver_priv;
	uint16_t flash_size_in_kb;
	uint32_t device_id;

	air001_flash_bank->probed = false;

	/* read device id in DBG_IDCODE register(air001 is reserved, the return value
	 * always is 0x60001000). */
	int retval = air001_read_id_code(bank->target, &device_id);
	if (retval != ERROR_OK)
		return retval;

	air001_flash_bank->idcode = device_id;

	LOG_INFO("air001 device id = 0x%08" PRIx32 "", device_id);

	/* Match the part_info->id of air001 according to the device_id & 0xFFF if the DBG_IDCODE
	 * has a ID. */
	air001_flash_bank->part_info = air001_parts[0];
	air001_flash_bank->flash_reg_base = air001_flash_bank->part_info.flash_reg_base;

	/* get the flash size from the flash size register of part_info->fsize_base,
     * but air001 not support, so just write here from datasheet.
     */
	flash_size_in_kb = air001_flash_bank->part_info.max_flash_size_kb;

	free(bank->sectors);

	bank->size = bank->size == 0 ? flash_size_in_kb * 1024 : bank->size;
	bank->base = bank->base == 0 ? FLASH_BANK0_ADDRESS : bank->base;

	/* calculate numbers of sectors (4kB per sector) */
	unsigned int num_sectors = bank->size / FLASH_SECTOR_SIZE;
	bank->num_sectors = num_sectors;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_sectors);
	if (!bank->sectors) {
		LOG_ERROR("failed to allocate bank sectors");
		return ERROR_FAIL;
	}

	for (unsigned int i = 0; i < num_sectors; i++) {
		bank->sectors[i].offset = i * FLASH_SECTOR_SIZE;
		bank->sectors[i].size = FLASH_SECTOR_SIZE;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}
	air001_flash_bank->probed = true;

	LOG_INFO("air001 flash size is %dkb, base address is 0x%08llX", flash_size_in_kb, bank->base);

	return ERROR_OK;
}

static int air001_auto_probe(struct flash_bank* bank)
{
	struct air001_flash_bank* air001_info = bank->driver_priv;

	if (air001_info->probed)
		return ERROR_OK;

	return air001_probe(bank);
}


static int air001_protect_check(struct flash_bank* bank)
{
	struct air001_flash_bank* air001_flash_bank = bank->driver_priv;
	struct target* target = bank->target;
	uint32_t wrpr;
	int retval;

	/*
	 * Read the WRPR word, and check each bit (corresponding to each
	 * flash sector
	 */
	retval = target_read_u32(target, air001_flash_bank->flash_reg_base + FLASH_WRPR, &wrpr);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		if (wrpr & (1 << i))
			bank->sectors[i].is_protected = 1;
		else
			bank->sectors[i].is_protected = 0;
	}
	return ERROR_OK;
}

static int air001_get_info(struct flash_bank* bank, struct command_invocation* cmd)
{
	const struct air001_flash_bank* air001_flash_bank = bank->driver_priv;
	const struct air001_part_info* air001_part_info = &air001_flash_bank->part_info;
	const uint16_t rev_id = air001_flash_bank->idcode >> 16;
	const char* rev_str = NULL;
	int retval;

	if (!air001_flash_bank->probed) {
		retval = air001_probe(bank);
		if (retval != ERROR_OK) {
			command_print_sameline(cmd, "Unable to find bank information.");
			return retval;
		}
	}

	for (unsigned int i = 0; i < air001_part_info->num_revs; i++)
		if (rev_id == air001_part_info->revs[i].rev)
			rev_str = air001_part_info->revs[i].str;

	if (rev_str) {
		command_print_sameline(cmd, "%s - Rev: %s", air001_part_info->device_str, rev_str);
	} else {
		command_print_sameline(cmd, "%s - Rev: unknown (0x%04x)", air001_part_info->device_str, rev_id);
	}

	return ERROR_OK;
}


static int air001_unlock_options_bytes(struct flash_bank* bank)
{
	const struct air001_flash_bank* air001_flash_bank = bank->driver_priv;
	struct target* target = bank->target;
	uint32_t reg32;
	int retval;

	/*
	* Unlocking the options bytes is done by unlocking the CR,
	* then by writing the 2 OPTKEY to the FLASH_OPTKEYR register
	*/

	/* check flash is not already unlocked */
	retval = target_read_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * OPTLOCK is in FLASH_CR register bit30, this bit set only by software, then
	 * the options bytes locked.
	 */
	if ((reg32 & FLASH_CR__OPTLOCK) == 0)
		return ERROR_OK;

	/* unlock FLASH_CR. */
	retval = air001_unlock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;

	/* To unlock the options bits in FLASH_CR, write the 2 OPTKEY to the FLASH_OPTKEYR register */
	retval = target_write_u32(target, air001_flash_bank->flash_reg_base + FLASH_OPTKEYR, OPTKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, air001_flash_bank->flash_reg_base + FLASH_OPTKEYR, OPTKEY2);
	if (retval != ERROR_OK)
		return retval;

	/* verify option bytes is unlock */
	retval = target_read_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, &reg32);
	if (retval != ERROR_OK)
		return retval;
	/* flash unlocked. */
	if ((reg32 & FLASH_CR__OPTLOCK) == 0)
		return ERROR_OK;

	LOG_ERROR("flash option bytes unlock fail: %" PRId32, reg32);
	return ERROR_FAIL;
}

static int air001_unlock_program_memory(struct flash_bank* bank)
{
	const struct air001_flash_bank* air001_flash_bank = bank->driver_priv;
	struct target* target = bank->target;
	uint32_t reg32;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = air001_wait_until_bsy_clear(bank);
	if (retval != ERROR_OK)
		return retval;

	/* check flash is not already unlocked.
     * LOCK is in FLASH_CR register bit31, this bit set only by software, then
     * the FLASH locked. Give unlock sequence to unlock.
     */
	retval = target_read_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, &reg32);
	if (retval != ERROR_OK)
		return retval;
	/* flash unlocked. */
	if ((reg32 & FLASH_CR__LOCK) == 0)
		return ERROR_OK;

	/* to unlock the FLASH_CR, write the 2 KEY to the FLASH_KEYR register */
	retval = target_write_u32(target, air001_flash_bank->flash_reg_base + FLASH_KEYR, KEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, air001_flash_bank->flash_reg_base + FLASH_KEYR, KEY2);
	if (retval != ERROR_OK)
		return retval;

	/* verify Flash is unlock */
	retval = target_read_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, &reg32);
	if (retval != ERROR_OK)
		return retval;
	/* flash unlocked. */
	if ((reg32 & FLASH_CR__LOCK) == 0)
		return ERROR_OK;

	LOG_ERROR("flash unlock fail: %" PRId32, reg32);
	return ERROR_FAIL;
}

static int air001_lock_program_memory(struct flash_bank* bank)
{
	const struct air001_flash_bank* air001_flash_bank = bank->driver_priv;
	struct target* target = bank->target;
	uint32_t reg32;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* check flash is not already unlocked.
     * LOCK is in FLASH_CR register bit31, this bit set only by software, then
     * the FLASH locked. Give unlock sequence to unlock.
     */
	retval = target_read_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, &reg32);
	if (retval != ERROR_OK)
		return retval;
	/* flash already locked. */
	if ((reg32 & FLASH_CR__LOCK) != 0)
		return ERROR_OK;

	/* set LOCK bit to lock flash.
     */
	retval = target_write_u32(target, air001_flash_bank->flash_reg_base + FLASH_CR, reg32 | FLASH_CR__LOCK);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int air001_wait_until_bsy_clear(struct flash_bank* bank)
{
	return air001_wait_until_bsy_clear_timeout(bank, 100);
}

static int air001_wait_until_bsy_clear_timeout(struct flash_bank* bank, int timeout)
{
	const struct air001_flash_bank* air001_flash_bank = bank->driver_priv;
	struct target* target = bank->target;
	uint32_t status;
	int retval;

	/* wait for busy to clear */
	for (;;) {
		retval = target_read_u32(target, air001_flash_bank->flash_reg_base + FLASH_SR, &status);
		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		if ((status & FLASH_SR__BSY) == 0)
			break;

		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (status & FLASH_SR__WRPERR) {
		LOG_ERROR("access denied / write protected");
		retval = ERROR_FAIL;
	}

	/* PGAERR not exist in air001 FLASH_SR register.  */

	/* Clear but report errors */
	if (status & FLASH_SR__OPTVERR) {
		/* If this operation fails, we ignore it and report the original retval */
		target_write_u32(target, air001_flash_bank->flash_reg_base + FLASH_SR, status & FLASH_SR__OPTVERR);
	}

	return retval;
}


static const struct command_registration air001_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = air001_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device. including available EEPROM",
	},
	{
		.name = "lock",
		.handler = air001_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Increase the readout protection to Level 1.",
	},
	{
		.name = "unlock",
		.handler = air001_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lower the readout protection from Level 1 to 0.",
	},
	{
		.name = "write_async",
		.handler = &air001_handle_write_async_command,
		.mode = COMMAND_ANY,
		.usage = "[on|off]",
		.help = "choose write image in async or sync.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration air001_command_handlers[] = {
	{
		.name = "air001",
		.mode = COMMAND_ANY,
		.help = "air001 flash command group",
		.usage = "",
		.chain = air001_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/* AIR001 flash driver. Here includes all operations on flash. */
const struct flash_driver air001_flash = {
	.name = "air001",
	.commands = air001_command_handlers,
	.flash_bank_command = air001_flash_bank_command,
	.erase = air001_erase,
	.write = air001_write,
	.read = default_flash_read,
	.probe = air001_probe,
	.auto_probe = air001_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = air001_protect_check,
	.info = air001_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
