/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
 *   Author: @author David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_identity.c
 * Implementation of SamV7 based Board identity API
 */

#include <px4_config.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <chip/sam_memorymap.h>
#include <chip/sam_eefc.h>

#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00ff0000) >> 8) | (((x) & 0x0000ff00) << 8) | ((x) << 24))

const uint32_t DUMMY_SIM_UIDH[4] = {0x12345678, 0x12345678, 0x12345678, 0x12345678 };

void board_get_uuid(uuid_byte_t uuid_bytes)
{
	uint32_t *chip_uuid = (uint32_t *) DUMMY_SIM_UIDH;
	uint32_t  *uuid_words = (uint32_t *) uuid_bytes;

	for (unsigned int i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		*uuid_words++ = SWAP_UINT32(chip_uuid[i]);
	}
}

void board_get_uuid32(uuid_uint32_t uuid_words)
{
	board_get_uuid(*(uuid_byte_t *) uuid_words);
}

int board_get_uuid32_formated(char *format_buffer, int size,
			      const char *format,
			      const char *seperator)
{
	uuid_uint32_t uuid;
	board_get_uuid32(uuid);

	int offset = 0;
	int sep_size = seperator ? strlen(seperator) : 0;

	for (unsigned int i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - ((i * 2 * sizeof(uint32_t)) + 1), format, uuid[i]);

		if (sep_size && i < PX4_CPU_UUID_WORD32_LENGTH - 1) {
			strcat(&format_buffer[offset], seperator);
			offset += sep_size;
		}
	}

	return 0;
}

int board_get_mfguid(mfguid_t mfgid)
{
	board_get_uuid(* (uuid_byte_t *) mfgid);
	return PX4_CPU_MFGUID_BYTE_LENGTH;
}

int board_get_mfguid_formated(char *format_buffer, int size)
{
	mfguid_t mfguid;

	board_get_mfguid(mfguid);
	int offset  = 0;

	for (unsigned int i = 0; i < PX4_CPU_MFGUID_BYTE_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - offset, "%02x", mfguid[i]);
	}

	return offset;
}
