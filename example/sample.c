/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * MOS650x - The MOS 6502 CPU emulator
 * Copyright 2021-2025 Renê de Souza Pinto
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include "cpu650x.h"

/** Exit flag */
uint8_t exit_prg;

/** System's memory */
uint8_t cpu_mem[65536];

/* SBUS functions implementation */

uint8_t sbus_read(uint16_t addr)
{
	return cpu_mem[addr];
}

void sbus_write(uint16_t addr, uint8_t value)
{
	cpu_mem[addr] = value;
}

void sbus_init()
{
	/* Nothing to do */
}

void sbus_destroy()
{
	/* Nothing to do */
}

/**
 * CPU instruction callback
 */
static void cpu_debug_cb(cpu650x_state_t debug_info)
{
	printf("cpu: %s | A:0x%.2x X:0x%.2x Y: 0x%.2x P: 0x%.2x | DATA: 0x%.2x | ADDR: 0x%.4x\n",
			debug_info.opcode_name,
			debug_info.CPU.A, debug_info.CPU.X,
			debug_info.CPU.Y, debug_info.CPU.P.reg,
			debug_info.data, debug_info.address);
}

/**
 * Signals callback
 */
static void sig_cb(int signum)
{
	exit_prg = 1;
}

/**
 * Main: Run a small binary code
 */
int main(int argc, const char *argv[])
{
	struct sigaction sact;

	memset(&sact, 0, sizeof(sact));
	sact.sa_handler = sig_cb;
	sigaction(SIGTERM, &sact, NULL);
	sigaction(SIGINT, &sact, NULL);

	/* Initialize CPU and System Bus */
	sbus_init();
	cpu_init();
	cpu_register_debug_cb(cpu_debug_cb);

	/*
	 * LDA #4
	 * LDX #2
	 * STA 0x0f00
	 * INC 0x0f00
	 * LDA 0x0f00,X
	 * LDA #6
	 * STA 0x50
	 * LDA #254
	 * ADC 0x50
	 * TAX
	 * LDA #211
	 * STA 0x51
	 * LDA #13
	 * ADC 0x51
	 * TAX
	 * SEC
	 * LDA #6
	 * STA 0x52
	 * LDA #254
	 * ADC 0x52
	 * loop:
	 * jmp loop
	 */
	char testcode[] = {
		0xA9, 0x04, 0xA2, 0x02, 0x8D, 0x00, 0x0F, 0xEE,
		0x00, 0x0F, 0xBD, 0x00, 0x0F, 0xA9, 0x06, 0x85,
		0x50, 0xA9, 0xFE, 0x65, 0x50, 0xAA, 0xA9, 0xD3,
		0x85, 0x51, 0xA9, 0x0D, 0x65, 0x51, 0xAA, 0x38,
		0xA9, 0x06, 0x85, 0x52, 0xA9, 0xFE, 0x65, 0x52,
		0x4C, 0x28, 0x00
	};

	/* Copy code to memory */
	for (int i = 0; i < sizeof(testcode)/sizeof(char); i++)
		sbus_write(i, testcode[i]);

	sbus_write(0xfffc, 0);
	sbus_write(0xfffd, 0);

	/* Reset CPU */
	printf("Starting execution...\n");
	cpu_reset();
	while (!exit_prg) {
		cpu_clock();
		usleep(30000);
	}

	cpu_destroy();
	sbus_destroy();
	printf("\nDone.\n");
	return 0;
}

