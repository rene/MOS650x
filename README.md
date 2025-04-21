# The MOS 650x CPU Emulator

The MOS 650x CPU Emulator aims to develop a lightweight, high portable,
fast and reduced memory foot print 6502 CPU Emulator.

# How to build and test

This project provides only the 6502 CPU emulator, to be integrated to
another emulators, such as a NES emulator, Commodore C64, etc. However, a
very simple test code is provided. The emulator can be built with the
following commands:

```sh
cd src/
make
```

The assembly test code is embedded in the source code. To test the
emulator, just run the generated binary:

```sh
./cpu6502
```

The program will output each instruction being executed along with some
registers and flag values:

```text
BRK | A:0x00	X:0x00	0x0f00:	0x00 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:0
LDA | A:0x04	X:0x00	0x0f00:	0x00 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:0
LDX | A:0x04	X:0x02	0x0f00:	0x00 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:0
STA | A:0x04	X:0x02	0x0f00:	0x04 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:0
INC | A:0x04	X:0x02	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:0
LDA | A:0x00	X:0x02	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:1|C:0
LDA | A:0x06	X:0x02	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:0
STA | A:0x06	X:0x02	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:0
LDA | A:0xfe	X:0x02	0x0f00:	0x05 N:1|V:0|res:1|B:1|D:0|I:1|Z:0|C:0
ADC | A:0x04	X:0x02	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
TAX | A:0x04	X:0x04	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
LDA | A:0xd3	X:0x04	0x0f00:	0x05 N:1|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
STA | A:0xd3	X:0x04	0x0f00:	0x05 N:1|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
LDA | A:0x0d	X:0x04	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
ADC | A:0xe1	X:0x04	0x0f00:	0x05 N:1|V:0|res:1|B:1|D:0|I:1|Z:0|C:0
TAX | A:0xe1	X:0xe1	0x0f00:	0x05 N:1|V:0|res:1|B:1|D:0|I:1|Z:0|C:0
SEC | A:0xe1	X:0xe1	0x0f00:	0x05 N:1|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
LDA | A:0x06	X:0xe1	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
STA | A:0x06	X:0xe1	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
LDA | A:0xfe	X:0xe1	0x0f00:	0x05 N:1|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
ADC | A:0x05	X:0xe1	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
JMP | A:0x05	X:0xe1	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
JMP | A:0x05	X:0xe1	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
JMP | A:0x05	X:0xe1	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
JMP | A:0x05	X:0xe1	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
JMP | A:0x05	X:0xe1	0x0f00:	0x05 N:0|V:0|res:1|B:1|D:0|I:1|Z:0|C:1
```

# How to integrate the CPU emulator

The code is developed to be high portable to other emulators and/or
architectures. Therefore some high level functions must be provided:

**RAM memory**

- *sbus_read(address)*: Read 1 byte from memory address
- *sbus_write(address, value)*: Writes 1 byte (value) to memory address
- *sbus_init()*: To initialize system bus
- *sbus_destroy()*: To finalize system bus

Functions related to RAM should be defined along with **SBUS_MEM**. If they are
not provided, a single vector **cpu_mem[65536]** will be declared and will store
CPU's RAM memory.

To make the CPU execution thread safe, **CPU_THREAD_SAFE** must be defined
and the following functions implemented:

- *s_init()*: Initialize semaphores
- *s_destroy()*: Destroy semaphores
- *s_lock()*: Semaphore UP
- *s_unlock()*: Semaphore DOWN

A standard implementation is already provided for POSIX environments.

The following defines can be used for debugging:

- **CPU_PRINT_INSTRUCTION** : Print each instruction under execution
- **CPU_TEST_CODE** : Sample main() function with a test code

For example, consider a NES emulator composed of three source files:

- *nes.c*: The NES emulator source code
- *sbus.c*: System Bus (*sbus*) implemented functions for the CPU emulator
- *cpu650x.c*: The MOS650x CPU emulator

The source code must be built with commands like the following:

```sh
gcc -DSBUS_MEM -DCPU_THREAD_SAFE nes.c sbus.c cpu650x.c -lpthread -lrt -o nes-emu
```

A minimal (sample) implementation can be found at [example](./example) directory:

```sh
cd example
make
./sample
```

# API overview

This section presents a generic definition of the API implemented by the
MOS 6502 CPU emulator. For more details, see the header file
[cpu650x.h](src/cpu650x.h).

## cpu_init()

Initialize CPU's internal structures.

**RETURN**

- 0 on success, -1 otherwise.

## cpu_destroy()

Finalize CPU's internal structures.

**RETURN**

- 0 on success, -1 otherwise.

## cpu_suspend()

Suspend CPU, so *cpu_clock()* will have no effect.

**RETURN**

- 0 on success, -1 otherwise.

## cpu_wakeup()

Wake up CPU, so *cpu_clock()* will work normally.

**RETURN**

- 0 on success, -1 otherwise.

## cpu_clock()

Performs one clock tick in the CPU.

**RETURN**

- None.

## cpu_reset()

Resets CPU to initial state.

**RETURN**

- None.

## cpu_get_state()

Return the current CPU state.

**RETURN**

- *CPU_SUSPENDED*: CPU is suspended
- *CPU_REQ_SUSPEND*: CPU was requested to suspend (it will suspend)
- *CPU_RUNNING*: CPU is running

## cpu_trigger_irq()

Trigger an interrupt (maskable) in the CPU.

**RETURN**

- None.

## cpu_trigger_nmi()

Trigger a non-maskeable interrupt in the CPU.

**RETURN**

- None.

## cpu_register_kill_cb(callback_function)

Register CPU's kill callback function in the CPU module. The callback
function will be called when the CPU goes to an unrecoverable state (it's
killed).

**PARAMETERS**

- *callback\_function(cpu\_info)*: The callback function should receive on
  its arguments all relevant CPU information, it can be a struct, object,
  etc.

**RETURN**

- 0 on success, -1 otherwise.

## cpu_register_debug_cb(callback_function)

Register function callback in the CPU module. The callback function will be
called on every instruction execution.

**PARAMETERS**

- *callback_function(debug_info)*: The callback function should receive
  on its arguments all relevant debug information, it can be a struct,
  object, etc.

**RETURN**

- 0 on success, -1 otherwise.

## cpu_unregister_debug_cb()

Unregister callback function registered with *cpu_register_debug_cb()*.

**PARAMETERS**

- None.

**RETURN**

- 0 on success, -1 otherwise.

# Source code documentation

An API documentation can be generated from the sources using the doxygen
tool:

```sh
cd docs
make
```

The output documentation will be generated in HTML and LaTeX formats under
the *dist* directory.
