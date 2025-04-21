#############################################################
CC = gcc

LDFLAGS = -lpthread -lrt
CFLAGS = -Wall -DSBUS_MEM -I../src/ -I./

executable = sample
objects = cpu650x.o sample.o
#############################################################

##
# General rules
#

all: $(executable)

cpu650x.o: ../src/cpu650x.c
	$(CC) $(CFLAGS) -c $<

sample.o: sample.c
	$(CC) $(CFLAGS) -c $<

$(executable) : $(objects)
	$(CC) $(objects) $(LDFLAGS) -o $(executable)

.Makefile.dep: *.c
	@$(CC) $(CFLAGS) $(CPP_FLAGS) -MM *.c > $@

-include .Makefile.dep

##
# clean
#
.PHONY clean:
	@rm -f *.o \#* *~  .Makefile.dep
	@rm -f $(executable)

