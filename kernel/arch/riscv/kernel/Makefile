C_ := $(wildcard *.c)
S_ := $(wildcard *.S)

OBJ_C := $(C_:.c=.o)
OBJ_S := $(S_:.S=.o)
OBJ := $(OBJ_C) $(OBJ_S)

.PHONY: all
all: $(OBJ)

%.o: %.c
	$(GCC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -c $<

%.o: %.S
	$(GCC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -c $<


.PHONY: clean
clean:
	rm -f *.o