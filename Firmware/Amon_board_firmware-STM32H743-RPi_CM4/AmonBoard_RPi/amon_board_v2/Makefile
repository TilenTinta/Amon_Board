# Spremenlivke
INCLUDES=-I ".\include"
CC=gcc # compiler
LIBS = #-lsrc
CFLAGS = -g -Wall # -g: debbug info, -wall: enable all compiler warnings
OBJS = test.o # končni pričakovani faili
all: program clean

# Vse datoteke in kombinacije ki se rabijo scompilat
program: ${OBJS}
	${CC} ${CFLAGS} ${INCLUDES} -o $@ ${OBJS} ${LIBS}
	
test.o: ./src/test.c
	${CC} ${CFLAGS} ${INCLUDES} -c $< -o $@
	
main.o: main.c
	${CC} ${CFLAGS} ${INCLUDES} -c $< -o $@

.PHONY: clean

# Brisanje .o datotek (samo za powershell - del, rm - linux)
clean:
	@echo "Brisem vse .o file"
	rm *.o 