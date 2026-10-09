# Host build of the Location decode / relay re-encode tests (gcc, no ESP-IDF).
#   make -C tests/odid_location run SRC=../../WSW-Firmware/main
SRC   ?= ../../WSW-Firmware/main
ODID  ?= ../third_party/opendroneid
CC    ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -O1 -g

test_odid_location: test_odid_location.c $(SRC)/odid_decoder.c $(SRC)/odid_encoder.c $(SRC)/odid_decoder.h $(SRC)/odid_encoder.h
	$(CC) $(CFLAGS) -I$(SRC) -Istubs -I$(ODID) -o $@ test_odid_location.c $(SRC)/odid_decoder.c $(SRC)/odid_encoder.c $(ODID)/opendroneid.c -lm

run: test_odid_location
	./test_odid_location

clean:
	rm -f test_odid_location

.PHONY: run clean
