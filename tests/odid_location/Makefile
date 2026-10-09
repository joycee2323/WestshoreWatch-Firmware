# Host build of the Location decode / relay re-encode tests (gcc, no ESP-IDF).
#   make -C tests/odid_location run SRC=../../WSW-Firmware/main
SRC   ?= ../../WSW-Firmware/main
ODID  ?= ../third_party/opendroneid
CC    ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -O1 -g

# legacy_1_3/: the 1.3-westshore decoder + relay encoders (reference only),
# built as its own object with its public symbols renamed.
LEGACY_RENAME = -Dodid_parse_message=l13_odid_parse_message -Dodid_parse_pack=l13_odid_parse_pack

legacy_1_3.o: legacy_1_3/legacy_1_3.c legacy_1_3/odid_decoder.c legacy_1_3/odid_decoder.h
	$(CC) $(CFLAGS) $(LEGACY_RENAME) -Ilegacy_1_3 -Istubs -c -o legacy_1_3_dec.o legacy_1_3/odid_decoder.c
	$(CC) $(CFLAGS) $(LEGACY_RENAME) -Ilegacy_1_3 -Istubs -c -o legacy_1_3_rel.o legacy_1_3/legacy_1_3.c
	ld -r -o $@ legacy_1_3_dec.o legacy_1_3_rel.o

test_odid_location: test_odid_location.c legacy_1_3.o $(SRC)/odid_decoder.c $(SRC)/odid_encoder.c $(SRC)/odid_decoder.h $(SRC)/odid_encoder.h
	$(CC) $(CFLAGS) -I$(SRC) -Istubs -I$(ODID) -o $@ test_odid_location.c legacy_1_3.o $(SRC)/odid_decoder.c $(SRC)/odid_encoder.c $(ODID)/opendroneid.c -lm

run: test_odid_location
	./test_odid_location

clean:
	rm -f test_odid_location *.o

.PHONY: run clean
