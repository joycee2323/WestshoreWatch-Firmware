# Host build of the relay-policy unit tests.
#   make -C tests/relay_policy SRC=../../WSW-Firmware/main UNITY=/path/to/Unity/src
SRC   ?= ../../WSW-Firmware/main
UNITY ?= unity/src
CC    ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -O1 -g

test_relay_policy: test_relay_policy.c $(SRC)/relay_policy.c $(SRC)/relay_policy.h
	$(CC) $(CFLAGS) -I$(SRC) -I$(UNITY) -o $@ test_relay_policy.c $(SRC)/relay_policy.c $(UNITY)/unity.c

run: test_relay_policy
	./test_relay_policy

clean:
	rm -f test_relay_policy

.PHONY: run clean
