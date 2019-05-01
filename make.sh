#!/bin/bash

# assumes this is where OF is installed:
# but apparently this isn't needed anyway
export OF_ROOT=../of_v0.10.1_release

# now build the project:

# make clean && make Debug && make run
make && make run