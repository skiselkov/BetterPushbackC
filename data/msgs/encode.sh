#!/bin/bash

# CDDL HEADER START
#
# This file and its contents are supplied under the terms of the
# Common Development and Distribution License ("CDDL"), version 1.0.
# You may only use this file in accordance with the terms of version
# 1.0 of the CDDL.
#
# A full copy of the text of the CDDL should have accompanied this
# source.  A copy of the CDDL is also available via the Internet at
# http://www.illumos.org/license/CDDL.
#
# CDDL HEADER END

# Copyright 2017 Saso Kiselkov. All rights reserved.

# Takes a bunch of WAV files in the current directory and encodes them
# using our radio OPUS settings.

for FILE in *.wav; do
	opusenc --bitrate 54 --framesize 60 "$FILE" "${FILE/%.wav/.opus}"
done

for FILE in *.flac; do
	opusenc --bitrate 54 --framesize 60 "$FILE" "${FILE/%.flac/.opus}"
done
