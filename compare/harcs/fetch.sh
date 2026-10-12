#!/bin/sh
# Fetch HARCS-C (public domain, by Matt DiPalma) at a pinned commit.
set -eu
cd "$(dirname "$0")"
REV=${HARCS_REV:-$(cat REV)}
[ -d vendor/.git ] || git clone -q https://github.com/xmdi/HARCS-C vendor
git -C vendor fetch -q origin && git -C vendor checkout -q "$REV"
# Upstream bug: applySequence masks moves with the int expression 63<<(6*m),
# which overflows from the 6th move on, so sequences longer than 6 moves
# (e.g. the depth-7 F2L table) are applied wrongly. Use a 64-bit mask.
sed -i.orig 's/(sequence&(63<<(6\*m)))>>(6\*m)/(sequence\&(63ULL<<(6*m)))>>(6*m)/g' vendor/main.c
grep -q '63ULL<<(6\*m)))>>(6\*m)' vendor/main.c
echo "HARCS-C at $(git -C vendor rev-parse --short HEAD)"
