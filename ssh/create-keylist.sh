#!/bin/bash
# create keylist for authorized_keys on car
# written by Marcel Ebbrecht <marcel.ebbrecht@googlemail.com>

# preamble
LANG=C

# execution
cat *.pub > authorized_keys
chmod 644 authorized_keys
