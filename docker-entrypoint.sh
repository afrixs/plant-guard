#!/bin/bash

sudo ip link set lo multicast on

# Execute the command passed into this entrypoint
exec "$@"