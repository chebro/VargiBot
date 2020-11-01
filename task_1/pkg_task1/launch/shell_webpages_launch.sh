#!/bin/bash

# Store URL in a variable
URL1="http://www.hivemq.com/demos/websocket-client/"
URL2="https://docs.google.com/spreadsheets/d/1zm6nhfMDdUijzCuldgtv2cyIp10_iwgtioJrOPhRiEk/edit"

# Print some message
echo "** Opening $URL1 in Firefox **"
echo "** Opening $URL2 in Firefox **"

# Use firefox to open the URL in a new window
firefox -new-tab -url $URL1 -new-tab -url $URL2
# firefox -new-window $URL1
# firefox -new-window $URL2