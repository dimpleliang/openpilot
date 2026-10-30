#!/bin/bash
# Mac: pull the device's drive reports (small) into ~/Downloads/ars510_reports. Usage: pull_reports.sh [ssh host]
H=${1:-comma}
mkdir -p ~/Downloads/ars510_reports
rsync -az --timeout=30 -e "ssh -o ConnectTimeout=15" "$H:/data/ars510_reports/" ~/Downloads/ars510_reports/ && ls -t ~/Downloads/ars510_reports | head
