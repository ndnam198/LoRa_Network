#!/bin/sh
git status
timestamp=$(date +"%D %T")
message=$1
git add -A
git commit -m "[$timestamp]: $message"
git push --force https://github.com/ndnam198/LoRa_Network.git master
echo "*******************************************************"
echo "Time commit: [$timestamp]"
echo "Commit message: \"$message\""