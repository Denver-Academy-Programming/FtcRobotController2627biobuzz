#!/bin/zsh
# Build and install the robot app onto the Control Hub.
#
#   ./upload.sh starter   only StarterBot OpModes
#   ./upload.sh full      only FullBot OpModes
#   ./upload.sh all       both (default)
#
# First join the Control Hub's Wi-Fi (your Mac's IP should start with 192.168.43.), or plug in USB.
set -e

BOT="${1:-all}"
case "$BOT" in
  starter|full|all) ;;
  *) echo "usage: ./upload.sh [starter|full|all]"; exit 1 ;;
esac

cd "$(dirname "$0")"
export JAVA_HOME=$(/usr/libexec/java_home -v 17)
export ANDROID_HOME=/opt/homebrew/share/android-commandlinetools
ADB=/opt/homebrew/bin/adb
HUB=192.168.43.1:5555

online() { "$ADB" devices | awk 'NR>1 && $2=="device"' | grep -q . }

if ! online; then
  echo "No device online, connecting to $HUB over Wi-Fi..."
  # A stale "offline" entry makes adb say "already connected" and the install fails, so start clean.
  "$ADB" disconnect >/dev/null 2>&1 || true
  "$ADB" kill-server >/dev/null 2>&1 || true
  "$ADB" start-server >/dev/null
  "$ADB" connect "$HUB" || true
  sleep 1
fi

"$ADB" devices
if ! online; then
  echo "Control Hub not found. Are you on its Wi-Fi (IP 192.168.43.x) or plugged in over USB?"
  exit 1
fi

echo "Installing bot=$BOT ..."
./gradlew :TeamCode:installDebug -Pbot="$BOT"
echo "Done: installed $BOT. Restart the Robot on the Driver Station if the OpMode list didn't refresh."
