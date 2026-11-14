first: join the Control Hub's Wi-Fi (Mac address should start with 192.168.43.)

connect:  /opt/homebrew/bin/adb disconnect
/opt/homebrew/bin/adb connect 192.168.43.1:5555
/opt/homebrew/bin/adb devices      <- must say "device", not "offline"

upload:  export JAVA_HOME=$(/usr/libexec/java_home -v 17)
export ANDROID_HOME=/opt/homebrew/share/android-commandlinetools
./gradlew :TeamCode:installDebug

to restart: adb kill-server
adb start-server
