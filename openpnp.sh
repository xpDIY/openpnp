#!/bin/bash

platform='unknown'
unamestr=`uname`
case "$unamestr" in
	Linux)
		platform='linux'
		rootdir="$(dirname $(readlink -f $0))"
	;;
	Darwin)
		platform='mac'
		rootdir="$(cd $(dirname $0); pwd -P)"
	;;
esac

# Set JAVA_CMD to use Homebrew's OpenJDK if available, otherwise use system java
if [ -f "/opt/homebrew/opt/openjdk/bin/java" ]; then
	JAVA_CMD="/opt/homebrew/opt/openjdk/bin/java"
elif [ -f "/usr/local/opt/openjdk/bin/java" ]; then
	JAVA_CMD="/usr/local/opt/openjdk/bin/java"
else
	JAVA_CMD="java"
fi

case "$platform" in
	mac)
		$JAVA_CMD -Xdock:name=OpenPnP --add-opens=java.base/java.lang=ALL-UNNAMED --add-opens=java.desktop/java.awt=ALL-UNNAMED --add-opens=java.desktop/java.awt.color=ALL-UNNAMED -jar $rootdir/target/openpnp-gui-0.0.1-alpha-SNAPSHOT.jar
	;;
	linux)
		$JAVA_CMD $1 --add-opens=java.base/java.lang=ALL-UNNAMED --add-opens=java.desktop/java.awt=ALL-UNNAMED --add-opens=java.desktop/java.awt.color=ALL-UNNAMED -jar $rootdir/target/openpnp-gui-0.0.1-alpha-SNAPSHOT.jar
	;;
esac
