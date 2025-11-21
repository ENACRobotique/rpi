#!/usr/bin/env bash
# Script pour lancer Qt plein écran sur framebuffer, sans curseur

# Désactiver curseur TTY et économiseur d’écran
setterm -cursor off -blank 0 -powerdown 0 < /dev/tty1 > /dev/tty1 2>/dev/null || true

# Forcer Qt à utiliser linuxfb sur /dev/fb0
export QT_QPA_PLATFORM=linuxfb
export QT_QPA_FB=/dev/fb0

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
exec $SCRIPT_DIR/build/hello_qt6 -platform linuxfb

