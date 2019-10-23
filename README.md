# !!!! IMPORTANT !!!!
DO NOT download BetterPushback using the green "Clone or Download" button
on the main repository page. Use the
[RELEASES](https://github.com/skiselkov/BetterPushbackC/releases) section, the click on "Assets" and download "BetterPushback.zip".

# About BetterPushback

This is a pushback plugin for the X-Plane 11 flight simulator.
It provides an overhead view to plan a pushback route and
accomplishes a fully automated "hands-off" pushback, letting the user
focus on aircraft startup and other pilot duties during pushback. It can
of course also tow you forward, or perform any arbitrarily complicated
pushback operation. To increase immersion, it speaks to you in a variety
of languages and accents, simulating ground staff at various places
around the world.

## Donations

To leave a voluntary donation, please follow the PayPal link below:

[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=8DN9LYD5VP4NY)

## Downloading BetterPushback

To download and use BetterPushback, simply click on "Release" at the top
download the file labeled `BetterPushback.zip` for the latest version.
The files labeled `Source Code` are *not* what you want.

## Building BetterPushback

To build BetterPushback yourself you need either a Linux or a Mac
machine. The Windows version is built through cross-compiling from Linux.
For the pre-requisites on each platform, see `qmake/build-win-lin` or
`qmake/build-mac`. Once you have everything installed, run the script
labeled `build_release` from this directory. It drives the build and
compiles everything as necessary.

For details on how to add tug liveries, see
`objects/tugs/LIVERIES_HOWTO.txt`.

To add a voice set, see `data/msgs/README.txt` for the information.
