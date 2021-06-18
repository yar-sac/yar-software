# YAR Avionics

This workspace contains the code that powers the on-board electronics of our
rocket(s). It job is to handle reading the attached sensors, logging the data
to non-volatile storage, and transmitting it in real-time to the ground
station.

## Setup for developing

You will need:

- [arduino-cli](https://arduino.github.io/arduino-cli/)
- [just](https://github.com/casey/just)

When you have these installed, use the command `just install-libs` to make sure
the necessary Arduino libraries are installed.

## Project structure

The workspace root holds general files for managing the project, such as this
`README.md` and the `justfile` with commands.

The `avionics` folder is the arduino sketch with the main source code.
