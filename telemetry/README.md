# CATS Telemetry

<img src="https://github.com/catsystems/cats-docs/blob/main/logo/PNG/logo_with_smile.png" alt = "CATS Logo" width="300" height="300">

*Always land on your paws*

This repository contains all firmware for the telemetry microcontroller.

## Features
* Frequency Hopping Spread Spectrum
* Support for GNSS modules

## Contribution
Contributions are welcome and encouraged. You can contribute in many ways:

* Implement a new feature in the firmware
* Bug reporting & fixes
* New feature ideas & suggestions

The best way to get started is to join our [Discord](https://discord.gg/H9Caj8XeBj) server.

## Open Source
All CATS code is open source and can be used free of charge without warranty. 

## Pushing to Remote Repository
In order to ensure consistency and easier diff review between commits,
*clang-format* should be used to format all modified C/C++ files. 

The entire codebase is already formatted with *clang-format v12.0.0*.

To download `clang-format` visit this [page](https://releases.llvm.org/download.html).

To automatically format the modified code before committing, you should
copy the pre-commit hook from `./hooks` to `./.git/hooks`.

Unfortunately, it is not easily possible to check whether the code is
formatted correctly on the server side (while you are doing a push) so
_**please**_ do this if you are working with C/C++ files. 
