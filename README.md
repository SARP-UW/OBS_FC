# Transcendent in-flight Telemetry and Avionics Nexus (“TITAN”) Flight Computer Software
- [Required Software](#required-software)
  - [MacOS (Darwin)](#macos-darwin)
  - [Windows](#windows)
- [Compiling](#compiling)
  - [Committing](#committing)

## Required Software
Note: May consider Docker/Dev Container workflow for future. Currently there are no instructions for Linux/x64 MacOS, but I would imagine the Linux one may be similar to the MacOS one.

### MacOS (Darwin)
First I'd recommend installing 3 things:
* Arduino IDE (https://www.arduino.cc/en/software/) - scroll down a tiny bit, you'll see an Arduino IDE 2.x.x version and download option (it should be MacOS Apple Silicon option already, if not, change it to that). This is not needed for devboard but useful regardless
* STM32CubeProgrammer (https://www.st.com/en/development-tools/stm32cubeprog.html#get-software) - this is pretty important for devboard, mainly just for clearing memory on devboard when needed. choose the CubePrg-Mac-Arm one.
* ArmGNUToolchain (https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) This is the main thing needed to run on devboard. Scroll down until you see the macOS header with the option of arm-gnu-toolchain-14.3.rel1-darwin-arm64-arm-none-eabi.pkg (version may be different if/when it gets updated)

do the installation processes for all 3 (should be simple enough)

specifically for armGNUToolchain - I would recommend doing this (so that you don't have to repeat this step every time you open a terminal:
* Go to finder, and go to your Users/username folder (ex: mines is Users/mahiremran)
* Press Cmd+Shift+. keys. You should see lot of "hidden" (grayed out) files. Look for the one that says ".zshrc" or ".bshrc" (depends if you use bash or zsh, generally don't worry too much about it. If you see both, then just edit both. not a big deal)
* Open the file in TextEdit (or any simple text editor)
* Add this line to the end of the file: 
```export PATH="$PATH:/Applications/ArmGNUToolchain/14.3.rel1/arm-none-eabi/bin/"```
* Save the file, and you can exit it/close finder

now after running openocd command, after running from run and debug menu on VScode with devboard plugged in you should be able to run. though I may be forgetting a step, not sure

For compiling code:
In a terminal, run 
```brew install cmake```
 (you should only need to do this once)
Also run 
```brew install open-ocd```

### Windows
Ok so since Windows doesnt have homebrew:

For compiling code:

Go to https://cmake.org/download/. I think your system is Windows x86-64, so download the Windows x64 installer (cmake-4.2.0-rc2-windows-x86_64.msi). .msi works like an executable, so just double click to run and follow installation.
* When looking up I see something about maybe having to add ```C:\Program Files\CMake\bin``` to system’s PATH env. variable. If you don’t know how to edit env variable let me know. but first try ```cmake --version``` in Powershell and see if that works.

next need to install openocd, go to https://github.com/xpack-dev-tools/openocd-xpack/releases. I would choose option xpack-openocd-0.12.0-7-win32-x64.zip, download it, unzip, and run installer and follow steps.

First I'd recommend installing 3 things:

* ⁠Arduino IDE (https://www.arduino.cc/en/software/) - scroll down a tiny bit, you'll see an Arduino IDE 2.x.x version and download option ( choose Windows Win 10 or newer option if not already selected). This is not needed for ddevboardbut useful regardless
* ⁠STM32CubeProgrammer (https://www.st.com/en/development-tools/stm32cubeprog.html\#get-software) - this is pretty important for devboard, mainly just for clearing memory on devboard when needed. choose the STM32CubePrg-W64 one.
* ⁠ArmGNUToolchain (https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) This is the main thing needed to run on devboard. Scroll down until you see the "Windows (mingw-w64-x86_64) hosted cross toolchains" header. Make sure you are under subheader AArch32 bare-metal target (arm-none-eabi) (I think) Choose first option: arm-gnu-toolchain-14.3.rel1-mingw-w64-x86_64-arm-none-eabi.zip


Do the installation processes for all 3 (should be simple enough)

* ⁠Similar to openocd/cmake, you may have to add  to PATH env variable for ArmGNUToolchain. I think it should be path of ⁠ C:\ArmGNUToolchain\14.3.rel1\arm-none-eabi\bin⁠, but may be wrong

## Compiling
The easiest way to compile will be using the provided ```./build.sh``` file. First you should make a folder in ```./src/build/``` if it does not exist already. You can input a target to make, such as ```titan```, e.g. ```./build.sh titan``` would compile the program with the main loop. Alternatively you could do ```./build.sh all``` to make all targets.
* Internally, what this does is temporarily do ```cd src/build```, then ```cmake ..```, and then ```make```ing each target. It also compiles the test_alloc executable separately. 

If you build a specific target, it will also try to flash to devboard. You may have to unplug/replug the STM32 devboard due to having to wipe flash memory.
* This is a reason I am considering Docker, as STM32 devboard has a CLI that allows for automating that, but it is not available on MacOS.

### Committing
* You should be passing the ```./build.sh commit_check``` check before committing/opening a PR.