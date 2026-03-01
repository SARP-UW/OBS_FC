#!/usr/bin/env bash
set -e
set -o pipefail

FW_TARGET="${1:-${FW_TARGET:-titan}}"
FW_TARGETS=(titan test_pwm test_spi test_usart test_oscilloscope)

case "$FW_TARGET" in
  titan|test_pwm|test_spi|test_usart|test_oscilloscope|commit_check|all|clean|docs)
    ;;
  *)
    echo "Unknown target: $FW_TARGET"
    echo "Valid targets: titan, test_pwm, test_spi, test_usart, test_oscilloscope, commit_check, all, clean, docs"
    exit 4
    ;;
esac

if [ "$FW_TARGET" = "docs" ]; then
  if ! command -v doxygen &> /dev/null; then
    echo "Error: doxygen is not installed. Install with: brew install doxygen"
    exit 5
  fi
  echo "Generating Doxygen documentation..."
  doxygen Doxyfile
  echo "Docs generated in docs/html/"
  exit 0
fi

if [ "$FW_TARGET" = "clean" ]; then
  echo "Cleaning build/ contents..."
  mkdir -p build
  find build -mindepth 1 -maxdepth 1 -exec rm -rf {} +
  echo "Clean complete."
  exit 0
fi

if [ "$FW_TARGET" = "commit_check" ]; then
  echo "Running local pre-commit check:"
  cmake -S . -B build || { echo "cmake configure failed"; exit 20; }

  cd build/ || exit 22
  for target in "${FW_TARGETS[@]}"; do
    echo "Building ${target}.elf"
    make "${target}.elf" || { echo "make failed for ${target}.elf"; exit 21; }
  done
  make test_alloc_target || { echo "make test_alloc failed"; exit 21; }
  echo "Built target test_alloc"
  ctest --output-on-failure || { echo "Unit tests failed"; exit 21; }
  cd ../
  echo "Local CMake check passed. Good to go for committing! :D"
  exit 0
fi

if [ "$FW_TARGET" = "all" ]; then
  echo "Running full build (make all):"
  cmake -S . -B build || { echo "cmake configure failed"; exit 20; }

  cd build/ || exit 22
  make all || { echo "make all failed"; exit 21; }
  echo "Built target test_alloc"
  ctest --output-on-failure || { echo "Unit tests failed"; exit 21; }
  cd ../

  echo "Full build passed."
  exit 0
fi

OPENOCD_CMD=(openocd -f interface/stlink.cfg -f target/stm32h7x_dual_bank.cfg -c "program ${FW_TARGET}.elf verify reset exit")

# Use OS to detect the binary
OS_TYPE=$(uname -s)
case "$OS_TYPE" in
  Darwin)
    USB_RESET_BIN="./stm_usb/stm_usb_mac"
    ;;
  Linux)
    USB_RESET_BIN="./stm_usb/stm_usb_linux"
    ;;
  MINGW*|MSYS*|CYGWIN*|Windows_NT)
    USB_RESET_BIN="./stm_usb/stm_usb_win.exe"
    ;;
  *)
    echo "Unsupported OS: $OS_TYPE"
    exit 1
    ;;
esac


# Make sure file was found
if [ ! -f "$USB_RESET_BIN" ]; then
  echo "USB reset binary not found: $USB_RESET_BIN"
  exit 2
fi

run_usb_reset() {
  echo "Running USB reset ($USB_RESET_BIN)..."
  "$USB_RESET_BIN"
  USB_RC=$?
  if [ $USB_RC -ne 0 ]; then
    echo "USB reset failed (rc=$USB_RC). Aborting."
    exit 3
  fi
}

# initial USB reset
run_usb_reset

# Building project
mkdir -p "build/"
cd "build/"

echo "Running cmake .."
cmake .. || { echo "cmake failed"; exit 20; }

echo "Running make"
make "${FW_TARGET}.elf" || { echo "make failed"; exit 21; }

# running openocd
MAX_ATTEMPTS=3
attempt_count=0

while [ $attempt_count -lt $MAX_ATTEMPTS ]; do
  attempt_count=$((attempt_count+1))
  echo "Flashing with OpenOCD (attempt $attempt_count/$MAX_ATTEMPTS)..."
  OPENOCD_OUTPUT_FILE="$(mktemp)"
  # Capture both stdout and stderr into the file
  if "${OPENOCD_CMD[@]}" >"$OPENOCD_OUTPUT_FILE" 2>&1; then
    OPENOCD_RC=0
  else
    OPENOCD_RC=$?
  fi

  # Normalize output
  output="$(
    tr -d '\r' <"$OPENOCD_OUTPUT_FILE" \
    | sed 's/\x1b\[[0-9;]*m//g'
  )"

  # Timeout waiting for algorithm
  if printf '%s\n' "$output" | grep -iq -- "timeout waiting for algorithm"; then
    echo "Timeout waiting for algorithm. Attempting automated recovery..."

    case "$OS_TYPE" in
      # no CLI on mac
      Darwin)
        echo "Open STM32CubeProgrammer app, connect board, do full chip erase,"
        echo "then disconnect the board and press Enter to continue."
        read -r -p "Press Enter after you have completed the full chip erase. "
        # re-run USB reset to re-enumerate device
        cd ../ 
        run_usb_reset
        cd build/
        rm -f "$OPENOCD_OUTPUT_FILE"
        continue
        ;;
      Linux)
        # linux - find CLI
        CLI_BIN="$(command -v STM32_Programmer_CLI || true)"
        # TODO: unsure if this is right path for CLI in linux
        [ -z "$CLI_BIN" ] && CLI_BIN="/opt/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
        ;;
      MINGW*|MSYS*|CYGWIN*|Windows_NT)
        # Windows - find CLI
        CLI_BIN="$(command -v STM32_Programmer_CLI.exe || true)"
        # TODO: unsure if this is right path for CLI in windows
        [ -z "$CLI_BIN" ] && CLI_BIN="/c/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe"
        ;;
      *)
        echo "Unsupported OS for automated CLI erase: $OS_TYPE"
        rm -f "$OPENOCD_OUTPUT_FILE"
        break
        ;;
    esac
    
    # run CLI (windows/linux only)
    if [[ "$OS_TYPE" != "Darwin" ]]; then
      if [ ! -x "$CLI_BIN" ] && ! command -v "$CLI_BIN" >/dev/null 2>&1; then
        echo "STM32_Programmer_CLI not found at: $CLI_BIN"
        echo "Please install STM32CubeProgrammer CLI or add it to PATH, then retry."
        rm -f "$OPENOCD_OUTPUT_FILE"
        break
      fi

      echo "Found STM32 CLI: $CLI_BIN"
      echo "Attempting connect -> usb replug -> full chip erase -> disconnect"
      # connect with devboard
      "$CLI_BIN" -c port=SWD -d 0
      rc=$?; if [ $rc -ne 0 ]; then echo "CLI connect failed (rc=$rc)"; rm -f "$OPENOCD_OUTPUT_FILE"; break; fi
      # usb replug
      "$CLI_BIN" -c port=SWD -d 0 -srst_only
      rc=$?; if [ $rc -ne 0 ]; then echo "CLI soft-reset failed (rc=$rc)"; "$CLI_BIN" -c port=SWD -d 0 -dis >/dev/null 2>&1 || true; rm -f "$OPENOCD_OUTPUT_FILE"; break; fi
      # full chip erase
      "$CLI_BIN" -c port=SWD -d 0 -e all
      rc=$?; if [ $rc -ne 0 ]; then echo "CLI erase failed (rc=$rc)"; "$CLI_BIN" -c port=SWD -d 0 -dis >/dev/null 2>&1 || true; rm -f "$OPENOCD_OUTPUT_FILE"; break; fi
      # disconnect
      "$CLI_BIN" -c port=SWD -d 0 -dis
      rc=$?; if [ $rc -ne 0 ]; then echo "CLI disconnect returned rc=$rc (nonfatal)"; fi

      cd ../ 
      run_usb_reset
      cd build/
      rm -f "$OPENOCD_OUTPUT_FILE"
      echo "Recovery via STM32CubeProgrammer CLI complete — retrying OpenOCD flash..."
      continue
    fi
  fi

  # Checksum mismatch
  if printf '%s\n' "$output" | grep -iq -- "checksum mismatch"; then
    echo "OpenOCD reported checksum mismatch (attempt $checksum_fail_count/$MAX_CHECKSUM_RETRIES)."
    cd ../
    echo "Running USB reset"
    run_usb_reset
    cd build/
    rm -f "$OPENOCD_OUTPUT_FILE"
    continue
  fi

  rm -f "$OPENOCD_OUTPUT_FILE"

  # Success
  if [ $OPENOCD_RC -eq 0 ]; then
    echo "Flash successful."
    break
  fi

done

if [ $attempt_count -ge $MAX_ATTEMPTS ]; then
  echo "OpenOCD did not succeed after $MAX_ATTEMPTS attempts."
  echo "Other issue likely occurred."
  exit 32
fi