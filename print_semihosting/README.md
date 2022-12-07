1. Add this line `-specs=rdimon.specs -lc -lrdimon` to `MCU GCC Linker -> Miscellaneous -> Other flags`
2. Add `extern void initialise_monitor_handles(void);` in file which use `printf` and add this line `initialise_monitor_handles();` before `printf`.
3. Exclude file `syscall.c` from build.
4. Change debug config to `ST-Link (OpenOCD)`
5. Add this line `monitor arm semihosting enable` to Run Commands.