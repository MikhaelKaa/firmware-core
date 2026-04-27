target extended-remote :3333
set print pretty on
set pagination off

# Load symbols
file build/firmware.elf

# Reset and halt
monitor reset halt

# Set breakpoint at usb_handle_setup
break usb_handle_setup

# Continue
continue
