ASPS-DAQ firmware.

Used libraries are under libraries/. You'll probably need to move them to the
libraries directory for your Energia install.

When the uC is unprogrammed, it's in bootloader mode, so you can just use LM Flash Programmer using the serial
connection and it will program OK. After it's programmed, give it a MAC address (don't screw up! It's a one-time
program, for the most part). After that it can connect to the network, and the Ethernet bootloader will work.

If you screw up the Ethernet bootloader somehow, recovery is still possible using the GPIOs of the CP2104, however,
it's not fun.

1) Get the AN223 software here: http://www.silabs.com/documents/public/example-code/AN223SW.zip
2) Open up CP210x Port Read/Write Example
3) Select the serial port for the Tiva (make sure it's not open elsewhere!).
4) Check off "Pins to Change" for GPIO 3. GPIO 3 State (in the Write Latch section) should be 0.
5) Click "Write Latch." GPIO 3 State (in the Read Latch section) should now be 0.
6) Uncheck GPIO3, check GPIO2. GPIO 2 State (in the Write Latch section) should be 0.
7) Click "Write Latch." GPIO 2 State (in the Read Latch section) should now be 0.
8) Click on the 0 in GPIO 2 State (in the Write Latch section). It should change to 1.
9) Click on "Write Latch." GPIO 2 State (in the Read Latch section) should now be 1.
10) Click on the 0 in GPIO 3 State (in the Write Latch section). It should change to 1.
11) Uncheck GPIO 2. Check GPIO 3.
12) Click on "Write Latch." GPIO 3 State (in the Read Latch section) should now be 1.
13) Close CP210x Port Read/Write Example.

The Tiva is now in bootloader mode on the serial port, and can be programmed. 
