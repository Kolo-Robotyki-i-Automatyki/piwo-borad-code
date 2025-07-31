This software was made for the Power Management Board (custom board) for the student martian rover called "Crower". The board was capable of:
- switching power on-off on relay magnetic switches
- Communicating with the onboard computer through CAN
- Measuring voltage on battery cells
- Measuring the current flowing, through I2C communication with high precision current sensor

Implementation:
The board was coded in CubeIDE as it has the most packages for STM boards. The code includes all features above. The on-off switching is setting HIGH-LOW on corresponding pins of the relay switches.

The board measures grouped voltage from cells: (1), (1 and 2), (1, 2 and 3), …, (1, 2, 3, 4, 5 and 6), which are scaled by 2k and 3k Ohms resistors by the corresponding factor of 2/(3*i), so: (2/3), (2/6), …, (2/18). After the measurement we want to pass the data through CAN, so we don’t need to convert it to decimal on the microcontroller, as it would be larger in size of data. We do that on the onboard rover computer. 

The I2C communication with the current flow sensor is implemented through sending the resistor and precision values on the initialisation. Then in the program loop there is a question to the sensor for the data, made by the microcontroller, which then receives the data. The exact current is once again calculated on the onboard computer, as there are no advantages for calculating it on the microcontroller.

The CAN communication has a message size of 8 bytes. Therefore we need to optimise the data, that is sent. The current flow has the size of 2 bytes. To minimise the overall bitrate through CAN (to many message could halt some data), the telemetry is send only in one message. Therefore I decreased the voltage measurement on the microcontroller itself to return the data in 8 bits, so that I can take the leftover 6 byte slots for voltage. The implementation of CAN required writing additional interrupts (in the files other, then main.c) so that the program redirects to a specified function and executes the shutdown or switching on-off the 24V high-power section.
