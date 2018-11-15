# STM32-HC-SR04
Driver for an HC-SR04 ultrasonic distance sensor for ARM-STM32 microcontroller (nucleo) using CubeHAL


/*
 * Driver for HC-SR04 ultrasonic distance sensor
 * for STM32-F401RE ARM Processor.
 * To get a reading from the sensor the Trig pin must be held high for 10us
 * the echo pin then goes high for X amount of time, which corresponds with
 * the time of flight of the ultrasonic burst. Once we have the time we divide by
 * the speed of sound and then divide by two to get the distnance from the object.
 * The code uses Output Compare to generate the 10us trigger and Input Compare to
 * determine the time taken. The IC is set in DMA mode and the Trigger timer is set
 * to fire only once (each time getReading() is called).
 * Uses pins D8 and D12 on nucleo (Arduino headers)
 * Take measurements at least 60ms apart.
 */
