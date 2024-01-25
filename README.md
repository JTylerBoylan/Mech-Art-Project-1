# Mechatronic Art Project 1

## Group members
* Kate
* Natalie
* Jonathan
* Lily
* Anthony

## Description

Our project goal is to have a wave generating tank communicate with Chataigne/Pilot to play musical notes based on the peaks of the waves generated. \
The wave generator is controlled by a simple DC motor with a speed that can be set by a float OSC message. The height of the water is determined by a water depth sensor mounted on the back wall of the tank. These readings are then filtered to remove noise, and normalized to be sent as a float value between 0.0 and 1.0 over OSC. The water level OSC value is read by Chataigne which converts the value to a musical note and sent over UDP to Pilot which plays the note. We also created a visualizer browser page that changes colors based on the value as well. 

## Images

### Electronics

![Electronics](https://github.com/JTylerBoylan/Mech-Art-Project-1/blob/main/images/Electronics.HEIC?raw=true)

### Tank

![Tank](https://github.com/JTylerBoylan/Mech-Art-Project-1/blob/main/images/Tank.HEIC?raw=true)