
<img src="images/Full Assembly v10.gif" alt="drawing" width="500"/>

# Design

<img src="images/Control Diagram.svg" alt="drawing" width="1000"/>

Symbol  | Meaning | Source
------------- | ------------- | -------------
$\Psi_{sp}$ | Odometry Set Point | Input by User (Controller)
$\Psi_{0}$  | Current Odometry Reading | Encoder Reading
$\Theta_{b}$ | Balanced Pitch | Measured
$\Theta_{sp}$ | Target Pitch (from balanced) | Calculated
$\Theta_{0}$ | Current Pitch | IMU Reading
$\Phi_{c}$ | Yaw Adjustment | User Input

# Assembly

## Bottom Assembly

Components  | #
------------- | -------------
Motor Mounts | 2
M2.5 threaded inserts  | 4
Soldering Gun | 1

Use the heated soldering gun to press the threaded inserts into the holes on the bottom of the motor mount.

<img src="images/1.jpg" alt="drawing" width="300"/>

Components  | #
------------- | -------------
Motor | 2
Motor Mounts  | 2
M2.5 x 6mm Nylock Screws | 2

Gather all the components and assemble as shown in the images below.

<img src="images/2.jpg" alt="drawing" width="300"/> 
<img src="images/3.jpg" alt="drawing" width="300"/>






Components  | #
------------- | -------------
Bottom Plate  | 1
2.5" standoffs  | 4
M2.5 x 8mm Screws | 4

Gather all the components and assemble as shown in the images below.

<img src="images/4.jpg" alt="drawing" width="300"/> 
<img src="images/5.jpg" alt="drawing" width="300"/>


Components  | #
------------- | -------------
Scooter Wheels 84 X 24 mm | 2
M3x20mm screws | 4
Wheel adapter | 2
Set screws | 2
Hex Key | 1
Motor and Mount | 2

<img src="images/6.jpg" alt="drawing" width="300"/> 

Follow the steps below:

1. First, press fit the machined wheel adaptor into one side of the wheels, ensuring a part of it sticks out.
2. Then press fit the flat metal piece into the opposite side, aligning the three holes.
3. Insert the M3x20mm screws to secure the parts together, as demonstrated in the second image.
4. Using the 2mm Hex wrench screw the set screws that come with the wheel adaptor kit into the wheel adaptors as shown in the 3rd and 4th image.

<img src="images/7.jpg" alt="drawing" width="500"/> 

5. Then you can directly put the wheel on, use the hex key to tight the wheels. Note that it is necessary to leave a bit of space between the wheel and the screw head of the motor, as depicted in the image below, otherwise the wheel hub will rub against the screws holding the motor causing excess friction or jamming the motors.

6. Mount the Wheels to the Bottom plate

<img src="images/8.jpg" alt="drawing" width="300"/> 

Components  | #
------------- | -------------
Bottom Assembly | 1
Raspberry Pi | 1
M2.5 x 16mm Screws | 2



Mount the raspberry Pi to the middle of the bottom plate.

<img src="images/9.jpg" alt="drawing" width="300"/>  
<img src="images/10.jpg" alt="drawing" width="300"/> 

## Middle Assembly

Components  | #
------------- | -------------
Robotics Control Board | 1
MBot PICO+ | 1
Jumper Cap (Shorting block)| 1

Put the jumper cap on. Notice that in the 1st image, there are 3 pins squared together. Since we are using 12V, make sure to position the jumper cap over the VM and 12V pins, as demonstrated in the 2nd image.

<img src="images/11.jpg" alt="drawing" width="300"/> <img src="images/1.jpg" alt="drawing" width="300"/>

Push the PICO+ into its mounts on the Control Board.

<img src="images/12.jpg" alt="drawing" width="300"/>

Components  | #
------------- | -------------
Middle Plate | 1
M2.5 8mm Nylon Standoffs | 4
M2.5 x 6mm Screws | 4

Gather all the components and assemble as shown in the images below.

<img src="images/13.jpg" alt="drawing" width="300"/>
<img src="images/14.jpg" alt="drawing" width="300"/>

Components  | #
------------- | -------------
Middle Plate | 1
M2.5 x 6mm Screws | 4
Pico and Cape | 1

Mount the Cape to the middle of the Middle Plate.

<img src="images/15.jpg" alt="drawing" width="300"/> 
<img src="images/16.jpg" alt="drawing" width="300"/>

Components  | #
------------- | -------------
Middle Plate | 1
1.5” Aluminum standoffs | 4
M2.5 x 8mm Screws | 4

Mount the aluminum standoffs on the side, one hole down from the corner.

<img src="images/17.jpg" alt="drawing" width="300"/> 
<img src="images/18.jpg" alt="drawing" width="300"/>

## Top Plate Assembly

Components  | #
------------- | -------------
Top Plate | 1
Zip Ties | 4
Battery Clips | 3

Tie the Battery clips to the top plate. One side will have 2 stacked, the other just one.

<img src="images/19.jpg" alt="drawing" width="300"/> 
<img src="images/20.jpg" alt="drawing" width="300"/>

Components  | #
------------- | -------------
Top Plate | 1
Battery | 1
Velcro | 1

Gather all the components and assemble as shown in the images below.

<img src="images/21.jpg" alt="drawing" width="300"/> 
<img src="images/22.jpg" alt="drawing" width="300"/>

## Full Assembly
Components  | #
------------- | -------------
Bottom Assembly | 1
Middle Assembly | 1
Top Assembly | 1
4-40 thumb screw 3/8” | 8

Now we mount the Bottom Assembly to the Middle Assembly, and the Middle Assembly to the Top Assembly. Note that **assembly direction matters.** The bottom assembly must be mounted to the middle assembly in such a way that the Motor cables can reach through the slot on the middle plate. The middle plate must be mounted to the top plate in such a way that the ports on the battery are on the same side as the ports of the cape. Also you might find it easier to attach the motor cables before you mount the top assembly. (See next step)

<img src="images/23.jpg" alt="drawing" width="300"/> 
<img src="images/24.jpg" alt="drawing" width="300"/>
<img src="images/25.jpg" alt="drawing" width="300"/>

Components  | #
------------- | -------------
Full Assembly | 1
DC power “Y” cable | 1
9” USB-C Cable| 2
Motor Cable | 2

Finally we connect all the cables and power:

1. Use the “Y” Cable to connect the power bank and the Cape
2. Use the USB-C to connect the power bank and the Pi
3. Use the USB-C Cable to connect the Robotics Control Board and Pi
   

<img src="images/26.jpg" alt="drawing" width="300"/>

4. Connect the motor cables to the control board. Note that **you will short the cape if you do it backwards.** If you are using the rainbow cables it connects like below:

Wire Color  | Signal
------------- | -------------
Red | Encoder B
Orange | Encoder A
Yellow| 3.3 V
Green | GND
Blue | Motor -
Black | Motor +

# Using the Code

After building upload the mbot.uf2 file to the pico while it is in bootsel mode.

To use the controller with the balance bot, connect the controller via bluetooth to the Pi, and run:

```
python ./python/balance_test_drive.py
```
Control  | Outcome
------------- | -------------
Left Joystick Up-Down | Robot moves forward and back
Right Joystick Left-Right | Robot moves left or right
B| Robot stops
A | Robot restarts

To use the PID Tuner run:

```
python ./python/Controller_Tuner.py
```
| Control  | Outcome |
| ------------- | ------------- |
| + | += .1 gain |
| - | -= .1 gain |
| * | *= 1.1 gain |
| / | /= 1.1 gain |
