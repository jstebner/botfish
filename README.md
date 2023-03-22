# Botfish

An integration of the UM-D R2ED Humanoid Robot designed to solve the N-Queens problem and place the pieces accordingly on a chess board. The goal of this project was to research manipulation of the Humanoid and potentially lay the groundwork for a future group to take our research and make a full fledged autonomous chess implementation.

### Dependencies

* REVIEW: Requires Python 3.10 on Ubunutu 22 
* Packages listed in botfish/requirements.txt

### Installing

```
git clone https://github.com/jstebner/botfish.git
cd botfish
py 3.10 -m pip install -r requirements
```

### Usage

```
# source your workspace
# build workspace


# run this for debug terminal and display
ros2 run botfish_debug terminal
```


## N-Queens Solver

A8 is always assumed to be the tile furthest from the arm's home position (ie the "queen loader")

| Row| A | B | C | D| E | F | G | H | Q |
| - | - | - | - | - | - | - | - | - | - |
| **1**| A1 | B1| C1| D1| E1| F1| G1| H1| QL|
| **2**| A2 | B2| C2| D2| E2| F2| G2| H2|   |
| **3**| A3 | B3| C3| D3| E3| F3| G3| H3|   |
| **4**| A4 | B4| C4| D4| E4| F4| G4| H4|   |
| **5**| A5 | B5| C5| D5| E5| F5| G5| H5|   |
| **6**| A6 | B6| C6| D6| E6| F6| G6| H6|   |
| **7**| A7 | B7| C7| D7| E7| F7| G7| H7|   |
| **8**| A8 | B8| C8| D8| E8| F8| G8| H8|   |

Moveit will plan to "optimal paths"

## Modules Needed to Implement Chess

>Vision

The entire vision component must be written as we were unable to make much headway in that front. Adding a vision component and gaining distance to the board positions also decreases the need for adjustments when running physical calibrations (see Known Issues for more)

>Orientation Constraints for Path Planning

A code snippet can be found in ```botfish_manipulation/src/manipulation_node.cpp``` 

> Manipulation

The queen loader needs to be changed to a position away from the board. When a new move string is passed we must go to that tile first and grab that piece, then the current implementation (drop off piece in cell, go to position away from board) can be used. An extra step must be performed for capturing pieces


>(Re)Interfacing the Engine Node with Stockfish

Rather than gaining move strings from the N-Queens node, moves need to be gained from Stockfish



## Known Issues
### Calibration
Due to a lack of a vision component it is impossible to completely calibrate the system within our desired specifications (moving pieces within 1.25 CM of the center of a tile). While the schunk arms do generally have good repeatability it is humanly impossible to find the exact positions to line up all of the tiles.

### Grippers and Fingers Die After Emergency Stop
Secure shell into the onboard PC ```ssh humanoid@192.168.131.111``` and use the password. Use ```sudo systemctl restart humanoid``` to restart the humanoid service. This has the added benefit of rerunning the hand initialization sequence without needing to completely power cycle the bot. In the event that the initialization doesn't work a full power cycle is necessary. Use ```sudo systemctl poweroff``` and wait for the onboard PC's light to fade. Then turn off the power button on the Ridgeback's base

### Left Thumb Doesn't Move 
Unfortunately the left thumb aboard R2ED is dead and as a result cannot respond to published messages. Onboard the Intel Nuc we've disabled the thumb flexion and thumb opposition joints in the corresponding array, positions ```[0,1]```. Should the thumb come back online the array can be found in ```humanoid/humanoid_control/launch/robot.launch.py```

### Right Thumb Doesn't move
Similarly to the left thumb, the right thumb is disabled to allow the initialization sequence to properly run. Note that only the right thumb flexion is disabled

### Right Index Finger Mechanical Issues
The motor for the right index finger works properly and is able to move it but we believe the finger has a busted spring and as a result has little resistance to movements and can dangle in place.

### Left Arm Plans to a Low Position and not the Desired One
We encountered this issues a few times while experimenting with the left arm. It only seemed to occur while power on for the first time after having not been moved for several weeks. Planning back to neutral fixed the issue. 

### System Shuts off After Long Sessions
As you may have assumed, when on battery power for too long the system will shut off automatically. This also can occur if left charging for too long, likely to prevent long term damage to the batteries.

## Authors

* [Jordan S-H](https://github.com/jstebner)
* [Christian Cuevas](https://github.com/cdawgc8)
* [Jordan Krasan](https://github.com/JordanKra)
* [James Elugbemi](https://github.com/James-Elugbemi)
* [William Castle](https://github.com/wicastle)

