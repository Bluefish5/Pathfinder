# Communication between STM32 <--> Raspbery:

| Enum | Value | function | description |
| --- | --- | --- | --- | 
| **EMERGENCY_STOP** | 0 | `void emergencyStop()` | both motors set speed to 0% |
| **MOVE_FORWARD** | 1 | `void moveForward()` | both motors set speed to 100% forward |
| **MOVE_REVERSE** | 2 | `void moveReverse()` | both motors set speed to 100% reverse |
| **TURN_LEFT** | 3 | `void turnLeft()` | motor A spining forwad and motor B spinnig revese with 100% speed |
| **TURN_RIGHT** | 4 | `void turnRight()` | motor A spining revese and motor B spinnig forwad with 100% speed |
| **SET_MOVMENT_SPEED** | 5 | `setMovmentSpeed(int motorA,int motorB)` | set speed of 2 motor individualy can accepting paramaeter (-100 to 100) |
| **GET_SENSOR_VALUES** | 7 | `void getSensorValues()` | check actual state of sensor and giving to serial port as binary ex. 10000 or 11111 |


Example:<br/>
if u want to use function setMovmentSpeed() u need to send "5 -10 10\n" to serial port of STM32.
All other functions are parametrless u just need to send number of function.
