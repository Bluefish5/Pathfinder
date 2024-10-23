# pathfinder

folder structure:<br/>
<br/>
|-> 📂software // code that is running on computers.<br/>
|<br/>
|-> 📂firmware // code that is running on electornic.<br/>

# Communication between stm32 <--> raspbery:
```
enum Communication{
	EMERGENCY_STOP,     // 0
	MOVE_FORWARD,       // 1
	MOVE_FORWARD_CURVE, // 2
	MOVE_REVERSE,       // 3
	MOVE_REVERSE_CURVE, // 4
	TURN_LEFT,          // 5
	TURN_RIGHT,         // 6
	GET_SENSOR_VALUES   // 7

};
```
Example:<br/>
if u want to use function moveForward() u need to send 1 to serial port of STM32.
