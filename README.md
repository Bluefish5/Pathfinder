# pathfinder

folder structure:<br/>
<br/>
|-> 📂software // code that is running on computers.<br/>
|<br/>
|-> 📂firmware // code that is running on electornic.<br/>

# Communication between stm32 <--> raspbery:
```
enum Communication{
	EMERGENCY_STOP,//0
	MOVE_FORWARD,//1
	MOVE_REVERSE,//2
	TURN_LEFT,//3
	TURN_RIGHT,//4
	SET_MOVMENT_SPEED,//5
	GET_SENSOR_VALUES//6

};
```
Example:<br/>
if u want to use function setMovmentSpeed() u need to send "5 -10 10\n" to serial port of STM32.
All other functions are parametrless u just need to send number of function.
