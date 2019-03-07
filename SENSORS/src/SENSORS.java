import Sensors.UltrasonicSensor;
import Sensors.BRICK;
import Sensors.ColorSensor;
import Sensors.Driving;
import Sensors.GyroSensor;
import Sensors.IRSensor;
import lejos.robotics.navigation.Waypoint;
import lejos.utility.Delay;
public class SENSORS {
	static UltrasonicSensor US=new UltrasonicSensor();
	static IRSensor IR= new IRSensor();
	static BRICK brick=new BRICK();
	//static ColorSensor CS = new ColorSensor();
	static GyroSensor gyro= new GyroSensor();
	static Driving drive=new Driving();
	
	public static void main(String[] args) {
		brick.beep(1, 5);
		Delay.msDelay(1000);
		gyro.reset();
		drive.driveToXY(new Waypoint(10,20,0),gyro);
		drive.driveToXY(new Waypoint(10,20,50), gyro);
		System.out.println("Angle:" + gyro.getAngle());
		Delay.msDelay(20000);
}
}
