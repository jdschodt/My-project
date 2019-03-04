import Sensors.UltrasonicSensor;
import Sensors.BRICK;
import Sensors.IRSensor;
import lejos.utility.Delay;
public class SENSORS {
	static UltrasonicSensor US=new UltrasonicSensor();
	static IRSensor IR= new IRSensor();
	static BRICK brick=new BRICK();
	public static void main(String[] args) {
		
		brick.beep(1, 100);
		Delay.msDelay(1000);
		brick.beep(2, 100);
		Delay.msDelay(1000);
		brick.beep(5, 100);
		Delay.msDelay(1000);
		for (int i=0;i<10;i++) {
			String text="Ultrasoon:"+Double.toString(US.getDistance());
			brick.println(text);
			int distance=IR.getDistance();
			text="IR"+Integer.toString(distance);
			brick.println(text);
			Delay.msDelay(2000);

	}

}
}
