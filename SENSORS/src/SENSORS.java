import Sensors.UltrasonicSensor;
import Sensors.BRICK;
import Sensors.ColorSensor;
import Sensors.Driving;
import Sensors.GyroSensor;
import Sensors.IRSensor;
import lejos.robotics.navigation.Navigator;

import lejos.robotics.navigation.Waypoint;
import lejos.utility.Delay;
public class SENSORS {
	public UltrasonicSensor US=new UltrasonicSensor();
	//static IRSensor IR= new IRSensor();
	public BRICK brick=new BRICK();
	//static ColorSensor CS = new ColorSensor();
	public GyroSensor gyro= new GyroSensor();
	public Driving drive=new Driving(gyro);
	
	public  SENSORS() {
		//new Regulator().start();
	}

	/*private class Regulator extends Thread {
		public Regulator() {
			setDaemon(true);
		}
		public void run() {
			while (true) {
				Delay.msDelay(250);
				drive.setHeading(gyro);

			}
		}
	}
	*/
	
	public static void main(String[] args) {
		
		SENSORS rob=new SENSORS();
		rob.brick.beep(1, 5);
		rob.gyro.reset();
		rob.drive.reset();
		System.out.println("Angle:" + rob.gyro.getAngle());
		for(int i=0;i<1;i++) {
			rob.drive.getPose();
			Delay.msDelay(1000);
		}
		System.out.println("start1");
			//rob.drive.goToTravel(400);
			//rob.drive.getPose();
			//Delay.msDelay(5000);
			rob.drive.driveTo(new Waypoint(40,0));
			System.out.println("start2");
			Delay.msDelay(500);
			rob.drive.getPose();
			Delay.msDelay(500);			
			rob.drive.driveTo(new Waypoint(40,40));
			System.out.println("start3");
			Delay.msDelay(500);
			rob.drive.getPose();
			Delay.msDelay(500);
			rob.drive.driveTo(new Waypoint(0, 40));
			System.out.println("start4");
			Delay.msDelay(500);
			rob.drive.getPose();
			Delay.msDelay(500);
			rob.drive.driveTo(new Waypoint(0, 0));
			System.out.println("start5");
			Delay.msDelay(500);
			rob.drive.getPose();
			Delay.msDelay(500);
			rob.drive.driveTo(new Waypoint(40,40));
			System.out.println("start6");
			Delay.msDelay(500);
			rob.drive.getPose();
			Delay.msDelay(500);
			rob.drive.driveTo(new Waypoint(40,0));
			System.out.println("start7");
			Delay.msDelay(500);
			rob.drive.getPose();
			Delay.msDelay(500);
			rob.drive.driveTo(new Waypoint(0, 40));
			System.out.println("start8");
			Delay.msDelay(500);
			rob.drive.getPose();
			Delay.msDelay(500);
			rob.drive.driveTo(new Waypoint(0, 0));
			Delay.msDelay(500);
			rob.drive.getPose();
			Delay.msDelay(500);
			Delay.msDelay(100000);
		
		
}
}
