import java.util.Random;
import lejos.hardware.motor.Motor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;

public class Vooruit {

	MovePilot pilot;
	Driving drive;
	Random ran = new Random();
	
	int period = 3; // for waiting
	int fastTravelSpeed = 8;
	int slowTravelSpeed = 2;
	
	public static void main(String[] args) {
		new Vooruit();
	}
	public Vooruit () {
		float d = 5.62f; //Diameter of the wheels
		float s = 30f;
		float y = 9.8f;
		//3 is half the distance between the wheels
		Wheel leftWheel = WheeledChassis.modelWheel(Motor.A, d).offset(-y);
		Wheel rightWheel = WheeledChassis.modelWheel(Motor.B, d).offset(y);
		Chassis chassis = new WheeledChassis(new Wheel[] {leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL); 
		pilot = new MovePilot(chassis);
		
		drive = new Driving(pilot);
		//drive.travelAndRotate();
		
		pilot.setLinearSpeed(fastTravelSpeed);
		pilot.setAngularSpeed(25);
		
		//while (Button.ESCAPE.isUp()) {
		//pilot.arc(9.5, 360);
		
		
		pilot.travel(s);
		pilot.arc(y, 90);
		pilot.travel(s);
		pilot.arc(y, 90);
		pilot.travel(s);
		pilot.arc(y, 90);
		pilot.travel(s);
		pilot.arc(y, 90);
		
		pilot.stop();
	}
	}
