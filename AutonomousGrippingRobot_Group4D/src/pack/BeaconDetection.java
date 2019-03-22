package pack;

import java.awt.Point;
import java.util.ArrayList;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.FixedRangeScanner;
import lejos.robotics.localization.BeaconTriangle;
import lejos.robotics.navigation.Waypoint;
import lejos.utility.Delay;

public class BeaconDetection {

	static EV3LargeRegulatedMotor leftMotor= new EV3LargeRegulatedMotor(MotorPort.A);
	static EV3LargeRegulatedMotor rightMotor= new EV3LargeRegulatedMotor(MotorPort.B);
	static GyroSensor gyroSensor=new GyroSensor();
	
	static Waypoint beacon1 = new Waypoint(600,300);
	static Waypoint beacon2 = new Waypoint(300,0);
	static Waypoint beacon3 = new Waypoint(0,300);

	static BeaconTriangle robotDetector = new BeaconTriangle(beacon1, beacon2, beacon3);
	static CorrectionPilot pilot=new CorrectionPilot(55, 104, leftMotor, rightMotor,gyroSensor);

	static IRSensor irSensor = new IRSensor();
	
	
	public static void main(String[] args) {
		pilot.setAngularSpeed(20);
		pilot.setAngularAcceleration(200);
		
			
			double aStart = gyroSensor.getAngle();
			double a1 = 0;
			double a2 = 0;
			double a3 = 0;
			double heading =30;
			double dist=0;
			double distance = irSensor.getRelationToBeacon(1);
			double mindist = distance;
			
					
			
			pilot.rotateImmediateReturn(360,true);
			while (pilot.isMoving()){
				heading = irSensor.getRelationToBeacon(4);


				if (heading<1 & heading>-1 & !(heading==0)){

					pilot.stop();
					a1 = gyroSensor.getAngle()-aStart;
				}
			}

			
			pilot.rotateImmediateReturn(-360,true);
			while (pilot.isMoving()){
				heading = irSensor.getRelationToBeacon(0);



				if (heading<1 & heading>-1 & !(heading==0) ){

					pilot.stop();
					a2 = gyroSensor.getAngle()-aStart;
				}
			}
			
			System.out.println("New");
			
			pilot.rotateImmediateReturn(-360,true);
			while (pilot.isMoving()){
				heading = irSensor.getRelationToBeacon(2);
				System.out.println("Heading: " +heading);



				if (heading<1 & heading>-1 & !(heading==0)){

					pilot.stop();
					a3 = gyroSensor.getAngle()-aStart;
				}
			}
			
			System.out.println("a1: " +a1);
			Delay.msDelay(3000);
			System.out.println("a2: " +a2);
			Delay.msDelay(3000);
			System.out.println("a3: " +a3);
			Delay.msDelay(3000);

			
			
			double x = robotDetector.calcPose(a1, a2, a3).getX();
			double y = robotDetector.calcPose(a1, a2, a3).getY();

			System.out.println("x: " +x);
			Delay.msDelay(3000);
			System.out.println("y: " +y);
			Delay.msDelay(3000);
			
			
		//}
		


	}

}
