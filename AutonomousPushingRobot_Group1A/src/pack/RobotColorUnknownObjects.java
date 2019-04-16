package pack;

import java.util.ArrayList;

import lejos.hardware.Battery;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.Gyroscope;
import lejos.robotics.SampleProvider;
import lejos.robotics.geometry.Point;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.utility.Delay;
import lejos.utility.GyroDirectionFinder;
import lejos.robotics.Color;


public class RobotColorUnknownObjects extends Thread{

	static EV3LargeRegulatedMotor leftMotor= new EV3LargeRegulatedMotor(MotorPort.A);
	static EV3LargeRegulatedMotor rightMotor= new EV3LargeRegulatedMotor(MotorPort.B);
	static GyroSensor gyroSensor=new GyroSensor();
	static CorrectionPilot pilot=new CorrectionPilot(56, 186, leftMotor, rightMotor,gyroSensor);
	static Pose pose = new Pose();

	static Navigator nav=new Navigator(pilot);
	static Waypoint collectorpoint=new Waypoint(0,0);
	static BrickList bricklist = new BrickList();
	static ColorSensor colorSensor = new ColorSensor();
	static IRSensor ir = new IRSensor();
	static ObjectFinder objectFinder= new ObjectFinder();
	static UltrasonicSensor ultrasonicsensor = new UltrasonicSensor();
	static int priority=0;
	static int diagonalbox=275; //diagonal of the box
	static int clearance = 400;

	static boolean gripBrick= false;



	public static void main(String[] args) {
		
		System.out.println(Battery.getVoltage());
		System.out.println("Version 16/4/14/9/32");
		Delay.msDelay(5000);
		nav.getPoseProvider().getPose().setLocation(0, 0);
		pilot.setLinearSpeed(200);
		pilot.setAngularSpeed(50);
		pilot.setAngularAcceleration(20);
		pilot.setLinearAcceleration(50);	

		//these 2 programs are multithreaded
		new Exit().start();
		new RobotColorUnknownObjects().start();
	}

	public void run(){

		boolean gotObject = false; //IS the box in the correct position
		boolean NoObjectDetected = true; //Go further to the next checkpoint
//		boolean objectAlreadyDetected = false;
//		boolean NotFirstPriority=false;

		int checkPoint = 0;//current position
		int destination=1; //destination checpoint
		int angleToRotate = 360;
		
		while (Button.ESCAPE.isUp()){

			pilot.setLinearSpeed(200);
			pilot.setAngularSpeed(50);
			pilot.setAngularAcceleration(20);
			pilot.setLinearAcceleration(50);	

			System.out.println("Start of Detection loop");		

			int colorsamp = 0;
			float objX = 0; // x position
			float objY = 0; // y position
			float objA = 0; //heading
			
			//If no object is found at a checkpoint, go to another checkpoint
			if (gotObject==false){
				if (destination==1){
					driveto(new Waypoint(500,500)); //checkpoint 1 to scan
					checkPoint=1;
					System.out.println("Checkpoint 1 reached");
				}
				else if(destination==2){
					driveto(new Waypoint(1500,500));
					System.out.println("Checkpoint 2 reached");
					checkPoint=2;
				}
				angleToRotate = 360; // angle to make complete turn
				while (!gotObject && angleToRotate>0){ //( !gripper.Object can be probably changed to !gotObject (so that we get rid of the gripper program).
					
					System.out.println("detecting....");
					gotObject=objectFinder.findObject(pilot,ultrasonicsensor,nav,gyroSensor,angleToRotate);
					pilot.setLinearSpeed(200);
					pilot.setAngularSpeed(50);
					pilot.setAngularAcceleration(20);
					pilot.setLinearAcceleration(50);
//save the coordinates of the object (needs to be adjusted for the size of the object)
					objA = gyroSensor.getHeadingAngle();
					objX = (float) (nav.getPoseProvider().getPose().getX()+ diagonalbox/2*Math.cos(objA*Math.PI/180));
					objY = (float) (nav.getPoseProvider().getPose().getY()+ diagonalbox/2*Math.sin(objA*Math.PI/180));
					if (checkPoint==1){
						destination=2;
						}
					else if(checkPoint==2){
						destination=1;
						}
					
//objectFinder.findObject returns true or false depending if there's an object detected or not
//The next if statement takes care of when an object is found
					if(gotObject){
						System.out.println("Object Reached");
						colorsamp = (int) colorSensor.getColor();
						System.out.println("Color: "+colorsamp);
					}
//Set the booleans to go to other checkpoint after parking the object
						
//Or if a zero is returned by objectFinder.findObject:
//then just set the booleans to go the to other checkpoint and stop the while loop
					else{
						break;
					}
				}
			}

//			If an object is found, the robot needs to reposition itself behind the object after which it drives to the right parkingspot.
			if (gotObject){
				// reposition the robot.
				System.out.println("Drive around");
				reposition(objX, objY, objA, colorsamp);
				System.out.println("Drive around completed");
				// in function of the color, drive to the right parking spot.
				// Here we can implement that the robot drives to 20 in front of the parking spot and then straight into it.
				if (colorsamp==2) {
					System.out.println("blauw");
					driveto(new Waypoint(500,0));
					driveto(new Waypoint(500,-350));
				}
				else if (colorsamp==0) {
					System.out.println("rood");
					driveto(new Waypoint(1000,0));
					driveto(new Waypoint(1000,-350));
				}
				else {
					System.out.println("rest");
					driveto(new Waypoint(1500,0));
					driveto(new Waypoint(1500,-350));
				}

				Delay.msDelay(500);
				System.out.println("Delivered");
				goToTravel(-300); // drive backwards out of the parking
				gotObject=false;


			}
			else {
				System.out.println("No object to deliver");

			}

		}
	}


	
	
//	We need to check this function for errors, I uploaded a separate java file to test this code.
	public static void reposition(float objX, float objY, float RobH, int colorsamp) {
		System.out.println(nav.getPoseProvider().getPose());
		System.out.println(objX +" "+ objY +" "+ RobH);
		Delay.msDelay(1000);
		goToTravel(-clearance);
		System.out.println(nav.getPoseProvider().getPose());
		Delay.msDelay(1000);
		float[] repos = {0,0};
		int[] parking = {0,0};
		float[] chp1 = {0,0};
		float[] chp2 = {0,0};
		if (colorsamp==2) {
			parking[0] = 500;
			parking[1] = 0;
		}
		else if (colorsamp==0) {
			parking[0] = 1000;
			parking[1] = 0;
		}
		else {
			parking[0] = 1500;
			parking[1] = 0;
		}
		System.out.println("objY" + objY);
		System.out.println("objX" + objX);
		
		float a=(float)(objY - parking[1])/(float)(objX - parking[0]);
		repos[1] =(float) (objY+clearance*Math.sin(Math.atan(Math.abs(a))));

		float b=objY-a*objX;
		repos[0] = (repos[1]-b)/a;


		if (a<0) {
			chp1[1] = (float) (objY+clearance*Math.cos(Math.atan(a)));
			chp1[0] = (float) (objX-clearance*Math.sin(Math.atan(a)));
			
			chp2[1] = (float) (objY-clearance*Math.cos(Math.atan(a)));
			chp2[0] = (float) (objX+clearance*Math.sin(Math.atan(a)));
			System.out.println("a<0");
		}
		else {
			chp1[1] = (float) (objY-clearance*Math.cos(Math.atan(a)));
			chp1[0] = (float) (objX+clearance*Math.sin(Math.atan(a)));
			
			chp2[1] = (float) (objY+clearance*Math.cos(Math.atan(a)));
			chp2[0] = (float) (objX-clearance*Math.sin(Math.atan(a)));
			System.out.println("a>0");
		}
		float DestH=(float)-Math.atan(1/a)*(float)180/(float)Math.PI-90;
		
		if(DestH>-90) {
			if((180>=RobH)&(RobH>=DestH+180)&(RobH>=0)|(DestH-90>RobH)&(RobH>=-180)) {
				System.out.println("1th quadrant");
				driveto(new Waypoint(chp1[0], chp1[1]));
				driveto(new Waypoint(repos[0], repos[1]));
			}
			else if((DestH>RobH)&(RobH>=DestH-90)) {
				System.out.println("2th quadrant");
				driveto(new Waypoint(repos[0], repos[1]));
			}
			else if((DestH+90>RobH)&(RobH>=DestH)){
				System.out.println("3th quadrant");
				driveto(new Waypoint(repos[0], repos[1]));
			}
			else if((DestH+180>RobH)&(RobH>=DestH+90)) {
				System.out.println("4th quadrant");
				driveto(new Waypoint(chp2[0], chp2[1]));
				driveto(new Waypoint(repos[0], repos[1]));
			}

		}
		if(-90>DestH) {
			if((DestH+270>RobH)&(RobH>=DestH+180)) {
				System.out.println("1th quadrant");
				driveto(new Waypoint(chp1[0], chp1[1]));
				driveto(new Waypoint(repos[0], repos[1]));
			}
			else if((180>=RobH)&(RobH>=DestH+270)|(DestH>RobH)&(RobH>=-90)) {
				System.out.println("2th quadrant");
				driveto(new Waypoint(repos[0], repos[1]));
			}
			else if((DestH+90>RobH)&(RobH>=DestH)){
				System.out.println("3th quadrant");	
				driveto(new Waypoint(repos[0], repos[1]));
			}
			else if((DestH+180>RobH)&(RobH>=DestH+90)) {
				System.out.println("4th quadrant");
				driveto(new Waypoint(chp2[0], chp2[1]));
				driveto(new Waypoint(repos[0], repos[1]));
			}
		}
		}
		
			

//What is the difference between driveTo and driveToObject?
	public static void driveto(Waypoint waypoint){
		
		float X = nav.getPoseProvider().getPose().getX();
		float Y = nav.getPoseProvider().getPose().getY();
		nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getHeadingAngle())); //only heading angle correction

		nav.goTo(waypoint);

		while(!nav.waitForStop()){
			Delay.msDelay(100);
		}


		Delay.msDelay(500);

		nav.getPoseProvider().setPose(new Pose(waypoint.x,waypoint.y,gyroSensor.getHeadingAngle()));
		Delay.msDelay(500);



	}

	public static void drivetoObject(Waypoint waypoint){
		double X = nav.getPoseProvider().getPose().getX();
		double Y = nav.getPoseProvider().getPose().getY();

		double distance = nav.getPoseProvider().getPose().distanceTo(waypoint)-diagonalbox/2;
		double angle = gyroSensor.getAngle() + nav.getPoseProvider().getPose().relativeBearing(waypoint);

		double x = X+ distance*Math.cos(angle*Math.PI/180);
		double y = Y+ distance* Math.sin(angle*Math.PI/180);



		driveto(new Waypoint(x,y));


	}

	public static void goToTravel(float distance){
		float X = nav.getPoseProvider().getPose().getX();
		float Y = nav.getPoseProvider().getPose().getY();
		double Angle = gyroSensor.getHeadingAngle();

		pilot.travel(distance,false);


		X = X+(float) distance*(float) Math.cos(Angle*Math.PI/180);
		Y = Y+(float) distance*(float) Math.sin(Angle*Math.PI/180);

		Delay.msDelay(500);


		nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
		Delay.msDelay(500);


	}

}

