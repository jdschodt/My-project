package pack;

import java.util.ArrayList;

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
	static CorrectionPilot pilot=new CorrectionPilot(57.72, 186, leftMotor, rightMotor,gyroSensor);
	static Pose pose = new Pose();

	static Navigator nav=new Navigator(pilot);
	static Waypoint collectorpoint=new Waypoint(0,0);
	static Gripper gripper=new Gripper();
	static BrickList bricklist = new BrickList();
	static ColorSensor colorSensor = new ColorSensor();
	static IRSensor ir = new IRSensor();
	static ObjectFinder objectFinder= new ObjectFinder();
	static UltrasonicSensor ultrasonicsensor = new UltrasonicSensor();
	static int priority=0;

	static boolean gripBrick= false;



	public static void main(String[] args) {

		nav.getPoseProvider().getPose().setLocation(0, 0);
		pilot.setLinearSpeed(200);
		pilot.setAngularSpeed(50);
		pilot.setAngularAcceleration(200);
		pilot.setLinearAcceleration(100);	

		new Exit().start();
		new RobotColorUnknownObjects().start();
	}

	public void run(){

		boolean firstCheckPoint = true;
		boolean secondCheckPoint = false;
		boolean gotObject = false;
		boolean NoObjectDetected = false;
//		boolean objectAlreadyDetected = false;
//		boolean NotFirstPriority=false;

		int checkPoint = 0;
		int angleToRotate = 360;

		int firstColor = Color.RED;
		int secondColor = Color.BLUE;
		int thirdColor = Color.GREEN;

		while (Button.ESCAPE.isUp()){

			pilot.setLinearSpeed(200);
			pilot.setAngularSpeed(50);
			pilot.setAngularAcceleration(200);
			pilot.setLinearAcceleration(100);	

			System.out.println("Start of Detection loop");		

			int colorsamp = 0;
			float objX = 0;
			float objY = 0;
			if (gotObject==false){
				if (firstCheckPoint){
					driveto(new Waypoint(500,500)); //checkpoint 1 to scan
					checkPoint=1;
					System.out.println("Checkpoint 1 reached");
					Delay.msDelay(1000); // may be deleted
				}
				else if(secondCheckPoint){
					driveto(new Waypoint(1000,1000));
					System.out.println("Checkpoint 2 reached");
					checkPoint=2;
				}

				firstCheckPoint=false;
				secondCheckPoint=false;

				angleToRotate = 360;
				float distance = 0;

				while (!gripper.Object()){
					
					System.out.println("detecting....");
					distance=objectFinder.findObject(ir,pilot,ultrasonicsensor,nav,gyroSensor,angleToRotate);
					pilot.setAngularSpeed(50);
//					NotFirstPriority=false;
//					objectAlreadyDetected = false;

					float X = nav.getPoseProvider().getPose().getX();
					float Y = nav.getPoseProvider().getPose().getY();
					double Angle = gyroSensor.getHeadingAngle();

					objX = nav.getPoseProvider().getPose().getX();
					objY = nav.getPoseProvider().getPose().getY();
					double objA = gyroSensor.getHeadingAngle();
					
				
					
					if(!(distance==0)){

						Point NewObject = nav.getPoseProvider().getPose().pointAt(distance, gyroSensor.getHeadingAngle());

						System.out.println("Distance travelled, Object Reached");
						gripper.grip();
						colorsamp = (int) colorSensor.getColor();
						
						System.out.println("Color: "+colorsamp);

						X = nav.getPoseProvider().getPose().getX();
						Y = nav.getPoseProvider().getPose().getY();
						Angle = gyroSensor.getHeadingAngle();

						double x = X+ 75*Math.cos(Angle*Math.PI/180);
						double y = Y+ 75*Math.sin(Angle*Math.PI/180);

						Waypoint waypoint = new Waypoint(nav.getPoseProvider().getPose().pointAt((float) 85, (float) gyroSensor.getHeadingAngle()));

						}

						if (checkPoint==1){
							firstCheckPoint=false;
							System.out.println("boolean set");
						}
						else if(checkPoint==2){
							secondCheckPoint=true;
						}
//						if ((priority==1)){
//							break;
//						}
						
						
					else if ((distance==0) | (angleToRotate<0)){
						NoObjectDetected = true;

						if (checkPoint==1){
							secondCheckPoint=true;
							firstCheckPoint=false;
						}
						break;

					}

				}

			}

			/* if the gripper holds an object it is brought to the collector point where it is released.*/
			if (gripper.Object()){
				reposition(objX, objY, colorsamp);
				System.out.println("om te grippen, grip");
				pilot.setLinearSpeed(200);
				pilot.setAngularSpeed(50);
				pilot.setAngularAcceleration(200);
				pilot.setLinearAcceleration(100);
				
				if (colorsamp==2) {
					System.out.println("blauw");
					drivetoObject(new Waypoint(500,0));
				}
				else if (colorsamp==0) {
					System.out.println("rood");
					drivetoObject(new Waypoint(700,0));
				}
				else {
					System.out.println("rest");
					drivetoObject(new Waypoint(900,0));
				}

				Delay.msDelay(500);
				

				gripper.release();
				Delay.msDelay(500);
				System.out.println("Afheleverd");

				goToTravel(-170);
				secondCheckPoint = true;


			}
			else if(!(gripper.Object()) & !(NoObjectDetected)) {
				goToTravel(-150);

			}

		}
	}

	public static void reposition(float objX, float objY, float RobH, int colorsamp) {
		float[] repos = {0,0};
		int[] parking = {0,0};
		if (colorsamp==2) {
			parking[0] = 500;
			parking[1] = 0;
		}
		else if (colorsamp==0) {
			parking[0] = 700;
			parking[1] = 0;
		}
		else {
			parking[0] = 900;
			parking[1] = 0;
		}
		float a=(objX - parking[0])/(objY - parking[1]);
		float b= objY -a*objX;
		float DestH=(float) Math.atan(a)-90;
		if(DestH>-90) {
			if((180>=RobH)&(RobH>=DestH+180)|(DestH-90>RobH)&(RobH>=-180)) {
				System.out.println("1th quadrant");
			}
			if((DestH>RobH)&(RobH>=DestH-90)) {
				System.out.println("2th quadrant");
			}
			if((DestH+90>RobH)&(RobH>=DestH)){
				System.out.println("3th quadrant");				
			}
			if((DestH+180>RobH)&(RobH>=DestH+90)) {
				System.out.println("4th quadrant");
			}
		}
		if(-90>DestH) {
			if((DestH+270>RobH)&(RobH>=DestH+180)) {
				System.out.println("1th quadrant");
			}
			if((180>=RobH)&(RobH>=DestH+270)|(DestH>RobH)&(RobH>=-90)) {
				System.out.println("2th quadrant");
			}
			if((DestH+90>RobH)&(RobH>=DestH)){
				System.out.println("3th quadrant");				
			}
			if((DestH+180>RobH)&(RobH>=DestH+90)) {
				System.out.println("4th quadrant");
			}	
		repos[1] =(float) (objY+200*Math.sin(Math.atan(Math.abs(a))));
		repos[0] = (repos[1]-b)/a;
		
		}
		
		
		
		
	}


	public static void driveto(Waypoint waypoint){
		
		float X = nav.getPoseProvider().getPose().getX();
		float Y = nav.getPoseProvider().getPose().getY();
		nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getHeadingAngle()));

		nav.goTo(waypoint);

		while(!nav.waitForStop()){
			Delay.msDelay(100);};


			Delay.msDelay(500);

			nav.getPoseProvider().setPose(new Pose(waypoint.x,waypoint.y,gyroSensor.getHeadingAngle()));
			Delay.msDelay(500);



	}

	public static void drivetoObject(Waypoint waypoint){
		double X = nav.getPoseProvider().getPose().getX();
		double Y = nav.getPoseProvider().getPose().getY();

		double distance = nav.getPoseProvider().getPose().distanceTo(waypoint)-85;
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

	public static void SortListOfBricks(int color1,int color2,int color3){

		/* This method detects the color of a new detected object and sorts the list of bricks by color priority. First it detects the color of the object
		 * in front of the robot, where after it is compared with the color priorities as given as an argument of this method. color1 has the first
		 * priority, color 2 has the second priority and color3 has the third priority. 
		 */
		int colorsamp = (int) colorSensor.getColor();
		priority=0;

		if(colorsamp==color1){
			priority=1;
		}
		if(colorsamp==color2){
			priority=2;
		}
		if(colorsamp==color3){
			priority=3;
		}	

		Delay.msDelay(3000);
		bricklist.get(0).changeColorPriority(priority);

		/* the method 'sortPriority' from the class BrickList is used to sort the list */
		bricklist.sortPriority();


		int i=0;
		boolean gripBrick= false;

		/* if the method encounters a object with the first color priority, the object is immedialtely gripped. */
		while(bricklist.size()>i && !gripBrick){
			if (bricklist.get(i).colorPriority==1){
				bricklist.add(0, bricklist.get(i));
				bricklist.remove(i+1);

				gripBrick=true;
			}
			i++;
		}

		if (gripBrick==true){
			Delay.msDelay(500);
			gripper.grip();
			Delay.msDelay(500);
			bricklist.remove(0);
			gripBrick=false;
		}
	}

	public static void SortAtDistance(){
		/* this method sorts the bricks in the list with the same color priority as the first birck on the distance between the robot and the brick. */
		int priority = bricklist.get(0).colorPriority;
		int priorityi = 0;
		
		/* first the amount of bricks with a color priority equal to the first brick are determined. */
		for (int i=0; i < bricklist.size(); i++){

			if (bricklist.get(i).colorPriority == priority) {
				priorityi = priorityi+1;
			}

		}
		
		/* for each brick, the distance from the brick to the robot is compared with the distance of the next brick in the list, with the same  
		 * color priority to the robot. If the distance is larger, the two bricks switch positions. This is repeated as many times as there are
		 * bricks with the same color priority as the first brick. After that the list is sorted on distance.
		 */
		for (int i=0; i < priorityi; i++){

			for  (int j=0; j < priorityi-1; j++){

				if (nav.getPoseProvider().getPose().distanceTo(bricklist.get(j).waypoint) > nav.getPoseProvider().getPose().distanceTo(bricklist.get(j+1).waypoint)) {
					Brick brick = bricklist.get(j+1);
					bricklist.set(j+1, bricklist.get(j));
					bricklist.set(j,brick);
				}
			}
		}


	}
}

