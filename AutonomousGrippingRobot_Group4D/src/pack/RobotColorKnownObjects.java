package pack;

import java.util.ArrayList;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.Gyroscope;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.utility.Delay;
import lejos.utility.GyroDirectionFinder;

import lejos.robotics.Color;


public class RobotColorKnownObjects extends Thread{

	static EV3LargeRegulatedMotor leftMotor= new EV3LargeRegulatedMotor(MotorPort.A);
	static EV3LargeRegulatedMotor rightMotor= new EV3LargeRegulatedMotor(MotorPort.B);
	static GyroSensor gyroSensor=new GyroSensor();
	static CorrectionPilot pilot=new CorrectionPilot(55, 104, leftMotor, rightMotor,gyroSensor);
	static Navigator nav=new Navigator(pilot);
	static Waypoint collectorpoint=new Waypoint(0,0);
	static Gripper gripper=new Gripper();
	static BrickList bricklist = new BrickList();
	static ColorSensor colorSensor = new ColorSensor();



	public static void main(String[] args) {
		nav.getPoseProvider().getPose().setLocation(0, 0);
		pilot.setLinearSpeed(200);
		pilot.setAngularSpeed(50);
		pilot.setAngularAcceleration(200);
		pilot.setLinearAcceleration(100);	

		/* bricks are added at known locations */
		bricklist.addBegin(0,new Brick(new Waypoint(0,1500)));
		bricklist.addBegin(1,new Brick(new Waypoint(500,1500)));
		bricklist.addBegin(2,new Brick(new Waypoint(1500,1500)));
		bricklist.addBegin(3,new Brick(new Waypoint(2000,1000)));


		new Exit().start();
		new RobotColorKnownObjects().start();
	}

	public void run(){

		int firstColor = Color.RED;
		int secondColor = Color.BLUE;
		int thirdColor = Color.GREEN;

		while (bricklist.size()>0 & Button.ESCAPE.isUp()){

			/* the robot drives towards the closest object */
			SortAtDistance();
			drivetoObject(bricklist.get(0).waypoint);

			gripper.grip();
			Delay.msDelay(500);
			gripper.release();
			Delay.msDelay(500);

			goToTravel(-10);

			/* if zeroPriorities does not equal zero, first the color of the object needs to be determined, and the list has to be sorted. This is done
			 * with the method 'sortListOfBricks'
			 */
			if (!(bricklist.zeroPriorities==0)){
				SortListOfBricks(firstColor,secondColor,thirdColor);
			}
			
			/* if zeroPriorities equals zero, the colors of all the objects are determined and the robot can grip the first object in the list, after 
			 * the color is checked again.
			 */
			if ((bricklist.zeroPriorities==0)){
				Delay.msDelay(3000);

				int colorsamp = (int) colorSensor.getColor();
				int priority=0;

				if(colorsamp==firstColor){
					priority=1;
				}
				if(colorsamp==secondColor){
					priority=2;
				}
				if(colorsamp==thirdColor){
					priority=3;
				}	

				if (bricklist.get(0).colorPriority==priority){
					gripper.grip();
					Delay.msDelay(500);
					bricklist.remove(0);
				}

			}



			/* the robot travels towards the collector points and drops the object. */
			if (gripper.Object()){
				drivetoObject(new Waypoint(500,0));
				Delay.msDelay(500);


				gripper.release();
				Delay.msDelay(500);

				goToTravel(-120);

			}
			else {
				goToTravel(-120);

			}

		}


	}


	public static void driveto(Waypoint waypoint){
		float X = nav.getPoseProvider().getPose().getX();
		float Y = nav.getPoseProvider().getPose().getY();

		nav.goTo(waypoint);

		while(!nav.waitForStop()){
			Delay.msDelay(100);};


			Delay.msDelay(1000);


			Delay.msDelay(500);

			nav.getPoseProvider().setPose(new Pose(waypoint.x,waypoint.y,gyroSensor.getHeadingAngle()));
			Delay.msDelay(500);



	}

	public static void drivetoObject(Waypoint waypoint){
		double X = nav.getPoseProvider().getPose().getX();
		double Y = nav.getPoseProvider().getPose().getY();


		double distance = nav.getPoseProvider().getPose().distanceTo(waypoint)-75;
		double angle = gyroSensor.getAngle() + nav.getPoseProvider().getPose().relativeBearing(waypoint);

		double x = X+ distance*Math.cos(angle*Math.PI/180);
		double y = Y+ distance* Math.sin(angle*Math.PI/180);

		Delay.msDelay(1000);


		driveto(new Waypoint(x,y));


	}

	public static void goToTravel(int distance){
		float X = nav.getPoseProvider().getPose().getX();
		float Y = nav.getPoseProvider().getPose().getY();

		pilot.travel(distance,false);


		X = X+(float) distance*(float) Math.cos(gyroSensor.getHeadingAngle()*Math.PI/180);
		Y = Y+(float) distance*(float) Math.sin(gyroSensor.getHeadingAngle()*Math.PI/180);

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
		int priority=0;

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




