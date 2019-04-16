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
	static CorrectionPilot pilot=new CorrectionPilot(55, 104, leftMotor, rightMotor,gyroSensor);
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


		bricklist.addBegin(0,new Brick(new Waypoint(0,0)));
		bricklist.addBegin(1,new Brick(new Waypoint(0,0)));
		bricklist.addBegin(2,new Brick(new Waypoint(0,0)));
		bricklist.addBegin(3,new Brick(new Waypoint(0,0)));

		new Exit().start();
		new RobotColorUnknownObjects().start();
	}

	public void run(){

		boolean firstCheckPoint = true;
		boolean secondCheckPoint = false;
		boolean NoObjectDetected = false;
		boolean objectAlreadyDetected = false;
		boolean NotFirstPriority=false;

		int checkPoint = 0;
		int angleToRotate = 360;

		int firstColor = Color.RED;
		int secondColor = Color.BLUE;
		int thirdColor = Color.GREEN;

		while (bricklist.size()>0 & Button.ESCAPE.isUp()){

			pilot.setLinearSpeed(200);
			pilot.setAngularSpeed(50);
			pilot.setAngularAcceleration(200);
			pilot.setLinearAcceleration(100);	


			if ((bricklist.zeroPriorities==0)){
				
				/* if zero priorities equals zero all objects are detected, so the robot has to drive towards the first object in the list. */

				pilot.setLinearSpeed(200);
				pilot.setAngularSpeed(50);
				pilot.setAngularAcceleration(200);
				pilot.setLinearAcceleration(100);
				
				/* The list is sorted at distance from the robots current location to the objects. */
				SortAtDistance();

				drivetoObject(bricklist.get(0).waypoint); 

				gripper.grip();
				Delay.msDelay(500);
				gripper.release();
				Delay.msDelay(500);

				goToTravel(-10);


				int colorsamp = (int) colorSensor.getColor();
				priority=0;

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

			/* if zeroPriorities does not equal zero there are still objects in the surroundings. The robot has to drive to a checkpoint and scan the 
			 * area for new objects. First it is determined to which checkpoint the robot should drive. If the robot has already checked the surroundings
			 * at checkpoint one, the robot should go to checkpoint two. If a larger area should be checked, extra checkpoints can be added. 
			 */
			if (!(bricklist.zeroPriorities==0)){
				if (firstCheckPoint){
					driveto(new Waypoint(500,500));
					checkPoint=1;
				}
				else if(secondCheckPoint){
					driveto(new Waypoint(1500,500));
					checkPoint=2;
				}

				firstCheckPoint=false;
				secondCheckPoint=false;
				NoObjectDetected = false;
				objectAlreadyDetected = false;
				NotFirstPriority=false;

				angleToRotate = 360;
				float distance = 0;
				
				/* While not all the objects surrounding a certain checkpoint are detected, The following code detects objects that are located in the
				 * surroundings of the checkpoint. AngleToRotate is the angle over which the robot should still rotate to have scanned the entire area
				 * around the checkpoint. The method 'findObject' is used to retrieve the distance to objects in the surroundings of the checkpoint. 
				 * Afterwards it is checked if the detected object is already detected before. If this is the case the robot returns to its checkpoint
				 * and continues rotating to check the rest of the area. If the detected object has not been detected before, the robot drives towards it,
				 * detects its color, sets its location in the list and sorts the list by color priority. If the whole area around the checkpoint is
				 * scanned, this is stored in the boolean firstCheckPoint or secondCheckPoint.
				 */
				while (angleToRotate>0){
					distance=objectFinder.findObject(ir,pilot,ultrasonicsensor,nav,gyroSensor,angleToRotate);
					pilot.setAngularSpeed(50);
					NotFirstPriority=false;
					objectAlreadyDetected = false;

					float X = nav.getPoseProvider().getPose().getX();
					float Y = nav.getPoseProvider().getPose().getY();
					double Angle = gyroSensor.getHeadingAngle();


					if(!(distance==0)){
						distance=(distance)*1000-10;




						Point NewObject = nav.getPoseProvider().getPose().pointAt(distance, gyroSensor.getHeadingAngle());

						for (int i=0; i < bricklist.size(); i++){

							pose.setLocation(bricklist.get(i).waypoint.x, bricklist.get(i).waypoint.y);

							if ((pose.distanceTo(NewObject))<250) {
								objectAlreadyDetected = true;
								pilot.rotate(objectFinder.angleAfterUltrasoon-(int) gyroSensor.getAngle());
								goToTravel((int) (-objectFinder.distanceTravelled));
								pilot.rotate(60);
								nav.getPoseProvider().setPose(new Pose(nav.getPoseProvider().getPose().getX(),nav.getPoseProvider().getPose().getY(),gyroSensor.getAngle()));

								angleToRotate = angleToRotate - ((int) gyroSensor.getAngle()-objectFinder.StartAngle);

							}

						}


						if (!objectAlreadyDetected){
							goToTravel(distance);


							X = nav.getPoseProvider().getPose().getX();
							Y = nav.getPoseProvider().getPose().getY();
							Angle = gyroSensor.getHeadingAngle();

							double x = X+ 75*Math.cos(Angle*Math.PI/180);
							double y = Y+ 75* Math.sin(Angle*Math.PI/180);

							Waypoint waypoint = new Waypoint(nav.getPoseProvider().getPose().pointAt((float) 85, (float) gyroSensor.getHeadingAngle()));

							bricklist.get(0).changeWaypoint(waypoint);

							gripper.grip();
							Delay.msDelay(500);
							gripper.release();
							Delay.msDelay(500);
							goToTravel(-10);

							SortListOfBricks(firstColor,secondColor,thirdColor);

							if (!(priority==1)){
								NotFirstPriority = true;
								goToTravel(-distance);
								pilot.rotate(objectFinder.angleAfterUltrasoon-(int) gyroSensor.getAngle());
								goToTravel((int) (-objectFinder.distanceTravelled));
								pilot.rotate(60);
								nav.getPoseProvider().setPose(new Pose(nav.getPoseProvider().getPose().getX(),nav.getPoseProvider().getPose().getY(),gyroSensor.getAngle()));
								angleToRotate = angleToRotate - ((int) gyroSensor.getAngle()-objectFinder.StartAngle);

							}

							if (checkPoint==1){
								firstCheckPoint=true;
							}
							else if(checkPoint==2){
								secondCheckPoint=true;
							}
							if ((priority==1)){
								break;
							}
						}


					}
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
				pilot.setLinearSpeed(200);
				pilot.setAngularSpeed(50);
				pilot.setAngularAcceleration(200);
				pilot.setLinearAcceleration(100);

				drivetoObject(new Waypoint(500,0));
				Delay.msDelay(500);


				gripper.release();
				Delay.msDelay(500);

				goToTravel(-120);


			}
			else if(!(gripper.Object()) & !(NoObjectDetected)& !(objectAlreadyDetected) & !(NotFirstPriority)) {
				goToTravel(-120);

			}



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

