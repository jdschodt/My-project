package pack;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.utility.Delay;

public class ObjectFinder {
	double startAngle;
	double endAngle;
	public float distance;
	public int startAngle2;
	public boolean directDetection = false;
	public double safetyfactor = 1.2;


	public boolean findObject(CorrectionPilot pilot, UltrasonicSensor usensor, Navigator nav, GyroSensor gyroSensor, int angleToRotate ){	
		/* this method detects if there are any objects in a circle with a radius of one meter around the robot. If an object is found, the method 
		 * returns the distance to this object. If no object is found, the method returns zero.
		 */

		pilot.setLinearSpeed(100);
		pilot.setAngularSpeed(25);
		pilot.setLinearAcceleration(10);
		pilot.setAngularAcceleration(10);

		double dist=0;
		
		/* while the robot is rotating, it takes distance samples from its ultrasonic sensor. If the distance measured is larger than the previous 
		 * sample and is smaller than 0.75 m, the rotation stops. This means the robot detected an object. In this case the robot travels towards 
		 * the object. If no object is detected the method returns zero. 
		 */
		int i = 0;
		pilot.rotateImmediateReturn(angleToRotate, true);
		distance=3;
		 //counter to detect direct object detection
		while(pilot.isMoving()&&Button.ESCAPE.isUp()){
			i++;
			dist = distance; //=3
			distance=usensor.getDistance(); //new: .22
			
//			If the robot detects an object at first measurement, the robot turns to the left until the object is not detected anymore.
			if (dist>distance & !(dist==0) & distance<0.75 & i==1){
				System.out.println("object at first sight");
				System.out.println("goingLeft...");
				while(pilot.isMoving()&&Button.ESCAPE.isUp()){
					dist = distance;
					distance=usensor.getDistance();
					if(distance>safetyfactor*dist) { //if this is true, object isn't detected anymore
						pilot.stop(); // stops, objected isn't detected anymore, now search for the other boundary
						startAngle = gyroSensor.getAngle();
						System.out.println("StartAngle: " +startAngle);
					}
				}
			distance = 3;
//			and after that the robot turns to the right until the object is not detected anymore.
//			The robot has then the 2 'boundary angles' and can take the mean of both to travel towards it
			pilot.rotateImmediateReturn(-360, true);
			System.out.println("GoingRight...");
			Delay.msDelay(2000);
			while(pilot.isMoving()&&Button.ESCAPE.isUp()){
				dist = distance;
				distance=usensor.getDistance();
				if(distance>safetyfactor*dist) { // detection for the second boundary
					pilot.stop();
					endAngle = gyroSensor.getAngle();
					System.out.println("endAngle: "+ endAngle);
//Broad detection of the object
					System.out.println("Ready To Center");
					pilot.rotate((startAngle-endAngle)/2, true); //rotate to the center (relative angle)
					float X = nav.getPoseProvider().getPose().getX();
					float Y = nav.getPoseProvider().getPose().getY();
					double Angle = gyroSensor.getHeadingAngle();
					distance=usensor.getDistance();	
					System.out.println(distance);
//Travel a quarter the distance that is measured, to execute the same code to find the object again.
					distance = distance * 250;
					pilot.travel((int) (distance),false);
					X = X+(float) (distance)*(float) Math.cos(Angle*Math.PI/180);
					Y = Y+(float) (distance)*(float) Math.sin(Angle*Math.PI/180);
	
					nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
					
//preciseDetection is exactly the same code
					distance = preciseDetection(pilot, usensor, nav, gyroSensor);
					if (distance==0){
						System.out.println("Something went wrong");
						return false;
					}
					else {
						distance = distance * 1000;
						pilot.travel((int) (distance),false);
						X = X+(float) (distance)*(float) Math.cos(Angle*Math.PI/180);
						Y = Y+(float) (distance)*(float) Math.sin(Angle*Math.PI/180);
						nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
						return true;	
					}
				}
			}
			}
			
			//If robot does not detect object at first sight.
			if(dist>distance & !(dist==0) & distance<.5 & i !=1){
				System.out.println("Not directly detected ");
				//If we detect an object, keep turning until object is out of sight.
				// Rotate back half of the rotated angle: The robot should be oriented in front of the object.
				startAngle = gyroSensor.getAngle();
				System.out.println("StartAngle: "+ startAngle);
				System.out.println("Going Further...");
				while(pilot.isMoving()&&Button.ESCAPE.isUp()){
					dist = distance;
					distance=usensor.getDistance();
					if(distance>safetyfactor*dist) {
						pilot.stop();
						endAngle = gyroSensor.getAngle();
						System.out.println("EndAngle: "+ endAngle);
						System.out.println("Ready To Center");
//Broad detection of the object						
						pilot.rotate(-(endAngle-startAngle)/2, true); //calculation of the angle of box, now in negative rotation sense
						distance=usensor.getDistance();
						float X = nav.getPoseProvider().getPose().getX();
						float Y = nav.getPoseProvider().getPose().getY();
						double Angle = gyroSensor.getHeadingAngle();
						distance = distance * 250;
						pilot.travel((int) (distance),false);
						X = X+(float) (distance)*(float) Math.cos(Angle*Math.PI/180);
						Y = Y+(float) (distance)*(float) Math.sin(Angle*Math.PI/180);
						nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));					
//again execute the same code again to more precisely detect the object.
						distance = preciseDetection(pilot, usensor, nav, gyroSensor);
						if (distance==0){
							System.out.println("Something went wrong");
							return false;
						}
						else {
							distance = distance * 1000;
							pilot.travel((int) (distance),false);
							X = X+(float) (distance)*(float) Math.cos(Angle*Math.PI/180);
							Y = Y+(float) (distance)*(float) Math.sin(Angle*Math.PI/180);
							nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
							return true;	
						}
						
					}
				}

			}

		}
		return false;
		
		}
	float preciseDetection(CorrectionPilot pilot, UltrasonicSensor usensor, Navigator nav, GyroSensor gyroSensor) {
		pilot.setLinearSpeed(100);
		pilot.setAngularSpeed(15);
		pilot.setLinearAcceleration(10);
		pilot.setAngularAcceleration(10);

		double distance=usensor.getDistance();
		double dist=0;
		
		/* while the robot is rotating, it takes distance samples from its ultrasonic sensor. If the distance measured is larger than the previous 
		 * sample and is smaller than one meter, the rotation stops. This means the robot detected an object. In this case the robot travels towards 
		 * the object. If no object is detected the method returns zero. 
		 */
		int i = 0;
		pilot.setAngularSpeed(10);
		pilot.rotateImmediateReturn(360, true);
		distance=3;
		 //counter for 1st measurement
		while(pilot.isMoving()&&Button.ESCAPE.isUp()){
			i =i+1;
			dist = distance; //=3
			distance=usensor.getDistance(); //new: .22
			
			if (dist>distance & !(dist==0) & distance<0.5 & i==1){
				System.out.println("object at first sight");
				pilot.rotateImmediateReturn(360, true);
				System.out.println("goingLeft");
				while(pilot.isMoving()&&Button.ESCAPE.isUp()){
					dist = distance;
					distance=usensor.getDistance();
					if(distance>safetyfactor*dist) {
						pilot.stop();
						startAngle = gyroSensor.getAngle();
						startAngle2 = (int) gyroSensor.getAngle();
						System.out.println("StartAngle: " +startAngle);
						System.out.println("StartAngle2: "+startAngle2);
					}
				}
			distance = 3;
			pilot.rotateImmediateReturn(-720, true);
			Delay.msDelay(2000);
			System.out.println("GoingRight");
			while(pilot.isMoving()&&Button.ESCAPE.isUp()){
				dist = distance;
				distance=usensor.getDistance();
				if(distance>safetyfactor*dist) {
					pilot.stop();
					endAngle = gyroSensor.getAngle();
					System.out.println("endAngle: "+ endAngle);
					System.out.println("startAngle: "+startAngle);
					System.out.println("Ready To Center");
					pilot.rotate((startAngle-endAngle)/2, true);
					float X = nav.getPoseProvider().getPose().getX();
					float Y = nav.getPoseProvider().getPose().getY();
					nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
					return (float) usensor.getDistance();
				}
			}
			}
			System.out.println("Distance: " + distance);
			if(dist>distance & !(dist==0) & distance<.5 & i !=1){
				System.out.println("Option 2 ");
				//If we detect an object, keep turning until object is out of sight.
				// Rotate back half of the rotated angle: The robot should be oriented in front of the object.
				startAngle = gyroSensor.getAngle();
				System.out.println("StartAngle: "+ endAngle);
				pilot.rotateImmediateReturn(140, true);
				while(pilot.isMoving()&&Button.ESCAPE.isUp()){
					System.out.println("Going Further2");
					dist = distance;
					distance=usensor.getDistance();
					if(distance>safetyfactor*dist) {
						pilot.stop();
						endAngle = gyroSensor.getAngle();
						System.out.println("EndAngle: "+ endAngle);
						pilot.rotate(-(endAngle-startAngle)/2, true);
						System.out.println("Turned");
						float X = nav.getPoseProvider().getPose().getX();
						float Y = nav.getPoseProvider().getPose().getY();
						nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
						return (float) usensor.getDistance();
					}
				}

			}

		}
		
		return 0;
		
	
		
	}
}
