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
import lejos.robotics.navigation.Waypoint;
import lejos.utility.Delay;

public class ObjectFinder {
	public double distanceTravelled=0;
	public int StartAngle= 0;
	public int angleAfterUltrasoon;
	double startAngle;
	double endAngle;
	public float distance;
	public boolean NoDetection=true;
	public int startAngle2;
	public boolean directDetection = false;


	public float findObject(IRSensor ir, CorrectionPilot pilot, UltrasonicSensor usensor, Navigator nav, GyroSensor gyroSensor, int angleToRotate ){	
		/* this method detects if there are any objects in a circle with a radius of one meter around the robot. If an object is found, the method 
		 * returns the distance to this object. If no object is found, the method returns zero.
		 */

		pilot.setLinearSpeed(50);
		pilot.setAngularSpeed(10);
		pilot.setLinearAcceleration(300);
		pilot.setAngularAcceleration(300);

		double distance=usensor.getDistance();
		double dist=0;
		
		StartAngle=(int) gyroSensor.getAngle();
		
		/* while the robot is rotating, it takes distance samples from its ultrasonic sensor. If the distance measured is larger than the previous 
		 * sample and is smaller than one meter, the rotation stops. This means the robot detected an object. In this case the robot travels towards 
		 * the object. If no object is detected the method returns zero. 
		 */
		int i = 0;
		pilot.setAngularSpeed(10);
		pilot.rotateImmediateReturn(angleToRotate, true);
		distance=3;
		 //counter for 1st measurement
		while(pilot.isMoving()&&Button.ESCAPE.isUp()){
			i =i+1;
			dist = distance; //=3
			distance=usensor.getDistance(); //new: .22
			
			if (dist>distance & !(dist==0) & distance<0.5 & i==1){
				System.out.println("object at first sight");
				pilot.rotateImmediateReturn(360, true);
				System.out.println("goingLeft...");
				while(pilot.isMoving()&&Button.ESCAPE.isUp()){
					dist = distance;
					distance=usensor.getDistance();
					if(distance>2*dist) {
						pilot.stop();
						startAngle = gyroSensor.getAngle();
						startAngle2 = (int) gyroSensor.getAngle();
						System.out.println("StartAngle: " +startAngle);
						System.out.println("StartAngle2: "+startAngle2);
						
						Delay.msDelay(2000);
					}
				}
			distance = 3;
			pilot.rotateImmediateReturn(-360, true);
			System.out.println("GoingRight...");
			Delay.msDelay(2000);
			while(pilot.isMoving()&&Button.ESCAPE.isUp()){
				dist = distance;
				distance=usensor.getDistance();
				if(distance>2*dist) {
					pilot.stop();
					endAngle = gyroSensor.getAngle();
					System.out.println("endAngle: "+ endAngle);
					System.out.println("startAngle: "+startAngle);
					System.out.println("Ready To Center");
					Delay.msDelay(1000);
					pilot.rotate((startAngle-endAngle)/2, true);
					float X = nav.getPoseProvider().getPose().getX();
					float Y = nav.getPoseProvider().getPose().getY();
					double Angle = gyroSensor.getHeadingAngle();
					distance=usensor.getDistance();
					distance = distance * 500; //usensor is in meters, while naviation in milimeters
					pilot.travel((int) (distance),false);
					
					X = X+(float) (distance)*(float) Math.cos(Angle*Math.PI/180);
					Y = Y+(float) (distance)*(float) Math.sin(Angle*Math.PI/180);
	
					nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
					Delay.msDelay(500);
//					
//					distance = preciseDetection(ir, pilot, usensor, nav, gyroSensor);
//					distance = distance * 1000-200;
//					pilot.travel((int) (distance),false);
//					
//					X = X+(float) (distance)*(float) Math.cos(Angle*Math.PI/180);
//					Y = Y+(float) (distance)*(float) Math.sin(Angle*Math.PI/180);
//	
//					nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
					
					NoDetection = false;
					distanceTravelled= distance;
					return (float) distance;
				}
			}
			}
			System.out.println("Distance: " + distance);
			if(dist>distance & !(dist==0) & distance<.5 & i !=1){
				System.out.println("Option 2 ");
				Delay.msDelay(2000);
				//If we detect an object, keep turning until object is out of sight.
				// Rotate back half of the rotated angle: The robot should be oriented in front of the object.
				startAngle = gyroSensor.getAngle();
				System.out.println("StartAngle: "+ endAngle);
				Delay.msDelay(1000);
				pilot.rotateImmediateReturn(140, true);
				System.out.println("Going Further...");
				while(pilot.isMoving()&&Button.ESCAPE.isUp()){
					dist = distance;
					distance=usensor.getDistance();
					if(distance>2*dist) {
						pilot.stop();
						endAngle = gyroSensor.getAngle();
						System.out.println("EndAngle: "+ endAngle);
						System.out.println("Ready To Center");
						pilot.rotate(-(endAngle-startAngle)/2, true);

						distance=usensor.getDistance();
						float X = nav.getPoseProvider().getPose().getX();
						float Y = nav.getPoseProvider().getPose().getY();
						double Angle = gyroSensor.getHeadingAngle();
						distance = distance * 500;
						pilot.travel((int) (distance),false);
						
						X = X+(float) (distance)*(float) Math.cos(Angle*Math.PI/180);
						Y = Y+(float) (distance)*(float) Math.sin(Angle*Math.PI/180);
		
						nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
						Delay.msDelay(500);
						distance = preciseDetection(ir, pilot, usensor, nav, gyroSensor);
//						
//						distance = distance * 1000-200;
//						pilot.travel((int) (distance),false);
//						
//						X = X+(float) (distance)*(float) Math.cos(Angle*Math.PI/180);
//						Y = Y+(float) (distance)*(float) Math.sin(Angle*Math.PI/180);
//		
//						nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
						
						NoDetection = false;
						distanceTravelled= distance;
						return (float) distance;
					}
				}

			}

		}
		if (NoDetection){
			return 0;
		}
		return (float) distance;
		}
	float preciseDetection(IRSensor ir, CorrectionPilot pilot, UltrasonicSensor usensor, Navigator nav, GyroSensor gyroSensor) {
		pilot.setLinearSpeed(50);
		pilot.setAngularSpeed(10);
		pilot.setLinearAcceleration(300);
		pilot.setAngularAcceleration(300);

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
					System.out.println(distance);
					if(distance>2*dist) {
						pilot.stop();
						startAngle = gyroSensor.getAngle();
						startAngle2 = (int) gyroSensor.getAngle();
						System.out.println("StartAngle: " +startAngle);
						System.out.println("StartAngle2: "+startAngle2);
						
						Delay.msDelay(2000);
					}
				}
			distance = 3;
			pilot.rotateImmediateReturn(-720, true);
			Delay.msDelay(2000);
			while(pilot.isMoving()&&Button.ESCAPE.isUp()){
				dist = distance;
				distance=usensor.getDistance();
				System.out.println("GoingRight");
				if(distance>2*dist) {
					pilot.stop();
					endAngle = gyroSensor.getAngle();
					System.out.println("endAngle: "+ endAngle);
					System.out.println("startAngle: "+startAngle);
					Delay.msDelay(1000);
					System.out.println("Ready To Center");
					Delay.msDelay(1000);
					pilot.rotate((startAngle-endAngle)/2, true);
					float X = nav.getPoseProvider().getPose().getX();
					float Y = nav.getPoseProvider().getPose().getY();
					double Angle = gyroSensor.getHeadingAngle();
					distance=usensor.getDistance();
					distance = distance * 1000;
					pilot.travel((int) (distance),false);
					
					X = X+(float) (distance)*(float) Math.cos(Angle*Math.PI/180);
					Y = Y+(float) (distance)*(float) Math.sin(Angle*Math.PI/180);
	
					nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
					Delay.msDelay(500);
//					
//					distance = preciseDetection(ir, pilot, usensor, nav, gyroSensor);
//					distance = distance * 1000-200;
//					pilot.travel((int) (distance),false);
//					
//					X = X+(float) (distance)*(float) Math.cos(Angle*Math.PI/180);
//					Y = Y+(float) (distance)*(float) Math.sin(Angle*Math.PI/180);
//	
//					nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
					
					NoDetection = false;
					distanceTravelled= distance;
					return (float) distance;
				}
			}
			}
			System.out.println("Distance: " + distance);
			if(dist>distance & !(dist==0) & distance<.5 & i !=1){
				System.out.println("Option 2 ");
				Delay.msDelay(2000);
				//If we detect an object, keep turning until object is out of sight.
				// Rotate back half of the rotated angle: The robot should be oriented in front of the object.
				startAngle = gyroSensor.getAngle();
				System.out.println("StartAngle: "+ endAngle);
				Delay.msDelay(1000);
				pilot.rotateImmediateReturn(140, true);
				while(pilot.isMoving()&&Button.ESCAPE.isUp()){
					System.out.println("Going Further2");
					dist = distance;
					distance=usensor.getDistance();
					if(distance>2*dist) {
						pilot.stop();
						endAngle = gyroSensor.getAngle();
						System.out.println("EndAngle: "+ endAngle);
						pilot.rotate(-(endAngle-startAngle)/2, true);
						System.out.println("Turned");
						distance=usensor.getDistance();
						float X = nav.getPoseProvider().getPose().getX();
						float Y = nav.getPoseProvider().getPose().getY();
						double Angle = gyroSensor.getHeadingAngle();
						distance = distance * 1000;
						pilot.travel((int) (distance),false);
						
						X = X+(float) (distance)*(float) Math.cos(Angle*Math.PI/180);
						Y = Y+(float) (distance)*(float) Math.sin(Angle*Math.PI/180);
		
						nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
						Delay.msDelay(500);
//						distance = preciseDetection(ir, pilot, usensor, nav, gyroSensor);
//						
//						distance = distance * 1000-200;
//						pilot.travel((int) (distance),false);
//						
//						X = X+(float) (distance)*(float) Math.cos(Angle*Math.PI/180);
//						Y = Y+(float) (distance)*(float) Math.sin(Angle*Math.PI/180);
//		
//						nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
						
						NoDetection = false;
						distanceTravelled= distance;
						return (float) distance;
					}
				}

			}

		}
		
		return 0;
		
	
		
	}
}
