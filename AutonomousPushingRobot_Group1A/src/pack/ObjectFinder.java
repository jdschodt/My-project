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
	public double startAngle;
	public double endAngle;
	public float distance;

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
		boolean NoDetection=true;
		StartAngle=(int) gyroSensor.getAngle();
		
		/* while the robot is rotating, it takes distance samples from its ultrasonic sensor. If the distance measured is larger than the previous 
		 * sample and is smaller than one meter, the rotation stops. This means the robot detected an object. In this case the robot travels towards 
		 * the object. If no object is detected the method returns zero. 
		 */

		pilot.setAngularSpeed(10);
		pilot.rotateImmediateReturn(angleToRotate, true);
		distance=usensor.getDistance();
		int i = 0; //counter for 1st measurement
		while(pilot.isMoving()&&Button.ESCAPE.isUp()){
			i =i+ 1;
			dist = distance;
			distance=usensor.getDistance();
			if (dist<distance & !(dist==0) & distance<1 & i==1){
				System.out.println("object at first sight");
				pilot.rotateImmediateReturn(80, true);
				while(pilot.isMoving()&&Button.ESCAPE.isUp()){
					dist = distance;
					distance=usensor.getDistance();
					if(distance>1.5*dist) {
						pilot.stop();
						double startAngle = gyroSensor.getHeadingAngle();
					}
				}
			pilot.rotateImmediateReturn(-140, true);
			while(pilot.isMoving()&&Button.ESCAPE.isUp()){
				dist = distance;
				distance=usensor.getDistance();
				if(distance>1.5*dist) {
					pilot.stop();
					double endAngle = gyroSensor.getHeadingAngle();
					pilot.rotateImmediateReturn(endAngle-startAngle, true);
					float X = nav.getPoseProvider().getPose().getX();
					float Y = nav.getPoseProvider().getPose().getY();
					double Angle = gyroSensor.getHeadingAngle();
					distance = distance * 1000;
					pilot.travel((int) (distance),false);
					
					X = X+(float) (distance)*(float) Math.cos(Angle*Math.PI/180);
					Y = Y+(float) (distance)*(float) Math.sin(Angle*Math.PI/180);
	
					nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
					Delay.msDelay(500);
					
					NoDetection = false;
					distanceTravelled= distance;
					return (float) distance;
				}
			}
			}
			System.out.println("Distance: " + distance);
			if(dist<distance & !(dist==0) & distance<1){ //????
				//If we detect an object, keep turning until object is out of sight.
				// Rotate back half of the rotated angle: The robot should be oriented in front of the object.
				double startAngle = gyroSensor.getHeadingAngle();
				pilot.rotateImmediateReturn(80, true);
				while(pilot.isMoving()&&Button.ESCAPE.isUp()){
					dist = distance;
					distance=usensor.getDistance();
					if(distance>1.5*dist) {
						pilot.stop();
						double endAngle = gyroSensor.getHeadingAngle();
						pilot.rotateImmediateReturn(endAngle-startAngle, true);
						float X = nav.getPoseProvider().getPose().getX();
						float Y = nav.getPoseProvider().getPose().getY();
						double Angle = gyroSensor.getHeadingAngle();
						distance = distance * 1000;
						pilot.travel((int) (distance),false);
						
						X = X+(float) (distance)*(float) Math.cos(Angle*Math.PI/180);
						Y = Y+(float) (distance)*(float) Math.sin(Angle*Math.PI/180);
		
						nav.getPoseProvider().setPose(new Pose(X,Y,gyroSensor.getAngle()));
						Delay.msDelay(500);
						
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
		
		}
}
