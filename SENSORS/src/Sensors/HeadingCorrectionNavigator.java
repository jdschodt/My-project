package Sensors;


import java.awt.Point;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Waypoint;
import lejos.utility.Delay;

public class HeadingCorrectionNavigator extends Navigator{
<<<<<<< HEAD
	MovePilot pilot;
	GyroSensor gyro;
=======
	CorrectionPilot pilot;
>>>>>>> f10c067f7aaba61436809129a5c5d3eb06c9edd8
	static float error;
	private static CompassPoseProvider provider;
	
	public HeadingCorrectionNavigator(CorrectionPilot pilot, CompassPoseProvider aposeProvider) {
		super(pilot);
		provider=aposeProvider;
		this.pilot=pilot;
		System.out.println("HeadingCorrectionNavigator");
		Delay.msDelay(1000);
		
	}
	public float normalize(float angle) {
		while (angle>180) {
			angle-=360;
		}
		while (angle<-180) {
			angle+=360;
		}
		return angle;
	}
	public void rotate(float angle) {
<<<<<<< HEAD
		error=super.getPoseProvider().getPose().getHeading();
		System.out.println("error"+ error);
=======
		error=provider.getPose().getHeading();
>>>>>>> f10c067f7aaba61436809129a5c5d3eb06c9edd8
		Delay.msDelay(50);
		pilot.rotate(angle, false);
		System.out.println("Draaihoek: "+angle);
		Delay.msDelay(50);
		error=normalize(angle)-provider.getPose().getHeading()+error;
		System.out.println("Nieuwe_err: " + error);
		Delay.msDelay(50);		
		System.out.println("error:"+error);
		//while (super.getPoseProvider().getPose().getHeading()-gyro.getAngle()>2) {
			//System.out.println("Corrigeren...");
			//pilot.rotate(super.getPoseProvider().getPose().getHeading()-gyro.getAngle());
			//}
		
		if(Math.abs(error)>1) {
			rotate(normalize(error));
		}
	}
	public void travel(float distance) {
		float targetAngle=provider.getPose().getHeading();
		pilot.travel(distance, true);
		System.out.println("Distancetraveled");
		while (pilot.isMoving()){
			Delay.msDelay(50);
			error=targetAngle-provider.getPose().getHeading();
			if (Math.abs(error)>3) {
				float traveledDistance=pilot.getMovement().getDistanceTraveled();
				pilot.stop();
				rotate(normalize(error));
				travel(distance-traveledDistance);
				
				
			}
		}
	}
	@Override
	public void goTo (Waypoint destination) {
		float destinationRelativeBearing=normalize(provider.getPose().angleTo(destination));
		float distance = provider.getPose().distanceTo(destination);
		rotate(destinationRelativeBearing);
		travel(distance);
		System.out.println("Go to");
	}
}
