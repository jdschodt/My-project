package Sensors;


import java.awt.Point;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Waypoint;
import lejos.utility.Delay;

public class HeadingCorrectionNavigator extends Navigator{
	MovePilot pilot;
	GyroSensor gyro;
	static float error;
	
	public HeadingCorrectionNavigator(MovePilot pilot, PoseProvider poseProvider) {
		super(pilot,poseProvider);
		this.pilot=pilot;
		System.out.println("HeadingCorrectionNavigator");
		
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
		error=super.getPoseProvider().getPose().getHeading();
		System.out.println("error"+ error);
		Delay.msDelay(50);
		pilot.rotate(angle, false);
		System.out.println("Draaihoek: "+angle);
		Delay.msDelay(50);
		error=normalize(angle)-super.getPoseProvider().getPose().getHeading()+error;
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
		float targetAngle=super.getPoseProvider().getPose().getHeading();
		pilot.travel(distance, true);
		while (pilot.isMoving()){
			Delay.msDelay(50);
			error=targetAngle-super.getPoseProvider().getPose().getHeading();
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
		float destinationRelativeBearing=normalize(super.getPoseProvider().getPose().angleTo(destination));
		float distance = super.getPoseProvider().getPose().distanceTo(destination);
		rotate(destinationRelativeBearing);
		travel(distance);
	}
}
