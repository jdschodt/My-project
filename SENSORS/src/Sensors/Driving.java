package Sensors;
import java.util.Random;

import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.PoseProvider;
import Sensors.CompassPoseProvider;
import lejos.robotics.navigation.Move;
import lejos.robotics.navigation.MoveListener;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.MoveProvider;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import Sensors.HeadingCorrectionNavigator;

import lejos.utility.Delay;


public class Driving {
	private static MovePilot pilot;
	private static Navigator navigator;
	private static PoseProvider provider;
	public Driving(GyroSensor gyro) {
		float d=5.771501925545571245186136071887f;//Diameter of the wheels
		float y=9.3f;
		Wheel leftWheel=WheeledChassis.modelWheel(Motor.A, d).offset(-y);
		Wheel rightWheel=WheeledChassis.modelWheel(Motor.B, d).offset(y);
		Chassis chassis = new WheeledChassis(new Wheel[] {leftWheel,rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);
		pilot=new MovePilot(chassis);
		navigator= new Navigator(pilot);
		provider=navigator.getPoseProvider();
		chassis.setLinearSpeed(20);
		chassis.setAngularSpeed(20);
		chassis.setLinearAcceleration(5);
		chassis.setAngularAcceleration(30);
		System.out.println("Pilot configured");
		Delay.msDelay(200);
	}
	public void reset() {
		provider.setPose(new Pose(0,0,0));
	}
	public void setLocation(Waypoint waypoint) {
		double X=waypoint.getX();
		double Y=waypoint.getY();
		double H=waypoint.getHeading();
		provider.setPose(new Pose((float)X,(float)Y,(float)H));
	}
	public void driveTo(int x, int y) {
		navigator.goTo(x, y);

		while (navigator.isMoving()) {
			Delay.msDelay(100);
		}

	}
	public void driveTo(Waypoint waypoint) {
		navigator.goTo(waypoint);
		while (navigator.isMoving()) {
			Delay.msDelay(100);
		}
		
	}
	public void goToTravel(int distance) {

		double Angle = provider.getPose().getHeading();
		double X = 0;
		double Y = 0;
		double posX = provider.getPose().getX();
		double posY = provider.getPose().getY();

		pilot.travel(distance);

		while (pilot.isMoving()) {
		Delay.msDelay(100);
		}

		X = posX + Math.cos(Angle) * distance;
		Y = posY + Math.sin(Angle) * distance;

		provider.getPose().setLocation((float) X, (float) Y);
		

	}		
		
	public void getPose() {
		System.out.println("Heading: " +provider.getPose().getHeading());
		System.out.println("X: " +provider.getPose().getX());
		System.out.println("Y: " +provider.getPose().getY());
	}
		
				
	public void setHeading(GyroSensor gyro) {
		provider.getPose().setHeading((float)gyro.getAngle());
		
	}
	public void addWaypoint(Waypoint awaypoint) {
		
	}
	public void rotate(double angle) {
		this.rotate(angle);
		double h= provider.getPose().getHeading()+angle;
		provider.getPose().setHeading((float)h);
	}
	
}
