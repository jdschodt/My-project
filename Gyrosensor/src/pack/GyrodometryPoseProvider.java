package pack;
import Sensors.GyroSensor;
import lejos.hardware.Sound;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Move;
import lejos.robotics.navigation.MoveListener;
import lejos.robotics.navigation.MoveProvider;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.utility.Delay;


public class GyrodometryPoseProvider implements PoseProvider, MoveListener {
	private float x=0,y=0,heading=0;
	private float angle0, distance0, angleGyro0;
	private MoveProvider mp;
	private boolean current=true;
	private GyroSensor gyro;
	private Regulator reg= new Regulator();
	
	
	public GyrodometryPoseProvider(MoveProvider mp, GyroSensor gyro) {
		mp.addMoveListener(this);
		this.gyro=gyro;
		reg.start();
		System.out.println("GyrodometryPose");
	}
	
	private class Regulator extends Thread {
		public Regulator() {
			setDaemon(true);
		}
		public void run() {
			while(true) {
			Delay.msDelay(1000);
			updatePose(mp.getMovement());
			
			}
		}
	}
	public Pose getPose() {
		if (!current) {
			updatePose(mp.getMovement());
		}
		return new Pose(x,y, heading);
	}
	public void moveStarted(Move event, MoveProvider mp) {
		angle0=0;
		distance0=angle0;
		current=false;
		this.mp=mp;
	}
	public void moveStopped(Move event, MoveProvider mp) {
		updatePose(mp.getMovement());
		
	}
	private void updatePose(Move event) {
		float headGyro=(float)gyro.getAngle()-angleGyro0;
		float angle=event.getAngleTurned()-angle0;
		float error=angle-headGyro;
		if(Math.abs(error)>1) {
			Sound.beep();
			angle=angle-error;
		}
		float distance=event.getDistanceTraveled()-distance0;
		double dx=0, dy=0;
		double headingRad=(Math.toRadians(heading));
		if (event.getMoveType()==Move.MoveType.TRAVEL||Math.abs(angle)<0.2f) {
			dx=(distance)*(float)Math.cos(headingRad);
			dy=(distance)*(float)Math.sin(headingRad);
		}
		else if (event.getMoveType()==Move.MoveType.ARC) {
			double angleRad=Math.toRadians(angle);
			double radius=distance/angleRad;
			dy=radius*(Math.cos(headingRad)-Math.cos(headingRad+angleRad));
			dx=radius*(Math.sin(headingRad+angleRad)-Math.sin(headingRad));			
		}
		x+=dx;
		y+=dy;
		heading=normalize(heading+angle);
		angle0=event.getAngleTurned();
		angleGyro0=(float)gyro.getAngle();
		distance0=event.getDistanceTraveled();
		current=!event.isMoving();
		
		
		
	}
	private float normalize(float angle) {
		while (angle>180) {
			angle-=360;
		}
		while (angle<-180) {
			angle+=360;
		}
		return angle;
	}
	public void setPosistion(Waypoint p) {
		x=p.x;
		y=p.y;
		current=true;
		
	}
	public void setHeading(float heading) {
		this.heading=heading;
		current=true;
	}
	@Override
	public void setPose(Pose aPose) {
		// TODO Auto-generated method stub
		
	}
}

	