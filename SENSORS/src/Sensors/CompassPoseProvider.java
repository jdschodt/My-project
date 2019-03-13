package Sensors;

import lejos.robotics.navigation.MoveProvider;
import lejos.robotics.navigation.Pose;

/**
 * Pose Provider using a compass (or other direction finder) to provide 
 * location and heading data.
 * 
 * Note: This is a temporary class to allow access compass data until we have
 * a more encompassing solution for data from multiple instrumentation.
 * @author BB
 *
 */
public class CompassPoseProvider extends OdometryPoseProvider {

	private GyroSensor gyro;
	
	public CompassPoseProvider(MoveProvider mp, GyroSensor agyro) {
		super(mp);
		this.gyro = agyro;
	}

	public Pose getPose() {
		Pose temp = super.getPose();
		temp.setHeading((float)gyro.getAngle());
		return temp;
	}
}