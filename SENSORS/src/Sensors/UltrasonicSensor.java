package Sensors;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.utility.Delay;


public class UltrasonicSensor {
	private static EV3UltrasonicSensor sonar;
	private static SampleProvider distanceProvider;
	private static float[] DistanceSample;
	
	public UltrasonicSensor() {
		Port s1 = LocalEV3.get().getPort("S1");
		sonar = new EV3UltrasonicSensor(s1);
		distanceProvider = sonar.getDistanceMode();
		DistanceSample = new float [distanceProvider.sampleSize()];
		System.out.println("US configured");
	    Delay.msDelay(1000);
	}
	public double getDistance() {
		distanceProvider.fetchSample(DistanceSample, 0);
		double distance=(double)DistanceSample[0];
		return distance;
	}
	
}
