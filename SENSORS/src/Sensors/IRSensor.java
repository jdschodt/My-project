package Sensors;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.utility.Delay;

public class IRSensor {
	private static EV3IRSensor IR;
	private static SampleProvider distanceProvider;
	private static SampleProvider beaconLocator;
	private static float[] DistanceSample;
	private static float[] BeaconSample;
	private static double distance;
public IRSensor() {
	Port s2 = LocalEV3.get().getPort("S2");
	IR = new EV3IRSensor(s2);
	distanceProvider = IR.getDistanceMode();
	DistanceSample = new float [distanceProvider.sampleSize()];
	beaconLocator=IR.getSeekMode();
	BeaconSample= new float[beaconLocator.sampleSize()];
	System.out.println("IR configured");
    Delay.msDelay(1000);
    
}
public double getDistance() {
	distanceProvider.fetchSample(DistanceSample, 0);
	distance=(double)DistanceSample[0];
	return distance;
}
public float[] getBeacon() {
	beaconLocator.fetchSample(BeaconSample, 0);
	return BeaconSample;
}
}
