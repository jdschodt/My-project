package pack;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class UltrasonicSensor {
	EV3UltrasonicSensor uSensor;
	SampleProvider uprov;
	float[] distanceSample2;
	
	public UltrasonicSensor(){
		uSensor = new EV3UltrasonicSensor(SensorPort.S1);
		distanceSample2 = new float [uSensor.getDistanceMode().sampleSize()];

	}
	public float getDistance(){
		uSensor.getDistanceMode().fetchSample(distanceSample2, 0);
		return distanceSample2[0];
	}
	
}