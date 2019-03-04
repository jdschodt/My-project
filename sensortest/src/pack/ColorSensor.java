package pack;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;


public class ColorSensor {
	EV3ColorSensor colorSensor;
	SampleProvider colorProvider;
	float[] colorSample;
	
	public ColorSensor(){
		Port s2 = LocalEV3.get().getPort("S2");
		colorSensor = new EV3ColorSensor(s2);
		colorProvider = colorSensor.getColorIDMode();
		colorSample = new float [colorProvider.sampleSize()];
	}
	
	public float getColor(){
			colorProvider.fetchSample(colorSample, 0);
			return colorSample[0];
		}
	
	public void close() {
		colorSensor.close();
		
	}
}

