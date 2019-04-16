package Sensors;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class GyroSensor {
	private static EV3GyroSensor gyro;
	private static SampleProvider angleProvider;
	private static float[] angleSample;
	private static double[]measurement=new double[10];
	private static double angle;
	public GyroSensor() {
		Port s4 = LocalEV3.get().getPort("S3");
		gyro = new EV3GyroSensor(s4);
		angleProvider = gyro.getAngleMode();
		angleSample = new float [angleProvider.sampleSize()];
		System.out.println("Angle: "+this.getAngle());
		System.out.println("Gyro configured");
	    Delay.msDelay(1000);
	}
	public double getAngle() {
		angleProvider.fetchSample(angleSample, 0);
		for(int i=0;i<10;i++) {
			angleProvider.fetchSample(angleSample, 0);
			measurement[i]=(double)angleSample[0];			
		}
		angle=0;
		for (int i=0;i<10;i++){
			angle+=measurement[i];
		}
		angle=angle/10;
		while(angle>180) {
			angle-=180;
		}
		while(angle<-180) {
			angle+=180;
		}

		
		return angle;
	}
	public void reset() {
		gyro.reset();
	}
	
}
