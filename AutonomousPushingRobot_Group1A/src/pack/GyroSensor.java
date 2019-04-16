package pack;

import lejos.ev3.tools.FileDrop.TransferableObject.Fetcher;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.Gyroscope;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class GyroSensor {

	EV3GyroSensor gyroSensor;
	SampleProvider gyroProvider;
	float[] gyroSample;

	public GyroSensor(){
		Port s3 = LocalEV3.get().getPort("S3");
		gyroSensor = new EV3GyroSensor(s3);
		gyroSensor.reset();
		gyroProvider=gyroSensor.getAngleMode();
		gyroSample = new float [gyroProvider.sampleSize()];
	}
		
	public float getAngle(){
		gyroProvider.fetchSample(gyroSample, 0);
		double Angle = gyroSample[0];
				
		return (float) Angle;
	}
	
	public float getHeadingAngle(){
		gyroProvider.fetchSample(gyroSample, 0);
		
		double Angle = gyroSample[0];

		return (float) normalize(Angle);
		

	}
		
	public void close(){
		gyroSensor.close();
	}
		
	public double normalize(double angle)
	{
	    double newAngle = angle;
	    while (newAngle <= -180) newAngle += 360.0;
	    while (newAngle > 180) newAngle -= 360.0;
	    return newAngle;
	}
	
}
	
