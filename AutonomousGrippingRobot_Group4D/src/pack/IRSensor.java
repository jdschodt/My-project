package pack;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.HiTechnicIRSeekerV2;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;
import lejos.utility.Delay;

public class IRSensor {
	
	EV3IRSensor irSensor;
	HiTechnicIRSeekerV2 seeker;
	float[] distanceSample;
	float[] beaconSample;
	SampleProvider averager;
	float[] sample;
	
	


	public IRSensor(){
		Port s4 = LocalEV3.get().getPort("S4");
		irSensor = new EV3IRSensor(s4);
		distanceSample = new float [irSensor.getDistanceMode().sampleSize()];
		beaconSample = new float [irSensor.getSeekMode().sampleSize()];
		averager = new MeanFilter(irSensor.getSeekMode(),10);
		sample = new float[averager.sampleSize()];

	}
	
	public float getDistanceToObject (){
		irSensor.getDistanceMode().fetchSample(distanceSample, 0);
		
		return distanceSample[0];
	}
	

	public float getRelationToBeacon(int i){
		averager.fetchSample(sample, 0);
	
		return sample[i];
	}
	

	
	public void close(){
		irSensor.close();
	}

}
