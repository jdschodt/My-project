	

	
	import lejos.hardware.port.SensorPort;
	import lejos.hardware.sensor.EV3GyroSensor;
	import lejos.robotics.SampleProvider;
	import lejos.utility.Delay;

	public class Gyrosensor {
	

		//Robot Configuration
		private static EV3GyroSensor gyroSensor1 = new EV3GyroSensor(SensorPort.S1);

		//Configuration
		private static int HALF_SECOND = 1000;

		public static void main(String[] args) {

			final SampleProvider sp = gyroSensor1.getAngleMode();
			int value = 0;

	        //Control loop
	        final int iteration_threshold = 20;
	        for(int i = 0; i <= iteration_threshold; i++) {

	        	float [] sample = new float[sp.sampleSize()];
	            sp.fetchSample(sample, 0);
	            value = (int)sample[0];

				//System.out.println("Iteration: " + i);
				System.out.println("Gyro angle: " + value);

	            Delay.msDelay(HALF_SECOND);
	        }

			
		}

	}	
	
	


