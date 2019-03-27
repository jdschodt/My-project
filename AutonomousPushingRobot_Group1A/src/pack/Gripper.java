package pack;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

import java.util.ArrayList;


public class Gripper {

//	static EV3MediumRegulatedMotor Motor = new EV3MediumRegulatedMotor(MotorPort.C);
//	static boolean stalledIN = false;
//	static boolean stalledOUT = false;
	static boolean objectPresent = false;
	
	
	public void grip () {
		
		/* while the motor is moving, it checks if it is stalled. If the motor is stalled, the motor stops and the object is gripped. */
				
//		stalledIN = false;
//		Motor.setStallThreshold(6,100);
//		
//		Motor.forward();
//		while (!stalledIN && Button.ESCAPE.isUp()){
//			
//			Delay.msDelay(250);
//			
//			if (Motor.isStalled()){
//				Motor.stop();
//				stalledIN = true;
//			}
//		}
		objectPresent = true;
		System.out.println("kaka pipi");
//		stalledOUT = false;
	}
	
	
	
	public void release() {
		
		/* while the motor is moving, it checks if it is stalled. If the motor is stalled, the motor stops and the object is released. */
//			
//		stalledOUT = false;
//		Motor.setStallThreshold(8,100);
//	
//		Motor.backward();
//		while (!stalledOUT && Button.ESCAPE.isUp()){
//			Delay.msDelay(250);
//			if (Motor.isStalled()){
//				Motor.stop();
//				stalledOUT = true;
//			}
//		}			
		objectPresent = false;
//		stalledIN = false;
	}
	
	
	
	public boolean Object(){
		return objectPresent;	
	}	
}
	
	
		
	

