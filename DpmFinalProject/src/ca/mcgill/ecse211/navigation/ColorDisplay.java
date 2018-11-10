package ca.mcgill.ecse211.navigation;

import java.util.HashMap;
import java.util.Map;

import lejos.hardware.Sound;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.navigation.*;


public class ColorDisplay implements Runnable{
	private TextLCD lcd;
	private SampleProvider ring_color_sample_provider;
	private float[] usData;
	private SampleProvider usDistance;
	
	private float[] color_samples;

	
	//under demo conditions
	private int[] greenRing_1 = {63283, 143777, 14267, 12707, 24629, 4062};
	private int[] orangeRing_1 = {120244, 41029, 7697, 23859, 9602, 2545};
	private int[] blueRing_1 = {30487, 127991, 78627, 6892, 18154, 10471};
	private int[] yellowRing_1 = {149021, 113338, 15292, 26255, 18191, 3927};
	
	
	private int redS, greenS, blueS;
	private long timeout = Long.MAX_VALUE;
	private boolean keepLooking = true;
	
	
	
	public ColorDisplay(TextLCD lcd) {
		this.lcd = lcd; 
		SensorModes ringSensor = MainClass.ringSensor;
		this.ring_color_sample_provider = ringSensor.getMode("RGB");
		this.color_samples = new float[ring_color_sample_provider.sampleSize()]; // ring sensor
		
		SensorModes usSensor = MainClass.usSensor; // usSensor is the instance
		this.usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		// this instance
		this.usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
		// returned

		
	}
	public ColorDisplay(TextLCD lcd, long timeout) {
		this.lcd = lcd; 
		this.timeout = timeout;
		SensorModes ringSensor = MainClass.ringSensor;
		this.ring_color_sample_provider = ringSensor.getMode("RGB");
		this.color_samples = new float[ring_color_sample_provider.sampleSize()]; // ring sensor
		
		SensorModes usSensor = MainClass.usSensor; // usSensor is the instance
		this.usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
		// this instance
		this.usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
		// returned
	}
	
	public void run(){
		lcd.clear();
		long updateStart, updateEnd;
		long tStart = System.currentTimeMillis();
		int displayPeriod = 25;
		
		
		while (true) { // avoiding the obstacles
			usDistance.fetchSample(usData, 0);
			float distance = usData[0] * 100;
			if(distance<6){
				fetchLightData();
				do {
					updateStart = System.currentTimeMillis();
					
					lcd.drawString("Object detected", 0, 0);
					fetchLightData();
					if(Math.abs(greenRing_1[0] - redS) <= (int) (2 * greenRing_1[3]) &&
							Math.abs(greenRing_1[1] - greenS) <= (int) (2 * greenRing_1[4]) &&
							Math.abs(greenRing_1[2] - blueS) <= (int) (2 * greenRing_1[5]))
					{
						lcd.drawString("Green Ring Detected", 0, 1);
					}
					
					else if(Math.abs(orangeRing_1[0] - redS) <= (int) (2 * orangeRing_1[3]) &&
							Math.abs(orangeRing_1[1] - greenS) <= (int) (2 * orangeRing_1[4]) &&
							Math.abs(orangeRing_1[2] - blueS) <= (int) (2 * orangeRing_1[5]))
					{
						lcd.drawString("Orange Ring Detected", 0, 2);
					}
					
					else if(Math.abs(blueRing_1[0] - redS) <= (int) (2 * blueRing_1[3]) &&
							Math.abs(blueRing_1[1] - greenS) <= (int) (2 * blueRing_1[4]) &&
							Math.abs(blueRing_1[2] - blueS) <= (int) (2 * blueRing_1[5]))
					{
						lcd.drawString("Blue Ring Detected", 0, 3);
					}
					
					else if(Math.abs(yellowRing_1[0] - redS) <= (int) (2 * yellowRing_1[3]) &&
							Math.abs(yellowRing_1[1] - greenS) <= (int) (2 * yellowRing_1[4]) &&
							Math.abs(yellowRing_1[2] - blueS) <= (int) (2 * yellowRing_1[5]))
					{
						lcd.drawString("Yellow Ring Detected", 0, 4);
					}
						
						
					updateEnd = System.currentTimeMillis();
					if(updateEnd-updateStart < displayPeriod) {
						try {
							Thread.sleep(displayPeriod-(updateEnd-updateStart));
						}
						catch(InterruptedException e) {
						}
					}
				}while((updateEnd-tStart)<= timeout);
				
			}
			
			else {
				lcd.clear();
			}
		}
		
		
		
		
	}
	public void fetchLightData() {
		ring_color_sample_provider.fetchSample(color_samples, 0);
		this.redS = (int) (color_samples[0] * 1000000);
		this.greenS = (int) (color_samples[1] * 1000000);
		this.blueS = (int) (color_samples[2] * 1000000);
	}
	
	
	
	
	
	
}

