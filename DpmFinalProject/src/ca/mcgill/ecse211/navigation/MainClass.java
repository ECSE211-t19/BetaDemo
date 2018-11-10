// Lab2.java
package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class MainClass {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
	public static final EV3ColorSensor lineSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
	public static final EV3ColorSensor ringSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	public static final EV3GyroSensor gyroSensor = new EV3GyroSensor(LocalEV3.get().getPort("S4"));

	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.13;
	public static final double TRACK = 9.66;
	public Odometer odometer;

	public static void main(String[] args) throws OdometerExceptions {
		int buttonChoice;
		// Sets up colour sensor and array holding data


		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, gyroSensor, TRACK, WHEEL_RAD);
		// ObstacleAvoidance obstacleAvoidance = new ObstacleAvoidance();
		Display odometryDisplay = new Display(lcd);
		ColorDisplay colorDisplay = new ColorDisplay(lcd);
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

		LightLocalizer lightLocalizer = new LightLocalizer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		
		ObstacleAvoidance obstacleavoidance = new ObstacleAvoidance(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		
		do {
			// clear the display
			lcd.clear();
			
			// ask the user whether the motors should drive in a square or float
            lcd.drawString("< Left    |  Right >", 0, 0);
            lcd.drawString("          |         ", 0, 1);
            lcd.drawString(" Color    | Start   ", 0, 2);
            lcd.drawString("Detection | Search   ", 0, 3);
            lcd.drawString("          |          ", 0, 4);
			
			
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		
		if (buttonChoice == Button.ID_LEFT)
		{
			Thread colorThread = new Thread(colorDisplay);
			colorThread.start();
		}
		else
		{

			// Start odometer and display threads
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			
				
			usLocalizer.run();
			lightLocalizer.run();
			Sound.beep();
			//go to start point
			
			obstacleavoidance.run(); // run the obstacleAvoidance
			
		}
		

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}