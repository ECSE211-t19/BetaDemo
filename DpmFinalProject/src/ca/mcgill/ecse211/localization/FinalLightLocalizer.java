package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import sun.management.Sensor;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.wifi.Wifi;
import lejos.hardware.ev3.LocalEV3;

/***
 * This class implements the light localization in Lab4 on the EV3 platform.
 * 
 * @authorAbedAtassi
 * @authorHyunSuAn
 */
public class FinalLightLocalizer implements Runnable {

	private SampleProvider color_sample_provider;
	private float[] color_samples;
	private float light_value;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final double d = 13.3; // distance between the center of the robot and the light sensor
	private static final double TILE_WIDTH = 30.48;
	private static final int ROTATE_SPEED = 110;
	private double TRACK;
	private double WHEEL_RAD;
	private Odometer odoData;
	private float prev_red;
	private float curr_red;
	private int startCorner;
	private double dX, dY, deltaX, deltaY, xZero, yZero;
	private float first, second, third, fourth, angleCorrection;
	private EV3GyroSensor gyroSensor;
	private Navigation navigation;
	private Wifi wifi;
	private float errorMargin = 100; //150
	private float BLACK = 220;

	/***
	 * Constructor
	 * 
	 * 
	 * @param leftMotor, rightMotor, TRACK, WHEEL_RAD
	 * @throws OdometerExceptions 
	 */
	public FinalLightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double TRACK,
			double WHEEL_RAD, EV3GyroSensor gyroSensor, Navigation navigation, Wifi wifi) throws OdometerExceptions {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odoData = Odometer.getOdometer();
		EV3ColorSensor colour_sensor = MainClass.lineSensor;
		this.color_sample_provider = colour_sensor.getMode("Red");
		this.color_samples = new float[colour_sensor.sampleSize()];
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		this.startCorner = startCorner;
		this.gyroSensor = gyroSensor;
		this.navigation = navigation;
		this.wifi = wifi;
	}

	public void run() {
		
		do_localization();
	}

	/***
	 * This method starts the light localization on the robot
	 *
	 * 
	 * 
	 */
	public void do_localization() {
		odoData.setX(0);
		odoData.setY(0);
		odoData.setTheta(0);
		int numberLines = 0;
		double[] angles = new double[4];
		boolean line = false;

		
		Sound.beep();
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
		rightMotor.rotate(-convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
		
//		
//		prev_red = fetchUSData();
//		
//		while (numberLines < 4) {
//			curr_red = fetchUSData();
//			
//			if ( prev_red - curr_red > 3.5) { //3.5
//				
//				angles[numberLines] = odoData.getXYT()[2];
//				
//				Sound.beep();
//				numberLines++;
//			}
//			
//			prev_red = curr_red;
//		}
		
		
		
		
		prev_red = fetchUSData();
		
        while (numberLines < 4)
        {
        	curr_red = fetchUSData();
        	
        	if (prev_red - curr_red > 19 && legitDetection(10))
        	{
        		angles[numberLines] = odoData.getXYT()[2];
        		Sound.beep();
        		numberLines++;
        	}
        	
        	prev_red = curr_red;
        }
        
        leftMotor.stop(true);
        rightMotor.stop(false);
		
		
		// do calculations
		double deltaX = angles[2] - angles[0];
		double deltaY = angles[3] - angles[1];

		double xZero = d * Math.cos(Math.toRadians(deltaY / 2));
		double yZero = d * Math.cos(Math.toRadians(deltaX / 2));

		
		navigation.travelTo(xZero, yZero, ROTATE_SPEED, ROTATE_SPEED);
		leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(xZero / yZero))), true);
        rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(xZero / yZero))), false);
		
	/*	leftMotor.setSpeed(90);
		rightMotor.setSpeed(90);
		leftMotor.backward();
		rightMotor.forward();

		prev_red = fetchUSData();

		while (numberLines < 5) {
			curr_red = fetchUSData();

			if (prev_red - curr_red > 3) { // 3.5

				Sound.beep();
				break;
			}

			prev_red = curr_red;
		}

		leftMotor.stop(true);
		rightMotor.stop(false);
		
		*/
        
        
        //---------------------------------------------------------------------------------
      
   
        resetGyro(gyroSensor);
        
        
        //----------------------------------------------------------------------------------
        
	}

public void doNavLocalization(double toX, double toY) {
        
        int case_no = 0;
        
        double[] currXYT = odoData.getXYT();
        
        // straighten the robot out
        if (currXYT[2] > 315.0 || currXYT[2] < 45.0)
        {
            if (currXYT[2] >= 0)
            {
                navigation.turnTo(-currXYT[2], ROTATE_SPEED);
            }
            else
            {
                navigation.turnTo(360.0 - currXYT[2], ROTATE_SPEED);
            }
        }
        else if (currXYT[2] > 45.0 && currXYT[2] < 135.0)
        {
            navigation.turnTo( 90.0 - currXYT[2], ROTATE_SPEED);
        }
        else if (currXYT[2] > 135.0 && currXYT[2] < 225.0)
        {
            navigation.turnTo( 180.0 - currXYT[2], ROTATE_SPEED);
        }
        else if (currXYT[2] > 225.0 && currXYT[2] < 315.0)
        {
            navigation.turnTo( 270.0 - currXYT[2], ROTATE_SPEED);
        }
        
        
        odoData.setTheta(0);
        
        // Determine four cases of localization, four different quadrants
        if (currXYT[0] > toX && currXYT[1] > toY)
        {
            case_no = 1;
        }
        else if (currXYT[0] <= toX && currXYT[1] > toY)
        {
            case_no = 2;
        }
        else if (currXYT[0] <= toX && currXYT[1] <= toY)
        {
            case_no = 3;
        }
        else if (currXYT[0] > toX && currXYT[1] <= toY)
        {
            case_no = 4;
        }
        
//      // hardcode turn
//
//      leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 25), true);
//        rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 25), false);
        
        
        int numberLines = 0;
        double[] angles = new double[4];
        
        leftMotor.setSpeed(ROTATE_SPEED);
        rightMotor.setSpeed(ROTATE_SPEED);
        
        //different turning orientation per case
        switch (case_no)
        {
            case 1:
            case 4:
                leftMotor.rotate(-convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
                rightMotor.rotate(convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
                break;
            case 2:
            case 3:
                leftMotor.rotate(convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
                rightMotor.rotate(-convertDistance(WHEEL_RAD, Math.PI * TRACK), true);
                break;
                
            default:
            	Sound.beepSequenceUp();
            	break;
        }
        
        
        // detect lines
//        prev_red = fetchUSData();
//        
//        while (numberLines < 4) {
//            curr_red = fetchUSData();
//            
//            if ( prev_red - curr_red > 3.5) { //3.5
//                
//                angles[numberLines] = odoData.getXYT()[2];
//                Sound.beep();
//                numberLines++;
//            }
//            
//            prev_red = curr_red;
//        }
        
        
        prev_red = fetchUSData();
        while (numberLines < 4)
        {
        	curr_red = fetchUSData();
        	
        	if (prev_red - curr_red > 19 && legitDetection(10))
        	{
        		angles[numberLines] = odoData.getXYT()[2];
        		Sound.beep();
        		numberLines++;
        	}
        	
        	prev_red = curr_red;
        }
        leftMotor.stop(true);
        rightMotor.stop(false);
        
        
        switch (case_no) {
            case 1:
                // do calculations
                deltaX = 360 - (angles[0] - angles[2]);
                deltaY = angles[1] - angles[3];
                xZero = d * Math.cos(Math.toRadians(deltaY / 2));
                yZero = d * Math.cos(Math.toRadians(deltaX / 2));
                
                navigation.travelTo(currXYT[0] - xZero, currXYT[1] - yZero, ROTATE_SPEED, ROTATE_SPEED);
                
                
                
              leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(yZero / xZero)) + 90), true);
              rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(yZero / xZero)) + 90), false);
                
//                leftMotor.setSpeed(50);
//                rightMotor.setSpeed(50);
//                leftMotor.forward();
//                rightMotor.backward();
//                
//                prev_red = fetchUSData();
//                
//                while (numberLines < 6) {
//                    curr_red = fetchUSData();
//                    
//                    if ( prev_red - curr_red > 3.5) { //3.5
//                        
//                        Sound.beep();
//                        break;
//                    }
//                    
//                    prev_red = curr_red;
//                }
//                
//                leftMotor.stop(true);
//                rightMotor.stop(false);
                
                break;
                
            case 2:
                // do calculations
                deltaX = 360 - (angles[2] - angles[0]);
                deltaY = angles[3] - angles[1];
                xZero = d * Math.cos(Math.toRadians(deltaY / 2));
                yZero = d * Math.cos(Math.toRadians(deltaX / 2));
                
                navigation.travelTo(currXYT[0] + xZero, currXYT[1] - yZero, ROTATE_SPEED, ROTATE_SPEED);
                
              leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(yZero / xZero)) + 90), true);
              rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(yZero / xZero)) + 90), false);
                
//                leftMotor.setSpeed(50);
//                rightMotor.setSpeed(50);
//                leftMotor.backward();
//                rightMotor.forward();
//                
//                prev_red = fetchUSData();
//                
//                while (numberLines < 6) {
//                    curr_red = fetchUSData();
//                    
//                    if ( prev_red - curr_red > 3.5) { //3.5
//                        
//                        Sound.beep();
//                        break;
//                    }
//                    
//                    prev_red = curr_red;
//                }
//                
//                leftMotor.stop(true);
//                rightMotor.stop(false);
                
                break;
                
            case 3:
                // do calculations
                deltaX = angles[2] - angles[0];
                deltaY = angles[3] - angles[1];
                xZero = d * Math.cos(Math.toRadians(deltaY / 2));
                yZero = d * Math.cos(Math.toRadians(deltaX / 2));
                
                navigation.travelTo(currXYT[0] + xZero, currXYT[1] + yZero, ROTATE_SPEED, ROTATE_SPEED);
                
                
                
              leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(xZero / yZero))), true);
              rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(xZero / yZero))), false);
                
//                leftMotor.setSpeed(50);
//                rightMotor.setSpeed(50);
//                leftMotor.backward();
//                rightMotor.forward();
//                
//                prev_red = fetchUSData();
//                
//                while (numberLines < 5) {
//                    curr_red = fetchUSData();
//                    
//                    if ( prev_red - curr_red > 3.5) { //3.5
//                        
//                        Sound.beep();
//                        break;
//                    }
//                    
//                    prev_red = curr_red;
//                }
//                
//                leftMotor.stop(true);
//                rightMotor.stop(false);
                
                break;
                
            case 4:
                // do calculations
                deltaX = angles[0] - angles[2];
                deltaY = angles[1] - angles[3];
                xZero = d * Math.cos(Math.toRadians(deltaY / 2));
                yZero = d * Math.cos(Math.toRadians(deltaX / 2));
                
                navigation.travelTo(currXYT[0] - xZero, currXYT[1] + yZero, ROTATE_SPEED, ROTATE_SPEED);
                
                
                
              leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(xZero / yZero))), true);
              rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, Math.toDegrees(Math.atan(xZero / yZero))), false);
                
//                leftMotor.setSpeed(50);
//                rightMotor.setSpeed(50);
//                leftMotor.forward();
//                rightMotor.backward();
//                
//                prev_red = fetchUSData();
//                
//                while (numberLines < 5) {
//                    curr_red = fetchUSData();
//                    
//                    if ( prev_red - curr_red > 3.5) { //3.5
//                        
//                        Sound.beep();
//                        break;
//                    }
//                    
//                    prev_red = curr_red;
//                }
//                
//                leftMotor.stop(true);
//                rightMotor.stop(false);
                
                break;
                
            default:
            	Sound.beepSequenceUp();
            	break;
        
        }
        
        
        odoData.setX(toX);
        odoData.setY(toY);
        
        // set angle to closest angle in multiple of 90 degrees
        if (currXYT[2] > 315.0 || currXYT[2] < 45.0)
        {
            odoData.setTheta(0);
        }
        else if (currXYT[2] > 45.0 && currXYT[2] < 135.0)
        {
            odoData.setTheta(90);
        }
        else if (currXYT[2] > 135.0 && currXYT[2] < 225.0)
        {
            odoData.setTheta(180);
        }
        else if (currXYT[2] > 225.0 && currXYT[2] < 315.0)
        {
            odoData.setTheta(270);
        }
    }
	
	
	
	
	
	/***
	 * This method fetches the US data
	 * 
	 */
	public float fetchUSData() {
		color_sample_provider.fetchSample(color_samples, 0);

		return (color_samples[0] * 1000);

	}
	
	public boolean legitDetection(int sampleCount) {
		float sum = 0;
		
		for (int i = 0; i < sampleCount; i++)
		{
			sum += fetchUSData();
		}
		
		float avg = sum / sampleCount;
		
		if (avg > BLACK - errorMargin  && avg < BLACK + errorMargin)
		{
			return true;
		}
		else 
		{
			return false;
		}
	}

	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	public static void resetGyro(EV3GyroSensor G) {
	G.reset();
	
	}
}