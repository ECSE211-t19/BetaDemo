package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.wifi.Wifi;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class Pilot implements Runnable{

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private double TRACK;
	private double WHEEL_RAD;
	public static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 150;
	private static final double TILE_WIDTH = 30.48;
	private Odometer odometer;
	private OdometerData odoData;
	Wifi wifi;
	int StartingCorner;
	int[] StartXY;
	int tunnel[][];
	int SearchZone[][];
	int tree[];
	int team;
	Navigation navigation;
	
	public Pilot(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
			final double WHEEL_RAD, Wifi wifi, Navigation navigation) throws OdometerExceptions {
		
		this.odometer = Odometer.getOdometer();
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;
		this.wifi = wifi;
		this.team = wifi.getTeam();
		this.StartingCorner = wifi.getStartingCorner(team);
		this.StartXY = wifi.getStartingCornerCoords();
		this.tunnel = wifi.getTunnel();
		this.tree = wifi.getRingSet();
		odoData = OdometerData.getOdometerData();
		odoData.setXYT(0,0,0);

		this.navigation = navigation;
		
		}
	
	public void run() {
		try {
			this.odoData = Odometer.getOdometer();
		} catch (OdometerExceptions e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(250); // reduced the acceleration to make it smooth
			
		}
		
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}
		odoData.setXYT(7 * TILE_WIDTH, TILE_WIDTH, 270);
		travelToTunnel();
		//travelThroughTunnel();
		//travelToRingSet();
	}
	
	
	/**
	 * Method to drive to the tunnel
	 */
	public void travelToTunnel() {
		
		if(wifi.isTunnelVertical()) {
			LCD.drawString("tunnel: " +tunnel[1][0] + "" + tunnel[1][1]+ " start: " + StartXY[1]  , 0, 3);
			//travel to the lower-right corner of the tunnel
			navigation.travelTo((tunnel[1][0] + 0.5) * TILE_WIDTH,StartXY[1] * TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
			navigation.travelTo((tunnel[1][0] + 0.5) * TILE_WIDTH, (tunnel[1][1]-1) * TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
			

		}
		else {
			LCD.drawString("lol"  , 0, 4);

			//travel to the point next to the upper-left corner
			navigation.travelTo((tunnel[3][0] - 1) * TILE_WIDTH, (StartXY[1]) * TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
			navigation.travelTo((tunnel[3][0] - 1) * TILE_WIDTH, (tunnel[3][1] + 0.5)* TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
			
		}
	}

	/**
	 * Method to drive through the tunnel
	 */
	public void travelThroughTunnel() {
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		
		if(wifi.isTunnelVertical()) {
			
			leftMotor.rotate(navigation.convertAngle(WHEEL_RAD, TRACK, 90), true);
			rightMotor.rotate(- navigation.convertAngle(WHEEL_RAD, TRACK, 90), false);
			navigation.travelTo((tunnel[1][0] + 0.5) * TILE_WIDTH, (tunnel[3][1] + 1)* TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
			navigation.travelTo((tunnel[1][0] + 1) * TILE_WIDTH, (tunnel[1][1] + 1) * TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
			
		}
		//assure that the robot is pointing 270
		else {
			leftMotor.rotate(- navigation.convertAngle(WHEEL_RAD, TRACK, 90), true);
			rightMotor.rotate(navigation.convertAngle(WHEEL_RAD, TRACK, 90), false);
			navigation.travelTo((tunnel[3][0] + 1) * TILE_WIDTH,  (tunnel[3][1] + 0.5) * TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
			navigation.travelTo((tunnel[3][0] + 1) * TILE_WIDTH, (tunnel[3][1] + 1)* TILE_WIDTH, FORWARD_SPEED, ROTATE_SPEED);
		}
		
	}
	/**
	 * Method to travel to the ringSet
	 * 
	 */
	public void travelToRingSet() {
		int uLX = tunnel[2][0];
		int uRX = tunnel[3][0];
		
		if((uLX + uRX) / 2 > tree[0]) {
			
			navigation.travelTo(odometer.getXYT()[0] / TILE_WIDTH, tree[1], FORWARD_SPEED, ROTATE_SPEED);
			navigation.travelTo(tree[0]+1,tree[1], FORWARD_SPEED, ROTATE_SPEED);
			
		}
		
		else {
			navigation.travelTo(odometer.getXYT()[0] / TILE_WIDTH, tree[1], FORWARD_SPEED, ROTATE_SPEED);
			navigation.travelTo(tree[0] - 1,tree[1], FORWARD_SPEED, ROTATE_SPEED);
		}
		
	}
	
	
	
	
	
}
