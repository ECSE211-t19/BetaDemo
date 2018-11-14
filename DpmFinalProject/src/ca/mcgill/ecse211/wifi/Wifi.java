package ca.mcgill.ecse211.wifi;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;


public class Wifi {

  // ** Set these as appropriate for your team and current situation **
  private static final String SERVER_IP = "192.168.2.2";
  private static final int TEAM_NUMBER = 19;
  private Map data; 
  // Enable/disable printing of debug info from the WiFi class
  private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;

  public Wifi () {
	  this.getMapData();
  }
  
  @SuppressWarnings("rawtypes")
  public void getMapData() {

    // Initialize WifiConnection class
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

    // Connect to server and get the data, catching any errors that might occur
    try {
     
      data = conn.getData();
      

    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }

    // Wait until user decides to end program
    //Button.waitForAnyPress();
  }
  /**
	 * Gets the color of the team you are in.
	 * 
	 * @return Team color for the team you are in
	 */
	public int getTeam() {
		return ((Long) data.get("GreenTeam")).intValue();
	}

	/**
	 * Gets the starting corner of the given team.
	 * 
	 * @return an int representing the starting corner of the team.
	 */
	public int getStartingCorner() {
		return ((Long) data.get("GreenCorner")).intValue();
	}

	/**
	 * Gets the starting corner coordinates 
	 *
	 * @return start
	 */
	public int[] getStartingCornerCoords() {
		int[] start = { 7, 1 };
		return start;
	}
	/**
	 * Gets the coordinates of the green zone.
	 * 
	 * @return greenZone
	 */
	public int[][] getGreenZone() {
	
		int llx = ((Long) data.get("Green_LL_x")).intValue();
		int lly = ((Long) data.get("Green_LL_y")).intValue();
		int urx = ((Long) data.get("Green_UR_x")).intValue();
		int ury = ((Long) data.get("Green_UR_y")).intValue();

	
		int[][] greenZone = { { llx, lly }, { urx, lly }, { urx, ury },{ llx, ury } };

		return greenZone;
	}

	/**
	 * Gets the coordinates of the tunnel
	 * 
	 * @return greenTunnelZone
	 */
	public int[][] getTunnel() {
		int llx, lly, urx, ury;

		
		llx = ((Long) data.get("TNG_LL_x")).intValue();
		lly = ((Long) data.get("TNG_LL_y")).intValue();
		urx = ((Long) data.get("TNG_UR_x")).intValue();
		ury = ((Long) data.get("TNG_UR_y")).intValue();
		LCD.drawString("URX T"+ urx, 0, 5);

		int[][] greenTunnelZone = { { llx, lly }, { urx, lly },{ urx, ury }, { llx, ury } };

		return greenTunnelZone;
	}

	/**
	 * Gets the search zone of the specified team
	 *  
	 * @return greenSearchZone
	 */
	public int[][] getSearchZone() {
		int llx, lly, urx, ury;
		
		llx = ((Long) data.get("Island_LL_x")).intValue();
		lly = ((Long) data.get("Island_LL_y")).intValue();
		urx = ((Long) data.get("Island_UR_x")).intValue();
		ury = ((Long) data.get("Island_UR_y")).intValue();
		
		int[][] greenSearchZone = { { llx, lly }, { urx, lly },{ urx, ury }, { llx, ury } };

		return greenSearchZone;
	}
	
	/**
	 * Gets the ring set coordinates
	 * 
	 * @return 1-D ringSet
	 */
	public int[] getRingSet() {
		int x,y;
		
		x = ((Long) data.get("TG_x")).intValue();
		y = ((Long) data.get("TG_y")).intValue();

		int[] ringSet = {x,y};

		return ringSet;

	}
	
	/**
	 * Determines the orientation of the tunnel
	 *
	 * @return boolean 
	 */
	public boolean isTunnelVertical() {
		
		int llx = ((Long) data.get("TNG_LL_x")).intValue();
		int urx = ((Long) data.get("TNG_UR_x")).intValue();
		
		if(urx-llx == 1) {
			return true;
		}
		else {
			return false;
		}	
	}
  
  
}
