package org.usfirst.frc.team1671.robot;

/**
 * 
 * @author Maverick Zhang
 * All field measurements are found in here.
 *
 */
public class FieldElements {
	//All units are in FEET. Decimals that are rounded have the exact value given in comments.
	//Left measurements are given as the right side can be found by taking the negative of the Y.
	
	/*
	 * Field is oriented like this:
	 * --------------
	 *|				 |		^ North
	 *|	<-(0,0)		 |
	 *|				 |
	 * --------------
	 * (0,0) is the east side center (basically where the robot is when it is going for switch)
	 * +x is towards east
	 * +y is towards north
	 */
	
	//Overall Field
	public static final double fieldWidth = 27.0;
	public static final double fieldLength = 54.0;
	public static final double halfFieldLength = 27.0;
	
	//Reference distance to alliance wall
	public static final double allianceWall = 22.0;
	public static final double halfAllianceWall = 11.0;
	public static final double wallToAutoLine = 10.0;
	public static final double wallToSwitchCube1 = 9.0;
	public static final double wallToSwitchCube2 = 10.17; //10 ft 2 in.
	public static final double wallToSwitch = 12.5;
	public static final double wallToFence = 15.5;
	public static final double wallToScale = 25.0;
	
	//Power cube area in front of switch
	public static final double powerCubeAreaSWCornerX = 9.0;
	public static final double powerCubeAreaSWCornerY = -1.875;
	public static final double powerCubeAreaNECornerX = 12.5;
	public static final double powerCubeAreaNECornerY = 1.875;
	
	//Switch measurements
	public static final double switchWidth = 12.0;
	public static final double switchLength = 3.0;
	public static final double northSwitchCenterX = 14.0;
	public static final double northSwitchCenterY = 4.0;
	
	//Scale measurements
	public static final double scaleWidth = 15.0;
	public static final double scaleLength = 4.0;
	public static final double scaleMaxHeight = 6.0;
	public static final double scaleMinHeight = 4.0;
	public static final double scaleStartHeight = 5.0;
	public static final double northScaleCenterX = 27.0;
	public static final double northScaleCenterY = 6.0;
	
	//Platform and ramp measurements
	public static final double innerPlatformSWCornerX = 23.5625;
	public static final double innerPlatformSWCornerY = -4.33; //-4ft 4 in.
	public static final double innerPlatformNECornerX = 27.0;
	public static final double innerPlatformNECornerY = 4.33; //4ft 4 in.
	public static final double innerToOuterPlatform = 1.0625;
	public static final double rampHeight = 0.292; //3.5 in.
	
	//Cube and Fence Cube measurements
	public static final double cube = 1.17; //1ft 1-2 in.
	public static final double fenceToScale = 7.979; //7ft 11-3/4 in.
	public static final double fenceCube1X = wallToFence + cube / 2.0;
	public static final double fenceCube1Y = 6.0 - cube / 2.0;
	public static final double fenceCube2X = fenceCube1X;
	public static final double fenceCube2Y = fenceCube1Y - 1.0; //The gap between cubes is 1-2 in.
	public static final double fenceCube3X = fenceCube1X;
	public static final double fenceCube3Y = fenceCube2Y - 1.0;

}
