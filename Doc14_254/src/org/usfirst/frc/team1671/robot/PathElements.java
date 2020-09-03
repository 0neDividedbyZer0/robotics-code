package org.usfirst.frc.team1671.robot;

import java.util.Arrays;
import java.util.List;

import jaci.pathfinder.Pathfinder;
import trajectory_lib.Waypoint;
import util.Util;

public class PathElements {
	public static final double width = RobotMap.ROBOT_WIDTH;
	public static final double length = RobotMap.ROBOT_LENGTH;
	public static final double halfWidth = width / 2.0;
	//(x,y,theta), angle is in degrees, the comments have not accounted for robot width yet
	
	public static final List<Waypoint> Test2 = Arrays.asList(new Waypoint(FieldElements.wallToSwitch-  6.5 ,0.0, Pathfinder.d2r(180.0))
			,new Waypoint(0.0,6.0,Pathfinder.d2r(90.0)));
	public static final List<Waypoint> Test = Arrays.asList(new Waypoint(0.0,0.0,Pathfinder.d2r(0.0)),
			new Waypoint(FieldElements.wallToSwitch-  6.5 ,0.0, Pathfinder.d2r(0.0)));
	public static final List<Waypoint> Test3 = Arrays.asList(new Waypoint(0.0,0.0,Pathfinder.d2r(180.0)),
			new Waypoint(FieldElements.wallToSwitch-6.5 ,6.0, Pathfinder.d2r(270.0)));
	
	//Switch
	//1.(0,0,0)->(12.0,4.0,0)
	//2.(12.0,4.0,180.0)->(1.0, 0.0, 180.0)
	//3.(1.0, 0.0, 0.0)-> (9.0,0.0,0.0)
	//4.(9.0,0.0, 180.0)->(1.0, 0.0, 180.0)
	//5.(1.0,0.0,0.0)->(12.0 ,6.0,0.0)
	//6.(12.0 ,6.0,180.0)->(1.0 ,0.0,180.0)
	//7.(1.0, 0.0, 0.0)-> (10.17,0.0,0.0)
	//8.(10.17.0,0.0, 180.0)->(1.0, 0.0, 180.0)
	//9.5.(1.0,0.0,0.0)->(12.0 ,6.0,0.0)
	//est. time from pathfinder in total is 14.63s
	public static final List<Waypoint> startToSwitchLeft = Arrays.asList(new Waypoint(length,0.0,Pathfinder.d2r(0.0)),
			new Waypoint(FieldElements.wallToSwitch + 0.8, FieldElements.northSwitchCenterY + 1.0, Pathfinder.d2r(0.0)));
	public static final List<Waypoint> switchLeftToShiftedCenter = Arrays.asList(new Waypoint(FieldElements.wallToSwitch + 0.8, FieldElements.northSwitchCenterY + 1.0, Pathfinder.d2r(180.0)),
			new Waypoint(length + 3.0,1.9,Pathfinder.d2r(180.0)));
	public static final List<Waypoint> shiftedCenterToSwitchCube1 = Arrays.asList(new Waypoint(length+ 3.0,0.0,Pathfinder.d2r(0.0)),
			new Waypoint(FieldElements.wallToSwitchCube1 + 2.6, 0.0, Pathfinder.d2r(0.0)));
	public static final List<Waypoint> switchCube1ToShiftedCenter = Arrays.asList(new Waypoint(FieldElements.wallToSwitchCube1 + 2.6, 0.0, Pathfinder.d2r(180.0)),
			new Waypoint(length + 3.0,0.0,Pathfinder.d2r(180.0)));
	public static final List<Waypoint> shiftedCenterToSwitchLeft = Arrays.asList(new Waypoint(length+ 3.0,2.0,Pathfinder.d2r(0.0)),
			new Waypoint(FieldElements.wallToSwitch, FieldElements.northSwitchCenterY + 2.0, Pathfinder.d2r(0.0)));
	public static final List<Waypoint> switchLeftToShiftedCenter2 = Arrays.asList(new Waypoint(FieldElements.wallToSwitch, FieldElements.northSwitchCenterY + 2.0, Pathfinder.d2r(180.0)),
			new Waypoint(length+ 3.0,2.0,Pathfinder.d2r(180.0)));
	public static final List<Waypoint> shiftedCenterToSwitchCube2 = Arrays.asList(new Waypoint(length+ 3.0,0.0,Pathfinder.d2r(0.0)),
			new Waypoint(FieldElements.wallToSwitchCube2 + 2.6, 0.0, Pathfinder.d2r(0.0)));
	public static final List<Waypoint> switchCube2ToShiftedCenter = Arrays.asList(new Waypoint(FieldElements.wallToSwitchCube2 + 2.6, 0.0, Pathfinder.d2r(180.0)),
			new Waypoint(length + 3.0,0.0,Pathfinder.d2r(180.0)));
	public static final List<Waypoint> shiftedCenterToSwitchLeft2 = Arrays.asList(new Waypoint(length+ 3.0,0.0,Pathfinder.d2r(0.0)),
			new Waypoint(FieldElements.wallToSwitch + 2.6, FieldElements.northSwitchCenterY + 2.0, Pathfinder.d2r(0.0)));
	
	public static final List<Waypoint> startToSwitchRight = Arrays.asList(new Waypoint(length,0.0,Pathfinder.d2r(0.0)),
			new Waypoint(FieldElements.wallToSwitch + 0.8, -FieldElements.northSwitchCenterY - 1.0, Pathfinder.d2r(0.0)));
	public static final List<Waypoint> switchRightToShiftedCenter = Arrays.asList(new Waypoint(FieldElements.wallToSwitch + 0.8, -FieldElements.northSwitchCenterY - 1.0, Pathfinder.d2r(180.0)),
			new Waypoint(length + 3.3,-1.4,Pathfinder.d2r(180.0)));
	public static final List<Waypoint> shiftedCenterToSwitchRight = Arrays.asList(new Waypoint(length + 3.0,-2.0, Pathfinder.d2r(0.0)),
			new Waypoint(FieldElements.wallToSwitch, -FieldElements.northSwitchCenterY-2.0,Pathfinder.d2r(0.0)));
	public static final List<Waypoint> switchRightToShiftedCenter2 = Arrays.asList(new Waypoint(FieldElements.wallToSwitch, -FieldElements.northSwitchCenterY - 2.0, Pathfinder.d2r(180.0)),
			new Waypoint(length+ 3.0,-2.0,Pathfinder.d2r(180.0)));
	public static final List<Waypoint> shiftedCenterToSwitchRight2 = Arrays.asList(new Waypoint(length+ 3.0,0.0,Pathfinder.d2r(0.0)),
			new Waypoint(FieldElements.wallToSwitch + 2.6, -FieldElements.northSwitchCenterY - 2.0, Pathfinder.d2r(0.0)));
	
	
	//Scale needs angle conversions after spin1 :/
	public static final List<Waypoint> startToScaleRightSame = Arrays.asList(new Waypoint(length,-FieldElements.halfAllianceWall + halfWidth,Pathfinder.d2r(0.0))
			, new Waypoint(FieldElements.wallToFence, -FieldElements.halfAllianceWall + halfWidth, Pathfinder.d2r(0.0)) 
			, new Waypoint(FieldElements.wallToScale - 3.5, -FieldElements.northScaleCenterY + 0.5, Pathfinder.d2r(25.0)));
	//spin1 
	public static final double finalAngleOfSpin1AbsRight = 160.0;
	public static final List<Waypoint> scaleRightToFenceCubeRight1 = Arrays.asList(new Waypoint(FieldElements.wallToScale, -FieldElements.northScaleCenterY - 1.5, finalAngleOfSpin1AbsRight)
			, new Waypoint(FieldElements.fenceCube1X, -FieldElements.fenceCube1Y, finalAngleOfSpin1AbsRight));
	//public static final List<Waypoint> fenceCubeRight1Reverse = Arrays.asList(new Waypoint(FieldElements.fenceCube1X, -FieldElements.fenceCube1Y, finalAngleOfSpin1AbsRight + Math.PI)
		//	, new Waypoint(FieldElements.fenceCube1X + length, -FieldElements.fenceCube1Y - length, finalAngleOfSpin1AbsRight / 2.0 + Math.PI));
	//spin2
	public static final List<Waypoint> fenceCubeRight1Reverse = Arrays.asList(new Waypoint(FieldElements.fenceCube1X, -FieldElements.fenceCube1Y, 0.0)
			, new Waypoint(FieldElements.fenceCube1X + 3.0, -FieldElements.fenceCube1Y - 2.0, - Math.PI / 2.0));
	
	public static final List<Waypoint> startToScaleLeftSame = Arrays.asList(new Waypoint(length,FieldElements.halfAllianceWall - halfWidth,Pathfinder.d2r(0.0))
			, new Waypoint(FieldElements.wallToFence, FieldElements.halfAllianceWall - halfWidth, Pathfinder.d2r(0.0)) 
			, new Waypoint(FieldElements.wallToScale - 3.5, FieldElements.northScaleCenterY - 0.5, Pathfinder.d2r(-25.0)));
	//spin1 
	public static final double finalAngleOfSpin1AbsLeft = -160.0;
	//170.0 degrees
	//8 ft, 1.5 ft
	public static final List<Waypoint> scaleLeftToFenceCubeLeft1 = Arrays.asList(new Waypoint(FieldElements.wallToScale, FieldElements.northScaleCenterY + 1.5, Pathfinder.d2r(170.0))
			, new Waypoint(FieldElements.fenceCube1X, FieldElements.fenceCube1Y, Pathfinder.d2r(170.0)));
	
}
