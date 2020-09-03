package org.usfirst.frc.team1671.auto.modes;

import java.lang.reflect.Array;
import java.util.Arrays;

import org.usfirst.frc.team1671.auto.AutoModeBase;
import org.usfirst.frc.team1671.auto.AutoModeEndedException;
import org.usfirst.frc.team1671.auto.actions.CharacterizeAcceleration;
import org.usfirst.frc.team1671.auto.actions.CharacterizeVoltage;
import org.usfirst.frc.team1671.auto.actions.FollowPath;
import org.usfirst.frc.team1671.auto.actions.IntakeCube;
import org.usfirst.frc.team1671.auto.actions.OpenCloseJaw;
import org.usfirst.frc.team1671.auto.actions.OpenLoopDrive;
import org.usfirst.frc.team1671.auto.actions.ParallelAction;
import org.usfirst.frc.team1671.auto.actions.SeriesAction;
import org.usfirst.frc.team1671.auto.actions.Shoot;
import org.usfirst.frc.team1671.auto.actions.SimpleDrive;
import org.usfirst.frc.team1671.auto.actions.SuperstructurePosition;
import org.usfirst.frc.team1671.auto.actions.TurnInPlace;
import org.usfirst.frc.team1671.auto.actions.Wait;
import org.usfirst.frc.team1671.robot.PathElements;
import org.usfirst.frc.team1671.robot.RobotMap;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class TestMode extends AutoModeBase {
	
	@Override
	protected void routine() throws AutoModeEndedException {
		
		//runAction(new FollowPath(PathElements.Test, false));
		//runAction(new FollowPath(PathElements.Test2, true));
		runAction(new ParallelAction(Arrays.asList(new TurnInPlace(180.0, true), 
				new SeriesAction(Arrays.asList(new Wait(2.0), 
						new FollowPath(PathElements.Test3, true))
					)
				)
			)
		);
	}

}
