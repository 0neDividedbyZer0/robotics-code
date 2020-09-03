package org.usfirst.frc.team1671.auto.modes;

import java.util.Arrays;

import org.usfirst.frc.team1671.auto.AutoModeBase;
import org.usfirst.frc.team1671.auto.AutoModeEndedException;
import org.usfirst.frc.team1671.auto.actions.FollowPath;
import org.usfirst.frc.team1671.auto.actions.IntakeCube;
import org.usfirst.frc.team1671.auto.actions.OpenCloseJaw;
import org.usfirst.frc.team1671.auto.actions.ParallelAction;
import org.usfirst.frc.team1671.auto.actions.SeriesAction;
import org.usfirst.frc.team1671.auto.actions.Shoot;
import org.usfirst.frc.team1671.auto.actions.SuperstructurePosition;
import org.usfirst.frc.team1671.auto.actions.Wait;
import org.usfirst.frc.team1671.robot.PathElements;

public class PathFollowingSwitchMode extends AutoModeBase {
	private boolean right;
	//do you want to run the right side?
	public PathFollowingSwitchMode(boolean right) {
		this.right = right;
	}
	
	public PathFollowingSwitchMode(String gameData) {
		if(gameData.charAt(0) == 'R') {
			right = true;
		} else {
			right = false;
		}
	}

	@Override
	protected void routine() throws AutoModeEndedException {
		if(right) {
			//Score Starting Cube
			runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.startToSwitchRight, false), 
					new SeriesAction(Arrays.asList(new SuperstructurePosition(30.0, 101.0, true), new Wait(0.2),
							new Shoot(-1.0))
							)
					)
				)
			);
			runAction(new FollowPath(PathElements.switchRightToShiftedCenter, true));
			//Score second 
			runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.shiftedCenterToSwitchCube1, false), 
					new SeriesAction(Arrays.asList(new OpenCloseJaw(true), new IntakeCube(true, true))
						)
					)
				)
			);
			runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.switchCube1ToShiftedCenter, true), 
					new SuperstructurePosition(30.0, 101.0, true))));
			runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.shiftedCenterToSwitchRight, false), 
					new SeriesAction(Arrays.asList(new Wait(2.0), 
							new Shoot(-1.0))
						)
					)
				)
			);
			/*runAction(new FollowPath(PathElements.switchRightToShiftedCenter2, true));
			//get third cube
			runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.shiftedCenterToSwitchCube2, false), 
					new SuperstructurePosition(10.0, 101.0, true), new SeriesAction(Arrays.asList(new OpenCloseJaw(true), new IntakeCube(false, true))
						)
					)
				)
			);
			runAction(new FollowPath(PathElements.switchCube2ToShiftedCenter, true));
			runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.shiftedCenterToSwitchRight2, false), 
					new SeriesAction(Arrays.asList(new SuperstructurePosition(30.0, 101.0, true), new Wait(2.0), 
							new Shoot(-1.0)))
					)
				)
			);*/
		} else {
			//Score Starting Cube
			runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.startToSwitchLeft, false), 
					new SeriesAction(Arrays.asList(new SuperstructurePosition(30.0, 101.0, true), new Wait(0.2),
							new Shoot(-1.0))
							)
					)
				)
			);
			runAction(new FollowPath(PathElements.switchLeftToShiftedCenter, true));
			//Score second 
			runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.shiftedCenterToSwitchCube1, false), 
					new SeriesAction(Arrays.asList(new OpenCloseJaw(true), new IntakeCube(true, true))
						)
					)
				)
			);
			runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.switchCube1ToShiftedCenter, true), 
					new SuperstructurePosition(30.0, 101.0, true))));
			runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.shiftedCenterToSwitchLeft, false), 
					new SeriesAction(Arrays.asList(new Wait(2.0), 
							new Shoot(-1.0))
						)
					)
				)
			);
			/*runAction(new FollowPath(PathElements.switchLeftToShiftedCenter2, true));
			//get third cube
			runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.shiftedCenterToSwitchCube2, false), 
					new SuperstructurePosition(10.0, 101.0, true), new SeriesAction(Arrays.asList(new OpenCloseJaw(true), new IntakeCube(false, true))
						)
					)
				)
			);
			runAction(new FollowPath(PathElements.switchCube2ToShiftedCenter, true));
			runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.shiftedCenterToSwitchLeft2, false), 
					new SeriesAction(Arrays.asList(new SuperstructurePosition(30.0, 101.0, true), new Wait(2.0), 
							new Shoot(-1.0)))
					)
				)
			);*/
		}
	}

}
