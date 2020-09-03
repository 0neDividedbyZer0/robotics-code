package org.usfirst.frc.team1671.auto.modes;

import java.util.Arrays;

import org.usfirst.frc.team1671.auto.AutoModeBase;
import org.usfirst.frc.team1671.auto.AutoModeEndedException;
import org.usfirst.frc.team1671.auto.actions.FollowPath;
import org.usfirst.frc.team1671.auto.actions.IntakeCube;
import org.usfirst.frc.team1671.auto.actions.OpenCloseJaw;
import org.usfirst.frc.team1671.auto.actions.OpenLoopDrive;
import org.usfirst.frc.team1671.auto.actions.ParallelAction;
import org.usfirst.frc.team1671.auto.actions.SeriesAction;
import org.usfirst.frc.team1671.auto.actions.Shoot;
import org.usfirst.frc.team1671.auto.actions.SuperstructurePosition;
import org.usfirst.frc.team1671.auto.actions.TurnInPlace;
import org.usfirst.frc.team1671.auto.actions.Wait;
import org.usfirst.frc.team1671.robot.PathElements;

public class PathFollowingScaleMode extends AutoModeBase {
	private final boolean startRight;
	private boolean right;
	
	//do you want to run the right side?
		public PathFollowingScaleMode(boolean startRight, boolean right) {
			this.startRight = startRight;
			this.right = right;
		}
		
		public PathFollowingScaleMode(boolean startRight, String gameData) {
			if(gameData.charAt(1) == 'R') {
				right = true;
			} else {
				right = false;
			}
			this.startRight = startRight;
		}
	
	@Override
	protected void routine() throws AutoModeEndedException {
		if(right) {
			if(startRight != true) {
				//Cross baseline
				runAction(new Wait(5.0));
				runAction(new OpenLoopDrive(0.3, 0.3, 3.5, false));
			} else {
				//Score first cube
				runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.startToScaleRightSame, false), 
						new SeriesAction(Arrays.asList(new Wait(2.0), new SuperstructurePosition(60.0, 45.0, true), new Wait(0.2),
								new Shoot(-1.0))
								)
						)
					)
				);
				runAction(new OpenLoopDrive(-0.3, -0.3, 0.75, false));
				runAction(new Wait(0.75));
				runAction(new SuperstructurePosition(0.0, 101.0, true));
				//runAction(new TurnInPlace(PathElements.finalAngleOfSpin1AbsRight, true));
				/*runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.scaleRightToFenceCubeRight1, false), 
						new SeriesAction(Arrays.asList(new SuperstructurePosition(0.0, 101.0, true), new OpenCloseJaw(true),
								new IntakeCube(true, true))
							)
						)
					)
				);
				runAction(new FollowPath(PathElements.fenceCubeRight1Reverse, true));*/
			}
		} else {
			if(startRight == true) {
				runAction(new Wait(5.0));
				runAction(new OpenLoopDrive(0.3, 0.3, 3.5, false));
			} else {
				runAction(new ParallelAction(Arrays.asList(new FollowPath(PathElements.startToScaleLeftSame, false), 
						new SeriesAction(Arrays.asList(new Wait(2.0), new SuperstructurePosition(60.0, 45.0, true), new Wait(0.2),
								new Shoot(-1.0))
								)
						)
					)
				);
				runAction(new OpenLoopDrive(-0.3, -0.3, 0.75, false));
				runAction(new Wait(0.75));
				runAction(new SuperstructurePosition(0.0, 101.0, true));
				//runAction(new TurnInPlace(PathElements.finalAngleOfSpin1AbsLeft, true));
			}
		}
	}

}
