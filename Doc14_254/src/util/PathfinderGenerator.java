package util;

import java.io.File;
import java.util.List;

import org.usfirst.frc.team1671.robot.RobotMap;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class PathfinderGenerator {
	//ft/s
	private static final FitMethod fit = FitMethod.HERMITE_QUINTIC;
	private static final int samples = Trajectory.Config.SAMPLES_FAST;
	private static final Trajectory.Config config = new Trajectory.Config(fit, samples, RobotMap.kLooperDt, 
			RobotMap.FOLLOWER_MAX_V, RobotMap.FOLLOWER_MAX_A, RobotMap.FOLLOWER_MAX_J);
	private static TankModifier modifier;
	
	public static TankModifier generateTrajectory(List<Waypoint> points) {
		Trajectory trajectory =  Pathfinder.generate((Waypoint[]) points.toArray(), config);
		modifier = new TankModifier(trajectory);
		modifier.modify(RobotMap.WHEELBASE);
		return modifier;
	}
	
	public static TankModifier getModifier() {
		return modifier;
	}
	
	private static void flipTrajectory(Trajectory traj) {
		
	}
	
	public static void save(String fileName, Trajectory trajectory) {
		File file = new File(fileName);
		Pathfinder.writeToFile(file, trajectory);
	}
	
	public static Trajectory read(String fileName) {
		File file = new File(fileName);
		return Pathfinder.readFromFile(file);
	}
	
	private PathfinderGenerator() {
	}

}
