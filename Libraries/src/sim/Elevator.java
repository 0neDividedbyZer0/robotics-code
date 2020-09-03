package sim;

import control.control.ProfileConstraints;
import control.control.ProfilePoint;
import control.control.TrapezoidalProfile;

public class Elevator {
	ProfileConstraints constraints = new ProfileConstraints(61.492, 139.852);
	ProfilePoint goal = goal = new ProfilePoint(30.0, 0.0);
	ProfilePoint updatingGoal = new ProfilePoint(0.0, 0.0);
	
	public ProfilePoint updateProfile() {
		TrapezoidalProfile profile = new TrapezoidalProfile(constraints, goal, updatingGoal);
		updatingGoal = profile.calculate(0.005);
		return updatingGoal;
	}
}
