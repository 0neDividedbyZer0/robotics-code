package control.control;

public class TrapezoidalProfile {
	private ProfileConstraints constraints;
	private ProfilePoint initial, goal;
	private double sign;
	//Important time positions
	private double endAccel, endFullSpeed, endDeccel;
	
	public TrapezoidalProfile(ProfileConstraints constraintsIn, ProfilePoint goalPoint, ProfilePoint initialPoint) {
		if(shouldFlipAccel(constraintsIn, goalPoint, initialPoint)) {
			sign = -1.0;
		} else {
			sign = 1.0;
		}
		
		constraints = constraintsIn;
		goal = flipPoint(goalPoint);
		initial = flipPoint(initialPoint);
		
		//Check for truncation (In other words, is the goal so close that the profile is triangular?)
		double timeCutoffBegin = initial.getVel() / constraints.getMaxAcceleration();
		double distanceCutoffBegin = timeCutoffBegin * timeCutoffBegin * constraints.getMaxAcceleration() / 2.0;
		
		double timeCutoffEnd = goal.getVel() / constraints.getMaxAcceleration();
		double distanceCutoffEnd = timeCutoffEnd * timeCutoffEnd * constraints.getMaxAcceleration() / 2.0;
		
		//Actual trapezoidal profile generation
		double completeDist = distanceCutoffBegin + (goal.getPos() - initial.getPos()) + distanceCutoffEnd;
		double accelTime = constraints.getMaxSpeed() / constraints.getMaxAcceleration();
		double fullSpeedDist = completeDist - accelTime * accelTime * constraints.getMaxAcceleration();
		
		//Handle the Triangular profile case
		if(fullSpeedDist < 0.0) {
			accelTime = Math.sqrt(completeDist / constraints.getMaxAcceleration());
			fullSpeedDist = 0.0;
		}
		
		endAccel = accelTime - timeCutoffBegin;
		endFullSpeed = endAccel + fullSpeedDist / constraints.getMaxAcceleration();
		endDeccel = endFullSpeed + accelTime - timeCutoffEnd;
		
		
	}
	
	/**
	 * 
	 * @param t time to query
	 * @return a profile position and velocity at t
	 */
	public ProfilePoint calculate(double t) {
		ProfilePoint result = new ProfilePoint(initial.pos, initial.vel);
		
		if(t < endAccel) {
			result.vel += t * constraints.getMaxAcceleration();
			result.pos += (initial.vel + constraints.getMaxAcceleration() * t / 2.0) * t;
		} else if(t < endFullSpeed) {
			result.vel = constraints.getMaxSpeed();
			result.pos += (initial.vel + endAccel * constraints.getMaxAcceleration() / 2.0) * endAccel + 
					constraints.getMaxSpeed() * (t - endAccel);
		} else if(t <= endDeccel) {
			result.vel = goal.vel + (endDeccel - t) * constraints.getMaxAcceleration();
			double timeLeft = endDeccel - t;
			result.pos = goal.pos - (goal.vel + timeLeft * constraints.getMaxAcceleration() / 2.0) * timeLeft;
		} else {
			result.vel = goal.vel;
			result.pos = goal.pos;
		}
		return flipPoint(result);
	}
	
	public double timeUntil(double target) {
		double position = initial.pos * sign;
		double velocity = initial.vel * sign;
		
		double timeEndAccel = endAccel * sign;
		double timeEndFullSpeed = endFullSpeed * sign - timeEndAccel;
		
		if(target < position) {
			timeEndAccel *= -1.0;
			timeEndFullSpeed *= -1.0;
			velocity *= -1.0;
		}
		
		timeEndAccel = Math.max(timeEndAccel, 0.0);
		timeEndFullSpeed = Math.max(timeEndFullSpeed, 0.0);
		double timeEndDeccel = endDeccel - timeEndAccel - timeEndFullSpeed;
		timeEndDeccel = Math.max(timeEndDeccel, 0.0);
		
		double acceleration = constraints.getMaxAcceleration();
		double decceleration = -constraints.getMaxAcceleration();
		
		double distToTarget = Math.abs(target - position);
		
		if(distToTarget < 1.0E-6) {
			return 0.0;
		}
		
		double accelDist = (velocity + acceleration * timeEndAccel / 2.0) * timeEndAccel;
		double deccelVel;
		if(timeEndAccel > 0.0) {
			deccelVel = Math.sqrt(Math.abs(velocity * velocity + 2.0 * acceleration * accelDist));
		} else {
			deccelVel = velocity;
		}
		double deccelDist = (deccelVel + decceleration * timeEndDeccel / 2.0) * timeEndDeccel;
		deccelDist = Math.max(deccelDist, 0.0);
		
		double fullSpeedDist = constraints.getMaxSpeed() * timeEndFullSpeed;
		
		if(accelDist > distToTarget) {
			accelDist = distToTarget;
			fullSpeedDist = 0.0;
			deccelDist = 0.0;
		} else if(accelDist + timeEndFullSpeed > distToTarget) {
			fullSpeedDist = distToTarget - accelDist;
			deccelDist = 0.0;
		} else {
			deccelDist = distToTarget - fullSpeedDist - accelDist;
		}
		
		double accelTime = (-velocity + Math.sqrt(Math.abs(velocity * velocity + 2.0 * acceleration * accelDist))) 
				/ acceleration;
		double deccelTime = (-deccelVel + Math.sqrt(Math.abs(deccelVel * deccelVel + 2.0 * decceleration * deccelDist))) 
				/ decceleration;
		
		double fullSpeedTime = fullSpeedDist / constraints.getMaxSpeed();
		
		return accelTime + fullSpeedTime + deccelTime;
	}
	
	public ProfileConstraints getConstraints() {
		return constraints;
	}
	
	public double totalTime() {
		return endDeccel;
	}
	
	private boolean shouldFlipAccel(ProfileConstraints constraints, ProfilePoint goal, ProfilePoint initial) {
		//change in velocity
		double dv = goal.getVel() - initial.getVel();
		//change in position
		double dx = goal.getPos() - initial.getPos();
		
		double t = Math.abs(dv) / constraints.getMaxAcceleration();
		boolean isFlipped = t * (dv / 2.0 + initial.getVel()) > dx;
		return isFlipped;
	}
	
	private ProfilePoint flipPoint(ProfilePoint in) {
		ProfilePoint result = new ProfilePoint(in.getPos() * sign, in.getVel() * sign);
		return result;
	}

}
