
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

import linalg.Matrix;
import sim.PIDController;

public class Test {
	/*private static final double dt = 0.02;
	private double goal;
	private PIDController controller;
	
	
	//returns voltage
	public double update(double encoder) {
		//double temp =  goal - 110.12340842 * ArmSim.getInstance().getAngle() - -160.37178794 * ArmSim.getInstance().getVel();
		//System.out.println(temp);
		return controller.calculate(true, false, encoder);//Math.max(-1.0 , Math.min(1.0, temp));//controller.calculate(true, false, encoder);
	}
	
	public Test() {
		controller = new PIDController(0.5, 0.0, 0.1, 0.0, dt);
		controller.setInputRange(ArmSim.d2r(-5.0), ArmSim.d2r(96.0));
		controller.setOutputRange(-1.0, 1.0);
	}
	//degrees
	private void setGoal(double goal) {
		this.goal = goal;
		controller.setSetpoint(goal);
	}
	
	private boolean checkGoal() {
		if (Math.abs(ArmSim.getInstance().getAngle() - goal) < 3.0 * Math.PI / 180.0 && Math.abs(ArmSim.getInstance().getVel()) < 0.01) {
			return true;
		} else {
			return false;
		}
	}
	
	public void testArm(double goal, int totalTime) {
		try {
		String csvFile = "test.csv";
        FileWriter writer = new FileWriter(csvFile);
			setGoal(goal);
			double currTime = 0.0;
			if(totalTime > 0) {
				while(currTime < totalTime) {
					ArmSim.getInstance().RK4(update(ArmSim.getInstance().getAngle()), dt);
					if(!ArmSim.getInstance().violatedLimits()) {
						if(checkGoal()) {
							System.out.println("Completed test successfully in " + currTime + " seconds, Arm is at " + ArmSim.r2d(ArmSim.getInstance().getAngle()));
							return;
						}
					} else {
						System.out.println("The arm violated its limits, Angle: " + ArmSim.r2d(ArmSim.getInstance().getAngle()) + ", Vel: " + ArmSim.r2d(ArmSim.getInstance().getVel()) + ", at " + currTime + " seconds");
						return;
					}
					currTime += dt;
				}
				System.out.println("The arm violated its time constraint, Angle: " + ArmSim.r2d(ArmSim.getInstance().getAngle()) + ", Vel: " + ArmSim.r2d(ArmSim.getInstance().getVel()));
				return;
			} else if(totalTime < 0) {
				while(true) {
					System.out.println("angle: " + ArmSim.r2d(ArmSim.getInstance().getAngle()) + " velocity: " + ArmSim.r2d(ArmSim.getInstance().getVel()) + " time: " + currTime);
					ArmSim.getInstance().RK4(update(ArmSim.getInstance().getAngle()), dt);
					/*if(!ArmSim.getInstance().violatedLimits()) {
						if(checkGoal()) {
							System.out.println("Completed test successfully in " + currTime + " seconds, Arm is at " + ArmSim.r2d(ArmSim.getInstance().getAngle()));
							return;
						}
						
					} else {
						System.out.println("The arm violated its limits, Angle: " + ArmSim.r2d(ArmSim.getInstance().getAngle()) + ", Vel: " + ArmSim.r2d(ArmSim.getInstance().getVel()) + ", at " + currTime + " seconds");
						return;
					}*/
					/*currTime += dt;
					Thread.sleep(30);
				}
			}
		} catch(Exception e) {
			e.printStackTrace();
		}
	}*/
	
	
	
	
	
	

}
