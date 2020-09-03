package sim;

import org.ojalgo.matrix.PrimitiveMatrix;

import control.ProfileConstraints;
import control.ProfilePoint;
import control.StateSpaceController;
import control.StateSpaceObserver;
import control.StateSpacePlant;
import control.TrapezoidalProfile;
import linalg.Matrix;
import linalg.MatrixReader;

public class Arm {
	/*public final Matrix A = MatrixReader.parseMatrix("src\\python\\armpy\\armA.csv");
	public final Matrix B = MatrixReader.parseMatrix("src\\python\\armpy\\armB.csv");
	public final Matrix C = MatrixReader.parseMatrix("src\\python\\armpy\\armC.csv");
	public final Matrix D = MatrixReader.parseMatrix("src\\python\\armpy\\armD.csv");
	public final Matrix K = MatrixReader.parseMatrix("src\\python\\armpy\\armK.csv");
	public final Matrix L = MatrixReader.parseMatrix("src\\python\\armpy\\armL.csv");
	
	public Matrix R;
	public Matrix X;
	public Matrix X_hat;
	public Matrix Y;
	
	public Matrix minU = new Matrix(new double[][] {{-12.0}});
	public Matrix maxU = new Matrix(new double[][] {{12.0}});
	
	public final double dt = 0.02;
	
	public Arm(Matrix init) {
		X = init;
		Y = C.times(X);
		X_hat = init;
	}
	
	public Arm() {
		X = new Matrix(new double[][] {{0},{0}});
		Y = C.times(X);
		X_hat = new Matrix(new double[][] {{0},{0}});
	}
	
	public void setGoal(Matrix R) {
		this.R = R; 
	}
	
	public void update(Matrix U) {
		X = A.times(X).plus(B.times(U));
		Y = C.times(X).plus(D.times(U));
	}
	
	public void predictObserver(Matrix U) {
		X_hat = A.times(X_hat).plus(B.times(U));
	}
	
	public void correctObserver(Matrix U) {
		Matrix Y_hat = C.times(X_hat).plus(D.times(U));
		X_hat = X_hat.plus(A.inv().times(L.times(Y.minus(Y_hat))));
	}
	
	public void updateObserver(Matrix U) {
		Matrix Y_hat = C.times(X_hat).plus(D.times(U));
		X_hat = A.times(X_hat).plus(B.times(U)).plus(L.times(Y.minus(Y_hat)));
	}
	
	public Matrix calculate() {
		Matrix U = K.times(R.minus(X_hat));
		return U;
	}
	
	public Matrix capU(Matrix U) {
		double[][] newU = new double[U.getNumRows()][1];
		for(int i = 0; i < U.getNumRows(); i++) {
			newU[i][0] = Math.max(minU.get(i,0), Math.min(maxU.get(i, 0), U.get(i, 0)));
		}
		return new Matrix(newU);
	}
	
	public void updateY(double enc, double encRate) {
		Y = new Matrix(new double[][] {{enc, encRate}});
	}
	
	/*public void runTest(Matrix goal, int steps) throws InterruptedException {
		X_hat = X.plus(new Matrix(new double[][] {{0.03},{0.03}}));
		double currTime = 0.0;
		setGoal(goal);
		if(steps >= 0.0) {
			for(int i = 0; i < steps; i++) {
				Matrix input = capU(calculate());
				predictObserver(input);
				System.out.print("Observer pos: " + X_hat.get(0,0));
				update(input);
				System.out.print(", State pos: " + X.get(0, 0));
				correctObserver(input);
				System.out.println(", time: " + currTime);
				currTime += dt;
			}
		} else {
			while(true) {
				Matrix input = capU(calculate());
				predictObserver(input);
				System.out.print("Observer pos: " + X_hat.get(0,0));
				update(input);
				System.out.print(", State pos: " + X.get(0, 0));
				correctObserver(input);
				System.out.println(", time: " + currTime);
				currTime += dt;
				
				Thread.sleep(100);
			}
		}
	}*/
	private static final double MAX_SPEED = 1.538; // rad/s
	private static final double MAX_ACCELERATION = 36.151; // rad/s^2
	
	private StateSpacePlant plant = new StateSpacePlant(1,3,1);
	private StateSpaceController controller = new StateSpaceController(1, 3, 1);
	private StateSpacePlant observerPlant = new StateSpacePlant(1,3,1);
	private StateSpaceObserver observer;
	
	private ProfileConstraints constraints = new ProfileConstraints(MAX_SPEED, MAX_ACCELERATION);
	private ProfilePoint goal, initial, current;
	
	private final double dt = 0.005;
	private double disturbCount = 0.0;
	
	public Arm() {
		plant.setA(MatrixReader.parseMatrix("src\\python\\armpy\\armA.csv"));
		plant.setB(MatrixReader.parseMatrix("src\\python\\armpy\\armB.csv"));
		plant.setC(MatrixReader.parseMatrix("src\\python\\armpy\\armC.csv"));
		plant.setD(MatrixReader.parseMatrix("src\\python\\armpy\\armD.csv"));
		plant.setX(PrimitiveMatrix.FACTORY.makeZero(3, 1));
		
		controller.setA(plant.A());
		controller.setK(MatrixReader.parseMatrix("src\\python\\armpy\\armK.csv"));
		controller.setU_max(PrimitiveMatrix.FACTORY.rows(new double[][] {{12.0}}));
		controller.setU_min(PrimitiveMatrix.FACTORY.rows(new double[][] {{-12.0}}));
		controller.setKff(MatrixReader.parseMatrix("src\\python\\armpy\\armKff.csv"));
		controller.setR(PrimitiveMatrix.FACTORY.makeZero(3, 1));
		
		observerPlant.setA(MatrixReader.parseMatrix("src\\python\\armpy\\armA.csv"));
		observerPlant.setB(MatrixReader.parseMatrix("src\\python\\armpy\\armB.csv"));
		observerPlant.setC(MatrixReader.parseMatrix("src\\python\\armpy\\armC.csv"));
		observerPlant.setD(MatrixReader.parseMatrix("src\\python\\armpy\\armD.csv"));
		observerPlant.setX(PrimitiveMatrix.FACTORY.rows(new double[][] {{0.03},{0.0},{0.0}}));
		
		observer = new StateSpaceObserver(1,3,1,observerPlant,MatrixReader.parseMatrix("src\\python\\armpy\\armL.csv"));
		
		initial = new ProfilePoint(0.03, 0.03);
		current = new ProfilePoint(0.03, 0.03);
	}

	public PrimitiveMatrix x() {
		return plant.x();
	}
	
	public void updatePlant(PrimitiveMatrix u) {
		plant.update(u);
	}
	
	public void updateObserver(PrimitiveMatrix u) {
		observer.plant().update(u);
	}
	
	public PrimitiveMatrix updateController() {
		return controller.update(observer.x());
	}
	
	public StateSpacePlant plant() {
		return plant;
	}
	
	public ProfilePoint updateProfiledGoal() {
		TrapezoidalProfile profile = new TrapezoidalProfile(constraints, goal, current);
		current = profile.calculate(0.005);
		System.out.println(Math.toDegrees(current.getPos()));
		return current;
	}
	
	public double timeLeftUntil(double angle, double finalAngle) {
		/*if(goal.getPos() > angle) {
			return 0.0;
		}*/
		
		ProfilePoint endGoal = new ProfilePoint(finalAngle, 0.0);
		TrapezoidalProfile tempProfile = new TrapezoidalProfile(constraints, endGoal, current);
		
		return tempProfile.timeUntil(angle);
	}
	
	public void runTest(int steps) throws InterruptedException {
		double currTime = 0.0;
		if(steps >= 0.0) {
			for(int i = 0; i < steps; i++) {
				System.out.print("State pos: " + plant.x().get(0, 0));
				System.out.println(", time: " + currTime);
				currTime += dt;
				plant.update(updateController());
			}
		} else {
			while(true) {
				System.out.print("State pos: " + plant.x().get(0, 0));
				System.out.println(", time: " + currTime);
				currTime += dt;
				plant.update(updateController());
				disturb();
				
				Thread.sleep(100);
			}
		}
	}
	
	public void runObserverTest(int steps) throws InterruptedException {
		int currTime = 0;
		if(steps >= 0.0) {
			for(int i = 0; i < steps; i++) {
				System.out.print("State pos: " + Math.toDegrees(plant.x().get(0, 0)));
				System.out.print(", Observer pos: " + Math.toDegrees(observer.plant().x().get(0, 0)));
				System.out.println(", time: " + ((double) (currTime)) * 0.005);
				currTime++;
				PrimitiveMatrix u = updateController();
				plant.update(u);
				if((currTime % 4) == 0.0) {
					observer.update(u, plant.y());
				} else {
					observer.plant().update(u);
				}
			}
		} else {
			while(true) {
				System.out.print("State pos: " + Math.toDegrees(plant.x().get(0, 0)));
				System.out.print(", Observer pos: " + Math.toDegrees(observer.plant().x().get(0, 0)));
				System.out.println(", time: " + ((double) (currTime)) * 0.005);
				currTime++;
				PrimitiveMatrix u = updateController();
				plant.update(u);
				if((currTime % 4) == 0.0) {
					observer.update(u, plant.y());
				} else {
					observer.plant().update(u);
				}
				disturb();
				
				Thread.sleep(100);
			}
		}
	}
	
	public void runSetpointTest(PrimitiveMatrix setpoint, int steps) throws InterruptedException {
		controller.setR(setpoint);
		int currTime = 0;
		if(steps >= 0.0) {
			for(int i = 0; i < steps; i++) {
				System.out.print("State pos: " + Math.toDegrees(plant.x().get(0, 0)));
				System.out.print(", Observer pos: " + Math.toDegrees(observer.plant().x().get(0, 0)));
				System.out.println(", time: " + ((double) (currTime)) * 0.005);
				currTime++;
				PrimitiveMatrix u = updateController();
				plant.update(u);
				if((currTime % 4) == 0.0) {
					observer.update(u, plant.y());
				} else {
					observer.plant().update(u);
				}
			}
		} else {
			while(true) {
				System.out.print("State pos: " + Math.toDegrees(plant.x().get(0, 0)));
				System.out.print(", Observer pos: " + Math.toDegrees(observer.plant().x().get(0, 0)));
				System.out.println(", time: " + ((double) (currTime)) * 0.005);
				currTime++;
				PrimitiveMatrix u = updateController();
				plant.update(u);
				if((currTime % 4) == 0.0) {
					observer.update(u, plant.y());
				} else {
					observer.plant().update(u);
				}
				
				Thread.sleep(100);
			}
		}
	}
	
	public void runProfiledTest(Matrix setpoint, int steps) throws InterruptedException {
		goal = new ProfilePoint(setpoint.get(0, 0), setpoint.get(1, 0));
		TrapezoidalProfile profile = new TrapezoidalProfile(constraints, goal, initial);
		System.out.println(profile.totalTime());
		int currTime = 0;
		if(steps >= 0.0) {
			for(int i = 0; i < steps; i++) {
				System.out.print("State pos: " + Math.toDegrees(plant.x().get(0, 0)));
				System.out.print(", Observer pos: " + Math.toDegrees(observer.plant().x().get(0, 0)));
				System.out.print(", time until next pos: " + timeLeftUntil(observer.plant().x().get(0, 0), setpoint.get(0, 0)));
				System.out.println(", time: " + ((double) (currTime)) * 0.005);
				currTime++;
				updateProfiledGoal();
				controller.setR(PrimitiveMatrix.FACTORY.rows(new double[][] {{current.getPos()},{current.getVel()},{0.0}}));
				
				PrimitiveMatrix u = controller.update(observer.x(), controller.r());
				plant.update(u);
				if((currTime % 4) == 0.0) {
					observer.update(u, plant.y());
				} else {
					observer.plant().update(u);
				}
			}
		} else {
			while(true) {
				System.out.print("State pos: " + Math.toDegrees(plant.x().get(0, 0)));
				System.out.print(", Observer pos: " + Math.toDegrees(observer.plant().x().get(0, 0)));
				System.out.print(", time until next pos: " + timeLeftUntil(observer.plant().x().get(0, 0), setpoint.get(0, 0)));
				System.out.println(", time: " + ((double) (currTime)) * 0.005);
				currTime++;
				updateProfiledGoal();
				PrimitiveMatrix r = PrimitiveMatrix.FACTORY.rows(new double[][] {{current.getPos()},{current.getVel()},{0.0}});
				
				PrimitiveMatrix u = controller.update(observer.x(), r);
				System.out.println("Voltage: " + u.get(0, 0));
				plant.update(u);
				if((currTime % 4) == 0.0) {
					observer.update(u, plant.y());
				} else {
					observer.plant().update(u);
				}
				
				Thread.sleep(100);
			}
		}
	}
	
	public void disturb() {
		if(Math.random() < 1.0 / (100.0 * disturbCount)) {
			disturbCount += 1.0;
			if(Math.random() > 0.5) {
				PrimitiveMatrix disturbance = PrimitiveMatrix.FACTORY.rows(new double[][] {{1.0},{0.0}});
				System.out.println("Disturbed");
				plant.setX(plant.x().add(disturbance));
			} else {
				PrimitiveMatrix disturbance = PrimitiveMatrix.FACTORY.rows(new double[][] {{-1.0},{0.0}});
				System.out.println("Disturbed");
				plant.setX(plant.x().add(disturbance));
			}
		}
	}
	
}
