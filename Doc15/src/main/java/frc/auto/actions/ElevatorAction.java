package frc.auto.actions;

import frc.subsystems.Elevator;
import frc.subsystems.Superstructure;

public class ElevatorAction implements Action {
    private Superstructure superstructure = Superstructure.getInstance();
    private double height;
    private double tolerance = 0.1;

    public ElevatorAction(double height, double tolerance) {
        this.height = height;
        this.tolerance = tolerance;
    }

    public ElevatorAction(double height) {
        this.height = height;
    }

    @Override
    public void start() {
        superstructure.setDesiredHeight(height);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        return Math.abs(height - superstructure.getObservedState().height) < tolerance;
    }

    @Override
    public void done() {
        
    }
}