package frc.statemachines;

import frc.robot.Constants;

public class GadgetStateMachine {

    GadgetState commandedState = new GadgetState();
    GadgetCommand command = new GadgetCommand();

    GadgetSystemState state = GadgetSystemState.SYNCHRONIZED;

    private double maxExtension = Constants.MAX_LENGTH;
    private double minExtension = Constants.MIN_LENGTH;

    private double drivingVelocity = 0.0;
    

    public enum GadgetSystemState {
        OPEN_LOOP,
        SYNCHRONIZED,
        UNSYNCHRONIZED
    }

    public enum GadgetWantedState {
        MANUAL,
        UNSYNCED,
        SYNCED
    }

    //Please make sure max > min
    private double constrainHeight(double wantedHeight) {
        //This restricts the wanted height between max and min extensions
        return Math.max(Math.min(wantedHeight, maxExtension),minExtension); 
    
    }

    public synchronized void setFrontActuatorHeight(double wantedHeight) {
        command.frontActuatorHeight = constrainHeight(wantedHeight);
    }

    public synchronized void setRearActuatorHeight(double wantedHeight) {
        command.rearActuatorHeight = constrainHeight(wantedHeight);
    }

    public synchronized void setVelocity(double wantedVelocity) {
        drivingVelocity = wantedVelocity;
    }


    public synchronized GadgetCommand update(double timestamp, GadgetWantedState wanted, GadgetState current) {
        synchronized(GadgetStateMachine.this) {
            GadgetSystemState newState;

            switch(state) {
                case SYNCHRONIZED:
                    newState = handleSynchronizedTransitions(wanted, current);
                    break;
                case UNSYNCHRONIZED:
                    newState = handleUnsynchronizedTransitions(wanted, current);
                    break;
                case OPEN_LOOP:
                    newState = handleOpenLoopTransitions(wanted, current);
                    break;
                default:
                    System.out.println("Unexpected Gadget State: " + state);
                    newState = state;
            }

            if(newState != state) {
				System.out.println(timestamp + ": Go Go Gadget changed state: " + state + " -> " + newState);
				state = newState;
            }
            
            if(newState == GadgetSystemState.SYNCHRONIZED) {
                command.synced = true;
            } else {
                command.synced = false;
            }

            if(newState == GadgetSystemState.OPEN_LOOP) {
                command.frontActuatorHeight = constrainHeight(current.frontActuatorLength);
                command.rearActuatorHeight = constrainHeight(current.rearActuatorLength);
            }
            
            switch(state) {
                case SYNCHRONIZED:
                    getSynchronizedCommandedState();
                    break;
                case UNSYNCHRONIZED:
                    getUnsynchronizedCommandedState();
                    break;
                case OPEN_LOOP:
                    getOpenLoopCommandedState();
                    break;
                default:
                    System.out.println("Unexpected Gadget output state: " + state);
                    break;
            }

            return command;

        }
    }

    private GadgetSystemState handleDefaultTransitions(GadgetWantedState wanted, GadgetState current) {
        if(wanted == GadgetWantedState.SYNCED) {
            return GadgetSystemState.SYNCHRONIZED;
        } else if(wanted == GadgetWantedState.UNSYNCED){
            return GadgetSystemState.UNSYNCHRONIZED;
        } else {
            return GadgetSystemState.OPEN_LOOP;
        }
    }

    private GadgetSystemState handleSynchronizedTransitions(GadgetWantedState wanted, GadgetState current) {
        return handleDefaultTransitions(wanted, current);
    }

    private GadgetSystemState handleUnsynchronizedTransitions(GadgetWantedState wanted, GadgetState current) {
        return handleDefaultTransitions(wanted, current);
    }

    private GadgetSystemState handleOpenLoopTransitions(GadgetWantedState wanted, GadgetState current) {
        return handleDefaultTransitions(wanted, current);
    }
    
    public void resetDrive() {
        command.velocity = 0.0;
    }

    private void getSynchronizedCommandedState() {
        command.synced = true;
        command.velocity = drivingVelocity;
        command.openLoop = false;
    }

    private void getUnsynchronizedCommandedState() {
        command.synced = false;
        command.velocity = 0.0;
        command.openLoop = false;
        
    }

    private void getOpenLoopCommandedState() {
        command.synced = false;
        command.velocity = 0.0;
        command.openLoop = true;
    }

    

    
}