package control;

import org.ojalgo.matrix.PrimitiveMatrix;

public class StateSpaceObserver {
	protected StateSpacePlant plant;
	protected PrimitiveMatrix L;
	
	public StateSpaceObserver(int inputs, int states, int outputs) {
		plant = new StateSpacePlant(inputs, states, outputs);
		L = PrimitiveMatrix.FACTORY.getBuilder(states, outputs).build();
	}
	
	public StateSpaceObserver(int inputs, int states, int outputs, StateSpacePlant plant, PrimitiveMatrix L) {
		this(inputs, states, outputs);
		this.plant = plant;
		this.L = L;
	}
	
	public void update(PrimitiveMatrix u, PrimitiveMatrix y) {
		plant.setX(plant.x().add(L().multiply(y.subtract(plant.y()))));
		plant.update(u);
	}
	
	public PrimitiveMatrix x() {
		return plant.x();
	}
	
	public void setL(PrimitiveMatrix L) {
		this.L = L;
	}
	
	public void setPlant(StateSpacePlant plant) {
		this.plant = plant;
	}
	
	public StateSpacePlant plant() {
		return plant;
	}
	
	public PrimitiveMatrix L() {
		return L;
	}

}
