package control;

import org.ojalgo.matrix.PrimitiveMatrix;

public class StateSpacePlant {
	protected PrimitiveMatrix A, B, C, D;
	protected PrimitiveMatrix x;
	
	/**
	 * 
	 * @param inputs
	 * @param states
	 * @param outputs
	 * YOU BETTER SET ALL OF THESE MATRICES TO SOMETHING LATER
	 */
	public StateSpacePlant(int inputs, int states, int outputs) {
		A = PrimitiveMatrix.FACTORY.getBuilder(states, states).build();
		B = PrimitiveMatrix.FACTORY.getBuilder(states, inputs).build();
		C = PrimitiveMatrix.FACTORY.getBuilder(outputs, states).build();
		D = PrimitiveMatrix.FACTORY.getBuilder(outputs, inputs).build();
		x = PrimitiveMatrix.FACTORY.getBuilder(states, 1).build();
	}
	
	public StateSpacePlant(int inputs, int states, int outputs, PrimitiveMatrix A, PrimitiveMatrix B, PrimitiveMatrix C, PrimitiveMatrix D, PrimitiveMatrix x) {
		this(inputs, states, outputs);
		this.A = A;
		this.B = B;
		this.C = C;
		this.D = D;
		this.x = x;
	}
	
	public void update(PrimitiveMatrix u) {
		x = A().multiply(x).add(B().multiply(u));
	}
	
	public void setA(PrimitiveMatrix A) {
		this.A = A;
	}
	
	public void setB(PrimitiveMatrix B) {
		this.B = B;
	}
	
	public void setC(PrimitiveMatrix C) {
		this.C = C;
	}
	
	public void setD(PrimitiveMatrix D) {
		this.D = D;
	}
	
	public PrimitiveMatrix A() {
		return A;
	}
	
	public PrimitiveMatrix B() {
		return B;
	}
	
	public PrimitiveMatrix C() {
		return C;
	}
	
	public PrimitiveMatrix D() {
		return D;
	}
	
	public PrimitiveMatrix x() {
		return x;
	}
	
	public void setX(PrimitiveMatrix in) {
		x = in;
	}
	
	public PrimitiveMatrix y() {
		return C().multiply(x());
	}
}
