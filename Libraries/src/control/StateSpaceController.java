package control;

import org.ojalgo.matrix.PrimitiveMatrix;

import linalg.Matrix;
import utils.math.Util;

public class StateSpaceController {
	protected PrimitiveMatrix K, Kff;
	protected PrimitiveMatrix A;
	protected PrimitiveMatrix r;
	protected PrimitiveMatrix u_min, u_max;
	
	/**
	 * 
	 * @param inputs
	 * @param states
	 * @param outputs
	 * YOU BETTER SET THESE ALL LATER
	 */
	public StateSpaceController(int inputs, int states, int outputs) {
		A = PrimitiveMatrix.FACTORY.getBuilder(states, states).build();
		K = PrimitiveMatrix.FACTORY.getBuilder(inputs, states).build();
		Kff = PrimitiveMatrix.FACTORY.getBuilder(inputs, states).build();
		r = PrimitiveMatrix.FACTORY.getBuilder(states, 1).build();
		u_min = PrimitiveMatrix.FACTORY.getBuilder(inputs, 1).build();
		u_max = PrimitiveMatrix.FACTORY.getBuilder(inputs, 1).build();
	}
	
	/**
	 * 
	 * @param inputs
	 * @param states
	 * @param outputs
	 * @param K
	 * @param u_min
	 * @param u_max
	 * You better set A to something. The others can be set to Matrix.zero()
	 */
	public StateSpaceController(int inputs, int states, int outputs, PrimitiveMatrix K, PrimitiveMatrix u_min, PrimitiveMatrix u_max) {
		this(inputs, states, outputs);
		this.K = K;
		this.u_min = u_min;
		this.u_max = u_max;
	}
	
	/**
	 * 
	 * @param inputs
	 * @param states
	 * @param outputs
	 * @param K
	 * @param Kff
	 * @param A
	 * @param u_min
	 * @param u_max
	 * Set the others to zero later
	 */
	public StateSpaceController(int inputs, int states, int outputs, PrimitiveMatrix K, PrimitiveMatrix Kff, PrimitiveMatrix A, PrimitiveMatrix u_min, PrimitiveMatrix u_max) {
		this(inputs, states, outputs);
		this.K = K;
		this.Kff = Kff;
		this.A = A;
		this.u_min = u_min;
		this.u_max = u_max;
	}
	
	public PrimitiveMatrix update(PrimitiveMatrix x) {
		PrimitiveMatrix term1 = K().multiply(r().subtract(x));
		PrimitiveMatrix term2 = Kff().multiply(r().subtract(A().multiply(r())));
		PrimitiveMatrix u = term1.add(term2);
		return Util.cap(u, u_min, u_max);
	}
	
	public PrimitiveMatrix update(PrimitiveMatrix x, PrimitiveMatrix r) {
		PrimitiveMatrix term1 = K().multiply(r().subtract(x));
		PrimitiveMatrix term2 = Kff().multiply(r.subtract(A().multiply(r())));
		this.r = r;
		PrimitiveMatrix u = term1.add(term2);
		return Util.cap(u, u_min, u_max);
	}
	
	public void setK(PrimitiveMatrix K) {
		this.K = K;
	}
	
	public void setKff(PrimitiveMatrix Kff) {
		this.Kff = Kff;
	}
	
	public void setA(PrimitiveMatrix A) {
		this.A = A;
	}
	
	public void setR(PrimitiveMatrix r) {
		this.r = r;
	}
	
	public void setU_min(PrimitiveMatrix u_min) {
		this.u_min = u_min;
	}
	
	public void setU_max(PrimitiveMatrix u_max) {
		this.u_max = u_max;
	}
	
	public PrimitiveMatrix K() {
		return K;
	}
	
	public PrimitiveMatrix Kff() {
		return Kff;
	}
	
	public PrimitiveMatrix A() {
		return A;
	}
	
	public PrimitiveMatrix r() {
		return r;
	}
	
	public PrimitiveMatrix u_min() {
		return u_min;
	}
	
	public PrimitiveMatrix u_max() {
		return u_max;
	}
}
