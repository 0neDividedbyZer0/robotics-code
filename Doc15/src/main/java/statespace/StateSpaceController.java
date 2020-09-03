package statespace;

import matrix.Matrix;

public class StateSpaceController {
    private Matrix K, Kff, A, r, u_min, u_max;

    public StateSpaceController(Matrix K, Matrix Kff, Matrix A, Matrix r,
        Matrix u_min, Matrix u_max) {

        this.K = K;
        this.Kff = Kff;
        this.A = A;
        this.r = r;
        this.u_min = u_min;
        this.u_max = u_max;
    }

    public Matrix update(Matrix x) {
        Matrix u = K.times(r.minus(x));
        u = u.plus(Kff.times(r.minus(A.times(r))));
        return capMatrix(u);
    }

    public Matrix update(Matrix x, Matrix r) {
        Matrix u = K.times(this.r.minus(x));
        u = u.plus(Kff.times(r.minus(A.times(this.r))));
        this.r = r;
        return capMatrix(u);
    }

    public Matrix A() {
        return A;
    }

    public void setA(Matrix newA) {
        A = newA;
    }

    public Matrix r() {
        return r;
    }

    public void setR(Matrix newR) {
        r = newR;
    }

    public Matrix K() {
        return K;
    }

    public void setK(Matrix newK) {
        K = newK;
    }

    public Matrix Kff() {
        return Kff;
    }

    public void setKff(Matrix newKff) {
        K = newKff;
    }

    public void setU_max(Matrix newU_max) {
        u_max = newU_max;
    }

    public Matrix u_max() {
        return u_max;
    }

    public void setU_min (Matrix newU_min) {
        u_min = newU_min;
    }

    public Matrix u_min() {
        return u_min;
    }

    private Matrix capMatrix(Matrix u) {
        for(int i = 0; i < u_max.getRows(); i++) {
            double val = Math.max(Math.min(u.get(i, 0), u_max.get(i, 0)), u_min.get(i, 0));
            u.set(i, 0, val); 
        }
        return u;
    }

}