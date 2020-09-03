package statespace;

import matrix.Matrix;

public class StateSpacePlant {
    private Matrix A, B, C, D, x;

    public StateSpacePlant(Matrix A, Matrix B, Matrix C, Matrix D,
        Matrix x) {

        this.A = A;
        this.B = B;
        this.C = C;
        this.D = D;
        this.x = x;
        
    }

    public Matrix A() {
        return A;
    }

    public void setA(Matrix newA) {
        A = newA;
    }

    public Matrix B() {
        return B;
    }

    public void setB(Matrix newB) {
        B = newB;
    }

    public Matrix C() {
        return C;
    }

    public void setC(Matrix newC) {
        C = newC;
    }

    public Matrix D() {
        return D;
    }

    public void setD(Matrix newD) {
        D = newD;
    }

    public Matrix x() {
        return x;
    }

    public void setX(Matrix newX) {
        x = newX;
    }

    public Matrix y() {
        return C.times(x);
    }
}