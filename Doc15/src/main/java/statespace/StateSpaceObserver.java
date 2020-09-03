package statespace;

import matrix.Matrix;

public class StateSpaceObserver {
    private StateSpacePlant plant;
    private Matrix L;

    //Make a new copy of the StateSpacePlant before you use it here
    public StateSpaceObserver( StateSpacePlant plant, Matrix L) {
        this.plant = plant;
        this.L = L;
    }

    public void update(Matrix x, Matrix y) {
        plant.setX(plant.x().plus(L.times(y.minus(plant.y()))));
    }

    public StateSpacePlant getPlant() {
        return plant;
    }

    public Matrix L() {
        return L;
    }

    public void setL(Matrix newL) {
        L = newL;
    }
}