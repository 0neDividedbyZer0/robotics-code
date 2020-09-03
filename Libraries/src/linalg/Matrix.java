package linalg;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class Matrix {
    private double[][] array;
    private int rows, cols;

    public Matrix(double[][] inputArray) {
        if(inputArray == null) {
            throw new NullPointerException("Input Array to Matrix is null");
        }
        if(inputArray.length <= 0) {
            throw new ArrayIndexOutOfBoundsException("Input Array Rows to Matrix is invalid: " + inputArray.length);
        }
        if(inputArray[0].length <= 0) {
            throw new ArrayIndexOutOfBoundsException("Input Array Cols to Matrix is invalid: " + inputArray[0].length);
        }
        array = inputArray.clone();
        rows = array.length;
        cols = array[0].length;
    }

    public Matrix plus(Matrix other) {
        if(rows != other.getRows() || cols != other.getCols()) {
            throw new MatrixArithmeticException("Invalid Matrix Addition, left dims: " + this.rows + "x" + this.cols + 
                ", right dims: " + other.getRows() + "x" + other.getCols(), this, other);
        }

        double[][] result = new double[rows][cols];
        for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {
                result[i][j] = array[i][j] + other.get(i, j);
            }
        }

        return new Matrix(result);
    }

    public Matrix minus(Matrix other) {
        if(rows != other.getRows() || cols != other.getCols()) {
            throw new MatrixArithmeticException("Invalid Matrix Subtraction, left dims: " + this.rows + "x" + this.cols + 
                ", right dims: " + other.getRows() + "x" + other.getCols(), this, other);
        }

        double[][] result = new double[rows][cols];
        for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {
                result[i][j] = array[i][j] - other.get(i, j);
            }
        }

        return new Matrix(result);
    }

    public Matrix times(double a) {
        double[][] result = new double[rows][cols];
        for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {
                result[i][j] = array[i][j] * a;
            }
        }

        return new Matrix(result);
    }

    public Matrix times(Matrix other) {
        if(cols != other.getRows()) {
            throw new MatrixArithmeticException("Invalid Matrix Multiplication, left dims: " + this.rows + "x" + this.cols + 
                ", right dims: " + other.getRows() + "x" + other.getCols(), this, other);
        }

        double[][] result = new double[rows][other.getCols()];
        for(int i = 0; i < rows; i++) {
            for(int j = 0; j < other.getCols(); j++) {
                double sum = 0.0;
                for(int k = 0; k < cols; k++) {
                    sum += array[i][k] * other.get(k, j);
                }
                result[i][j] = sum;
            }
        }

        return new Matrix(result);
    }

    /**
     * Transpose
     * @return transposed Matrix
     */
    public Matrix T() {
        double[][] result = new double[cols][rows];
        for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {
                result[i][j] = array[j][i];
            }
        }

        return new Matrix(result);
    }

    public Matrix inv() {
        if(getRows() != getCols()) {
            throw new MatrixArithmeticException("Matrix not Square :" + getRows() + "x" + getCols());
        }
        double determinant = det(this);
        if(Math.abs(determinant) <= 1.0E-8) {
            throw new MatrixArithmeticException("Matrix has no inverse, determinant is 0");
        }

        return adj(this).times(1.0 / determinant);

    }

    public static Matrix adj(Matrix input) {
        if(input.getRows() != input.getCols()) {
            throw new MatrixArithmeticException("Matrix not Square :" + input.getRows() + "x" + input.getCols());
        }

        double[][] result = new double[input.getRows()][input.getCols()];
        for(int i = 0; i < input.getRows(); i++) {
            for(int j = 0; j < input.getCols(); j++) {
                double sign;
                if((i + j) % 2 == 0) {
                    sign = 1.0;
                } else {
                    sign = -1.0;
                }
                result[j][i] = sign * det(input.minor(i, j));
            }
        }
        return new Matrix(result);
        
    }

    public static double det(Matrix input) {
        if(input.getRows() != input.getCols()) {
            throw new MatrixArithmeticException("Matrix not Square :" + input.getRows() + "x" + input.getCols());
        }

        if(input.rows == 1) {
            return input.get(0,0);
        } else if(input.rows == 2) {
            return input.get(0,0) * input.get(1,1) - input.get(0,1) * input.get(1,0);
        } else {
            double sum = 0.0;
            for(int i = 0; i < input.rows; i++) {
                double sign;
                if(i % 2 == 0) {
                    sign = 1.0;
                } else {
                    sign = -1.0;
                }
                
                sum += sign * input.get(0, i) * det(input.minor(0, i));
            }
            return sum;
        }
    } 

    public Matrix getRowVector(int r) {
        if(r < 0) {
            throw new IndexOutOfBoundsException("Index out of bounds: a negative index was queried");
        }

        double[][] result = new double[1][cols];

        for(int i = 0; i < cols; i++) {
            result[0][i] = array[r][i];
        }

        return new Matrix(result);


    }

    public Matrix getColVector(int c) {
        if(c < 0) {
            throw new IndexOutOfBoundsException("Index out of bounds: a negative index was queried");
        }

        double[][] result = new double[rows][1];

        for(int i = 0; i < rows; i++) {
            result[i][0] = array[i][c];
        }

        return new Matrix(result);


    }

    

    public int rank() {
        
        
        double epsilon = 1.0E-9;
        int rank = Math.max(cols, rows);
        ArrayList<Boolean> row_selected = new ArrayList<Boolean>();
        for(int i = 0; i < rows; i++) {
            row_selected.add(Boolean.valueOf(false));
        }

        
        double[][] copy = array.clone();
        

        for(int i = 0; i < cols; i++) {
            int j;
            for(j = 0; j < rows; j++) {
                if(!row_selected.get(j).booleanValue() && Math.abs(copy[j][i]) > epsilon) {
                    break;
                }
            }
            if(j == rows) {
                rank--;
            } else {
                row_selected.set(i, Boolean.valueOf(true));
                for(int p = i + 1; p < cols; p++) {
                    copy[j][p] /= copy[j][i];
                }
                for(int k = 0; k < rows; k++) {
                    if(k != j && Math.abs(copy[k][i]) > epsilon) {
                        for(int p = i+1; p < rows; p++) {
                            copy[k][p] -= copy[j][p] * copy[k][i];
                        }
                    }
                }
            }
        }
       
        return rank;

    }

   
    public Matrix minor(int r, int c) {
        if(r < 0 || c < 0) {
            throw new IndexOutOfBoundsException("Index out of bounds: a negative index was queried");
        }

        if(rows == 1) {
            throw new MatrixArithmeticException("No Minor exists for a 1x1 Matrix");
        }

        double[][] result = new double[rows - 1][cols - 1];
        for(int i = 0; i < rows - 1; i++) {
            for(int j = 0; j < cols - 1; j++) {
                if(i < r) {
                    if(j < c) {
                        result[i][j] = array[i][j];
                    } else {
                        result[i][j] = array[i][j + 1];
                    }
                } else {
                    if(j < c) {
                        result[i][j] = array[i + 1][j];
                    } else {
                        result[i][j] = array[i + 1][j + 1];
                    }
                }
            }
        }
        return new Matrix(result);
    }

    public static Matrix zero(int r, int c) {
        double[][] result = new double[r][c];
        for(int i = 0; i < r; i++) {
            for(int j = 0; j < c; j++) {
                result[i][j] = 0.0;
            }
        }

        return new Matrix(result);
    }

    public static Matrix zero(int r) {
        return zero(r, r);
    }

    public static Matrix I(int r) {
        double[][] result = new double[r][r];
        for(int i = 0; i < r; i++) {
            for(int j = 0; j < r; j++) {
                if(i == j) {
                    result[i][j] = 1.0; 
                } else {
                    result[i][j] = 0.0;
                }
            }
        }

        return new Matrix(result);
    }

    public static Matrix constants(int r, int c, double val) {
        double[][] result = new double[r][c];
        for(int i = 0; i < r; i++) {
            for(int j = 0; j < c; j++) {
                result[i][j] = val;
            }
        }

        return new Matrix(result);
    }

    public int getRows() {
        return rows;
    }

    public int getCols() {
        return cols;
    }

    public double get(int i, int j) {
        return array[i][j];
    }

    public void set(int i, int j, double val) {
        array[i][j] = val;
    }

    public double[][] get2DArray() {
        return array;
    }

    public String toString() {
        String s = "";

        for(int i = 0; i < rows; i++) {
            s += "[ ";
            for(int j = 0; j < cols; j++) {
                s += array[i][j] + " ";
            }
            s += "]\n";
        }
        return s;
    }
}