package matrix;

public class MatrixArithmeticException extends RuntimeException {
    private int leftRows, leftCols, rightRows, rightCols;

    public MatrixArithmeticException(String s, Matrix left, Matrix right) {
        super(s);
        leftRows = left.getRows();
        leftCols = left.getCols();
        rightRows = right.getRows();
        rightCols = right.getCols();
    }

    public MatrixArithmeticException(String s) {
        super(s);
    }
}