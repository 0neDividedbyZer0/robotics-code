package linalg;

public class MatrixIndexException extends Exception {

	/**
	 * 
	 */
	private static final long serialVersionUID = 2886517666915889615L;
	
	public MatrixIndexException(Matrix A, Matrix B) {
		super("Indices differ, A is " + A.getNumRows() + "x" + A.getNumCols() + ", B is " + B.getNumRows() + "x" + B.getNumCols());
	}
	
	public MatrixIndexException(Matrix A) {
		super("Matrix is not SQUARE, A is " + A.getNumRows() + "x" + A.getNumCols());
	}

}
