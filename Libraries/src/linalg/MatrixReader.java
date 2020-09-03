package linalg;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.ojalgo.matrix.PrimitiveMatrix;

public class MatrixReader {
	
	public static PrimitiveMatrix parseMatrix(String pathtocsvfile) {
		List<String[]> rowList = new ArrayList<String[]>();
		try (BufferedReader br = new BufferedReader(new FileReader(pathtocsvfile))) {
		    String line;
		    while ((line = br.readLine()) != null) {
		        String[] lineItems = line.split(",");
		        rowList.add(lineItems);
		    }
		    br.close();
		}
		catch(IOException e){
			e.printStackTrace();
		}
		
		String[][] matrix = new String[rowList.size()][];
		for (int i = 0; i < rowList.size(); i++) {
		    String[] row = rowList.get(i);
		    matrix[i] = row;
		}
		double[][] constant = new double[matrix.length][matrix[0].length];
		for(int i = 0; i < constant.length; i++) {
			for(int j = 0; j < constant[0].length; j++) {
				constant[i][j] = Double.parseDouble(matrix[i][j]);
			}
		}
		return PrimitiveMatrix.FACTORY.rows(constant);
	}
}
