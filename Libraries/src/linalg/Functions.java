package linalg;

public class Functions {
	
	public Functions() {
		
	}
	
	public static Complex exp(Complex z) {
		double x = z.Re();
		double y = z.Im();
		double r = Math.exp(x);
		x = r*Math.cos(y);
		y = r*Math.sin(y);
		return new Complex(x,y);
	}
	
	public static Complex cos(Complex z) {
		double x = z.Re();
		double y = z.Im();
		double re = Math.cos(x)*Math.cosh(y);
		double im = -Math.sin(x)*Math.sinh(y);
		return new Complex(re,im);
	}
	
	public static Complex sin(Complex z) {
		double x = z.Re();
		double y = z.Im();
		double re = Math.sin(x)*Math.cosh(y);
		double im = Math.cos(x)*Math.sinh(y);
		return new Complex(re, im);
	}
	
	public static Complex tan(Complex z) {
		return Functions.sin(z).divide(Functions.cos(z));
	}
	
	public static Complex cosh(Complex z) {
		double x = z.Re();
		double y = z.Im();
		double re = (Math.exp(x)*Math.cos(y) + Math.exp(-x)*Math.cos(y))/2.0;
		double im = (Math.exp(x)*Math.sin(y)-Math.exp(-x)*Math.sin(y))/2.0;
		return new Complex(re,im);
	}
	
	public static Complex sinh(Complex z) {
		double x = z.Re();
		double y = z.Im();
		double re = (Math.exp(x)*Math.cos(y)-Math.exp(-x)*Math.cos(y))/2.0;
		double im = (Math.exp(x)*Math.sin(y)+Math.exp(-x)*Math.sin(y))/2.0;
		return new Complex(re,im);
	}
	
	public static Complex tanh(Complex z) {
		return Functions.sinh(z).divide(Functions.cosh(z));
	}
	
	public static Complex sqrt(Complex z) {
		double r = Math.sqrt(z.Mod());
		Complex temp = Functions.exp(new Complex (0,z.Arg()/2));
		temp = temp.times(new Complex(r,0));
		return temp;
	}
	
	public static Complex Log(Complex z) {
		return new Complex(Math.log(z.Mod()),z.Arg());
	}
	
	public static Complex Acos(Complex z) {
		Complex temp = new Complex(z.Re()*z.Re()-z.Im()*z.Im()-1,2*z.Re()*z.Im());
		temp = Functions.sqrt(temp);
		temp = temp.plus(z);
		temp = Functions.Log(temp);
		temp = temp.times(new Complex(0,-1));
		return temp;
	}
	
	public static Complex Asin(Complex z) {
		Complex temp = new Complex(1-z.Re()*z.Re()+z.Im()*z.Im(),-2*z.Re()*z.Im());
		temp = Functions.sqrt(temp);
		temp = temp.plus(new Complex(-z.Im(),z.Re()));
		temp = Functions.Log(temp);
		temp = temp.times(new Complex(0,-1));
		return temp;
	}
	
	public static Complex Atan(Complex z) {
		Complex temp = new Complex(z.Im()-1,-z.Re());
		temp = temp.divide(new Complex(-z.Im()-1,z.Re()));
		temp = Functions.Log(temp);
		temp = temp.times(new Complex(0,-0.5));
		return temp;
		
	}
	
	public static Complex Acosh(Complex z) {
		Complex temp = new Complex(z.Re()*z.Re()-z.Im()*z.Im()-1,2*z.Re()*z.Im());
		temp = Functions.sqrt(temp);
		temp = temp.plus(z);
		temp = Functions.Log(temp);
		return temp;
	}
	
	public static Complex Asinh(Complex z) {
		Complex temp = new Complex(z.Re()*z.Re()-z.Im()*z.Im()+1,2*z.Re()*z.Im());
		temp = Functions.sqrt(temp);
		temp = temp.plus(z);
		temp = Functions.Log(temp);
		return temp;
	}
	
	public static Complex Atanh(Complex z) {
		Complex temp = new Complex(-z.Re()-1,-z.Im());
		temp = temp.divide(new Complex(z.Re()-1,z.Im()));
		temp = Functions.Log(temp);
		temp = temp.times(new Complex(0.5,0));
		return temp;
	}
	
	public static Complex LogL(Complex z) {
		Complex temp1 = new Complex(z.Re(),z.Im());
		Complex sum = new Complex(0,0);
		for(int i = 1; i < 50; i++) {
			Complex temp = Complex.pow(temp1, new Complex(i,0));
			temp = temp.times(new Complex(Math.pow(-1,i+1)/i,0));
			sum = sum.plus(temp);
		}
		for(int i = -1; i > -50;i--) {
			Complex temp = Complex.pow(temp1, new Complex(i,0));
			temp = temp.times(new Complex(Math.pow(-1,i+1)/i,0));
			sum = sum.plus(temp);
		}
		return sum;
	}
	
}
	

