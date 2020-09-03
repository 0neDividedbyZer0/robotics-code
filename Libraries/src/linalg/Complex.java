package linalg;

public class Complex {
	private double a,b;
	
	public Complex(double a, double b) {
		this.a = a;
		this.b = b;
	}
	
	public String toString() {
		return a + " + " + b + "i";
	}
	
	public double Re() {
		return a;
	}
	
	public double Im() {
		return b;
	}

	public double Arg() {
		return Math.atan2(b,a);
	}
	
	public double Mod() {
		return Math.sqrt(a*a + b*b);
	}
	
	public Complex times(Complex z) {
		return new Complex(a * z.Re() - b * z.Im(), a * z.Im() + b * z.Re());
	}
	
	public Complex reciprocal() {
		return new Complex(a/(a*a+b*b),-b/(a*a+b*b));
	}
	
	public Complex divide(Complex z) {
		return times(z.reciprocal());
	}
	
	public Complex plus(Complex z) {
		return new Complex(a + z.Re(), b + z.Im());
	}
	
	public Complex minus(Complex z) {
		return new Complex(a - z.Re(), b - z.Im());
	}
	
	public Complex exp(double n) {
		return new Complex(Math.exp(Math.log(n)*a)*Math.cos(Math.log(n)*b),Math.exp(Math.log(n)*a)*Math.sin(Math.log(n)*b));
	}
	
	public static Complex pow(Complex z, Complex w) {
		Complex exp = Functions.Log(z);
		exp = exp.times(w);
		return new Complex(Math.exp(exp.Re())*Math.cos(exp.Im()),Math.exp(exp.Re())*Math.sin(exp.Im()));
	}

}
