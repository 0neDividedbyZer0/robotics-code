package sim;
public class PIDController {
	
	  public static final double kDefaultPeriod = .05;
	  private static int instances = 0;
	  @SuppressWarnings("MemberName")
	  private double m_P; // factor for "proportional" control
	  @SuppressWarnings("MemberName")
	  private double m_I; // factor for "integral" control
	  @SuppressWarnings("MemberName")
	  private double m_D; // factor for "derivative" control
	  @SuppressWarnings("MemberName")
	  private double m_F; // factor for feedforward term
	  private double m_maximumOutput = 1.0; // |maximum output|
	  private double m_minimumOutput = -1.0; // |minimum output|
	  private double m_maximumInput = 0.0; // maximum input - limit setpoint to this
	  private double m_minimumInput = 0.0; // minimum input - limit setpoint to this
	  private double m_inputRange = 0.0; // input range - difference between maximum and minimum
	  // do the endpoints wrap around? eg. Absolute encoder
	  private boolean m_continuous = false;
	  private boolean m_enabled = false; // is the pid controller enabled
	  // the prior error (used to compute velocity)
	  private double m_prevError = 0.0;
	  // the sum of the errors for use in the integral calc
	  private double m_totalError = 0.0;
	  // the tolerance object used to check if on target
	  private Tolerance m_tolerance;
	  private double m_setpoint = 0.0;
	  private double m_prevSetpoint = 0.0;
	  @SuppressWarnings("PMD.UnusedPrivateField")
	  private double m_error = 0.0;
	  private double m_result = 0.0;
	  private double m_period = kDefaultPeriod;
	  
	  public PIDController(double Kp, double Ki, double Kd, double Kf, double period) {
		
		m_P = Kp;
		m_I = Ki;
		m_D = Kd;
		m_F = Kf;
	
		m_period = period;
		
	}	
	
	 public double calculate(boolean enabled, boolean usingRate, double input) {
    if (enabled) {
      // Storage for function inputs
      double P;
      double I;
      double D;
      double feedForward = calculateFeedForward(usingRate, input, m_period);
      double minimumOutput;
      double maximumOutput;

      // Storage for function input-outputs
      double prevError;
      double error;
      double totalError;

        P = m_P;
        I = m_I;
        D = m_D;
        minimumOutput = m_minimumOutput;
        maximumOutput = m_maximumOutput;

        prevError = m_prevError;
        error = getContinuousError(m_setpoint - input);
        totalError = m_totalError;

      // Storage for function outputs
      double result;

      if (usingRate) {
        if (P != 0) {
          totalError = clamp(totalError + error, minimumOutput / P,
              maximumOutput / P);
        }

        result = P * totalError + D * error + feedForward;
      } else {
        if (I != 0) {
          totalError = clamp(totalError + error, minimumOutput / I,
              maximumOutput / I);
        }

        result = P * error + I * totalError + D * (error - prevError)
            + feedForward;
      }

      result = clamp(result, minimumOutput, maximumOutput);

      // Ensures m_enabled check and pidWrite() call occur atomically
      
        m_prevError = error;
        m_error = error;
        m_totalError = totalError;
        m_result = result;
        
        return result;
      }
       return 0.0;
    }
	 
	 public void setPID(double p, double i, double d, double f) {
	      m_P = p;
	      m_I = i;
	      m_D = d;
	      m_F = f;
	   
	 }
	 
	 public void setContinuous(boolean continuous) {
	    if (continuous && m_inputRange <= 0) {
	      throw new RuntimeException("No input range set when calling setContinuous().");
	    }
	   
	      m_continuous = continuous;
	   
	  }
	 
	 public void setInputRange(double minimumInput, double maximumInput) {
	    
      m_minimumInput = minimumInput;
      m_maximumInput = maximumInput;
      m_inputRange = maximumInput - minimumInput;
    

	    setSetpoint(m_setpoint);
	 }
	 
	 public void setSetpoint(double setpoint) {
    
      if (m_maximumInput > m_minimumInput) {
        if (setpoint > m_maximumInput) {
          m_setpoint = m_maximumInput;
        } else if (setpoint < m_minimumInput) {
          m_setpoint = m_minimumInput;
        } else {
          m_setpoint = setpoint;
        }
      } else {
        m_setpoint = setpoint;
      }
    
  }
	 
	 public void setOutputRange(double minimumOutput, double maximumOutput) {
	      m_minimumOutput = minimumOutput;
	      m_maximumOutput = maximumOutput;
	  }
	 
	 public interface Tolerance {
		    boolean onTarget(double input);
	  }

	  /**
	   * Used internally for when Tolerance hasn't been set.
	   */
	  public class NullTolerance implements Tolerance {
	    @Override
	    public boolean onTarget(double input) {
	      throw new RuntimeException("No tolerance value set when calling onTarget().");
	    }
	  }

	  public class PercentageTolerance implements Tolerance {
	    private final double m_percentage;

	    PercentageTolerance(double value) {
	      m_percentage = value;
	    }

	    @Override
	    public boolean onTarget(double input) {
	      return Math.abs(getError(input)) < m_percentage / 100 * m_inputRange;
	    }
	  }

	  public class AbsoluteTolerance implements Tolerance {
	    private final double m_value;

	    AbsoluteTolerance(double value) {
	      m_value = value;
	    }
	    
	  
	    
	    @Override
	    public boolean onTarget(double input) {
	      return Math.abs(getError(input)) < m_value;
	    }
	    
	  }
	  
	  public double getError(double input) {
          return getContinuousError(getSetpoint() - input);
      }
	  
	  protected double getContinuousError(double error) {
	    if (m_continuous && m_inputRange > 0) {
	      error %= m_inputRange;
	      if (Math.abs(error) > m_inputRange / 2) {
	        if (error > 0) {
	          return error - m_inputRange;
	        } else {
	          return error + m_inputRange;
	        }
	      }
	    }

	    return error;
	  }

	  
	  public double getSetpoint() {
	    	return m_setpoint;
	    }
	  
	  protected double calculateFeedForward(boolean usingRate, double input, double dt) {
	    if (usingRate) {
	      return m_F * getSetpoint();
	    } else {
	      double temp = m_F * getDeltaSetpoint(dt);
	      m_prevSetpoint = m_setpoint;
	      return temp;
	    }
	  }
	  private static double clamp(double value, double low, double high) {
	    return Math.max(low, Math.min(value, high));
	  }
	  
	  public double getDeltaSetpoint(double dt) {
	      return (m_setpoint - m_prevSetpoint) / dt;
	  }
	  
	  public void enable() {
	      m_enabled = true;
	  }
	  
	  public void disable() {
	      m_enabled = false;
	  }
	  
	  public boolean isEnabled() {
		  return m_enabled;
	  }
	  
	  public void reset() {
		  disable();

   
      m_prevError = 0;
      m_totalError = 0;
      m_result = 0;
    
  }
}
