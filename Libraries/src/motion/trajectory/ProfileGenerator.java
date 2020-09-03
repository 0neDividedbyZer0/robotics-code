package motion.trajectory;

public class ProfileGenerator {
	
	private ProfileGenerator() {
		
	}
	
	public static Trajectory secondOrderFilter(double max_vel, double dt, double start_vel,
			int Tf1, int Tf2, double I, int T) {
		Trajectory traj = new Trajectory();
		Segment last = new Segment();
		last.vel = start_vel;
		last.acc = 0;
		last.jerk = 0;
		last.dt = dt;
		last.pos = 0;
		
	
		double impulseSignal = I;
		double[] f1_buffer = new double[T];
		
		f1_buffer[0] = (start_vel / max_vel) * Tf1;
		for(int i = 0; i < T; i++) {
			double input = Math.min(impulseSignal, 1);
			if(input < 1) {
				input -= 1;
				impulseSignal = 0;
			} else {
				impulseSignal -= input;
			}
			
			double f1_last;
			if(i > 0) {
				f1_last = f1_buffer[i - 1];
			} else {
				f1_last = f1_buffer[0];
			}
			
			f1_buffer[i] = Math.max(0, Math.min(Tf1, f1_last + input));
			double f2 = 0.0;
			
			for(int j = 0; j < Tf2; j++) {
				if(i - j < 0) {
					break;
				}
				f2 += f1_buffer[i - j];
			}
			
			f2 = f2 / Tf1;
			
			traj.seg[i].vel = (f2 / Tf2) * max_vel;
			traj.seg[i].pos = ((last.vel + traj.seg[i].vel) / 2.0) * dt + last.pos;
			traj.seg[i].acc = (traj.seg[i].vel - last.vel) / dt;
			traj.seg[i].jerk = (traj.seg[i].acc - last.acc) / dt;
			traj.seg[i].dt = dt;
			last = traj.seg[i];
		}
		
		return traj;
	}

}
