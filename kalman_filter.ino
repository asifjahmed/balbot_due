#include "kalman_filter.h"

void kalman_filter_init (struct kalman_filter_data *p_kd) {
    p_kd->P[0][0] = 1.0;
    p_kd->P[0][1] = 0.0;
    p_kd->P[1][0] = 0.0;
    p_kd->P[1][1] = 1.0;
    p_kd->angle = 0;
    p_kd->q_bias = 0;
    p_kd->rate = 0;
    p_kd->Pdot[0] = 0;
    p_kd->Pdot[1] = 0;
    p_kd->Pdot[2] = 0;
    p_kd->Pdot[3] = 0;
}


/*
 * state_update is called every dt with a biased gyro measurement
 * by the user of the module.  It updates the current angle and
 * rate estimate.
 *
 * The pitch gyro measurement should be scaled into real units, but
 * does not need any bias removal.  The filter will track the bias.
 *
 * Our state vector is:
 *
 *	X = [ angle, gyro_bias ]
 *
 * It runs the state estimation forward via the state functions:
 *
 *	Xdot = [ angle_dot, gyro_bias_dot ]
 *
 *	angle_dot	= gyro - gyro_bias
 *	gyro_bias_dot	= 0
 *
 * And updates the covariance matrix via the function:
 *
 *	Pdot = A*P + P*A' + Q
 *
 * A is the Jacobian of Xdot with respect to the states:
 *
 *	A = [ d(angle_dot)/d(angle)     d(angle_dot)/d(gyro_bias) ]
 *	    [ d(gyro_bias_dot)/d(angle) d(gyro_bias_dot)/d(gyro_bias) ]
 *
 *	  = [ 0 -1 ]
 *	    [ 0  0 ]
 *
 * Due to the small CPU available on the microcontroller, we've
 * hand optimized the C code to only compute the terms that are
 * explicitly non-zero, as well as expanded out the matrix math
 * to be done in as few steps as possible.  This does make it harder
 * to read, debug and extend, but also allows us to do this with
 * very little CPU time.
 */
void kalman_filter_state_update (float q_m, struct kalman_filter_data  *p_kd, float dt) {
  	float		q;
    q = q_m - p_kd->q_bias;
  
  	p_kd->Pdot[0] = Q_ACCEL - p_kd->P[0][1] - p_kd->P[1][0];	
  	p_kd->Pdot[1] = 0 - p_kd->P[1][1];
  	p_kd->Pdot[2] = 0 - p_kd->P[1][1];
  	p_kd->Pdot[3] = Q_GYRO;
  
  	p_kd->rate = q;
  
  	p_kd->angle += (q * dt);
  
  	p_kd->P[0][0] += (p_kd->Pdot[0] * dt);
  	p_kd->P[0][1] += (p_kd->Pdot[1] * dt);
  	p_kd->P[1][0] += (p_kd->Pdot[2] * dt);
  	p_kd->P[1][1] += (p_kd->Pdot[3] * dt);
}


/*
 * kalman_update is called by a user of the module when a new
 * accelerometer measurement is available.  axy_m and az_m do not
 * need to be scaled into actual units, but must be zeroed and have
 * the same scale.
 *
 * This does not need to be called every time step, but can be if
 * the accelerometer data are available at the same rate as the
 * rate gyro measurement.
 *
 * For a two-axis accelerometer mounted perpendicular to the rotation
 * axis, we can compute the angle for the full 360 degree rotation
 * with no linearization errors by using the arctangent of the two
 * readings.
 *
 * As commented in state_update, the math here is simplified to
 * make it possible to execute on a small microcontroller with no
 * floating point unit.  It will be hard to read the actual code and
 * see what is happening, which is why there is this extensive
 * comment block.
 *
 * The C matrix is a 1x2 (measurements x states) matrix that
 * is the Jacobian matrix of the measurement value with respect
 * to the states.  In this case, C is:
 *
 *	C = [ d(angle_m)/d(angle)  d(angle_m)/d(gyro_bias) ]
 *	  = [ 1 0 ]
 *
 * because the angle measurement directly corresponds to the angle
 * estimate and the angle measurement has no relation to the gyro
 * bias.
 * CHANGED SUCH THAT ANGLE IS CALCULATED EXTERNALLY
 */
void kalman_filter_angle_update (float angle_m, struct kalman_filter_data  *p_kd) {
    float		angle_err;
    float		C_0;
    float		PCt_0;
    float		PCt_1;
    float		E;
    float		K_0, K_1;
    float		t_0;
    float		t_1;
  	
  	angle_err = angle_m - p_kd->angle;
  
    p_kd->err = angle_err;
  
  	C_0 = 1;
  
  	PCt_0 = C_0 * p_kd->P[0][0];
  	PCt_1 = C_0 * p_kd->P[1][0];
  		
  	E =	R_ANGLE	+ C_0 * PCt_0;
  	
  	K_0 = PCt_0 / E;
  	K_1 = PCt_1 / E;
  		
  	t_0 = PCt_0;
  	t_1 = C_0 * p_kd->P[0][1];
  
  	p_kd->P[0][0] -= (K_0 * t_0);
  	p_kd->P[0][1] -= (K_0 * t_1);
  	p_kd->P[1][0] -= (K_1 * t_0);
  	p_kd->P[1][1] -= (K_1 * t_1);
  	
  	p_kd->angle	+= (K_0 * angle_err);
  	p_kd->q_bias	+= (K_1 * angle_err);
}
