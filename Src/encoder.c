#include "encoder.h"

/* ----------------------------------------------------------
 * Configuration
 * ---------------------------------------------------------- */

#define ENCODER_CPR                 400.0f

#define SAMPLE_PERIOD_S             0.01f
#define SAMPLE_PERIOD_TICK_S        0.000001f

#define STOP_TIMEOUT_US             60000U

#define SLOW_EST_JITTER_REDUCTION   1U

#define FAST_GAIN \
    (1.0f / (ENCODER_CPR * SAMPLE_PERIOD_S))

#define ALPHA_FAST_LOW_PASS 0.9f


/* ----------------------------------------------------------
 * Private variables
 * ---------------------------------------------------------- */

static TIM_HandleTypeDef *encoder_timer = NULL;
static TIM_HandleTypeDef *timestamp_timer = NULL;
static TIM_HandleTypeDef *estimator_timer = NULL;

static volatile int32_t encoder_counter = 0;
static volatile int32_t prev_encoder_cnt = 0;

static volatile int32_t counts_last_sample = 0;

static volatile uint16_t slow_dt = 0;
static volatile int32_t slow_delta = 0;
static volatile uint16_t last_change_time = 0;

static volatile int8_t slow_flag = 0;
static volatile int8_t slow_est_active = 0;

static volatile float vel_fast_rps = 0.0f;
static volatile float vel_fast_rps_old = 0.0f;
static volatile float vel_slow_rps = 0.0f;
static volatile float vel_rps = 0.0f;
//static volatile float vel_rps_old = 0.0f;

static float vel_smooth_lower_bnd = 0.0f;
static float vel_smooth_upper_bnd = 0.0f;
static uint8_t low_pass = 0;

static uint8_t capture_event_count = 0;


/* ----------------------------------------------------------
 * Private functions
 * ---------------------------------------------------------- */

static float Clamp(float x, float lo, float hi) {
	if(x < lo) return lo;
	if(x > hi) return hi;
	return x;
}


/**
@brief Detects when the encoder has stopped moving.

The timestamp of the last encoder transition is compared with
the current timestamp. If the elapsed time exceeds
STOP_TIMEOUT_US, the final velocity estimate is set to zero.
This function is called when the calculated velocity is close
to zero.
*/
void VelocityBackgroundTask(void) {
	uint16_t now = __HAL_TIM_GET_COUNTER(timestamp_timer);


	if((uint16_t)(now - last_change_time) > STOP_TIMEOUT_US)
	{
		vel_rps = 0.0f;
	}


}



/* ----------------------------------------------------------
 * Public functions
 * ---------------------------------------------------------- */


/**
@brief Calculates the final encoder velocity estimate.

The fast and slow velocity estimates are blended according to
the current magnitude of the fast velocity.
At low speeds, the slow estimator receives more weight because
measuring the time between encoder transitions provides better
resolution.
At high speeds, the fast estimator receives more weight because
encoder counts are changing frequently enough for the fixed-period
estimator to provide a stable result.
*/
void UpdateVelocityEstimate(void) {
	float speed_mag = (vel_fast_rps >= 0) ? vel_fast_rps : -vel_fast_rps;

	float alpha = (speed_mag - vel_smooth_lower_bnd) / (vel_smooth_upper_bnd - vel_smooth_lower_bnd);
	alpha = Clamp(alpha, 0.0f, 1.0f);
	alpha = alpha * alpha * (3.0f - 2.0f * alpha);
	vel_rps = (1.0f - alpha) * vel_slow_rps + alpha * vel_fast_rps;
	if(vel_rps < vel_smooth_lower_bnd) {
		if(vel_rps > -vel_smooth_lower_bnd) {
			VelocityBackgroundTask();
		}
	}

}



/**
@brief Initializes the encoder measurement module.

Stores the timer handles, resets all encoder/velocity state,
and stores the velocity blending and filtering configuration.
*/
void Encoder_Init(TIM_HandleTypeDef *encoder_timer_,
                  TIM_HandleTypeDef *timestamp_timer_,
                  TIM_HandleTypeDef *estimator_timer_,
				  float vel_smooth_lower_bnd_,
				  float vel_smooth_upper_bnd_,
				  uint8_t low_pass_) {

    encoder_timer = encoder_timer_;
    timestamp_timer = timestamp_timer_;
    estimator_timer = estimator_timer_;

    encoder_counter = 0;
    prev_encoder_cnt = 0;
    counts_last_sample = 0;

    slow_dt = 0;
    slow_delta = 0;
    last_change_time = 0;

    slow_flag = 0;
    slow_est_active = 0;
    capture_event_count = 0;

    vel_fast_rps = 0.0f;
    vel_fast_rps_old = 0.0f;
    vel_slow_rps = 0.0f;
    vel_rps = 0.0f;

    vel_smooth_lower_bnd = vel_smooth_lower_bnd_;
    vel_smooth_upper_bnd = vel_smooth_upper_bnd_;
    low_pass = low_pass_;
}


/**
@brief Processes an encoder capture event.

When the slow estimator is enabled, this function measures the
number of encoder counts and the elapsed time since the previous
encoder transition. These values are then used to calculate the
slow velocity estimate.
The function should be called from the appropriate STM32 timer
callback.
@param htim Pointer to the timer handle that generated the callback.
*/
void Encoder_CaptureCallback(TIM_HandleTypeDef *htim) {

	if (htim != encoder_timer)
	        return;

	if (slow_flag == 1) {
		capture_event_count++;
		if(capture_event_count >= SLOW_EST_JITTER_REDUCTION) {
			static int32_t prev_cnt = 0;

			if (slow_est_active == 0) {
				slow_est_active = 1;
				prev_cnt = __HAL_TIM_GET_COUNTER(encoder_timer);
			}

			int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(encoder_timer);
			int32_t delta = (int32_t)((uint32_t)cnt - (uint32_t)prev_cnt);
			int32_t counts = (delta >= 0) ? delta : -delta;

			prev_cnt = cnt;

			uint16_t now = __HAL_TIM_GET_COUNTER(timestamp_timer);
			slow_dt = (uint16_t)(now - last_change_time);
			if(slow_dt > 0) {
				vel_slow_rps = (float)counts / (ENCODER_CPR * (slow_dt * SAMPLE_PERIOD_TICK_S));
				vel_slow_rps = (delta >= 0) ? vel_slow_rps : -vel_slow_rps;
			}
			else {
				vel_slow_rps = 0.0f;
			}
			last_change_time = now;
			capture_event_count = 0;
		}


	}
	encoder_counter = (int32_t)__HAL_TIM_GET_COUNTER(encoder_timer);
}


/**
@brief Performs the periodic fast velocity estimation.

The encoder count difference over the fixed estimator period is
converted directly into revolutions per second.
This estimator is particularly useful at higher encoder speeds,
where enough encoder counts occur during each sample period.
@param htim Pointer to the timer handle that generated the callback.
*/
void Encoder_PeriodicUpdate(TIM_HandleTypeDef *htim) {
	if (htim != estimator_timer)
	        return;
	// fast estimator:
	int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(encoder_timer);
	int32_t fast_delta = (int32_t)((uint32_t)cnt - (uint32_t)prev_encoder_cnt);  // prevent rollover err.
	prev_encoder_cnt = cnt;
	counts_last_sample = (fast_delta >= 0) ? fast_delta : -fast_delta;
	vel_fast_rps = ((float)fast_delta) * FAST_GAIN ;  // Make precalculation to save time in ISR
	// turn on slow estimator if vel_fast_rps is below threshold, turn off if it is above threshold
	float speed_mag = (vel_fast_rps >= 0) ? vel_fast_rps : -vel_fast_rps;
	if (speed_mag > vel_smooth_upper_bnd) {
		slow_flag = 0;
		slow_est_active = 0;
	}
	else {
		slow_flag = 1;
	}
	if (low_pass == 1) {
		vel_fast_rps = ALPHA_FAST_LOW_PASS * vel_fast_rps + (1 - ALPHA_FAST_LOW_PASS) * vel_fast_rps_old;
		vel_fast_rps_old = vel_fast_rps;
	}

}



float Encoder_GetVelocity(void)
{
    return vel_rps;
}


float Encoder_GetFastVelocity(void)
{
    return vel_fast_rps;
}


float Encoder_GetSlowVelocity(void)
{
    return vel_slow_rps;
}


int32_t Encoder_GetCount(void)
{
    return encoder_counter;
}
