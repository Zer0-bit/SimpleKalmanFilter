/*
 * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 * Created by Denys Sene, January, 1, 2017.
 * Released under MIT License - see LICENSE file for details.
 */

#include "Arduino.h"
#include "SimpleKalmanFilter.h"
#include <math.h>

SimpleKalmanFilter::SimpleKalmanFilter(float mea_e, float est_e, float q, bool adapt_q)
{
  _err_measure = mea_e;
  _err_estimate = est_e;
  _q = q;
  _adaptive_q = adapt_q;
}

float SimpleKalmanFilter::updateEstimate(float mea)
{
  float mea_diff = fabsf(mea - _last_estimate);

  static float smoothed_diff = 0.0f;
  smoothed_diff = smoothed_diff * 0.9f + mea_diff * 0.1f;
  const float noise_threshold = 0.01f;
  
  if (_adaptive_q)
  {
      float target_q = 0.1f + 0.2f * (smoothed_diff / (smoothed_diff + 0.1f));
      _q += (target_q - _q) * 0.1f;
  }

  _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate = (1.0f - _kalman_gain) * _err_estimate + fabsf(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;

  return _current_estimate;
}

void SimpleKalmanFilter::setMeasurementError(float mea_e)
{
  _err_measure = mea_e;
}

void SimpleKalmanFilter::setEstimateError(float est_e)
{
  _err_estimate = est_e;
}

void SimpleKalmanFilter::setProcessNoise(float q)
{
  _q = q;
}

float SimpleKalmanFilter::getKalmanGain()
{
  return _kalman_gain;
}

float SimpleKalmanFilter::getEstimateError()
{
  return _err_estimate;
}
