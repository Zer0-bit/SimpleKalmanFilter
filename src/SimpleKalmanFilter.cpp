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
  _adaptive_q = adapt_q; // flag to control whether adaptive q is enabled
}

float SimpleKalmanFilter::updateEstimate(float mea)
{
  // Calculate difference between current and last measurement
  float mea_diff = fabsf(mea - _last_estimate);

  // Dynamically adjust the process noise based on the measurement difference
  if (_adaptive_q)
  {
    // Example logic to adjust process noise based on the rate of change
    _q = fmin(1.0f, mea_diff * 0.05); // Adjust this scaling factor to control sensitivity
  }

  _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate = (1.0f - _kalman_gain) * _err_estimate + fabsf(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;

  return _current_estimate;
}

void SimpleKalmanFilter::enableAdaptiveProcessNoise(bool enable)
{
  _adaptive_q = enable;  // Enable or disable adaptive process noise
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
