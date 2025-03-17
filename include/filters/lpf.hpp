#pragma once
#include <Eigen/Dense>
#include <cmath>

// Define M_PI if not already defined.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Templated low‑pass filter for Eigen vectors.
// The filter is applied element‑wise using:
//    y[k] = α * x[k] + (1 – α) * y[k-1]
// where α = Ts / (RC + Ts) and RC = 1/(2π * cutoff_frequency)
template <typename Derived>
class LowPassFilter {
public:
  using VectorType = Derived;

  // Constructor: cutoff_frequency in Hz, Ts is the sampling period in seconds.
  LowPassFilter(double cutoff_frequency, double Ts)
    : cutoff_frequency_(cutoff_frequency), Ts_(Ts)
  {
    prev_output_ = VectorType::Zero();
    double RC = 1.0 / (2.0 * M_PI * cutoff_frequency_);
    alpha_ = Ts_ / (RC + Ts_);
  }

  // Process one sample vector.
  VectorType filter(const VectorType & input) {
    VectorType output = alpha_ * input + (1.0 - alpha_) * prev_output_;
    prev_output_ = output;
    return output;
  }

private:
  double cutoff_frequency_;
  double Ts_;
  double alpha_;
  VectorType prev_output_;
};
