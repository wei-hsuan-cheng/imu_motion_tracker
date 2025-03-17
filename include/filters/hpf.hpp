#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <complex>

// Define M_PI if not already defined.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Templated high‑pass filter for Eigen vectors.
// Derived should be an Eigen fixed‑size vector type (e.g., Eigen::Vector3d).
template <typename Derived>
class HighPassFilter {
public:
  using VectorType = Derived;

  // Constructor: cutoff_frequency in Hz, Ts is the sampling period in seconds.
  HighPassFilter(double cutoff_frequency, double Ts)
    : cutoff_frequency_(cutoff_frequency), Ts_(Ts)
  {
    // Initialize previous input and output to zero.
    prev_input_ = VectorType::Zero();
    prev_output_ = VectorType::Zero();

    // RC time constant and alpha calculation for a first‑order high‑pass filter:
    // RC = 1/(2π * cutoff_frequency)
    // α = RC / (RC + Ts)
    double RC = 1.0 / (2.0 * M_PI * cutoff_frequency_);
    alpha_ = RC / (RC + Ts_);
  }

  // Process one sample vector (element‑wise filtering):
  // y[k] = α * ( y[k-1] + x[k] - x[k-1] )
  VectorType filter(const VectorType & input) {
    VectorType output = alpha_ * (prev_output_ + input - prev_input_);
    prev_input_ = input;
    prev_output_ = output;
    return output;
  }

private:
  double cutoff_frequency_;
  double Ts_;
  double alpha_;
  VectorType prev_input_;
  VectorType prev_output_;
};
