#pragma once
#include "srrg_geometry/geometry_defs.h"

namespace srrg2_core {
  struct ColorPalette {
    ColorPalette() {
    }
    ~ColorPalette() {
    }

    // ia basic colors
    static Vector3f color3fRed() {
      return Vector3f(0.90, 0.10, 0.10);
    }
    static Vector3f color3fOrange() {
      return Vector3f(0.96, 0.67, 0.20);
    }
    static Vector3f color3fYellow() {
      return Vector3f(0.90, 0.90, 0.10);
    }
    static Vector3f color3fGreen() {
      return Vector3f(0.10, 0.90, 0.10);
    }
    static Vector3f color3fCyan() {
      return Vector3f(0.52, 0.79, 0.91);
    }
    static Vector3f color3fMagenta() {
      return Vector3f(1.00, 0.25, 0.45);
    }
    static Vector3f color3fViolet() {
      return Vector3f(0.73, 0.26, 1.00);
    }
    static Vector3f color3fBlue() {
      return Vector3f(0.10, 0.10, 0.90);
    }
    static Vector3f color3fBlack() {
      return Vector3f(0.10, 0.10, 0.10);
    }
    static Vector3f color3fWhite() {
      return Vector3f(0.95, 0.95, 0.95);
    }

    static Vector4f color4fRed(const float& alpha_ = 1.0f) {
      return Vector4f(0.90, 0.10, 0.10, alpha_);
    }
    static Vector4f color4fOrange(const float& alpha_ = 1.0f) {
      return Vector4f(0.96, 0.67, 0.20, alpha_);
    }
    static Vector4f color4fYellow(const float& alpha_ = 1.0f) {
      return Vector4f(0.90, 0.90, 0.10, alpha_);
    }
    static Vector4f color4fGreen(const float& alpha_ = 1.0f) {
      return Vector4f(0.10, 0.90, 0.10, alpha_);
    }
    static Vector4f color4fCyan(const float& alpha_ = 1.0f) {
      return Vector4f(0.52, 0.79, 0.91, alpha_);
    }
    static Vector4f color4fMagenta(const float& alpha_ = 1.0f) {
      return Vector4f(1.00, 0.25, 0.45, alpha_);
    }
    static Vector4f color4fViolet(const float& alpha_ = 1.0f) {
      return Vector4f(0.73, 0.26, 1.00, alpha_);
    }
    static Vector4f color4fBlue(const float& alpha_ = 1.0f) {
      return Vector4f(0.10, 0.10, 0.90, alpha_);
    }
    static Vector4f color4fBlack(const float& alpha_ = 1.0f) {
      return Vector4f(0.10, 0.10, 0.10, alpha_);
    }
    static Vector4f color4fWhite(const float& alpha_ = 1.0f) {
      return Vector4f(0.95, 0.95, 0.95, alpha_);
    }

    // //ia dark variants
    static Vector3f color3fDarkRed() {
      return Vector3f(0.40, 0.10, 0.10);
    }
    static Vector3f color3fDarkYellow() {
      return Vector3f(0.46, 0.37, 0.10);
    }
    static Vector3f color3fDarkGreen() {
      return Vector3f(0.10, 0.40, 0.10);
    }
    static Vector3f color3fDarkCyan() {
      return Vector3f(0.22, 0.39, 0.41);
    }
    static Vector3f color3fDarkMagenta() {
      return Vector3f(0.50, 0.15, 0.23);
    }
    static Vector3f color3fDarkViolet() {
      return Vector3f(0.33, 0.16, 0.50);
    }
    static Vector3f color3fDarkBlue() {
      return Vector3f(0.05, 0.05, 0.30);
    }

    static Vector4f color4fDarkRed(const float& alpha_ = 1.0f) {
      return Vector4f(0.40, 0.10, 0.10, alpha_);
    }
    static Vector4f color4fDarkYellow(const float& alpha_ = 1.0f) {
      return Vector4f(0.46, 0.37, 0.10, alpha_);
    }
    static Vector4f color4fDarkGreen(const float& alpha_ = 1.0f) {
      return Vector4f(0.10, 0.40, 0.10, alpha_);
    }
    static Vector4f color4fDarkCyan(const float& alpha_ = 1.0f) {
      return Vector4f(0.22, 0.39, 0.41, alpha_);
    }
    static Vector4f color4fDarkMagenta(const float& alpha_ = 1.0f) {
      return Vector4f(0.50, 0.15, 0.23, alpha_);
    }
    static Vector4f color4fDarkViolet(const float& alpha_ = 1.0f) {
      return Vector4f(0.33, 0.16, 0.50, alpha_);
    }
    static Vector4f color4fDarkBlue(const float& alpha_ = 1.0f) {
      return Vector4f(0.05, 0.05, 0.30, alpha_);
    }

    // ia particular colors
    static Vector3f color3fMint() {
      return Vector3f(0.72, 0.86, 0.85);
    }
    static Vector4f color4fMint(const float& alpha_ = 1.0f) {
      return Vector4f(0.72, 0.86, 0.85, alpha_);
    }
    static Vector3f color3fLightGray() {
      return Vector3f(0.72, 0.76, 0.75);
    }
    static Vector4f color4fLightGray(const float& alpha_ = 1.0f) {
      return Vector4f(0.72, 0.76, 0.75, alpha_);
    }
    static Vector3f color3fEerieBlack() {
      return Vector3f(0.06, 0.10, 0.16);
    }
    static Vector4f color4fEerieBlack(const float& alpha_ = 1.0f) {
      return Vector4f(0.06, 0.10, 0.16, alpha_);
    }
    static Vector3f color3fPaleYellow() {
      return Vector3f(0.95, 1.00, 0.74);
    }
    static Vector4f color4fPaleYellow(const float& alpha_ = 1.0f) {
      return Vector4f(0.95, 1.00, 0.74, alpha_);
    }
    static Vector3f color3fGunmetal() {
      return Vector3f(0.18, 0.19, 0.22);
    }
    static Vector4f color4fGunmetal(const float& alpha_ = 1.0f) {
      return Vector4f(0.18, 0.19, 0.22, alpha_);
    }

    // ia interpolators to create gradients
    static Vector3f gradientGreenBlue(const size_t& current_sample_,
                                      const size_t& max_samples_) {
      return Vector3f(0.f,
                      2.f * float(current_sample_) / max_samples_,
                      2.f * (1.0 - float(current_sample_) / max_samples_));
    }

    static Vector3f gradientRedGreen(const size_t& current_sample_,
                                     const size_t& max_samples_) {
      return Vector3f(2.f * float(current_sample_) / max_samples_,
                      2.f * (1.0 - float(current_sample_) / max_samples_),
                      0.0f);
    }
  };
} // namespace srrg2_core
