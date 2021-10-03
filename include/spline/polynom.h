/**
 * \file polynom.h
 * \brief Definition of a cubic spline.
 * \author Steve T.
 * \version 0.1
 * \date 06/17/2013
 *
 * This file contains definitions for the polynom struct.
 * It allows the creation and evaluation of natural
 * smooth splines of arbitrary dimension and order
 */

#ifndef _STRUCT_POLYNOM
#define _STRUCT_POLYNOM

#include "MathDefs.h"

#include "curve_abc.h"

#include <iostream>
#include <algorithm>
#include <functional>
#include <stdexcept>

namespace spline {
/// \class polynom
/// \brief Represents a polynomf arbitrary order defined on the interval
/// [tBegin, tEnd]. It follows the equation
/// x(t) = a + b(t - t_min_) + ... + d(t - t_min_)^N, where N is the order
///
template <typename Time = double, typename Numeric = Time, std::size_t Dim = 3, bool Safe = false,
          typename Point = Eigen::Matrix<Numeric, Dim, 1>,
          typename T_Point = std::vector<Point, Eigen::aligned_allocator<Point> > >
struct polynom : public curve_abc<Time, Numeric, Dim, Safe, Point> {
  typedef Point point_t;
  typedef T_Point t_point_t;
  typedef Time time_t;
  typedef Numeric num_t;
  typedef curve_abc<Time, Numeric, Dim, Safe, Point> curve_abc_t;
  typedef Eigen::Matrix<double, Dim, Eigen::Dynamic> coeff_t;
  typedef Eigen::Ref<coeff_t> coeff_t_ref;

  /* Constructors - destructors */
 public:
  ///\brief Constructor
  ///\param coefficients : a reference to an Eigen matrix where each column is a coefficient,
  /// from the zero order coefficient, up to the highest order. Spline order is given
  /// by the number of the columns -1.
  ///\param min: LOWER bound on interval definition of the spline
  ///\param max: UPPER bound on interval definition of the spline
  polynom(const coeff_t& coefficients, const time_t min, const time_t max)
      : curve_abc_t(),
        coefficients_(coefficients),
        dim_(Dim),
        order_(coefficients_.cols() - 1),
        t_min_(min),
        t_max_(max) {
    safe_check();
  }

  ///\brief Constructor
  ///\param coefficients : a container containing all coefficients of the spline, starting
  /// with the zero order coefficient, up to the highest order. Spline order is given
  /// by the size of the coefficients
  ///\param min: LOWER bound on interval definition of the spline
  ///\param max: UPPER bound on interval definition of the spline
  polynom(const T_Point& coefficients, const time_t min, const time_t max)
      : curve_abc_t(),
        coefficients_(init_coeffs(coefficients.begin(), coefficients.end())),
        dim_(Dim),
        order_(coefficients_.cols() - 1),
        t_min_(min),
        t_max_(max) {
    safe_check();
  }

  ///\brief Constructor
  ///\param zeroOrderCoefficient : an iterator pointing to the first element of a structure containing the coefficients
  /// it corresponds to the zero degree coefficient
  ///\param out   : an iterator pointing to the last element of a structure ofcoefficients
  ///\param min: LOWER bound on interval definition of the spline
  ///\param max: UPPER bound on interval definition of the spline
  template <typename In>
  polynom(In zeroOrderCoefficient, In out, const time_t min, const time_t max)
      : coefficients_(init_coeffs(zeroOrderCoefficient, out)),
        dim_(Dim),
        order_(coefficients_.cols() - 1),
        t_min_(min),
        t_max_(max) {
    safe_check();
  }

  ///\brief Destructor
  ~polynom() {
    // NOTHING
  }

  polynom(const polynom& other)
      : coefficients_(other.coefficients_),
        dim_(other.dim_),
        order_(other.order_),
        t_min_(other.t_min_),
        t_max_(other.t_max_) {}

  // polynom& operator=(const polynom& other);

 private:
  void safe_check() {
    if (Safe) {
      if (t_min_ > t_max_) std::out_of_range("TODO");
      if (coefficients_.size() != int(order_ + 1)) std::runtime_error("Spline order and coefficients do not match");
    }
  }

  /* Constructors - destructors */

  /*Operations*/
 public:
  /*///  \brief Evaluation of the cubic spline at time t.
  ///  \param t : the time when to evaluate the spine
  ///  \param return : the value x(t)
  virtual point_t operator()(const time_t t) const
  {
      if((t < t_min_ || t > t_max_) && Safe){ throw std::out_of_range("TODO");}
      time_t const dt (t-t_min_);
      time_t cdt(1);
      point_t currentPoint_ = point_t::Zero();
      for(int i = 0; i < order_+1; ++i, cdt*=dt)
          currentPoint_ += cdt *coefficients_.col(i);
      return currentPoint_;
  }*/

  ///  \brief Evaluation of the cubic spline at time t using horner's scheme.
  ///  \param t : the time when to evaluate the spine
  ///  \param return : the value x(t)
  virtual point_t operator()(const time_t t) const {
    if ((t < t_min_ || t > t_max_) && Safe) {
      throw std::out_of_range("TODO");
    }
    time_t const dt(t - t_min_);
    point_t h = coefficients_.col(order_);
    for (int i = (int)(order_ - 1); i >= 0; i--) h = dt * h + coefficients_.col(i);
    return h;
  }

  ///  \brief Evaluation of the derivative spline at time t.
  ///  \param t : the time when to evaluate the spline
  ///  \param order : order of the derivative
  ///  \param return : the value x(t)
  virtual point_t derivate(const time_t t, const std::size_t order) const {
    if ((t < t_min_ || t > t_max_) && Safe) {
      throw std::out_of_range("TODO");
    }
    time_t const dt(t - t_min_);
    time_t cdt(1);
    point_t currentPoint_ = point_t::Zero();
    for (int i = (int)(order); i < (int)(order_ + 1); ++i, cdt *= dt)
      currentPoint_ += cdt * coefficients_.col(i) * fact(i, order);
    return currentPoint_;
  }

 private:
  num_t fact(const std::size_t n, const std::size_t order) const {
    num_t res(1);
    for (std::size_t i = 0; i < order; ++i) res *= (num_t)(n - i);
    return res;
  }

  /*Operations*/

  /*Helpers*/
 public:
  ///  \brief Returns the minimum time for wich curve is defined
  num_t virtual min() const { return t_min_; }
  ///  \brief Returns the maximum time for wich curve is defined
  num_t virtual max() const { return t_max_; }
  /*Helpers*/

  /*Attributes*/
 public:
  coeff_t coefficients_;  // const
  std::size_t dim_;       // const
  std::size_t order_;     // const

 private:
  time_t t_min_, t_max_;
  /*Attributes*/

 private:
  template <typename In>
  coeff_t init_coeffs(In zeroOrderCoefficient, In highestOrderCoefficient) {
    std::size_t size = std::distance(zeroOrderCoefficient, highestOrderCoefficient);
    coeff_t res = coeff_t(Dim, size);
    int i = 0;
    for (In cit = zeroOrderCoefficient; cit != highestOrderCoefficient; ++cit, ++i) res.col(i) = *cit;
    return res;
  }
};  // class polynom
}  // namespace spline
#endif  //_STRUCT_POLYNOM
