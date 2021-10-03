/**
 * \file bezier_curve.h
 * \brief class allowing to create a Bezier curve of dimension 1 <= n <= 3.
 * \author Steve T.
 * \version 0.1
 * \date 06/17/2013
 */

#ifndef _CLASS_BEZIERCURVE
#define _CLASS_BEZIERCURVE

#include "curve_abc.h"
#include "bernstein.h"
#include "curve_constraint.h"

#include "MathDefs.h"

#include <vector>
#include <stdexcept>

#include <iostream>

namespace spline {
/// \class BezierCurve
/// \brief Represents a Bezier curve of arbitrary dimension and order.
/// For degree lesser than 4, the evaluation is analitycal.Otherwise
/// the bernstein polynoms are used to evaluate the spline at a given location.
///
template <typename Time = double, typename Numeric = Time, std::size_t Dim = 3, bool Safe = false,
          typename Point = Eigen::Matrix<Numeric, Dim, 1> >
struct bezier_curve : public curve_abc<Time, Numeric, Dim, Safe, Point> {
  typedef Point point_t;
  typedef Time time_t;
  typedef Numeric num_t;
  typedef curve_constraints<point_t> curve_constraints_t;
  typedef std::vector<point_t, Eigen::aligned_allocator<point_t> > t_point_t;
  typedef typename t_point_t::const_iterator cit_point_t;
  typedef bezier_curve<Time, Numeric, Dim, Safe, Point> bezier_curve_t;

  /* Constructors - destructors */
 public:
  ///\brief Constructor
  ///\param PointsBegin, PointsEnd : the points parametering the Bezier curve
  ///
  template <typename In>
  bezier_curve(In PointsBegin, In PointsEnd)
      : T_(1.),
        mult_T_(1.),
        size_(std::distance(PointsBegin, PointsEnd)),
        degree_(size_ - 1),
        bernstein_(spline::makeBernstein<num_t>((unsigned int)degree_)) {
    assert(bernstein_.size() == size_);
    In it(PointsBegin);
    if (Safe && (size_ < 1 || T_ <= 0.))
      throw std::out_of_range("can't create bezier min bound is higher than max bound");  // TODO
    for (; it != PointsEnd; ++it) pts_.push_back(*it);
  }

  ///\brief Constructor
  ///\param PointsBegin, PointsEnd : the points parametering the Bezier curve
  ///
  template <typename In>
  bezier_curve(In PointsBegin, In PointsEnd, const time_t T)
      : T_(T),
        mult_T_(1.),
        size_(std::distance(PointsBegin, PointsEnd)),
        degree_(size_ - 1),
        bernstein_(spline::makeBernstein<num_t>((unsigned int)degree_)) {
    assert(bernstein_.size() == size_);
    In it(PointsBegin);
    if (Safe && (size_ < 1 || T_ <= 0.))
      throw std::out_of_range("can't create bezier min bound is higher than max bound");  // TODO
    for (; it != PointsEnd; ++it) pts_.push_back(*it);
  }

  ///\brief Constructor
  ///\param PointsBegin, PointsEnd : the points parametering the Bezier curve
  ///
  template <typename In>
  bezier_curve(In PointsBegin, In PointsEnd, const time_t T, const time_t mult_T)
      : T_(T),
        mult_T_(mult_T),
        size_(std::distance(PointsBegin, PointsEnd)),
        degree_(size_ - 1),
        bernstein_(spline::makeBernstein<num_t>((unsigned int)degree_)) {
    assert(bernstein_.size() == size_);
    In it(PointsBegin);
    if (Safe && (size_ < 1 || T_ <= 0.))
      throw std::out_of_range("can't create bezier min bound is higher than max bound");  // TODO
    for (; it != PointsEnd; ++it) pts_.push_back(*it);
  }

  ///\brief Constructor
  /// This constructor will add 4 points (2 after the first one, 2 before the last one)
  /// to ensure that velocity and acceleration constraints are respected
  ///\param PointsBegin, PointsEnd : the points parametering the Bezier curve
  ///\param constraints : constraints applying on start / end velocities and acceleration
  ///
  template <typename In>
  bezier_curve(In PointsBegin, In PointsEnd, const curve_constraints_t& constraints, const time_t T = 1.)
      : T_(T),
        mult_T_(1.),
        size_(std::distance(PointsBegin, PointsEnd) + 4),
        degree_(size_ - 1),
        bernstein_(spline::makeBernstein<num_t>((unsigned int)degree_)) {
    if (Safe && (size_ < 1 || T_ <= 0.))
      throw std::out_of_range("can't create bezier min bound is higher than max bound");
    t_point_t updatedList = add_constraints<In>(PointsBegin, PointsEnd, constraints);
    for (cit_point_t cit = updatedList.begin(); cit != updatedList.end(); ++cit) pts_.push_back(*cit);
  }

  ///\brief Destructor
  ~bezier_curve() {
    // NOTHING
  }

 private:
  //	bezier_curve(const bezier_curve&);
  //  bezier_curve& operator=(const bezier_curve&);
  /* Constructors - destructors */

  /*Operations*/
 public:
  ///  \brief Evaluation of the cubic spline at time t.
  ///  \param t : the time when to evaluate the spine
  ///  \param return : the value x(t)
  virtual point_t operator()(const time_t t) const {
    if (Safe & !(0 <= t && t <= T_)) throw std::out_of_range("can't evaluate bezier curve, out of range");  // TODO
    if (size_ == 1) {
      return mult_T_ * pts_[0];
    } else {
      return evalHorner(t);
    }
  }

  ///  \brief Computes the derivative curve at order N.
  ///  \param order : order of the derivative
  ///  \param return : the value x(t)
  bezier_curve_t compute_derivate(const std::size_t order) const {
    if (order == 0) return *this;
    t_point_t derived_wp;
    for (typename t_point_t::const_iterator pit = pts_.begin(); pit != pts_.end() - 1; ++pit)
      derived_wp.push_back((num_t)degree_ * (*(pit + 1) - (*pit)));
    if (derived_wp.empty()) derived_wp.push_back(point_t::Zero(Dim));
    bezier_curve_t deriv(derived_wp.begin(), derived_wp.end(), T_, mult_T_ * (1. / T_));
    return deriv.compute_derivate(order - 1);
  }

  ///  \brief Computes the primitive of the curve at order N.
  ///  \param constant : value of the primitive at t = 0
  ///  \param return : the curve x_1(t) such that d/dt(x_1(t)) = x_1(t)
  bezier_curve_t compute_primitive(const std::size_t order) const {
    if (order == 0) return *this;
    num_t new_degree = (num_t)(degree_ + 1);
    t_point_t n_wp;
    point_t current_sum = point_t::Zero(Dim);
    // recomputing waypoints q_i from derivative waypoints p_i. q_0 is the given constant.
    // then q_i = (sum( j = 0 -> j = i-1) p_j) /n+1
    n_wp.push_back(current_sum);
    for (typename t_point_t::const_iterator pit = pts_.begin(); pit != pts_.end(); ++pit) {
      current_sum += *pit;
      n_wp.push_back(current_sum / new_degree);
    }
    bezier_curve_t integ(n_wp.begin(), n_wp.end(), T_, mult_T_ * T_);
    return integ.compute_primitive(order - 1);
  }

  ///  \brief Evaluates the derivative at order N of the curve.
  ///  If the derivative is to be evaluated several times, it is
  ///  rather recommended to compute the derivative curve using compute_derivate
  ///  \param order : order of the derivative
  ///  \param t : the time when to evaluate the spine
  ///  \param return : the value x(t)
  virtual point_t derivate(const time_t t, const std::size_t order) const {
    bezier_curve_t deriv = compute_derivate(order);
    return deriv(t);
  }

  ///
  /// \brief Evaluates all Bernstein polynomes for a certain degree
  /// Warning: the horner scheme is about 100 times faster than this method.
  /// This method will probably be removed in the future
  ///
  point_t evalBernstein(const Numeric t) const {
    const Numeric u = t / T_;
    point_t res = point_t::Zero(Dim);
    typename t_point_t::const_iterator pts_it = pts_.begin();
    for (typename std::vector<Bern<Numeric> >::const_iterator cit = bernstein_.begin(); cit != bernstein_.end();
         ++cit, ++pts_it)
      res += cit->operator()(u) * (*pts_it);
    return res * mult_T_;
  }

  ///
  /// \brief Evaluates all Bernstein polynomes for a certain degree using horner's scheme
  ///
  point_t evalHorner(const Numeric t) const {
    const Numeric u = t / T_;
    typename t_point_t::const_iterator pts_it = pts_.begin();
    Numeric u_op, bc, tn;
    u_op = 1.0 - u;
    bc = 1;
    tn = 1;
    point_t tmp = (*pts_it) * u_op;
    ++pts_it;
    for (unsigned int i = 1; i < degree_; i++, ++pts_it) {
      tn = tn * u;
      bc = bc * ((num_t)(degree_ - i + 1)) / i;
      tmp = (tmp + tn * bc * (*pts_it)) * u_op;
    }
    return (tmp + tn * u * (*pts_it)) * mult_T_;
  }

  const t_point_t& waypoints() const { return pts_; }

  /**
   * @brief evalDeCasteljau evaluate the curve value at time t using deCasteljau algorithm
   * @param t unNormalized time
   * @return the point at time t
   */
  point_t evalDeCasteljau(const Numeric t) const {
    // normalize time :
    const Numeric u = t / T_;
    t_point_t pts = deCasteljauReduction(waypoints(), u);
    while (pts.size() > 1) {
      pts = deCasteljauReduction(pts, u);
    }
    return pts[0] * mult_T_;
  }

  t_point_t deCasteljauReduction(const Numeric t) const { return deCasteljauReduction(waypoints(), t / T_); }

  /**
   * @brief deCasteljauReduction compute the de Casteljau's reduction of the given list of points at time t
   * @param pts the original list of points
   * @param u the NORMALIZED time
   * @return the reduced list of point (size of pts - 1)
   */
  t_point_t deCasteljauReduction(const t_point_t& pts, const Numeric u) const {
    if (u < 0 || u > 1) throw std::out_of_range("In deCasteljau reduction : u is not in [0;1]");
    if (pts.size() == 1) return pts;

    t_point_t new_pts;
    for (cit_point_t cit = pts.begin(); cit != (pts.end() - 1); ++cit) {
      new_pts.push_back((1 - u) * (*cit) + u * (*(cit + 1)));
    }
    return new_pts;
  }

  /**
   * @brief split split the curve in 2 at time t
   * @param t
   * @return
   */
  std::pair<bezier_curve_t, bezier_curve_t> split(const Numeric t) {
    if (t == T_) throw std::runtime_error("can't split curve, interval range is equal to original curve");
    t_point_t wps_first(size_), wps_second(size_);
    const double u = t / T_;
    wps_first[0] = pts_.front();
    wps_second[degree_] = pts_.back();
    t_point_t casteljau_pts = waypoints();
    size_t id = 1;
    while (casteljau_pts.size() > 1) {
      casteljau_pts = deCasteljauReduction(casteljau_pts, u);
      wps_first[id] = casteljau_pts.front();
      wps_second[degree_ - id] = casteljau_pts.back();
      ++id;
    }

    bezier_curve_t c_first(wps_first.begin(), wps_first.end(), t, mult_T_);
    bezier_curve_t c_second(wps_second.begin(), wps_second.end(), T_ - t, mult_T_);
    return std::make_pair(c_first, c_second);
  }

  bezier_curve_t extract(const Numeric t1, const Numeric t2) {
    if (t1 < 0. || t1 > T_ || t2 < 0. || t2 > T_) throw std::out_of_range("In Extract curve : times out of bounds");
    if (t1 == 0. && t2 == T_) return bezier_curve_t(waypoints().begin(), waypoints().end(), T_, mult_T_);
    if (t1 == 0.) return split(t2).first;
    if (t2 == T_) return split(t1).second;

    std::pair<bezier_curve_t, bezier_curve_t> c_split = this->split(t1);
    return c_split.second.split(t2 - t1).first;
  }

 private:
  template <typename In>
  t_point_t add_constraints(In PointsBegin, In PointsEnd, const curve_constraints_t& constraints) {
    t_point_t res;
    point_t P0, P1, P2, P_n_2, P_n_1, PN;
    P0 = *PointsBegin;
    PN = *(PointsEnd - 1);
    P1 = P0 + constraints.init_vel / (num_t)degree_;
    P_n_1 = PN - constraints.end_vel / (num_t)degree_;
    P2 = constraints.init_acc / (num_t)(degree_ * (degree_ - 1)) + 2 * P1 - P0;
    P_n_2 = constraints.end_acc / (num_t)(degree_ * (degree_ - 1)) + 2 * P_n_1 - PN;

    res.push_back(P0);
    res.push_back(P1);
    res.push_back(P2);

    for (In it = PointsBegin + 1; it != PointsEnd - 1; ++it) res.push_back(*it);

    res.push_back(P_n_2);
    res.push_back(P_n_1);
    res.push_back(PN);
    return res;
  }

  /*Operations*/

  /*Helpers*/
 public:
  virtual time_t min() const { return 0.; }
  virtual time_t max() const { return T_; }
  /*Helpers*/

 public:
  /*const*/ time_t T_;
  /*const*/ time_t mult_T_;
  /*const*/ std::size_t size_;
  /*const*/ std::size_t degree_;
  /*const*/ std::vector<Bern<Numeric> > bernstein_;

 private:
  t_point_t pts_;

 public:
  static bezier_curve_t zero(const time_t T = 1.) {
    std::vector<point_t> ts;
    ts.push_back(point_t::Zero(Dim));
    return bezier_curve_t(ts.begin(), ts.end(), T);
  }
};
}  // namespace spline
#endif  //_CLASS_BEZIERCURVE
