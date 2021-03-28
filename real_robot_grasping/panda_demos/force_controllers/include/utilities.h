#ifndef UTILITIES_H
#define UTILITIES_H

#include <kdl/tree.hpp>
#include <Eigen/Dense>

/*! \brief Calculates the Moore-Penrose Pseudoinverse for any sized matrices.
 *  \author http://eigendobetter.com/ (edited by Marcus A Johansson) */
template<typename Derived>
Derived pinv(const Eigen::MatrixBase<Derived>& a)
{
    typedef typename Eigen::MatrixBase<Derived>::RealScalar RealScalar;
    if (a.rows() < a.cols())
    {
        auto svd = a.derived().transpose().jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        RealScalar tolerance = (RealScalar)std::numeric_limits<RealScalar>::epsilon() * std::max((RealScalar)a.cols(), (RealScalar)a.rows()) * svd.singularValues().array().abs().maxCoeff();
        return (svd.matrixV() * Derived((svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0)).asDiagonal() * svd.matrixU().adjoint()).transpose();
    }
    Eigen::JacobiSVD<Derived> svd = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    RealScalar tolerance = (RealScalar)std::numeric_limits<RealScalar>::epsilon() * std::max((RealScalar)a.cols(), (RealScalar)a.rows()) * svd.singularValues().array().abs().maxCoeff();
    return svd.matrixV() * Derived((svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0)).asDiagonal() * svd.matrixU().adjoint();
}

/*! \brief Gets the q-number (used in KDL for identifying joints) from a joint name.
 *  \author Marcus A Johansson */
int kdl_getQNrFromJointName(const KDL::Tree& kdl_tree, const std::string& joint_name);

#endif