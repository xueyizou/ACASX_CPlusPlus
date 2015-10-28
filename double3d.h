/* *************************************************************************************
 * Copyright (C) Xueyi Zou - All Rights Reserved
 * Written by Xueyi Zou <xz972@york.ac.uk>, 2015
 * You are free to use/modify/distribute this file for whatever purpose!
 -----------------------------------------------------------------------
 |THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 |WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
 |AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
 |DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
 |MISUSING THIS SOFTWARE.
 ------------------------------------------------------------------------

 **************************************************************************************/

#ifndef DOUBLE3D_H
#define DOUBLE3D_H

/**
 * \file       Double3D.h
 * \brief      Contains the Double3D class.
 */

#include <iostream>
#include <cmath>
using namespace std;

namespace acasx {
    /**
     * \brief      Defines a three-dimensional Double3D.
     */
    class Double3D {
    public:
        /**
         * \brief      Constructs and initializes a three-dimensional Double3D instance
         *             to (0.0, 0.0, 0.0).
         */
        inline Double3D() : x_(0.0), y_(0.0), z_(0.0) { }

        /**
         * \brief      Constructs and initializes a three-dimensional Double3D from
         *             the specified xyz-coordinates.
         * \param      x               The x-coordinate of the three-dimensional
         *                             Double3D.
         * \param      y               The y-coordinate of the three-dimensional
         *                             Double3D.
         * \param      z               The z-coordinate of the three-dimensional
         *                             Double3D.
         */
        inline Double3D(double x, double y, double z) : x_(x), y_(y), z_(z){ }

        /**
         * \brief      Returns the x-coordinate of this three-dimensional Double3D.
         * \return     The x-coordinate of the three-dimensional Double3D.
         */
        inline double x() const { return x_; }

        /**
         * \brief      Returns the y-coordinate of this three-dimensional Double3D.
         * \return     The y-coordinate of the three-dimensional Double3D.
         */
        inline double y() const { return y_; }

        /**
         * \brief      Returns the z-coordinate of this three-dimensional Double3D.
         * \return     The z-coordinate of the three-dimensional Double3D.
         */
        inline double z() const { return z_; }

        /**
         * \brief      Computes the length of this three-dimensional Double3D.
         * \return     The length of this three-dimensional Double3D.
         */
        inline double length() const
        {
            return std::sqrt(x_*x_ + y_*y_+ z_*z_);
        }

        /**
         * \brief      resize the three-dimensional Double3D to length.
         * \return     a new three-dimensional Double3D.
         */
        inline Double3D resize(double newLength) const
        {
            double factor = newLength==0? 0:std::sqrt(x_*x_ + y_*y_+ z_*z_)/newLength;
            return Double3D(x_ * factor, y_ * factor, z_ * factor);
        }

        /**
         * \brief      Computes the negation of this three-dimensional Double3D.
         * \return     The negation of this three-dimensional Double3D.
         */
        inline Double3D operator-() const
        {
            return Double3D(-x_, -y_, -z_);
        }

        /**
         * \brief      Computes the dot product of this three-dimensional Double3D with
         *             the specified three-dimensional Double3D.
         * \param      double3D          The three-dimensional Double3D with which the
         *                             dot product should be computed.
         * \return     The dot product of this three-dimensional Double3D with a
         *             specified three-dimensional Double3D.
         */
        inline double operator*(const Double3D& double3D) const
        {
            return x_ * double3D.x() + y_ * double3D.y()+ z_ * double3D.z();
        }

        /**
         * \brief      Computes the scalar multiplication of this
         *             three-dimensional Double3D with the specified scalar value.
         * \param      s               The scalar value with which the scalar
         *                             multiplication should be computed.
         * \return     The scalar multiplication of this three-dimensional Double3D
         *             with a specified scalar value.
         */
        inline Double3D operator*(double s) const
        {
            return Double3D(x_ * s, y_ * s, z_ * s);
        }

        /**
         * \brief      Computes the scalar division of this three-dimensional Double3D
         *             with the specified scalar value.
         * \param      s               The scalar value with which the scalar
         *                             division should be computed.
         * \return     The scalar division of this three-dimensional Double3D with a
         *             specified scalar value.
         */
        inline Double3D operator/(double s) const
        {
            const double invS = 1.0 / s;

            return Double3D(x_ * invS, y_ * invS, z_ * invS);
        }

        /**
         * \brief      Computes the Double3D sum of this three-dimensional Double3D with
         *             the specified three-dimensional Double3D.
         * \param      double3D          The three-dimensional Double3D with which the
         *                             Double3D sum should be computed.
         * \return     The Double3D sum of this three-dimensional Double3D with a
         *             specified three-dimensional Double3D.
         */
        inline Double3D operator+(const Double3D& double3D) const
        {
            return Double3D(x_ + double3D.x(), y_ + double3D.y(), z_ + double3D.z());
        }

        /**
         * \brief      Computes the Double3D difference of this three-dimensional
         *             Double3D with the specified three-dimensional Double3D.
         * \param      double3D          The three-dimensional Double3D with which the
         *                             Double3D difference should be computed.
         * \return     The Double3D difference of this three-dimensional Double3D with a
         *             specified three-dimensional Double3D.
         */
        inline Double3D operator-(const Double3D& double3D) const
        {
            return Double3D(x_ - double3D.x(), y_ - double3D.y(), z_ - double3D.z());
        }

        /**
         * \brief      Tests this three-dimensional Double3D for equality with the
         *             specified three-dimensional Double3D.
         * \param      double3D          The three-dimensional Double3D with which to
         *                             test for equality.
         * \return     True if the three-dimensional Double3Ds are equal.
         */
        inline bool operator==(const Double3D& double3D) const
        {
            return x_ == double3D.x() && y_ == double3D.y()&& z_ == double3D.z();
        }

        /**
         * \brief      Tests this three-dimensional Double3D for inequality with the
         *             specified three-dimensional Double3D.
         * \param      double3D          The three-dimensional Double3D with which to
         *                             test for inequality.
         * \return     True if the three-dimensional Double3Ds are not equal.
         */
        inline bool operator!=(const Double3D& double3D) const
        {
            return x_ != double3D.x() || y_ != double3D.y()|| z_ != double3D.z();
        }

        /**
         * \brief      Sets the value of this three-dimensional Double3D to the scalar
         *             multiplication of itself with the specified scalar value.
         * \param      s               The scalar value with which the scalar
         *                             multiplication should be computed.
         * \return     A reference to this three-dimensional Double3D.
         */
        inline Double3D& operator*=(double s)
        {
            x_ *= s;
            y_ *= s;
            z_ *= s;

            return *this;
        }

        /**
         * \brief      Sets the value of this three-dimensional Double3D to the scalar
         *             division of itself with the specified scalar value.
         * \param      s               The scalar value with which the scalar
         *                             division should be computed.
         * \return     A reference to this three-dimensional Double3D.
         */
        inline Double3D& operator/=(double s)
        {
            const double invS = 1.0 / s;
            x_ *= invS;
            y_ *= invS;
            z_ *= invS;

            return *this;
        }

        /**
         * \brief      Sets the value of this three-dimensional Double3D to the Double3D
         *             sum of itself with the specified three-dimensional Double3D.
         * \param      double3D          The three-dimensional Double3D with which the
         *                             Double3D sum should be computed.
         * \return     A reference to this three-dimensional Double3D.
         */
        inline Double3D& operator+=(const Double3D& double3D)
        {
            x_ += double3D.x();
            y_ += double3D.y();
            z_ += double3D.z();

            return *this;
        }

        /**
         * \brief      Sets the value of this three-dimensional Double3D to the Double3D
         *             difference of itself with the specified three-dimensional
         *             Double3D.
         * \param      double3D          The three-dimensional Double3D with which the
         *                             Double3D difference should be computed.
         * \return     A reference to this three-dimensional Double3D.
         */
        inline Double3D& operator-=(const Double3D& double3D)
        {
            x_ -= double3D.x();
            y_ -= double3D.y();
            z_ -= double3D.z();

            return *this;
        }

    private:
        double x_;
        double y_;
        double z_;
    };

    /**
     * \relates    Double3D
     * \brief      Computes the scalar multiplication of the specified
     *             three-dimensional Double3D with the specified scalar value.
     * \param      s               The scalar value with which the scalar
     *                             multiplication should be computed.
     * \param      double3D          The three-dimensional Double3D with which the scalar
     *                             multiplication should be computed.
     * \return     The scalar multiplication of the three-dimensional Double3D with the
     *             scalar value.
     */
    inline Double3D operator*(double s, const Double3D& double3D)
    {
        return Double3D(s * double3D.x(), s * double3D.y(), s * double3D.z());
    }

    /**
     * \relates    Double3D
     * \brief      Inserts the specified three-dimensional Double3D into the specified
     *             output stream.
     * \param      os              The output stream into which the three-dimensional
     *                             Double3D should be inserted.
     * \param      double3D          The three-dimensional Double3D which to insert into
     *                             the output stream.
     * \return     A reference to the output stream.
     */
    inline std::ostream& operator<<(std::ostream &os, const Double3D& double3D)
    {
        os << "(" << double3D.x() << "," << double3D.y() << "," << double3D.z() << ")";

        return os;
    }

    /**
     * \relates    Double3D
     * \brief      Computes the length of a specified three-dimensional
     *             Double3D.
     * \param      double3D          The three-dimensional Double3D whose squared length
     *                             is to be computed.
     * \return     The length of the three-dimensional Double3D.
     */
    inline double length(const Double3D& double3D)
    {
        return std::sqrt(double3D * double3D);
    }

    /**
     * \relates    Double3D
     * \brief      Computes the squared length of a specified three-dimensional
     *             Double3D.
     * \param      double3D          The three-dimensional Double3D whose squared length
     *                             is to be computed.
     * \return     The squared length of the three-dimensional Double3D.
     */
    inline double lengthSq(const Double3D& double3D)
    {
        return double3D * double3D;
    }


    /**
     * \relates    Double3D
     * \brief      Computes the normalization of the specified three-dimensional
     *             Double3D.
     * \param      double3D          The three-dimensional Double3D whose normalization
     *                             is to be computed.
     * \return     The normalization of the three-dimensional Double3D.
     */
    inline Double3D normalize(const Double3D& double3D)
    {
        return double3D / length(double3D);
    }
}

#endif // DOUBLE3D_H

