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

#ifndef DOUBLE2D_H
#define DOUBLE2D_H


/**
 * \file       Double2D.h
 * \brief      Contains the Double2D class.
 */

#include <iostream>
#include <cmath>
using namespace std;

namespace acasx {
    /**
     * \brief      Defines a two-dimensional double2D.
     */
    class Double2D {
    public:
        /**
         * \brief      Constructs and initializes a two-dimensional double2D instance
         *             to (0.0, 0.0).
         */
        inline Double2D() : x_(0.0), y_(0.0) { }

        /**
         * \brief      Constructs and initializes a two-dimensional double2D from
         *             the specified xy-coordinates.
         * \param      x               The x-coordinate of the two-dimensional
         *                             double2D.
         * \param      y               The y-coordinate of the two-dimensional
         *                             double2D.
         */
        inline Double2D(double x, double y) : x_(x), y_(y) { }

        /**
         * \brief      Returns the x-coordinate of this two-dimensional double2D.
         * \return     The x-coordinate of the two-dimensional double2D.
         */
        inline double x() const { return x_; }

        /**
         * \brief      Returns the y-coordinate of this two-dimensional double2D.
         * \return     The y-coordinate of the two-dimensional double2D.
         */
        inline double y() const { return y_; }

        /**
         * \brief      Computes the length of this two-dimensional double2D.
         * \return     The length of this two-dimensional double2D.
         */
        inline double length() const
        {
            return std::sqrt(x_*x_ + y_*y_);
        }

        /**
         * \brief      Computes the angle of this two-dimensional double2D.
         * \return     The angle (in radians) of this two-dimensional double2D.
         */
        inline double angle() const
        {
            return std::atan2(y_, x_);
        }

        /**
         * \brief      resize the two-dimensional double2D to length.
         * \return     a new two-dimensional double2D.
         */
        inline Double2D resize(double newLength) const
        {
            double factor = newLength==0? 0:std::sqrt(x_*x_ + y_*y_)/newLength;
            return Double2D(x_ * factor, y_ * factor);
        }

        /**
         * \brief      Computes the negation of this two-dimensional double2D.
         * \return     The negation of this two-dimensional double2D.
         */
        inline Double2D operator-() const
        {
            return Double2D(-x_, -y_);
        }

        /**
         * \brief      Computes the dot product of this two-dimensional double2D with
         *             the specified two-dimensional double2D.
         * \param      double2D          The two-dimensional double2D with which the
         *                             dot product should be computed.
         * \return     The dot product of this two-dimensional double2D with a
         *             specified two-dimensional double2D.
         */
        inline double operator*(const Double2D& double2D) const
        {
            return x_ * double2D.x() + y_ * double2D.y();
        }

        /**
         * \brief      Computes the scalar multiplication of this
         *             two-dimensional double2D with the specified scalar value.
         * \param      s               The scalar value with which the scalar
         *                             multiplication should be computed.
         * \return     The scalar multiplication of this two-dimensional double2D
         *             with a specified scalar value.
         */
        inline Double2D operator*(double s) const
        {
            return Double2D(x_ * s, y_ * s);
        }

        /**
         * \brief      Computes the scalar division of this two-dimensional double2D
         *             with the specified scalar value.
         * \param      s               The scalar value with which the scalar
         *                             division should be computed.
         * \return     The scalar division of this two-dimensional double2D with a
         *             specified scalar value.
         */
        inline Double2D operator/(double s) const
        {
            const double invS = 1.0 / s;

            return Double2D(x_ * invS, y_ * invS);
        }

        /**
         * \brief      Computes the double2D sum of this two-dimensional double2D with
         *             the specified two-dimensional double2D.
         * \param      double2D          The two-dimensional double2D with which the
         *                             double2D sum should be computed.
         * \return     The double2D sum of this two-dimensional double2D with a
         *             specified two-dimensional double2D.
         */
        inline Double2D operator+(const Double2D& double2D) const
        {
            return Double2D(x_ + double2D.x(), y_ + double2D.y());
        }

        /**
         * \brief      Computes the double2D difference of this two-dimensional
         *             double2D with the specified two-dimensional double2D.
         * \param      double2D          The two-dimensional double2D with which the
         *                             double2D difference should be computed.
         * \return     The double2D difference of this two-dimensional double2D with a
         *             specified two-dimensional double2D.
         */
        inline Double2D operator-(const Double2D& double2D) const
        {
            return Double2D(x_ - double2D.x(), y_ - double2D.y());
        }

        /**
         * \brief      Tests this two-dimensional double2D for equality with the
         *             specified two-dimensional double2D.
         * \param      double2D          The two-dimensional double2D with which to
         *                             test for equality.
         * \return     True if the two-dimensional double2Ds are equal.
         */
        inline bool operator==(const Double2D& double2D) const
        {
            return x_ == double2D.x() && y_ == double2D.y();
        }

        /**
         * \brief      Tests this two-dimensional double2D for inequality with the
         *             specified two-dimensional double2D.
         * \param      double2D          The two-dimensional double2D with which to
         *                             test for inequality.
         * \return     True if the two-dimensional double2Ds are not equal.
         */
        inline bool operator!=(const Double2D& double2D) const
        {
            return x_ != double2D.x() || y_ != double2D.y();
        }

        /**
         * \brief      Sets the value of this two-dimensional double2D to the scalar
         *             multiplication of itself with the specified scalar value.
         * \param      s               The scalar value with which the scalar
         *                             multiplication should be computed.
         * \return     A reference to this two-dimensional double2D.
         */
        inline Double2D& operator*=(double s)
        {
            x_ *= s;
            y_ *= s;

            return *this;
        }

        /**
         * \brief      Sets the value of this two-dimensional double2D to the scalar
         *             division of itself with the specified scalar value.
         * \param      s               The scalar value with which the scalar
         *                             division should be computed.
         * \return     A reference to this two-dimensional double2D.
         */
        inline Double2D& operator/=(double s)
        {
            const double invS = 1.0 / s;
            x_ *= invS;
            y_ *= invS;

            return *this;
        }

        /**
         * \brief      Sets the value of this two-dimensional double2D to the double2D
         *             sum of itself with the specified two-dimensional double2D.
         * \param      double2D          The two-dimensional double2D with which the
         *                             double2D sum should be computed.
         * \return     A reference to this two-dimensional double2D.
         */
        inline Double2D& operator+=(const Double2D& double2D)
        {
            x_ += double2D.x();
            y_ += double2D.y();

            return *this;
        }

        /**
         * \brief      Sets the value of this two-dimensional double2D to the double2D
         *             difference of itself with the specified two-dimensional
         *             double2D.
         * \param      double2D          The two-dimensional double2D with which the
         *                             double2D difference should be computed.
         * \return     A reference to this two-dimensional double2D.
         */
        inline Double2D& operator-=(const Double2D& double2D)
        {
            x_ -= double2D.x();
            y_ -= double2D.y();

            return *this;
        }

    private:
        double x_;
        double y_;
    };

    /**
     * \relates    Double2D
     * \brief      Computes the scalar multiplication of the specified
     *             two-dimensional double2D with the specified scalar value.
     * \param      s               The scalar value with which the scalar
     *                             multiplication should be computed.
     * \param      double2D          The two-dimensional double2D with which the scalar
     *                             multiplication should be computed.
     * \return     The scalar multiplication of the two-dimensional double2D with the
     *             scalar value.
     */
    inline Double2D operator*(double s, const Double2D& double2D)
    {
        return Double2D(s * double2D.x(), s * double2D.y());
    }

    /**
     * \relates    Double2D
     * \brief      Inserts the specified two-dimensional double2D into the specified
     *             output stream.
     * \param      os              The output stream into which the two-dimensional
     *                             double2D should be inserted.
     * \param      double2D          The two-dimensional double2D which to insert into
     *                             the output stream.
     * \return     A reference to the output stream.
     */
    inline std::ostream& operator<<(std::ostream& os, const Double2D& double2D)
    {
        os << "(" << double2D.x() << "," << double2D.y() << ")";

        return os;
    }

    /**
     * \relates    Double2D
     * \brief      Computes the length of a specified two-dimensional
     *             double2D.
     * \param      double2D          The two-dimensional double2D whose squared length
     *                             is to be computed.
     * \return     The length of the two-dimensional double2D.
     */
    inline double length(const Double2D& double2D)
    {
        return std::sqrt(double2D * double2D);
    }

    /**
     * \relates    Double2D
     * \brief      Computes the squared length of a specified two-dimensional
     *             double2D.
     * \param      double2D          The two-dimensional double2D whose squared length
     *                             is to be computed.
     * \return     The squared length of the two-dimensional double2D.
     */
    inline double lengthSq(const Double2D& double2D)
    {
        return double2D * double2D;
    }

    /**
     * \relates    Double2D
     * \brief      Computes the determinant of a two-dimensional square matrix with
     *             rows consisting of the specified two-dimensional double2Ds.
     * \param      double2D1         The top row of the two-dimensional square
     *                             matrix.
     * \param      Double2D         The bottom row of the two-dimensional square
     *                             matrix.
     * \return     The determinant of the two-dimensional square matrix.
     */
    inline double det(const Double2D& a, const Double2D& b)
    {
        return a.x() * b.y() - a.y() * b.x();
    }

    /**
     * \relates    Double2D
     * \brief      Computes the normalization of the specified two-dimensional
     *             double2D.
     * \param      double2D          The two-dimensional double2D whose normalization
     *                             is to be computed.
     * \return     The normalization of the two-dimensional double2D.
     */
    inline Double2D normalize(const Double2D& double2D)
    {
        return double2D / length(double2D);
    }
}

#endif // DOUBLE2D_H

