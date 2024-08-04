/**
 * @file Statistic.h
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Statistics calculation API
 * @version 0.1
 * @date 2024-06-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

namespace Measurements
{
    template <typename Type>
    class Statistic
    {
    public:
        /**
         * @brief Reset statistics
         */
        void reset();

        /**
         * @brief Calculate statistics on the data set
         *
         * @param data Data buffer
         * @param size Data size
         */
        void calculate(const Type *data, size_t size);

        /**
         * @brief Get the maximum from the data set
         *
         * @return Maximum value
         */
        Type max() const;

        /**
         * @brief Get the minimum from the data set
         *
         * @return Maximum value
         */
        Type min() const;

        /**
         * @brief Get the average from the data set
         *
         * @return Average value
         */
        double mean() const;

        /**
         * @brief Get the standard deviation from the data set
         *
         * @return Deviation value
         */
        double deviation() const;

    private:
        /**
         * @brief Check if the maximum/minimum should be updated with new value
         *
         * @param[in] value New value to check
         */
        void updateMaxMin(Type value);

        bool _isReset = true;
        Type _max;
        Type _min;
        double _mean;
        double _deviation;
    };
} // namespace Measurements
