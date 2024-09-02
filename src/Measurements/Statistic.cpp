/**
 * @file Statistic.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-08-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <Measurements/Statistic.h>

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

using namespace Measurements;

/**
 * @brief Reset statistics
 */
template <typename Type>
void Statistic<Type>::reset()
{
    _isReset = true;
}

/**
 * @brief Calculate statistics on the data set
 *
 * @param data Data buffer
 * @param size Data size
 */
template <typename Type>
void Statistic<Type>::calculate(const Type *data, size_t size)
{
    assert(data);
    assert(size > 0);

    if (_isReset == true)
    {
        _isReset = false;

        // Set maximum/minimum to the first value
        _max = data[0];
        _min = data[0];

        // Reset average/deviation value
        _mean = 0;
        _deviation = 0;
    }

    for (size_t idx = 0; idx < size; idx++)
    {
        Type value = data[idx];

        updateMaxMin(value);

        _mean += static_cast<double>(value);
    }

    // Calculate average
    _mean /= size;

    for (size_t idx = 0; idx < size; idx++)
    {
        double diff = static_cast<double>(data[idx]) - _mean;

        _deviation += (diff * diff);
    }

    // Calculate standard deviation
    _deviation /= size;
    _deviation = sqrt(_deviation);
}

/**
 * @brief Get the maximum from the data set
 *
 * @return Maximum value
 */
template <typename Type>
Type Statistic<Type>::max() const
{
    return _max;
}

/**
 * @brief Get the minimum from the data set
 *
 * @return Maximum value
 */
template <typename Type>
Type Statistic<Type>::min() const
{
    return _min;
}

/**
 * @brief Get the average from the data set
 *
 * @return Average value
 */
template <typename Type>
double Statistic<Type>::mean() const
{
    return _mean;
}

/**
 * @brief Get the standard deviation from the data set
 *
 * @return Deviation value
 */
template <typename Type>
double Statistic<Type>::deviation() const
{
    return _deviation;
}

/**
 * @brief Check if the maximum/minimum should be updated with new value
 *
 * @param[in] value New value to check
 */
template <typename Type>
void Statistic<Type>::updateMaxMin(Type value)
{
    if (_min > value)
    {
        _min = value;
    }

    if (_max < value)
    {
        _max = value;
    }
}

template class Statistic<int16_t>;
template class Statistic<float>;
