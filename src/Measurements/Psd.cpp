/**
 * @file Psd.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Power Spectral Density calculation module implementation
 * @version 0.1
 * @date 2024-06-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "Measurements/Psd.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include <arduinoFFT.h>

#include <Log.hpp>

using namespace Measurements;

namespace
{
    // Coefficient for Hamming window correction
    constexpr float windowCorrection = 1.59;

    /**
     * Calculate average value for elements
     */
    template <typename T>
    float getAverage(T *elements, size_t count)
    {
        assert(elements);
        assert(count > 0);

        float sum = 0;
        for (size_t idx = 0; idx < count; idx++)
        {
            sum += elements[idx];
        }

        float result = sum / count;
        return result;
    }

    /*
    These are the input and output vectors
    Input vectors receive computed results from FFT
    */
    float vReal[samplesCountMax];
    float vImag[samplesCountMax];

    // FFT object
    auto fft = ArduinoFFT<float>();
} // namespace

/**
 * @brief Prepare PSD calculations, setup data segment parameters
 *
 * @param[in] sampleCount Samples count in the segment
 * @param[in] sampleFrequency Sampling frequency, Hz
 */
template <typename Type>
void PSD<Type>::setup(size_t sampleCount, size_t sampleFrequency)
{
    assert(sampleCount <= samplesCountMax);

    // Save samples count
    _sampleCount = sampleCount;
    // Save sampling frequency
    _sampleFrequency = sampleFrequency;
    // Calculate bins count (only the first N/2 + 1 are usefull, where N = sampleCount)
    _binCount = sampleCount / 2 + 1;

    // Reset computed segment count
    _segmentCount = 0;
}

/**
 * @brief Compute PSD for the next segment
 *
 * @param[in] samples Data samples of the segmennt
 */
template <typename Type>
void PSD<Type>::computeSegment(const Type *samples)
{
    if (_segmentCount == 0)
    {
        // Clear PSD results before adding new data
        clear();
    }

    float average = getAverage(samples, _sampleCount);
    for (size_t idx = 0; idx < _sampleCount; idx++)
    {
        vReal[idx] = static_cast<float>(samples[idx]) - average;
        vImag[idx] = 0;
    }

    fft.windowing(vReal, _sampleCount, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    fft.compute(vReal, vImag, _sampleCount, FFT_FORWARD);
    fft.complexToMagnitude(vReal, vImag, _sampleCount);

    for (size_t idx = 0; idx < _binCount; idx++)
    {
        float bin = static_cast<float>(vReal[idx]) * vReal[idx] / _sampleFrequency / _sampleCount;
        if (idx > 0)
        {
            bin *= 2;
        }

        _bins[idx] += bin;
    }

    _segmentCount++;
}

/**
 * @brief Return PSD results
 * Reset accumulated segment count (finish previous segments computing) if there are any segments
 *
 * @param[out] pCoreBin Pointer to the core (maximum amplitude) bin in the results (nullptr if no need)
 * @return Calculated bins
 */
template <typename Type>
const float *PSD<Type>::getResult(PsdBin *pCoreBin)
{
    if (_segmentCount > 0)
    {
        // Set index to the invalid value first
        size_t binMaxIdx = _sampleCount;
        float binMaxAmplitude = 0;

        // Calculate average bins
        for (size_t idx = 0; idx < _binCount; idx++)
        {
            _bins[idx] = (_bins[idx] / _segmentCount) * windowCorrection * windowCorrection;

            // Check if index is invalid or bigger bin found
            if (binMaxIdx == _sampleCount || _bins[idx] > binMaxAmplitude)
            {
                // Update the bin attributes
                binMaxIdx = idx;
                binMaxAmplitude = _bins[idx];
            }

            LOG_TRACE("Bin[%d]: %lf", idx, _bins[idx]);
        }

        float deltaFrequency = static_cast<float>(_sampleFrequency) / _sampleCount;
        _coreBin.frequency = binMaxIdx * deltaFrequency;
        _coreBin.amplitude = binMaxAmplitude;

        // Reset number of segment to prevent repeated result calculation
        _segmentCount = 0;
    }

    if (pCoreBin != nullptr)
    {
        *pCoreBin = _coreBin;
    }

    return _bins;
}

/**
 * @brief Clear PSD results
 */
template <typename Type>
void PSD<Type>::clear()
{
    // Clear all bins
    for (size_t idx = 0; idx < _binCount; idx++)
    {
        _bins[idx] = 0;
    }
}

template class PSD<uint16_t>;
template class PSD<int16_t>;
template class PSD<float>;
