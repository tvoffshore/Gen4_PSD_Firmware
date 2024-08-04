/**
 * @file Psd.h
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Power Spectral Density calculation module API
 * @version 0.1
 * @date 2024-05-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

namespace Measurements
{
    class PSD
    {
    public:
        // Maximum allowed samples in the segment
        static constexpr size_t samplesCountMax = 1024;

        /**
         * @brief Prepare PSD calculations, setup data segment parameters
         *
         * @param[in] sampleCount Samples count in the segment
         * @param[in] sampleFrequency Sampling frequency
         */
        void setup(size_t sampleCount, size_t sampleFrequency);

        /**
         * @brief Compute PSD for the next segment
         *
         * @param[in] samples Data samples of the segmennt
         */
        void computeSegment(const int16_t *samples);

        /**
         * @brief Return PSD results
         * Reset accumulated segment count (finish previous segments computing) if there are any segments
         *
         * @return Calculated bins (only the first N/2 + 1 are usefull, where N = sampleCount)
         */
        double *getResult();

    private:
        /**
         * @brief Clear PSD results
         */
        void clear();

        size_t _sampleFrequency; // Sampling frequency
        size_t _sampleCount;     // Number of sample in segment
        size_t _segmentCount;    // Number of computed segments

        double _bins[samplesCountMax]; // PSD results
    };
} // namespace Measurements
