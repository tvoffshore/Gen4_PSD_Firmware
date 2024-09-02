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
    // Maximum allowed samples in the segment
    constexpr size_t samplesCountMax = 1024;

    /**
     * @brief PSD bin information structure
     */
    struct PsdBin
    {
        double frequency;
        double amplitude;
    };

    template <typename Type>
    class PSD
    {
    public:
        /**
         * @brief Prepare PSD calculations, setup data segment parameters
         *
         * @param[in] sampleCount Samples count in the segment
         * @param[in] sampleFrequency Sampling frequency, Hz
         */
        void setup(size_t sampleCount, size_t sampleFrequency);

        /**
         * @brief Compute PSD for the next segment
         *
         * @param[in] samples Data samples of the segmennt
         */
        void computeSegment(const Type *samples);

        /**
         * @brief Return PSD results
         * Reset accumulated segment count (finish previous segments computing) if there are any segments
         *
         * @param[out] coreBin Pointer to the core (maximum amplitude) bin in the results (nullptr if no need)
         * @return Calculated bins
         */
        const double *getResult(PsdBin *pCoreBin = nullptr);

    private:
        /**
         * @brief Clear PSD results
         */
        void clear();

        size_t _sampleFrequency; // Sampling frequency
        size_t _sampleCount;     // Number of sample in segment
        size_t _segmentCount;    // Number of computed segments
        size_t _binCount;        // Number of bins

        PsdBin _coreBin; // Core (maximum amplitude) bin

        double _bins[samplesCountMax / 2 + 1]; // PSD results (only the first N/2 + 1 are usefull, where N = sampleCount)
    };
} // namespace Measurements
