#pragma once

#include <stddef.h>

#include <assert.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

namespace RTOS
{
    /**
     * @brief Event group class to set/clear/wait events
     */
    class EventGroup
    {
    public:
        /**
         * @brief Construct a new Event Group object
         */
        EventGroup()
        {
            _eventGroupHandle = xEventGroupCreateStatic(&_eventGroupBuffer);
            assert(_eventGroupHandle);
        }

        /**
         * @brief Set events from task context
         * @warning This function cannot be called from an interrupt, use setIsr instead
         *
         * @param[in] events Events bit mask to set
         */
        void set(EventBits_t events)
        {
            xEventGroupSetBits(_eventGroupHandle, events);
        }

        /**
         * @brief Clear events from task context
         * @warning This function cannot be called from an interrupt, use clearIsr instead
         *
         * @param[in] events Events bit mask to clear
         */
        void clear(EventBits_t events)
        {
            xEventGroupClearBits(_eventGroupHandle, events);
        }

        /**
         * @brief Set events from interrupt context
         *
         * @param[in] events Events bit mask to set
         * @return true if setting succeed, false otherwise
         */
        bool setIsr(EventBits_t events)
        {
            BaseType_t higherPriorityTaskWoken = pdFALSE;

            auto result = xEventGroupSetBitsFromISR(_eventGroupHandle, events, &higherPriorityTaskWoken);
            if (result == pdPASS && higherPriorityTaskWoken == pdTRUE)
            {
                portYIELD_FROM_ISR();
            }

            return (result == pdPASS);
        }

        /**
         * @brief Clear events from interrupt context
         *
         * @param[in] events Events bit mask to clear
         * @return true if clearing succeed, false otherwise
         */
        bool clearIsr(EventBits_t events)
        {
            auto result = xEventGroupClearBitsFromISR(_eventGroupHandle, events);

            return (result == pdPASS);
        }

        /**
         * @brief Wait events
         *
         * @param[in] events Events bit mask to wait
         * @param[in] timeout Waiting timeout, OS ticks
         * @param[in] clearOnExit Clear all set events (true) or left raised (false)
         * @param[in] waitForAllBits Wait for all events to set (true) or anyone (false)
         * @return Set events bit mask or zeto if timeout
         */
        EventBits_t wait(EventBits_t events, size_t timeout = portMAX_DELAY, BaseType_t clearOnExit = pdTRUE, BaseType_t waitForAllBits = pdFALSE)
        {
            EventBits_t setEvents = xEventGroupWaitBits(_eventGroupHandle, events, clearOnExit, waitForAllBits, timeout);

            return setEvents;
        }

    private:
        EventGroupHandle_t _eventGroupHandle; // Event group's handle
        StaticEventGroup_t _eventGroupBuffer; // Event group's static data structure
    };
} // namespace RTOS
