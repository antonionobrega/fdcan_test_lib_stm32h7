/*
 * um_fdcan_lib.h
 *
 *  Created on: Sep 2, 2025
 *      Author: anton
 */

#ifndef INC_UM_FDCAN_LIB_H_
#define INC_UM_FDCAN_LIB_H_

/* Includes -----------------------------------------------------------------------------------------------------------------------------------------------------------*/
#include "stm32h7xx_hal.h" // HAL library for stm32h7
#include "cmsis_os.h"     // Use CMSIS-OS v2 for RTOS objects
/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/* Exported types -----------------------------------------------------------------------------------------------------------------------------------------------------*/

// Forward declaration of the internal subscription structure.
// This is to explain to the compiler that this struct exists
// This makes the handle "opaque" - the user knows it's a type, but not what's inside.
struct FDCAN_Subscription_Internal;

/**
 * @brief An opaque handle representing a subscription to a specific CAN message ID.
 * The user receives this from FDCAN_Subscribe() and uses it for all other functions.
 */
typedef struct FDCAN_Subscription_Internal* FDCAN_SubscriptionHandle_t;

/**
 * @brief Defines the operating mode for the FDCAN wrapper.
 */
typedef enum {
    /** @brief Normal operation with transmit and receive capabilities. */
    FDCAN_MODE_NORMAL,

    /** @brief Bus Monitoring Mode. The peripheral will only listen to the bus
               and will not transmit messages or acknowledge frames. Ideal for diagnostics. */
    FDCAN_MODE_BUS_MONITORING

} FDCAN_OperatingMode_t;

/**
 * @brief Main FDCAN wrapper object. An instance of this should be created for each
 * FDCAN peripheral in use (e.g., one for FDCAN1, one for FDCAN2).
 */
typedef struct {
    // Hardware & RTOS handles
    FDCAN_HandleTypeDef*    hfdcan;
    osThreadId_t            processing_thread_handle;
    osMutexId_t             subscription_list_mutex;

    // Filter Management


    // Subscription management (internal details are in the .c file)
    // This will point to a dynamically allocated array of FDCAN_Subscription_Internal structs.
    struct FDCAN_Subscription_Internal* subscription_list;
    uint32_t                  subscription_count;
    uint32_t                  max_subscriptions;

} FDCAN_Wrapper_t;

/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/* Exported functions -------------------------------------------------------------------------------------------------------------------------------------------------*/

/**
 * @brief Initializes the FDCAN wrapper, its hardware filters, and its processing thread.
 * 
 * @param wrapper Pointer to the FDCAN_Wrapper_t object to initialize.
 * @param hfdcan Pointer to the HAL FDCAN handle (e.g., &hfdcan1).
 * @param op_mode The operating mode for the FDCAN peripheral (Normal or Bus Monitoring).
 * @param interrupt_line The interrupt line to use for new message notifications (e.g., FDCAN_INTERRUPT_LINE0).
 * @param std_filters An array of pre-configured FDCAN_FilterTypeDef structures for standard IDs.
 * @param std_filter_count The number of filters in the 'std_filters' array.
 * @param ext_filters An array of pre-configured FDCAN_FilterTypeDef structures for extended IDs.
 * @param ext_filter_count The number of filters in the 'ext_filters' array.
 * @param max_subscriptions The maximum number of unique software subscriptions this wrapper will support.
 * @param huart_debug Optional. Pointer to a UART handle for debug logging. Can be NULL.
 * @param uart_mutex Optional. Mutex to protect the debug UART. Must be provided if huart_debug is not NULL.
 * 
 * @return osOK on success, otherwise an error code.
 *
 * @example
 *
 * // In user's code (e.g. main.c)
 * // Assume huart3 and uart_debug_mutex are initialized elsewhere.
 *
 * // 1. Define the filter configurations
 * FDCAN_FilterTypeDef my_can1_std_filters[2];
 *
 * // Filter 0: Accept a specific ID 0x100
 * my_can1_std_filters[0].IdType = FDCAN_STANDARD_ID;
 * my_can1_std_filters[0].FilterIndex = 0; // User must manage the index
 * my_can1_std_filters[0].FilterType = FDCAN_FILTER_DUAL;
 * my_can1_std_filters[0].FilterConfig = FDCAN_FILTER_TO_RX_FIFO0;
 * my_can1_std_filters[0].FilterID1 = 0x100;
 * my_can1_std_filters[0].FilterID2 = 0x100; // Set same as ID1 to match only one
 *
 * // Filter 1: Accept a range of IDs from 0x200 to 0x210
 * my_can1_std_filters[1].IdType = FDCAN_STANDARD_ID;
 * my_can1_std_filters[1].FilterIndex = 1; // User must manage the index
 * my_can1_std_filters[1].FilterType = FDCAN_FILTER_RANGE;
 * my_can1_std_filters[1].FilterConfig = FDCAN_FILTER_TO_RX_FIFO0;
 * my_can1_std_filters[1].FilterID1 = 0x200;
 * my_can1_std_filters[1].FilterID2 = 0x210;
 *
 * // 2. Call your wrapper's Init function
 * FDCAN_Wrapper_t can1_wrapper;
 * uint32_t std_filter_count = sizeof(my_can1_std_filters) / sizeof(my_can1_std_filters[0]);
 * FDCAN_Wrapper_Init(&can1_wrapper, 
 *                    &hfdcan1,
 *                    FDCAN_MODE_NORMAL,
 *                    FDCAN_INTERRUPT_LINE0,
 *                    my_can1_std_filters, 
 *                    std_filter_count, 
 *                    NULL,  // No extended filters
 *                    0, 
 *                    10,    // Max 10 software subscriptions
 *                    &huart3, // Optional debug UART
 *                    uart_debug_mutex); // Mutex for the debug UART
 */
osStatus_t FDCAN_Wrapper_Init(FDCAN_Wrapper_t *wrapper,
                              FDCAN_HandleTypeDef *hfdcan,
                              FDCAN_OperatingMode_t op_mode,
                              uint32_t interrupt_line,
                              const FDCAN_FilterTypeDef* std_filters,
                              uint32_t std_filter_count,
                              const FDCAN_FilterTypeDef* ext_filters,
                              uint32_t ext_filter_count,
                              uint32_t max_subscriptions,
                              UART_HandleTypeDef* huart_debug,
                              osMutexId_t uart_mutex);

/**
 * @brief Subscribes to a specific CAN message ID.
 * This creates a dedicated queue for the message and returns a handle to the subscription.
 * @param wrapper Pointer to the initialized FDCAN_Wrapper_t object.
 * @param message_id The CAN message ID to listen for.
 * @return A handle to the subscription, or NULL on failure (e.g., max subscriptions reached).
 */
FDCAN_SubscriptionHandle_t FDCAN_Subscribe(FDCAN_Wrapper_t *wrapper, uint32_t message_id);

/**
 * @brief Unsubscribes and cleans up resources for a given subscription.
 * @param wrapper Pointer to the initialized FDCAN_Wrapper_t object.
 * @param handle The subscription handle that was returned by FDCAN_Subscribe().
 * @return osOK on success, otherwise an error code.
 */
osStatus_t FDCAN_Unsubscribe(FDCAN_Wrapper_t *wrapper, FDCAN_SubscriptionHandle_t handle);

/**
 * @brief Waits to receive a message for a specific subscription. This is a blocking function.
 * @param handle The subscription handle.
 * @param header Pointer to a structure to store the message header.
 * @param data Pointer to a buffer to store the message payload.
 * @param timeout The maximum time to wait in milliseconds (or osWaitForever).
 * @return osOK if a message was received, osErrorTimeout if the timeout was reached.
 */
osStatus_t FDCAN_Receive(FDCAN_SubscriptionHandle_t handle, FDCAN_RxHeaderTypeDef *header, uint8_t *data, uint32_t timeout);

/**
 * @brief (For advanced use) Gets the underlying queue from a subscription handle.
 * This allows a single task to wait on multiple subscriptions using a QueueSet.
 * @param handle The subscription handle.
 * @return The osMessageQueueId_t for the subscription, or NULL if the handle is invalid.
 */
osMessageQueueId_t FDCAN_GetQueueFromSubscription(FDCAN_SubscriptionHandle_t handle);


/* Private functions -------------------------------------------------------------------------------------------------------------------------------------------------*/

















#endif /* INC_UM_FDCAN_LIB_H_ */
