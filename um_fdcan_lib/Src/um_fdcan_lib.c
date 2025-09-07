#include "um_fdcan_lib.h"
#include <string.h> // For memset and strlen
#include <stdio.h>  // For sprintf

// NOTAS:
// Revisar el uso de osMessageQueuePut en HAl_FDCAN_RxFifo0Callback y HAL_FDCAN_RxFifo1Callback
// Revisar / rehacer HAL_FDCAN_ErrorStatusCallback -> explicar tambien el tema de los errores de status @defgroup FDCAN_Error_Status_Interrupts FDCAN Error Status Interrupts
// Revisar fdcan_processing_task y el tema de los mensajes 
// Revisar Init // Central queue for ISR-to-task communication

/* Private types ------------------------------------------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief Internal representation of a single message subscription.
 * This structure is hidden from the user.
 */
struct FDCAN_Subscription_Internal {
    uint32_t message_id;            /**< The CAN message ID for this subscription. */
    osMessageQueueId_t message_queue; /**< The queue to hold received messages for this subscription. */
    uint8_t is_active;              /**< Flag to indicate if this subscription slot is in use. */
};

/**
 * @brief Structure to hold a complete FDCAN message for queuing.
 * This is the data type that will be sent to subscriber queues.
 */
typedef FDCAN_Message_t FDCAN_QueuedMessage_t;

/* Private variables --------------------------------------------------------------------------------------------------------------------------------------------------*/
// Global registry for wrapper instances
static FDCAN_Wrapper_t* g_fdcan_wrappers[MAX_FDCAN_INSTANCES] = {NULL};
static uint8_t g_wrapper_count = 0;

/* Private function prototypes ----------------------------------------------------------------------------------------------------------------------------------------*/
static FDCAN_Wrapper_t* get_wrapper(FDCAN_HandleTypeDef *hfdcan);
static void fdcan_processing_task(void *argument); // This is the prototype for the task function so that the compiler knows about it before use. We will be defining it later. However, we need to declare it here so that the compiler knows about it before we use it in FDCAN_Wrapper_Init.
static void dispatch_message(FDCAN_Wrapper_t* wrapper, FDCAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data);

static void debug_print(FDCAN_Wrapper_t* wrapper, const char* message, int message_length);


/* HAL Callback implementations ---------------------------------------------------------------------------------------------------------------------------------------*/
/** @defgroup UM_FDCAN_LIB_CALLBACKS HAL Callback Implementations
 *  @ingroup um_fdcan_lib
 *  @brief Implementations of weak HAL FDCAN callbacks overridden by the library.
 *
 *  These functions are called by the STM32 HAL driver in response to FDCAN interrupts.
 *  Their primary role is to capture the event type and quickly pass it to the appropriate
 *  wrapper's processing task via a queue, keeping the time spent in the ISR to an absolute minimum.
 * 
 *  We have the global registry to be able to find the wrapper instance in the HAL callbacks
 *  This is important because the HAL callbacks do not provide different callbacks for different FDCAN instances
 *  By registering the wrapper here, we can find it later in the callbacks and handle the callbacks appropriately for each fdcan peripheral instance
 *
 *  @{
 */

/**
  * @brief  Rx FIFO 0 New Message callback. This function is called by the HAL driver when a new message arrives in FIFO0.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signaled.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_Wrapper_t* wrapper = get_wrapper(hfdcan);
    if (wrapper == NULL || wrapper->processing_thread_handle == NULL) {
        return; // Wrapper not found or not fully initialized
    }

    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        // Signal the processing task that a message is ready in FIFO0.
        // This is a lightweight and safe way to notify from an ISR.
        osThreadFlagsSet(wrapper->processing_thread_handle, FDCAN_FLAG_RX_FIFO0);
    }

    // I may have to update this callback to handle other interrupts, like FIFO full so that I can use it when in busmonitoring mode
}

/**
  * @brief  Rx FIFO 1 New Message callback. This function is called by the HAL driver when a new message arrives in FIFO1.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure.
  * @param  RxFifo1ITs: indicates which Rx FIFO 1 interrupts are signaled.
  * @retval None
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    FDCAN_Wrapper_t* wrapper = get_wrapper(hfdcan);
    if (wrapper == NULL || wrapper->processing_thread_handle == NULL) {
        return; // Wrapper not found or not fully initialized
    }

    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        // Signal the processing task that a message is ready in FIFO1.
        osThreadFlagsSet(wrapper->processing_thread_handle, FDCAN_FLAG_RX_FIFO1);
    }

    // I may have to update this callback to handle other interrupts, like FIFO full so that I can use it when in busmonitoring mode
}

/**
  * @brief  Error Status callback. This is called by the HAL driver on error status changes, including Bus Off.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure.
  * @param  ErrorStatusITs: indicates which Error Status interrupts are signaled.
  * @retval None
  * 
  * @note Check FDCAN_Error_Status_Interrupts group in the documentation for more details.
  *       Additionally, you may want to check HAL_FDCAN_IRQHandler in stm32h7xx_hal_fdcan.c to see how the HAL handles these interrupts.
  *       As a summary, HAL only calls this callback for bus-off events, error warning or error passive states, which are defined in the FDCAN_Error_Status_Interrupts group.
  */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    FDCAN_Wrapper_t* wrapper = get_wrapper(hfdcan);
    if (wrapper == NULL || wrapper->processing_thread_handle == NULL) {
        return;
    }

    // Check if the Bus Off interrupt is active
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
        // Notify the processing task to handle the Bus Off event.
        osThreadFlagsSet(wrapper->processing_thread_handle, FDCAN_FLAG_BUS_OFF);
    }
    
    // Check for Error Passive state
    if ((ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) != RESET) {
        osThreadFlagsSet(wrapper->processing_thread_handle, FDCAN_FLAG_ERROR_PASSIVE);
    }

    // Other error statuses (Error Warning) can be handled here if needed.
    // Check FDCAN_Error_Status_Interrupts group in the documentation for more details.
}

/**
  * @brief  Error callback. This is called by the HAL driver on HAL_FDCAN_IRQHandler
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure.
  * @retval None
  * 
  * @note Check FDCAN_Error_Interrupts group in the documentation for more details.
  *       Additionally, you may want to check HAL_FDCAN_IRQHandler in stm32h7xx_hal_fdcan.c to see how the HAL handles these interrupts.
  *       As a summary, HAL only calls this callback for Arbitration and Data phase protocol errors, 
  *       RAM access failures, Reserved Address Access, and Message RAM Watchdog events, which are defined in the FDCAN_Error_Interrupts group.
  */

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan){
    
    FDCAN_Wrapper_t* wrapper = get_wrapper(hfdcan);
    if (wrapper == NULL || wrapper->processing_thread_handle == NULL) {
        return;
    }

    uint32_t error_code = HAL_FDCAN_GetError(hfdcan);

    // Check for any protocol error (arbitration, form, stuff, etc.)
    if ((error_code & FDCAN_E_PROTOCOL) != 0) {
        osThreadFlagsSet(wrapper->processing_thread_handle, FDCAN_FLAG_ERROR_PROTOCOL);
    }

    // You could add more specific flags for other error types if needed,
    // for example, for RAM access failure (FDCAN_E_RAM_ACCESS).
}

/** @} */ // End of UM_FDCAN_LIB_CALLBACKS group

/* Private functions --------------------------------------------------------------------------------------------------------------------------------------------------*/
/** @defgroup UM_FDCAN_LIB_PRIVATE_FUNCTIONS Private Functions
 *  @ingroup um_fdcan_lib
 *  @brief Internal helper functions for the FDCAN library.
 *
 *  These functions are not part of the public API and should not be called directly by user code.
 *  These functions are for internal use ONLY
 *  @{
 */

/**
 * @brief Finds the FDCAN_Wrapper_t instance associated with a given HAL handle.
 * @param hfdcan Pointer to the HAL FDCAN handle.
 * @return Pointer to the corresponding FDCAN_Wrapper_t, or NULL if not found.
 */
static FDCAN_Wrapper_t* get_wrapper(FDCAN_HandleTypeDef *hfdcan)
{
    for (uint8_t i = 0; i < g_wrapper_count; i++) {
        if (g_fdcan_wrappers[i] != NULL && g_fdcan_wrappers[i]->hfdcan == hfdcan) {
            return g_fdcan_wrappers[i];
        }
    }
    return NULL; // Should not happen if Init is called correctly
}

/**
 * @brief Dispatches a received FDCAN message to the appropriate subscriber.
 * This function iterates through the subscription list and, if a match is found,
 * queues the message for the subscriber's task.
 * @param wrapper Pointer to the FDCAN_Wrapper_t object.
 * @param rx_header Pointer to the header of the received message.
 * @param rx_data Pointer to the data payload of the received message.
 */
static void dispatch_message(FDCAN_Wrapper_t* wrapper, FDCAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data)
{
    if (osMutexAcquire(wrapper->subscription_list_mutex, 10) != osOK) {
        // Failed to get mutex.
        // It's better to drop the message than to risk a deadlock or data corruption.
        return;
    }

    for (uint32_t i = 0; i < wrapper->subscription_count; i++) {
        // Check if the subscription is active and the ID matches.
        // We use Identifier instead of FilterIndex because we are matching the exact ID.
        if (wrapper->subscription_list[i].is_active && wrapper->subscription_list[i].message_id == rx_header->Identifier) {
            
            FDCAN_QueuedMessage_t msg_to_queue;
            msg_to_queue.header = *rx_header;
            memcpy(msg_to_queue.data, rx_data, rx_header->DataLength); // we copy only the actual data length thanks to the DLC frame

            // Put the message into the subscriber's dedicated queue.
            // Use a timeout of 0 to avoid blocking the processing task.
            if (osMessageQueuePut(wrapper->subscription_list[i].message_queue, &msg_to_queue, 0U, 0U) == osErrorResource)
            {
                // The queue is full. Overwrite the oldest message.
                FDCAN_QueuedMessage_t dummy; // Dummy variable to receive the discarded message.
                // 1. Remove the oldest message from the front of the queue to make space.
                osMessageQueueGet(wrapper->subscription_list[i].message_queue, &dummy, NULL, 0U);
                // 2. Try again to put the new message at the back of the queue. This should now succeed.
                osMessageQueuePut(wrapper->subscription_list[i].message_queue, &msg_to_queue, 0U, 0U);
            }

            // Assuming one message ID has only one subscriber.
            // If multiple subscribers for the same ID are allowed, remove this break.
            break; 
        }
    }

    osMutexRelease(wrapper->subscription_list_mutex);
}

/**
 * @brief The main processing task for an FDCAN wrapper.
 * This task waits for notifications from ISRs and processes them.
 * @param argument Pointer to the FDCAN_Wrapper_t object.
 */
static void fdcan_processing_task(void *argument)
{
    FDCAN_Wrapper_t* wrapper = (FDCAN_Wrapper_t*)argument;
    uint32_t flags;

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[MAX_FDCAN_DATA_LENGTH]; // Max FDCAN data size is 64 bytes

    uint8_t peripheral_number = 0;
    if (wrapper->hfdcan == FDCAN1) { peripheral_number = 1; }
    else if (wrapper->hfdcan == FDCAN2) { peripheral_number = 2; }
    else if (wrapper->hfdcan == FDCAN3) { peripheral_number = 3; }

    char debug_message[75];

    while (1)
    {
        // Wait indefinitely for any notification flag from an ISR

        flags = osThreadFlagsWait(FDCAN_FLAG_RX_FIFO0 | FDCAN_FLAG_RX_FIFO1 | FDCAN_FLAG_BUS_OFF | FDCAN_FLAG_ERROR_PROTOCOL_ARBITRATION | FDCAN_FLAG_ERROR_PASSIVE, osFlagsWaitAny, osWaitForever)

        // Check for Bus Off flag
        if (flags & FDCAN_FLAG_BUS_OFF)
        {
            // Handle Bus Off state by attempting to restart the peripheral.
            if (HAL_FDCAN_Stop(wrapper->hfdcan) == HAL_OK) {

                if (wrapper->debug_target_initialized)
                {
                    sprintf(debug_message, "FDCAN%d Bus-Off. Attempting recovery.\r\n", peripheral_number);
                    debug_print(wrapper, debug_message, strlen(debug_message));
                }
                
                // Small delay before attempting to recover communication.
                osDelay(100);

                if (HAL_FDCAN_Start(wrapper->hfdcan) == HAL_OK) {
                    // Log recovery if debug is enabled
                     if (wrapper->debug_target_initialized)
                     {
                        sprintf(debug_message, "FDCAN%d recovery successful\r\n", peripheral_number);
                         debug_print(wrapper, debug_message, strlen(debug_message));
                     }
                }
            }
        }

        // Check for FIFO0 new message flag
        if (flags & FDCAN_FLAG_RX_FIFO0)
        {
            // Drain the FIFO: process all messages currently in it.
            while (HAL_FDCAN_GetRxFifoFillLevel(wrapper->hfdcan, FDCAN_RX_FIFO0) > 0)
            {
                if (HAL_FDCAN_GetRxMessage(wrapper->hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
                {
                    dispatch_message(wrapper, &rx_header, rx_data);
                }
                else
                {
                    // Error getting message, break to avoid infinite loop
                    break;
                }
            }
        }

        // Check for FIFO1 new message flag
        if (flags & FDCAN_FLAG_RX_FIFO1)
        {
            // Drain the FIFO
            while (HAL_FDCAN_GetRxFifoFillLevel(wrapper->hfdcan, FDCAN_RX_FIFO1) > 0)
            {
                if (HAL_FDCAN_GetRxMessage(wrapper->hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK)
                {
                    dispatch_message(wrapper, &rx_header, rx_data);
                }
                else
                {
                    break;
                }
            }
        }

        if (flags & FDCAN_FLAG_ERROR_PROTOCOL_ARBITRATION)
        {
            if (wrapper->debug_target_initialized)
                {
                    sprintf(debug_message, "FDCAN%d Arbitration Error.\r\n", peripheral_number);
                    debug_print(wrapper, debug_message, strlen(debug_message));
                }
            FDCAN_ProtocolStatusTypeDef protocol_status;
            FDCAN_ErrorCountersTypeDfef error_counters;

            if (HAL_FDCAN_GetProtocolStatus(wrapper->hfdcan, &protocol_status) == HAL_OK &&
                HAL_FDCAN_GetErrorCounters(wrapper->hfdcan, &error_counters) == HAL_OK)
            {
                if (wrapper->debug_target_initialized){
                    switch (protocol_status.LastErrorCode){
                        case FDCAN_PROTOCOL_ERROR_NONE: { 
                            sprintf(debug_message, "FDCAN%d No Error.\r\n", peripheral_number);
                            debug_print(wrapper, debug_message, strlen(debug_message));
                            break;
                        }
                        case FDCAN_PROTOCOL_ERROR_STUFF: {
                            sprintf(debug_message, "FDCAN%d Stuff Error.\r\n", peripheral_number);
                            debug_print(wrapper, debug_message, strlen(debug_message));
                            break;
                        } 
                        
                        case FDCAN_PROTOCOL_ERROR_FORM: {
                            sprintf(debug_message, "FDCAN%d Form Error.\r\n", peripheral_number);
                            debug_print(wrapper, debug_message, strlen(debug_message));
                            break;
                        }

                        case FDCAN_PROTOCOL_ERROR_ACK: {
                            sprintf(debug_message, "FDCAN%d ACK Error. No other nodes, bad termination...\r\n", peripheral_number);
                            debug_print(wrapper, debug_message, strlen(debug_message));
                            break;
                        }
                        
                        case FDCAN_PROTOCOL_ERROR_BIT0: {
                            sprintf(debug_message, "FDCAN%d Bit0 Error. Signal Integrity, EMI, termination\r\n", peripheral_number);
                            debug_print(wrapper, debug_message, strlen(debug_message));
                            break;
                        }
                        case FDCAN_PROTOCOL_ERROR_BIT1: {
                            sprintf(debug_message, "FDCAN%d Bit1 Error. Signal Integrity, EMI, termination\r\n", peripheral_number);
                            debug_print(wrapper, debug_message, strlen(debug_message));
                            break;
                        }
                        case FDCAN_PROTOCOL_ERROR_CRC: {
                            sprintf(debug_message, "FDCAN%d CRC Error. Possible Timing mismatch between nodes \r\n", peripheral_number);
                            debug_print(wrapper, debug_message, strlen(debug_message));
                            break;
                        }

                        case FDCAN_PROTOCOL_ERROR_NO_CHANGE: {
                            sprintf(debug_message, "FDCAN%d No Change Error.\r\n", peripheral_number);
                            debug_print(wrapper, debug_message, strlen(debug_message));
                            break;
                        }
                        default:
                            break;            
                    }
                    
                }
            }
        }

        if (flags & FDCAN_FLAG_ERROR_PASSIVE)
        {
            if (wrapper->debug_target_initialized)
            {
                sprintf(debug_message, "FDCAN%d Error Passive.\r\n", peripheral_number);
                debug_print(wrapper, debug_message, strlen(debug_message));
            }
        }
    }
}

static void debug_print(FDCAN_Wrapper_t* wrapper, const char* message, int message_length){
    if (wrapper->debug_target_initialized)
    {
        osMutexAcquire(wrapper->uart_mutex, osWaitForever);
        HAL_UART_Transmit(wrapper->huart_debug, (uint8_t*)message, message_length, HAL_MAX_DELAY);
        osMutexRelease(wrapper->uart_mutex);
    }
}

/* Public Functions  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/** @} */ // End of UM_FDCAN_LIB_PRIVATE_FUNCTIONS group

/** @addtogroup UM_FDCAN_LIB_PUBLIC_FUNCTIONS
 *  @brief Functions intended for direct use by the application.
 *  @{
 */

/**
 * @brief Initializes the FDCAN wrapper, its hardware filters, and its processing thread.
 */
osStatus_t FDCAN_Wrapper_Init(FDCAN_Wrapper_t *wrapper,
                              FDCAN_HandleTypeDef *hfdcan,
                              const FDCAN_FilterTypeDef* std_filters,
                              uint32_t std_filter_count,
                              const FDCAN_FilterTypeDef* ext_filters,
                              uint32_t ext_filter_count)
{
    // 0. PROGRAMMER ERROR CHECKS (FATAL)
    // These checks valitdate the paramters passed during initialiation
    // An error here is considered a FATAL PROGRAMMING MISTAKE, not something we can recover.
    // When testing the car, these errors should never occur. 
    // These checks here are to inmediately catch incorrect setup when developing our application
    if (wrapper == NULL || hfdcan == NULL || g_wrapper_count >= MAX_FDCAN_INSTANCES) {
        return osErrorParameter;
    }
    if (wrapper->debug_target == FDCAN_DEBUG_UART && (wrapper->huart_debug == NULL || wrapper->uart_mutex == NULL)) {
        // This is an invalid configuration, so we return an error immediately.
        return osErrorParameter;
    }

    if (std_filter_count > hfdcan->Init.StdFiltersNbr || ext_filter_count > hfdcan->Init.ExtFiltersNbr) {
        return osErrorParameter; // More filters than configured in hfdcan.
    }

    if (wrapper->debug_target == FDCAN_DEBUG_UART && !wrapper->debug_target_initialized) {
        // Initialize the UART debug target only once
        if (wrapper->huart_debug == NULL || wrapper->uart_mutex == NULL) {
            return osErrorParameter; // Invalid UART configuration
        }
        wrapper->debug_target_initialized = true;
    }

    if (wrapper->debug_target == FDCAN_DEBUG_UART && wrapper->huart_debug != NULL && wrapper->uart_mutex != NULL) {
        wrapper->debug_target_initialized = true;
    }
    else {
        wrapper->debug_target_initialized = false;
    }



    // 1. Register the wrapper instance in the global registry
    // We have the global registry to be able to find the wrapper instance in the HAL callbacks
    // This is important because the HAL callbacks do not provide different callbacks for different FDCAN instances
    // By registering the wrapper here, we can find it later in the callbacks and handle the callbacks appropriately for each fdcan peripheral instance
    
    memset(wrapper, 0, sizeof(FDCAN_Wrapper_t));
    g_fdcan_wrappers[g_wrapper_count++] = wrapper;
    wrapper->hfdcan = hfdcan;

    // 2. Create RTOS objects
    // Mutex for protecting the subscription list, as to avoid race conditions
    // We need this mutex because the subscription list can be modified by user tasks (when subscribing/unsubscribing) and it can be read by the internal thread when parsing the data to signal messages
    const osMutexAttr_t sub_mutex_attr = { .name = "fdcan_sub_mutex" };
    wrapper->subscription_list_mutex = osMutexNew(&sub_mutex_attr);
    if (wrapper->subscription_list_mutex == NULL) {
        return osErrorResource;
    }

    // Mutex for protecting the TX peripheral access
    const osMutexAttr_t tx_mutex_attr = { .name = "fdcan_tx_mutex" };
    wrapper->tx_mutex = osMutexNew(&tx_mutex_attr);
    if (wrapper->tx_mutex == NULL) {
        // Cleanup previously created mutex
        osMutexDelete(wrapper->subscription_list_mutex);
        return osErrorResource;
    }

    // 3. Configure hardware filters
    for (uint32_t i = 0; i < std_filter_count; i++) {
        HAL_FDCAN_ConfigFilter(hfdcan, (FDCAN_FilterTypeDef*)&std_filters[i]);
    }
    for (uint32_t i = 0; i < ext_filter_count; i++) {
        HAL_FDCAN_ConfigFilter(hfdcan, (FDCAN_FilterTypeDef*)&ext_filters[i]);
    }

    // 4. Configure and activate interrupts  based on CUBE IDE
    // Activate FIFO new message interrupts if the corresponding FIFOs are used
    if (hfdcan->Init.RxFifo0ElmtsNbr > 0) {
        HAL_FDCAN_ConfigInterruptLines(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
    if (hfdcan->Init.RxFifo1ElmtsNbr > 0) {
        HAL_FDCAN_ConfigInterruptLines(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, FDCAN_INTERRUPT_LINE1);
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    }

    // 4.5 I used to have HAL_FDCAN_EnableTxBufferRequest(hfdcan), but I think it's not needed

    // 5. Activate error status interrupts (including Bus Off)
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE, FDCAN_E_ALL);

    // 5. Start the FDCAN peripheral
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return osError; // Indicate a hardware start failure
    }

    // 6. Create the dedicated processing task for this FDCAN instance
    char task_name[16]; // Name the task based on the initialization order (e.g., "fdcan_task_1", "fdcan_task_2")
    int peripheral_number = 0;
    
    if (hfdcan->Instance == FDCAN1) {
        sprintf(task_name, "fdcan1_task");
        peripheral_number = 1;
    } else if (hfdcan->Instance == FDCAN2) {
        sprintf(task_name, "fdcan2_task");
        peripheral_number = 2;
    } else if (hfdcan->Instance == FDCAN3) {
        sprintf(task_name, "fdcan3_task");
        peripheral_number = 3;
    } else {
        sprintf(task_name, "fdcan_task_%d", g_wrapper_count); // Fallback naming
        peripheral_number = g_wrapper_count;
    }

    const osThreadAttr_t task_attr = {
        .name = task_name,
        .stack_size = INTERNAL_THREAD_STACK_SIZE, // Stack size in bytes
        .priority = INTERNAL_THREAD_PRIORITY,
    };

    wrapper->processing_thread_handle = osThreadNew(fdcan_processing_task, wrapper, &task_attr);
    // Note: it is important to note that we pass 'wrapper' as the argument to the task function
    // this way the task has access to all the context it needs.

    if (wrapper->processing_thread_handle == NULL) {
        HAL_FDCAN_Stop(hfdcan); // Clean up on failure
        return osErrorResource;
    }

    // 7. Log successful initialization if debug is enabled
    if (wrapper->debug_target_initialized){
        char msg[64];
        sprintf(msg, "FDCAN Wrapper Initialized Successfully: FDCAN%d.\r\n", peripheral_number);
        debug_print(wrapper, msg, strlen(msg));
    }

    return osOK;
}

FDCAN_SubscriptionHandle_t FDCAN_Subscribe(FDCAN_Wrapper_t *wrapper, uint32_t message_id)
{
    if (wrapper == NULL) {
        return NULL;
    }

    if (osMutexAcquire(wrapper->subscription_list_mutex, osWaitForever) != osOK) {
        return NULL; // Failed to get mutex
    }

    // Find the first inactive subscription slot
    int sub_index = -1;
    for (uint32_t i = 0; i < MAX_SUBSCRIPTIONS_PER_WRAPPER; i++) {
        if (!wrapper->subscription_list[i].is_active) {
            sub_index = i;
            break;
        }
    }

    if (sub_index == -1) {
        // No available subscription slots
        osMutexRelease(wrapper->subscription_list_mutex);
        return NULL;
    }

    // Configure the subscription
    struct FDCAN_Subscription_Internal* sub = &wrapper->subscription_list[sub_index];
    // We create a temporary variable to shorten the lenght of the access to the pointer
    sub->message_id = message_id;
    sub->is_active = 1;

    // Create a dedicated message queue for this subscription
    // The queue will hold full FDCAN messages.
    sub->message_queue = osMessageQueueNew(SUBSCRIBER_QUEUE_DEPTH, sizeof(FDCAN_QueuedMessage_t), NULL);

    if (sub->message_queue == NULL) {
        // Failed to create queue, roll back
        sub->is_active = 0;
        osMutexRelease(wrapper->subscription_list_mutex);
        return NULL;
    }
    
    wrapper->subscription_count++;

    osMutexRelease(wrapper->subscription_list_mutex);

    // Return the handle (pointer to the internal struct)
    return (FDCAN_SubscriptionHandle_t)sub;
}

osStatus_t FDCAN_Unsubscribe(FDCAN_Wrapper_t *wrapper, FDCAN_SubscriptionHandle_t handle)
{
    if (wrapper == NULL || handle == NULL) {
        return osErrorParameter;
    }

    if (osMutexAcquire(wrapper->subscription_list_mutex, osWaitForever) != osOK) {
        return osErrorResource; // Could not acquire mutex
    }

    // Find the subscription in the wrapper's list to validate the handle
    int sub_index = -1;
    for (uint32_t i = 0; i < MAX_SUBSCRIPTIONS_PER_WRAPPER; i++) {
        // Check if the pointer address matches and if it's currently active
        if (&wrapper->subscription_list[i] == (struct FDCAN_Subscription_Internal*)handle) {
            if (wrapper->subscription_list[i].is_active) {
                sub_index = i;
            }
            break; // Found the handle, break either way
        }
    }

    // If the handle was not found in our list or was already inactive, it's an error
    if (sub_index == -1) {
        osMutexRelease(wrapper->subscription_list_mutex);
        return osErrorParameter;
    }

    // Now we know it's a valid, active subscription we can safely remove it
    struct FDCAN_Subscription_Internal* sub = &wrapper->subscription_list[sub_index];

    // 1. Mark as inactive
    sub->is_active = 0;

    // 2. Delete the message queue
    if (sub->message_queue != NULL) {
        osMessageQueueDelete(sub->message_queue);
        sub->message_queue = NULL; // Good practice
    }
    
    // 3. Clear the message ID
    sub->message_id = 0;

    // 4. Decrement the total count of active subscriptions
    wrapper->subscription_count--;

    osMutexRelease(wrapper->subscription_list_mutex);

    return osOK;
}


osStatus_t FDCAN_Receive(FDCAN_SubscriptionHandle_t subscription_handle, FDCAN_Message_t *message, uint32_t timeout)
{
    if (subscription_handle == NULL || message == NULL) {
        return osErrorParameter;
    }

    // The handle is a pointer to the internal subscription struct.
    struct FDCAN_Subscription_Internal* sub = (struct FDCAN_Subscription_Internal*)subscription_handle;

    // Check if the subscription is still active and has a valid queue.
    // This is a safety check in case the user calls Receive after unsubscribing.
    if (!sub->is_active || sub->message_queue == NULL) {
        return osErrorParameter;
    }

    // Wait for a message to arrive in the subscription's queue.
    // The osMessageQueueGet function will block the calling task until a message is available or the timeout occurs.
    return osMessageQueueGet(sub->message_queue, message, NULL, timeout);
}

osStatus_t FDCAN_Transmit(FDCAN_Wrapper_t *wrapper, const FDCAN_TxMessage_t *message)
{
    if (wrapper == NULL || message == NULL) {
        return osErrorParameter;
    }

    // Acquire the mutex to ensure exclusive access to the TX FIFO.
    if (osMutexAcquire(wrapper->tx_mutex, osWaitForever) != osOK) {
        return osErrorResource; // Failed to get mutex
    }

    // Check if there is space in the TX FIFO before attempting to add a message.
    // This prevents HAL_FDCAN_AddMessageToTxFifo from blocking indefinitely if the FIFO is full.
    while (HAL_FDCAN_GetTxFifoFreeLevel(wrapper->hfdcan) == 0)
    {
        // Yield the processor if the TX FIFO is full, allowing other tasks to run.
        osDelay(1); 
    }

    osStatus_t status = osOK;
    if (HAL_FDCAN_AddMessageToTxFifo(wrapper->hfdcan, (FDCAN_TxHeaderTypeDef*)&message->header, (uint8_t*)message->data) != HAL_OK) {
        status = osError; // Transmission failed
    }

    // Release the mutex as soon as the operation is complete.
    osMutexRelease(wrapper->tx_mutex);

    return status;
}


/** @} */ // End of UM_FDCAN_LIB_PUBLIC_FUNCTIONS group

