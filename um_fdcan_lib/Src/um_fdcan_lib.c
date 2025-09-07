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
typedef struct {
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[MAX_FDCAN_DATA_LENGTH];
} FDCAN_QueuedMessage_t;

/* Private variables --------------------------------------------------------------------------------------------------------------------------------------------------*/
// Global registry for wrapper instances
static FDCAN_Wrapper_t* g_fdcan_wrappers[MAX_FDCAN_INSTANCES] = {NULL};
static uint8_t g_wrapper_count = 0;

/* Private function prototypes ----------------------------------------------------------------------------------------------------------------------------------------*/
static FDCAN_Wrapper_t* get_wrapper(FDCAN_HandleTypeDef *hfdcan);
static void fdcan_processing_task(void *argument); // This is the prototype for the task function so that the compiler knows about it before use. We will be defining it later. However, we need to declare it here so that the compiler knows about it before we use it in FDCAN_Wrapper_Init.
static void dispatch_message(FDCAN_Wrapper_t* wrapper, FDCAN_RxHeaderTypeDef* rx_header, uint8_t* rx_data);

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
    if (wrapper == NULL || wrapper->isr_to_task_queue == NULL) {
        return; // Wrapper not found or not fully initialized
    }

    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        // To keep the ISR short, we only send a notification to the processing task.
        // The task will then retrieve the full message from the FIFO.
        uint32_t notification = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
        osMessageQueuePut(wrapper->isr_to_task_queue, &notification, 0U, 0U);
    }
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
    if (wrapper == NULL || wrapper->isr_to_task_queue == NULL) {
        return; // Wrapper not found or not fully initialized
    }

    if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        uint32_t notification = FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
        osMessageQueuePut(wrapper->isr_to_task_queue, &notification, 0U, 0U);
    }
}

/**
  * @brief  Error Status callback. This is called by the HAL driver on error status changes, including Bus Off.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure.
  * @param  ErrorStatusITs: indicates which Error Status interrupts are signaled.
  * @retval None
  */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    FDCAN_Wrapper_t* wrapper = get_wrapper(hfdcan);
    if (wrapper == NULL || wrapper->isr_to_task_queue == NULL) {
        return;
    }

    // Check if the Bus Off interrupt is active
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
        // Notify the processing task to handle the Bus Off event.
        uint32_t notification = FDCAN_IT_BUS_OFF;
        osMessageQueuePut(wrapper->isr_to_task_queue, &notification, 0U, 0U);
    }
    // Other error statuses (Error Warning, Error Passive) can be handled here if needed.
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
        // Failed to get mutex, maybe log an error.
        // It's better to drop the message than to risk a deadlock or data corruption.
        return;
    }

    for (uint32_t i = 0; i < wrapper->subscription_count; i++) {
        // Check if the subscription is active and the ID matches.
        // We use Identifier instead of FilterIndex because we are matching the exact ID.
        if (wrapper->subscription_list[i].is_active && wrapper->subscription_list[i].message_id == rx_header->Identifier) {
            
            FDCAN_QueuedMessage_t msg_to_queue;
            msg_to_queue.header = *rx_header;
            memcpy(msg_to_queue.data, rx_data, rx_header->DataLength);

            // Put the message into the subscriber's dedicated queue.
            // Use a small timeout (or 0) to avoid blocking the processing task.
            osMessageQueuePut(wrapper->subscription_list[i].message_queue, &msg_to_queue, 0U, 0U);

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
    uint32_t notification;
    osStatus_t status;

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[MAX_FDCAN_DATA_LENGTH]; // Max FDCAN data size is 64 bytes

    while (1)
    {
        // Wait indefinitely for a notification from an ISR
        status = osMessageQueueGet(wrapper->isr_to_task_queue, &notification, NULL, osWaitForever);
        if (status == osOK)
        {
            if (notification == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
            {
                // New message in FIFO0, retrieve it
                if (HAL_FDCAN_GetRxMessage(wrapper->hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
                {
                    dispatch_message(wrapper, &rx_header, rx_data);
                }
            }
            else if (notification == FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
            {
                // New message in FIFO1, retrieve it
                if (HAL_FDCAN_GetRxMessage(wrapper->hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK)
                {
                    dispatch_message(wrapper, &rx_header, rx_data);
                }
            }
            else if (notification == FDCAN_IT_BUS_OFF)
            {
                // Handle Bus Off state by attempting to restart the peripheral.
                // This is a common recovery strategy.
                if (HAL_FDCAN_Stop(wrapper->hfdcan) == HAL_OK) {
                    if (HAL_FDCAN_Start(wrapper->hfdcan) == HAL_OK) {
                        // Log recovery if debug is enabled
                         if (wrapper->debug_target == FDCAN_DEBUG_UART && wrapper->huart_debug != NULL) {
                            char msg[] = "FDCAN Bus Off detected. Recovery successful.\r\n";
                            osMutexAcquire(wrapper->uart_mutex, osWaitForever);
                            HAL_UART_Transmit(wrapper->huart_debug, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                            osMutexRelease(wrapper->uart_mutex);
                        }
                    }
                }
            }
        }
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
    const osMutexAttr_t mutex_attr = { .name = "fdcan_sub_mutex" };
    wrapper->subscription_list_mutex = osMutexNew(&mutex_attr);
    if (wrapper->subscription_list_mutex == NULL) {
        return osErrorResource;
    }

    // Central queue for ISR-to-task communication
    const osMessageQueueAttr_t queue_attr = { .name = "fdcan_isr_queue" };
    wrapper->isr_to_task_queue = osMessageQueueNew(ISR_QUEUE_DEPTH, sizeof(uint32_t), &queue_attr);
    if (wrapper->isr_to_task_queue == NULL) {
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
    if (wrapper->debug_target == FDCAN_DEBUG_UART && wrapper->huart_debug != NULL && wrapper->uart_mutex != NULL){
        char msg[64];
        sprintf(msg, "FDCAN Wrapper Initialized Successfully: FDCAN%d.\r\n", peripheral_number);
        osMutexAcquire(wrapper->uart_mutex, osWaitForever);
        HAL_UART_Transmit(wrapper->huart_debug, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        osMutexRelease(wrapper->uart_mutex);
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

/**
 * @brief Waits to receive a message for a specific subscription. This is a blocking function.
 * @param handle The subscription handle.
 * @param header Pointer to a structure to store the message header.
 * @param data Pointer to a buffer to store the message payload.
 * @param timeout The maximum time to wait in milliseconds (or osWaitForever).
 * @return osOK if a message was received, osErrorTimeout if the timeout was reached.
 */
osStatus_t FDCAN_Receive(FDCAN_SubscriptionHandle_t handle, FDCAN_RxHeaderTypeDef *header, uint8_t *data, uint32_t timeout);



/** @} */ // End of UM_FDCAN_LIB_PUBLIC_FUNCTIONS group

