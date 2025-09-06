/*
 * um_fdcan_lib.c
 *
 *  Created on: Sep 2, 2025
 *      Author: anton
 */

#include "um_fdcan_lib.h"

/**
 * @brief Initializes the FDCAN wrapper, its hardware filters, and its processing thread.
 * @param wrapper Pointer to the FDCAN_Wrapper_t object to initialize.
 * @param hfdcan Pointer to the HAL FDCAN handle (e.g., &hfdcan1).
 * @param filters An array of pre-configured FDCAN_FilterTypeDef structures.
 * @param filter_count The number of filters in the 'filters' array.
 * @param max_subscriptions The maximum number of unique software subscriptions this wrapper will support.
 * 
 * @return osOK on success, otherwise an error code.
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
                              osMutexId_t uart_mutex){
    
    // 0. PROGRAMMER ERROR CHECKS (FATAL)
    // These checks valitdate the paramters passed during initialiation
    // An error here is considered a FATAL PROGRAMMING MISTAKE, not something we can recover.
    // When testing the car, these errors should never occur. 
    // These checks here are to inmediately catch incorrect setup when developing our application
    if (wrapper == NULL || hfdcan == NULL) {
        return osErrorParameter;
    }

    if (huart_debug != NULL && uart_mutex == NULL) {
        // This is an invalid configuration, so we return an error immediately.
        return osErrorParameter;
    }

    // Check if the number of filters provided exceeds the number configured in CubeMX.
    if (std_filter_count > hfdcan->Init.StdFiltersNbr || ext_filter_count > hfdcan->Init.ExtFiltersNbr) {
        return osErrorParameter; // The user is trying to configure more filters than allocated.
    }

    // 1. Configure all the filters:
    for (uint32_t i = 0; i < std_filter_count; i++) {
        HAL_FDCAN_ConfigFilter(hfdcan, &std_filters[i]);
    }

    for (uint32_t i = 0; i < ext_filter_count; i++) {
        HAL_FDCAN_ConfigFilter(hfdcan, &ext_filters[i]);
    }

    // 2. Configure Interrupts for FDCANX
    HAL_FDCAN_ConfigInterruptLines(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);

    // 3. Configure notifications handling for the interrupts
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    // 4. Enable Tx buffer request for FDCAn to be able to transmit data if needed
    // HAL_FDCAN_EnableTxBufferRequest(hfdcan, FDCAN_TX_FIFO_OPERATION);

    // 5. Start the FDCAN peripheral
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        // Handle error
    }

    // 5.5 Log FDCAN start successful if in

    // 6. Activate Notifications if needed
    /*	HAL_FDCAN_ActivateNotification(hfdcan1,
	                               FDCAN_IT_ERROR_WARNING   |
	                               FDCAN_IT_ERROR_PASSIVE   |
	                               FDCAN_IT_BUS_OFF         |
	                               FDCAN_IT_ARB_PROTOCOL_ERROR |
	                               FDCAN_IT_DATA_PROTOCOL_ERROR,
	                               0);*/
}


  // Initialize FDCAN1 and FDCAN2
  if (HAL_FDCAN_Start(&hfdcan1) == HAL_OK ){
	  char start_msg_one[] = "[F7] CAN1 Started - Ready for communication\r\n";
	  HAL_UART_Transmit(&huart3, (uint8_t*)start_msg_one, strlen(start_msg_one), HAL_MAX_DELAY);
  };
  if (HAL_FDCAN_Start(&hfdcan2) == HAL_OK ){
	  char start_msg_two[] = "[F7] CAN2 Started - Ready for communication\r\n";
	  HAL_UART_Transmit(&huart3, (uint8_t*)start_msg_two, strlen(start_msg_two), HAL_MAX_DELAY);
	  HAL_UART_Transmit(&huart3, "\r\n", 2, HAL_MAX_DELAY);
  };


