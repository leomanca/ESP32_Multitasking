/*
 * Project: ESP32_Multitasking
 * File: main.c
 * Description:
 *   This project demonstrates the creation of two FreeRTOS tasks running on different cores
 *   of an ESP32 using the ESP-IDF framework. Task 1 runs every 10 ms, measures execution
 *   time intervals, and stores them in a buffer. After 1 second, Task 1 signals Task 2.
 *   Task 2 waits for the signal, computes minimum, maximum, and mean values of the buffer,
 *   and prints these values for debugging purposes.
 * 
 * Functionality:
 *   - Task 1:
 *     - Runs every 10 ms using the RTOS delay functions.
 *     - Stores execution time intervals in a buffer of size 100.
 *     - After 1 second, signals Task 2.
 *   - Task 2:
 *     - Waits for the signal from Task 1.
 *     - Computes minimum, maximum, and mean values from the buffer.
 *     - Prints the computed values for debugging purposes.
 * 
 * Notes:
 *   - Tasks are designed to be non-blocking to ensure efficient CPU utilization.
 *   - Minimal use of global variables to avoid threading issues.
 *   - Uses FreeRTOS synchronization mechanisms for task communication.
 * 
 * Solution:
 *   The code implements a global queue of one element (a buffer with a 100 elements capacity) to handle threading issues.
 *   The queue is implemented containing only one element, so it's either filled or not.
 *   Task 2 is signalled that it should retrieve the data from the queue when there's actual data inside of it.
 *   Due to its implementation, the queue gets automatically flushed when the data is read.
 *   Using a 1 element queue, the queue can't be written if full and can't be flushed if not full.
 *   If the queue is full, it won't be written and the current buffer will be saved and sent in the next execution.
 *   This doesn't stop the execution of Task 1 in the meantime, that will still run until 1s has passed,
 *   while at the same time Task 2 will always wait for a "full signal" from the queue.
 *  
 *   In total, for every cycle of Task 1, there should be maximum 101 recorded intervals.
 *   This is because the time intervals are computed at: 
 *      0 + ∆t, 1 + ∆t, 2 + ∆t, ... , 99 + ∆t (notice how here 1s hasn't been reached), 100 + ∆t = 101 iterations.
 *   If the period of 10ms of Task 1 is respected, then when i == 100 (101 iterations), 10ms * 100 = 1s will have passed.
 *   Thus, the buffer has been implemented circular so compensate this overflowing problem.
 *   Should delays happen, the recorded intervals will be less and the buffer will not "overflow".
 * 
 *   Timers: 
 *      last_time: records the elapsed time from the previous cycle
 *      current_time: records the time at the start of the current cycle
 *      start_time_1s: tracks if 1s has passed, so that the queue can be filled
 * 
 * Alternative Solution:
 *   Instead of having the overhead from the buffer copy in case not being sent, a binary semaphore could be introduced to directly
 *   signal Task 2 to retrieve the value if sent. After Task 2 takes the semaphore, it should be passed back to Task 1 in case the data
 *   is copied by Task 2.
 * 
 * Author: Leonardo Manca
 * Date: 14.07.2024
 * Version: 1.0
 */

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "string.h"

#define STACK_DEPTH 4096
#define TASK1_PERIOD_MS 10
#define BUFFER_SIZE 100
#define ONE_SECOND_IN_US 1000000
#define ITERATIONS_BEFORE_SYNC 2

QueueHandle_t queue;

void computeMaxMinAvg(int64_t raw_buffer[BUFFER_SIZE]){
    int64_t min = raw_buffer[0], max = raw_buffer[0], sum = 0;
    double avg;
    for(int i = 0; i < BUFFER_SIZE; i++){
        if (max < raw_buffer[i])
            max = raw_buffer[i];
        if (min > raw_buffer[i])
            min = raw_buffer[i];
        sum += raw_buffer[i];
    }
    avg = (double)sum / BUFFER_SIZE;
    printf("max: %lld us    min: %lld us   avg: %.2f us\n", max, min, avg);
}


void vTask1(void* pvParameters){
    int64_t current_time, last_time, copy_buffer[BUFFER_SIZE], buffer[BUFFER_SIZE] = {0};
    uint8_t i = 0, counter_sync = ITERATIONS_BEFORE_SYNC; //this sync counter is needed just to make sure that there are no discrepancies between start_clock and last_time
    TickType_t start_clock;
    bool buffer_not_sent = false;
    start_clock = xTaskGetTickCount();
    while(1){
        current_time = esp_timer_get_time();
        if((buffer_not_sent == true) && (xQueuePeek(queue, &buffer, 0) == pdFALSE)){ //if the buffer is empty and wasn't sent before, try to send it again
            if(xQueueSend(queue, &copy_buffer, 0) == pdTRUE) // Reattemp to send the data
                buffer_not_sent = false;
        }
        if (i == BUFFER_SIZE){ //if i reaches 100, 10ms should have elapsed
            i = 0; // The buffer is implemented as circular, so it gets overwritten if 1s hasn't elapsed yet
            if (xQueuePeek(queue, &buffer, 0) == pdTRUE){ // If this is true, the queue is not empty, thus I won't send the data
                buffer_not_sent = true; //flag to acknowledge that the buffer has not been sent
                memcpy(copy_buffer, buffer, sizeof(buffer)); // copy the current buffer on a copy buffer to send it again at the next cycle
            }
            else{
                if(xQueueSend(queue, &buffer, 0) == pdTRUE) // A queue copies all the element, so to save data, I'll just send the address of the buffer
                    buffer_not_sent = false;
            }
        }  
        if (counter_sync != 0)
            counter_sync -= 1;
        else
            buffer[i++] = current_time - last_time;
        last_time = current_time;
        vTaskDelayUntil(&start_clock, pdMS_TO_TICKS(TASK1_PERIOD_MS));
    }
    vTaskDelete(NULL);
}

void vTask2(void* pvParameters){
    int64_t received_buffer[BUFFER_SIZE];
    while(1){
        if (xQueueReceive(queue, received_buffer, portMAX_DELAY) == pdTRUE){ // Using portMAX_DELAY blocks the task's execution, making it not blocking CPU time
            computeMaxMinAvg(received_buffer);
        }
    }
    vTaskDelete(NULL);
}


void app_main(void)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS); // startup time (Wi-Fi, Bluetooth module...)
    queue = xQueueCreate(1, sizeof(int64_t)*BUFFER_SIZE); // Create the queue
    if (queue == NULL){
        printf("Error creating the queue");
        return;
    }
    if (xTaskCreatePinnedToCore(vTask1, "Task1", STACK_DEPTH, NULL, 5, NULL, 0) != pdPASS){
        printf("Task1 not created");
        return;
    }
    if (xTaskCreatePinnedToCore(vTask2, "Task2", STACK_DEPTH, NULL, 1, NULL, 1) != pdPASS){
        printf("Task2 not created");
        return;
    }
}
