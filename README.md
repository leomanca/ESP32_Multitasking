# ESP32_Multitasking

## Create two RTOS tasks running on different cores

### Task one functionality:
- This task should run every 10 ms using the RTOS delay functions

- This task has a buffer of size 100 holding the time interval for each execution measured by the integrated ESP timing functions

- After 1 s the task should send the second task a signal getting the content of the buffer
### Task two functionality:
- This task waits for the signal of the first task
  
- If signalized, the task computes minimum, maximum and mean of the values
from the buffer of the first task

- After computation, print out the values for debugging purposes

### Constraints:
- Tasks are not blocking CPU time
  
- Use as little as possible global variables
  
- Avoid threading issues

### Solution:
- The solution uses a queue consisting only of one element which is the size of the buffer itself
- Task 1 (Producer) can only write to the queue, while Task 2 (Consumer) can only read from it. Since the queue can contain only one element, if the queue is filled, it can't be written again by Task 1, it must be first consumed by Task 2, this avoids threading issues while allowing at the same time to signal Task 2 with xQueueReceive() if the queue is filled. This approach uses only 1 global variable.
- xQueueReceive() takes as parameter portMAX_DELAY, which allows the Task to be blocked until the queue is filled, allowing not to block CPU time.
