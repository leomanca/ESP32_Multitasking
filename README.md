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
