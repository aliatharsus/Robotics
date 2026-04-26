# Robotics

## CANBUS Communication
### Iteration 1:
Focused on basic can communication based on youtube tutorials and making stuff work
### Iteration 2:
Focused on bi-directional communication using the RTR. Can be scaled for reading raw wheel positions or PID parameters on startup

## PID velocity Control

### Incremental Encoder

** This part has 2 iterations posted here **

-> First Iteration is done via interrupts and CPU reading and counting the encoder ticks manually. This is OK for smaller resolution encoders, but will backfire when scaled to higher PPR encoders.

-> Second Iteration is done first for USB based debugging. Use the HTML+JS tool for tuning any new motor that needs to be used. Set various velocity targets to acheive the goal. This can be used for Zeigler-nichols PID tuning by finding the critical gains, and adjusting the parameters accordingly.

-> Second Iteration is also tailored to be used as a slave card code, that gets its instructions from the CANBUS master. This will be tailored for bidirectional communication in the future IA.

### Absolute Magnetic Encoder

** Pending **