# UART Print with eyalroz/printf

[eyalroz/printf: Tiny, fast(ish), self-contained, fully loaded printf, sprinf etc. implementation; particularly useful in embedded systems.](https://github.com/eyalroz/printf)

Check the code under the [User](./User/) folder.

But the results of the experiment are (with `-O0`):

- eyalroz/printf: 18.45 KB
- Standard STM32 Arm stdio.h: 10.58 KB

No very detailed comparison was made.

- UART: 115200-8-n-1
- STM32CubeIDE: `v1.14.1`
