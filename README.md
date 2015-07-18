# SM8_SWIM
Download program by STM8‘s SWIM
Using S3C6410

Driver ko:
    stm8_swim.c
    stm8_swim.h
    main.c

    Make them to product swim.ko, and insmod swim.ko in mini6410 board.


User cli test:
    swim_cli.c

    Using arm-linux-gcc -o swim_cli swim_cli.c to compile it.
    Run swim_cli in mini6410's shell.

