/* Error messages:

SD Card Mounted Successfully
Writing file: /hello.txt
Appending to file: /hello.txt
.Reading file: /lognum.txt
Writing file: /lognum.txt
Writing file: /FN0052robot.txt
SD write task created successfully on Core 0
Setup complete
SD Write Task started on core: 0
LocalBufLen=0 0.929, 0.0, 0.0, 0.0,   0,   0,0.00,   0, 0.0
LocalBufLen=46 1.180, 0.0, 0.0, 0.0,   0,   0,0.00,   0, 0.0
LocalBufLen=92 1.430, 0.0, 0.0, 0.0,   0,   0,0.00,   0, 0.0
LocalBufLen=138 1.680, 0.0, 0.0, 0.0,   0,   0,0.00,   0, 0.0
...
LocalBufLen=791 5.231,15.2, 1.0, 0.0,1500,1500,15.00,   0,10.0
LocalBufLen=838 5.484,15.2, 1.0, 0.0,1500,1500,15.00,   0,-16.9
Appending to file: /FN0052robot.txt
Guru Meditation Error: Core  0 panic'ed (LoadStoreAlignment). Exception was unhandled.

Core  0 register dump:
PC      : 0x400904d4  PS      : 0x00060933  A0      : 0x8008e325  A1      : 0x3ffd7400  
A2      : 0x6568434d  A3      : 0xb33fffff  A4      : 0x0000abab  A5      : 0x00060923  
A6      : 0x00060920  A7      : 0x0000cdcd  A8      : 0x0000cdcd  A9      : 0xffffffff  
A10     : 0x00000003  A11     : 0x00060923  A12     : 0x00060920  A13     : 0x3ffc4654  
A14     : 0x25e8434d  A15     : 0x003fffff  SAR     : 0x00000020  EXCCAUSE: 0x00000009  
EXCVADDR: 0x6568434d  LBEG    : 0x4008ab41  LEND    : 0x4008ab51  LCOUNT  : 0xfffffffb  


Backtrace: 0x400904d1:0x3ffd7400 0x4008e322:0x3ffd7440 0x400d6820:0x3ffd7480 0x400d8a3b:0x3ffd74b0 0x400d8ded:0x3ffd74e0 0x401030be:0x3ffd7510 0x401033c5:0x3ffd7530 0x4010535f:0x3ffd7570 0x40106c63:0x3ffd77e0 0x400e65aa:0x3ffd7950 0x40149c45:0x3ffd7970 0x400d750f:0x3ffd7990 0x400d6b49:0x3ffd7a10 0x400d2f2a:0x3ffd7a40 0x400d30b1:0x3ffd7a80


*/