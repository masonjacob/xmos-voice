// build command
// xcc main.xc -O0 -target=XU316-1024-QF60A-C24  -o test.xe  
// run command
// xrun --xscope test.xe
// sim command
// xsim test.xe  --vcd-tracing "-o trace.vcd -tile tile[0] -ports -ports-detailed -clock-blocks -instructions -cores"

#include <platform.h>
#include <xs1.h>
#include <timer.h>
#include <stdio.h>

<<<<<<< Updated upstream
port p = XS1_PORT_4F;
=======
port p = XS1_PORT_4F ;

// int main(void) {
//   while (1) {
    p <: 10;
//     delay_milliseconds(200);
//     p <: 0;
//     delay_milliseconds(200);
//   }
//   return 0;
// }
>>>>>>> Stashed changes

int main(void) {
  while (1) {
    p <: 14;
    delay_milliseconds(200);
    p <: 0;
    delay_milliseconds(200);
  }
  return 0;
}

// int main(void) {
//   printf("Hello world!\n");
//   return 0;
// }