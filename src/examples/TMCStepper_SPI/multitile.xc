#include <platform.h>


typedef chanend chanend_t;

extern "C" {
  void main_tile0(chanend_t);
  void main_tile1(chanend_t);
}

int main(void)
{
  chan c;

  par {
    on tile[0]: main_tile0(c);
    on tile[1]: main_tile1(c);
  }

  return 0;
}