#include <xpcc/processing/rtos_abstraction.hpp>
#include <ch.h>

namespace xpcc {
  IRQWrapper::IRQWrapper() {
    CH_IRQ_PROLOGUE();
  }
  //epilogue
  IRQWrapper::~IRQWrapper() {
    CH_IRQ_EPILOGUE();
  }
}
