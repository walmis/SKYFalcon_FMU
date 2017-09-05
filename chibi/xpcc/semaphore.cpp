/*
 * semaphore.h
 *
 *  Created on: Oct 12, 2014
 *      Author: walmis
 */

 #include <xpcc/processing/rtos_abstraction.hpp>
 #include <ch.h>
 #include <chbsem.h>

namespace xpcc {

  void Semaphore::give() {
    binary_semaphore_t* bsem = (binary_semaphore_t*)semaphore;
    syssts_t s = chSysGetStatusAndLockX();
    chBSemSignalI(bsem);
    chSysRestoreStatusX(s);
  }

  bool Semaphore::take(uint32_t timeout_ms) {
    binary_semaphore_t* bsem = (binary_semaphore_t*)semaphore;

    if(timeout_ms == TIME_INFINITE)
      return chBSemWaitTimeout(bsem, TIME_INFINITE) == MSG_OK;

    return chBSemWaitTimeout(bsem, MS2ST(timeout_ms)) == MSG_OK;
  }

  bool Semaphore::take_nonblocking() {
    binary_semaphore_t* bsem = (binary_semaphore_t*)semaphore;
    return chBSemWaitTimeout(bsem, TIME_IMMEDIATE) == MSG_OK;
  }

  bool Semaphore::taken() {
    binary_semaphore_t* bsem = (binary_semaphore_t*)semaphore;
    syssts_t s = chSysGetStatusAndLockX();
    bool state = chBSemGetStateI(bsem);
    chSysRestoreStatusX(s);
    return state;
  }

  Semaphore::Semaphore() {
    binary_semaphore_t* bsem = new binary_semaphore_t;
    chBSemObjectInit(bsem, false);
    semaphore = (void*)bsem;
  }

  Semaphore::~Semaphore() {
    binary_semaphore_t* bsem = (binary_semaphore_t*)semaphore;
    delete bsem;
  }

}
