/*------------------------------------------------------------------------*/
/* Sample code of OS dependent controls for FatFs                         */
/* (C)ChaN, 2012                                                          */
/*------------------------------------------------------------------------*/

#include <stdlib.h>		/* ANSI memory controls */
#include <malloc.h>		/* ANSI memory controls */
#include <ff.h>

#if _FS_REENTRANT
#include <ch.h>
#include <chbsem.h>
/*------------------------------------------------------------------------*/
/* Create a Synchronization Object                                        */
/*------------------------------------------------------------------------*/
int ff_cre_syncobj(BYTE vol, _SYNC_t *sobj) {

  *sobj = malloc(sizeof(semaphore_t));
  chSemObjectInit(*((semaphore_t**)sobj), 1);
  return TRUE;
}

/*------------------------------------------------------------------------*/
/* Delete a Synchronization Object                                        */
/*------------------------------------------------------------------------*/
int ff_del_syncobj(_SYNC_t sobj) {

  free(sobj);
  return TRUE;
}

/*------------------------------------------------------------------------*/
/* Request Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
int ff_req_grant(_SYNC_t sobj) {

  msg_t msg = chSemWaitTimeout((semaphore_t*)sobj, (systime_t)_FS_TIMEOUT);
  return msg == MSG_OK;
}

/*------------------------------------------------------------------------*/
/* Release Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
void ff_rel_grant(_SYNC_t sobj) {

  chSemSignal((semaphore_t*)sobj);
}
#endif /* _FS_REENTRANT */





#if _USE_LFN == 3	/* LFN with a working buffer on the heap */
/*------------------------------------------------------------------------*/
/* Allocate a memory block                                                */
/*------------------------------------------------------------------------*/
/* If a NULL is returned, the file function fails with FR_NOT_ENOUGH_CORE.
*/

void* ff_memalloc (	/* Returns pointer to the allocated memory block */
	UINT msize		/* Number of bytes to allocate */
)
{
	return malloc(msize);
}


/*------------------------------------------------------------------------*/
/* Free a memory block                                                    */
/*------------------------------------------------------------------------*/

void ff_memfree (
	void* mblock	/* Pointer to the memory block to free */
)
{
	free(mblock);
}

#endif
