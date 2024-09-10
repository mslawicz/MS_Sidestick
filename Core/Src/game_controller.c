#include "game_controller.h"
#include "cmsis_os.h"

/* game controller loop
    upon reception of an event, the function prepares the controll data
    for the host and sends the appriopriate report */
void gameControllerLoop(void)
{
  while(1)
  {
    osDelay(100);   //XXX test
  }
}  