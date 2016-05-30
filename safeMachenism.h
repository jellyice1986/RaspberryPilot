
#define MAX_COUNTER 40 // 2 Sec, time to achieve MAX_COUNTER is TRANSMIT_TIMER * MAX_COUNTER
#define MIN_COUNTER 0

void initPacketAccCounter();
void decreasePacketAccCounter();
void increasePacketAccCounter();
int getPacketAccCounter();
void resetPacketAccCounter();


