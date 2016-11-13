#define MAX_COUNTER 40 
#define MIN_COUNTER 0

void securityMechanismInit();
void decreasePacketCounter();
void increasePacketCounter();
int getPacketCounter();
void resetPacketCounter();
void triggerSecurityMechanism();

