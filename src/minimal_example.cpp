#include <abb_librws/rws_state_machine_interface.h>
int main() {
    abb::rws::RWSStateMachineInterface rws_state_machine_interface_  = 
        abb::rws::RWSStateMachineInterface("192.168.125.1");
    rws_state_machine_interface_.services().sg().rightCalibrate();
    
    usleep(2*1000000);
    rws_state_machine_interface_.services().sg().rightGripOut();
    
    usleep(2*1000000);
    rws_state_machine_interface_.services().sg().rightGripIn();
    return 0;
}