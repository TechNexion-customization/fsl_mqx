Readme for Flexcan MCC Demo
----------------------------------------
This demo is based on the flexcan example and MCC feature is added into it.

The basic functionality is the same as flexcan example. After receiving packets from another
CAN peer, other than displaying the message on M4 console, the message is also sent to A9
peer, A9 display it on its console at the same time.

There may be more complicated protocol between M4 and A9 on how to coperate on the CAN
message, here we only use a "print-as-it-like" to demo the capability.


How to Run the Demo
----------------------------------------
1. Prepare for the Linux/MQX dual core running environment.
2. Open 2 consoles, one for Linux and one for MQX.
3. Reboot the system. Linux u-boot will kick off MQX.
4. On MQX console there will be prompt message as the following

        ***** MCC FLEXCAN EXAMPLE *****                                                 
        Please wait :                                                                   
            1) A9 peer is ready                                                         
            2) CAN peer is ready                                                        
        Then press "S" to start the demo                                                
        *******************************                                                 
                                                                                        
        Press "S" to start the demo :

   The A9 peer means the Linux OS running on A9. Wait for A9 peer ready means we must wait
   until Linux OS finished running, and the MCC demo on Linux side is activated. Please
   refer to Linux How-To to make this ready.

   CAN peer means another CAN host. It can be either a CAN Probe or another IMX6SX board
   running the CAN demo. Please refer to the How-To in "mqx\examples\can\flexcan" to make
   this ready

5. After both the A9 and the CAN peer becomes ready. Press "S" on the M4 console to start
   the demo

Expected Result
----------------------------------------
The M4 console will print the can message received from the CAN peer, these messages are
sent to A9 via MCC. A9 will print these messages on A9 console.
The content of the CAN communication should be the same on M4 console and M9 console.
