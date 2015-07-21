--- Introduction ---

This simple example shows MCC communication between two Endpoints (EP), 
each of them created on different core. Endpoints are receive buffer 
queues, implemented in shared memory, and are addressed by a triplet 
containing core, node, and port:

   Core : Identifies the core within the processor. In case of 
          i.MX6SX platform, the A9 is core 0, and the M4 is 
          core 1.
   Node : In Linux any user process participating in MCC is a unique 
          node. Node numbering is arbitrary. MQX has only one node
          and can also be an arbitrary number.
   Port : All OSes can have an arbitrary number of ports per node 
          (up to a configurable maximum), arbitrarily numbered with the 
          exception of port 0 (MCC_RESERVED_PORT_NUMBER) being reserved.

In case of i.MX6SX platform the A9 core is the sender and the M4 core is
the responder. The following EPs are defined on the application level:
    A9 core: mqx_endpoint_a5 = {0, MCC_MQX_NODE_A5, MCC_MQX_SENDER_PORT}
    M4 core: mqx_endpoint_m4 = {1, MCC_MQX_NODE_M4, MCC_MQX_RESPONDER_PORT}
Here we keep the names as a5 for historical reasons. The code is inherited
from other socs which has A5 as M4 peer, so we keep the name here. Put in
mind that it acturally means A9 port in i.MX6 projects.

The application running on both core first initializes the MCC (if not 
already done by the other core), then it checks that both cores are 
using the same version of the MCC library and finally creates 
particular EPs on each core.

Once the receiver EP is created the sender initiates sending simple 
messages to the receiver EP. The message contains just a counter value, 
starting with the value of 1. Each time the receiver EP receives a new 
message, it increments the counter value and sends the message back to 
the sender. This sequence repeats on each EP/Core and the ¡°message 
pingpong¡± continues forever, unless a receive error occurs.
You should see outputs of both applications on the console (default 
console for each core).

--- Running the example ---
Start a terminal applications for both of the 2 cores then start the example
for both cores. After starting M4 side applications, you will see prompt
messages as the following


    ***** MCC PINGPANG EXAMPLE *****                                                
    Please wait :                                                                   
        1) A9 peer is ready                                                         
    Then press "S" to start the demo                                                
    ********************************                                                
                                                                                    
    Press "S" to start the demo :

After A9 side MCC conterpart is running, press the "S" key to let M4
application run, you will see the following messages indicate the MCC
data ping pong are successfully running.

    Responder task received a msg                                                   
    Message: Size=4, DATA = 1                                                       
    Responder task received a msg                                                   
    Message: Size=4, DATA = 3                                                       
    Responder task received a msg                                                   
    Message: Size=4, DATA = 5                                                       
    Responder task received a msg                                                   
    Message: Size=4, DATA = 7                                                       
    Responder task received a msg                                                   
    Message: Size=4, DATA = 9                                                       
    Responder task received a msg                                                   
    Message: Size=4, DATA = b                                                       
    Responder task received a msg                                                   
    Message: Size=4, DATA = d               
    .....

--- Explanation of the example ---
The flow of the tasks is described in the next figure. There is the
main task that runs on one core (A9) and the responder task running on 
the other core (M4). These tasks are exchanging messages (pingpong) 
between EPs created on both sides/cores.

            +-----------+                                   +----------------+       
            | Main Task |                                   | Responder Task |       
            +-----------+                                   +----------------+       
                  |                                                 |                
            Initialize MCC                                    Initialize MCC         
                  |                                                 |                
         Checks the MCC Version                            Checks the MCC Version    
                  |                                                 |                
          Creates A9 Endpoint                             Creates mqx_endpoint_m4    
       [0,0, MCC_MQX_SENDER_PORT]                       [1,0, MCC_MQX_RECEIVER_PORT] 
                  |                                                 |                
   +------------->+                                  +------------->+                
   |              |                                  |              |                
   |      Sends a message to                         |      Receives a message       
   |         m4 endpoint                             |              |                
   |              |                                  |           message             
   |        mqx_endpoint_m4                          |          received             
   |           exists?                               |        successfully?          
   |    (N)       |                                  |    (N)       |                
   |<-------------+                                  |<-------------+                
   |              | (Y)                              |              | (Y)            
   |              |                                  |              |                
   |    Receives the response                        |   Increments the counter      
   |              |                                  |              |                
   |           message                               |     Sends a message to        
   |          received                               |        a9 endpoint            
   |        successfully?                            |              |                
   |    (N)       |                                  |<-------------+                
   |<-------------+
   |              | (Y)
   |              |
   |      Increments the counter
   |              |
   |<-------------+

In i.MX6SX project, only the Responder Task is running, it is the A9's
responsibility to run the Main Task.
