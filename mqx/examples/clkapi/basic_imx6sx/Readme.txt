Note for the clkapi basic demo
----------------------------------------
This demo shows basic usage of clkapi usage by manipulating. The demo has
3 phases.

The first phase is opeation on ECSPI nodes. ECSPI1 and ECSPI2 are enabled
then disabled using "clock_enable" and "clock_disable" API. ECSPIs have
a common parent node "PLL3_60". When all its childs are not active,
"PLL3_60" is not active either. When the first child becomes active, it
requires this parent to become active too. When the last child becomes
non-active, it requires this parent to become non-active either. The dump
message will shows such sequence.

>>>>>>> Dump Message
[ECSPI] 2. Now enable ECSPI1 clock
    [clk_drv] : will enable <ECSPI1>
    [clk_drv] : will enable <PLL3_60>   // parent node is enabled passively
...
[ECSPI] 5. Now disable ECSPI2 clock
	[clk_drv] : will disable <ECSPI2>
	[clk_drv] : will disable <PLL3_60>  // parent node is disabled passively
<<<<<<<

During the first phase, the clock nodes information will be dumped out before
and after each operation. Pay attention to OPEN/CLOSE status and
EN_CNT/ACTIVE_CHILD fields of each node. The changes can be summarized as the
following

[ECSPI] 2. Now enable ECSPI1 clock :
    ECSPI1 node : 
        CLOSE -> OPEN
        EN_CNT : 0 -> 1
    PLL3_60 node :
        CLOSE -> OPEN
        ACTIVE_CHILD : 0 -> 1
[ECSPI] 3. Now enable ECSPI2 clock :
    ECSPI2 node :
        CLOSE -> OPEN
        EN_CNT : 0 -> 1
    PLL3_60 node :
        ACTIVE_CHILD : 1 -> 2
[ECSPI] 4. Now disable ECSPI1 clock :
    ECSPI1 node : 
        OPEN -> CLOSE
        EN_CNT : 1 -> 0
    PLL3_60 node :
        ACTIVE_CHILD : 2 -> 1
[ECSPI] 5. Now disable ECSPI2 clock :
    ECSPI2 node :
        OPEN -> CLOSE
        EN_CNT : 1 -> 0
    PLL3_60 node :
        ACTIVE_CHILD : 1 -> 0
        OPEN -> CLOSE

* EN_CNT is the software enable count for a node, a "clock_enable" will cause
the EN_CNT plus 1, a "clock_disable" will cause the EN_CNT minus 1
* ACTIVE_CHILD is the count of node's child which becomes active, it is
passively increased/decreased as child node becomes open and close
* The first one in "EN_CNT" and "ACTIVE_CHILD" which becomes from 0 to 1 will
cause the clock node to be physically opened. The last one which becomes from
1 to 0 will cause the clock node to be physically closed

The second phase is similar to the first phase, the associated nodes are SSI1~3,
This time, no clock dump is there. Clk driver output can be examined to see how
child node and parent node works together. It indicate the same rule as phase 1

* The first one in "EN_CNT" and "ACTIVE_CHILD" which becomes from 0 to 1 will
cause the clock node to be physically opened. The last one which becomes from
1 to 0 will cause the clock node to be physically closed

Finally, all clock nodes information of Vybrid is dumped. From which we can see
each clock nodes' OPEN/CLOSE status, its frequency, its parent, its child, its
clock path.

This demo is not a real world application, instead, it is a user guide for clock
management driver user. The user of the driver can use "clock_dump" to see
detailed information of a clock node. And he can use "clock_enable" and
"clock_disable" to manipulate the end clock nodes. What he need to care is only
the end nodes of a module (For example, the end nodes for UART module is "CLK_UART"
and "CLK_UART_IPG"). He don't need to care about the clock tree hierarchy. 
The clock management driver will handle all this for him. Another advantage to use
clock tree managment driver is that the use count based model will faciliate
system driver owner to achieve low power objectives.
