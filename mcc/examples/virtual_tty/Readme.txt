--- Introduction ---

This example demos the MCC's ability to transfer up to 1000 charactors in one
bulk transfer.


--- Running the example ---

Similar to the pingpong example, once M4 side example starts, the following
information will be shown


    ***** MCC Virtual TTY EXAMPLE *****
    Please wait :
        1) A9 peer is ready
    Then press "S" to start the demo
    ************************************

When A9 side application runs successfully, press 'S' to kick off the M4 side
application. M4 will wait then for A9 input. Enter an arbitray sequence of
charactors in A9 and press Enter, the string will be sent to M4 via MCC. On
receiving the message, M4 will print the string as it is on its own console.
Meanwhile as an acknoledgement to A9, the same message will be sent back to A9
as well.

The output on M4 looks like the following:

    Responder task received a msg from [0, 0, 1] endpoint
    Message: Size=1000, DATA="foo"
    Responder task received a msg from [0, 0, 1] endpoint
    Message: Size=1000, DATA="bar"
    Responder task received a msg from [0, 0, 1] endpoint
    Message: Size=1000, DATA="foobar"
    ...
