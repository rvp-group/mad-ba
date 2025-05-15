#ia generic viewer buffer message. it contains all the fields of a MemoryBuffer
#ia in this way we use ros to send serialized packets containing objects and commands
#ia those can be either:
#ia   - received, deserialized and passed through a qglVieweport
#ia   - received, deserialized and converted into RViz compliant stuff

Header  header
uint64 size           #ia size in bytes
uint64 num_packets    #ia number of packets serialized in this buffer
uint8 status          #ia status of the buffer (Free=0, Read=1, Write=2, Ready=3)
uint8[] data          #ia actual buffer of stuff
