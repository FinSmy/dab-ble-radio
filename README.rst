Rough Plan
=======
- Believe that overwriting the pointer to rx_data every time is causing corrupt data in FIFO
  - To resolve this, allocate memory for FIFO on the stack
- Understand if the heap space is big enough - taking into account memory slab and anything else
- Otherwise, it seems that FIFO is being emptied faster than it's being written - is this because i2s is completely asynchronous and just chomps through?
- Try to make use of i2s_buf_write which should directly write to memory slab for us
