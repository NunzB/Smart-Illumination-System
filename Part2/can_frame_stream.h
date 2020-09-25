#ifndef CAN_FRAME_STREAM_H
#define CAN_FRAME_STREAM_H

//static defines the object's lifetime during execution;
//constexpr specifies that the object should be available during compilation.
class can_frame_stream {
  private:
    static constexpr int buffsize = 5; //space for 10 can_messages - increase if needed
    can_frame cf_buffer[buffsize]; //cf_buffer array com tamanho = buffsize de variáveis do tipo can_frame
    int read_index; //where to read next message
    int write_index; //where to write next message
    bool write_lock; //buffer full

  public:
    can_frame_stream() : read_index{0}, write_index{0}, write_lock{false} {};
    
    int put(can_frame &frame) {
      if (write_lock){
        return 0; //buffer full
      }
      cf_buffer[write_index] = frame;
      write_index = (++write_index) % buffsize;
      if (write_index == read_index) write_lock = true; //cannot write more
      return 1;
    }
    
    int get(can_frame &frame) {
      if (!write_lock && (read_index == write_index) ){
        return 0; //empty buffer
      }
      if (write_lock && (read_index == write_index) ) write_lock = false; //release lock
      frame = cf_buffer[read_index];
      read_index = (++read_index) % buffsize;
      return 1;
    }
};

#endif
