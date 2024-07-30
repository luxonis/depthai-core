#include <iostream>
#include <fstream>
#include "depthai/depthai.hpp"
#include "protos/messages.pb.h"


int main(int argc, char const *argv[])
{
    hunter_example::Messages messages_in;

    //try to read the message from disk if exists
    std::fstream input("outputProto.bin", std::ios::in | std::ios::binary);
    if (!input) {
      std::cout << "Fileo outputProto.bin not found. Creating a new file." << std::endl;
    } else if (!messages_in.ParseFromIstream(&input)) {
      std::cerr << "Failed to parse outputProto.bin." << std::endl;
      return -1;
    }

    // Print messages.
    for (int i = 0; i < messages_in.msgs_size(); i++)
    {
        const hunter_example::Message& msg = messages_in.msgs(i);
        std::cout << "Read line: " << msg.line() << std::endl;
    }


    hunter_example::Messages messages_out;
    messages_out.add_msgs()->set_line("Hello, World!");
    messages_out.add_msgs()->set_line("Hello, World 2!");

    // Write the message to disk.
    std::fstream output("outputProto.bin", std::ios::out | std::ios::trunc | std::ios::binary);
    if (!messages_out.SerializeToOstream(&output)) {
      std::cerr << "Failed to write to outputProto." << std::endl;
      return -1;
    }

    return 0;
}