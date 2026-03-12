#include "agv/render_ipc_server.hpp"

#include <iostream>

int main() {
    agv::ipc::RenderIpcServer server;
    server.run(std::cin, std::cout);
    return 0;
}
