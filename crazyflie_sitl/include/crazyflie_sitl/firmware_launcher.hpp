#pragma once
#include <thread>

class FirmwareLauncher {

public:
    FirmwareLauncher(
        std::string& sitl_socket_path, 
        std::string& client_socket_path);

    ~FirmwareLauncher();
    
private: 
    pid_t m_firmware_pid;     

};

