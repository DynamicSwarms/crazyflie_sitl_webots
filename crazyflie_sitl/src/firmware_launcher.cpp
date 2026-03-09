#include "crazyflie_sitl/firmware_launcher.hpp"

#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>

#include <vector>
#include <string>
#include <ostream>
#include <iostream>
FirmwareLauncher::FirmwareLauncher(
    std::string& sitl_socket_path, 
    std::string& client_socket_path)
{
    m_firmware_pid = fork();

    if (m_firmware_pid == 0) 
    {
        setpgid(0,0); //Create new process group so that we can kill all child processes later       
        
        std::vector<char*> argv;

        argv.push_back(const_cast<char*>("ros2"));
        argv.push_back(const_cast<char*>("run"));
        argv.push_back(const_cast<char*>("crazyflie_sitl"));
        argv.push_back(const_cast<char*>("cf2"));

        argv.push_back(const_cast<char*>("unix"));
        argv.push_back(const_cast<char*>(sitl_socket_path.c_str()));
        argv.push_back(const_cast<char*>(client_socket_path.c_str()));
        argv.push_back(nullptr);
        
        execvp("ros2", argv.data());
        _exit(1);
    }
}

FirmwareLauncher::~FirmwareLauncher()
{
    if (m_firmware_pid > 0)
    {
        
        kill(-m_firmware_pid, SIGINT); //Kill all processes in the process group
        waitpid(m_firmware_pid, nullptr, 0); //Wait for the firmware process to exit
        m_firmware_pid = -1;
    }
}
    
