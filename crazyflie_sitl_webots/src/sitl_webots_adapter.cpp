#include "rclcpp/rclcpp.hpp"

#include "crazyflie_sitl_webots/webots_driver/webots_crazyflie_driver.hpp"


class SitlWebotsAdapter : public rclcpp::Node
{
public:
    SitlWebotsAdapter()
    : Node("sitl_webots_adapter")
    , m_id(declare_parameter("id", rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
    , m_webots_port(declare_parameter("webots_port", rclcpp::ParameterValue(1234), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
    , m_webots_use_tcp(declare_parameter("webots_use_tcp", rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<bool>())
    , m_webots_tcp_ip(declare_parameter("webots_tcp_ip", rclcpp::ParameterValue("127.0.0.1"), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<std::string>())
    , m_wb_driver(std::make_shared<WebotsCrazyflieDriver>(m_id, m_webots_port, m_webots_use_tcp, m_webots_tcp_ip))
    {
        RCLCPP_INFO(this->get_logger(), "SITL Webots Adapter Node has been started.");

        m_webots_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto timer_callback = [this]() -> void {
            m_wb_driver->step();
        };

        m_webots_step_timer = this->create_wall_timer(
            std::chrono::milliseconds(10), // 100 Hz = 10 ms
            std::bind(&SitlWebotsAdapter::webots_step_timer_callback, this),
            m_webots_callback_group
        );

        // Launch a separate thread to run the webots_step_timer_callback in a loop
        //m_webots_thread = std::make_shared<std::thread>([this]() {
        //    while (rclcpp::ok()) {
        //        webots_step_timer_callback();
        //    }
        //});
//
        //// Detach the thread to allow it to run independently
        //m_webots_thread->detach();
    }

    ~SitlWebotsAdapter()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down SITL Webots Adapter Node.");
        //m_webots_thread->join();
        if (m_wb_driver) {
            m_wb_driver.reset();
        }
    }

    void webots_step_timer_callback()
    {
        //RCLCPP_INFO(this->get_logger(), "Current time step: %d", m_wb_driver->get_time_step());
        m_wb_driver->get_time_step();
        if (m_wb_driver) {
            auto start_time = std::chrono::high_resolution_clock::now();
            if (!m_wb_driver->step()) {
                m_wb_driver.reset();
                RCLCPP_INFO(get_logger(), "Webots simulation ended, shutting down Crazyflie node.");
                //this->shutdown();
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
            RCLCPP_INFO(this->get_logger(), "Step function execution time: %ld microseconds", duration);
        }
    }



private: 
    uint8_t m_id; 

    int m_webots_port;
    bool m_webots_use_tcp; 
    std::string m_webots_tcp_ip; 


    std::shared_ptr<WebotsCrazyflieDriver> m_wb_driver;    
    std::shared_ptr<rclcpp::CallbackGroup> m_webots_callback_group;
    std::shared_ptr<rclcpp::TimerBase> m_webots_step_timer;

    std::shared_ptr<std::thread> m_webots_thread;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SitlWebotsAdapter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}