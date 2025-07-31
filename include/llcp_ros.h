#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/timer_handler.h>


#include <mrs_modules_msgs/msg/llcp.hpp>


#include <mrs_serial/serial_port.h>

extern "C"{
    #include "llcp.h"
}

#include <stdlib.h>
#include <string>
#include <thread>

#define SERIAL_BUFFER_SIZE 1024


/* class MrsLlcpRos //{ */

class MrsLlcpRos : public rclcpp::Node {

public:
    MrsLlcpRos(rclcpp::NodeOptions options);

private:
    struct msg_counter_t {
        uint8_t id;
        int num;
    };

    struct msg_stats_t {
        uint8_t id;
        int num;
        float avrg_per_s;
        float last_s;
        float time;
        int last_num_;
    };

    // | ----------------------- init ------------------------------------------------------------ |
    
    rclcpp::Node::SharedPtr  node_;
    rclcpp::Clock::SharedPtr clock_;

    rclcpp::TimerBase::SharedPtr timer_preinitialization_;
    void                         timerPreInitialization();
    void initialize();
    std::atomic<bool> is_initialized_ = false;

    // | ----------------------- node parameters ------------------------------------------------- |

    /* String like "/dev/ttyACM0 */
    std::string portname_;
    int         baudrate_;
    /* Enables printing more informations */
    bool        pretty_log_;

    // | ----------------------- subscribers ----------------------------------------------------- |
    
    mrs_lib::SubscriberHandler<mrs_modules_msgs::msg::Llcp> sh_llcp_tx_;

    // | ----------------------- publishers ------------------------------------------------------ |
    
    mrs_lib::PublisherHandler<mrs_modules_msgs::msg::Llcp> ph_llcp_rx_;
    
    // | ----------------------- timers ---------------------------------------------------------- |
    
    std::shared_ptr<TimerType> timer_connection_;
    void                       timerConnection();
    double                     timer_connection_rate_ = 1.0;

    std::shared_ptr<TimerType> timer_statistics_;
    void                       timerStatistics();
    double                     timer_statistics_rate_ = 1.0;


    // | ----------------------- serial port ----------------------------------------------------- |
    
    LLCP_Receiver_t llcp_receiver_;
    serial_port::SerialPortThreadsafe serial_port_;
    std::thread serial_thread_;

    bool running_     = true;
    bool initialized_ = false;
    bool connected_   = false;
    std::mutex mutex_connected_; // guards connected_
    std::vector<msg_counter_t> received_msgs_;
    std::mutex mutex_received_msgs_; // guards received_msgs_
    std::vector<msg_counter_t> sent_msgs_;
    std::mutex mutex_sent_msgs_; // guards sent_msgs_
    std::vector<msg_stats_t> received_msgs_stats_;
    std::vector<msg_stats_t> sent_msgs_stats_;

    void serialThreadRx(void);
    void sendLlcpMessage(const mrs_modules_msgs::msg::Llcp::ConstSharedPtr msg);
    void connectToSerial(void);
    void callbackSendMessage(void);
    bool openSerialPort(std::string portname, int baudrate);

    // | ---------------------- other member functions ------------------------------------------- |

    void printStatistics(void);
    void updateStatistics(void);
};
