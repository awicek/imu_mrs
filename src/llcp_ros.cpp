#include "llcp_ros.h"


const std::string MrsLlcpRos::received_msgs_label_ = "Received Messages";
const std::string MrsLlcpRos::sent_msgs_label_ = "Sent Messages";

/*MrsLlcpRos::MrsLlcpRos() //{ */

MrsLlcpRos::MrsLlcpRos(rclcpp::NodeOptions options) :
    Node("llcp_ros", options)
{
  timer_preinitialization_ = create_wall_timer(
        std::chrono::duration<double>(1.0),
        std::bind(&MrsLlcpRos::timerPreInitialization, this));
}

//}

/*timerPreInitialization() //{*/

void MrsLlcpRos::timerPreInitialization()
{    
    node_  = this->shared_from_this();
    clock_ = node_->get_clock();
    
    initialize();

    timer_preinitialization_->cancel();
}

//}

/* onInit() //{ */

void MrsLlcpRos::initialize()
{
    RCLCPP_INFO(node_->get_logger(), "Initializing MrsLlcpRos...");

    // | ----------------------- parameters ------------------------------------------------------ |

    mrs_lib::ParamLoader param_loader(node_, "llcp_ros");

    param_loader.addYamlFileFromParam("config_private");

    param_loader.loadParam("portname", portname_);
    param_loader.loadParam("baudrate", baudrate_);
    param_loader.loadParam("statistic_period_s", stat_period_s_);

    if (!param_loader.loadedSuccessfully()) {
        RCLCPP_ERROR(node_->get_logger(), "[Ros Llcp]: Could not load all parameters!");
        rclcpp::shutdown();
        exit(1);
    }
    
    // | ----------------------- subscribers ----------------------------------------------------- |
    
    mrs_lib::SubscriberHandlerOptions shopts;
    shopts.node               = node_;
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;

    sh_llcp_tx_ = mrs_lib::SubscriberHandler<mrs_modules_msgs::msg::Llcp>(shopts, "~/llcp_out",
            &MrsLlcpRos::sendLlcpMessage, this);

    // | ----------------------- publishers ------------------------------------------------------ |
    
    ph_llcp_rx_ = mrs_lib::PublisherHandler<mrs_modules_msgs::msg::Llcp>(node_, "~/llcp_in");

    // | --------------------- serial port ------------------------------------------------------- |
    
    serial_port_.set_node(node_);
    connectToSerial();
    initialized_ = true;
    
    // | ----------------------- timers ---------------------------------------------------------- |
    
    mrs_lib::TimerHandlerOptions timer_opts_start;

    timer_opts_start.node      = node_;
    timer_opts_start.autostart = true;

    {
        std::function<void()> timer_connection_cb = std::bind(
            &MrsLlcpRos::timerCbConnection, this);
        timer_connection_ = std::make_shared<TimerType>(timer_opts_start,
            rclcpp::Rate(timer_connection_rate_, clock_), timer_connection_cb);
    }

    if (stat_period_s_ > 0)
    {
        timer_statistics_rate_ = 1.0 / stat_period_s_;
        std::function<void()> timer_statistics_cb = std::bind(
            &MrsLlcpRos::timerCbStatistics, this);
        timer_statistics_ = std::make_shared<TimerType>(timer_opts_start, 
            rclcpp::Rate(timer_statistics_rate_, clock_), timer_statistics_cb);
    }


    RCLCPP_INFO(node_->get_logger(), "MrsLlcpRos sucessfully initialized.");
}

//}

/* connectToSerial //{ */

void MrsLlcpRos::connectToSerial()
{
    if (serial_thread_.joinable())
        serial_thread_.join();


    bool tmp_connected = false;
    while (!tmp_connected)
    {
        tmp_connected = openSerialPort(portname_, baudrate_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    {
        std::scoped_lock lock(mutex_connected_);
        connected_ = true;
        llcp_initialize(&llcp_receiver_);
    }

    serial_thread_ = std::thread(&MrsLlcpRos::serialThreadRx, this);
}

//}

/* openSerialPort() //{ */

bool MrsLlcpRos::openSerialPort(std::string portname, int baudrate)
{
    RCLCPP_INFO(node_->get_logger(), "Opening serial port %s at baudrate %d", portname.c_str(),
        baudrate); 

    if (!serial_port_.connect(portname, baudrate)) {
        RCLCPP_ERROR(node_->get_logger(), "Could not connect to the serial port %s at baudrate %d",
                portname.c_str(), baudrate);
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Connected to sensor on port %s at baudrate %d", 
        portname.c_str(), baudrate);
    
    return true;
}

//}

/* timerCbConnection() //{ */

void MrsLlcpRos::timerCbConnection()
{
    RCLCPP_DEBUG(node_->get_logger(), "Maintainer timer called");

    bool connected;
    {
        std::scoped_lock lock(mutex_connected_);
        connected = connected_;
    }

    if (connected)
    {
        if (!serial_port_.checkConnected())
        {
            {
                std::scoped_lock lock(mutex_connected_);
                connected_ = false;
            }
            RCLCPP_ERROR(node_->get_logger(), "Serial device has disconnected!");
            connectToSerial();
            {
                std::scoped_lock lock(mutex_connected_);
                connected_ = true;
            }
        }
    }
    else 
    {
        connectToSerial();
    }
}

//}

/* printStatistic() //{ */

void MrsLlcpRos::printStatistic(std::vector<msg_stats_t> &stats, std::mutex &stat_mtx,
    const std::string &label)
{
    RCLCPP_INFO(this->get_logger(), "------------------- %s -------------------", label.c_str());
    RCLCPP_INFO(this->get_logger(), " ID |  Count | Avg/s  | Last Δ/s");
    RCLCPP_INFO(this->get_logger(), "----+--------+--------+-----------");

    {
        std::scoped_lock lock(stat_mtx);
        for (const auto &stat : stats)
        {
            RCLCPP_INFO(this->get_logger(), "%3d | %6d | %6.2f | %7.2f",
            stat.id, stat.num, stat.avrg_per_s, stat.last_s);
        }
    }

    RCLCPP_INFO(this->get_logger(), "------------------------------------------------------");
}

//}

/* updateStatistic() //{ */

void MrsLlcpRos::updateStatistic(std::vector<msg_counter_t> &msgs, std::vector<msg_stats_t> &stats,
        std::mutex & stat_mtx)
{
    std::scoped_lock lock(stat_mtx);

    for (const auto &msg : msgs)
    {
        auto it = std::find_if(stats.begin(), stats.end(),
        [&](const msg_stats_t &stat) { return stat.id == msg.id; });
        
        if (it == stats.end())
        {
            msg_stats_t stat;
            stat.id = msg.id;
            stat.num = msg.num;
            stat.avrg_per_s = static_cast<float>(msg.num) *
                              static_cast<float>(timer_statistics_rate_);
            stat.last_num_ = msg.num;
            stat.time = 1 / timer_statistics_rate_;
            stat.last_s = static_cast<float>(msg.num) / 
                          static_cast<float>(timer_statistics_rate_);
            stats.push_back(stat);
        }
        else
        {
            it->last_s = static_cast<float>(msg.num - it->last_num_) * 
                         static_cast<float>(timer_statistics_rate_);
            it->num = msg.num;
            it->time += (1 / timer_statistics_rate_);
            it->avrg_per_s = static_cast<float>(msg.num) / it->time;
            it->last_num_ = msg.num;
        }
    }
}

//}

/* timerCbStatistics() //{ */

void MrsLlcpRos::timerCbStatistics()
{
    {
        std::scoped_lock lock(mutex_connected_);
        if (! connected_)
            return;
    }

    updateStatistic(received_msgs_, received_msgs_stats_, mutex_received_msgs_);
    updateStatistic(sent_msgs_, sent_msgs_stats_, mutex_sent_msgs_);
    printStatistic(received_msgs_stats_, mutex_received_msgs_, received_msgs_label_); 
    printStatistic(sent_msgs_stats_, mutex_sent_msgs_, sent_msgs_label_); 
};

//}

/* serialThreadRx() //{ */
// TODO: Do it in blocking way, not spinning
void MrsLlcpRos::serialThreadRx(void)
{
    uint8_t rx_buffer[SERIAL_BUFFER_SIZE];
    int bytes_read;

    RCLCPP_INFO(this->get_logger(), "Serial thread starting");

    while(running_)
    {
        bool connected;
        {
            std::scoped_lock lock(mutex_connected_);
            connected = connected_;
        }
    
        if (!connected)
        {
            RCLCPP_WARN(this->get_logger(),
            "Terminating serial thread because the serial port was disconnected");
            resetStatistics();
            return;
        }

        bytes_read = serial_port_.readSerial(rx_buffer, SERIAL_BUFFER_SIZE);
        if (bytes_read > 0 && bytes_read < SERIAL_BUFFER_SIZE) 
        {
            /**
             *  If the serial device is disconnected and readSerial() is called,
             *  it will return max_int number of read bytes, thats why we check it against the
             *  SERIAL_BUFFER_SIZE
             */
            
            RCLCPP_DEBUG(this->get_logger(), 
            "Read %d bytes from serial port",
                    bytes_read);
            
            std::stringstream ss;
            for (int i = 0; i < bytes_read; ++i) {
                ss << std::hex << std::setw(2) << std::setfill('0') << (int)(uint8_t)rx_buffer[i] << " ";
            }
            RCLCPP_DEBUG(this->get_logger(), "Data: %s", ss.str().c_str());
            

            for (uint16_t i = 0; i < bytes_read; i++)
            {
                LLCP_Message_t *message_in;

                bool checksum_matched = false;
                if (llcp_processChar(rx_buffer[i], &llcp_receiver_, &message_in,
                        &checksum_matched))
                {
                    RCLCPP_DEBUG(this->get_logger(), 
                        "Received message id = %d  size %d checksum is: %d",
                        message_in->payload[0], llcp_receiver_.payload_size, checksum_matched);
          
                    mrs_modules_msgs::msg::Llcp ros_msg; 
                    ros_msg.stamp = clock_->now();
                    ros_msg.checksum_matched = checksum_matched;
                    ros_msg.id = message_in->payload[0];
                    ros_msg.payload = std::vector<uint8_t>(message_in->payload,
                                            message_in->payload + llcp_receiver_.payload_size);

                    ph_llcp_rx_.publish(ros_msg);

                    addMsgToStatistic(message_in->payload[0], received_msgs_, mutex_received_msgs_);
                }
            }   
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

//}

/* sendLlcpMessage() //{ */

void MrsLlcpRos::sendLlcpMessage(const mrs_modules_msgs::msg::Llcp::ConstSharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Sending message id: %d, len %ld",
            msg->id, msg->payload.size());
    
    bool connected;
    {
        std::scoped_lock lock(mutex_connected_);
        connected = connected_;
    }
    if (!connected)
    {
        RCLCPP_ERROR(this->get_logger(), "Can not send llcp message. Port %s not connected.", 
                portname_.c_str());
        return;
    }

    if (!initialized_)
    {
        RCLCPP_ERROR(this->get_logger(), "Can not send llcp message. Node not initialized.");
        return;
    }

    uint8_t out_buffer[SERIAL_BUFFER_SIZE];
    uint16_t msg_len = llcp_prepareMessage((uint8_t *)msg->payload.data(),
            (uint8_t)msg->payload.size(), out_buffer);

    if (! serial_port_.sendCharArray(out_buffer, msg_len))
        RCLCPP_ERROR(this->get_logger(), "Erorr during sending of the llcp message.");

    addMsgToStatistic(msg->id, sent_msgs_, mutex_sent_msgs_);
}

//}

/* resetStatistics() //{ */

void MrsLlcpRos::resetStatistics(void)
{
    {
        std::scoped_lock lock(mutex_received_msgs_);
        received_msgs_.clear();
        received_msgs_stats_.clear();
    }
    {
        std::scoped_lock lock(mutex_sent_msgs_);
        sent_msgs_.clear();
        sent_msgs_stats_.clear();
    }
}

//}

/* addMsgToStatistic() //{ */

void MrsLlcpRos::addMsgToStatistic(uint8_t msg_id, std::vector<msg_counter_t> &stat,
         std::mutex &stat_mtx)
{
    std::scoped_lock lock(stat_mtx);

     auto it = std::find_if(stat.begin(), stat.end(),
                    [&](const struct msg_counter_t& msg) {
                    return msg.id == msg_id;
                    });
    if (it != stat.end())
    {
        it->num++;
    }
    else
    {
        msg_counter_t tmp;
        tmp.id  = msg_id;
        tmp.num = 1;
        stat.push_back(tmp);
    }
}

//}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MrsLlcpRos)
