#pragma once

#include <sstream>
#include <string>

#include <boost/function.hpp>
#include <ros/ros.h>

#include <xwidgets/xbutton.hpp>
#include <xwidgets/xbox.hpp>
#include <xwidgets/xtextarea.hpp>

#include "ros1_init.hpp"

/**
 * @brief ROS Subscriber with widgets for jupyter notebook
 * 
 * @tparam Arg 
 */
template <class Arg>
class ZeusSubscriber {
private:
    ros::Subscriber sub;
    int sub_queue_size = 10;

    xw::vbox box;
    xw::button button;
    xw::textarea textarea;

    bool append = false;

    using CbType = void(Arg);
    std::function<CbType> _cb;

public:
    ZeusSubscriber(std::string topic, std::function<CbType> cb_, bool append_ = false)
            : _cb(cb_)
            , append(append_) {
        ros::NodeHandle nh;
        sub = nh.subscribe(topic, sub_queue_size, &ZeusSubscriber::cb, this);
        button.on_click([this] { toggle(); });
        button.description = "[ " + topic + " ]";  // actual start state might not be subscribed... sometimes

        box.add(button);
        box.add(textarea);
        box.display();  // @TODO: adjust width of the box
    }

    ~ZeusSubscriber() { sub.shutdown(); }

    ZeusSubscriber(std::string topic, CbType* cb_, bool append_ = false)
            : ZeusSubscriber(topic, std::function<CbType>(cb_), append_) {}

    ZeusSubscriber(std::string topic, boost::function<CbType> cb_, bool append_ = false)
            : ZeusSubscriber(topic, std::function<CbType>(cb_), append_) {}

    /**
     * @brief Get the ROS Subscriber Queue Size
     */
    inline int getSubscriberQueueSize() { return sub_queue_size; }

    /**
     * @brief Set the ROS Subscriber Queue Size
     */
    inline void setSubscriberQueueSize(int val) { sub_queue_size = val; }

private:
    /**
     * @brief The callback for ros subscriber
     * The std::cout is redirected to stringstream so we can add it to textarea
     */
    void cb(const Arg& arg) {
        std::stringstream zeus_cout;                  // do smthg for color
        std::streambuf* coutbuf = std::cout.rdbuf();  // save old buf
        // divert cout to string stream
        std::cout.rdbuf(zeus_cout.rdbuf());

        _cb(arg);
        // set the text output as the captured stdout
        textarea.value = (append ? textarea.value() : "") + zeus_cout.str();

        // restore cout
        std::cout.rdbuf(coutbuf);
    }

    /**
     * @brief Start & stop subscribing when button is clicked
     */
    void toggle() {
        auto topic = sub.getTopic();
        if (sub) {
            sub.shutdown();
            button.description = "Start [ " + topic + " ]";
        } else {
            ros::NodeHandle nh;
            sub = nh.subscribe(topic, sub_queue_size, &ZeusSubscriber::cb, this);
            button.description = "Stop [ " + topic + " ]";
        }
    }
};
