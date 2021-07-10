#pragma once

#include <atomic>
#include <sstream>
#include <thread>

#include <boost/function.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "xwidgets/xbox.hpp"
#include "xwidgets/xbutton.hpp"
#include "xwidgets/xlabel.hpp"
#include "xwidgets/xtext.hpp"
#include "xwidgets/xtextarea.hpp"

#include "init.hpp"

/**
 * @brief Simple ROS publisher widgets to publish std_msgs::String based on user input in textbox
 */
class ZeusSimplePublisher {
private:
    ros::Publisher pub;
    xw::label pub_value;
    xw::text pub_value_txt;
    xw::button btn;

    xw::hbox main_layout;
    xw::vbox left_section;
    xw::vbox right_section;
    xw::hbox pub_layout;
    std_msgs::String value_pub;
    std::atomic<bool> thr_running = false;
    std::thread thr;
    size_t sleep_usec;

public:
    ZeusSimplePublisher(const std::string& topic, int pub_queue_size = 10, size_t sleep_usec = 100'000)
            : sleep_usec(sleep_usec) {
        ros::NodeHandle nh;
        pub = nh.advertise<std_msgs::String>(topic, pub_queue_size);
        pub_value.value = "Input String to Publisher";
        pub_value_txt.on_submit((std::bind(&ZeusSimplePublisher::publish_msg, this)));
        btn.description = "Publish Value";
        btn.on_click([this]() {
            if (thr_running) {
                stop_thread();
            } else {
                start_thread();
            }
        });

        // setup layout
        // pub_value | pub_value_text       btn
        pub_layout.add(pub_value);
        pub_layout.add(pub_value_txt);
        left_section.add(pub_layout);
        right_section.add(btn);
        main_layout.add(left_section);
        main_layout.add(right_section);
        main_layout.display();
    }

    ~ZeusSimplePublisher() { pub.shutdown(); }

private:
    void publish_msg() {
        value_pub.data = pub_value_txt.value;
        pub.publish(value_pub);
    }

    void start_thread() {
        thr_running = true;
        pub_value_txt.disabled = true;
        btn.description = "Stop Publishing";
        thr = std::thread(&ZeusSimplePublisher::thread_loop, this);
    }

    void thread_loop() {
        while (thr_running) {
            value_pub.data = pub_value_txt.value;
            pub.publish(value_pub);
            usleep(sleep_usec);
        }
    }

    void stop_thread() {
        thr_running = false;
        pub_value_txt.disabled = false;
        btn.description = "Start Publishing";

        usleep(1'000);

        if (thr.joinable()) {
            thr.join();
        }
    }
};

/**
 * @brief ROS publisher widgets that takes arbitrary message and accept callback function
 */
template <typename Arg>
class ZeusPublisher {
private:
    ros::Publisher pub;
    xw::button btn;
    xw::textarea txt;

    xw::vbox main_layout;

    std::atomic<bool> thr_running = false;
    std::thread thr;
    size_t sleep_usec;
    bool append;  // append text or replace text

    using CbType = Arg(void);
    std::function<CbType> cb;

public:
    ZeusPublisher(const std::string& topic, std::function<CbType> cb, int pub_queue_size = 10,
            size_t sleep_usec = 100'000, bool append = false)
            : cb(cb)
            , sleep_usec(sleep_usec)
            , append(append) {
        ros::NodeHandle nh;
        pub = nh.advertise<Arg>(topic, pub_queue_size);

        btn.description = "Start Publishing";
        btn.on_click([this]() {
            if (thr_running) {
                stop_thread();
            } else {
                start_thread();
            }
        });

        // setup layout
        // btn
        // ---
        // txt
        main_layout.add(btn);
        main_layout.add(txt);
        main_layout.display();
    }

    ~ZeusPublisher() { pub.shutdown(); }

    ZeusPublisher(const std::string& topic, CbType* cb_, int pub_queue_size = 10, size_t sleep_usec = 100'000,
            bool append = false)
            : ZeusPublisher(topic, std::function<CbType>(cb_), pub_queue_size, sleep_usec, append) {}

    ZeusPublisher(const std::string& topic, boost::function<CbType> cb_, int pub_queue_size = 10,
            size_t sleep_usec = 100'000, bool append = false)
            : ZeusPublisher(topic, std::function<CbType>(cb_), pub_queue_size, sleep_usec, append) {}

private:
    void start_thread() {
        thr_running = true;
        btn.description = "Stop Publishing";
        thr = std::thread(&ZeusPublisher::thread_loop, this);
    }

    void thread_loop() {
        Arg msg;
        while (thr_running) {
            msg = cb();
            std::stringstream ss;
            ss << msg;
            txt.value = (append ? txt.value() : "") + ss.str();

            pub.publish(msg);

            usleep(sleep_usec);
        }
    }

    void stop_thread() {
        thr_running = false;
        btn.description = "Start Publishing";

        usleep(1'000);

        if (thr.joinable()) {
            thr.join();
        }
    }
};
