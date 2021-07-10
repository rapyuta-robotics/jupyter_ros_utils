#pragma once

#include <atomic>
#include <sstream>
#include <thread>

#include <boost/function.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "xwidgets/xbutton.hpp"
#include "xwidgets/xbox.hpp"
#include "xwidgets/xtextarea.hpp"
#include "xwidgets/xbox.hpp"
#include "xwidgets/xtogglebutton.hpp"
#include "xwidgets/xoutput.hpp"
#include "xwidgets/xlabel.hpp"
#include "xwidgets/xtext.hpp"

#include "init.hpp"

/**
 * @brief ROS Subscriber with widgets for jupyter notebook
 */
class ZeusPublisher {
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
    ZeusPublisher(const std::string& topic, int pub_queue_size = 10, size_t sleep_usec = 100'000)
            : sleep_usec(sleep_usec) {
        ros::NodeHandle nh;
        pub = nh.advertise<std_msgs::String>(topic, pub_queue_size);
        pub_value.value = "Input String to Publisher";
        pub_value_txt.on_submit((std::bind(&ZeusPublisher::publish_msg, this)));
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

    ~ZeusPublisher() { pub.shutdown(); }

private:
    void publish_msg() {
        value_pub.data = pub_value_txt.value;
        pub.publish(value_pub);
    }

    void start_thread() {
        thr_running = true;
        pub_value_txt.disabled = true;
        btn.description = "Stop Publishing";
        thr = std::thread(&ZeusPublisher::thread_loop, this);
    }

    void thread_loop() {
        while (thr_running) {
            value_pub.data = pub_value_txt.value;
            pub.publish(value_pub);
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
