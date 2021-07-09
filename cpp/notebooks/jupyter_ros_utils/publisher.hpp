#include "xwidgets/xbutton.hpp"
#include "xwidgets/xbox.hpp"
#include "xwidgets/xtextarea.hpp"
#include "xwidgets/xbox.hpp"
#include "xwidgets/xtogglebutton.hpp"
#include "xwidgets/xoutput.hpp"
#include "xwidgets/xlabel.hpp"
#include "xwidgets/xtext.hpp"
#include <boost/function.hpp>
#include <sstream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <ros/spinner.h>

#include <cstring>
#include <iostream>
#include <string>
#include <thread>

void ros_init()
{
    int argc = 1;
    char *argv[1];
    argv[0] = new char[9];
    strcpy(argv[0], "jupyter_ros_pub");
    ros::init(argc, argv, "jupyter_ros_pub");
    static ros::AsyncSpinner spinner(4);
    spinner.start();
}

class ZeusPublisher {
    private:
        ros::Publisher pub;
        xw::label pub_value;
        xw::text  pub_value_txt;
        xw::button btn;

        xw::hbox main_layout;
        xw::vbox left_section;
        xw::vbox right_section;
        xw::hbox pub_layout;
        std_msgs::String value_pub;
        bool stop_publisher = 1;
        bool thr_running = false;
        std::thread thr;


    public:
        ZeusPublisher(std::string topic) {
            ros::NodeHandle nh;
            pub = nh.advertise<std_msgs::String>(topic, 10);
            pub_value.value = "Input String to Publisher";
            pub_value_txt.on_submit((std::bind(&ZeusPublisher::publish_msg, this)));
            pub_layout.add(pub_value);
            pub_layout.add(pub_value_txt);
            left_section.add(pub_layout);
            btn.description = "Publish Value";
            btn.on_click([this](){
                if (thr_running) {
                    stop_thread();
                } else {
                    start_thread();
                }
            });
            right_section.add(btn);
            // layout
            // pub_value | pub_value_text       btn
            main_layout.add(left_section);
            main_layout.add(right_section);
            main_layout.display();
        }

        ~ZeusPublisher(){
            pub.shutdown();
        }

        void publish_msg() {
            value_pub.data = pub_value_txt.value;
            pub.publish(value_pub);
        }

    void start_thread () {
        thr_running = true;
        pub_value_txt.disabled = true;
        btn.description = "Stop Publishing";
        thr = std::thread(&ZeusPublisher::thread_loop, this);
    }

    void thread_loop () {
        while (thr_running) {
            value_pub.data = pub_value_txt.value;
            pub.publish(value_pub);
        }
    }

    void stop_thread () {
        thr_running = false;
        pub_value_txt.disabled = false;
        btn.description = "Start Publishing";

        usleep(1'000);

        if (thr.joinable()) {
            thr.join();
        }
    }

};
