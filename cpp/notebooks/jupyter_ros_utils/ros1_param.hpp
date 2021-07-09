#pragma once

#include <atomic>
#include <thread>

#include <xwidgets/xbox.hpp>
#include <xwidgets/xbutton.hpp>
#include <xwidgets/xlabel.hpp>
#include <xwidgets/xtext.hpp>
#include <xwidgets/xvalid.hpp>

#include "init.hpp"

struct ZeusParam {
private:
    // widget objects
    xw::valid valid;
    xw::label lbl_param;
    xw::text txt_param;
    xw::label lbl_value;
    xw::text txt_value;
    xw::button btn_wait;
    xw::button btn_set;

    // widget layout
    xw::hbox main_layout;
    xw::vbox left_section;
    // middle section is just the valid button, no need for layout
    xw::vbox right_section;

    xw::hbox label_txt_param_layout;  // part of left_section
    xw::hbox label_txt_value_layout;  // part of left_section

    // ros stuff
    ros::NodeHandle nh;

    std::atomic<bool> thr_running;
    std::thread thr;
    std::size_t sleep_usec;

public:
    ZeusParam(const std::string& default_param_name = "", const std::string& default_param_value = "",
            std::size_t sleep_usec = 100'000)
            : valid()
            , txt_param()
            , btn_wait()
            , txt_value()
            , btn_set()
            , main_layout()
            , sleep_usec(sleep_usec) {
        thr_running = false;

        // init compontents
        lbl_param.value = "param_name";
        txt_param.value = default_param_name;
        txt_param.on_submit([this]() { valid.value = nh.hasParam(txt_param.value); });
        lbl_value.value = "param_value";
        txt_value.value = default_param_value;
        txt_value.on_submit(std::bind(&ZeusParam::set_param_value, this));
        btn_wait.description = "Start Waiting";
        btn_wait.on_click([this]() {
            if (thr_running) {
                stop_thread();
            } else {
                start_thread();
            }
        });
        btn_set.description = "Set Param";
        btn_set.on_click(std::bind(&ZeusParam::set_param_value, this));

        // set up layout
        // | lbl_param | txt_param | valid  | btn_wait
        // | lbl_value | txt_value |        | btn_set
        label_txt_param_layout.add(lbl_param);
        label_txt_param_layout.add(txt_param);

        label_txt_value_layout.add(lbl_value);
        label_txt_value_layout.add(txt_value);

        left_section.add(label_txt_param_layout);
        left_section.add(label_txt_value_layout);
        main_layout.add(left_section);

        main_layout.add(valid);

        right_section.add(btn_wait);
        right_section.add(btn_set);

        main_layout.add(right_section);
    }

    void display() { main_layout.display(); }

private:
    void set_param_value() {
        nh.setParam(txt_param.value, txt_value.value);
        valid.value = nh.hasParam(txt_param.value);
    }

    void start_thread() {
        thr_running = true;
        txt_param.disabled = true;
        txt_value.disabled = true;
        btn_wait.description = "Stop Waiting";

        thr = std::thread(&ZeusParam::thread_loop, this);
    }

    void thread_loop() {
        while (thr_running) {
            if (nh.hasParam(txt_param.value)) {
                valid.value = true;
                std::string tmp;
                nh.getParam(txt_param.value, tmp);
                txt_value.value = tmp;
            } else {
                valid.value = false;
                txt_value.value = "";
            }
            usleep(sleep_usec);
        }
    }

    void stop_thread() {
        thr_running = false;
        txt_param.disabled = false;
        txt_value.disabled = false;
        btn_wait.description = "Start Waiting";

        usleep(1'000);

        if (thr.joinable()) {
            thr.join();
        }
    }
};
