#include "xwidgets/xbutton.hpp"
#include "xwidgets/xbox.hpp"
#include "xwidgets/xtextarea.hpp"
#include <boost/function.hpp>
#include <sstream>
#include <ros/ros.h>
#include <ros/spinner.h>

#include <cstring>
#include <iostream>
#include <string>

void ros_init()
{
    int argc = 1;
    char *argv[1];
    argv[0] = new char[9];
    strcpy(argv[0], "jupyter_ros_sub");
    ros::init(argc, argv, "jupyter_ros_sub");
    static ros::AsyncSpinner spinner(4);
    spinner.start();
}

template<class Arg>
class ZeusSubscriber {
    private:
        xw::vbox box;
        xw::button button;
        ros::Subscriber sub;
        xw::textarea textarea;
        bool append = false;

        using CbType = void(Arg);
        std::function<CbType> _cb;

    public:
        ZeusSubscriber(std::string topic, std::function<CbType> cb_, bool append_=false): _cb(cb_), append(append_) {
            ros::NodeHandle nh;
            sub = nh.subscribe(topic, 10, &ZeusSubscriber::cb, this);
            button.on_click([this]{toggle();});
            button.description = "[ " + topic + " ]";  // actual start state might not be subscribed... sometimes

            box.add(button);
            box.add(textarea);
            box.display();  // @TODO: adjust width of the box
        }

        ~ZeusSubscriber(){
            sub.shutdown();
        }

        ZeusSubscriber(std::string topic, CbType* cb_, bool append_=false): ZeusSubscriber(topic, std::function<CbType>(cb_), append_) {}
        ZeusSubscriber(std::string topic, boost::function<CbType> cb_, bool append_=false): ZeusSubscriber(topic, std::function<CbType>(cb_), append_) {}

        void cb(const Arg& arg) {
            std::stringstream zeus_cout;  // do smthg for color
            std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
            // divert cout to string stream
            std::cout.rdbuf(zeus_cout.rdbuf());

            _cb(arg);
            // set the text output as the captured stdout
            textarea.value = (append ? textarea.value() : "") + zeus_cout.str();

            // restore cout
            std::cout.rdbuf(coutbuf);

            }

        void toggle() {
            auto topic = sub.getTopic();
            if (sub) {
                sub.shutdown();
                button.description = "Start [ " + topic + " ]";
            } else {
                ros::NodeHandle nh;
                sub = nh.subscribe(topic, 10, &ZeusSubscriber::cb, this);
                button.description = "Stop [ " + topic + " ]";
            }
        }
};
