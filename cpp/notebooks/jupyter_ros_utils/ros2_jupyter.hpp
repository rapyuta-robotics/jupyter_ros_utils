#pragma once

#include <cstring>
#include <memory>
#include <thread>
#include <functional>

#include <xwidgets/xbox.hpp>
#include <xwidgets/xbutton.hpp>
#include <xwidgets/xlabel.hpp>
#include <xwidgets/xtext.hpp>
#include <xwidgets/xtextarea.hpp>
#include <xwidgets/xvalid.hpp>

#include <rclcpp/rclcpp.hpp>

struct ROS2Init {
    ROS2Init() { ros_init(); }

    void ros_init() {
        if (rclcpp::ok()) { return; }
        int argc = 1;
        char *argv[1];
        argv[0] = new char[9];
        strcpy(argv[0], "jupyter");
        rclcpp::init(argc, argv);
    }
};


struct CaptureStdCout {
    using Func = std::function<void(const std::string&)>;

    CaptureStdCout(Func func_): func(func_) {
        // do smthg for color
        coutbuf = std::cout.rdbuf();  // save old buf
        // divert cout to string stream
        std::cout.rdbuf(cap_cout.rdbuf());
    }
    ~CaptureStdCout() {
        // set the text output as the captured stdout
        func(cap_cout.str());
        // restore cout
        std::cout.rdbuf(coutbuf);
    }
    std::stringstream cap_cout;
    std::streambuf* coutbuf;
    Func func;
};

template <class T>
struct simplify_callback { using type = T; };
template <class T>
using simplified_callback = typename simplify_callback<T>::type;

template <class R, class... Args>
struct simplify_callback<R(*)(Args...)>: simplify_callback<R(Args...)> {};
template <class R, class... Args>
struct simplify_callback<R(&)(Args...)>: simplify_callback<R(Args...)> {};

template <class MsgT, class SubT, class CallbackT, class AllocatorT, class MessageMemoryStrategyT, class SimplifiedCallbackT=simplified_callback<CallbackT>>
class ZeusSubscriber  {};

template <class MsgT, class SubT, class CallbackT, class AllocatorT, class MessageMemoryStrategyT, class R, class... Args>
class ZeusSubscriber<MsgT, SubT, CallbackT, AllocatorT, MessageMemoryStrategyT, R(Args...)>  {
    std::function<R(Args...)> m_cb;
    std::shared_ptr<SubT> sub;
    bool m_append;
    rclcpp::Node* m_nh;

    std::string m_topic;
    rclcpp::QoS m_qos;
    rclcpp::SubscriptionOptionsWithAllocator< AllocatorT > m_options;
    typename MessageMemoryStrategyT::SharedPtr m_msg_mem_strat;

    xw::vbox box;
    xw::button button;
    xw::textarea textarea;

    R cb(Args&&... args) {
        CaptureStdCout cap{[this](const auto& data) { update(data); }};
        return m_cb(std::forward<Args>(args)...);
    }

    void update(const std::string& data) {
        textarea.value = (m_append ? textarea.value() : "") + data;
    }
    public:
    ZeusSubscriber(rclcpp::Node* nh, const std::string &topic_name, const rclcpp::QoS &qos, CallbackT &&callback,
                   const rclcpp::SubscriptionOptionsWithAllocator< AllocatorT > &options,
                   typename MessageMemoryStrategyT::SharedPtr msg_mem_strat, bool append)
            : m_cb(callback)
            , m_append(append)
            , m_nh(nh)
            , m_topic(topic_name)
            , m_qos(qos)
            , m_options(options)
            , m_msg_mem_strat(msg_mem_strat) {
        toggle();

        button.on_click([this] { toggle(); });
        button.description = "Start [ " + m_topic + " ]";  // actual start state might not be subscribed... sometimes

        box.add(button);
        box.add(textarea);
        box.display();  // @TODO: adjust width of the box
    }

    void toggle() {
        if (sub) {
            std::weak_ptr<SubT> weak_sub { sub };
            sub.reset();
            if (weak_sub.expired()) {
                button.description = "Start [ " + m_topic + " ]";
            } else {
                button.description = "Someone else shares the subscriber";
                button.disabled = true;
            }
        } else if (!button.disabled()) {
            auto func = std::function<R(Args...)>{[this](auto&&...args){ return this->cb(std::forward<Args>(args)...);}};
            sub = m_nh->create_subscription<MsgT>(m_topic, m_qos, func, m_options, m_msg_mem_strat);
            button.description = "Stop [ " + m_topic + " ]";
        }
    }

    std::weak_ptr<SubT> get_subscriber() {
        return sub;
    }
    // returns true if the subscriber has been shut down
    bool try_shutdown() {
        sub.reset();
        return !sub;
    }
};


class ZeusParamWidget{
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

    std::atomic<bool> thr_running;
    std::thread thr;

    // ros stuff
    rclcpp::Node* node;

public:
    ZeusParamWidget(rclcpp::Node* nh)
            : nh(nh)
            , valid()
            , txt_param()
            , btn_wait()
            , txt_value()
            , btn_set()
            , main_layout() {
        thr_running = false;

        // init compontents
        lbl_param.value = "param_name";
        txt_param.on_submit([this]() { valid.value = nh->has_parameter(txt_param.value); });
        lbl_value.value = "param_value";
        txt_value.on_submit(std::bind(&ZeusParamWidget::set_param_value, this));
        btn_wait.description = "Start Waiting";
        btn_wait.on_click([this]() {
            if (thr_running) {
                stop_thread();
            } else {
                start_thread();
            }
        });
        btn_set.description = "Set Param";
        btn_set.on_click(std::bind(&ZeusParamWidget::set_param_value, this));

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

        display();
    }

private:
    void display() { main_layout.display(); }

    void set_param_value() {
        if (!nh->has_parameter(txt_param.value)) {
            nh->declare_parameter(txt_param.value, rclcpp::ParameterValue(txt_value.value));
        }
        auto param = rclcpp::Parameter(txt_param.value, rclcpp::ParameterValue(txt_value.value));
        nh->set_parameter(param);
        valid.value = nh->has_parameter(txt_param.value);
    }

    void start_thread() {
        thr_running = true;
        txt_param.disabled = true;
        txt_value.disabled = true;
        btn_wait.description = "Stop Waiting";

        thr = std::thread(&ZeusParamWidget::thread_loop, this);
    }

    void thread_loop() {
        while (thr_running) {
            if (nh->has_parameter(txt_param.value)) {
                valid.value = true;
                std::string tmp;
                nh->get_parameter(txt_param.value, tmp);
                txt_value.value = tmp;
            } else {
                valid.value = false;
                txt_value.value = "";
            }
            usleep(200'000);
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


template <class NodeType = rclcpp::Node>
class JupyterNodeAdvanced: ROS2Init, public NodeType {
    xw::button spin_btn;
    xw::valid ros_valid;
    xw::hbox disp_box;
    
    std::thread spin_thread;
    rclcpp::Rate m_rate;
    
    void toggle(bool skip_check = false) {
        if (skip_check == false && rclcpp::ok()) {
        std::cout << "shutdown\n";
            rclcpp::shutdown();
            spin_btn.description = "Start";
        } else {
        std::cout <<"restart\n";
            ros_init();
            setup_thread();
            spin_btn.description = "Stop";
        }
        ros_valid.value = rclcpp::ok();
    }
    
    void setup_thread() {
        if (spin_thread.joinable()) {
            // should never happen
            spin_thread.join();
        }
        spin_thread = std::thread([this]{ 
            ros_valid.value = rclcpp::ok();
            while(rclcpp::ok()) { 
                rclcpp::spin_some(getNode()); 
                m_rate.sleep();
            }
            ros_valid.value = false; 
        });
    
    }

public:
    std::shared_ptr<NodeType> getNode() {
        return this->shared_from_this();
    }

    JupyterNodeAdvanced(double rate = 10): NodeType("interative_node"), m_rate(rate) {
        spin_btn.description = "Stop";
        spin_btn.on_click([this]{toggle();});
        
        disp_box.add(spin_btn);
        disp_box.add(ros_valid);
        disp_box.display();
        
        setup_thread();
    }


public:
    auto create_param_widgets() {
        return ZeusParamWidget(this);

    }
    template<typename MessageT , typename CallbackT , typename AllocatorT = std::allocator<void>, typename CallbackMessageT = typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>, typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<CallbackMessageT, AllocatorT>>
    auto create_subscription(const std::string &topic_name, const rclcpp::QoS &qos, CallbackT &&callback,
            const rclcpp::SubscriptionOptionsWithAllocator< AllocatorT > &options=rclcpp::SubscriptionOptionsWithAllocator< AllocatorT >(),
            typename MessageMemoryStrategyT::SharedPtr msg_mem_strat=(MessageMemoryStrategyT::create_default()), bool append=false) {
        return ZeusSubscriber<MessageT, SubscriptionT, CallbackT, AllocatorT, MessageMemoryStrategyT>(this, topic_name, qos, callback, options, msg_mem_strat, append);
    }

    template<typename MessageT , typename CallbackT , typename AllocatorT = std::allocator<void>, typename CallbackMessageT = typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>, typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<CallbackMessageT, AllocatorT>>
    auto create_subscription(const std::string &topic_name, const rclcpp::QoS &qos, CallbackT &&callback, bool append_) {
        return create_subscription<MessageT>(topic_name, qos, callback,
            rclcpp::SubscriptionOptionsWithAllocator< AllocatorT >(), (MessageMemoryStrategyT::create_default()), append_);
    }
};

using JupyterNode = JupyterNodeAdvanced<>;

std::shared_ptr<JupyterNode> ros_init() {
    static auto node_ptr = std::make_shared<JupyterNode>();
    return node_ptr;
}