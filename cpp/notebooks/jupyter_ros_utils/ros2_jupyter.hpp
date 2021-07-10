#pragma once

#include <rclcpp/rclcpp.hpp>

#include <cstring>

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

#include <rclcpp/rclcpp.hpp>

#include <xwidgets/xbutton.hpp>
#include <xwidgets/xbox.hpp>
#include <xwidgets/xtextarea.hpp>

#include <memory>
#include <functional>

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

#include <xwidgets/xbutton.hpp>
#include <xwidgets/xvalid.hpp>
#include <xwidgets/xbox.hpp>

#include <memory>
#include <thread>

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