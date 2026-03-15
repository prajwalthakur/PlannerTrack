// Copyright 2024 TIER IV, Inc.
#pragma once
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>


namespace mpl_rclcpp_utils
{

 
/**
 * @brief Creates a SensorDataQoS profile with a single depth.
 * @return rclcpp::SensorDataQoS The QoS profile with depth set to 1.
*/

inline rclcpp::SensorDataQoS single_depth_sensor_qos()
{   

    rclcpp::SensorDataQoS qos;
    qos.get_rmw_qos_profile().depth = 1;
    return qos;
}

namespace polling_policy
{
    /**
    * @brief Polling policy that keeps the latest received message.
    *
    * This policy retains the latest received message and provides it when requested. If a new message
    * is received, it overwrites the previously stored message.
    *
    * @tparam MessageT The message type.
    */
    template <typename MessageT>
    class Latest
    {
        public:

            /**
            *
            *  @brief Retrieve the latest data. If no new data has been received, the previously received
            * data
            *
            * @return typename MessageT::ConstSharedPtr The latest data.
            */
            typename MessageT::ConstSharedPtr takeData();
            /**
            * @brief Getter for timestamp of the last message
            *
            * @pre To receive a valid timestamp, at least one message must have been received via
            * take_data(). The timestamp is retained until a new message is received.
            * @return std::optional<rclcpp::Time> The last timestamp. std::nullopt if no message has been
            * received.
            */
            std::optional<rclcpp::Time> lastTakenDataTimeStamp() const
            {
                return mTimeStamp;
            }
        
        protected:
            /**
            * @brief Check the QoS settings for the subscription.
            *
            * @param qos The QoS profile to check.
            * @throws std::invalid_argument If the QoS depth is not 1.
            */
            void checkQos(const rclcpp::QoS& qos)
            {
                if(qos.get_rmw_qos_profile().depth >1)
                {
                    throw std::invalid_argument(
                        "InternalProcessPollingSubscriber will be used with depth > 1, which may cause inefficient "
                        "serialization while updateLatestData()");
                }
            }

        private:
            typename MessageT::ConstSharedPtr mData{nullptr};
            std::optional<rclcpp::Time> mTimeStamp{std::nullopt};
    };
    template <typename MessageT>
    class Newest
    {
        public:

            /**
            *
            *  @brief Retrieve the latest data. If no new data has been received, the previously received
            * data
            *
            * @return typename MessageT::ConstSharedPtr The latest data.
            */
            typename MessageT::ConstSharedPtr takeData();
            /**
            * @brief Getter for timestamp of the last message
            *
            * @pre To receive a valid timestamp, at least one message must have been received via
            * take_data(). The timestamp is retained until a new message is received.
            * @return std::optional<rclcpp::Time> The last timestamp. std::nullopt if no message has been
            * received.
            */
            std::optional<rclcpp::Time> lastTakenDataTimeStamp() const
            {
                return mTimeStamp;
            }
        
        protected:
            /**
            * @brief Check the QoS settings for the subscription.
            */
            void checkQos(const rclcpp::QoS& qos) 
            { 
                if (qos.get_rmw_qos_profile().depth > 1) 
                {
                    throw std::invalid_argument(
                    "InternalProcessPollingSubscriber will be used with depth > 1, which may cause inefficient "
                    "serialization while updateLatestData()");
                }
            } 

        private:
            typename MessageT::ConstSharedPtr mData{nullptr};
            std::optional<rclcpp::Time> mTimeStamp{std::nullopt};
    };

    template <typename MessageT>
    class All
    {
        public:
            /**
            *
            *  @brief Retrieve the latest data. If no new data has been received, the previously received
            * data
            *
            * @return typename MessageT::ConstSharedPtr The latest data.
            */
            std::vector<typename MessageT::ConstSharedPtr> takeData();
            /**
            * @brief Getter for timestamp of the last message
            *
            * @pre To receive a valid timestamp, at least one message must have been received via
            * take_data(). The timestamp is retained until a new message is received.
            * @return std::optional<rclcpp::Time> The last timestamp. std::nullopt if no message has been
            * received.
            */
            std::optional<rclcpp::Time> lastTakenDataTimeStamp() const
            {
                return mTimeStamp;
            }
        protected:
            /**
            * @brief Check the QoS settings for the subscription.
            */
            void checkQos(const rclcpp::QoS& ) 
            { 
            } 
        private:
            typename MessageT::ConstSharedPtr mData{nullptr};
            std::optional<rclcpp::Time> mTimeStamp{std::nullopt};
        
    };

}//// namespace polling_policy

/**
 * @brief Subscriber class that uses a specified polling policy.
 *
 * @tparam MessageT The message type.
 * @tparam PollingPolicy The polling policy to use.
 */
template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
class InternalProcessPollingSubscriber : public PollingPolicy<MessageT>
{
    friend PollingPolicy<MessageT>;
    public:
        using SharedPtr = std::shared_ptr<InternalProcessPollingSubscriber<MessageT,PollingPolicy>>;

        /**
        * @brief Construct a new InterProcessPollingSubscriber object.
        *
        * @param node The node to attach the subscriber to.
        * @param topic_name The topic name to subscribe to.
        * @param qos The QoS profile to use for the subscription.
        */
        explicit InternalProcessPollingSubscriber(rclcpp::Node* node, const std::string& topicName, const rclcpp::QoS& qos = rclcpp::QoS{1})
        {
            this->checkQos(qos);

            // do not exec the callback
            auto noexecCallbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
            auto noexecSubscriptionOptions = rclcpp::SubscriptionOptions();
            noexecSubscriptionOptions.callback_group = noexecCallbackGroup;
            mSubscriber = node->create_subscription<MessageT>(
                topicName, qos, [node]([[maybe_unused]] const typename MessageT::ConstSharedPtr msg){ assert(false);}, noexecSubscriptionOptions);
        }
        /**
        * @brief Create a subscription.
        *
        * @param node The node to attach the subscriber to.
        * @param topic_name The topic name to subscribe to.
        * @param qos The QoS profile to use for the subscription.
        * @return SharedPtr The created subscription.
        */

        static SharedPtr createSubscription(rclcpp::Node * node, const std::string& topicName, const rclcpp::QoS & qos = rclcpp::QoS{1})
        {
            return std::make_shared<InternalProcessPollingSubscriber<MessageT, PollingPolicy>>(
                node, topicName, qos
            );
        }

        typename rclcpp::Subscription<MessageT>::SharedPtr subscriber()
        {
            return mSubscriber;
        }

    private:
        typename rclcpp::Subscription<MessageT>::SharedPtr mSubscriber{nullptr};
};

namespace polling_policy
{
    //Keep the last received message even if no new one arrives
    template<typename MessageT>
    typename MessageT::ConstSharedPtr Latest<MessageT>::takeData()
    {
        auto& subscriber = static_cast<InternalProcessPollingSubscriber<MessageT,Latest>*>(this)->mSubscriber;
        auto newData = std::make_shared<MessageT>();
        rclcpp::MessageInfo messageInfo;
        const bool success = subscriber->take(*newData, messageInfo);
        if(success)
        {
            mData = newData;
            mTimeStamp = rclcpp::Time(messageInfo.get_rmw_message_info().source_timestamp,RCL_ROS_TIME);

        }
        return mData;
    }
    //Keep the last received message even if no new one arrives
    template<typename MessageT>
    typename MessageT::ConstSharedPtr Newest<MessageT>::takeData()
    {
        auto& subscriber = static_cast<InternalProcessPollingSubscriber<MessageT,Newest>*>(this)->mSubscriber;
        auto newData = std::make_shared<MessageT>();
        rclcpp::MessageInfo messageInfo;
        const bool success = subscriber->take(*newData, messageInfo);
        if(success)
        {
            mTimeStamp = rclcpp::Time(messageInfo.get_rmw_message_info().source_timestamp,RCL_ROS_TIME);
            return newData;
        }
        mTimeStamp = std::nullopt;
        return nullptr;

    }

    template<typename MessageT>
    std::vector<typename MessageT::ConstSharedPtr> All<MessageT>::takeData()
    {
        auto& subscriber = static_cast<InternalProcessPollingSubscriber<MessageT,All>*>(this)->mSubscriber;
        std::vector<typename MessageT::ConstSharedPtr> data;
        rclcpp::MessageInfo messageInfo;
        for(;;)
        {
            auto dataum = std::make_shared<MessageT>();
            if(subscriber->take(*dataum,messageInfo))
            {
                data.push_back(dataum);
                mTimeStamp = rclcpp::Time(messageInfo.get_rmw_message_info().source_timestamp,RCL_ROS_TIME);

            }
            else
            {
                break;
            }
        }
        if(data.empty())
        {
            mTimeStamp = std::nullopt;
        }
        return data;
        
    }

}// namespace polling_policy

}//mpl_rclcpp_utils