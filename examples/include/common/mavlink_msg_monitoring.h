#pragma once
// MESSAGE MONITORING PACKING

#define MAVLINK_MSG_ID_MONITORING 180


typedef struct __mavlink_monitoring_t {
 uint8_t ready; /*<  Flight readiness status.*/
 uint8_t battery; /*< [%] Remaining battery level.*/
 uint8_t safety; /*<  Safety button ON/OFF status.*/
} mavlink_monitoring_t;

#define MAVLINK_MSG_ID_MONITORING_LEN 3
#define MAVLINK_MSG_ID_MONITORING_MIN_LEN 3
#define MAVLINK_MSG_ID_180_LEN 3
#define MAVLINK_MSG_ID_180_MIN_LEN 3

#define MAVLINK_MSG_ID_MONITORING_CRC 137
#define MAVLINK_MSG_ID_180_CRC 137



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MONITORING { \
    180, \
    "MONITORING", \
    3, \
    {  { "ready", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_monitoring_t, ready) }, \
         { "battery", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_monitoring_t, battery) }, \
         { "safety", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_monitoring_t, safety) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MONITORING { \
    "MONITORING", \
    3, \
    {  { "ready", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_monitoring_t, ready) }, \
         { "battery", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_monitoring_t, battery) }, \
         { "safety", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_monitoring_t, safety) }, \
         } \
}
#endif

/**
 * @brief Pack a monitoring message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ready  Flight readiness status.
 * @param battery [%] Remaining battery level.
 * @param safety  Safety button ON/OFF status.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_monitoring_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t ready, uint8_t battery, uint8_t safety)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MONITORING_LEN];
    _mav_put_uint8_t(buf, 0, ready);
    _mav_put_uint8_t(buf, 1, battery);
    _mav_put_uint8_t(buf, 2, safety);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MONITORING_LEN);
#else
    mavlink_monitoring_t packet;
    packet.ready = ready;
    packet.battery = battery;
    packet.safety = safety;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MONITORING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MONITORING;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
}

/**
 * @brief Pack a monitoring message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ready  Flight readiness status.
 * @param battery [%] Remaining battery level.
 * @param safety  Safety button ON/OFF status.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_monitoring_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t ready,uint8_t battery,uint8_t safety)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MONITORING_LEN];
    _mav_put_uint8_t(buf, 0, ready);
    _mav_put_uint8_t(buf, 1, battery);
    _mav_put_uint8_t(buf, 2, safety);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MONITORING_LEN);
#else
    mavlink_monitoring_t packet;
    packet.ready = ready;
    packet.battery = battery;
    packet.safety = safety;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MONITORING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MONITORING;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
}

/**
 * @brief Encode a monitoring struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param monitoring C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_monitoring_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_monitoring_t* monitoring)
{
    return mavlink_msg_monitoring_pack(system_id, component_id, msg, monitoring->ready, monitoring->battery, monitoring->safety);
}

/**
 * @brief Encode a monitoring struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param monitoring C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_monitoring_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_monitoring_t* monitoring)
{
    return mavlink_msg_monitoring_pack_chan(system_id, component_id, chan, msg, monitoring->ready, monitoring->battery, monitoring->safety);
}

/**
 * @brief Send a monitoring message
 * @param chan MAVLink channel to send the message
 *
 * @param ready  Flight readiness status.
 * @param battery [%] Remaining battery level.
 * @param safety  Safety button ON/OFF status.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_monitoring_send(mavlink_channel_t chan, uint8_t ready, uint8_t battery, uint8_t safety)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MONITORING_LEN];
    _mav_put_uint8_t(buf, 0, ready);
    _mav_put_uint8_t(buf, 1, battery);
    _mav_put_uint8_t(buf, 2, safety);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, buf, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#else
    mavlink_monitoring_t packet;
    packet.ready = ready;
    packet.battery = battery;
    packet.safety = safety;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, (const char *)&packet, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#endif
}

/**
 * @brief Send a monitoring message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_monitoring_send_struct(mavlink_channel_t chan, const mavlink_monitoring_t* monitoring)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_monitoring_send(chan, monitoring->ready, monitoring->battery, monitoring->safety);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, (const char *)monitoring, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#endif
}

#if MAVLINK_MSG_ID_MONITORING_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_monitoring_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t ready, uint8_t battery, uint8_t safety)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, ready);
    _mav_put_uint8_t(buf, 1, battery);
    _mav_put_uint8_t(buf, 2, safety);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, buf, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#else
    mavlink_monitoring_t *packet = (mavlink_monitoring_t *)msgbuf;
    packet->ready = ready;
    packet->battery = battery;
    packet->safety = safety;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, (const char *)packet, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#endif
}
#endif

#endif

// MESSAGE MONITORING UNPACKING


/**
 * @brief Get field ready from monitoring message
 *
 * @return  Flight readiness status.
 */
static inline uint8_t mavlink_msg_monitoring_get_ready(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field battery from monitoring message
 *
 * @return [%] Remaining battery level.
 */
static inline uint8_t mavlink_msg_monitoring_get_battery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field safety from monitoring message
 *
 * @return  Safety button ON/OFF status.
 */
static inline uint8_t mavlink_msg_monitoring_get_safety(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a monitoring message into a struct
 *
 * @param msg The message to decode
 * @param monitoring C-struct to decode the message contents into
 */
static inline void mavlink_msg_monitoring_decode(const mavlink_message_t* msg, mavlink_monitoring_t* monitoring)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    monitoring->ready = mavlink_msg_monitoring_get_ready(msg);
    monitoring->battery = mavlink_msg_monitoring_get_battery(msg);
    monitoring->safety = mavlink_msg_monitoring_get_safety(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MONITORING_LEN? msg->len : MAVLINK_MSG_ID_MONITORING_LEN;
        memset(monitoring, 0, MAVLINK_MSG_ID_MONITORING_LEN);
    memcpy(monitoring, _MAV_PAYLOAD(msg), len);
#endif
}
