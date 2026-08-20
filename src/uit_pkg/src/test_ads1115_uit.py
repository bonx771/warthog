#!/usr/bin/env python3
import sys
import time

import smbus

try:
    import rospy
    import rosgraph
    from std_msgs.msg import Float32, String
except ImportError:
    rospy = None
    rosgraph = None
    Float32 = None
    String = None


I2C_BUS = 7
ADS1115_ADDR = 0x48
CONFIG_REG = 0x01
CONVERT_REG = 0x00

UIT_CHANNELS = [2, 3]

FULL_SCALE_VOLT = 4.096
POLL_INTERVAL_SEC = 0.1
DIVIDER_RATIO = 3.0

SENSOR_MIN_DISTANCE_MM = 250.0
SENSOR_MAX_DISTANCE_MM = 3500.0

SENSOR_OUTPUT_MIN_VOLT = 0.0
SENSOR_OUTPUT_MAX_VOLT = 10.0

OBSTACLE_SENSOR_VOLT_MAX = 5

NO_OBSTACLE_DISTANCE_MM = -1.0

DEFAULT_TOPIC_PREFIX = "/uit502"


CHANNEL_CONFIGS = {
    0: 0xC383,  # A0, single-ended, +/-4.096V
    1: 0xD383,  # A1, single-ended, +/-4.096V
    2: 0xE383,  # A2, single-ended, +/-4.096V
    3: 0xF383,  # A3, single-ended, +/-4.096V
}


def read_raw(bus, channel):
    if channel not in CHANNEL_CONFIGS:
        raise ValueError("Channel phai la 0, 1, 2 hoac 3")

    config = CHANNEL_CONFIGS[channel]

    bus.write_i2c_block_data(
        ADS1115_ADDR,
        CONFIG_REG,
        [
            (config >> 8) & 0xFF,
            config & 0xFF,
        ],
    )

    time.sleep(0.02)

    data = bus.read_i2c_block_data(
        ADS1115_ADDR,
        CONVERT_REG,
        2,
    )

    value = (data[0] << 8) | data[1]

    if value > 32767:
        value -= 65536

    return value


def raw_to_voltage(raw_value):
    return raw_value * FULL_SCALE_VOLT / 32768.0


def adc_to_sensor_voltage(adc_voltage):
    return adc_voltage * DIVIDER_RATIO


def sensor_voltage_to_distance_mm(sensor_voltage):
    clamped_voltage = max(
        SENSOR_OUTPUT_MIN_VOLT,
        min(
            SENSOR_OUTPUT_MAX_VOLT,
            sensor_voltage
        )
    )

    span_mm = SENSOR_MAX_DISTANCE_MM - SENSOR_MIN_DISTANCE_MM
    span_v = SENSOR_OUTPUT_MAX_VOLT - SENSOR_OUTPUT_MIN_VOLT

    return SENSOR_MIN_DISTANCE_MM + (
        clamped_voltage - SENSOR_OUTPUT_MIN_VOLT
    ) * span_mm / span_v


def detect_state(sensor_voltage):
    if sensor_voltage <= OBSTACLE_SENSOR_VOLT_MAX:
        return "co_vat_can"

    return "khong_vat_can"


def build_channel_name(channel):
    return "a{}".format(channel)


def init_ros_publishers(channels):
    if rospy is None or rosgraph is None:
        print(
            "Khong import duoc rospy/rosgraph/std_msgs. "
            "Script van chay terminal-only, khong publish topic.",
            flush=True,
        )

        return {}


    try:
        rosgraph.Master(
            "test_ads1115_voltage_terminal"
        ).getPid()

    except Exception as error:
        print(
            "Khong ket noi duoc ROS master ({}). "
            "Script van chay terminal-only.".format(error),
            flush=True,
        )

        return {}


    try:
        rospy.init_node(
            "test_ads1115_voltage_terminal",
            anonymous=True
        )

        publishers = {}

        for channel in channels:

            channel_name = build_channel_name(channel)

            distance_topic = "{}/{}/distance_mm".format(
                DEFAULT_TOPIC_PREFIX,
                channel_name,
            )

            state_topic = "{}/{}/state".format(
                DEFAULT_TOPIC_PREFIX,
                channel_name,
            )

            distance_pub = rospy.Publisher(
                distance_topic,
                Float32,
                queue_size=10
            )

            state_pub = rospy.Publisher(
                state_topic,
                String,
                queue_size=10,
                latch=True
            )

            publishers[channel] = {
                "distance_pub": distance_pub,
                "state_pub": state_pub,
                "distance_topic": distance_topic,
                "state_topic": state_topic,
            }

        return publishers

    except Exception as error:

        print(
            "Khong khoi tao duoc ROS publisher ({}). "
            "Script van tiep tuc chay terminal-only.".format(error),
            flush=True,
        )

        return {}


def publish_ros(
    publishers,
    channel,
    distance_mm,
    state
):

    if channel not in publishers:
        return

    distance_pub = publishers[channel]["distance_pub"]
    state_pub = publishers[channel]["state_pub"]

    distance_value = (
        distance_mm
        if state == "co_vat_can"
        else NO_OBSTACLE_DISTANCE_MM
    )

    distance_pub.publish(
        Float32(data=distance_value)
    )

    state_pub.publish(
        String(data=state)
    )


def main():

    bus = smbus.SMBus(I2C_BUS)

    publishers = init_ros_publishers(UIT_CHANNELS)

    print(
        "Dang doc ADS1115 bus {} addr 0x{:02X}.".format(
            I2C_BUS,
            ADS1115_ADDR,
        ),
        flush=True,
    )

    print(
        "UIT1 -> A{}".format(UIT_CHANNELS[0]),
        flush=True,
    )

    print(
        "UIT2 -> A{}".format(UIT_CHANNELS[1]),
        flush=True,
    )

    print(
        "Script dang gia su dung cau chia ap 20k/10k, "
        "nen dien ap tai ADS1115 = dien ap cam bien / 3.",
        flush=True,
    )

    print(
        "Nguong tam thoi tren dien ap cam bien: "
        "<= {:.1f}V = co_vat_can, "
        "> {:.1f}V = khong_vat_can.".format(
            OBSTACLE_SENSOR_VOLT_MAX,
            OBSTACLE_SENSOR_VOLT_MAX,
        ),
        flush=True,
    )


    if publishers:

        for channel in UIT_CHANNELS:

            print(
                "A{} publish: {} (Float32), {} (String)".format(
                    channel,
                    publishers[channel]["distance_topic"],
                    publishers[channel]["state_topic"],
                ),
                flush=True,
            )


    try:

        while True:

            for channel in UIT_CHANNELS:

                raw_value = read_raw(
                    bus,
                    channel
                )

                adc_voltage = raw_to_voltage(
                    raw_value
                )

                sensor_voltage = adc_to_sensor_voltage(
                    adc_voltage
                )

                distance_mm = sensor_voltage_to_distance_mm(
                    sensor_voltage
                )

                state = detect_state(
                    sensor_voltage
                )

                distance_text = (
                    "{:.1f}".format(distance_mm)
                    if state == "co_vat_can"
                    else "__"
                )

                publish_ros(
                    publishers,
                    channel,
                    distance_mm,
                    state
                )

                print(
                    "{:.3f} | A{} | "
                    "adc = {:.3f} V | "
                    "dist = {} mm | "
                    "state = {}".format(
                        time.time(),
                        channel,
                        adc_voltage,
                        distance_text,
                        state,
                    ),
                    flush=True,
                )


            time.sleep(
                POLL_INTERVAL_SEC
            )


    except KeyboardInterrupt:

        print(
            "\nDung doc ADS1115.",
            flush=True,
        )

    finally:

        bus.close()


if __name__ == "__main__":
    sys.exit(main())