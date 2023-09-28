# from IPython.core.debugger import set_trace
# from importlib import reload, import_module

import numpy as np

# bag_parser = reload(import_module("bag_parser"))
import bag_parser

file_names = ["rosbag2-YYYY_MM_DD-HH_MM_SS"]

fdir = '/gp2_ws/ublox_read_2/data/bags-YYYY-MM-DD'
bag_name = [
    '/flt1_rtk.bag',
    '/flt2_rtk.bag'
]

database_name = [
    f'{bag_name[0]}_0.db3',
    f'{bag_name[1]}_0.db3'
]

topics = [
    [
        "/v2/PosVelEcef",
        "/v2/PosVelTime",
        "/v2/RelPos",
        "/v2/RelPosFlags",
        "/v2/RTCMInput"
    ],
    [
        "/v1/PosVelEcef",
        "/v1/PosVelTime",
        "/v1/RelPos",
        "/v1/RelPosFlags",
        "/v1/RTCMInput",

        "/v2/PosVelEcef",
        "/v2/PosVelTime",
        "/v2/RelPos",
        "/v2/RelPosFlags",
        "/v2/RTCMInput"
    ]
]

# for bag_file_name in file_names:
for ii, (b_name, db_name) in enumerate(zip(bag_name, database_name)):
    # bag_file = f"{bag_file_name}/{bag_file_name}_0.db3"
    bag_file = fdir + b_name + db_name
    parser = bag_parser.BagParser(bag_file)

    file_data = {}

    for flt_topic in topics[ii]:
        msg_list = parser.get_messages(flt_topic)

        topic_data = parser.get_msg_data(msg_list)
        topic_safe = flt_topic.replace("/", "_")
        file_data[topic_safe] = topic_data
    #

    # np.savez("blah" + ".npz", **file_data)
    # set_trace()

    flt_name = b_name.removesuffix(".bag")
    np.savez(fdir + flt_name + ".npz", **file_data)
#

# to access data:

# my_data       = np.load("data_file.npz", allow_pickle=True)
# header_time   = my_data['_v1_PosVelEcef'][()]['header_time']
# lla           = my_data['_v1_PosVelEcef'][()]['lla']

# to extract timestamp without loss of resolution ---
# it also sets initial time to 0.0:
# htt_sec   = header_time[0] - header_time[0][0]
# tspan     = htt_sec + 1e-9 * header_time[1]
