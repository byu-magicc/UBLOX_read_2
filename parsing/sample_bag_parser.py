from IPython.core.debugger import set_trace

import numpy as np

import bag_parser

file_names = ["rosbag2-YYYY_MM_DD-HH_MM_SS"]

fdir = '/gp2_ws/ublox_read_2/data/bags-YYYY-MM-DD'
bag_name = [
    '/v1_relpos.bag',
    '/v2_relpos.bag',
    '/v3_relpos.bag'
]

database_name = [
    f'{bag_name[0]}_0.db3',
    f'{bag_name[1]}_0.db3',
    f'{bag_name[2]}_0.db3'
]

topics = [
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

# for bag_file_name in file_names:
for b_name, db_name in zip(bag_name, database_name):
    # bag_file = f"{bag_file_name}/{bag_file_name}_0.db3"
    bag_file = fdir + b_name + db_name
    parser = bag_parser.BagParser(bag_file)

    for topic in topics:
        msg_list = parser.get_messages(topic)

        header_time, topic_data = parser.get_msg_data(msg_list)

        topic_safe = topic.replace("/", "_")
        np.savez(fdir + b_name + topic_safe + ".npz", header_time=header_time, **topic_data)
    #

    # # np.savetxt(bag_file_name + ".csv", pos_all, delimiter=",", header="north, east, down")
    # np.savez(fdir + b_name + ".npz", header_time=header_time, **topic_data)

#
