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

    messages_rel_pos = parser.get_messages("/r1/RelPos")
    set_trace()

    message_time = []
    header_time = []
    pos_all = []
    for mm in messages_rel_pos:
        # mtt = 1e-9 * mm[0]                        # do not convert this to a 64-bit double, you will lose precision
        message_time.append(mm[0])

        htt_sec = mm[1].header.stamp.sec
        htt_nano = mm[1].header.stamp.nanosec
        # htt = htt_sec + 1e-9 * htt_nano           # do not convert this to a 64-bit double, you will lose precision
        header_time.append([htt_sec, htt_nano])

        curr_pos = mm[1].rel_pos_ned
        pos_all.append(curr_pos)
    #

    message_time = np.array(message_time)
    header_time = np.array(header_time).T
    pos_all = np.array(pos_all).T

    gabe = "Gabe"

    # # np.savetxt(bag_file_name + ".csv", pos_all, delimiter=",", header="north, east, down")
    # np.savez(b_name[1:] + ".npz", message_time=message_time, header_time=header_time, pos_all=pos_all)
    np.savez(fdir + b_name + ".npz", message_time=message_time, header_time=header_time, pos_all=pos_all)
    np.savez(fdir + b_name + ".npz", gabe=message_time, header_time=header_time, pos_all=pos_all)


#
