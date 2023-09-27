from IPython.core.debugger import set_trace

# import types

import numpy as np
import sqlite3

from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

class BagParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of, name_of, type_of in topics_data}
    #

    def __del__(self):
        self.conn.close()
    #

    def get_messages(self, topic_name):
        topic_id = self.topic_id[topic_name]
        # rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        rows = self.cursor.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {topic_id}").fetchall()
        return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]
    #

    def get_msg_data(self, msg_list):
        # 'FLAGS_CARR_SOLN_FIXED'
        # 'FLAGS_CARR_SOLN_FLOAT'
        # 'FLAGS_DIFF_SOLN'
        # 'FLAGS_GNSS_FIX_OK'
        # 'FLAGS_IS_MOVING'
        # 'FLAGS_REF_OBS_MISS'
        # 'FLAGS_REF_POS_MISS'
        # 'FLAGS_REL_POS_HEADING_VALID'
        # 'FLAGS_REL_POS_VALID'
        # 'SLOT_TYPES'

        # 'acc_heading'
        # 'acc_length'
        # 'acc_ned'
        # 'arrow_length'
        # 'arrow_ned'
        # 'arrow_rpy'
        # 'flags'
        # 'get_fields_and_field_types'
        # 'header'
        # 'ref_station_id'
        # 'rel_pos_heading'
        # 'rel_pos_hp_length'
        # 'rel_pos_hp_ned'
        # 'rel_pos_length'
        # 'rel_pos_ned'

        # msg[1].get_fields_and_field_types().keys()
        # dict_keys([
        #     'header'
        #     'ref_station_id'
        #     'rel_pos_ned'
        #     'rel_pos_length'
        #     'rel_pos_heading'
        #     'rel_pos_hp_ned'
        #     'rel_pos_hp_length'
        #     'acc_ned'
        #     'acc_length'
        #     'acc_heading'
        #     'flags'
        #     'arrow_ned'
        #     'arrow_length'
        #     'arrow_rpy'
        # ])

        msg_time = []
        header_time = []

        # rel_pos_ned = []
        # topic_data = types.SimpleNamespace()
        # initing = True
        topic_data = {}

        # msg_keys = msg_list[0][1].get_fields_and_field_types().keys()
        msg_keys = list(msg_list[0][1].get_fields_and_field_types().keys())
        msg_keys.remove('header')
        # set_trace()

        for mk in msg_keys:
            topic_data[mk] = []
        #

        for msg in msg_list:
            msg_time.append(msg[0])

            h_sec = msg[1].header.stamp.sec
            h_nano = msg[1].header.stamp.nanosec
            header_time.append([h_sec, h_nano])

            for mk in msg_keys:
                # if mk == 'header':
                #     continue
                # if initing:
                #     topic_data[mk] = []
                tmp = getattr(msg[1], mk)
                topic_data[mk].append(tmp)
            #

            # initing = False
        #
        header_time = np.array(header_time).T

        for mk in msg_keys:
            topic_data[mk] = np.array(topic_data[mk]).T
        #

        # rel_pos_ned = np.array(rel_pos_ned).T
        return header_time, topic_data
#
