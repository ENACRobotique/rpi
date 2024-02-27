#!/usr/bin/python3

import argparse
from ecal.measurement import hdf5 as h
import sys
from os import walk
from os import path
sys.path.append('../../generated')

import robot_state_pb2 as pbr
import lidar_data_pb2 as pbl
#from google.protobuf import descriptor as _descriptor
#from google.protobuf import reflection as _reflection
#from google.protobuf import message as _message


def find_hdf5(base_path):
    for (dirpath, dirnames, filenames) in walk(args.meas):
        for filename in filenames:
            if filename.endswith('.hdf5'):
                return path.join(dirpath, filename)
    else:
        raise Exception(f"No hdf5 file in {base_path}")

def print_channels(m: h.Meas):
    channels = m.get_channel_names()
    if not channels:
        print("No channels in this file!")
    else:
        print("Channels:")
        max_len = max(map(lambda c: len(c), channels))
        for channel in channels:
            channel_type = m.get_channel_type(channel).split(':')[-1]
            print(m.get_channel_type(channel))
            print(f"  {channel:<{max_len}}: {channel_type}")
            #print(m.get_channel_description(channel))
            #file_descriptor = _descriptor.FileDescriptor('', '', serialized_pb=m.get_channel_description(channel))
            #for name, msg_descriptor in file_descriptor.message_types_by_name.items():
            #    Msg = _reflection.GeneratedProtocolMessageType(name, (_message.Message,), {'DESCRIPTOR' : msg_descriptor})

#def get_pb_msg_type():
#    pass


def get_data(m: h.Meas, channel_name):
    channel_type = m.get_channel_type(channel_name).split('.')[-1]
    t0 = m.get_min_timestamp(channel_name)
    for entry in m.get_entries_info(channel_name):
        t = (entry['rcv_timestamp'] - t0) / 1e6
        id = entry['id']
        data = m.get_entry_data(id)
        try:
            msg = pbr.__getattribute__(channel_type).FromString(data)
        except AttributeError:
            msg = pbl.__getattribute__(channel_type).FromString(data)
        print(f"{t:.2f}")
        print(msg)
        #fic.write(f"{t:.2f};{pos.x:.3f};{pos.y:.3f};{pos.theta:.3f}\n")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='eCAL log extractor', description='Export eCAL recording to CSV')
    parser.add_argument( 'meas', help='hdf5 measurement file')
    #parser.add_argument('-c', '--channels', help='channels', nargs='+')
    parser.add_argument('-c', '--channel', help='channel (topic)')
    parser.add_argument('-o', '--out', help='output file')
    parser.add_argument('-l', '--list_channels', action='store_true')
    args = parser.parse_args()

    hf = find_hdf5(args.meas)
    print(f"Using {hf}")

    m=h.Meas()
    m.open(hf, 0)

    if args.list_channels:
        print_channels(m)
        exit(0)

    get_data(m, args.channel)

    #plop(args.meas, args.topic[0], "test.csv")


