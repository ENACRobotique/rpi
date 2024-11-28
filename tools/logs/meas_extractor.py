#!/usr/bin/env python3

import argparse
from ecal.measurement import hdf5 as h
import sys
import json
from os import walk
from os import path
sys.path.append('../../generated')

from google.protobuf.json_format import MessageToDict
import robot_state_pb2 as pbr
import lidar_data_pb2 as pbl
#import pdb
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
            #print(m.get_channel_type(channel))
            print(f"  {channel:<{max_len}}: {channel_type}")
            #print(m.get_channel_description(channel))
            #file_descriptor = _descriptor.FileDescriptor('', '', serialized_pb=m.get_channel_description(channel))
            #for name, msg_descriptor in file_descriptor.message_types_by_name.items():
            #    Msg = _reflection.GeneratedProtocolMessageType(name, (_message.Message,), {'DESCRIPTOR' : msg_descriptor})

#def get_pb_msg_type():
#    pass


def get_data_csv(m: h.Meas, channel_name, fileout):
    channel_type = m.get_channel_type(channel_name).split('.')[-1]
    try:
        msg_class = pbr.__getattribute__(channel_type)
    except:
        msg_class = pbl.__getattribute__(channel_type)
    fields_names = msg_class.DESCRIPTOR.fields_by_name.keys()
    print(fields_names)
    t0 = m.get_min_timestamp(channel_name)
    with open(fileout, 'w') as fic:
        fic.write(";".join(["time"] + fields_names) + '\n')
        for entry in m.get_entries_info(channel_name):
            t = (entry['rcv_timestamp'] - t0) / 1e6
            id = entry['id']
            data = m.get_entry_data(id)
            msg = msg_class.FromString(data)
            #breakpoint()
            values = [t]
            values.extend([msg.__getattribute__(f) for f in fields_names])
            fic.write(";".join([f"{v}" for v in values]) + '\n')

def get_data_json(m: h.Meas, channel_name, fileout):
    json_data = []
    channel_type = m.get_channel_type(channel_name).split('.')[-1]
    try:
        msg_class = pbr.__getattribute__(channel_type)
    except:
        msg_class = pbl.__getattribute__(channel_type)
    fields_names = msg_class.DESCRIPTOR.fields_by_name.keys()
    print(fields_names)
    t0 = m.get_min_timestamp(channel_name)
    for entry in m.get_entries_info(channel_name):
        t = (entry['rcv_timestamp'] - t0) / 1e6
        id = entry['id']
        data = m.get_entry_data(id)
        msg = msg_class.FromString(data)
        data_point = MessageToDict(msg)
        d = {'time':t, 'data': data_point}
        json_data.append(d)
    with open(fileout, 'w') as fic:
        json.dump(json_data, fic)



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

    if args.channel is None:
        print("channel needed")
        exit(1)

    if args.out is None:
        args.out = args.channel + '.csv'

    if args.out.endswith('csv'):
        get_data_csv(m, args.channel, args.out)
    elif args.out.endswith('json'):
        get_data_json(m, args.channel, args.out)

    #plop(args.meas, args.topic[0], "test.csv")


