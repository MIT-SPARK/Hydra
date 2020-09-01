#!/usr/bin/env python3

import gdown

# Put this in a yaml file perhaps
ids = {"apartment_scene":
       {
           "uHumans2_apartment_s1_00h.bag": '1kU_drpyG7glQ8pJyeztpiy214WbBEtbM',
           "uHumans2_apartment_s1_01h.bag": '1jp7HrRsfGbmC-z757wwXDEpgNPHw0SbK',
           "uHumans2_apartment_s1_02h.bag": '1ai2p6QVNFaPJfEFOOec6URp5Anu0MBoo'
       },
       "office_scene":
       {
           "uHumans2_office_s1_00h.bag": '1CA_1Awu-bewJKpDrILzWok_H_6cOkGDb',
           "uHumans2_office_s1_06h.bag": '1zECekG47mlGafaJ84vCbwcx3Dz03NuvN',
           "uHumans2_office_s1_12h.bag": '1Of7s_QTE9nL1Hd69SFW1R5uDiJDiQrZr'
       },
       "subway_scene":
       {
           "uHumans2_subway_s1_00h.bag": '1ChL1SW1tfZrCjn5XEf4GJG8nm_Cb5AEm',
           "uHumans2_subway_s1_24h.bag": '1ifatqW3hzL9yo8m7Jt3BCqIr-6kzIols',
           "uHumans2_subway_s1_36h.bag": '1xFG565R-9LKXC60Rfx-7fruy3BP4TrHl'
       },
       "neighborhood_scene":
       {
           "uHumans2_neighborhood_s1_00h.bag": '1p_Uv4RLbl1GtjRxu2tldopFRKmgg_Vsy',
           "uHumans2_neighborhood_s1_24h.bag": '1LXloULyuohBzFLumBoScBMlFrT5nRPcE',
           "uHumans2_neighborhood_s1_36h.bag": '1AwgGpqe2g12T2Lm4Nilz4EaNm2_OtfHL'
       }
      }
url = 'https://drive.google.com/uc?id='

# md5 = 'fa837a88f0c40c513d975104edf3da17'
# gdown.cached_download(url, output, md5=md5, postprocess=gdown.extractall)

import os
import errno

def create_full_path_if_not_exists(filename):
    if not os.path.exists(os.path.dirname(filename)):
        try:
            print('Creating non-existent path: %s' % filename)
            os.makedirs(os.path.dirname(filename))
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                print("Could not create inexistent filename: " + filename)

def ensure_dir(dir_path):
    """ Check if the path directory exists: if it does, returns true,
    if not creates the directory dir_path and returns if it was successful"""
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)
    return True

def run(args):
    assert(ensure_dir(args.output_dir))
    for dataset_name in ids.keys():
        assert(ensure_dir(os.path.join(args.output_dir, dataset_name)))
        for rosbag_name in ids[dataset_name].keys():
            print("Downloading rosbag: %s" % rosbag_name)
            rosbag_url = url + ids[dataset_name][rosbag_name]
            gdown.download(rosbag_url, os.path.join(args.output_dir, dataset_name, rosbag_name), quiet=False)
    print("Done downloading dataset.")

    return True

def parser():
    import argparse
    basic_desc = "Download uHumans2 dataset in google drive."

    shared_parser = argparse.ArgumentParser(add_help=True, description="{}".format(basic_desc))

    input_opts = shared_parser.add_argument_group("input options")
    output_opts = shared_parser.add_argument_group("output options")

    output_opts.add_argument(
        "--output_dir", type=str, help="Path to the output directory where the datasets will be saved.", required=True)

    main_parser = argparse.ArgumentParser(description="{}".format(basic_desc))
    sub_parsers = main_parser.add_subparsers(dest="subcommand")
    sub_parsers.required = True
    return shared_parser

import argcomplete
import sys
if __name__ == '__main__':
    parser = parser()
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    # try:
    if run(args):
        sys.exit(os.EX_OK)
    # except Exception as e:
    #     print("error: ", e)
    #     raise Exception("Main evaluation run failed.")
