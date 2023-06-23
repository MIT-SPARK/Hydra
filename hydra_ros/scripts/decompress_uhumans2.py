# Copyright 2022, Massachusetts Institute of Technology.
# All Rights Reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Research was sponsored by the United States Air Force Research Laboratory and
# the United States Air Force Artificial Intelligence Accelerator and was
# accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
# and conclusions contained in this document are those of the authors and should
# not be interpreted as representing the official policies, either expressed or
# implied, of the United States Air Force or the U.S. Government. The U.S.
# Government is authorized to reproduce and distribute reprints for Government
# purposes notwithstanding any copyright notation herein.
#
#
#!/usr/bin/env python3

# Put this in a yaml file perhaps
ids = {  # "apartment_scene":
    # {"uHumans2_apartment_s1_00h.bag": '1kU_drpyG7glQ8pJyeztpiy214WbBEtbM',
    # "uHumans2_apartment_s1_01h.bag": '1jp7HrRsfGbmC-z757wwXDEpgNPHw0SbK',
    # "uHumans2_apartment_s1_02h.bag": '1ai2p6QVNFaPJfEFOOec6URp5Anu0MBoo'
    # },
    # "office_scene":
    # {"uHumans2_office_s1_00h.bag": '1CA_1Awu-bewJKpDrILzWok_H_6cOkGDb',
    # "uHumans2_office_s1_06h.bag": '1zECekG47mlGafaJ84vCbwcx3Dz03NuvN',
    # "uHumans2_office_s1_12h.bag": '1Of7s_QTE9nL1Hd69SFW1R5uDiJDiQrZr'
    # },
    "subway_scene": {
        "uHumans2_subway_s1_00h.bag": "1ChL1SW1tfZrCjn5XEf4GJG8nm_Cb5AEm",
        "uHumans2_subway_s1_24h.bag": "1ifatqW3hzL9yo8m7Jt3BCqIr-6kzIols",
        "uHumans2_subway_s1_36h.bag": "1xFG565R-9LKXC60Rfx-7fruy3BP4TrHl",
    },
    "neighborhood_scene": {
        "uHumans2_neighborhood_s1_00h.bag": "1p_Uv4RLbl1GtjRxu2tldopFRKmgg_Vsy",
        "uHumans2_neighborhood_s1_24h.bag": "1LXloULyuohBzFLumBoScBMlFrT5nRPcE",
        "uHumans2_neighborhood_s1_36h.bag": "1AwgGpqe2g12T2Lm4Nilz4EaNm2_OtfHL",
    },
}
url = "https://drive.google.com/uc?id="

import os
import errno


def run(args):
    assert os.path.exists(args.dataset_dir)
    for dataset_name in ids.keys():
        assert os.path.exists(os.path.join(args.dataset_dir, dataset_name))
        print("Decompressing Rosbags for dataset: %s" % dataset_name)
        for rosbag_name in ids[dataset_name].keys():
            print("Decompressing rosbag: %s" % rosbag_name)
            os.system(
                "rosbag decompress %s"
                % os.path.join(args.dataset_dir, dataset_name, rosbag_name)
            )
            print("Done decompressing rosbag: %s" % rosbag_name)

    print("Done decompressing rosbags.")

    return True


def parser():
    import argparse

    basic_desc = "Decompress uHumans2 dataset."

    shared_parser = argparse.ArgumentParser(
        add_help=True, description="{}".format(basic_desc)
    )

    output_opts = shared_parser.add_argument_group("output options")
    output_opts.add_argument(
        "--dataset_dir",
        type=str,
        help="Path to the directory where the datasets are.",
        required=True,
    )

    main_parser = argparse.ArgumentParser(description="{}".format(basic_desc))
    sub_parsers = main_parser.add_subparsers(dest="subcommand")
    sub_parsers.required = True
    return shared_parser


import argcomplete
import sys

if __name__ == "__main__":
    parser = parser()
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    # try:
    if run(args):
        sys.exit(os.EX_OK)
    # except Exception as e:
    #     print("error: ", e)
    #     raise Exception("Main evaluation run failed.")
