#!/usr/bin/env python3
# Script to download trained face detection weights from Google Drive

import os
import sys
import argparse
import gdown


WEIGHT_FILE_URL = None  # TODO - change None for your weights URL


def yes_no(question):
    """
    Helper function to handle yes/no question
    :param question: string with question
    """
    yes = {'yes', 'y', 'ye', ''}
    no = {'no', 'n'}

    while True:
        choice = input(question).lower()
        if choice in yes:
            return True
        elif choice in no:
            return False
        else:
            print("Please respond with 'yes' or 'no'\n")


def main():
    """Download model weight"""

    # script arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-path", help="output path for the downloaded file",
                        type=str, default="weights.h5")
    parser.add_argument("-f", "--force", help="rewrite file without asking?",
                        type=bool, default=False)
    args = parser.parse_args()

    if WEIGHT_FILE_URL is None:
        print("No URL has been provided. Please change the 'WEIGHT_FILE_URL' at the top of this script to "
              "download the weights.")
        sys.exit(1)

    # check if the output path already exists
    if not args.force:
        if os.path.exists(args.output_path):
            overwrite = yes_no("The file '{}' already exists. Are you sure that you want to overwrite it? (y/n)".
                               format(args.output_path))
            if not overwrite:
                print("OK. Closing the program now.")
                sys.exit(0)

    # download the file
    out = gdown.download(WEIGHT_FILE_URL, args.output_path, quiet=False)
    if out is None:
        print("Failed to download {} from Google Drive. Check that the path is correct and the file can be downloaded "
              "by anybody without having to log into Google".format(WEIGHT_FILE_URL))
        sys.exit(1)


if __name__ == "__main__":
    main()
    sys.exit(0)
