import argparse
import glob
from pathlib import Path
def import_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f1 = open("large-dense-30-196x196.txt", "a")

    f = open(filename, 'r')
    lines = f.readlines()
    count = 0
    # Strips the newline character
    for line in lines:
        o = ""
        for i in line:
            o += i+" "
        f1.write(o)
    f1.close()
    print(count)
import_instance("./internetsample.txt")