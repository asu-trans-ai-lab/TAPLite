from os import chdir, getcwd

from taplite import assignment


ORIG_DIR = getcwd()


def test_assignment(sample_data_dir):
    chdir(sample_data_dir)
    assignment()
    chdir(ORIG_DIR)
