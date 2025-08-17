from os import chdir, getcwd

from taplite import assignment, simulation


ORIG_DIR = getcwd()


def test_assignment(sample_data_dir):
    chdir(sample_data_dir)
    assignment()
    chdir(ORIG_DIR)


def test_simulation(sample_data_dir):
    """ it requires assignment() to be invoked in the first place """
    chdir(sample_data_dir)
    assignment()
    simulation()
    chdir(ORIG_DIR)
