from os import chdir, getcwd

from taplite import assignment, map_matching, simulation


ORIG_DIR = getcwd()


def test_assignment(sample_data_dir):
    chdir(sample_data_dir)
    assignment()
    chdir(ORIG_DIR)


def test_simulation(sample_data_dir):
    """ it requires assignment() to be invoked in the first place """
    chdir(sample_data_dir)
    simulation()
    chdir(ORIG_DIR)


# def test_map_matching(sample_data_dir):
#     chdir(sample_data_dir)
#     map_matching()
#     chdir(ORIG_DIR)
