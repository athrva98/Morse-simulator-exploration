from __future__ import print_function, absolute_import, division

import morse_simulator.cpp


def test_bindings(debug=False):
    assert morse_simulator.cpp.__doc__
    assert morse_simulator.cpp.c_astar.__doc__
    assert morse_simulator.cpp.c_oriented_astar.__doc__
    assert morse_simulator.cpp.c_get_astar_angles.__doc__
    assert morse_simulator.cpp.c_check_for_collision.__doc__

    if debug:
        print(morse_simulator.cpp.__doc__)
        print(morse_simulator.cpp.c_astar.__doc__)
        print(morse_simulator.cpp.c_oriented_astar.__doc__)
        print(morse_simulator.cpp.c_get_astar_angles.__doc__)
        print(morse_simulator.cpp.c_check_for_collision.__doc__)


if __name__ == '__main__':
    test_bindings(debug=True)
