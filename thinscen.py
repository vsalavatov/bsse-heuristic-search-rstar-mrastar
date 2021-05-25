#!/usr/bin/env python3
import os

def thin_scenario_file(path, n):
    scenf = open(path, 'r')
    thin_scenf = open(path + '.thin', 'w')

    line = scenf.readline()
    assert(line.startswith('version'))
    thin_scenf.write(line)

    tasks = [x for x in scenf.readlines() if len(x.strip()) > 0]
    tasks = tasks[::(len(tasks) + n - 1) // n]
    thin_scenf.writelines(tasks)

    scenf.close()
    thin_scenf.close()

if __name__ == '__main__':
    n = 200
    for root, dirs, files in os.walk('./dataset/'):
        for f in files:
            if not f.endswith('.scen'):
                continue
            path = os.path.join(root, f)
            print(f'processing {path}')
            thin_scenario_file(path, n)
